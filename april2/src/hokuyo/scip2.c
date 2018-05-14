/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#include "scip2.h"

#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "common/time_util.h"
#include "common/io_util.h"
#include "common/serial_util.h"
#include "common/zarray.h"
#include "common/zhash.h"
#include "common/tcp_util.h"

#define MAX_LINE_LENGTH 128

#define STATUS_WAITING 0
#define STATUS_DONE 1
#define STATUS_CHECKSUM_ERR 2

typedef struct scip2_transaction scip2_transaction_t;
struct scip2_transaction
{
    uint32_t xid;

    zarray_t *response;
    int status;

    // used to wake up the right caller when their response arrives.
    pthread_mutex_t mutex;
    pthread_cond_t cond;
};

static void handle_response(scip2_t *scip, zarray_t *response)
{
    if (zarray_size(response) < 2)
        goto error;

    char *line0;
    char *line1;
    zarray_get(response, 0, &line0);
    zarray_get(response, 1, &line1);

    int buggy_checksum = !strncmp(line0, "PP", 2) || !strncmp(line0, "II", 2) || !strncmp(line0, "VV", 2);

    // check the checksums (on every line except the first line, which
    // doesn't have one.)
    for (int i = 1; i+1 < zarray_size(response); i++) {
        char *line;
        zarray_get(response, i, &line);

        int len = strlen(line);
        int chklen = len - 2; // skip newline and sent checksum byte

        // For commands like 'PP', the semicolon does not appear to be
        // included in the checksum. This is a bug in the
        // documentation?
        if (buggy_checksum && line[chklen-1] == ';')
            chklen--;

        uint8_t chk = 0;
        for (int j = 0; j < chklen; j++)
            chk += line[j];

        int sent_chk = line[len-2];

        chk = (chk&0x3f) + 0x30;

        if (chk != sent_chk) {
            printf("scip: bad checksum %c != %c on line %d\n", chk, sent_chk, i);
            scip2_response_free(scip, response);
            return;
        }
    }

    // Handle data with a status code of '99'. This status code is
    // used for laser data, and isn't the reply to any of our
    // commands.
    if (scip->on_99_data != NULL && zarray_size(response) > 1) {
        if (!strncmp(line1, "99", 2))
            scip->on_99_data(response, scip->on_99_data_user);

        scip2_response_free(scip, response);
        return;
    }

    // This is a response to one of our commands. Look up the
    // transaction by pulling out the transaction ID and handing off
    // the response.
    pthread_mutex_lock(&scip->mutex);

    char *xids = index(line0, ';');
    if (xids == NULL) {
        printf("scip handle-response: no transaction id %s\n", line0);
        goto error;
    }

    uint32_t xid = strtoll(&xids[1], NULL, 16);
    scip2_transaction_t *trans;
    if (!zhash_get(scip->transactions, &xid, &trans)) {
        printf("scip handle-response: unknown transaction %s\n", line0);
        goto error;
    }

    pthread_mutex_unlock(&scip->mutex);

    // Let the creator of the transaction know that their result is ready.
    pthread_mutex_lock(&trans->mutex);
    trans->response = response;
    trans->status = STATUS_DONE;
    pthread_cond_broadcast(&trans->cond);
    pthread_mutex_unlock(&trans->mutex);

    return;

error:
    pthread_mutex_unlock(&scip->mutex);

    // nobody else is going to use this response!
    scip2_response_free(scip, response);
}

static int scip2_read_line(scip2_t *scip, void *buf_in, int maxlen)
{
    int len = 0;
    char *buf = (char*) buf_in;

    while (len < maxlen) {

      // refill buffer if necessary
      if (scip->rxbuf_pos == scip->rxbuf_avail) {
	// loop on error
	while (1) {
	  scip->rxbuf_avail = read(scip->fd, scip->rxbuf, RXBUF_SIZE);
	  scip->rxbuf_pos = 0;

	  if (scip->rxbuf_avail < 1) {
	    sleep(1);
	    scip->rxbuf_avail = 0;
	    // try again
	    continue;
	  }

	  break;
	}
      }

      // grab the character
      char c = scip->rxbuf[scip->rxbuf_pos++];

      buf[len++] = c;
      if (c=='\r' || c=='\n')
	break;
    }

    buf[len] = '\0';
    return len;
}

static void *reader_thread(void *_a)
{
    scip2_t *scip = (scip2_t*) _a;

    // Packets from the URG are sequences of ASCII lines, each
    // terminated by a LF. The packet ends with an extra LF.
    while (1) {
        zarray_t *response = zarray_create(sizeof(char*));

        while (1) {
            char *line = malloc(MAX_LINE_LENGTH);
            int res = scip2_read_line(scip, line, MAX_LINE_LENGTH);

            if (res < 0) {
                exit(1);
            }

            if (scip->debug)
                printf("READ : %s", line);

            zarray_add(response, &line);

            // last line of the response?
            if ((line[0]=='\n') && line[1] == '\0')
                break;
        }
        handle_response(scip, response);

        // "response" is deallocated once the response has been handled
        // (see scip2_response_free, which is called by the user.)
    }

    return NULL;
}

void scip2_response_free(scip2_t *scip, zarray_t *response)
{
    // Free the memory.
    zarray_vmap(response, free);
    zarray_destroy(response);
}

// A SCIP request is a line of ASCII, usually terminated by a LF. In
// this case, the line feed should be omitted when passed in: we will
// append a transaction ID so that we can return the appropriate
// response.  The caller should call scip2_response_free() on the
// result.  The first response (which is usually an acknowledgement)
// is returned. Subsequent asynchronous responses are passed to the
// callback.
//
// @param command   The command (without newlines or identification strings.) E.g. "RS"
//
// @param callback  If non-null, any future messages bearing the
//                  identification code will be sent to the callback
//                  function. The callback should free the
//                  response. If the callback returns non-zero, future
//                  callbacks will be ignored.
//
// @param user      A context variable passed to callback. May be NULL.
//
// @param timeoutms Maximum delay before receiving confirmation. Use 0 for no timeout.
//
// @returns         The response to the original command, which the user
//                  should deallocate with
//                  scip2_response_free. Returns NULL on timeout.
//
zarray_t *scip2_transaction(scip2_t *scip, const char *command, int timeoutms)
{
    scip2_transaction_t *trans = (scip2_transaction_t*) calloc(1, sizeof(scip2_transaction_t));
    pthread_mutex_init(&trans->mutex, NULL);
    pthread_cond_init(&trans->cond, NULL);
    trans->response = NULL;

    ////////////////////////////////////////////////
    // prepare the transaction
    pthread_mutex_lock(&scip->mutex);

    trans->xid = scip->xid;
    scip->xid++;

    ////////////////////////////////////////////////
    // write the request, appending the transaction id.
    char fullcmd[1024];
    int len = snprintf(fullcmd, 1024, "%s;%08x\n", command, trans->xid);

    zhash_put(scip->transactions, &trans->xid, &trans, NULL, NULL);

    write_fully(scip->fd, fullcmd, len);

    if (scip->debug)
        printf("WRITE: %s\n", fullcmd);

    pthread_mutex_unlock(&scip->mutex);

    ////////////////////////////////////////////////
    // wait for the response
    pthread_mutex_lock(&trans->mutex);
    if (trans->status == STATUS_WAITING) {
        if (timeoutms > 0) {
            struct timespec ts;
            //timespec_now(&ts);
            //timespec_addms(&ts, timeoutms);
            int64_t now = utime_now();
            now += timeoutms * 1000;
            utime_to_timespec(now, &ts);
            pthread_cond_timedwait(&trans->cond, &trans->mutex, &ts);
        } else {
            pthread_cond_wait(&trans->cond, &trans->mutex);
        }
    }
    pthread_mutex_unlock(&trans->mutex);

    zarray_t *response = trans->response;

    ////////////////////////////////////////////////
    // free the transaction
    pthread_mutex_lock(&scip->mutex);

    zhash_remove(scip->transactions, &trans->xid, NULL, NULL);

    pthread_cond_destroy(&trans->cond);
    pthread_mutex_destroy(&trans->mutex);
    free(trans);

    pthread_mutex_unlock(&scip->mutex);

    return response;
}

void scip2_set_99_data_handler(scip2_t *scip, void (*on_99_data)(zarray_t *response, void *user), void *user)
{
    scip->on_99_data = on_99_data;
    scip->on_99_data_user = user;
}

// path can be e.g. "/dev/hokuyo" or "tcp://192.168.0.10:10940".
scip2_t *scip2_create(const char *path)
{
    scip2_t *scip = (scip2_t*) calloc(1, sizeof(scip2_t));

    if (!strncmp(path, "tcp://", 6)) {
        char *p = strdup(path);
        char *host = &p[6];
        int port = 10940;
        char *port_string = strchr(host, ':');
        if (port_string != NULL) {
            *port_string = 0;
            port = atoi(&port_string[1]);
        }
        scip->fd = tcp_util_connect(host, port);
        free(p);
    } else {
        scip->fd = serial_open(path, 115200, 1);
    }

    if (scip->fd < 0)
        return NULL;

    scip->transactions = zhash_create(sizeof(uint32_t), sizeof(scip2_transaction_t*),
                                      zhash_uint32_hash, zhash_uint32_equals);
    srandom(time(NULL));
    scip->xid = random();

    /*    scip->f = fdopen(scip->fd, "r");
    if (scip->f == NULL)
        return NULL;
    */

    pthread_mutex_init(&scip->mutex, NULL);
    pthread_create(&scip->reader_thread, NULL, reader_thread, scip);

    return scip;
}
