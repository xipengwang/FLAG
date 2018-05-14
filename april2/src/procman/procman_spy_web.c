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

#include <assert.h>
#include <inttypes.h>
#include "common/zarray.h"
#include "common/time_util.h"
#include "common/april_util.h"
#include "common/getopt.h"
#include "common/encode_bytes.h"
#include "httpd/httpd.h"
#include "httpd/httpd_websocket.h"
#include "common/http_advertiser.h"
#include "lcmtypes/procman_process_list_t.h"
#include "lcmtypes/procman_status_list_t.h"
#include "lcmtypes/procman_output_t.h"
#include "lcmtypes/procman_command_t.h"

#define PROC_LIST 100
#define STAT_LIST 101
#define PROC_OUTPUT 102
#define PROC_CMD 200
#define PROC_SUB 201
#define PROC_USUB 202

//State
typedef struct
{
    int       running;
    lcm_t    *lcm;
    httpd_t  *httpd;
    uint64_t  init_utime;
    uint64_t  utime;
    websocket_server_t *wsserver;
    uint8_t   new;

    zhash_t  *subs;
    pthread_mutex_t subs_mutex;

} state_t;

void send_msg(state_t * state, uint8_t command, void * data, uint32_t datalen,
              websocket_client_t *client)
{
    uint8_t cmd[5];
    cmd[0] = 0x21;
    cmd[1] = 0x43;
    cmd[2] = 0x65;
    cmd[3] = 0x87;
    cmd[4] = command;

    const void *bufs[2] = {&cmd, data};
    uint64_t lens[2] = {5, datalen};

    if(client)
        websocket_client_sendv(client, 2, bufs, lens);
    else
        websocket_server_send_allv(state->wsserver, 2, bufs, lens);
}

void on_proc_list(const lcm_recv_buf_t *rbuf, const char *channel,
                  const procman_process_list_t *msg, void *user)
{
    state_t * state = user;
    const procman_process_list_t * list = msg;

    //New init
    if(list->init_utime > state->init_utime)
    {
        state->new = true;
        printf("New controller ID current:%" PRIu64 " new: %" PRIu64 "\n", state->init_utime,
               list->init_utime);
        state->init_utime = list->init_utime;

        pthread_mutex_lock(&state->subs_mutex);
        zhash_clear(state->subs);
        pthread_mutex_unlock(&state->subs_mutex);
    }

    //dated Init
    if(list->init_utime < state->init_utime)
    {
        printf("WARN Stale controller ID current:%" PRIu64 " stale:%" PRIu64 "\n",
               state->init_utime, list->init_utime);
        return;
    }

    if(list->utime > state->utime)
        state->utime = list->utime;

    if(msg->exit)
    {
        printf("procman controller going down...\n");
        return;
    }

    send_msg(state, PROC_LIST, rbuf->data, rbuf->data_size, NULL);
    state->new = false;
}

void on_stat_list(const lcm_recv_buf_t *rbuf, const char *channel,
                  const procman_status_list_t *msg, void *user)
{
    state_t * state = user;
    const procman_status_list_t * list = msg;

    //New init
    if(list->received_init_utime != state->init_utime)
    {
        printf("WARN status list from wrong controller %" PRId64 "\n", list->received_init_utime);
        return;
    }
    send_msg(state, STAT_LIST, rbuf->data, rbuf->data_size, NULL);
}

void on_output(const lcm_recv_buf_t *rbuf, const char *channel, const procman_output_t *msg,
               void *user)
{
    state_t * state = user;

    if(msg->received_init_utime != state->init_utime)
        return;

    uint8_t * buf = malloc(12+strlen(msg->data)+1);
    uint32_t buf_pos= 0;

    if(msg->stream)
    {
        if(buf_pos == 0) { //encode once
            encode_u32(buf, &buf_pos, msg->procid);
            encode_u32(buf, &buf_pos, msg->stream);
            encode_string_u32(buf, &buf_pos, msg->data);
        }
        send_msg(state, PROC_OUTPUT, buf, buf_pos, NULL);
        free(buf);
        return;
    }

    zhash_iterator_t zit;
    int procid = -1;
    websocket_client_t * client;
    zhash_iterator_init_const(state->subs, &zit);
    while (zhash_iterator_next(&zit, &client, &procid)) {
        if(procid == msg->procid) {

            if(buf_pos == 0) { //encode once
                encode_u32(buf, &buf_pos, msg->procid);
                encode_u32(buf, &buf_pos, msg->stream);
                encode_string_u32(buf, &buf_pos, msg->data);
            }
            send_msg(state, PROC_OUTPUT, buf, buf_pos, client);
        }
    }
    free(buf);
}

static int on_connect(websocket_client_t *wsclient, struct httpd_basic_handler_results *bhres,
                      void *user)
{
    state_t * state = user;
    state->new = true;
    return 0;
}

static int on_message(websocket_client_t *wsclient, const uint8_t *msg, int msg_len, void *user)
{
    state_t * state = user;

    uint32_t inpos = 0;

    if (msg_len < 8 || decode_u32(msg, &inpos, msg_len) != 0x78563412) {
        printf("--spurious message of length %d\n", msg_len);
        for (int i = 0; i < msg_len; i++) {
            if ((i % 16) == 0)
                printf("%04x: ", i);
            printf("%02x ", msg[i]);
            if ((i % 16) == 15)
                printf("\n");
        }
        printf("\n");
        return 0;
    }

    uint32_t code = decode_u32(msg, &inpos, msg_len);

    switch (code) {

        case PROC_CMD: {
            procman_command_t cmd;
            cmd.utime = utime_now();
            cmd.received_init_utime = state->init_utime;
            cmd.received_utime = state->utime;
            cmd.procid =  decode_u32(msg, &inpos, msg_len);
            cmd.enabled = decode_u8(msg, &inpos, msg_len);
            procman_command_t_publish(state->lcm, "PROCMAN_COMMAND", &cmd);

            break;
        }
        case PROC_SUB: {
            int procid =  decode_u32(msg, &inpos, msg_len);
            pthread_mutex_lock(&state->subs_mutex);
            zhash_put(state->subs, &wsclient, &procid, NULL,NULL);
            pthread_mutex_unlock(&state->subs_mutex);
            break;
        }
        case PROC_USUB: {
            pthread_mutex_lock(&state->subs_mutex);
            zhash_remove(state->subs, &wsclient, NULL, NULL);
            pthread_mutex_unlock(&state->subs_mutex);
            break;
        }

        default: {
            printf("Message type: %d\n", code);
            break;
        }

    }
    return 0;
}

static int on_disconnect(websocket_client_t *wsclient, void *user)
{
    state_t * state = user;

    pthread_mutex_lock(&state->subs_mutex);
    zhash_remove(state->subs, &wsclient, NULL, NULL);
    pthread_mutex_unlock(&state->subs_mutex);

    return 0;
}

void init_http(state_t * state)
{
    int port = 8181;
    state->httpd = httpd_create();
    http_advertiser_create(state->lcm, port, "ProcSpy", "See what processes are running");

    if (httpd_listen(state->httpd, port, 10, 0)) {
        fprintf(stderr, "Cannot create httpd on port %d, goodbye\n", port);
        exit(1);
    }

    zhash_t *mime_types = zhash_create(sizeof(char*), sizeof(char*),
                                       zhash_str_hash, zhash_str_equals);
    char **mts = (char*[]) { "png", "image/png",
                             "jpg", "image/jpeg",
                             "jpeg", "image/jpeg",
                             "txt", "text/html",
                             "html", "text/html",
                             "zip", "application/x-compressed",
                             "js", "text/javascript",
                             "css", "text/css",
                             NULL };

    for (int i = 0; mts[i] != NULL; i+=2) {
        char *k = strdup(mts[i]);
        char *v = strdup(mts[i+1]);
        zhash_put(mime_types, &k, &v, NULL, NULL);
    }

    zarray_t *default_files = zarray_create(sizeof(char*));
    char *s = strdup("/procman_spy/index.html");
    zarray_add(default_files, &s);

    struct httpd_basic_handler_config *config = calloc(1, sizeof(struct httpd_basic_handler_config));
    config->host = NULL;
    config->port = 0;
    config->base_url = strdup("");
    config->default_files = default_files;
    config->mime_types = mime_types;

    char base_path[2048];
    snprintf(base_path, 2048, "%s/web/",april_util_root_path());
    config->base_path = strdup(base_path);
    httpd_add_handler(state->httpd, httpd_file_handler, config);

    config = calloc(1, sizeof(struct httpd_basic_handler_config));
    config->host = NULL;
    config->port = 0;
    config->base_url = strdup("/websocket");
    config->base_path = NULL;
    config->default_files = NULL;
    config->mime_types = NULL;

    state->wsserver = httpd_websocket_create(state->httpd, config,
                                                 on_connect, on_message, on_disconnect, state);
}

int main (int argc, char **argv)
{
    printf("PATH: %s\n", april_util_root_path());

    getopt_t * gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show Help");
    if(!getopt_parse(gopt, argc, argv, 1) ||
            getopt_get_bool(gopt, "help"))
    {
        getopt_do_usage(gopt);
        exit(EXIT_SUCCESS);
    }


    state_t *state = calloc(1,sizeof(state_t));
    state->running    = 1;
    state->lcm        = lcm_create(NULL);
    state->init_utime = 0;

    init_http(state);

    pthread_mutex_init(&state->subs_mutex, NULL);
    state->subs = zhash_create(sizeof(websocket_client_t*), sizeof(int),
                                  zhash_ptr_hash, zhash_ptr_equals);

    procman_process_list_t_subscribe (state->lcm, "PROCMAN_PROCESS_LIST", on_proc_list, state);
    procman_status_list_t_subscribe (state->lcm, "PROCMAN_STATUS_LIST", on_stat_list, state);
    procman_output_t_subscribe (state->lcm, "PROCMAN_OUTPUT", on_output, state);

    while(state->running)
    {
        lcm_handle(state->lcm);
    }

    return 0;
}
