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
#include <stdbool.h>
#include <float.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include "common/encode_bytes.h"
#include "common/zarray.h"
#include "common/zhash.h"
#include "common/time_util.h"
#include "common/lcm_util.h"
#include "common/string_util.h"
#include "common/http_advertiser.h"
#include "lcmtypes/http_advert_t.h"

#include "httpd/httpd.h"
#include "httpd/httpd_websocket.h"

#define TIMEOUT_SECONDS 3

static volatile int running;

static void signal_handler(int signum)
{
    switch (signum) {
        case SIGINT:
            running = 0;
            break;
        default:
            break;
    }
}

typedef struct state state_t;
struct state
{
    lcm_t * lcm;
    httpd_t * httpd;

    pthread_mutex_t lock;

    zhash_t * adverts;
};

char * styling =
" <style> \n"\
" body {font-family: \"Helvetica\", \"Arial\", sans-serif;} \n"\
" #list-container {float: left;} \n"\
" #msg-container {float: left;} \n"\
" table {border-collapse: collapse;} \n"\
" tr.even {background: #def;} \n"\
" tr.odd  {background: #fff;} \n"\
" th, td {white-space: pre; padding: 2px 8px;} \n"\
" .obj-block {margin-left: 20px;} \n"\
" </style>\n"\
"<script>\n"\
"document.title = \"HOME:\" + location.hostname;\n"\
"</script>\n"\
"<div style=\"font-weight: bold; font-size: 32px; padding: 10px\" > HomePage </div>\n";

int httpd_homepage_handler(httpd_t *httpd, struct httpd_client *client,
                       struct httpd_request *request, zarray_t *response_headers, void *user)
{
    int handled = 0;

    state_t * state = user;

    httpd_add_header_date(response_headers, "Last-Modified", utime_now()/1000000);
    httpd_add_header(response_headers, "Content-Transfer-Encoding", "identity");

    char *mime_type = "text/html";
    httpd_add_header(response_headers, "Content-Type", mime_type);

    ////////////////////////////////////////////////
    // build the output HTML
    string_buffer_t *sb = string_buffer_create();


    string_buffer_appendf(sb, "<html>\n");
    string_buffer_appendf(sb, " <meta name=\"viewport\" content=\"width=device-width\" />\n");
    string_buffer_appendf(sb, "<body>\n");
    string_buffer_append_string(sb, styling);
    string_buffer_appendf(sb, "<div id=\"list-container\">\n");
    string_buffer_appendf(sb, "<table id=\"list\">\n");
    string_buffer_appendf(sb, "<tbody>\n");
    string_buffer_appendf(sb, "<tr><th>Port</th><th>Name</th><th align=\"left\">Description</th>\n");

    pthread_mutex_lock(&state->lock);
    zhash_iterator_t zit;
    zhash_iterator_init(state->adverts, &zit);
    uint32_t port;
    http_advert_t * ad;
    int64_t now = utime_now();
    int count = 0;
    while(zhash_iterator_next(&zit, &port, &ad))
    {
        if(ad->utime < now - TIMEOUT_SECONDS*1e6)
        {
            http_advert_t_destroy(ad);
            zhash_iterator_remove(&zit);
            continue;
        }

        string_buffer_appendf(sb,
                       "<tr class=\"%s\"> <td> %d </td><td><a href=\"/\" onclick=\"javascript:event.target.port=%6d\">%s</a></td><td>%s</td></tr>\n",
                       (count % 2 == 0) ? "even" : "odd",
                       ad->port,
                       ad->port,
                       ad->name,
                       ad->desc);
        count++;
    }
    pthread_mutex_unlock(&state->lock);

    string_buffer_appendf(sb, "</tbody>\n");
    string_buffer_appendf(sb, "</table>\n");
    string_buffer_appendf(sb, "</div>\n");
    string_buffer_appendf(sb, "</body>\n");
    string_buffer_appendf(sb, "</html>\n");

    httpd_add_header(response_headers, "Content-Length",
                     "%d", string_buffer_size(sb));

    httpd_send_headers(client, request, response_headers, HTTP_OK);
    estream_printf(client->stream, "\r\n");

    uint64_t bufsz[1] = {string_buffer_size(sb)};
    char * buf = string_buffer_to_string(sb);

    client->stream->writev_fully(client->stream, 1, (const void*[]) { buf }, bufsz);

    free(buf);
    string_buffer_destroy(sb);

    handled = 1;

    return handled;
}

void advert_handler(const lcm_recv_buf_t *rbuf,
             const char *channel, const http_advert_t *_msg, void *userdata)
{
    state_t * state = userdata;

    uint32_t port = _msg->port;
    http_advert_t *msg = http_advert_t_copy(_msg);

    http_advert_t * oldmsg = NULL;
    zhash_get(state->adverts, &port, &oldmsg);
    if(oldmsg) {
        if(msg->utime > oldmsg->utime) {
            zhash_put(state->adverts, &port, &msg, NULL, NULL);
            http_advert_t_destroy(oldmsg);
        } else {
            http_advert_t_destroy(msg);
        }
    } else {
        zhash_put(state->adverts, &port, &msg, NULL, NULL);
    }

    assert(pthread_mutex_trylock(&state->lock));
}

int main()
{
    running = 1;
    int32_t port = 3912;

    state_t *state = calloc(1, sizeof(state_t));
    state->adverts = zhash_create(sizeof(uint32_t), sizeof(struct http_advert_t*),
                                  zhash_uint32_hash,
                                  zhash_uint32_equals);
    state->lcm = lcm_create(NULL);
    http_advertiser_t * advertiser = http_advertiser_create(state->lcm, port, "HomePage", "This page");

    http_advert_t_subscribe(state->lcm, "HTTP_ADVERTS", advert_handler, state);

    state->httpd = httpd_create();
    state->httpd->verbose = 1;
    httpd_add_handler(state->httpd, httpd_homepage_handler, state);

    httpd_listen(state->httpd, port, 10, 0);

    signal(SIGINT, signal_handler);

    while (running) {
        lcm_handle_mutex(state->lcm, &state->lock);
    }
}
