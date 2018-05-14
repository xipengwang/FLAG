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

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <libgen.h>
#include <pthread.h>
#include <dirent.h>

#include <lcm/lcm.h>
#include <lcm/lcm_coretypes.h>

#include "common/april_util.h"
#include "common/encode_bytes.h"
#include "common/getopt.h"
#include "common/april_util.h"
#include "common/http_advertiser.h"
#include "common/encode_bytes.h"
#include "common/json.h"
#include "common/math_util.h"
#include "common/string_util.h"
#include "common/string_util_regex.h"
#include "common/time_util.h"
#include "common/zhash.h"

#include "httpd/httpd.h"
#include "httpd/httpd_websocket.h"

#include "spy/lcmgen.h"

#define SPY_SUMMARY_REQ  ((uint8_t) 101)
#define SPY_SUMMARY_RESP ((uint8_t) 102)
#define SPY_DETAIL_REQ   ((uint8_t) 211)
#define SPY_DETAIL_RESP  ((uint8_t) 212)
#define SPY_DETAIL_CANC  ((uint8_t) 213)


typedef struct {
    httpd_t *httpd;
    websocket_server_t *ws;

    lcmgen_t *lcmg;
    lcm_t *lcm;

    pthread_t stats_thread;
    pthread_mutex_t mutex;

    zhash_t *cds;    // channel name string -> channel_data_t*
    zhash_t *subscriptions; // channel name -> zarray of websocket_client_t*
} state_t;

typedef struct {
    char *name;
    lcm_struct_t *type;    // may be NULL if type is unknown
    int64_t fingerprint;

    uint64_t nreceived;
    uint64_t nerrors;

    // updated by status thread
    double hz;
    double jitter;         // max_interval-min_interval in ms
    double bandwidth;      // in bytes/second

    // bookkeeping
    int32_t min_interval;
    int32_t max_interval;
    uint64_t bytes;
    uint64_t last_nreceived;
    int64_t last_utime;

} channel_data_t;

struct lcmtype {
    int64_t rhash;
    char *name;
};

channel_data_t *channel_data_create(const char *name)
{
    channel_data_t *cd = calloc(1, sizeof(channel_data_t));
    cd->name = strdup(name);

    return cd;
}

void channel_data_destroy(channel_data_t *cd)
{
    free(cd->name);
    free(cd);
}

static void send_channel_list(state_t *state)
{
    pthread_mutex_lock(&state->mutex);

    // Serialize into buffer
    uint32_t capacity = 8192;
    uint32_t size = 0;
    uint8_t *buf = malloc(capacity);

    zhash_iterator_t zit;
    zhash_iterator_init_const(state->cds, &zit);

    channel_data_t *cd;
    while (zhash_iterator_next(&zit, NULL, &cd)) {
        // Ensure space
        const char *type = cd->type ? cd->type->structname->shortname : "?";
        size_t needed = 44 + strlen(cd->name) + strlen(type);
        while (size + needed > capacity) {
            capacity *= 2;
            buf = realloc(buf, capacity);
        }

        encode_string_u32(buf, &size, cd->name);
        encode_string_u32(buf, &size, type);
        encode_u64(buf, &size, cd->fingerprint);
        encode_u64(buf, &size, cd->nreceived);
        encode_u64(buf, &size, cd->nerrors);
        encode_f32(buf, &size, cd->hz);
        encode_f32(buf, &size, cd->jitter);
        encode_f32(buf, &size, cd->bandwidth/1024.0);
    }

    pthread_mutex_unlock(&state->mutex);

    uint8_t header[] = {0x7f, 0xc4, 0xa1, 0xa3, 1};
    websocket_server_send_allv(state->ws, 2,
            (const void *[]){header, buf},
            (uint64_t[]) {sizeof(header), size});

    free(buf);
}

static void clear_channel_data(state_t *state)
{
    pthread_mutex_lock(&state->mutex);
    zhash_vmap_values(state->cds, channel_data_destroy);
    zhash_clear(state->cds);
    pthread_mutex_unlock(&state->mutex);
}

static int on_connect(websocket_client_t *wsclient, struct httpd_basic_handler_results *bhres, void *user)
{
    state_t *state = user;

    //printf("Connect\n");
    send_channel_list(state);
    return 0;
}

static int on_message(websocket_client_t *wsclient, const uint8_t *msg, int msg_len, void *user)
{
    state_t *state = user;
    uint32_t pos = 0;
    uint8_t opcode = decode_u8(msg, &pos, msg_len);

    if (opcode == 2) {
        clear_channel_data(state);
        send_channel_list(state);
        return 0;
    }

    char *channel = decode_string_u32(msg, &pos, msg_len);

    pthread_mutex_lock(&state->mutex);

    zarray_t *clients;
    if (!zhash_get(state->subscriptions, &channel, &clients)) {
        clients = zarray_create(sizeof(void*));
        zhash_put(state->subscriptions, &channel, &clients, NULL, NULL);
    } else {
        free(channel);
    }

    if (opcode == 0) {
        // Remove subscription
        zarray_remove_value(clients, &wsclient, true);
    } else if (opcode == 1) {
        // Remove all previous subscriptions to other channels
        zhash_iterator_t zit;
        zhash_iterator_init_const(state->subscriptions, &zit);
        zarray_t *tmp_clients;
        while (zhash_iterator_next(&zit, NULL, &tmp_clients)) {
            zarray_remove_value(tmp_clients, &wsclient, true);
        }

        // Add subscription
        if (!zarray_contains(clients, &wsclient))
            zarray_add(clients, &wsclient);
    }

    //printf("Subscribers to channel %s: %d\n", channel, zarray_size(clients));

    pthread_mutex_unlock(&state->mutex);

    return 0;
}

static int on_disconnect(websocket_client_t *wsclient, void *user)
{
    state_t *state = user;

    //printf("Disconnect\n");

    // Remove all of client's subscriptions
    zhash_iterator_t zit;
    zhash_iterator_init_const(state->subscriptions, &zit);
    zarray_t *clients;
    while (zhash_iterator_next(&zit, NULL, &clients)) {
        zarray_remove_value(clients, &wsclient, true);
    }

    return 0;
}
void on_lcm(const lcm_recv_buf_t *rbuf, const char *channel, void *user)
{
    static int dropped = 0;

    state_t *state = user;

    // Discard stale messages over 10ms old
    int64_t now = utime_now();
    int64_t utime = rbuf->recv_utime;
    if (now-utime > 10000) {
        dropped += 1;
        if (dropped % 10 == 1)
            printf("WRN: dropped %d stale messages\n", dropped);
        return;
    }

    // Find lcmtype by fingerprint
    int64_t hash = 0;
    __int64_t_decode_array(rbuf->data, 0, rbuf->data_size, &hash, 1);
    lcm_struct_t * st = NULL;
    int ntypes = zarray_size(state->lcmg->structs);
    for (int i = 0; i < ntypes; i++) {
        lcm_struct_t * this_st = NULL;
        zarray_get(state->lcmg->structs, i, &this_st);
        int64_t th = this_st->rhash;
        if(th == hash) {
            st = this_st;
            break;
        }
    }

    // Update channel stats
    pthread_mutex_lock(&state->mutex);

    channel_data_t *cd;
    if (!zhash_get(state->cds, &channel, &cd)) {
        cd = channel_data_create(channel);
        cd->type = st;            // will be null for undecodable types
        cd->fingerprint = hash;
        zhash_put(state->cds, &cd->name, &cd, NULL, NULL);
    }

    // Received a different fingerprint from the last message on this channel?
    if (hash != cd->fingerprint) {
        cd->type = st;
        cd->fingerprint = hash;
        cd->nerrors += 1;
    } else if (st == NULL) {
        cd->nerrors += 1;
    }
    cd->nreceived += 1;

    if (cd->last_utime) {
        int32_t interval = utime - cd->last_utime;
        cd->min_interval = min(cd->min_interval, interval);
        cd->max_interval = max(cd->max_interval, interval);
    }
    cd->last_utime = utime;
    cd->bytes += rbuf->data_size;

    // Send structure to all subscribers
    zarray_t *clients;
    if (cd->type &&
        zhash_get(state->subscriptions, &channel, &clients) &&
        zarray_size(clients) > 0) {

        json_object_t *root = json_hash_create();
        json_hash_add(root, "c", json_string_create_copy(cd->name));
        lcmgen_decode_to_json(state->lcmg, cd->type, rbuf->data,
                rbuf->data_size, 8, root);
        char *json = json_object_tostring(root);
        size_t len = strlen(json);

        uint8_t header[9] = {0x7f, 0xc4, 0xa1, 0xa3, 2,
            len >> 24, len >> 16, len >> 8, len};

        for (int i = 0; i < zarray_size(clients); i += 1) {
            websocket_client_t *wsclient;
            zarray_get(clients, i, &wsclient);
            websocket_client_sendv(wsclient, 2,
                    (const void *[]){header, json},
                    (uint64_t[]) {sizeof(header), len});
        }

        json_object_destroy(root);
        free(json);
    }

    pthread_mutex_unlock(&state->mutex);
}

void *stats_loop(void *user)
{
    state_t *state = user;
    int64_t last_utime = utime_now();

    while (1) {
        timeutil_usleep(1000000);

        pthread_mutex_lock(&state->mutex);

        int64_t utime = utime_now();
        double dt = (utime - last_utime) / 1e6;
        zhash_iterator_t zit;
        channel_data_t *cd;
        zhash_iterator_init_const(state->cds, &zit);
        while (zhash_iterator_next(&zit, NULL, &cd)) {

            cd->hz = ((double)(cd->nreceived - cd->last_nreceived)) / dt;
            cd->last_nreceived = cd->nreceived;

            if (cd->max_interval > 0)
                cd->jitter = (cd->max_interval - cd->min_interval) / 1000.0;
            else
                cd->jitter = -1.0;
            cd->max_interval = -1;
            cd->min_interval = INT32_MAX;

            cd->bandwidth = cd->bytes / dt;
            cd->bytes = 0;
        }
        last_utime = utime;

        pthread_mutex_unlock(&state->mutex);

        send_channel_list(state);
    }

    return NULL;
}

void init_http(state_t * state, int port)
{
    state->httpd = httpd_create();
    http_advertiser_create(state->lcm, port, "LCMSpy", "See what lcm messages are being sent");

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
    char *s = strdup("index.html");
    zarray_add(default_files, &s);

    struct httpd_basic_handler_config *config = calloc(1, sizeof(struct httpd_basic_handler_config));
    config->host = NULL;
    config->port = 0;
    config->base_url = strdup("");
    config->default_files = default_files;
    config->mime_types = mime_types;

    char *base_path = sprintf_alloc("%s/web/lcm_spy", april_util_root_path());
    printf("Using %s\n", base_path);
    config->base_path = strdup(base_path);
    httpd_add_handler(state->httpd, httpd_file_handler, config);

    config = calloc(1, sizeof(struct httpd_basic_handler_config));
    config->host = NULL;
    config->port = 0;
    config->base_url = strdup("/websocket");
    config->base_path = NULL;
    config->default_files = NULL;
    config->mime_types = NULL;

    state->ws = httpd_websocket_create(state->httpd, config,
            on_connect, on_message, on_disconnect, state);
}


int main(int argc, char *argv[])
{
    setlinebuf(stdout);

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Print this help");
    getopt_add_int(gopt, 'p', "port", "5267", "Webserver port");
    getopt_add_string(gopt, 'u', "url", "", "lcm url");
    getopt_add_string(gopt, 'r', "regex", ".*", "channel regex");
    getopt_add_bool(gopt, 't', "tokenize", 0, "Show tokenization");

    if (!getopt_parse(gopt, argc, argv, 0) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        return 1;
    }

    state_t *state = calloc(1, sizeof(state_t));
    state->cds = zhash_create(sizeof(char*), sizeof(void*),
                              zhash_str_hash, zhash_str_equals);
    state->subscriptions = zhash_create(sizeof(char*), sizeof(void*),
            zhash_str_hash, zhash_str_equals);

    state->lcmg = lcmgen_create();
    state->lcmg->gopt = gopt;

    const zarray_t * dirs = getopt_get_extra_args(gopt);
    for (unsigned int i = 0; i < zarray_size(dirs); i++) {
        char *path;
        zarray_get(dirs, i, &path);

        DIR           *d;
        struct dirent *dir;
        d = opendir(path);
        if (!d) {
            if(!str_ends_with(path, ".lcm")) continue;
            printf("FILE: %s\n", path);
            if(!str_ends_with(path, ".lcm")) continue;
            int res = lcmgen_handle_file(state->lcmg, path);
            if (res)
                return res;
        } else {
            printf("DIR:  %s\n", path);
            while ((dir = readdir(d)) != NULL) {
                char buf[1024];
                sprintf(buf, "%s%s", path, dir->d_name);
                if(!str_ends_with(buf, ".lcm")) continue;
                int res = lcmgen_handle_file(state->lcmg, buf);
                if (res)
                    return res;
            }
        }

        closedir(d);
    }
    resolve_hashes(state->lcmg);

    // LCM init
    state->lcm = lcm_create(getopt_get_string(gopt, "url"));
    lcm_subscribe(state->lcm, getopt_get_string(gopt, "regex"), on_lcm, state);

    int port = getopt_get_int(gopt, "port");
    init_http(state, port);

    pthread_mutex_init(&state->mutex, NULL);
    pthread_create(&state->stats_thread, NULL, stats_loop, state);

    while (1) {
        lcm_handle_timeout(state->lcm, 10000);
    }
}
