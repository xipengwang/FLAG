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

#ifndef _HTTPD_WEBSOCKET_H
#define _HTTPD_WEBSOCKET_H

#include "httpd.h"
#include "common/estream.h"

typedef struct websocket_client websocket_client_t;
typedef struct websocket_server websocket_server_t;

// NB: not really a websocket_client... a client of a websocket_server.
struct websocket_client
{
    websocket_server_t *server;
    char *protocol;

    estream_t *stream;
    pthread_mutex_t mutex; // guards writes on response->fd and destroy_called.
    pthread_cond_t cond; // notified when websocket_client_destroy is called.
    int destroy_called; //  set to one when websocket_client_destroy is called.
};

struct websocket_server
{
    struct httpd_basic_handler_config *basic_config;

    int max_message_len;

    int (*on_connect)(websocket_client_t *wsclient, struct httpd_basic_handler_results *bhres, void *user);
    int (*on_message)(websocket_client_t *wsclient, const uint8_t *msg, int msg_len, void *user);
    int (*on_disconnect)(websocket_client_t *wsclient, void *user);

    void *user;

    zarray_t *clients;
    pthread_mutex_t mutex; // guards 'clients'
};


// A websocket_client_t is destroyed (and the underlying socket closed) upon the LATER of two events:
// 1) the on_disconnect() handler returning,
// 2) websocket_client_destroy() being called.
//
// This allows a websocket_client user to, for example, create a
// thread which performs slow synchronous writes on the websocket
// (e.g. via _client_sendv), without the websocket_client being
// destroyed out from underneath it. In this case, if the remote side closes
// the socket, on_disconnect() will be called asynchronously, and
// _sendv will timeout/fail. The writer thread can then call _destroy().

websocket_server_t *httpd_websocket_create(httpd_t *httpd,
                                           struct httpd_basic_handler_config *basic_config,
                                           int (*on_connect)(websocket_client_t *wsclient, struct httpd_basic_handler_results *bhres, void *user),
                                           int (*on_message)(websocket_client_t *wsclient, const uint8_t *msg, int msg_len, void *user),
                                           int (*on_disconnect)(websocket_client_t *wsclient, void *user),
                                           void *user);

// returns total number of bytes written, or -1 on error (such as the socket having been closed)
int websocket_client_sendv(websocket_client_t *wsclient, int niov, const void **bufs, uint64_t *lens);
int websocket_client_send(websocket_client_t *wsclient, const void *msg, uint64_t msg_len);

// send to every connected client. Returns total number of clients.
int websocket_server_send_allv(websocket_server_t *wsserver, int niov, const void **bufs, uint64_t *lens);
int websocket_server_send_all(websocket_server_t *wsserver, const void *msg, uint64_t msg_len);

void websocket_client_destroy(websocket_client_t *wsclient);

#endif
