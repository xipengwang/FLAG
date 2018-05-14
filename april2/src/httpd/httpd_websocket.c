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

#include <stdint.h>
#include <poll.h>
#include <unistd.h>

#include "common/io_util.h"
#include "common/sha1.h"
#include "common/base64.h"
#include "httpd.h"
#include "httpd_websocket.h"

// when we know data is owed to us, how long do we wait?
#define TIMEOUT_MS 10000

// if the connection is just open and waiting, how long until we
// eventually give up?
#define CONNECTION_TIMEOUT_MS (600*1000)

// we basically steal the thread and fd from httpd.
int websocket_handler(httpd_t *httpd, struct httpd_client *client,
                      struct httpd_request *request, zarray_t *response_headers, void *user)
{
    int handled = 0;
    websocket_server_t *wsserver = user;

    struct httpd_basic_handler_config *config = wsserver->basic_config;
    struct httpd_basic_handler_results *bhres = httpd_basic_handler_create(config, request);

    if (!bhres->matches) {
        handled = 0;
        goto cleanup;
    }

    if (bhres->recommend_redirect_directory) {
        httpd_basic_handler_redirect_directory(config, client, request, response_headers);
        handled = 1;
        goto cleanup;
    }

    estream_printf(client->stream, "HTTP/1.1 101 Switching Protocols\r\n");
    estream_printf(client->stream, "Upgrade: websocket\r\n");
    estream_printf(client->stream, "Connection: Upgrade\r\n");

    // compute the brain-dead Sec-WebSocket-Accept cookie. (good lord).
    if (1) {
        char *wskey = "Sec-WebSocket-Key";
        char *wsval;
        if (!zhash_get(request->headers, &wskey, &wsval))
            goto cleanup;

        int len = strlen(wsval);
        if (len > 256)
            goto cleanup;

        char keybuf[1024];
        snprintf(keybuf, sizeof(keybuf), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", wsval);

        sha1_context ctx;
        sha1_starts(&ctx);
        sha1_update(&ctx, (void*) keybuf, strlen(keybuf));
        uint8_t digest[20];
        sha1_finish(&ctx, digest);

        uint8_t cookie[1024];
        int cookie_len;
        base64_encode(digest, 20, cookie, &cookie_len);

        estream_printf(client->stream, "Sec-WebSocket-Accept: %s\r\n", cookie);
    }

    ////////////////////////////////////////////
    // Now we're connected.
    websocket_client_t *wsclient = calloc(1, sizeof(websocket_client_t));
    wsclient->server = wsserver;
    wsclient->stream = client->stream;

    pthread_mutex_init(&wsclient->mutex, NULL);
    pthread_cond_init(&wsclient->cond, NULL);

    if (1) {
        char *protocolname = "Sec-WebSocket-Protocol";
        char *protocol = NULL;
        zhash_get(request->headers, &protocolname, &protocol);

        if (protocol) {
            estream_printf(wsclient->stream, "Sec-WebSocket-Protocol: %s\r\n", protocol);
            wsclient->protocol = strdup(protocol);
        }
    }

    estream_printf(client->stream, "\r\n");

    pthread_mutex_lock(&wsserver->mutex);
    zarray_add(wsserver->clients, &wsclient);
    pthread_mutex_unlock(&wsserver->mutex);

    if (wsserver->on_connect)
        wsserver->on_connect(wsclient, bhres, wsserver->user);

    int opcode = -1;
    uint64_t payload_length = 0;
    uint8_t *payload = NULL;

/*
      0                   1                   2                   3
      0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
     +-+-+-+-+-------+-+-------------+-------------------------------+
     |F|R|R|R| opcode|M| Payload len |    Extended payload length    |
     |I|S|S|S|  (4)  |A|     (7)     |             (16/64)           |
     |N|V|V|V|       |S|             |   (if payload len==126/127)   |
     | |1|2|3|       |K|             |                               |
     +-+-+-+-+-------+-+-------------+ - - - - - - - - - - - - - - - +
     |     Extended payload length continued, if payload len == 127  |
     + - - - - - - - - - - - - - - - +-------------------------------+
     |                               |Masking-key, if MASK set to 1  |
     +-------------------------------+-------------------------------+
     | Masking-key (continued)       |          Payload Data         |
     +-------------------------------- - - - - - - - - - - - - - - - +
     :                     Payload Data continued ...                :
     + - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - +
     |                     Payload Data continued ...                |
     +---------------------------------------------------------------+
*/

    while (1) {
        uint8_t hdr[2];

        if (2 != estream_read_fully_timeout(wsclient->stream, hdr, 2, -1)) {
            printf("httpd_websocket: Connection error\n");
            goto teardown;
        }

        if ((hdr[1] & 0x80) == 0) {
            // per RFC, all client transmissions must have mask set.
            printf("httpd_websocket: BAD MASK\n");
            goto teardown;
        }

        if ((hdr[0] & 0x70) != 0) {
            printf("httpd_websocket: unhandled RSV flags: %02x\n", hdr[0]);
            // try to continue
        }

        uint64_t this_payload_length = hdr[1] & 0x7f;

        if (this_payload_length == 126) {
            uint8_t tmp[2];
            if (estream_read_fully_timeout(wsclient->stream, tmp, 2, TIMEOUT_MS) < 0) {
                printf("Read timeout\n");
                goto teardown;
            }

            this_payload_length = 0;
            for (int i = 0; i < 2; i++)
                this_payload_length = (this_payload_length << 8) | tmp[i];

        } else if (this_payload_length == 127) {

            uint8_t tmp[8];
            if (estream_read_fully_timeout(wsclient->stream, tmp, 8, TIMEOUT_MS) < 0)
                goto teardown;
            this_payload_length = 0;
            for (int i = 0; i < 8; i++)
                this_payload_length = (this_payload_length << 8) | tmp[i];
        }

        uint8_t mask[4] = { 0, 0, 0, 0};
        if (hdr[1] & 0x80) {
            if (estream_read_fully_timeout(wsclient->stream, mask, 4, TIMEOUT_MS) < 0)
                goto teardown;
        }

        if (wsserver->max_message_len > 0 && this_payload_length > wsserver->max_message_len) {
            printf("httpd_websocket: illegal message length %d\n", (int) this_payload_length);
            goto teardown;
        }

        // first frame of a new message?
        if (payload_length == 0) {
            opcode = hdr[0] & 0xf;
        }

        if (1) {
            // read the payload.
            uint64_t offset = payload_length;
            payload = realloc(payload, offset + this_payload_length);

            if (estream_read_fully_timeout(wsclient->stream, &payload[offset], this_payload_length, TIMEOUT_MS) < 0) {
                printf("httpd_websocket: bad payload read.");
                free(payload);
                payload = NULL;
                continue;
            }

            for (int i = 0; i < this_payload_length; i++) {
                payload[offset + i] ^= mask[i&3];
            }
            payload_length += this_payload_length;
        }

        if ((hdr[0] & 0x80) == 0) {
            printf("httpd_websocket: non-FIN\n");
            // this is a non-FIN frame
            continue;
        }

        // this is a FIN frame.
        switch (opcode) {
            case 0:
                printf("WEBSOCKET WARNING: unhandled continuation frame.\n ");
                break;

            case 1:   // text frame
            case 2:   // binary frame
                if (wsserver->on_message)
                    wsserver->on_message(wsclient, payload, payload_length, wsserver->user);
                break;

            case 8: // connection close
                printf("WEBSOCKET WARNING: Connection close frame (ignoring payload of size %d).\n", (int) this_payload_length);
                goto teardown;
                break;

            case 9: // ping
                printf("WEBSOCKET WARNING: untested ping frame.\n");
                break;

            case 10: // pong. (Unsolicited, do nothing.
                break;

            default:
                printf("WEBSOCKET WARNING: unknown opcode %d\n", opcode);
                break;
        }

        // get ready for the next message
        free(payload);
        payload = NULL;
        payload_length = 0;
        opcode = -1;
    }

  teardown:
    free(payload);

    estream_printf(client->stream, "Connection: close\r\n");

    if (wsserver->on_disconnect)
        wsserver->on_disconnect(wsclient, wsserver->user);

    // wait for wsclient_destroy to be called.
    pthread_mutex_lock(&wsclient->mutex);
    while (!wsclient->destroy_called)
        pthread_cond_wait(&wsclient->cond, &wsclient->mutex);
    pthread_mutex_unlock(&wsclient->mutex);

    pthread_mutex_lock(&wsclient->server->mutex);
    zarray_remove_value(wsclient->server->clients, &wsclient, 0);
    pthread_mutex_unlock(&wsclient->server->mutex);
    pthread_mutex_destroy(&wsclient->mutex);
    free(wsclient->protocol);
    free(wsclient);

    // fd will be closed by httpd.c
  cleanup:

    httpd_basic_handler_destroy(bhres);

    return handled;
}

// signals the reader thread that all writing has been complete and so
// the connection can be destroyed.
void websocket_client_destroy(websocket_client_t *wsclient)
{
    pthread_mutex_lock(&wsclient->mutex);
    wsclient->destroy_called = 1;
    pthread_cond_broadcast(&wsclient->cond);
    pthread_mutex_unlock(&wsclient->mutex);
}

int websocket_client_sendv(websocket_client_t *wsclient, int vlen, const void **bufs, uint64_t *lens)
{
    uint64_t msg_len = 0;
    for (int i = 0; i < vlen; i++)
        msg_len += lens[i];

    int res = 0;

    pthread_mutex_lock(&wsclient->mutex);

    uint8_t hdr[16];
    int hdr_len = 0;

    hdr[hdr_len++] = 2;
    if (msg_len < 126) {
        hdr[hdr_len++] = msg_len;
    } else if (msg_len <= 65535) {
        hdr[hdr_len++] = 126;
        hdr[hdr_len++] = (msg_len >> 8);
        hdr[hdr_len++] = (msg_len & 0xff);
    } else {
        hdr[hdr_len++] = 127;
        hdr[hdr_len++] = ((msg_len >> 56) & 0xff);
        hdr[hdr_len++] = ((msg_len >> 48) & 0xff);
        hdr[hdr_len++] = ((msg_len >> 40) & 0xff);
        hdr[hdr_len++] = ((msg_len >> 32) & 0xff);
        hdr[hdr_len++] = ((msg_len >> 24) & 0xff);
        hdr[hdr_len++] = ((msg_len >> 16) & 0xff);
        hdr[hdr_len++] = ((msg_len >> 8) & 0xff);
        hdr[hdr_len++] = ((msg_len >> 0) & 0xff);
    }

    // set FIN bit; all our messages are self-contained.
    hdr[0] |= 0x80;

    if (wsclient->stream->writev_fully(wsclient->stream, 1, (const void*[]) { hdr }, (uint64_t[]) { hdr_len }) < 0) {
        res = -1;
        goto cleanup;
    }

    if (wsclient->stream->writev_fully(wsclient->stream, vlen, bufs, lens)) {
        res = -1;
        goto cleanup;
    }

    res = msg_len;

  cleanup:

    pthread_mutex_unlock(&wsclient->mutex);

    return res;
}

int websocket_server_send_allv(websocket_server_t *wsserver, int niov, const void **bufs, uint64_t *lens)
{
    int nclients = 0;

    pthread_mutex_lock(&wsserver->mutex);

    for (int i = 0; i < zarray_size(wsserver->clients); i++) {
        websocket_client_t *wsclient;
        zarray_get(wsserver->clients, i, &wsclient);

        if (websocket_client_sendv(wsclient, niov, bufs, lens))
            nclients++;
    }

    pthread_mutex_unlock(&wsserver->mutex);

    return nclients;
}

int websocket_server_send_all(websocket_server_t *wsserver, const void *msg, uint64_t msg_len)
{
    return websocket_server_send_allv(wsserver, 1, (const void*[]) { msg }, (uint64_t[]) { msg_len});
}

int websocket_client_send(websocket_client_t *wsclient, const void *msg, uint64_t msg_len)
{
    return websocket_client_sendv(wsclient, 1, (const void*[]) { msg }, (uint64_t[]) { msg_len });
}

websocket_server_t *httpd_websocket_create(httpd_t *httpd,
                                           struct httpd_basic_handler_config *basic_config,
                                           int (*on_connect)(websocket_client_t *wsclient, struct httpd_basic_handler_results *bhres, void *user),
                                           int (*on_message)(websocket_client_t *wsclient, const uint8_t *msg, int msg_len, void *user),
                                           int (*on_disconnect)(websocket_client_t *wsclient, void *user),
                                           void *user)
{
    websocket_server_t *wsserver = calloc(1, sizeof(websocket_server_t));
    wsserver->basic_config = basic_config;
    wsserver->on_connect = on_connect;
    wsserver->on_message = on_message;
    wsserver->on_disconnect = on_disconnect;
    wsserver->user = user;
    wsserver->clients = zarray_create(sizeof(websocket_client_t*));
    wsserver->max_message_len = 16*1024*1024; // prevent DoS

    pthread_mutex_init(&wsserver->mutex, NULL);

    httpd_add_handler(httpd, websocket_handler, wsserver);

    return wsserver;
}

void websocket_server_destroy(websocket_server_t *wsserver)
{
    printf("websocket_server_destroy: not implemented\n");
}
