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

#ifndef _HTTPD_H
#define _HTTPD_H

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>

#include "ek/ek.h"

#include "common/estream.h"
#include "common/zhash.h"

enum { HTTP_OK = 200,
       HTTP_PARTIAL_CONTENT = 206,
       HTTP_MOVED_PERMANENTLY = 301,
       HTTP_FOUND = 302,
       HTTP_NOT_MODIFIED = 304,
       HTTP_BAD_REQUEST = 400,
       HTTP_UNAUTHORIZED = 401,
       HTTP_FORBIDDEN = 403,
       HTTP_NOT_FOUND = 404,
       HTTP_METHOD_NOT_ALLOWED = 405,
       HTTP_INTERNAL_SERVER_ERROR = 501,
};


struct httpd_request
{
    char *type;       // e.g. GET
    char *url;        // e.g. /index.php?variable=value&foo=bar
    char *url_path;   // e.g. /index.php
    char *url_args;   // e.g. variable=value&foo=bar. Can be NULL.
    char *version;    // e.g. HTTP/1.0
    int  iversion;    // major*10 + minor. e.g., 11 for 1.1.
    zhash_t *headers; // char* => char*

    int data_read;    // set by httpd_read_request_data
};

typedef struct httpd httpd_t;

typedef struct httpd_listener httpd_listener_t;
struct httpd_listener
{
    httpd_t *httpd;
    int port;

    int fd_listen;
    pthread_t accept_thread;

    ek_tls_server_t *server;
};

struct httpd
{
    int verbose; // 1 = verbose, 2 = debug

    pthread_mutex_t mutex;
    zarray_t *handlers; // struct httpd_handler_record. (nb: not pointers)
};

struct httpd_client
{
    httpd_t *httpd;
    httpd_listener_t *listener; // who generated this?

    estream_t *stream;

    struct sockaddr addr;
    socklen_t addr_len;

    int iversion; // what version is this client working from?

    int client_id; // assigned by client_thread
};

typedef int (*httpd_handler_proc_t)(httpd_t *httpd, struct httpd_client *client,
                                    struct httpd_request *request, zarray_t *response_headers,
                                    void *user);


struct httpd_handler_record
{
    httpd_handler_proc_t proc;
    void *user;
};

// Note: will disable SIGPIPE.
httpd_t *httpd_create(void);
int httpd_listen(httpd_t *httpd, int port, int listen_queue, int localhost_only);

int httpd_listen_tls(httpd_t *httpd, ek_tls_server_t *server,
                     int port, int listen_queue, int localhost_only);

// handler returns 1 if it handled the request.
void httpd_add_handler(httpd_t *httpd, httpd_handler_proc_t proc, void *user);

int httpd_default_error_handler(httpd_t *httpd, struct httpd_client *client,
                                struct httpd_request *request, zarray_t *response_headers,
                                void *user);

// a pointer to this structure should be passed in to user.
struct httpd_basic_handler_config {
    // if set, this handler will only accept requests whose HTTP HOST
    // header matches. If no HOST header is specified, requests will
    // pass this check.
    char *host;
    int   port;

    // only accept requests whose URL begins with the path below. Should NOT
    // end with a trailing slash. E.g., "/foobar"
    char *base_url;

    // in translating a URL to a file path, the base_url is first
    // stripped and the base_path is prepended. Should NOT end with a
    // trailing slash. Can be NULL if path translation isn't needed.
    char *base_path;

    // if the referenced path is a directory, search for a file named
    // as below... e.g. "index.html". Can be NULL.
    zarray_t *default_files; // char *

    // Can be NULL in odd cases like websockets.
    zhash_t *mime_types; // char * => char *.. (e.g., "png" => "image/png")
};

struct httpd_basic_handler_results
{
    // the base_url, host, and port match the config.
    int matches;

    // requested host and port (as reported from Host header).
    char *host;
    int  port;

    // The local path, computed by removing base_url and prepending
    // base_path.
    char *computed_path;

    // If computed_path is a directory, try adding
    // config->default_names to find an actual file.  If no match,
    // will be "\0". If not a directory, will be equal to
    // computed_path.
    char *resolved_path;

    // set to 1 if they request /somedirectory, so that we can redirect them
    // to /somedirectory/
    int recommend_redirect_directory;
};

/////////////////////////////////////////////////////
// Many handlers can inherit the basic functionality of httpd_basic_handler
struct httpd_basic_handler_results *httpd_basic_handler_create(
    struct httpd_basic_handler_config *config,
    struct httpd_request *request);

void httpd_basic_handler_redirect_directory(struct httpd_basic_handler_config *config,
                                            struct httpd_client *client,
                                            struct httpd_request *request,
                                            zarray_t *response_headers);

void httpd_basic_handler_destroy(struct httpd_basic_handler_results *res);

// utilities for those implementing handlers

// is the path "tricky"? I.e., contains special characters, "..", etc.?
int httpd_is_tricky_path(const char *path);

void httpd_add_header(zarray_t *headers, const char *key, const char *fmt, ...);
void httpd_add_header_date(zarray_t *headers, const char *key, time_t t);

void httpd_send_headers(struct httpd_client *client, struct httpd_request *request,
                        zarray_t *headers, int code);

////////////////////////////////////////////////////
// Built-in handlers for common cases.

int httpd_file_handler(httpd_t *httpd, struct httpd_client *client,
                       struct httpd_request *request, zarray_t *response_headers, void *user);

int httpd_dir_handler(httpd_t *httpd, struct httpd_client *client,
                      struct httpd_request *request, zarray_t *response_headers, void *user);

int httpd_php_handler(httpd_t *httpd, struct httpd_client *client,
                      struct httpd_request *request, zarray_t *response_headers, void *user);

// redirects all requests not on a https scheme to the same URL with an https scheme.
int httpd_redirect_tls_handler(httpd_t *httpd, struct httpd_client *client,
			       struct httpd_request *request, zarray_t *response_headers, void *user);

////////////////////////////////////////////////////

int httpd_read_request_data(httpd_t *httpd, struct httpd_client *client,
                            struct httpd_request *request,
                            char **_content, long *_content_len);

zhash_t *httpd_read_post_data(httpd_t *httpd, struct httpd_client *client,
                              struct httpd_request *request);

zhash_t *httpd_cookies_parse(struct httpd_request *request);

#endif
