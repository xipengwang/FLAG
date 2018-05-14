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
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <netinet/in.h>
#include <ctype.h>
#include <time.h>
#include <assert.h>
#include <poll.h>
#include <signal.h>

#include "httpd.h"
#include "ek/ek.h"

#include "common/tcp_util.h"
#include "common/io_util.h"
#include "common/string_util.h"
#include "common/string_util_regex.h"

#define VERBOSE(httpd, ...) if ((httpd)->verbose >= 1) { fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); }
#define DEBUG(httpd, ...) if ((httpd)->verbose >= 2) { fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); }

#define TIMEOUT_MS 60000

struct http_header {
    char *key;
    char *value;
};

static void buffer_appendf(uint8_t **_buf, int *bufpos, void *fmt, ...)
{
    uint8_t *buf = *_buf;
    va_list ap;

    int salloc = 128;
    char *s = malloc(salloc);

    va_start(ap, fmt);
    int slen = vsnprintf(s, salloc, fmt, ap);
    va_end(ap);

    if (slen >= salloc) {
        s = realloc(s, slen + 1);
        va_start(ap, fmt);
        vsprintf((char*) s, fmt, ap);
        va_end(ap);
    }

    buf = realloc(buf, *bufpos + slen + 1);
    *_buf = buf;

    memcpy(&buf[*bufpos], s, slen + 1); // get trailing \0
    (*bufpos) += slen;

    free(s);
}

static const char* status_code_to_string(int code)
{
    switch (code) {
        case 200:
            return "OK";
        case 206:
            return "Partial Content";
        case 301:
            return "Moved";
        case 302:
            return "Found";
        case 304:
            return "Not Modified";
        case 400:
            return "Bad Request";
        case 401:
            return "Unauthorized";
        case 403:
            return "Forbidden";
        case 404:
            return "Not Found";
        case 405:
            return "Method Not Allowed";
        default:
            return "Unknown Failure";
    }
}

// code: e.g. 404. You must send the final \r\n yourself.
void httpd_send_headers(struct httpd_client *client, struct httpd_request *request, zarray_t *headers, int code)
{
    if (request->iversion < 10)
        return;

    estream_printf(client->stream, "%s %d %s\r\n", request->version, code, status_code_to_string(code));

    for (int i = 0; i < zarray_size(headers); i++) {
        struct http_header *header;
        zarray_get_volatile(headers, i, &header);

        estream_printf(client->stream, "%s: %s\r\n", header->key, header->value);
    }
}

void client_trace(struct httpd_client *client, const char *fmt, ...)
{
    struct timeval tv;
    struct timezone tz;

    if (gettimeofday(&tv, &tz)) {
        perror("gettimeofday");
        return;
    }

    printf("%15.3f ", tv.tv_sec + tv.tv_usec / 1.0E3);

    va_list ap;

    if (client->addr.sa_family == AF_INET) {
        struct sockaddr_in *in = (struct sockaddr_in*) &client->addr;
        uint32_t addr = in->sin_addr.s_addr;
        printf("%16s", inet_ntoa(in->sin_addr));
    } else {
        printf("(non IPv4)");
    }
    printf(" %4d ", client->client_id);

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}

#define CLIENT_DEBUG(client, ...) if ((client)->httpd->verbose >= 2) { client_trace(client, __VA_ARGS__); }
#define CLIENT_VERBOSE(client, ...) if ((client)->httpd->verbose >= 1) { client_trace(client, __VA_ARGS__); }

static void *client_thread(void *_user)
{
    static int next_client_id = 0;
    char *disconnect_reason = NULL;

    struct httpd_client *client = (struct httpd_client*) _user;
    client->client_id = (next_client_id++);

    httpd_t *httpd = client->httpd;

    int allow_another_request = 1;

    CLIENT_DEBUG(client, "connected on port %d", client->listener->port);

  another_request: ;

    struct httpd_request *request = calloc(1, sizeof(struct httpd_request));
    request->headers = zhash_create(sizeof(char*), sizeof(char*),
                                    zhash_str_hash, zhash_str_equals);

    zarray_t *response_headers = NULL;

    zarray_t *toks = NULL;

    // read the main request line, e.g. GET / HTTP/1.0
    char line[16384];
    int linelen;
//  read_request_again: ;

    int ret;
    if ((ret = estream_read_line_timeout(client->stream, line, sizeof(line), TIMEOUT_MS, &linelen))) {
        allow_another_request = 0;
        disconnect_reason = sprintf_alloc("timeout waiting for request (%d)", ret);
        goto cleanup;
    }

    /*  if (0 && !strncmp(line, "PROXY ", 6)) {
        printf("PROXY: %s\n", line);
        goto read_request_again;
    }
    */
    DEBUG(httpd, "\nREQUEST: %s", line);

    toks = str_split_spaces(line);

    // invalid header?
    if (zarray_size(toks) < 2) {
        allow_another_request = 0;
        disconnect_reason = sprintf_alloc("invalid request line: %s", line);
        goto cleanup;
    }

    zarray_get(toks, 0, &request->type);
    zarray_get(toks, 1, &request->url);

    if (zarray_size(toks) == 3)
        zarray_get(toks, 2, &request->version);
    else {
        DEBUG(httpd, "No version specified %d", zarray_size(toks));
        request->version = "HTTP/0.9";
        allow_another_request = 0;
    }

    if (strlen(request->version) != 8 ||
        !isdigit(request->version[5]) ||
        !isdigit(request->version[7])) {
        DEBUG(httpd, "Funny version: %s", request->version);
        request->version = "HTTP/0.9"; // fallback
    }
    request->iversion = 10*atoi(&request->version[5]) + atoi(&request->version[7]);

    if (request->iversion < 11)
        allow_another_request = 0;

    request->type = strdup(request->type);
    request->url = strdup(request->url);
    request->version = strdup(request->version);

    request->url_path = strdup(request->url);
    request->url_args = strrchr(request->url_path, '?');
    if (request->url_args != NULL) {
        request->url_args[0] = 0;
        request->url_args = strdup(&request->url_args[1]);
    }

    if (strcmp(request->type, "GET") &&
        strcmp(request->type, "POST") &&
        strcmp(request->type, "OPTIONS")) {
        disconnect_reason = sprintf_alloc("bad request type: %s\n", request->type);
        goto cleanup;
    }

    // read headers until we're sick of it.
    while (request->iversion >= 10) {
        if (estream_read_line_timeout(client->stream, line, sizeof(line), TIMEOUT_MS, &linelen)) {
            DEBUG(httpd, "Timeout reading headers");
            disconnect_reason = strdup("timeout reading headers\n");
            allow_another_request = 0;
            goto cleanup;
        }

        if (linelen == 0)
            break;

        char *colon = strchr(line, ':');
        if (colon == NULL) {
            disconnect_reason = sprintf_alloc("bad header: %s\n", line);
            goto cleanup;
        }

        colon[0] = '\0';
        char *key = strdup(line);
        char *value = &colon[1];
        while (value[0] == ' ')
            value++;
        value = strdup(value);

        // they could specify headers multiply...
        char *oldkey, *oldvalue;
        if (zhash_put(request->headers, &key, &value, &oldkey, &oldvalue)) {
            free(oldkey);
            free(oldvalue);
        }

        if (zhash_size(request->headers) > 1000) {
            allow_another_request = 0;
            disconnect_reason = strdup("too many headers\n");
            goto cleanup;
        }
    }

    //////////////////////////////////////////////////
    // dispatch request
    CLIENT_VERBOSE(client, "%s %s", request->type, request->url);

    if (httpd->verbose) {
        zhash_iterator_t zit;
        zhash_iterator_init(request->headers, &zit);
        char *key, *value;
        while (zhash_iterator_next(&zit, &key, &value)) {
            DEBUG(httpd, " %20s ==> %s", key, value);
        }
    }

    int handled = 0;

    response_headers = zarray_create(sizeof(struct http_header));

    httpd_add_header_date(response_headers, "Date", time(NULL));

    for (int i = 0; i < zarray_size(httpd->handlers); i++) {
        struct httpd_handler_record *r;
        zarray_get_volatile(httpd->handlers, i, &r);

        int this_handled = r->proc(httpd, client, request, response_headers, r->user);
        if (this_handled) {
            handled = this_handled;
            break;
        }
    }

    if (1) {
        // flush request data if the handler didn't retrieve it. (otherwise we lose sync
        // with the http protocol)
        char *content = NULL;
        long content_len;

        if (!httpd_read_request_data(httpd, client, request, &content, &content_len)) {
            // if successful, then free data. (throw it away).
            free(content);
        }
    }

    if (handled == 0) {
        DEBUG(httpd, "No handler!");

        char *resp = strdup("Unspecified error\r\n");
        int resplen = strlen(resp);

        httpd_add_header(response_headers, "Content-Type", "text/html");
        httpd_add_header(response_headers, "Content-Length", "%d", resplen);

        httpd_send_headers(client, request, response_headers, HTTP_NOT_FOUND);
        estream_printf(client->stream, "\r\n");

        client->stream->writev_fully(client->stream, 1, (const void*[]) { resp }, (uint64_t[]) { resplen });
        free(resp);
    }

    if (handled & 0x8000) {
        disconnect_reason = strdup("handler disabled an additional request");
        allow_another_request = 0;
    }

    //////////////////////////////////////////////////
    // cleanup after a single request
  cleanup:

    if (toks) {
        zarray_vmap(toks, free);
        zarray_destroy(toks);
    }

    // deallocate
    if (request->headers) {
        zhash_iterator_t zit;
        zhash_iterator_init(request->headers, &zit);
        char *key, *value;
        while (zhash_iterator_next(&zit, &key, &value)) {
            free(key);
            free(value);
        }

        zhash_destroy(request->headers);
    }

    if (response_headers) {
        for (int i = 0; i < zarray_size(response_headers); i++) {
            struct http_header *header;
            zarray_get_volatile(response_headers, i, &header);
            free(header->key);
            free(header->value);
        }

        zarray_destroy(response_headers);
    }

    free(request->type);
    free(request->url);
    free(request->url_args);
    free(request->url_path);
    free(request->version);
    free(request);

    if (allow_another_request)
        goto another_request;

    // clean up the connection
    client->stream->close(client->stream);
    client->stream->destroy(client->stream);

    CLIENT_DEBUG(client, "disconnected: %s", disconnect_reason);

    free(client);
    free(disconnect_reason);

    return NULL;
}

static void *accept_thread(void *_user)
{
    struct httpd_listener *listener = _user;
    httpd_t *httpd = listener->httpd;

    while (1) {
        struct httpd_client *client = calloc(1, sizeof(struct httpd_client));
        client->httpd = httpd;
        client->listener = listener;
        client->addr_len = sizeof(client->addr);

        int fd = accept(listener->fd_listen, &client->addr, &client->addr_len);

        client->stream = estream_create_from_fd(fd);

        pthread_t _client_thread;
        pthread_create(&_client_thread, NULL, client_thread, client);
        pthread_detach(_client_thread);

        // XXX limit # of threads for DoS
    }

    return NULL;
}

// returns zero on success
int httpd_listen(httpd_t *httpd, int port, int listen_queue, int localhost_only)
{
    struct httpd_listener *listener = calloc(1, sizeof(struct httpd_listener));
    listener->httpd = httpd;
    listener->port = port;

    listener->fd_listen = tcp_util_listen(listener->port, listen_queue, localhost_only);
    if (listener->fd_listen < 0) {
        free(listener);
        return -1;
    }

    pthread_create(&listener->accept_thread, NULL, accept_thread, listener);
    pthread_detach(listener->accept_thread);

    return 0;
}

struct client_thread_tls_ctx
{
    ek_tls_server_t *server;
    struct httpd_client *client;
    int fd;
};

static void *client_thread_tls(void *_user)
{
    int fd;
    ek_tls_server_t *server;
    struct httpd_client *client;

    if (1) {
        struct client_thread_tls_ctx *ctx = _user;
        fd = ctx->fd;
        server = ctx->server;
        client = ctx->client;
        free(ctx);
    }

    estream_t *tcp_stream = estream_create_from_fd(fd);

    client->stream = ek_tls_server_conn_create(server, tcp_stream)->tls_stream;

    client_thread(client);

    return NULL;
}

static void *accept_thread_tls(void *_user)
{
    struct httpd_listener *listener = _user;
    httpd_t *httpd = listener->httpd;

    while (1) {
        struct httpd_client *client = calloc(1, sizeof(struct httpd_client));
        client->httpd = httpd;
        client->listener = listener;
        client->addr_len = sizeof(client->addr);

        int fd = accept(listener->fd_listen, &client->addr, &client->addr_len);

        struct client_thread_tls_ctx *ctx = calloc(1, sizeof(struct client_thread_tls_ctx));
        ctx->client = client;
        ctx->fd = fd;
        ctx->server = listener->server;

        pthread_t _client_thread;
        pthread_create(&_client_thread, NULL, client_thread_tls, ctx);
        pthread_detach(_client_thread);

        // XXX limit # of threads for DoS
    }

    return NULL;
}

int httpd_listen_tls(httpd_t *httpd, ek_tls_server_t *server,
                     int port, int listen_queue, int localhost_only)
{
    struct httpd_listener *listener = calloc(1, sizeof(struct httpd_listener));
    listener->httpd = httpd;
    listener->port = port;
    listener->server = server;

    listener->fd_listen = tcp_util_listen(listener->port, listen_queue, localhost_only);
    if (listener->fd_listen < 0) {
        free(listener);
        return -1;
    }

    pthread_create(&listener->accept_thread, NULL, accept_thread_tls, listener);
    pthread_detach(listener->accept_thread);

    return 0;
}

httpd_t *httpd_create()
{
    signal(SIGPIPE, SIG_IGN);

    httpd_t *httpd = calloc(1, sizeof(httpd_t));

    httpd->handlers = zarray_create(sizeof(struct httpd_handler_record));

    return httpd;
}

void httpd_add_handler(httpd_t *httpd, httpd_handler_proc_t proc, void *user)
{
    struct httpd_handler_record r = { .proc = proc, .user = user };

    zarray_add(httpd->handlers, &r);
}

void httpd_add_header(zarray_t *headers, const char *key, const char *fmt, ...)
{
    va_list ap;
    char value[1024];

    va_start(ap, fmt);
    vsnprintf(value, sizeof(value), fmt, ap);
    va_end(ap);

    struct http_header header = { .key = strdup(key),
                                  .value = strdup(value) };
    zarray_add(headers, &header);
}

void httpd_add_header_date(zarray_t *headers, const char *key, time_t t)
{
    struct tm thetm;
    gmtime_r(&t, &thetm);

    // example from apache:
    // Date: Mon, 01 Sep 2014 18:00:21 GMT

    char *days[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    char *months[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

    char value[1024];
    snprintf(value, sizeof(value), "%s, %02d %s %d %02d:%02d:%02d GMT",
             days[thetm.tm_wday], thetm.tm_mday, months[thetm.tm_mon], thetm.tm_year+1900,
             thetm.tm_hour, thetm.tm_min, thetm.tm_sec);

    struct http_header header = { .key = strdup(key),
                                  .value = strdup(value) };
    zarray_add(headers, &header);
}

void httpd_basic_handler_destroy(struct httpd_basic_handler_results *res)
{
    if (res) {
        free(res->host);
        free(res->computed_path);
        free(res->resolved_path);
        free(res);
    }
}

void httpd_basic_handler_redirect_directory(struct httpd_basic_handler_config *config,
                                            struct httpd_client *client,
                                            struct httpd_request *request,
                                            zarray_t *response_headers)
{
    char *resp = strdup("Unspecified error");
    int resplen = strlen(resp);

    httpd_add_header(response_headers, "Content-Type", "text/html");
    if (request->url_args)
        httpd_add_header(response_headers, "Location",
                         sprintf_alloc("%s/?%s", request->url_path, request->url_args));
    else
        httpd_add_header(response_headers, "Location",
                         sprintf_alloc("%s/", request->url_path));

    httpd_send_headers(client, request, response_headers, HTTP_FOUND);
    estream_printf(client->stream, "\r\n");

    client->stream->writev_fully(client->stream, 1, (const void*[]) { resp }, (uint64_t[]) { resplen });
    free(resp);
}

struct httpd_basic_handler_results *httpd_basic_handler_create(struct httpd_basic_handler_config *config,
                                                               struct httpd_request *request)
{
    struct httpd_basic_handler_results *res = calloc(1, sizeof(struct httpd_basic_handler_results));

    // was there an HTTP HOST header? (e.g. Host=www.myserver.com:1234. If no header,
    // return match.
    char *_host;
    char *key = "Host";
    if (zhash_get(request->headers, &key, &_host)) {
        res->host = strdup(_host);

        char *port_s = strchr(res->host, ':');
        res->port = 0;

        if (port_s != NULL) {
            // there is a port. Parse it.
            *port_s = 0;
            port_s = &port_s[1];
            res->port = atoi(port_s);
        }
    }

    res->matches = 1;

    if (config->host) {
        if (strcmp(res->host, config->host))
            res->matches = 0;
    }

    if (config->port > 0) {
        if (config->port != res->port)
            res->matches = 0;
    }

    // does the base_url match?
    if (strncmp(request->url, config->base_url, strlen(config->base_url))) {
        res->matches = 0;
    }

    res->computed_path = sprintf_alloc("%s/%s",
                                       config->base_path,
                                       &request->url_path[strlen(config->base_url)]);

    // filter the computed path. Replace multiple forward slashes with a single slash
    if (1) {
        int outpos = 0;

        for (int inpos = 0; res->computed_path[inpos] != 0; inpos++) {
            if (res->computed_path[inpos] == '/') {
                if (outpos ==0 || res->computed_path[outpos-1] != '/')
                    res->computed_path[outpos++] = res->computed_path[inpos];
                continue;
            }

            res->computed_path[outpos++] = res->computed_path[inpos];
        }
        res->computed_path[outpos] = 0;
    }

    struct stat s;

    // doesn't exist at all.
    if (stat(res->computed_path, &s)) {
        res->resolved_path = NULL;
        return res;
    }

    // a regular file? we're happy!
    if (S_ISREG(s.st_mode)) {
        res->resolved_path = strdup(res->computed_path);
        return res;
    }

    // no default files specified? We don't handle this.
    if (!config->default_files) {
        res->resolved_path = NULL;
        return res;
    }

    if (S_ISDIR(s.st_mode) && !str_ends_with(res->computed_path, "/"))
        res->recommend_redirect_directory = 1;

    if (S_ISDIR(s.st_mode)) {
        for (int i = 0; i < zarray_size(config->default_files); i++) {
            char *name;
            zarray_get(config->default_files, i, &name);
            char *trypath = sprintf_alloc("%s/%s", res->computed_path, name);

            if (!stat(trypath, &s) && S_ISREG(s.st_mode)) {
                res->resolved_path = trypath;
                // trypath not leaking; we had to return allocated resolved_path.
                return res;
            }

            free(trypath);
        }
    }

    return res;
}

// is the path "tricky"? I.e., contains special characters, "..", etc.?
int httpd_is_tricky_path(const char *path)
{
    int len = strlen(path);

    for (int i = 0; i < len; i++) {
        char c = path[i];

        // per POSIX "Fully portable filenames" [A-Za-z0-9._-]
        if (c >= 'a' && c <= 'z')
            continue;

        if (c >= 'A' && c <= 'Z')
            continue;

        if (c >= '0' && c <= '9')
            continue;

        if (c == '_' || c == '-')
            continue;

        // a trailing dot is okay
        if (c == '.' && i+1 >= len)
            continue;

        // NOT part of the POSIX spec; but we'll accept it
        if (c == '~' && i+1 >= len)
            continue;

        // a dot followed by something non-dot is okay
        if (c == '.' && path[i+1] != '.')
            continue;

        if (c == '/')
            continue;

        // NB: two consecutive dots are assumed to be tricky.
        return 1;
    }

    return 0;
}

int httpd_default_error_handler(httpd_t *httpd, struct httpd_client *client,
                                struct httpd_request *request, zarray_t *response_headers, void *user)
{
    char *resp = strdup("The requested resource was not found [error].\r\n");
    int resplen = strlen(resp);

    httpd_add_header(response_headers, "Content-Type", "text/html");
    httpd_add_header(response_headers, "Content-Length", "%d", resplen);

    httpd_send_headers(client, request, response_headers, HTTP_NOT_FOUND);
    estream_printf(client->stream, "\r\n");

    client->stream->writev_fully(client->stream, 1, (const void*[]) { resp }, (uint64_t[]) { resplen });
    free(resp);

    return 1;
}

int httpd_dir_handler(httpd_t *httpd, struct httpd_client *client,
                       struct httpd_request *request, zarray_t *response_headers, void *user)

{
    int handled = 0;

    struct httpd_basic_handler_config *config = (struct httpd_basic_handler_config*) user;
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

    struct stat s;
    stat(bhres->computed_path, &s);
    if (!S_ISDIR(s.st_mode)) {
        handled = 0;
        goto cleanup;
    }

    ////////////////////////////////////////////////
    // get (and sort) the list of directory entries
    DIR *dir = opendir(bhres->computed_path);

    zarray_t *names = zarray_create(sizeof(char*));
    struct dirent *de;
    while ((de = readdir(dir)) != NULL) {
        char *n = strdup(de->d_name);
        zarray_add(names, &n);
    }
    closedir(dir);

    zarray_sort(names, zstrcmp);

    ////////////////////////////////////////////////
    // build the output HTML
    uint8_t *buf = NULL;
    int bufsz = 0;

    buffer_appendf(&buf, &bufsz, "<html><head><title>Directory</title></head><body>\n");
    for (int i = 0; i < zarray_size(names); i++) {
        char *n;
        zarray_get(names, i, &n);

        buffer_appendf(&buf, &bufsz,
                       "<a href=\"%s%s\">%s</a><br>\n",
                       request->url_path, n, n);
    }
    buffer_appendf(&buf, &bufsz, "</body></html>\n");

    httpd_add_header(response_headers, "Content-Type", "text/html");
    httpd_add_header(response_headers, "Content-Length", "%d", bufsz);

    httpd_send_headers(client, request, response_headers, HTTP_OK);
    estream_printf(client->stream, "\r\n");

    client->stream->writev_fully(client->stream, 1, (const void*[]) { buf }, (uint64_t[]) { bufsz });
    free(buf);

    handled = 1;

    zarray_vmap(names, free);
    zarray_destroy(names);

  cleanup:

    httpd_basic_handler_destroy(bhres);
    return handled;

}

int httpd_file_handler(httpd_t *httpd, struct httpd_client *client,
                       struct httpd_request *request, zarray_t *response_headers, void *user)
{
    int handled = 0;

    struct httpd_basic_handler_config *config = (struct httpd_basic_handler_config*) user;
    struct httpd_basic_handler_results *bhres = httpd_basic_handler_create(config, request);

    FILE *f = NULL;

    if (!bhres->matches) {
        handled = 0;
        goto cleanup;
    }

    if (bhres->recommend_redirect_directory) {
        httpd_basic_handler_redirect_directory(config, client, request, response_headers);
        handled = 1;
        goto cleanup;
    }

    if (bhres->resolved_path == NULL || httpd_is_tricky_path(bhres->resolved_path)) {
        handled = 0;
        goto cleanup;
    }

    f = fopen(bhres->resolved_path, "r");

    if (f == NULL) {
        printf("failed to open %s\n", bhres->resolved_path);
        handled = 0;
        goto cleanup;
    }

    fseek(f, 0L, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0L, SEEK_SET);

    struct stat s;
    fstat(fileno(f), &s);
    if (!S_ISREG(s.st_mode)) {
        fclose(f);
        return 0;
    }

    char *range;
    long pos0 = 0;
    long pos1 = file_size - 1;

    char *key = "Range";
    if (zhash_get(request->headers, &key, &range)) {
        // range: pos0 - pos1 / length (where pos0 and pos1 are
        // inclusive). Thus, to retrieve an entire 100 byte file:
        // range: 0 - 99 / 100
        // was an HTTP_RANGE specified?

        string_feeder_t *sf = string_feeder_create(range);
        char *units = string_feeder_next_regex(sf, "^bytes=");
        if (units) {
            free(units);

            char *spos0 = string_feeder_next_regex(sf, "^[0-9]+");
            char *sep0 = string_feeder_next_regex(sf, "^ - ");
            char *spos1 = string_feeder_next_regex(sf, "^[0-9]+");
            char *sep1 = string_feeder_next_regex(sf, "^ / ");
            char *sz = string_feeder_next_regex(sf, "^[0-9]+");

            if (spos0)
                pos0 = atol(spos0);
            if (spos1)
                pos1 = atol(spos1);

            free(spos0);
            free(spos1);
            free(sep0);
            free(sep1);
            free(sz);
        }

        string_feeder_destroy(sf);
    }

    // XXX error handling.
    fseek(f, pos0, SEEK_SET);

    if (pos0 != 0 || pos1 != (file_size - 1))
        httpd_add_header(response_headers, "Content-Range", "bytes %" PRId64 "-%" PRId64 "/%" PRId64 "", pos0, pos1, file_size);

    httpd_add_header_date(response_headers, "Last-Modified", s.st_mtime);
    httpd_add_header(response_headers, "Content-Transfer-Encoding", "identity");
    httpd_add_header(response_headers, "Accept-Ranges", "bytes");

    char *mime_type = "text/html";
    char *type_suffix = strrchr(request->url_path, '.');
    if (config->mime_types && type_suffix != NULL) {
        type_suffix = &type_suffix[1];
        zhash_get(config->mime_types, &type_suffix, &mime_type);
    }

    httpd_add_header(response_headers, "Content-Type", mime_type);

    ssize_t len = pos1 - pos0 + 1;

    httpd_add_header(response_headers, "Content-Length", "%d", len);

    httpd_send_headers(client, request, response_headers,
                 (pos0 != 0 || pos1 != (file_size - 1)) ? HTTP_PARTIAL_CONTENT : HTTP_OK);
    estream_printf(client->stream, "\r\n");

    long total_sent = 0;
    while (total_sent < len) {
        char buf[4096];
        int this_sent = len - total_sent;
        if (this_sent > sizeof(buf))
            this_sent = sizeof(buf);

        ssize_t actually_read = fread(buf, 1, this_sent, f);
        if (actually_read != this_sent) {
//            perror("fread X");
            break;
        }

        int actually_written = client->stream->writev_fully(client->stream, 1, (const void*[]) { buf }, (uint64_t[]) { this_sent });
        if (actually_written != this_sent) {
//            perror("fwrite");
            break;
        }

        total_sent += this_sent;
    }
    fclose(f);

    handled = 1;

  cleanup:

    httpd_basic_handler_destroy(bhres);
    return handled;
}

static int hexchar2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return 0;
}

// unescaped string is stored in-place. (It's never longer than the
// original string.)
static void unescape(char *s)
{
    int inpos = 0;
    int outpos = 0;

    while (s[inpos] != 0) {

        if (s[inpos] == '%' && s[inpos+1] != 0 && s[inpos+2] != 0) {
            s[outpos++] = (hexchar2int(s[inpos+1]) << 4) + hexchar2int(s[inpos+2]);
            inpos += 3;
        } else {
            s[outpos++] = s[inpos++];
        }
    }

    s[outpos] = 0;
}

int httpd_read_request_data(httpd_t *httpd, struct httpd_client *client,
                            struct httpd_request *request,
                            char **_content, long *_content_len)
{
    if (request->data_read) // already read!
        return -1;

    request->data_read = 1;
    char *s = zhash_str_str_get(request->headers, "Content-Length");
    if (!s)
        return -1;

    long content_len = atoi(s);
    if (content_len < 0 || content_len > 16*1024*1024) {
        return -2;
    }

    char *content = malloc(content_len + 1);
    if (estream_read_fully_timeout(client->stream, content, content_len, TIMEOUT_MS) < 0) {
        free(content);
        return -3;
    }

    content[content_len] = 0;

    *_content = content;
    *_content_len = content_len;
    return 0;
}

zhash_t *httpd_read_post_data(httpd_t *httpd, struct httpd_client *client,
                              struct httpd_request *request)
{
    zhash_t *pairs = zhash_create(sizeof(char*), sizeof(char*),
                                  zhash_str_hash, zhash_str_equals);

    char *hdrname = "Content-Length";
    char *hdrval;
    if (!zhash_get(request->headers, &hdrname, &hdrval)) {
        return pairs;
    }

    printf("hdrval: %s\n", hdrval);
    int content_len = atoi(hdrval);
    if (content_len < 0) {
        printf("ack 2\n");
        return pairs;
    }

    char *content = malloc(content_len + 1);
    if (estream_read_fully_timeout(client->stream, content, content_len, TIMEOUT_MS) < 0) {
        printf("timeout\n");
        free(content);
        return pairs;
    }

    content[content_len] = 0;

    int start_pos = 0;

    for (int end_pos = 0; end_pos <= content_len; end_pos++) {
        // is 'end_pos' the end of a key=value pair?
        if (end_pos == content_len || content[end_pos] == '&') {

            // look for the equal sign between start and end
            for (int eq_pos = start_pos; eq_pos < end_pos; eq_pos++) {

                // we found it; pull out the key and value.
                if (content[eq_pos] == '=') {
                    char *key;
                    if (1) {
                        int offset = start_pos;
                        int len = eq_pos - offset;
                        key = malloc(len + 1);
                        memcpy(key, &content[offset], len);
                        key[len] = 0;
                        unescape(key);
                    }

                    char *value;
                    if (1) {
                        int offset = eq_pos + 1;
                        int len = end_pos - offset;
                        value = malloc(end_pos - eq_pos + 1);
                        memcpy(value, &content[offset], len);
                        value[len] = 0;
                        unescape(value);
                    }

                    char *oldkey, *oldvalue;
                    if (zhash_put(pairs, &key, &value, &oldkey, &oldvalue)) {
                        free(oldkey);
                        free(oldvalue);
                    }
                    break;
                }
            }

            start_pos = end_pos + 1;
        }
    }

    free(content);
    return pairs;
}

/*
// user should be a pointer to a httpd_basic_handler_config.
int httpd_php_handler(httpd_t *httpd, struct httpd_client *client,
                      struct httpd_request *request, zarray_t *response_headers, void *user)
{
    int handled = 0;

    struct httpd_basic_handler_config *config = (struct httpd_basic_handler_config*) user;
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

    DEBUG(httpd, "computed path: %s", bhres->computed_path);
    DEBUG(httpd, "resolved path: %s", bhres->resolved_path);

    if (bhres->resolved_path == NULL || httpd_is_tricky_path(bhres->resolved_path)) {
        handled = 0;
        goto cleanup;
    }

    // only process URLs ending in .php
    if (!str_ends_with(bhres->resolved_path, ".php")) {
        handled = 0;
        goto cleanup;
    }

    if (httpd_is_tricky_path(bhres->resolved_path)) {
        handled = 0;
        goto cleanup;
    }

    int pipefds[2];
    if (pipe(pipefds) < 0) {
        perror("pipe");
        return 0;
    }

    // php-cgi will output its own set of headers.  bug: only output
    // if HTTP ver > 1.0. (But php-cgi will probably output headers
    // anyway, so...)
    httpd_add_header(response_headers, "Connection", "close");
    httpd_send_headers(client, request, response_headers, HTTP_OK);

    // don't send \r\n, cgi will
    if (fork() == 0)  {

        // Reminder: don't use printf from here on out; it'll be
        // directed to the CGI and could even block!
        int res = close(1);         // close stdout
        assert(res == 0);

        int newstdout = dup(pipefds[1]);  // this will create a new stdout
        assert(newstdout == 1);

        if (1) {
            close(0);         // close stdin
            int newstdin = dup(client->rx_fd);   // this will create a new stdin
            assert(newstdin == 0);
        }

        // our parent will retain open copies of the pipe fds... we
        // are done though.
        close(pipefds[0]);
        close(pipefds[1]);

        // some scripts may rely on PATH or other env
        // variables. Presumably, scripts are at least reasonably
        // trustworthy, or we wouldn't be handing them over to exec!

//        clearenv();

        ////////////////////////////////////////////
        char *envmap[] = { "Content-Length", "CONTENT_LENGTH",
                           "Content-Type", "CONTENT_TYPE",
                           "Accept", "HTTP_ACCEPT",
                           "Connection", "HTTP_CONNECTION",
                           "Host", "HTTP_HOST",
                           "User-Agent", "HTTP_USER_AGENT",
                           "Accept-Encoding", "HTTP_ACCEPT_ENCODING",
                           "Accept-Language", "HTTP_ACCEPT_LANGUAGE",
                           NULL
        };

        for (int i = 0; envmap[i] != NULL; i += 2) {
            char *value;
            if (zhash_get(request->headers, &envmap[i], &value)) {
                setenv(envmap[i+1], value, 1);
            }
        }

        char *php_path = "/usr/bin/php-cgi";

        char *argv[] = { php_path, bhres->resolved_path, NULL };

        setenv("REDIRECT_STATUS", "1", 1);
        setenv("SCRIPT_FILENAME", bhres->resolved_path, 1);
        setenv("GATEWAY_INTERFACE", "CGI/1.1", 1);

        char buf[1024];
        snprintf(buf, sizeof(buf), "%d", client->listener->port);
        setenv("SERVER_PORT", buf, 1);

        setenv("REQUEST_METHOD", request->type, 1);
        setenv("REQUEST_URI", request->url, 1);
        if (request->url_args)
            setenv("QUERY_STRING", request->url_args, 1);
        setenv("SCRIPT_NAME", request->url_path, 1);
        setenv("SERVER_SIGNATURE", "edhttpd "__DATE__, 1);
        setenv("REQUEST_SCHEME", "http", 1);

        fprintf(stderr, "PHP-CGI executing %s %s\n", php_path, argv[1]);

        // this should never return
        res = execvp(php_path, argv);

        fprintf(stderr, "PHP Handler res: %d\n", res);

        exit(1);
    }

    close(pipefds[1]); // close parent's write pipe so we get an EOF

    while (1) {
        // assume that the read below will generate (possibly) additional headers,
        // and the blank \r\n

        char buf[4096];
        ssize_t actually_read = read(pipefds[0], buf, sizeof(buf));

        if (actually_read <= 0)
            break;

        ssize_t actually_written = write_fully(client->tx_fd, buf, actually_read);
        assert(actually_written == actually_read);
    }
    close(pipefds[0]);

    handled = 1 | 0x8000; // 0x8000 = no more requests on this connection

  cleanup:
    httpd_basic_handler_destroy(bhres);
    return handled;
}

*/


zhash_t *httpd_cookies_parse(struct httpd_request *request)
{
    zhash_t *kvs = zhash_create(sizeof(char*), sizeof(char*),
                                zhash_str_hash, zhash_str_equals);

    // the cookie header may contain multiple cookies
    char *cookie = zhash_str_str_get(request->headers, "Cookie");
    if (cookie) {
        cookie = strdup(cookie);
        int cookie_len = strlen(cookie);

        int start_pos = 0;
        for (int end_pos = 0; end_pos <= cookie_len; end_pos++) {
            if (end_pos == cookie_len || cookie[end_pos] == ';') {
                // found a cookie!
                cookie[end_pos] = 0; // NULL terminate it

                char *c = &cookie[start_pos];
                int c_len = strlen(c);

                do {
                    int eq_pos = 0;
                    while (eq_pos < c_len) {
                        if (c[eq_pos] == '=')
                            break;
                        eq_pos++;
                    }

                    // bad looking cookie, no equals sign?
                    if (c[eq_pos] != '=')
                        break;

                    c[eq_pos] = 0;
                    char *key = c;
                    while (key[0] == ' ') // skip leading white space
                        key = &key[1];

                    key = strdup(key);

                    char *value = &c[eq_pos + 1];
                    int value_len = strlen(value);

                    value = strdup(value);

                    char *oldkey, *oldvalue;
                    if (zhash_put(kvs, &key, &value, &oldkey, &oldvalue)) {
                        free(oldkey);
                        free(oldvalue);
                    }
                } while (0);

                start_pos = end_pos + 1;
            }
        }

        free(cookie);
    }

    return kvs;
}
