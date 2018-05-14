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

#ifndef _WEBVX_H
#define _WEBVX_H

#include "vx.h"
#include "httpd/httpd.h"
#include "httpd/httpd_websocket.h"

typedef struct webvx webvx_t;
struct webvx
{
    int       port;
    char     *rootdir;
    httpd_t  *httpd;

    zarray_t *canvas_definitions; // struct webvx_canvas_definition*
};

// creates a new webserver listening on port 'port' whose root directory is mapped to
// the specified directory. (I.e., rootdir should contain 'index.html'.) If port is zero,
// an available port is automatically found (NOT IMPLEMENTED).
webvx_t *webvx_create_server(int port, const char *rootdir, const char *default_file);

// creates a new websocket handler for a canvas/protocol named 'name'.
void webvx_define_canvas(webvx_t *webvx, const char *name,
                         void (*on_create_canvas)(vx_canvas_t *vc, const char *name, void *impl),
                         void (*on_destroy_canvas)(vx_canvas_t *vc, void *impl),
                         void *impl);

#endif
