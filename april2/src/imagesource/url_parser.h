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

#ifndef _URL_PARSER_H
#define _URL_PARSER_H

typedef struct url_parser url_parser_t;

// In the examples below, consider the input:
// URL = http://www.google.com:8080/search?q=a

url_parser_t *url_parser_create(const char *s);
void url_parser_destroy(url_parser_t *urlp);

// e.g., "http://"
const char* url_parser_get_protocol(url_parser_t *urlp);

// e.g., "www.google.com"
const char* url_parser_get_host(url_parser_t *urlp);

// "/search"  (and if no path is specified, just "/")
const char* url_parser_get_path(url_parser_t *urlp);

// e.g. 8080 (or -1 if no port specified)
int url_parser_get_port(url_parser_t *urlp);

// returns null def if no parameter specified.
const char* url_parser_get_parameter(url_parser_t *urlp, const char *key, const char *def);

// how many parameters were manually specified?
int url_parser_num_parameters(url_parser_t *urlp);

// what was the name of the nth specified parameter?
const char* url_parser_get_parameter_name(url_parser_t *urlp, int idx);
const char* url_parser_get_parameter_value(url_parser_t *urlp, int idx);

#endif
