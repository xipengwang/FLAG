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

#ifndef _UDP_UTIL_H
#define _UDP_UTIL_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <inttypes.h>

/** Convert a string like "192.168.1.4:12345" into a sockaddr. **/
int ip4_parse_addr(const char *s, struct sockaddr_in *addr, int default_port, int *_actual_port);

/** Create a socket for sending UDP messages, using a specified FROM
 * interface. addr can be NULL.**/
int udp_socket_create(struct sockaddr_in *addr, int broadcast);

/** make and bind a udp socket to a specified port. Returns the fd. **/
int udp_socket_listen(int port);

/** return the local port number for a socket. **/
int udp_socket_get_port(int sock);

// convenience method that sends a one-off udp message
// return 0 on success
int udp_send(const char *ipaddr, int port, void *data, int datalen);

#define udp_send_string(ipaddr, port, string) udp_send(ipaddr, port, string, strlen(string))

#endif
