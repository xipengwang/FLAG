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

#include <unistd.h>
#include <stdio.h>
#include "tcp_util.h"

// returns the fd
int tcp_util_connect(const char *hostname, int port)
{
	struct hostent *host = gethostbyname(hostname);
	if (host == NULL)
		return -1;

	int fd = socket(AF_INET, SOCK_STREAM, 0);

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons(0);
	sa.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(fd, (struct sockaddr *) &sa, sizeof (sa)) < 0) {
		close(fd);
		return -1;
	}

	sa.sin_port = htons(port);
	sa.sin_addr = *(struct in_addr *) host->h_addr_list[0];

	if (connect(fd, (struct sockaddr *) &sa, sizeof (sa))) {
		if (errno!=EINPROGRESS) {
			close(fd);
			return -1;
		}
	}

	// prevent "broken pipe" signals.
	signal(SIGPIPE, SIG_IGN);

	return fd;
}

int tcp_util_listen(int port, int listenqueue, int localhost_only)
{
	int fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd == -1) {
        perror("socket");
		return -1;
    }

	// avoid address already in use errors
	int n = 1;
	if (setsockopt (fd, SOL_SOCKET, SO_REUSEADDR, (char*) &n, sizeof(n)) < 0) {
		close(fd);
        perror("setsockopt");
		return -1;
	}

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons(port);
	sa.sin_addr.s_addr = htonl(localhost_only ? INADDR_LOOPBACK : INADDR_ANY);

	if (bind (fd, (struct sockaddr *) &sa, sizeof (sa)) < 0) {
		close(fd);
        perror("bind");
		return -1;
	}

	if (listen(fd, listenqueue)) {
		close(fd);
		perror("listen");
		return -1;
	}

    return fd;
}

int tcp_util_accept(int fd, struct sockaddr *addr, socklen_t *addrlen)
{
    if (addr) {
		memset(addr, 0, sizeof(struct sockaddr));
	}


	int newfd = accept(fd, addr, addrlen);
    if (newfd < 0) {
		perror("accept failed");
        return -1;
	}

	return newfd;
}
