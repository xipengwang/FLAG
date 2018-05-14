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
#include <stdlib.h>
#include <poll.h>
#include <assert.h>
#include <stdint.h>

#include "common/getopt.h"
#include "common/net_util.h"
#include "common/string_util.h"
#include "common/time_util.h"
#include "common/timesync.h"

#include "lcmtypes/raw_t.h"

#define DATA_PORT 2368
#define POSE_PORT 8308

struct sensor
{
    char *data_channel, *pose_channel;
    unsigned long s_addr;
    int message_counts[2];

    timesync_t * timesync;
};

typedef struct state state_t;
struct state {
    lcm_t *lcm;

    getopt_t *gopt;
    int data_fd;
    int pose_fd;

    zarray_t *sensors;
};

/*
  Pass in specifications for the velodyne sensors as "extra" arguments.

  ./velodyne32 192.168.5.201:VELODYNE_DATA:VELODYNE_POSE

 */
int main(int argc, char *argv[])
{
    setlinebuf(stderr);
    setlinebuf(stdout);

    state_t *state = calloc(1, sizeof(state_t));

    state->gopt = getopt_create();
    getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help");

    if (!getopt_parse(state->gopt, argc, argv, 0) || getopt_get_bool(state->gopt, "help")) {
        printf("Usage: %s [options] <velodyne specifications>\n", argv[0]);
        getopt_do_usage(state->gopt);
        printf("\nSpecifications are: <ip>:<data LCM channel>:<pose LCM channel>, e.g.:\n");
        printf(" 192.168.5.201:VELODYNE_DATA:VELODYNE_POSE\n");
        printf("\n");
        exit(-1);
    }

    const zarray_t *extras = getopt_get_extra_args(state->gopt);
    if (zarray_size(extras) == 0) {
        printf("no velodyne 32s specified\n");
        exit(-2);
    }

    state->lcm = lcm_create(NULL);

    // parse sensor configurations
    state->sensors = zarray_create(sizeof(struct sensor*));
    for (int i = 0; i < zarray_size(extras); i++) {
        const char *spec;
        zarray_get(extras, i, &spec);

        zarray_t *toks = str_split(spec, ":");
        if (zarray_size(toks) != 3) {
            printf("Illegal sensor specification. Example: 192.168.5.201:VELODYNE_DATA:VELODYNE_POSE");
            exit(-4);
        }

        const char *ipaddr;
        zarray_get(toks, 0, &ipaddr);
        struct sockaddr_in addrin;
        if (ip4_parse_addr(ipaddr, &addrin, 0, NULL)) {
            printf("Failed to parse IP address %s\n", ipaddr);
            exit(-5);
        }

        struct sensor *sensor = calloc(1, sizeof(struct sensor));
        sensor->s_addr = addrin.sin_addr.s_addr;

        if (1) {
            char *tmp;
            zarray_get(toks, 1, &tmp);
            sensor->data_channel = strdup(tmp);
        }

        if (1) {
            char *tmp;
            zarray_get(toks, 2, &tmp);
            sensor->pose_channel = strdup(tmp);
        }

        sensor->timesync = timesync_create(1e6, (int64_t)3600*1000*1000, 0.00001, 20);

        zarray_add(state->sensors, &sensor);
    }

    state->data_fd = udp_socket_listen(DATA_PORT);
    if (state->data_fd < 0) {
        printf("Couldn't bind to port %d\n", DATA_PORT);
        exit(-1);
    }

    state->pose_fd = udp_socket_listen(POSE_PORT);
    if (state->pose_fd < 0) {
        printf("Couldn't bind to port %d", POSE_PORT);
        exit(-1);
    }

    /////////// main read loop ////////////
    uint64_t last_report_utime = utime_now();

    while (1) {

        if (1) {
            uint64_t now = utime_now();
            double dt = (now - last_report_utime) / 1.0E6;

            if (dt > 1.0) {
                last_report_utime = now;

                printf("%.3f report: \n", now / 1.0E6);

                for (int i = 0; i < zarray_size(state->sensors); i++) {
                    struct sensor *sensor;
                    zarray_get(state->sensors, i, &sensor);

                    uint32_t v = ntohl(sensor->s_addr);
                    printf("     %d.%d.%d.%d  data: %8.2f Hz    pose: %8.2f Hz\n",
                           (v >> 24) & 0xff, (v >> 16) & 0xff, (v >> 8) & 0xff, (v >> 0) & 0xff,
                           sensor->message_counts[0] / dt,
                           sensor->message_counts[1] / dt);

                    sensor->message_counts[0] = 0;
                    sensor->message_counts[1] = 0;
                }
                printf("\n");
            }
        }

        struct pollfd *fds = (struct pollfd[]) { { .fd = state->data_fd,
                                                   .events = POLLIN,
                                                   .revents = 0 },
                                                 { .fd = state->pose_fd,
                                                   .events = POLLIN,
                                                   .revents = 0 } };


        int ret = poll(fds, 2, 1000);
        if (ret == 0)
            continue;

        uint64_t poll_time = utime_now();

        assert(ret > 0);

        for (int pollidx = 0; pollidx < 2; pollidx++) {
            if (fds[pollidx].revents == POLLIN) {
                uint8_t buf[2048];
                struct sockaddr_in src_addr;
                socklen_t src_addr_len = sizeof(src_addr);
                ssize_t recvlen = recvfrom(pollidx == 0 ? state->data_fd : state->pose_fd,
                                           buf, sizeof(buf), 0,
                                           (struct sockaddr*) &src_addr, &src_addr_len);

//                printf("%d   %d   %d\n", pollidx, fds[pollidx].fd, recvlen);

                // find the velodyne spec
                int found = 0;

                for (int sensoridx = 0; sensoridx < zarray_size(state->sensors); sensoridx++) {
                    struct sensor *sensor;
                    zarray_get(state->sensors, sensoridx, &sensor);
                    if (sensor->s_addr == src_addr.sin_addr.s_addr) {
                        found = 1;

                        raw_t msg;
                        memset(&msg, 0, sizeof(raw_t));
                        msg.utime = poll_time;
                        msg.len = recvlen;
                        msg.buf = buf;

                        if(recvlen == 1206)
                        {
                            uint32_t dev_utime = 0;
                            dev_utime |= (msg.buf[1200] & 0xff) << 0;
                            dev_utime |= (msg.buf[1201] & 0xff) << 8;
                            dev_utime |= (msg.buf[1202] & 0xff) << 16;
                            dev_utime |= (msg.buf[1203] & 0xff) << 24;
                            timesync_update(sensor->timesync, poll_time, dev_utime);
                            msg.utime = timesync_get_host_utime(sensor->timesync, dev_utime);
                        }

                        raw_t_publish(state->lcm,
                                      pollidx == 0 ? sensor->data_channel : sensor->pose_channel,
                                      &msg);

                        sensor->message_counts[pollidx] ++;
                    }
                }

                if (!found) {
                    uint32_t v = ntohl(src_addr.sin_addr.s_addr);
                    printf("Warning: received data from unknown velodyne with IP %d.%d.%d.%d\n",
                           (v >> 24) & 0xff, (v >> 16) & 0xff, (v >> 8) & 0xff, (v >> 0) & 0xff);
                }
            }
        }
    }

    return 0;
}
