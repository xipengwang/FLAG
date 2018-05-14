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

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <lcm/lcm.h>

#include "common/getopt.h"

static int running = 1;

struct state {
    const char *outchan;
    lcm_t *lcm;
};

void signal_cb(int signal)
{
    switch (signal) {
        case SIGINT:
            printf("SIGINT: Stop running\n");
            running = 0;
            break;
        default:
            printf("Signal %d not handled\n", signal);
    }
}

void lcm_cb(const lcm_recv_buf_t *rbuf,
            const char *channel,
            void *user)
{
    struct state *state = user;
    lcm_publish(state->lcm, state->outchan, rbuf->data, rbuf->data_size);
}

int main(int argc, char **argv)
{
    setlinebuf(stderr);
    setlinebuf(stdout);
    signal(SIGINT, signal_cb);

    getopt_t *gopt = getopt_create();

    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_string(gopt, 'i', "input-channel", "", "Input LCM channel");
    getopt_add_string(gopt, 'o', "output-channel", "", "Output LCM channel");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(gopt);
        exit(1);
    }

    if (!getopt_was_specified(gopt, "input-channel")) {
        printf("Must specify an input channel\n");
        exit(1);
    }

    if (!getopt_was_specified(gopt, "output-channel")) {
        printf("Must specify an output channel\n");
        exit(1);
    }

    struct state *state = calloc(1, sizeof(struct state));
    state->lcm = lcm_create(NULL);
    state->outchan = getopt_get_string(gopt, "output-channel");
    const char *inchan = getopt_get_string(gopt, "input-channel");

    if (!strcmp(inchan, state->outchan)) {
        printf("Matching input and output channels not allowed\n");
        exit(1);
    }

    lcm_subscribe(state->lcm, inchan, lcm_cb, state);
    while (running) {
        lcm_handle_timeout(state->lcm, 1000);
    }

    lcm_destroy(state->lcm);
    free(state);

    return 0;
}
