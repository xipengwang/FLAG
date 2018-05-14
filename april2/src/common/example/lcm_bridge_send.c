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
#include <lcm/lcm.h>

#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>

#include "common/udpr.h"
#include "common/getopt.h"
#include "common/net_util.h"
#include "common/rand_util.h"
#include "common/time_util.h"

typedef struct{
    lcm_t * lcm;

    pthread_mutex_t mutex;
    pthread_cond_t cond;


    struct sockaddr_in addr;
    int port;

    int nbytes;
    uint8_t * bytes;

    uint64_t serial;

    udpr_send_params_t params;
    int rate;

} state_t;

void print_guid(const uint8_t *p)
{
    for (int i = 0; i < 32; i++)
        printf("%02x", p[i]);
}

void * send_loop(void * impl){
    state_t * state = impl;

    while(1)
    {
        int nbytes_local;
        uint8_t * bytes_local = NULL;
        pthread_mutex_lock(&state->mutex);
        while(state->bytes == NULL) {
            pthread_cond_wait(&state->cond, &state->mutex);
        }
        nbytes_local = state->nbytes;
        bytes_local = state->bytes;

        state->nbytes = 0;
        state->bytes = NULL;

        pthread_mutex_unlock(&state->mutex);

        static int last_send = 0;

        uint8_t guid[32];
        for (int i = 0; i < 32; i++)
            guid[i] = randi_uniform(0, 256);

        guid[0] = (state->serial >> 24) & 0xff;
        guid[1] = (state->serial >> 16) & 0xff;
        guid[2] = (state->serial >> 8) & 0xff;
        guid[3] = (state->serial >> 0) & 0xff;

	//print_guid(guid);
	//printf("\n");

        int res = udpr_send(&state->params,
                                    &state->addr,
                                    bytes_local, nbytes_local, guid);

        printf("sent %d bytes, bytes[0] = %d\n", nbytes_local, bytes_local[0]);

        state->serial++;
        free(bytes_local);
    }

}

void on_lcm(const lcm_recv_buf_t *rbuf, const char * channel, void * impl){
    state_t * state = impl;

    pthread_mutex_lock(&state->mutex);
    if(state->bytes != NULL)
        free(state->bytes);

    state->nbytes = rbuf->data_size;
    state->bytes = malloc(state->nbytes);
    memcpy(state->bytes, rbuf->data, state->nbytes);
    pthread_cond_broadcast(&state->cond);

    pthread_mutex_unlock(&state->mutex);
}


int main(int argc, char *argv[])
{
    srand(utime_now());
    getopt_t *gopt = getopt_create();

    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");
    getopt_add_string(gopt, 'c', "channel", "", "LCM regex");
    getopt_add_string(gopt, 'a', "addr", "IPv4_addr", "IPv4 address to send messages to");
    getopt_add_int(gopt, 'r', "rate", "1", "max rate to send (Hz)");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options] \n", argv[0]);
        getopt_do_usage(gopt);
        exit(0);
    }

    state_t * state = calloc(1, sizeof(state_t));

    udpr_send_params_init(&state->params);
    state->params.send_sleep_us = 1e3;
    state->params.verbosity = 1;

    state->lcm = lcm_create(NULL);
    assert(state->lcm);

    if(ip4_parse_addr(getopt_get_string(gopt, "addr"), &state->addr, state->port, &state->port)) {
        printf("Failed to parse %s\n", getopt_get_string(gopt, "addr"));
        printf("Usage: %s [options] \n", argv[0]);
        getopt_do_usage(gopt);
        exit(0);
    }

    state->rate = getopt_get_int(gopt, "rate");

    pthread_t send_thread;
    pthread_create(&send_thread, NULL, send_loop, state);
    lcm_subscribe(state->lcm, getopt_get_string(gopt, "channel"), on_lcm, state);

    while(1)
        lcm_handle(state->lcm);


    return 0;
}
