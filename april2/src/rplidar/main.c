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
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>

#include "rplidar.h"

#include "common/getopt.h"
#include "common/serial_util.h"


#define VERBOSE 1

/* Set up serial communication with the RPLIDAR and broadcast LCM on
 * an appropriate channel */
typedef struct state
{
    getopt_t *gopt;

    // Device file descriptor
    int dev;
    bool dev_stopped;

    // Thread stuff
    pthread_t scan_thread;
    pthread_t lcm_thread;
    pthread_mutex_t device_lock;

    // LCM stuff
    const char *channel;
    // char *channel;
    lcm_t *lcm;
} state_t;
static state_t *state;

static void sig_handler(int signo)
{
    rp_lidar_stop(state->dev);
}

static void* scan_loop(void *args)
{
    state_t *state = (state_t*)args;

    printf("Beginning scans...\n");
    // This loops forever, barring an error
    rp_lidar_scan(state->dev, state->lcm, state->channel);
    printf("Terminating rplidar...\n");

    return NULL;
}

static void *lcm_loop(void *user)
{
    state_t *state = user;

    while (1) {
        lcm_handle_timeout(state->lcm, 1000);
    }

    return NULL;
}

int main(int argc, char* argv[])
{
    state = (state_t*) calloc(1, sizeof(state_t));

    state->gopt = getopt_create();
    getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string(state->gopt, 'd', "device", "/dev/ttyUSB0", "Serial device");
    getopt_add_int(state->gopt, 'b', "baud", "115200", "Baud rate");

    if (!getopt_parse(state->gopt, argc, argv, 1) || getopt_get_bool(state->gopt, "help")) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(state->gopt);
        return 0;
    }

    signal(SIGTERM, sig_handler);
    signal(SIGINT, sig_handler);

    // Open device
    const char *name = getopt_get_string(state->gopt, "device");
    // char *name = getopt_get_string(state->gopt, "device");
    int baud = getopt_get_int(state->gopt, "baud");
    state->dev = serial_open(name, baud, 0);

    if (state->dev == -1) {
        printf("ERR: Could not open device at %s\n", name);
        return -1;
    }

    state->lcm = lcm_create(NULL);
    pthread_create(&state->lcm_thread, NULL, lcm_loop, state);
    state->channel = getopt_get_string(state->gopt, "channel");

    // Check device health
    if (rp_lidar_check_health(state->dev) != HEALTH_GOOD) {
        printf("rplidar bad health");
        return -2;
    }

    // Check device info
    if (VERBOSE)
        rp_lidar_check_info(state->dev);

    // Begin scanning
    pthread_create(&state->scan_thread, NULL, scan_loop, state);
    pthread_join(state->scan_thread, NULL);

    lcm_destroy(state->lcm);
    getopt_destroy(state->gopt);
    serial_close(state->dev);
    free(state);

    return 0;
}
