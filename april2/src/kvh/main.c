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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "lcm/lcm.h"
#include "common/time_util.h"
#include "common/timesync.h"
#include "common/getopt.h"
#include "common/serial_util.h"
#include "common/io_util.h"
#include "lcmtypes/kvh_t.h"
#include "lcmtypes/alarm_t.h"

typedef struct state state_t;
struct state
{
    getopt_t *gopt;
    lcm_t    *lcm;

    int       decimate;

    // how many messages have we integrated during this integration
    // period?  (will reach 'decimate')
    int       decimate_count;

    // log all raw data
    int64_t   *log_utimes;
    int32_t   *log_samples;

    // integration since the beginning of time
    int64_t   integrator;
    int64_t integrator_utime0;

    // average rate data over the decimation integration period
    int64_t   rate_integrator;
    int64_t   rate_usecs;

    int64_t   last_sample_utime;

    int64_t   last_error_utime;
    int       last_error_flags;

    timesync_t *ts;
    int64_t     ts_count; // our sensor "clock"
};

int main(int argc, char *argv[])
{
    setlinebuf(stdout);
    setlinebuf(stderr);

    state_t *state = calloc(1, sizeof(state_t));
    state->gopt = getopt_create();
    getopt_add_bool(state->gopt, 'h', "help", 0, "Show usage");
    getopt_add_string(state->gopt, 'd', "device", "/dev/kvh", "KVH device");

    if (!getopt_parse(state->gopt, argc, argv, 1) || getopt_get_bool(state->gopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(state->gopt);
        return 1;
    }

    state->lcm = lcm_create(NULL);
    state->decimate = 10;
    state->log_utimes = calloc(sizeof(int64_t), state->decimate);
    state->log_samples = calloc(sizeof(int32_t), state->decimate);

    const char *device = getopt_get_string(state->gopt, "device");
    int fd = serial_open(device, 115200, 1);
    if (fd < 0) {
        printf("Couldn't open %s\n", device);
        exit(1);
    }

/*    FILE *f = fdopen(fd, "r");
    if (!f) {
        printf("Couldn't open %s\n", device);
        exit(1);
    }
*/

    printf("kvh: device %s successfully opened\n", device);

    int phase_counters[6]; // what is the counter for each phase?
    int phase_good[6];     // how many consecutive good phases?
    int phase_lock = -1;   // non-zero when locked
    uint8_t buffer[6];     // store the last 6 characters

    for (int i = 0; i < 6; i++) {
        phase_counters[i] = 0;
        phase_good[i] = 0;
        buffer[i] = 0;
    }

    state->ts = timesync_create(1000, INT64_MAX, 0.05, 0.1);
    state->ts_count = 0;

    for (int phase = 0; 1; phase = (phase + 1) % 6) {

        uint8_t c;
        int res = read_timeout(fd, &c, 1, 1000);
        if (res == 0) {
            printf("read timeout...\n");
            continue;
        }
        if (res < 0) {
            printf("read error, exiting.\n");
            exit(1);
        }
/*
// generate lost characters in order to test synchronization
while ((random() % 500) == 0) {
printf("dropping byte\n");
c = fgetc(f);
}
*/

        buffer[phase] = c;

        int this_counter = c >> 6;
        if (this_counter == ((phase_counters[phase] + 1) & 3)) {
            phase_good[phase]++;
        } else {
            phase_good[phase] = 0;

            if (phase == phase_lock) {
                alarm_t alarm = { .utime = utime_now(),
                                  .topic = "KVH",
                                  .message = "KVH lost lock" };
                alarm_t_publish(state->lcm, "ALARM_KVH", &alarm);

                printf("lock lost\n");
                phase_lock = -1;
                for (int i = 0; i < 6; i++)
                    phase_good[i] = 0;

                state->last_error_utime = utime_now();
                state->last_error_flags = 1;
                timesync_reset(state->ts);
            }
        }
        phase_counters[phase] = this_counter;

        // if we're not locked, is there a phase that looks a lot
        // better than any of the other phases?
        if (phase_lock == -1 && phase == 0) {
            printf("no lock: ");
            for (int i = 0; i < 6; i++)
                printf("%5d ", phase_good[i]);
            printf("\n");

            int best = 0;
            for (int i = 0; i < 6; i++)
                if (phase_good[i] > phase_good[best])
                    best = i;

            int nextbest = (best + 1) % 6; // anything other than best.
            for (int i = 0; i < 6; i++)
                if (i != best && phase_good[i] > phase_good[nextbest])
                    nextbest = i;

            if (phase_good[best] > 20 && phase_good[nextbest] < 3) {
                phase_lock = best;
                printf("locked [phase %d]\n", phase_lock);
                state->last_sample_utime = utime_now();
                if (state->integrator_utime0 == 0)
                    state->integrator_utime0 = state->last_sample_utime;
            }
        }

        // if we are locked, process a record
        if (phase_lock >= 0 && ((phase + 1) % 6) == phase_lock) {

            // this is the utime of the last character we've read.
            state->ts_count++;
            if (timesync_update(state->ts, utime_now(), state->ts_count)) {
                state->last_error_utime = utime_now();
                state->last_error_flags = 4;

                static int first_sync_error = 1;

                if (!first_sync_error) {
                    alarm_t alarm = { .utime = utime_now(),
                                      .topic = "KVH",
                                      .message = "KVH timesync lost" };
                    alarm_t_publish(state->lcm, "ALARM_KVH", &alarm);
                }

                first_sync_error = 0;
            }

            int64_t utime = timesync_get_host_utime(state->ts, state->ts_count);

            uint8_t data[6] = { buffer[(phase + 1) % 6],
                                buffer[(phase + 2) % 6],
                                buffer[(phase + 3) % 6],
                                buffer[(phase + 4) % 6],
                                buffer[(phase + 5) % 6],
                                buffer[(phase + 6) % 6] };

            int32_t validity = (data[0] & 0x10);
            if (!validity) {
                state->last_error_utime = utime_now();
                state->last_error_flags = 2;

                alarm_t alarm = { .utime = utime_now(),
                                  .topic = "KVH",
                                  .message = "KVH not valid" };
                alarm_t_publish(state->lcm, "ALARM_KVH", &alarm);

                printf("%15.3f not valid\n", utime_now() / 1.0E6);
                // if not valid for more than 5 seconds at beginning,
                // or transition to non-valid after validity, emit warning.
                continue;
            }

            int32_t sample = ((data[3] & 0x3f) << 16) |
                (data[4] << 8) |
                (data[5]);

            if (sample &  0x00200000)
                sample |= 0xffc00000; // sign extend

            // do a timesync observation

            ////////////////////////////////////////
            // handle the sample.
            int64_t usecs = utime - state->last_sample_utime;

            usecs = 1.0E6 / 1000;

            int64_t acc = sample * usecs;

            state->integrator += acc;

            state->log_utimes[state->decimate_count] = utime_now();
            state->log_samples[state->decimate_count] = sample;

            state->rate_integrator += acc;
            state->rate_usecs += usecs;
            state->decimate_count++;

            state->last_sample_utime = utime;

            if (state->decimate_count == state->decimate) {
                kvh_t msg;
                memset(&msg, 0, sizeof(kvh_t));
                msg.utime = utime;
                msg.scale = 476.8e-12 * M_PI / 180;
                msg.integrator = state->integrator;
                msg.integrator_utime0 = state->integrator_utime0;
                msg.rate = msg.scale * state->rate_integrator / (state->rate_usecs / 1.0E6);
                msg.last_error_utime = state->last_error_utime;
                msg.last_error_flags = state->last_error_flags;
                msg.nlog = state->decimate;
                msg.log_utimes = state->log_utimes;
                msg.log_samples = state->log_samples;

                kvh_t_publish(state->lcm, "KVH", &msg);

                state->rate_integrator = 0;
                state->rate_usecs = 0;
                state->decimate_count = 0;

                static int64_t last_report_utime = 0;
                int64_t now = utime_now();
                if ((now - last_report_utime) > 1E6) {
                    printf("%15.3f raw: %8d integrator: %15"PRId64" yaw: %8.1f rate: %8.4g\n",
                           utime_now() / 1.0E6,
                           sample, msg.integrator, msg.integrator * msg.scale, msg.rate);
                    last_report_utime = now;
                }
            }

/*            for (int i = 0; i < 6; i++)
              printf("%02x ", data[i]);
              printf(" %08x\n", sample); */
        }
    }
}
