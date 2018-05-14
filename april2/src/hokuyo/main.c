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
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
#include <math.h>
#include <assert.h>

#include "scip2.h"

#include "common/getopt.h"
#include "common/time_util.h"
#include "common/timesync.h"
#include "common/zarray.h"
#include "common/zhash.h"

#include "lcmtypes/laser_t.h"
#include "lcm/lcm.h"

#define PI 3.14159265358979323846264338

volatile int watchdog_got_scan = 0;

/** Note: timing synchronization corresponds to the time at which the
 * sensor was point directly backwards. **/
typedef struct state state_t;
struct state
{
    getopt_t *gopt;
    scip2_t *scip;

    zhash_t *properties; // char* => char*. responses from PP, VV, II commands

    int64_t last_summary_utime;
    uint32_t last_scan_count;
    uint32_t scan_count;
    uint32_t rx_bytes;

    timesync_t *ts;

    const char *channel;
    lcm_t *lcm;
};

char *get_property(state_t *state, const char *propname)
{
    char *value;
    if (!zhash_get(state->properties, &propname, &value)) {
        fprintf(stderr, "Missing property: %s", value);
        exit(-1);
    }

    return value;
}

void add_property(zhash_t *zh, const char *key, const char *value)
{
    char *oldkey, *oldvalue;
    char *newkey, *newvalue;
    newkey = strdup(key);
    newvalue = strdup(value);
    if (zhash_put(zh, &newkey, &newvalue, &oldkey, &oldvalue)) {
        free(oldkey);
        free(oldvalue);
    }
}

void do_reset(state_t *state)
{
retry: ; // extra semi-colon to deal with gcc parsing bug.

    zarray_t *response = scip2_transaction(state->scip, "RS", 200);
    if (response == NULL)
        goto retry;

    scip2_response_free(state->scip, response);
}

// My UTM-30LX seems to only support SCIP2.0 mode; sending SCIP2.0
// returns an error code (0x0e).
void do_scip2_mode(state_t *state)
{
    zarray_t *response = scip2_transaction(state->scip, "SCIP2.0", 0);
    scip2_response_free(state->scip, response);
}

// Decode responses from PP, VV, II commands and insert key/value
// pairs into properties hash table.
void decode_properties(state_t *state, zarray_t *response)
{
    if (zarray_size(response) < 2) {
        printf("Invalid response to properties command\n");
        exit(1);
    }

    char *v;
    zarray_get(response, 1, &v);
    if (strncmp(v, "00", 2)) {
        printf("Failure on properties command\n");
        exit(1);
    }

    // break a line like "DMAX:60000;J" into key/value pairs.
    for (int i = 2; i < zarray_size(response); i++) {
        char *line;
        zarray_get(response, i, &line);
        char *colon = index(line, ':');
        char *semi = index(line, ';'); // don't use rindex, because ';' can occur as the checksum.

        if (colon == NULL || semi == NULL)
            continue;

        char *key = strndup(line, colon - line);
        char *value = strndup(colon+1, semi-colon-1);

        char *oldkey, *oldvalue;

        if (zhash_put(state->properties, &key, &value, &oldkey, &oldvalue)) {
            free(oldkey);
            free(oldvalue);
        }
    }
}

// query various information from the sensor
void do_get_info(state_t *state)
{
    if (1) {
        zarray_t *response = scip2_transaction(state->scip, "VV", 0);
        decode_properties(state, response);
        scip2_response_free(state->scip, response);
    }

    if (1) {
        zarray_t *response = scip2_transaction(state->scip, "PP", 0);
        decode_properties(state, response);
        scip2_response_free(state->scip, response);
    }

    if (1) {
        zarray_t *response = scip2_transaction(state->scip, "II", 0);
        decode_properties(state, response);
        scip2_response_free(state->scip, response);
    }
}

void *watchdog_task(void *arg)
{
    while (1) {
        watchdog_got_scan = 0;
        sleep(2);
        if (watchdog_got_scan == 0) {
            printf("Watchdog forcing exit.\n");
            exit(-1);
        }
    }
}


// decode an integer encoded in base 10
static uint32_t ascii_decimal(const char *s, int length)
{
    uint32_t acc = 0;
    for (int i = 0; i < length; i++) {
        acc *= 10;
        acc += s[i] - '0';
    }

    return acc;
}

static uint32_t max(uint32_t a, uint32_t b)
{
    if (a > b)
        return a;
    return b;
}

// decode a urg-formatted number, where each character encodes six
// bytes and has 0x30 added. (multi-byte quantities are big endian).
static uint32_t sixbit_decode(const char *s, int length)
{
    uint32_t acc = 0;
    for (int i = 0; i < length; i++) {
        acc <<= 6;
        uint32_t v = (s[i]-0x30)&0x3f;
        acc += v;
    }

    return acc;
}

typedef struct
{
    zarray_t *response;
    int line;
    int pos;
} scip_stream_t;

static void scip_stream_init(scip_stream_t *ss, zarray_t *response, int firstline)
{
    ss->response = response;
    ss->line = firstline;
    ss->pos = 0;
}

static char scip_stream_next(scip_stream_t *ss)
{
    if (ss->pos == 64) {
        ss->line++;
        ss->pos = 0;
    }

    char *line;
    zarray_get(ss->response, ss->line, &line);
    return line[ss->pos++];
}

static uint32_t scip_stream_get_int(scip_stream_t *ss, int length)
{
    char buf[length];
    for (int i = 0; i < length; i++) {
        buf[i] = scip_stream_next(ss);
    }

    return sixbit_decode(buf, length);
}

// Data with intensity (see UTM-30LX_Specification)
static void on_ME_99(state_t *state, zarray_t *response)
{
    char *line0;
    char *line2;
    zarray_get(response, 0, &line0);
    zarray_get(response, 2, &line2);

    uint32_t step0 = ascii_decimal(&line0[2], 4);
    uint32_t step1 = ascii_decimal(&line0[6], 4);
    uint32_t cluster = ascii_decimal(&line0[10], 2);

    uint32_t timestamp = sixbit_decode(&line2[0], 4);

    int64_t host_utime = utime_now();
    timesync_update(state->ts, host_utime, timestamp);

    int nranges = (step1 - step0 + 1) / max(1, cluster);

    laser_t msg;
    bzero(&msg, sizeof(laser_t));
    msg.utime = timesync_get_host_utime(state->ts, timestamp);
    msg.nranges = nranges;
    msg.ranges = (float*) calloc(nranges, sizeof(float));
    msg.nintensities = nranges;
    msg.intensities = (float*) calloc(nranges, sizeof(float));

    // resolution of the sensor
    char *ares = get_property(state, "ARES");
    char *afrt = get_property(state, "AFRT");

    double radres = (360.0 / atoi(ares)) * PI / 180.0;
    msg.radstep = radres * max(1, cluster);
    msg.rad0 = (step0 - (float) atoi(afrt)) * radres;

    scip_stream_t ss;
    scip_stream_init(&ss, response, 3);

    for (int i = 0; i < nranges; i++) {
        int range_code = scip_stream_get_int(&ss, 3);
        if (range_code < 20)
            msg.ranges[i] =  -range_code; // error code.
        else
            msg.ranges[i] = range_code * 0.001; // convert from mm to meters.

        msg.intensities[i] = scip_stream_get_int(&ss, 3) / 4000.0;
    }

    laser_t_publish(state->lcm, state->channel, &msg);
    watchdog_got_scan = 1;

    free(msg.ranges);
    free(msg.intensities);
}

// Data with range (see SCIP 2.0 specification)
static void on_MD_99(state_t *state, zarray_t *response)
{
    char *line0;
    char *line2;
    zarray_get(response, 0, &line0);
    zarray_get(response, 2, &line2);

    uint32_t step0 = ascii_decimal(&line0[2], 4);
    uint32_t step1 = ascii_decimal(&line0[6], 4);
    uint32_t cluster = ascii_decimal(&line0[10], 2);

    uint32_t timestamp = sixbit_decode(&line2[0], 4);
    int64_t host_utime = utime_now();
    timesync_update(state->ts, host_utime, timestamp);

    int nranges = (step1 - step0 + 1) / max(1, cluster);

    laser_t msg;
    bzero(&msg, sizeof(laser_t));
    msg.utime = timesync_get_host_utime(state->ts, timestamp);
    msg.nranges = nranges;
    msg.ranges = (float*) calloc(nranges, sizeof(float));
    msg.nintensities = 0;
    msg.intensities = NULL;

    // resolution of the sensor
    char *ares = get_property(state, "ARES");
    char *afrt = get_property(state, "AFRT");

    double radres = (360.0 / atoi(ares)) * PI / 180.0;
    msg.radstep = radres * max(1, cluster);
    msg.rad0 = (step0 - (float) atoi(afrt)) * radres;

    scip_stream_t ss;
    scip_stream_init(&ss, response, 3);

    for (int i = 0; i < nranges; i++) {
        int range_code = scip_stream_get_int(&ss, 3);
        if (range_code < 20)
            msg.ranges[i] = -range_code; // error code.
        else
            msg.ranges[i] = range_code * 0.001; // convert from mm to meters.
    }

    laser_t_publish(state->lcm, state->channel, &msg);
    watchdog_got_scan = 1;

    //if (1) {
    //    sync_debug_t sd;
    //    sd.rx_utime = host_utime;
    //    sd.sensor_time = timestamp;
    //    sd.estimated_utime = msg.utime;

    //    sync_debug_t_publish(state->lcm, "SYNC_DEBUG", &sd);
    //}

    free(msg.ranges);
    free(msg.intensities);
}

static void on_99_data(zarray_t *response, void *_a)
{
    state_t *state = (state_t*) _a;

    /////////////////////////////////////////////////////////////////
    // Pass the data to the correct handler
    char *line0;
    zarray_get(response, 0, &line0);

    if (!strncmp(line0, "ME", 2))
        on_ME_99(state, response);
    if (!strncmp(line0, "MD", 2))
        on_MD_99(state, response);

    /////////////////////////////////////////////////////////////////
    // Produce summary information
    for (int i = 0; i < zarray_size(response); i++) {
        char *line;
        zarray_get(response, i, &line);
        state->rx_bytes += strlen(line);
    }

    state->scan_count++;
    int64_t utime = utime_now();
    double dt = (utime - state->last_summary_utime) / 1000000.0;
    if (dt > 1.0) {
        printf("[hokuyo] rate: %.2f Hz  %7.2f kB/s  sync error: %5.3f s  resyncs: %d   devtime: %06x\n",
               (state->scan_count - state->last_scan_count) / dt,
               (state->rx_bytes / dt) / 1024.0,
               state->ts->last_sync_error,
               state->ts->resync_count,
               (int) state->ts->last_device_ticks_wrapping);

        state->last_summary_utime = utime;
        state->last_scan_count = state->scan_count;
        state->rx_bytes = 0;
    }
}

int main(int argc, char *argv[])
{
    state_t *state = (state_t*) calloc(1, sizeof(state_t));

    setlinebuf (stdout);
    state->gopt = getopt_create();

    getopt_add_bool(state->gopt, 'h',"help", 0,"Show this");

    getopt_add_string(state->gopt, 'c', "channel", "HOKUYO_LIDAR", "LC channel name (used if channel-map is empty)");
    getopt_add_string(state->gopt, '\0', "channel-map", "", "serial1=channel1;serial2=channel2;...");
    getopt_add_bool(state->gopt, 'i', "intensities", 0, "Use intensities (reduces angular resolution by half)");

    getopt_add_spacer(state->gopt, "");
    getopt_add_string(state->gopt,'d',  "device",          "/dev/hokuyo",  "Device, e.g. tcp://192.168.0.10:10940");
    getopt_add_bool(state->gopt, '\0', "no-version-check", 0, "Disable firmware version check");
    getopt_add_bool(state->gopt, '\0',"scip-debug", 0,"Show SCIP communications");
    getopt_add_bool(state->gopt, '\0',"time-test", 0, "Measure clock drift");

    if (!getopt_parse(state->gopt, argc, argv, 1) || getopt_get_bool(state->gopt,"help")
        || zarray_size(getopt_get_extra_args(state->gopt)) != 0) {

        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(state->gopt);
        return 0;
    }

    if (1) {
        pthread_t pt;
        pthread_create(&pt, NULL, watchdog_task, NULL);
    }

    printf("Opening device '%s'\n", getopt_get_string(state->gopt, "device"));
    state->scip = scip2_create(getopt_get_string(state->gopt, "device"));
    if (state->scip == NULL) {
        perror(getopt_get_string(state->gopt, "device"));
        return 1;
    }
    printf("Device opened successfully\n");

    state->scip->debug = getopt_get_bool(state->gopt, "scip-debug");

    // Hokuyo has a 1ms resolution clock with 24 bits. It has a low drift rate.
    //timesync_init(&state->ts, 1000, 1L<<24, 0.001, 0.5);
    state->ts = timesync_create(1000, 1L<<24, 0.001, 0.5);

    state->properties = zhash_create(sizeof(char*), sizeof(char*),
                                     zhash_str_hash, zhash_str_equals);
    zhash_t *propdesc = zhash_create(sizeof(char*), sizeof(char*),
                                     zhash_str_hash, zhash_str_equals);

    add_property(propdesc, "ARES", "Number of angular steps per 360deg");
    add_property(propdesc, "AMIN", "Minimum angular step index");
    add_property(propdesc, "AMAX", "Maximum angular step index");
    add_property(propdesc, "AFRT", "Angular index pointing forward");
    add_property(propdesc, "DMAX", "Maximum range (mm)");
    add_property(propdesc, "DMIN", "Minimum range (mm)");
    add_property(propdesc, "FIRM", "Firmware version");
    add_property(propdesc, "MODL", "Model number");
    add_property(propdesc, "LASR", "Laser power status");
    add_property(propdesc, "MESM", "Measurement mode");
    add_property(propdesc, "STAT", "Sensor health status");
    add_property(propdesc, "SERI", "Serial number");
    add_property(propdesc, "PROD", "Product number");
    add_property(propdesc, "PROT", "Protocol Revision");
    add_property(propdesc, "SCAN", "Scan rate (rpm)");
    add_property(propdesc, "SCSP", "Current scan rate (rpm)");
    add_property(propdesc, "SBPS", "Serial bits per second");
    add_property(propdesc, "TIME", "Internal time stamp (ms)");
    add_property(propdesc, "VEND", "Vendor");

    do_reset(state);

//    do_scip2_mode(state);

    do_get_info(state);

    // Display properties
    if (1) {
        zhash_iterator_t vit;
        zhash_iterator_init(state->properties, &vit);
        char *key, *value;
        while (zhash_iterator_next(&vit, &key, &value)) {
            char *desc = NULL;
            if (!zhash_get(propdesc, &key, &desc))
                desc = "Unknown property";

            printf("%35s (%s) = %s\n", desc, key, value);
        }
    }

    state->lcm = lcm_create(NULL);

    if (getopt_get_bool(state->gopt, "time-test")) {
        // This is a crude test for measuring the accuracy of the
        // hokuyo's clock. Note that it does NOT handle the 24bit
        // wrap-around of the clock.
        //
        // Empirically, UTM-30LX seems to have a quartz-quality clock,
        // with drift < 0.01%
        double t0 = utime_now() / 1000000.0;
        double s0 = strtol(get_property(state, "TIME"), NULL, 16) / 1000.0;

        double delta0 = t0 - s0;

        while (1) {
            do_get_info(state);

            double t = utime_now() / 1000000.0;
            double s = strtol(get_property(state, "TIME"), NULL, 16) / 1000.0;

            double delta = t - s;

            printf("delta: %15f, delta rate: %15f, s: %15s\n", (delta - delta0), (delta - delta0) / (t - t0), get_property(state, "TIME"));

            usleep(100000);
        }
    }

    if (!getopt_get_bool(state->gopt, "no-version-check")) {
        // expect a version of the form "1.18.01(09/Jul./2010)"
        int v[3] = { 0, 0, 0};
        int verspos = 0;
        char *vers = get_property(state, "FIRM");

        for (int i = 0; verspos < 3 && i < strlen(vers); i++) {
            char c = vers[i];
            if (c >= '0' && c <= '9') {
                v[verspos] *= 10;
                v[verspos] += (c - '0');
                continue;
            }
            verspos++;
        }

        if (v[0] <= 1 && v[1] <= 18 && v[2] < 1) {
            printf("Firmware version too old. Need 1.18.1 or newer.\n");
            exit(1);
        }
    }

    // pick our LCM channel
    state->channel = getopt_get_string(state->gopt, "channel");
    char *channelmap = strdup(getopt_get_string(state->gopt, "channel-map"));

    // parse a semi-colon delimited list of serial=channel pairs
    // e.g., 00904100=HOKUYO_FRONT;0091533=HOKUYO_BACK
    if (channelmap[0] != 0) {
        const char *pair_serial = channelmap;
        char *pair_channel = NULL;

        char *dev_serial;
        if (!zhash_get(state->properties, "SERI", &dev_serial))
            assert(0);

        int matched = 0;

        for (int i = 0; channelmap[i] != 0; i++) {
            if (channelmap[i] == '=') {
                channelmap[i] = 0; // terminate the serial number
                pair_channel = &channelmap[i+1];
                continue;
            }

            if (channelmap[i] == ';' || channelmap[i+1] == 0) {

                if (channelmap[i] == ';')
                    channelmap[i] = 0; // zero terminate the LCM channel name

                if (pair_channel == NULL || strlen(pair_channel)==0) {
                    printf("No channel specified for serial number %s\n", pair_serial);
                } else {
                    if (!strcmp(pair_serial, dev_serial)) {
                        printf("Matched serial %s, using channel %s\n", pair_serial, pair_channel);
                        state->channel = strdup(pair_channel);
                        matched = 1;
                    }
                }

                // get ready for the next pair
                pair_serial = &channelmap[i+1];
                pair_channel = NULL;
            }
        }

        if (!matched)
            printf("No matching serial number found: using channel %s\n", state->channel);
    }

    scip2_set_99_data_handler(state->scip, on_99_data, state);

    char cmd[1024];
    sprintf(cmd, "MD%04d%04d%02d%1d%02d",
            atoi(get_property(state, "AMIN")),
            atoi(get_property(state, "AMAX")),
            0, // cluster count
            0, // scan interval
            0); // scan count (0 = infinity)

    if (getopt_get_bool(state->gopt, "intensities")) {
        sprintf(cmd, "ME%04d%04d%02d%1d%02d",
                atoi(get_property(state, "AMIN")),
                atoi(get_property(state, "AMAX")),
                2, // cluster count: reduce resolution by a factor of two so we still get full-rate data.
                0, // scan interval
                0); // scan count (0 = infinity)
    }

    // trigger our data.
    zarray_t *response = scip2_transaction(state->scip, cmd, 0);
    scip2_response_free(state->scip, response);

    while (1) {
        sleep(1);
    }
}
