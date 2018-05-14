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

#include "rplidar.h"

#include <math.h>
#include <stdio.h>
#include <sys/ioctl.h>

#include "common/serial_util.h"
#include "common/io_util.h"
#include "common/time_util.h"

#define VERBOSE 0

volatile int halt = 0;

// Debugging
static void dump(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; ++i)
        printf("%x ", buf[i]);
    printf("\n");
}

int rp_lidar_setDTR(int dev, unsigned short level)
{
    int status;
    if (ioctl(dev, TIOCMGET, &status) == -1) {
        perror("setDTR(): TIOCMGET");
        return 0;
    }
    if (level) status |= TIOCM_DTR;
    else status &= ~TIOCM_DTR;    if (ioctl(dev, TIOCMSET, &status) == -1) {
        perror("setDTR: TIOCMSET");
        return 0;
    }
    return 1;
}

void rp_lidar_start_spin(int dev)
{
    rp_lidar_setDTR(dev, DTR_LEVEL_LOW);
}

void rp_lidar_stop_spin(int dev)
{
    rp_lidar_setDTR(dev, DTR_LEVEL_HIGH);
}

/* Send a request to the device. Note that not all requests have responses, and
 * some requests have more than one response.
 */
static void send_command_raw(int dev, uint8_t request, uint8_t *payload, uint8_t len)
{
    uint8_t *req = (uint8_t*) malloc((4 + len)*sizeof(uint8_t));
    req[0] = MAGIC_0;
    req[1] = request;
    req[2] = len;

    int i;
    for (i = 0; i < len; ++i)
        req[3+i] = payload[i];

    // Checksum
    int csidx = 4+len-1;
    req[csidx] = 0;
    for (i = 0; i < csidx; ++i)
        req[csidx] ^= req[i];

    // Write to device
    int res = write(dev, req, 4+len);
    if (res != 4+len)
        printf("ERR: There was a problem writing to the devices %i\n",request);
    free(req);
}

static void read_response_descriptor(int dev, rp_descriptor_t *rd)
{
    rd->len = -1;

    // First, look for the magic bytes
    int res = 0;
    int header_have = 0;
    uint8_t buf[7];
    while (header_have < 2) {
        res = read_fully_timeout(dev, buf+header_have, 2-header_have, TIMEOUT_MS);
        if (res < 1)
            return;

        // Check for magic bytes
        if (buf[0] == MAGIC_0 && buf[1] == MAGIC_1)
           break;

        // Otherwise, shift buffer and read one more byte in
        header_have = 1;
        buf[0] = buf[1];
    }

    // Read in last 5 bytes of response descriptor
    res = read_fully_timeout(dev, buf+2, 5, TIMEOUT_MS);
    if (res < 1)
        return;
    if (VERBOSE)
        dump(buf, 7);

    rd->len = (buf[2] << 22) | (buf[3] << 14) | (buf[4] << 6) | ((buf[5] & 0xF7) >> 2);
    rd->send_mode = (0x3 & buf[5]);
    rd->data_type = buf[6];

    return;
}

void rp_lidar_stop(int dev)
{
    halt = 1;
    send_command_raw(dev, REQUEST_STOP, NULL, 0);
}

void rp_lidar_reset(int dev)
{
    halt = 0;
    send_command_raw(dev, REQUEST_RESET, NULL, 0);
}

void rp_lidar_scan(int dev, lcm_t *lcm, const char *channel)
// void rp_lidar_scan(int dev, lcm_t *lcm, char *channel)
{
    send_command_raw(dev, REQUEST_SCAN, NULL, 0);

    // Reading info back. Multiple response. Loops forever!
    rp_descriptor_t rd;
    read_response_descriptor(dev, &rd);
    if (rd.len < 0) {
        printf("ERR: Could not read back scan descriptor\n");
        return;
    }
    printf("After if command raw %i\n", halt);

    float d2r = 2*M_PI/360;

    // Gather information forever and broadcast complete scans
    // Scan packets are 5 bytes
    int res;
    uint8_t buf[5];
    int32_t count = 0;
    float ranges[2000];
    float thetas[2000];
    int64_t times[2000];
    float intensities[2000];

    int64_t now;
    int16_t angle, range;
    int8_t quality;

    unfixed_laser_t laser;
    laser.ranges = ranges;
    laser.thetas = thetas;
    laser.times = times;
    laser.intensities = intensities;
    while (!halt) {
        // Read in bytes
        res = read_fully_timeout(dev, buf, 5, TIMEOUT_MS);
        now = utime_now();
        if (res < 1) {
            if (VERBOSE)
                printf("ERR: Could not read range return\n");
            continue;
        }

        // Check for new scan
        if ((buf[0] & 0x1) && !(buf[0] & 0x2)) {
            if (count) {
                laser.nranges = count;
                laser.nintensities = count;
                unfixed_laser_t_publish(lcm, channel, &laser);
                count = 0;
            }
            laser.utime = now;
        }

        quality = (buf[0] & 0xfc) >> 2;
        angle = ((buf[1] & 0xfe) >> 1) | (buf[2] << 7);
        range = buf[3] | (buf[4] << 8);

        ranges[count] = (range/4.0f)/1000;
        thetas[count] = (angle/64.0f)*d2r;
        times[count] = now;
        intensities[count] = (float)quality/0x3f;

        count++;
    }

    halt = 0;
}

void rp_lidar_force_scan(int dev, lcm_t *lcm, char *channel)
{
    printf("NOT FULLY IMPLEMENTED\n");
    send_command_raw(dev, REQUEST_FORCE_SCAN, NULL, 0);
}

void rp_lidar_check_info(int dev)
{
    send_command_raw(dev, REQUEST_GET_INFO, NULL, 0);

    // Read info back. Single response
    rp_descriptor_t rd;
    read_response_descriptor(dev, &rd);
    if (rd.len < 0) {
        printf("ERR: Could not read back device info descriptor\n");
        return;
    }

    // Next 20 bytes are information
    int i;
    uint8_t buf[20];
    int res = read_fully_timeout(dev, buf, 20, TIMEOUT_MS);
    if (res < 0) {
        printf("ERR: Could not read back device info\n");
        return;
    }
    if (VERBOSE)
        dump(buf, 20);

    printf("Model: %d\nFirmware: %d.%d\nHardware: %d\nSerial: ", buf[0], buf[2], buf[1], buf[3]);
    for (i = 4; i < 20; i++)
        printf("%x ", buf[i]);
    printf("\n");
}

int rp_lidar_check_health(int dev)
{
    send_command_raw(dev, REQUEST_GET_HEALTH, NULL, 0);

    // Read health back. Single response
    rp_descriptor_t rd;
    read_response_descriptor(dev, &rd);
    if (rd.len < 0) {
        printf("ERR: Could not read back device health descriptor\n");
        return 2;
    }

    // Next 3 bytes are information
    uint8_t buf[3];
    int res = read_fully_timeout(dev, buf, 3, TIMEOUT_MS);
    if (res < 0) {
        printf("ERR: Could not read back device health\n");
        return 2;
    }

    if (VERBOSE)
        dump(buf, 3);

    if (buf[0] != HEALTH_GOOD) {
        switch (buf[0]) {
            case HEALTH_ERROR:
                printf("Health ERROR: ");
                break;
            case HEALTH_WARN:
                printf("Health WARNING: ");
                break;
        }
        printf("%x %x\n", buf[1], buf[2]);
    }

    return buf[0];
}
