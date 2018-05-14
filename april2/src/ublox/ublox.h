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

#ifndef __ublox_h
#define __ublox_h

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>

// Set buffer size >= (max NMEA message size + 1)
#define UBLOX_BUFFER_SIZE  512

#define UBLOX_SYNC1   0xB5
#define UBLOX_SYNC2   0x62

typedef struct {
    uint8_t class;
    uint8_t id;
    uint16_t length;
    uint8_t *payload;
    uint8_t checksum_a;
    uint8_t checksum_b;
} ublox_packet_t;

typedef struct {
    int fd;
    FILE *in;
    void (*ubx_callback) (int64_t time, ublox_packet_t *packet);
    void (*nmea_callback) (int64_t time, const char *msg);
} ublox_t;


ublox_t *ublox_create(const char *portname, int baud);
void ublox_set_ubx_callback(ublox_t *ublox,
                            void (*callback) (int64_t time, ublox_packet_t *packet));
void ublox_set_nmea_callback(ublox_t *ublox,
                             void (*callback) (int64_t time, const char *msg));
pthread_t ublox_start(ublox_t *ublox);
void ublox_command(ublox_t *ublox, uint8_t class, uint8_t id, uint16_t length, uint8_t *payload);
void ublox_destroy(ublox_t *ublox);

void ublox_packet_print(ublox_packet_t *packet);

#endif
