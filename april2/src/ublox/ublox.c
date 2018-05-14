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
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include "common/serial_util.h"
#include "common/io_util.h"
#include "common/time_util.h"

#include "ublox.h"


enum {
    WAIT,
    SYNC,
    HEADER,
    CLASS,
    ID,
    LENGTH1,
    LENGTH2,
    PAYLOAD,
    CHECKSUM
};

static int hex2int(uint8_t c)
{
	if (c>='0' && c<='9')
		return c - '0';
	if (c>='a' && c<='f')
		return c - 'a' + 10;
	if (c>='A' && c<='F')
		return c - 'A' + 10;

	return -10000;
}

static int compute_checksum(const char *buf)
{
    if (buf[0]!='$') {
        //printf("string doesn't start with $\n");
        return -1;
    }

    uint8_t chk = 0;
    int pos = 1;

    while (buf[pos]!=0 && buf[pos]!='*')
        chk ^= buf[pos++];

    return chk;
}

static int checksum_okay(const char *buf)
{
    int len = strlen(buf);
    if (len < 3 || buf[len-3]!='*')
        return 0;

    uint8_t chk = 0;
    int chk0 = hex2int(buf[len-2]);
    int chk1 = hex2int(buf[len-1]);
    if (chk0 < 0 || chk1 < 0)
        return 0;

    chk = (chk0<<4) + chk1;

    if (chk == compute_checksum(buf))
        return 1;

    return 0;
}



ublox_t *ublox_create(const char *portname, int baud)
{
    ublox_t *ublox = (ublox_t *)malloc(sizeof(ublox_t));

    ublox->fd = serial_open(portname, baud, 1);
    ublox->in = fdopen(ublox->fd, "r");
    ublox->ubx_callback = NULL;
    ublox->nmea_callback = NULL;

    if (ublox->fd <= 0) {
        free(ublox);
        return NULL;
    }
    return ublox;
}

static void *ublox_read_loop(void* data)
{
    ublox_t *ublox = (ublox_t *)data;

    char buf[UBLOX_BUFFER_SIZE];

    ublox_packet_t packet;
    uint16_t payloadindex;
    uint16_t payloadcapacity = 0;
    uint8_t cka = 0, ckb = 0;
    int state = WAIT;
    int c;

    while ((c = fgetc(ublox->in)) != EOF) {
        switch (state) {
        case WAIT:
            if (c == UBLOX_SYNC1)
                state = SYNC;

            // Receiving NMEA message
            else if (c == '$') {
                size_t len;

                buf[0] = c;
                fgets(buf+1, UBLOX_BUFFER_SIZE-1, ublox->in);
                len = strlen(buf);

                // Strip newline sequence "\r\n"
                if (len > 1 && buf[len-1] == '\n')
                    buf[len-2] = '\0';

                if (ublox->nmea_callback != NULL && checksum_okay(buf)) {
                    int64_t now = utime_now();
                    ublox->nmea_callback(now, buf);
                } else {
                    fprintf(stderr, "Failed checksum!\n%s\n", buf);
                }
            }
            break;

        case SYNC:
            if (c == UBLOX_SYNC2)
                state = CLASS;
            else
                state = WAIT;
            break;

        case CLASS:
            packet.class = c;
            state = ID;
            break;

        case ID:
            packet.id = c;
            state = LENGTH1;
            break;

        case LENGTH1:
            packet.length = c;
            state = LENGTH2;
            break;

        case LENGTH2:
            packet.length |= (c << 8);
            state = PAYLOAD;
            // If current payload size is too small, allocate a larger one
            if (packet.length > payloadcapacity) {
                if (payloadcapacity != 0)
                    free(packet.payload);
                packet.payload = (uint8_t*)malloc(packet.length);
                payloadcapacity = packet.length;
            }
            payloadindex = 0;

            // Compute checksum over class, id, and length
            cka = ckb = 0;
            {
                uint8_t data[] = {packet.class, packet.id,
                                  packet.length, packet.length >> 8};
                int j;
                for (j = 0; j < 4; j += 1) {
                    cka += data[j];
                    ckb += cka;
                }
            }
            break;

        case PAYLOAD:
            if (payloadindex < packet.length) {
                packet.payload[payloadindex++] = c;
                cka += c;
                ckb += cka;
            } else {
                packet.checksum_a = c;
                state = CHECKSUM;
            }
            break;

        case CHECKSUM:
            packet.checksum_b = c;
            if (packet.checksum_a == cka && packet.checksum_b == ckb) {
                if (ublox->ubx_callback != NULL) {
                    int64_t now = utime_now();
                    ublox->ubx_callback(now, &packet);
                }
            } else {
                fprintf(stderr, "UBX packet %02x %02x : Failed checksum! ",
                        packet.class, packet.id);
            }
            state = WAIT;
            break;

        default:
            state = WAIT;
        }
    }

    return NULL;
}

void ublox_set_ubx_callback(ublox_t *ublox,
                            void (*callback) (int64_t time, ublox_packet_t *packet))
{
    ublox->ubx_callback = callback;
}

void ublox_set_nmea_callback(ublox_t *ublox,
                             void (*callback) (int64_t time, const char *msg))
{
    ublox->nmea_callback = callback;
}

pthread_t ublox_start(ublox_t *ublox)
{
    pthread_t pid;
    pthread_create(&pid, NULL, ublox_read_loop, ublox);

    return pid;
}

void ublox_command(ublox_t *ublox, uint8_t class, uint8_t id, uint16_t length,
                   uint8_t *payload)
{
    uint8_t header[6];
    uint8_t cka = 0, ckb = 0;
    int i;

    header[0] = UBLOX_SYNC1;
    header[1] = UBLOX_SYNC2;
    header[2] = class;
    header[3] = id;
    header[4] = (uint8_t)length;
    header[5] = (uint8_t)(length >> 8);

    // Compute checksum
    for (i = 2; i < 6; i += 1) {
        cka += header[i];
        ckb += cka;
    }
    for (i = 0; i < length; i += 1) {
        cka += payload[i];
        ckb += cka;
    }

    write_fully(ublox->fd, header, 6);
    if (payload != NULL)
        write_fully(ublox->fd, payload, length);
    header[0] = cka;  // use header as a tmp buffer
    header[1] = ckb;
    write_fully(ublox->fd, header, 2);
}

void ublox_destroy(ublox_t *ublox)
{
    fclose(ublox->in);
    //close(ublox->fd);
    free(ublox);
}

void ublox_packet_print(ublox_packet_t *packet)
{
    int i;

    printf("UBX packet %02x %02x : length %d [ ", packet->class, packet->id,
           packet->length);
    for (i = 0; i < packet->length; i += 1)
        printf("%02x ", packet->payload[i]);
    printf("] checksum %02x %02x\n", packet->checksum_a, packet->checksum_b);
}
