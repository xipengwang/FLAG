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

#ifndef _DYNAMIXEL_H
#define _DYNAMIXEL_H

#include <stdint.h>

#define DYNAMIXEL_MAX_LEN 256

#define DYNAMIXEL_INST_PING           0x01
#define DYNAMIXEL_INST_READ_DATA      0x02
#define DYNAMIXEL_INST_WRITE_DATA     0x03
#define DYNAMIXEL_INST_REG_WRITE      0x04
#define DYNAMIXEL_INST_ACTION         0x05
#define DYNAMIXEL_INST_RESET_DATA     0x06
#define DYNAMIXEL_INST_SYNC_WRITE     0x83

typedef struct dynamixel_msg dynamixel_msg_t;
struct dynamixel_msg
{
    int len;
    uint8_t *buf;
};

typedef struct dynamixel_bus dynamixel_bus_t;
struct dynamixel_bus
{
    // non-zero on error
    int (*send_command)(dynamixel_bus_t *bus, int id,
                        int instruction,
                        const void *params, int params_len,
                        void *response, int *response_len,
                        int64_t *response_utime);

    void (*destroy)(dynamixel_bus_t *bus);

    union {
        struct {
            int timeout_ms;
            int fd;
        } fd;
        struct {
            void *impl;
        } impl;
    } u;
};

int dynamixel_bus_get_device_model(dynamixel_bus_t *bus, uint8_t id);

void dynamixel_init_instr(uint8_t id,
                          uint8_t instruction,
                          const void *_params, int params_len,
                          void *_out, int *out_len);

#endif
