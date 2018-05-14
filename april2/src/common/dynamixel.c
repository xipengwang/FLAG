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

#include <stdlib.h>
#include <string.h>

#include "dynamixel.h"

void dynamixel_init_instr(uint8_t id,
                          uint8_t instruction,
                          const void *_params, int params_len,
                          void *_out, int *out_len)
{
    uint8_t *out = _out;
    const uint8_t *params = _params;

    out[0] = 255;   // MAGIC
    out[1] = 255;   // MAGIC
    out[2] = id;    // device id
    out[3] = params_len + 2;
    out[4] = instruction;

    for (int i = 0; i < params_len; i++)
        out[5 + i] = params[i];

    uint8_t checksum = 0;
    for (int i = 2; i < params_len + 6 - 1; i++)
        checksum += out[i];

    out[5 + params_len] = checksum ^ 0xff;
    *out_len = 5 + params_len + 1;
}

///////////////////////////////////////////////////
int dynamixel_bus_get_device_model(dynamixel_bus_t *bus, uint8_t id)
{
    uint8_t cmd[] = { 0x00, 0x03 };

    uint8_t resp[DYNAMIXEL_MAX_LEN];
    int resp_len;
    int64_t resp_utime;

    if (bus->send_command(bus, id,
                          DYNAMIXEL_INST_READ_DATA,
                          cmd, sizeof(cmd),
                          resp, &resp_len, &resp_utime))
        return -1;


    return resp[1] + (resp[2] << 8);
}

/*
void dynamixel_set_id(dynamixel_bus_t *bus, int id, int newid)
{
    assert(newid >=0 && newid < 254);

    dynamixel_msg_t *msg = dynamixel_msg_create_empty(2);
    msg->buf[0] = 0x03;
    msg->buf[1] = (newid & 0xff);
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    if (resp == NULL || resp->len < 1 || resp->buf[0] != 0) {
        printf("set_id failed for %d. Aborting in order to avoid EEPROM wear-out.", device->id);
        exit(-1);
    }

    dynamixel_msg_destroy(msg);
    dynamixel_msg_destroy(resp);
}

void dynamixel_set_baud(dynamixel_device_t *device, int baud)
{
    int code = 0;

    switch (baud) {
        case 1000000:
            code = 1;
            break;
        case 500000:
            code = 3;
            break;
        case 115200:
            code = 16;
            break;
        case 57600:
            code = 24;
            break;
        default:
            // Unknown baud rate
            assert(0);
    }

    dynamixel_msg_t *msg = dynamixel_msg_create_empty(2);
    msg->buf[0] = 0x04;
    msg->buf[1] = code;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    if (resp == NULL || resp->len < 1 || resp->buf[0] != 0) {
        printf("set_baud failed for %d. Aborting in order to avoid EEPROM wear-out.", device->id);
        exit(-1);
    }

    dynamixel_msg_destroy(msg);
    dynamixel_msg_destroy(resp);
}

void dynamixel_set_id(dynamixel_device_t *device, int newid)
{
    assert(newid >=0 && newid < 254);

    dynamixel_msg_t *msg = dynamixel_msg_create_empty(2);
    msg->buf[0] = 0x03;
    msg->buf[1] = (newid & 0xff);
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    if (resp == NULL || resp->len < 1 || resp->buf[0] != 0) {
        printf("set_id failed for %d. Aborting in order to avoid EEPROM wear-out.", device->id);
        exit(-1);
    }

    dynamixel_msg_destroy(msg);
    dynamixel_msg_destroy(resp);
}

void dynamixel_set_baud(dynamixel_device_t *device, int baud)
{
    int code = 0;

    switch (baud) {
        case 1000000:
            code = 1;
            break;
        case 500000:
            code = 3;
            break;
        case 115200:
            code = 16;
            break;
        case 57600:
            code = 24;
            break;
        default:
            // Unknown baud rate
            assert(0);
    }

    dynamixel_msg_t *msg = dynamixel_msg_create_empty(2);
    msg->buf[0] = 0x04;
    msg->buf[1] = code;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    if (resp == NULL || resp->len < 1 || resp->buf[0] != 0) {
        printf("set_baud failed for %d. Aborting in order to avoid EEPROM wear-out.", device->id);
        exit(-1);
    }

    dynamixel_msg_destroy(msg);
    dynamixel_msg_destroy(resp);
}
*/
