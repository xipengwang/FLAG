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
#include <math.h>
#include "dynamixel.h"
#include "dynamixel_servo.h"

static inline double dclamp(double v, double min, double max)
{
    if (v < min)
        v = min;
    if (v > max)
        v = max;
    return v;
}

static inline double to_radians(double deg)
{
    return deg * M_PI / 180;
}

static int generic_set_goal(dynamixel_servo_t *servo, double radians, int pos_bits, double speed, double torque)
{
    speed = dclamp(speed, 0, 1);
    torque = dclamp(torque, 0, 1);
    radians = dclamp(radians, servo->radians_min, servo->radians_max);

    int posv = (int) round((radians - servo->radians_min) / (servo->radians_max - servo->radians_min) * pos_bits);

    int speedv = (int) (speed * 0x3ff);
    int torquev = (int) (torque * 0x3ff);

    uint8_t params[] = { 0x1e,
                         posv & 0xff,
                         (posv >> 8) & 0xff,
                         speedv & 0xff,
                         (speedv >> 8) & 0xff,
                         torquev & 0xff,
                         (torquev >> 8) & 0xff };

    uint8_t response[DYNAMIXEL_MAX_LEN];
    int response_len;
    int64_t response_utime;

    if (servo->bus->send_command(servo->bus, servo->id,
                                 DYNAMIXEL_INST_WRITE_DATA,
                                 params, sizeof(params),
                                 response, &response_len, &response_utime))
        return -1;

    // Handle torque limit s.t. 0 torque actually sets torque enable bit to 0.
    uint8_t torque_params[] = { 0x18, torquev ? 1 : 0};
    if (servo->bus->send_command(servo->bus, servo->id,
                                 DYNAMIXEL_INST_WRITE_DATA,
                                 torque_params, sizeof(torque_params),
                                 response, &response_len, &response_utime))
        return -1;

    return 0;
}

static int generic_get_status(dynamixel_servo_t *servo, int pos_bits, dynamixel_servo_status_t *status)
{
    uint8_t params[] = { 0x24, 8 };
    uint8_t response[DYNAMIXEL_MAX_LEN];
    int response_len;
    int64_t response_utime;

    if (servo->bus->send_command(servo->bus, servo->id,
                                 DYNAMIXEL_INST_READ_DATA,
                                 params, sizeof(params),
                                 response, &response_len, &response_utime))
        return -1;

    status->utime = response_utime;

    status->position_radians = ((response[1] + (response[2] << 8)) & pos_bits) * (servo->radians_max - servo->radians_min) / pos_bits + servo->radians_min;

    int tmp = response[3] + ((response[4] & 0x3f) << 8);
    if (tmp < 1024)
        status->speed = tmp / 1023.0;
    else
        status->speed = -(tmp - 1024)/1023.0;

    // load is signed, we scale to [-1, 1]
    tmp = response[5] + (response[6] << 8);
    if (tmp < 1024)
        status->load = tmp / 1023.0;
    else
        status->load = -(tmp - 1024) / 1023.0;

    status->voltage = response[7] / 10.0;   // scale to voltage
    status->temperature = response[8];      // deg celsius
    status->error_flags = response[0];

    return 0;
}

static int ax_set_goal(dynamixel_servo_t *servo, double radians, double speed, double torque)
{
    return generic_set_goal(servo, radians, 0x3ff, speed, torque);
}

static int ax_get_status(dynamixel_servo_t *servo, dynamixel_servo_status_t *status)
{
    return generic_get_status(servo, 0x3ff, status);
}

static int mx_set_goal(dynamixel_servo_t *servo, double radians, double speed, double torque)
{
    return generic_set_goal(servo, radians, 0xfff, speed, torque);
}

static int mx_get_status(dynamixel_servo_t *servo, dynamixel_servo_status_t *status)
{
    return generic_get_status(servo, 0xfff, status);
}

dynamixel_servo_t *dynamixel_servo_create(dynamixel_bus_t *bus, int id)
{
    int model = dynamixel_bus_get_device_model(bus, id);
    if (model < 0)
        return NULL;

    switch (model) {

        case 0x000c: { // AX12+
            dynamixel_servo_t *servo = calloc(1, sizeof(dynamixel_servo_t));
            servo->bus = bus;
            servo->id = id;
            servo->radians_min = to_radians(-150);
            servo->radians_max = to_radians( 150);
            servo->set_goal = ax_set_goal;
            servo->get_status = ax_get_status;
            return servo;
        }

        case 0x00ff: // 4-pin MX28
        case 0x001d: // MX28
        case 0x0136: // MX64
        case 0x0140: { // MX106
            dynamixel_servo_t *servo = calloc(1, sizeof(dynamixel_servo_t));
            servo->bus = bus;
            servo->id = id;
            servo->radians_min = -M_PI;
            servo->radians_max =  M_PI;
            servo->set_goal = mx_set_goal;
            servo->get_status = mx_get_status;
            return servo;
        }

        default:
            return NULL;
    }

    dynamixel_servo_t *servo = calloc(1, sizeof(dynamixel_servo_t));
    servo->bus = bus;
    servo->id = id;
    return servo;
}
