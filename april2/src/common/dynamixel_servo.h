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

#ifndef _DYNAMIXEL_SERVO
#define _DYNAMIXEL_SERVO

#include <stdint.h>

// a compatibility layer that tries to make various types of dynamixel

typedef struct dynamixel_servo_status dynamixel_servo_status_t;
struct dynamixel_servo_status
{
    int64_t utime;

    double position_radians;
    double speed;
    double load;
    double voltage;
    double temperature;

    int32_t error_flags;
};


typedef struct dynamixel_servo dynamixel_servo_t;
struct dynamixel_servo
{
    dynamixel_bus_t *bus;
    int id;

    double radians_min;
    double radians_max;

    // returns non-zero on error
    int (*set_goal)(dynamixel_servo_t *servo, double radians, double speed, double torque);

    // returns non-zero on error
    int (*get_status)(dynamixel_servo_t *servo, dynamixel_servo_status_t *status);
};

dynamixel_servo_t *dynamixel_servo_create(dynamixel_bus_t *bus, int id);

#endif
