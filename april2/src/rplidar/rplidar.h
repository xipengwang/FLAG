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

#ifndef _RPLIDAR_H
#define _RPLIDAR_H

#include <unistd.h>
#include <stdint.h>

#include "lcmtypes/unfixed_laser_t.h"

#define TIMEOUT_MS 50

// Request packet format. Checksum is XOR of all bytes.
// | START FLAG    | COMMAND | PAYLOAD LEN | PAYLOAD     | CHECKSUM |
// ---------------------------------------------------------------
// | 1 byte (0xA5) | 1 byte  | 1 byte      | < 256 bytes | 1 byte   |

// Response descriptor format
// | START FLAG 0  | START FLAG 1  | DATA RESP. LEN | SEND MODE | DATA TYPE |
// --------------------------------------------------------------------------
// | 1 byte (0xA5) | 1 byte (0x5A) | 30 bits        | 2 bits    | 1 byte    |

// Response format varies w/ request type

#define MAGIC_0 0xA5
#define MAGIC_1 0x5A

// 0x2 and 0x3 are reserved for future use
#define SEND_MODE_SINGLE_RESPONSE 0x0
#define SEND_MODE_MULTI_RESPONSE  0x1

#define REQUEST_STOP        0x25
#define REQUEST_RESET       0x40
#define REQUEST_SCAN        0x20
#define REQUEST_FORCE_SCAN  0x21
#define REQUEST_GET_INFO    0x50
#define REQUEST_GET_HEALTH  0x52

#define HEALTH_GOOD         0x0
#define HEALTH_WARN         0x1
#define HEALTH_ERROR        0x2

#define DTR_LEVEL_HIGH      1
#define DTR_LEVEL_LOW       0

typedef struct rp_descriptor
{
    int32_t len;
    uint8_t send_mode;
    uint8_t data_type;
} rp_descriptor_t;


/* Sets DTR to high so device can spin.*/
void rp_lidar_start_spin(int dev);

/* Sets DTR to low so device stops spinning.*/
void rp_lidar_stop_spin(int dev);

/* Exit current device state */
void rp_lidar_stop(int dev);

/* Reboot device */
void rp_lidar_reset(int dev);

/* Scan when ready */
void rp_lidar_scan(int dev, lcm_t *lcm, const char *channel);
// void rp_lidar_scan(int dev, lcm_t *lcm, char *channel);

/* Force a scan regardless of rotation speed */
void rp_lidar_force_scan(int dev, lcm_t *lcm, char *channel);

/* Check the information for the device (serial #, etc) and print it to term */
void rp_lidar_check_info(int dev);

/* Check the health of the device. Prints health status and returns status code. */
int rp_lidar_check_health(int dev);


#endif
