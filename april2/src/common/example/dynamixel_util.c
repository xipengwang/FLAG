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

#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>

#include "common/dynamixel.h"
#include "common/dynamixel_serial_bus.h"
#include "common/dynamixel_servo.h"
#include "common/getopt.h"

int main(int argc, char *argv[])
{
    getopt_t *gopt = getopt_create();

    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(gopt, 's', "scan", 0, "Perform a scan for devices (--baud is ignored)");
    getopt_add_string(gopt, 'd', "device", "/dev/ttyUSB0", "Serial device to connect");
    getopt_add_int(gopt, 'b', "baud", "57600", "Baud rate");
    getopt_add_int(gopt, 'm', "maxid", "253", "Maximum ID to scan");
    getopt_add_spacer(gopt, "");
    getopt_add_int(gopt, 'i', "id", "-1", "ID to use for device-specific commands below");
    getopt_add_double(gopt, 'p', "position", "-100", "Set device position in radians (-100= ignore)");
    getopt_add_double(gopt, '\0', "torque", ".5", "Torque when using --position");
    getopt_add_double(gopt, '\0', "speed", ".5", "Speed when using --position");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options] <input files>\n", argv[0]);
        getopt_do_usage(gopt);
        exit(0);
    }

    if (getopt_get_bool(gopt, "scan")) {
        const char *device = getopt_get_string(gopt, "device");

        int bauds[] = { 9600, 19200, 38400, 57600, 115200, 200000,
                        230400, 250000, 400000, 460800, 500000,
                        1000000, 2250000, 2500000, 3000000, -1 };

        dynamixel_bus_t *bus = dynamixel_serial_bus_create(device,  bauds[0]);

        for (int bidx = 0; bauds[bidx] > 0; bidx++) {
            int baud = bauds[bidx];

            if (1) {
                printf("setting baud %d\n", baud);
            }

            dynamixel_serial_bus_set_baud(bus, baud);

            if (!bus) {
                printf("failed to connect to device: %s\n", device);
                exit(1);
            }

            printf("\n\nlooking for dynamixel devices at baud %d...\n", baud);

            for (int id = 0; id <= getopt_get_int(gopt, "maxid"); id++) {
                int model = dynamixel_bus_get_device_model(bus, id);
                if (model >= 0) {
                    printf("%3d : %04x\n", id, model);
                } else {
                    printf("%3d\r", id);
                    fflush(NULL);
                }
            }
            printf("      \n");
        }

        usleep(100e3);

        bus->destroy(bus);
        exit(0);
    }

    if (getopt_get_double(gopt, "position") != -100) {
        const char *device = getopt_get_string(gopt, "device");
        int baud = getopt_get_int(gopt, "baud");

        dynamixel_bus_t *bus = dynamixel_serial_bus_create(device, baud);
        if (!bus) {
            printf("failed to create bus\n");
            exit(1);
        }

        dynamixel_servo_t *servo = dynamixel_servo_create(bus, getopt_get_int(gopt, "id"));
        if (!servo) {
            printf("failed to create servo\n");
            exit(1);
        }

        if (servo->set_goal(servo,
                            getopt_get_double(gopt, "position"),
                            getopt_get_double(gopt, "speed"),
                            getopt_get_double(gopt, "torque"))) {
            printf("set position failed\n");
            exit(1);
        }

        dynamixel_servo_status_t status;
        if (servo->get_status(servo, &status)) {
            printf("get status failed\n");
            exit(1);
        } else {
            printf("utime:   %18"PRId64"\n", status.utime);
            printf("pos:     %18f\n", status.position_radians);
            printf("speed:   %18f\n", status.speed);
            printf("load:    %18f\n", status.load);
            printf("voltage: %18f V\n", status.voltage);
            printf("temp:    %18f deg. Celsius\n", status.temperature);
            printf("err:     %18.0f\n", (float) status.error_flags);
        }
    }


    return 0;
}
