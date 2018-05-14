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

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "dynamixel.h"
#include "serial_util.h"
#include "io_util.h"
#include "time_util.h"

#define VERBOSE 0

static void destroy(dynamixel_bus_t *bus)
{
    close(bus->u.fd.fd);
    free(bus);
}

// === Bus specific implementation ===================
// Send an instruction with the specified parameters. The error code,
// body, and checksum of the response are returned (while the initial 4
// bytes of the header are removed)

static int send_command(dynamixel_bus_t *bus, int id,
                        int instruction,
                        const void *_params, int params_len,
                        void *_response, int *_response_len, int64_t *response_utime)
{
    const uint8_t *params = _params;
    uint8_t *response = _response;

    int fd = bus->u.fd.fd;

    uint8_t cmd[DYNAMIXEL_MAX_LEN];
    int cmd_len;
    dynamixel_init_instr(id, instruction, params, params_len, cmd, &cmd_len);

    if (cmd_len != write_fully(fd, cmd, cmd_len))
        return -1;

    // Read response. The header is really 5 bytes, but we put the
    // error code in the body so that the caller knows what went wrong
    // if something bad happens. Synchronize on the first two 0xffff
    // characters.
    const int header_len = 4;
    uint8_t header[header_len];
    int header_have = 0;

    while (header_have < header_len) {
        int res = read_fully_timeout(fd,
                                    header + header_have,
                                    header_len - header_have,
                                    bus->u.fd.timeout_ms);
        if (res <= 0)
            return -1;

        assert (res <= (4 - header_have));
        assert (res + header_have == 4);

        // If the first two bytes are the sync bytes, we're done
        if (header[0] == 0xff && header[1] == 0xff)
            break;

        // Shift buffer, read one more character
        header_have = 3;
        for (int i = 0; i < 3; i++)
            header[i] = header[i+1];
    }

    if ((header[2]) != id) {
        printf("serial_bus: Received response for wrong servo %d\n",
               header[2]);
        return -2;
    }

    *response_utime = utime_now();

    int response_len = header[3];

    if (response_len < 2)
        return -3;

    int res = read_fully_timeout(fd,
                                 response, response_len,
                                 bus->u.fd.timeout_ms);
    assert(res == response_len);

    if (1) {
        uint8_t checksum = 0;
        for (int i = 2; i < header_len; i++)
            checksum += header[i];
        for (int i = 0; i < response_len-1; i++)
            checksum += response[i];
        checksum = checksum ^ 0xff;

        if (response[response_len - 1] != checksum) {
            printf("serial_bus: Bad checksum %02x %02x\n",
                   response[response_len - 1],
                   checksum);
            return -4;
        }
    }

    *_response_len = response_len;
    return 0;
}

// === Bus creation and destruction ==================
dynamixel_bus_t* dynamixel_serial_bus_create(const char *device, int baud)
{
    int fd = serial_open(device, baud, 1);
    if (fd < 0)
        return NULL;

    dynamixel_bus_t *bus = calloc(1, sizeof(dynamixel_bus_t));
    bus->u.fd.timeout_ms = 50;
    bus->u.fd.fd = fd;

    bus->send_command = send_command;
    bus->destroy = destroy;

    return bus;
}

int dynamixel_serial_bus_set_baud(dynamixel_bus_t *bus, int baud)
{
    return serial_set_baud(bus->u.fd.fd, baud);
}
