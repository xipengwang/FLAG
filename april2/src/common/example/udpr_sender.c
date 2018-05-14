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

#include "common/udpr.h"
#include "common/time_util.h"
#include "common/rand_util.h"

static void print_guid(const uint8_t *p)
{
    for (int i = 0; i < 32; i++)
        printf("%02x", p[i]);
}

int main(int argc, char *argv[])
{
    srand(utime_now());

    uint64_t serial = 0;

    udpr_send_params_t params;
    udpr_send_params_init(&params);

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--drop")) {
            params.drop_rate = atof(argv[i+1]);
            i++;
        } else if (!strcmp(argv[i], "--verbosity")) {
            params.verbosity = atoi(argv[i+1]);
            i++;
        } else {
            printf("Unrecognized option %s\n", argv[i]);
        }
    }

    struct sockaddr_in addr;
    int res = ip4_parse_addr("127.0.0.1", &addr, 12345, NULL);

    while (1) {
        uint8_t guid[32];
        for (int i = 0; i < 32; i++)
            guid[i] = randi_uniform(0, 256);

        guid[0] = (serial >> 24) & 0xff;
        guid[1] = (serial >> 16) & 0xff;
        guid[2] = (serial >> 8) & 0xff;
        guid[3] = (serial >> 0) & 0xff;

        int datalen = randi_uniform(0, 128*1024);
        uint8_t data[datalen];

        for (int i = 0; i < datalen; i++)
            data[i] = i ^ datalen;

        int res = udpr_send(&params,
                            &addr,
                            data, datalen, guid);

        printf("SENT: ");
        print_guid(guid);
        printf("\n");

        serial++;
    }
}
