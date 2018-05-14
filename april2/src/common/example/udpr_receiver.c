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

static void print_guid(const uint8_t *p)
{
    for (int i = 0; i < 32; i++)
        printf("%02x", p[i]);
}


int main(int argc, char *argv[])
{
    udpr_receiver_params_t params;
    udpr_receiver_params_init(&params);

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

    udpr_receiver_t *urr = udpr_receiver_create(&params, 12345);

    while (1) {
        udpr_receiver_data_t *h = udpr_receiver_get(urr);
        printf("RECV: ");
        print_guid(h->guid);
        printf("\n");

        // force corruption
//        ((uint8_t*) h->data)[3] = 0;

        for (int i = 0; i < h->datalen; i++)
            assert(((uint8_t*) h->data)[i] == (uint8_t) (i ^ h->datalen));

        udpr_receiver_data_destroy(h);
    }

    return 0;

}
