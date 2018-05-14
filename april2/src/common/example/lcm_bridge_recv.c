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
#include <lcm/lcm.h>

#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>

#include "common/udpr.h"
#include "common/getopt.h"
#include "common/net_util.h"

int main(int argc, char *argv[])
{
    getopt_t *gopt = getopt_create();

    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");
    getopt_add_string(gopt, 'c', "channel", "", "LCM channel to send on");
    getopt_add_int(gopt, 'p', "port","1234", "port to listen on");
    getopt_add_int(gopt, 'v', "verbosity", "1", "verbosity level");

    udpr_receiver_params_t params;
    udpr_receiver_params_init(&params);


    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options] \n", argv[0]);
        getopt_do_usage(gopt);
        exit(0);
    }

    params.verbosity = getopt_get_int(gopt, "verbosity");
    udpr_receiver_t *urr = udpr_receiver_create(&params, getopt_get_int(gopt, "port"));

    int port = getopt_get_int(gopt, "port");
    printf("Port %d\n", port);

    lcm_t * lcm = lcm_create(NULL);
    assert(lcm);

    while(1) {
        int nbytes = 0;
        uint8_t * bytes = NULL;

        udpr_receiver_data_t *h = udpr_receiver_get(urr);
        lcm_publish(lcm, getopt_get_string(gopt, "channel"), h->data, h->datalen);
        udpr_receiver_data_destroy(h);
    }


    return 0;
}
