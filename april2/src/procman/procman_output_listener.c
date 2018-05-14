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
#include <string.h>

#include <lcm/lcm.h>

#include "common/getopt.h"
#include "common/string_util.h"

#include "lcmtypes/procman_output_t.h"

static void
output_handler(const lcm_recv_buf_t *rbuf,
               const char* channel,
               const procman_output_t *msg,
               void *usr)
{
    int *_usr = (int*) usr;
    int procid = *_usr;

    if (procid == -1 || msg->procid == procid)
    {
        printf("%s", msg->data);
        fflush(stdout);
    }
}

int main(int argc, char **argv)
{
    ////////////////////////////////////////
    // argument handling
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h',"help", 0,"Show this");
    getopt_add_int(gopt, 'i', "procid", "-1", "Process id (-1 for no filtering)");

    // parse and print help
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help")) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(gopt);
        exit(1);
    }

    int procid = getopt_get_int(gopt, "procid");

    ////////////////////////////////////////
    // lcm
    lcm_t *lcm = lcm_create(NULL);
    if (lcm == NULL) {
        printf("Error: Could not create LCM\n");
        exit(1);
    }

    procman_output_t_subscribe(lcm, "PROCMAN_OUTPUT", &output_handler, &procid);

    while (1)
    {
        lcm_handle(lcm);
    }

    getopt_destroy(gopt);
}
