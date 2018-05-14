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
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "lcmtypes/speak_t.h"

#include "common/time_util.h"
#include "common/string_util.h"

int main(int argc, char **argv)
{
    if (argc != 2 && argc != 3) {
        printf("Usage: spoof_speak [optional espeak args] [text to speak]\n");
        return 1;
    }

    speak_t *speak = malloc(sizeof(speak_t));
    speak->utime = utime_now();
    speak->args = strdup(argc == 2 ? "" : argv[1]);
    speak->priority = 1;
    speak->message = strdup(argc == 2 ? argv[1] : argv[2]);

    lcm_t *lcm = lcm_create(NULL);
    speak_t_publish(lcm, "SPEAK", speak);

    speak_t_destroy(speak);
    lcm_destroy(lcm);
}
