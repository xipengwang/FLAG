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

#include "common/interpolator.h"

#include <math.h>

#include "lcmtypes/pose_t.h"
#include "common/doubles.h"

int main(int argc, char *argv[])
{
    interpolator_t *interp = interpolator_create(sizeof(pose_t), offsetof(pose_t, utime), 2.0, 50000);
    interpolator_add_field(interp, INTERPOLATOR_DOUBLE_LINEAR, 3, offsetof(pose_t, pos));
    interpolator_add_field(interp, INTERPOLATOR_DOUBLE_QUAT, 4, offsetof(pose_t, orientation));

    pose_t posea = { .utime = 1000,
                     .pos = { 0, -10, 0 },
                     .orientation = { 1, 0, 0, 0 }
    };
    doubles_angleaxis_to_quat((double[]) { 0, 1, 0, 0}, posea.orientation);
    interpolator_add(interp, &posea);

    pose_t poseb = { .utime = 5000,
                     .pos = { 10, 0, 100 },
                     .orientation = { 1, 0, 0, 0 }
    };
    doubles_angleaxis_to_quat((double[]) { M_PI / 2.0, 1, 0, 0}, poseb.orientation);
    interpolator_add(interp, &poseb);

    for (uint64_t utime = 0; utime < 6000; utime += 500) {
        pose_t p;

        int res = interpolator_get(interp, utime, &p);
        printf("res; %d\n", res);

        printf("%d %15f %15f %15f, %15f %15f %15f %15f\n",
               (int) p.utime,
               p.pos[0], p.pos[1], p.pos[2],
               p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3]);
    }
}
