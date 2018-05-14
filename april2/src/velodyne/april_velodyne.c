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

#include "april_velodyne.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "common/zarray.h"
#include "common/math_util.h"
#include "common/floats.h"

#define VELO_CONVERSION_FACTOR -2.0 * M_PI / 36000.0

float vertical_angle_degrees_32[] = {
    -30.67, -9.33,  -29.33, -8.00,
    -28.00, -6.66,  -26.66, -5.33,
    -25.33, -4.00,  -24.00, -2.67,
    -22.67, -1.33,  -21.33,  0.00,
    -20.00,  1.33,  -18.67,  2.67,
    -17.33,  4.00,  -16.00,  5.33,
    -14.67,  6.67,  -13.33,  8.00,
    -12.00,  9.33,  -10.67, 10.67
};

int vertical_angle_idx_32[] = {
    0,  16,  1, 17,
    2,  18,  3, 19,
    4,  20,  5, 21,
    6,  22,  7, 23,
    8,  24,  9, 25,
    10, 26, 11, 27,
    12, 28, 13, 29,
    14, 30, 15, 31
};

float vertical_angle_degrees_16[] = {
    -15, 1,
    -13, 3,
    -11, 5,
    -9, 7,
    -7, 9,
    -5, 11,
    -3, 13,
    -1, 15
};

int vertical_angle_idx_16[] = {
    0, 8,
    1, 9,
    2, 10,
    3, 11,
    4, 12,
    5, 13,
    6, 14,
    7, 15
};

struct april_velodyne {
    int mode;   // The number of lasers. 0 when uninitialized

    float *vertical_angle_sincos;
    float *vertical_angle_degrees;
    int *vertical_angle_idx;

    float last_theta;
    float prev_slice_theta;

    zarray_t *pts;
    zarray_t *ranges;
    zarray_t *intensities;
};

// Create a velodyne object for handling 16 and 32 laser models
april_velodyne_t *april_velodyne_create()
{
    april_velodyne_t *velo = calloc(1, sizeof(april_velodyne_t));

    return velo;
}

void april_velodyne_destroy(april_velodyne_t *velo)
{
    free(velo->vertical_angle_sincos);
    free(velo);

    if (velo->pts) {
        zarray_destroy(velo->pts);
        zarray_destroy(velo->ranges);
        zarray_destroy(velo->intensities);
    }

    return;
}

/** Initialize a velodyne based on the data buffer from a message
 *  from the device in question. Returns 0 on success, or 1 if message
 *  disagrees with already initialized velodyne object
 */
static inline int april_velodyne_init(april_velodyne_t *velo,
                                      const uint8_t *buf,
                                      int32_t buflen)
{
    const uint8_t *factory = &buf[1204];
    int mode = -1;
    switch (factory[1]) {
        case VELO_FACTORY_HDL_32E:
            mode = 32;
            break;
        case VELO_FACTORY_VLP_16:
            mode = 16;
            break;
        default:
            fprintf(stderr, "ERR: Bad velodyne mode %d\n", mode);
            break;
    }

    if (velo->mode > 0 && velo->mode != mode)
        return 1;

    if (velo->mode == mode)
        return 0;

    velo->mode = mode;
    switch (velo->mode) {
        case 16:
            velo->vertical_angle_idx = vertical_angle_idx_16;
            velo->vertical_angle_degrees = vertical_angle_degrees_16;
            break;
        case 32:
            velo->vertical_angle_idx = vertical_angle_idx_32;
            velo->vertical_angle_degrees = vertical_angle_degrees_32;
            break;
    }

    float xyz[3] = {0,0,0};
    float range = 0;
    uint8_t intensity = 0;
    velo->pts = zarray_create(3*sizeof(float));
    velo->ranges = zarray_create(sizeof(float));
    velo->intensities = zarray_create(sizeof(uint8_t));
    velo->vertical_angle_sincos = malloc(velo->mode * 2 * sizeof(float));
    for (int i = 0; i < velo->mode; ++i) {
        float rad = velo->vertical_angle_degrees[i] * M_PI/180.0f;
        velo->vertical_angle_sincos[2*i + 0] = sin(rad);
        velo->vertical_angle_sincos[2*i + 1] = cos(rad);

        zarray_add(velo->pts, xyz);
        zarray_add(velo->ranges, &range);
        zarray_add(velo->intensities, &intensity);
    }

    return 0;
}

// XXX TODO: Handle pose of sensor somewhere.
void april_velodyne_on_packet(april_velodyne_t *velo,
                              const uint8_t *buf,
                              int32_t buflen,
                              float xform[16],
                              void (*on_slice)(const zarray_t *pts, const zarray_t *ranges, const zarray_t* intensities, uint8_t mode, void *user),
                              void (*on_sweep)(void *user),
                              void *user)
{
    if (buflen != 1206) {
        fprintf(stderr, "ERR: Skipping velodyne message with bad length %d\n",
                buflen);
        return;
    }

    if (april_velodyne_init(velo, buf, buflen))
        return;

    // Factory bytes
    const uint8_t *factory = &buf[1204];

    // Positional information
    float range, horiz, theta, theta_step;
    float xyz[3], velo_xyz[3];
    float s_psi, c_psi, s_t, c_t;
    uint8_t intensity;

    for (int blocks = 0; blocks < 12; blocks++) {
        const uint8_t *data = &buf[100 * blocks];
        uint32_t block = (data[0] << 0) + (data[1] << 8);

        if (block != 0xeeff && block != 0xddff) {
            fprintf(stderr, "ERR: bad magic on block %d\n", block);
            continue;
        }

        theta = VELO_CONVERSION_FACTOR * ((data[2] << 0) + (data[3] << 8));

        // This method of interpolation is technically looking BACK one value
        // instead of forward, but should be functionally similar to the
        // suggested interpolation metho in the velodyne specs.
        if (velo->mode == 16)
            theta_step = 0.5*mod2pi(theta - velo->last_theta);

        if ((factory[0]) != VELO_FACTORY_DUAL_RETURN) {
            velo->last_theta = theta;
        } else if (blocks % 2 == 0) {
            velo->last_theta = theta;
        }


        s_t = sin(theta);
        c_t = cos(theta);

        for (int laseridx = 0; laseridx < 32; laseridx++) {
            range = ((data[4 + 3*laseridx + 0] << 0) +
                     (data[4 + 3*laseridx + 1] << 8)) * 0.002f;
            intensity = data[4 + laseridx*3 + 2];

            // Handle 16 laser mode
            if (laseridx == velo->mode) {
                if (on_slice) {
                    uint8_t mode = factory[0];
                    if(mode == VELO_FACTORY_DUAL_RETURN){
                       if(blocks % 2 == 0)
                            mode = VELO_FACTORY_LAST_RETURN;
                       else
                            mode = VELO_FACTORY_STRONGEST_RETURN;
                    }
                    on_slice(velo->pts,
                             velo->ranges,
                             velo->intensities,
                             mode,
                             user);
                }

                s_t = sin(theta + theta_step);
                c_t = cos(theta + theta_step);
            }

            s_psi = velo->vertical_angle_sincos[2*(laseridx % velo->mode) + 0];
            c_psi = velo->vertical_angle_sincos[2*(laseridx % velo->mode) + 1];
            horiz = c_psi * range;

            velo_xyz[0] = c_t * horiz;
            velo_xyz[1] = s_t * horiz;
            velo_xyz[2] = s_psi * range;
            floats_mat_transform_xyz(xform, velo_xyz, xyz);

            int slice_idx = velo->vertical_angle_idx[laseridx % velo->mode];
            zarray_set(velo->pts, slice_idx, xyz, NULL);
            zarray_set(velo->ranges, slice_idx, &range, NULL);
            zarray_set(velo->intensities, slice_idx, &intensity, NULL);

        }

        if (on_slice) {
            uint8_t mode = factory[0];
            if(mode == VELO_FACTORY_DUAL_RETURN){
               if(blocks % 2 == 0)
                    mode = VELO_FACTORY_LAST_RETURN;
               else
                    mode = VELO_FACTORY_STRONGEST_RETURN;
            }
            on_slice(velo->pts,
                     velo->ranges,
                     velo->intensities,
                     mode,
                     user);
        }

        // Batch complete. Process this batch of data before moving on.
        if (theta > velo->prev_slice_theta && on_sweep) {
            on_sweep(user);
        }
        velo->prev_slice_theta = theta;
    }
}

