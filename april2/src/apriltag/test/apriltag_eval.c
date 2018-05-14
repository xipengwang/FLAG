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

#include <stdbool.h>
#include <stdio.h>

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "common/getopt.h"
#include "common/image_convert.h"
#include "common/rand_util.h"
#include "raytracer/raytracer.h"


typedef struct {
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    int nthreads;
} state_t;

static bool do_test(state_t *state, double pos[3], double rpy[3])
{
    bool detected = false;

    const double tagsize = 0.1;
    const double f = 2.0;
    camera_t *c = camera_create(f, f,
                                (double[]){0, 0, 0},   // pos
                                (double[]){0, 0, -1},   // lookat
                                (double[]){0, 1, 0});  // up
    scene_t *s = scene_create((float[]) {0, 0, 0},           // background
                              (float[]) {0.2f, 0.2f, 0.2f}); // ambient
    s->fog_density = 0.01;

    // Checkerboard
    scene_add(s, so_chain(so_matrix_translate(0, -1, 0),
                          so_matrix_rotatex(-M_PI/2),
                          so_plane(surface_checkerboard(0.0, 100),
                                   NULL,
                                   (double[]){-DBL_MAX, -DBL_MAX},
                                   (double[]){DBL_MAX, DBL_MAX})));

    // Tag
    double M[16];
    double quat[4];
    doubles_rpy_to_quat(rpy, quat);
    doubles_quat_xyz_to_mat44(quat, pos, M);

    double scale = tagsize / 2.0;
    int tagid = randi_uniform(0, state->tf->ncodes);
    image_u8_t *tag_im = apriltag_to_image(state->tf, tagid);
    scene_object_t tag_plane =
        so_plane(surface_image_u8(tag_im, (double[]){-scale, -scale},
                    (double[]){scale, scale},
                    true, false, 0.0, 100),
                surface_constant((float[]){.2f, .8f, .2f},
                    (float[]){1, 1, 1},
                    0.3, 50),
                (double[]){-scale, -scale},
                (double[]){scale, scale});
    scene_add(s, so_chain(so_matrix(M),
                tag_plane));

    // Lights
    scene_add(s, so_chain(so_matrix_translate(-10, 10, 10),
                          so_light((float[]){.25f, .25f, .25f})));
    scene_add(s, so_chain(so_matrix_translate(10, 10, 10),
                          so_light((float[]){.25f, .25f, .25f})));

    raytracer_t *r = raytracer_create(c, s);
    const int width = 400;
    const int height = 400;
    const int antialias = 3;
    image_u8x3_t *im = raytracer_render(r, width, height, antialias,
            state->nthreads);
    image_u8_t *im_gray = image_u8x3_to_u8(im);

    // Detect tags
    zarray_t *detections = apriltag_detector_detect(state->td, im_gray);
    if (zarray_size(detections) == 1) {
        apriltag_detection_t *det;
        zarray_get(detections, 0, &det);

        detected = (det->id == tagid);
    }

    // Cleanup
    image_u8x3_destroy(im);
    image_u8_destroy(tag_im);
    image_u8_destroy(im_gray);

    camera_destroy(c);
    scene_destroy(s);
    raytracer_destroy(r);

    apriltag_detections_destroy(detections);

    return detected;
}

int main(int argc, char *argv[])
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Print this help");
    getopt_add_int(gopt, 'n', "ntrials", "100", "Number of trials per step");
    getopt_add_int(gopt, 't', "nthreads", "4", "Number of threads to use");
    if (!getopt_parse(gopt, argc, argv, 1) ||
            getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        return 1;
    }

    state_t state = {};
    state.tf = tag36h11_create();
    state.td = apriltag_detector_create();
    apriltag_detector_add_family(state.td, state.tf);
    state.td->nthreads = 8;
    state.td->refine_edges = 1;
    state.td->quad_decimate = 1.0;
    state.td->quad_sigma = 0.0;
    state.nthreads = getopt_get_int(gopt, "nthreads");

    //srand(time(NULL));

    int n = getopt_get_int(gopt, "ntrials");
    printf("Detection rate by distance:\n");
    for (double z = 2.6; z < 4.0; z += 0.1) {
        // Use the same poses and tagid for each iteration
        srand(0);

        int count = 0;

        for (int i = 0; i < n; i += 1) {
            double pos[3] = {0, 0, -z};
            double rpy[3] = {randf_uniform(-0.4, 0.4),
                randf_uniform(-0.4, 0.4),
                randf_uniform(-M_PI, M_PI)};

            if (do_test(&state, pos, rpy))
                count += 1;

            printf("z = %5.2f: %7.2f %% detected (%3d of %3d)\r",
                    z, count * 100.0 / (i+1), count, i+1);
            fflush(stdout);
        }
        printf("\n");
    }

    getopt_destroy(gopt);
    apriltag_detector_destroy(state.td);
    tag36h11_destroy(state.tf);
}
