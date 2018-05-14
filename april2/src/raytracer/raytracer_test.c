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

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"

#include "raytracer.h"


int main(int argc, char *argv[])
{
    double f = 2.0;
    camera_t *c = camera_create(f, f,
                                (double[]){0, -5, 2},  // pos
                                (double[]){0, 0, 0},   // lookat
                                (double[]){0, 0, 1});  // up
    scene_t *s = scene_create((float[]) {0, 0, 0},           // background
                              (float[]) {0.2f, 0.2f, 0.2f}); // ambient
    s->fog_density = 0.01;

    double time = 0;

    apriltag_family_t *tf = tag36h11_create();
    image_u8_t *tag_im = apriltag_to_image(tf, 20);

    scene_add(s, so_chain(so_matrix_rotatez(time/2.0),
                          so_chain(so_matrix_translate(0, 0, 1),
                                   so_sphere(surface_radial_stripes(
                                                 (float[]){1, 0, 0},
                                                 (float[]){.3f, .5f, 0},
                                                 (float[]){1, 1, 1},
                                                 0.5, 10, 10),
                                             1)),
                          so_chain(so_matrix_translate(0.9, 0, 1),
                                   so_sphere(surface_constant(
                                                 (float[]){0, 0, 1},
                                                 (float[]){1, 1, 1},
                                                 0.25, 100),
                                             0.5))));

    scene_add(s, so_chain(so_matrix_translate(1, -2, 0),
                          so_matrix_rotatez(time/5.0),
                          so_matrix_rotatex(M_PI/2),
                          so_plane(surface_image_u8(tag_im, (double[]){0, 0},
                                                    (double[]){1, 1},
                                                    true, false, 0.0, 100),
                                   surface_constant((float[]){.2f, .8f, .2f},
                                                    (float[]){1, 1, 1},
                                                    0.3, 50),
                                   (double[]){0, 0},
                                   (double[]){1, 1})));

    scene_add(s, so_plane(surface_checkerboard(0.5, 100),
                          NULL,
                          (double[]){-DBL_MAX, -DBL_MAX},
                          (double[]){DBL_MAX, DBL_MAX}));

    scene_add(s, so_chain(so_matrix_rotatez(time/10.0),
                          so_matrix_translate(-10, -10, 10),
                          so_light((float[]){.5f, .5f, .5f})));

    raytracer_t *r = raytracer_create(c, s);
    int width = 400;
    int height = 400;
    int antialias = 2;
    int nthreads = 2;
    image_u8x3_t *im = raytracer_render(r, width, height, antialias, nthreads);
    image_u8x3_write_pnm(im, "/tmp/raytrace.pnm");
    printf("Wrote test image to /tmp/raytrace.pnm\n");
    image_u8x3_destroy(im);

    image_u8_destroy(tag_im);
    camera_destroy(c);
    scene_destroy(s);
    raytracer_destroy(r);
    tag36h11_destroy(tf);
}
