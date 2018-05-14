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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <assert.h>

#include "common/getopt.h"
#include "common/image_u32.h"
#include "common/zarray.h"
#include "common/rand_util.h"
#include "common/image_u32.h"
#include "common/image_u8.h"
#include "common/time_util.h"
#include "common/matd.h"

#include "../scanmatch.h"

#include "lcmtypes/laser_t.h"

typedef struct state state_t;
struct state
{
    getopt_t *gopt;

    image_u32_t *map;
    float map_meters_per_pixel;
    float map_width_m, map_height_m;

    zarray_t *pose_datas; // indexed by seed, struct pose_data*
};

static inline double sq(double v)
{
    return v*v;
}

// returns pixel coordinates
void sample_red_pose(image_u32_t *map, double *xy)
{
    while (1) {
        xy[0] = (int) (randf_uniform(0, map->width - 1));
        xy[1] = (int) (randf_uniform(0, map->height - 1));

        uint32_t abgr = map->buf[(int) (xy[1]*map->stride + xy[0])];

        if (abgr == 0xff0000ff)
            return;
    }
}

// returns pixel coordinates
void sample_red_poses(image_u32_t *map, int ensure_covisible, double *xy0, double *xy1)
{
    if (!ensure_covisible) {
        sample_red_pose(map, xy0);
        sample_red_pose(map, xy1);
        return;
    }

    // are red poses connected by red?
  again:
    sample_red_pose(map, xy0);
    sample_red_pose(map, xy1);

    // only accept poses within 3 meters of each other
    if (sq(xy0[0]-xy1[0]) + sq(xy0[1]-xy1[1]) > 9)
        goto again;

    for (float alpha = 0; alpha <= 1; alpha += .001) {
        int xy[2];
        for (int i = 0; i < 2; i++)
            xy[i] = alpha*xy0[i] + (1.0 - alpha)*xy1[i];

        uint32_t abgr = map->buf[xy[1]*map->stride + xy[0]];
        if (abgr != 0xff0000ff)
            goto again;
    }
}

void covisible_pose_probability(image_u32_t *map)
{
    double xy0[2], xy1[2];

    // are red poses connected by red?
    int trials = 0;
    int covisible = 0;
    while (1) {
      again:

        trials++;

        sample_red_pose(map, xy0);
        sample_red_pose(map, xy1);

        // only accept poses within 3 meters of each other
        if (sq(xy0[0]-xy1[0]) + sq(xy0[1]-xy1[1]) > 9)
            goto again;

        for (float alpha = 0; alpha <= 1; alpha += .001) {
            int xy[2];
            for (int i = 0; i < 2; i++)
                xy[i] = alpha*xy0[i] + (1.0 - alpha)*xy1[i];

            uint32_t abgr = map->buf[xy[1]*map->stride + xy[0]];
            if (abgr != 0xff0000ff)
                goto again;
        }

        covisible++;
        printf("%.15f\n", 1.0*covisible / trials);
    }
}

laser_t *laser_raycast(image_u32_t *map, float meters_per_pixel, double *xyt, double rad0, double radstep, int nranges,
                       float max_range, float range_stddev)
{
    laser_t *laser = calloc(1, sizeof(laser_t));

    laser->utime = utime_now();
    laser->rad0 = rad0;
    laser->nranges = nranges;
    laser->radstep = radstep;
    laser->nintensities = 0;
    laser->ranges = calloc(laser->nranges, sizeof(float));
    laser->intensities = NULL;

    double drange = meters_per_pixel / 8;

    for (int i = 0; i < laser->nranges; i++) {
        double x = xyt[0];
        double y = xyt[1];
        double theta = xyt[2] + laser->rad0 + i*laser->radstep;
        double u = drange*cos(theta), v = drange*sin(theta);

        double range = 0;

        for (double ex = x, ey = y; range < max_range; range += drange) {
            ex += u;
            ey += v;

            int ix = (int) (ex / meters_per_pixel + .5);
            int iy = (int) (ey / meters_per_pixel + .5);

            if (ix < 0 || iy < 0 || ix >= map->width || iy >= map->height)
                continue;

            int abgr = map->buf[iy*map->stride + ix];
            // use green channel
            if ((abgr & 0x0000ff00) > 0)
                break;
        }

        laser->ranges[i] = range + randf_normal()*range_stddev;
    }

    return laser;
}

sm_points_data_t *create_points(laser_t *laser, uint32_t flags)
{
    zarray_t *points = zarray_create(sizeof(float[2]));

    for (int i = 0; i < laser->nranges; i++) {
        double theta = laser->rad0 + laser->radstep*i;
        double c = cos(theta), s = sin(theta);
        double x = laser->ranges[i]*c;
        double y = laser->ranges[i]*s;
        zarray_add(points, (float[]) { x, y });
    }

    return sm_points_data_create_flags(points, flags);
}

sm_model_data_t *create_model(laser_t *laser, double meters_per_pixel, int nlevels)
{
    double xmin = HUGE, xmax = -HUGE;
    double ymin = HUGE, ymax = -HUGE;

    for (int i = 0; i < laser->nranges; i++) {
        double theta = laser->rad0 + laser->radstep*i;
        double c = cos(theta), s = sin(theta);
        double x = laser->ranges[i]*c;
        double y = laser->ranges[i]*s;

        xmin = fmin(x, xmin);
        xmax = fmax(x, xmax);
        ymin = fmin(y, ymin);
        ymax = fmax(y, ymax);
    }

    // Render the model
    double pad = 1;
    image_u8_t *im = image_u8_create((xmax-xmin+2*pad) / meters_per_pixel,
                                     (ymax-ymin+2*pad) / meters_per_pixel);

    double x0 = xmin - pad; // position of pixel 0 (in meters);
    double y0 = ymin - pad;

// how far away (in meters) does it take for our cost model to drop to 0?
#define MODEL_COST_RANGE 0.1

    image_u8_lut_t lut;
    if (1) {
        memset(&lut, 0, sizeof(image_u8_lut_t));
        lut.scale = 1;
        float max_dist2 = MODEL_COST_RANGE * MODEL_COST_RANGE / (meters_per_pixel * meters_per_pixel);

        lut.nvalues = (int) (max_dist2 / lut.scale) + 1;
        lut.values = calloc(lut.nvalues, sizeof(uint8_t));
        for (int i = 0; i < lut.nvalues; i++) {
            int penalty = 255.0*i*i / ((lut.nvalues-1)*(lut.nvalues-1));
            if (penalty > 255)
                penalty = 255;
            lut.values[i] = 255 - penalty;
        }
    }

    float last_xy[2];

    for (int i = 0; i < laser->nranges; i++) {
        double theta = laser->rad0 + laser->radstep*i;
        double c = cos(theta), s = sin(theta);
        double x = laser->ranges[i]*c;
        double y = laser->ranges[i]*s;

        // convert to pixel coordinates
        float model_xy[] = { (x - x0) / meters_per_pixel,
                             (y - y0) / meters_per_pixel };

        image_u8_fill_line_max(im, &lut, model_xy, model_xy);

        if (i > 0) {
            double dist2 = sq(x - last_xy[0]) + sq(y - last_xy[1]);
            float xy[2] = { x, y };
            if (dist2 < 1) {
                image_u8_fill_line_max(im, &lut,
                                       (float[]) { (last_xy[0] - x0) / meters_per_pixel,
                                               (last_xy[1] - y0) / meters_per_pixel },
                                       (float[]) { (xy[0] - x0) / meters_per_pixel,
                                               (xy[1] - y0) / meters_per_pixel });
            }
        }
        last_xy[0] = x;
        last_xy[1] = y;
    }

    image_u8_write_pnm(im, "/tmp/costlut.pnm");
    if (0) {
        // leaks like a sieve!

        int64_t utime0 = utime_now();
        sm_model_data_t *a = sm_model_data_create_reference(im, x0 / meters_per_pixel, y0 / meters_per_pixel, meters_per_pixel, nlevels);
        int64_t utime1 = utime_now();
        sm_model_data_t *b = sm_model_data_create(im, x0 / meters_per_pixel, y0 / meters_per_pixel, meters_per_pixel, nlevels);
        int64_t utime2 = utime_now();

//        printf("\nMODEL %d %d %10.3f %10.3f\n", im->width, im->height, (utime1-utime0)/1.0E3, (utime2-utime1)/1.0E3);
    }

    return sm_model_data_create(im, x0 / meters_per_pixel, y0 / meters_per_pixel, meters_per_pixel, nlevels);
}

void xyt_inv_mul(const double a[3], const double b[3], double X[3])
{
    double theta = a[2];
    double ca = cos(theta), sa = sin(theta);
    double dx = b[0] - a[0];
    double dy = b[1] - a[1];

    X[0] = ca*dx + sa*dy;
    X[1] = -sa*dx + ca*dy;
    X[2] = b[2] - a[2];
}

void draw_laser(image_u32_t *im, double meters_per_pixel, laser_t *laser, double *xyt, uint32_t color, int sz)
{
    for (int i = 0; i < laser->nranges; i++) {
        float theta = laser->rad0 + laser->radstep*i;
        float c = cos(theta), s = sin(theta);
        float x = laser->ranges[i] * c, y = laser->ranges[i] * s;

        float c2 = cos(xyt[2]), s2 = sin(xyt[2]);
        float px = (x * c2 - y * s2 + xyt[0] ) / meters_per_pixel;
        float py = (x * s2 + y * c2 + xyt[1] ) / meters_per_pixel;
        image_u32_draw_circle(im, px, py, sz, color);
    }

    image_u32_draw_circle(im,
                          xyt[0] / meters_per_pixel,
                          xyt[1] / meters_per_pixel, 6*sz, color);
}

// dt
double do_one_scanmatch(sm_model_data_t *model, sm_points_data_t *points,
                        double *tnoisy, double *ttruth, double trans_range,
                        double rotrange, double rad_step)
{
    int wstrings = 0;

    for (int repeat = 0; repeat < 2; repeat++) {

        int64_t utime0 = utime_now();

        // A = BT,  T = inv(B)*A
        sm_search_t *sm_search = sm_search_create();

        float rad_step = 1.0 / 180 * M_PI;
        int npoints = zarray_size(points->points);

        sm_search_add(sm_search, points, model,
                      (tnoisy[0] - trans_range) / model->meters_per_pixel,
                      (tnoisy[0] + trans_range) / model->meters_per_pixel,
                      (tnoisy[1] - trans_range) / model->meters_per_pixel,
                      (tnoisy[1] + trans_range) / model->meters_per_pixel,
                      tnoisy[2] - rotrange, tnoisy[2] + rotrange, rad_step,
                      1.0 / npoints, NULL, NULL, -100);

        sm_result_t *sm_result = sm_search_run(sm_search);
        int64_t utime1 = utime_now();

        printf(" %s %12.8f %12.8f %12.8f %10.5f %10.5f %7.2f",
               wstrings ? "'sm'" : "999",
               sm_result->xyt[0], sm_result->xyt[1], sm_result->xyt[2],
               sm_result->score,
               sqrt(sq(sm_result->xyt[0] - ttruth[0]) + sq(sm_result->xyt[1] - ttruth[1])),
               (utime1-utime0)/1.0E3);

        sm_result_destroy(sm_result);
        sm_search_destroy(sm_search);
    }
    return 0;
}

void accuracy(state_t *state, int iteration)
{
    int wstrings = 0;

  again:

    srand(iteration);
    srandom(iteration);

    printf("%5d", iteration);

    iteration++;

    /////////////////////////////////////////////////////////
    // create ground-truth data
    double xytb[3], xyta[3];
    sample_red_poses(state->map, 1, xyta, xytb);

    for (int i = 0; i < 2; i++) {
        xyta[i] *= state->map_meters_per_pixel;
        xytb[i] *= state->map_meters_per_pixel;
    }
    xyta[2] = randf_uniform(0, 2*M_PI);
    xytb[2] = randf_uniform(0, 2*M_PI);

    double ttruth[3];
    xyt_inv_mul(xyta, xytb, ttruth);
    if (ttruth[2] < 0)
        ttruth[2] += 2 * M_PI;

    // create noisy version of xyta.
    double noise[3];
    while (1) {
        // rejection sampling. sample in a circle of radius r.
        double r = 10;

        noise[0] = randf_uniform(-r, r);
        noise[1] = randf_uniform(-r, r);
        if (sq(noise[0]) + sq(noise[1]) > sq(r))
            continue;
        noise[2] = randf_uniform(-M_PI, M_PI);
        break;
    }

    double nxyta[3];
    for (int i = 0; i < 3; i++) {
        nxyta[i] = xyta[i] + noise[i];
    }

    printf(" %s %12.8f %12.8f %12.8f",
           wstrings ? "'noise'" : "999",
           noise[0], noise[1], noise[2]);

    printf(" %s %12.8f %12.8f %12.8f",
           wstrings ? "'truth'" : "999",
           ttruth[0], ttruth[1], ttruth[2]);

    double tnoisy[3];
    xyt_inv_mul(nxyta, xytb, tnoisy);
    if (tnoisy[2] < 0)
        tnoisy[2] += 2 * M_PI;

    int npoints = 4*360;
    laser_t *lasera = laser_raycast(state->map, state->map_meters_per_pixel, xyta, 0, 2*M_PI / npoints, npoints, 30, 0.01);
    laser_t *laserb = laser_raycast(state->map, state->map_meters_per_pixel, xytb, 0, 2*M_PI / npoints, npoints, 30, 0.01);

    if (0) {
        image_u32_t *im = image_u32_copy(state->map);
        image_u32_scale_gray(im, 0.25);
        draw_laser(im, state->map_meters_per_pixel, lasera, xyta, 0x0000ff00, 6); // ABGR
        draw_laser(im, state->map_meters_per_pixel, laserb, xytb, 0x000000ff, 3); // ABGR
        image_u32_write_pnm(im, "/tmp/output.pnm");
        image_u32_destroy(im);
    }

    if (1) {
        // what resolution will we render our scan matching lookup table at?
        // 2009 paper: 1.0 / 32
        double model_meters_per_pixel = 1.0 / 32;

        sm_model_data_t *model = create_model(lasera, model_meters_per_pixel, 8);
        sm_points_data_t *points = create_points(laserb, 0);

        double trans_range = 15; // meters
        double rotrange = 360 * M_PI / 180; // radians

        for (int repeat = 0; repeat < 2; repeat++) {
            int dryrun = (repeat == 0);

            int64_t utime0 = utime_now();

            // A = BT,  T = inv(B)*A
            sm_search_t *sm_search = sm_search_create();
            float rad_step = 1.0 / 180 * M_PI;

            sm_search_add(sm_search, points, model,
                          (tnoisy[0] - trans_range) / model->meters_per_pixel,
                          (tnoisy[0] + trans_range) / model->meters_per_pixel,
                          (tnoisy[1] - trans_range) / model->meters_per_pixel,
                          (tnoisy[1] + trans_range) / model->meters_per_pixel,
                          tnoisy[2] - rotrange, tnoisy[2] + rotrange, rad_step,
                          1.0 / npoints, NULL, NULL, -100);

            sm_result_t *sm_result = sm_search_run(sm_search);
            int64_t utime1 = utime_now();

            if (!dryrun)
                printf(" %s %12.8f %12.8f %12.8f %10.5f %10.5f %7.2f",
                       wstrings ? "'sm'" : "999",
                       sm_result->xyt[0], sm_result->xyt[1], sm_result->xyt[2],
                       sm_result->score,
                       sqrt(sq(sm_result->xyt[0] - ttruth[0]) + sq(sm_result->xyt[1] - ttruth[1])),
                       (utime1-utime0)/1.0E3);

            sm_result_destroy(sm_result);

            utime0 = utime_now();
            sm_hillclimb_params_t hcparams = { .maxiters = 1000,
                                               .initial_step_sizes = { model_meters_per_pixel / 2,
                                                   model_meters_per_pixel / 2, 0.5 * M_PI / 180 },
                                               .step_size_shrink_factor = 0.5, // was .5
                                               .max_step_size_shrinks = 8 }; // was 8
            sm_hillclimb_result_t *smhc_result = sm_hillclimb(points, model, sm_result->xyt, &hcparams,
                                                              1, NULL, NULL);

            utime1 = utime_now();

            if (!dryrun) {
                printf(" %s %12.8f %12.8f %12.8f %10.5f %4d %10.5f %7.2f",
                       wstrings ? "'smhc'" : "999",
                       smhc_result->xyt[0], smhc_result->xyt[1], smhc_result->xyt[2],
                       smhc_result->score / npoints, smhc_result->iters,
                       sqrt(sq(smhc_result->xyt[0] - ttruth[0]) + sq(smhc_result->xyt[1] - ttruth[1])),
                       (utime1-utime0)/1.0E3);
            }

            sm_search_destroy(sm_search);
        }

        sm_points_data_destroy(points);
        sm_model_data_destroy(model);
    }

    if (1) {
//        sm_hillclimb_params_t hcparams = { .maxiters = 50 };
//        sm_hillclimb_result_t hc_result = sm_hillclimb(points, model, tnoisy, &hcparams);
    }

    ////////////////////////////////////////////////////////////
    // Do ICP
    if (1) {
        zarray_t *pointsb = zarray_create(sizeof(float[2]));
        zarray_t *pointsa = zarray_create(sizeof(float[2]));

        for (int i = 0; i < laserb->nranges; i++) {
            float theta = laserb->rad0 + laserb->radstep*i;
            float c = cos(theta), s = sin(theta);
            float xy[2] = { laserb->ranges[i] * c,
                            laserb->ranges[i] * s };
            zarray_add(pointsb, xy);
        }

        for (int i = 0; i < lasera->nranges; i++) {
            float theta = lasera->rad0 + lasera->radstep*i;
            float c = cos(theta), s = sin(theta);
            float xy[2] = { lasera->ranges[i] * c,
                            lasera->ranges[i] * s };
            zarray_add(pointsa, xy);
        }

        int64_t utime0 = utime_now();

        sm_icp_params_t icp_params = { .trans_thresh_m = 0.5,
                                       .rad_thresh = 0.01*M_PI/180.0,
                                       .maxiters = 100,
                                       .max_dist_ratio = 2.0 };
        sm_icp_result_t icp_result = sm_icp(pointsa, pointsb, tnoisy, &icp_params);

        int64_t utime1 = utime_now();

        printf(" %s %12.8f %12.8f %12.8f %10.5f %4d %10.5f %7.2f",
               wstrings ? "'icp'" : "999",
               icp_result.xyt[0], icp_result.xyt[1], icp_result.xyt[2],
               icp_result.weight, icp_result.iters,
               sqrt(sq(icp_result.xyt[0] - ttruth[0]) + sq(icp_result.xyt[1] - ttruth[1])),
               (utime1-utime0) / 1.0E3);

        zarray_destroy(pointsb);
        zarray_destroy(pointsa);
    }

    printf("\n");
    goto again;
}


void point_decimation_speedup(state_t *state)
{
    srand(time(NULL));
    srandom(time(NULL));
int wstrings = 0;

    int npoints = 20;
    int nrepeats = 10;
    int repeats = 0;

  again: ;

    if (repeats == nrepeats) {
        npoints *= 1.25;
        repeats = 0;
    }
    repeats++;

    /////////////////////////////////////////////////////////
    // create ground-truth data
    double xytb[3], xyta[3];
    sample_red_poses(state->map, 1, xyta, xytb);

    for (int i = 0; i < 2; i++) {
        xyta[i] *= state->map_meters_per_pixel;
        xytb[i] *= state->map_meters_per_pixel;
    }
    xyta[2] = randf_uniform(0, 2*M_PI);
    xytb[2] = randf_uniform(0, 2*M_PI);

    double ttruth[3];
    xyt_inv_mul(xyta, xytb, ttruth);
    if (ttruth[2] < 0)
        ttruth[2] += 2 * M_PI;

    // create noisy version of xyta.
    double noise[3];
    while (1) {
        // rejection sampling. sample in a circle of radius r.
        double r = 10;

        noise[0] = randf_uniform(-r, r);
        noise[1] = randf_uniform(-r, r);
        if (sq(noise[0]) + sq(noise[1]) > sq(r))
            continue;
        noise[2] = randf_uniform(-M_PI, M_PI);
        break;
    }

    double nxyta[3];
    for (int i = 0; i < 3; i++) {
        nxyta[i] = xyta[i] + noise[i];
    }

    printf(" %s %12.8f %12.8f %12.8f",
           wstrings ? "'noise'" : "999",
           noise[0], noise[1], noise[2]);

    printf(" %s %12.8f %12.8f %12.8f",
           wstrings ? "'truth'" : "999",
           ttruth[0], ttruth[1], ttruth[2]);

    double tnoisy[3];
    xyt_inv_mul(nxyta, xytb, tnoisy);
    if (tnoisy[2] < 0)
        tnoisy[2] += 2 * M_PI;

    printf(" %8d", npoints);
    laser_t *lasera = laser_raycast(state->map, state->map_meters_per_pixel, xyta, 0, 2*M_PI / npoints, npoints, 30, 0.01);
    laser_t *laserb = laser_raycast(state->map, state->map_meters_per_pixel, xytb, 0, 2*M_PI / npoints, npoints, 30, 0.01);

    if (1) {
        // what resolution will we render our scan matching lookup table at?
        // 2009 paper: 1.0 / 32
        double model_meters_per_pixel = 1.0 / 32;

        sm_model_data_t *model = create_model(lasera, model_meters_per_pixel, 8);
        sm_points_data_t *points_dec = create_points(laserb, 0);
        sm_points_data_t *points_nodec = create_points(laserb, SM_POINTS_NODECIMATE);

        double trans_range = 15; // meters
        double rotrange = 360 * M_PI / 180; // radians
        double rad_step = 1.0 * M_PI / 180;

        do_one_scanmatch(model, points_dec, tnoisy, ttruth,
                         trans_range, rotrange, rad_step);

        do_one_scanmatch(model, points_nodec, tnoisy, ttruth,
                         trans_range, rotrange, rad_step);

        sm_points_data_destroy(points_dec);
        sm_points_data_destroy(points_nodec);
        sm_model_data_destroy(model);
    }
    printf("\n");
    goto again;
}

int main(int argc, char *argv[])
{
    setlinebuf(stdout);

    //////////////////////////////
    state_t *state = calloc(1, sizeof(state_t));
    state->gopt = getopt_create();
    getopt_add_bool(state->gopt, 'h', "help", 0, "Show help");
    getopt_add_string(state->gopt, '\0', "url", "", "Camera URL");
    getopt_add_bool(state->gopt, '\0', "manyone", 0, "Many-One, no GUI");
    getopt_add_bool(state->gopt, '\0', "npoints", 0, "# lasers vs time experiment, no GUI");

    if (!getopt_parse(state->gopt, argc, argv, 1) || getopt_get_bool(state->gopt, "help"))
    {
        printf("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage(state->gopt);
        exit(1);
    }

    // load the map
    //state->map = image_u32_create_from_pnm("intelmapbig.pnm");
    if (state->map == NULL) {
        printf("couldn't open intelmapbig.pnm\n");
        return -1;
    }

    state->map_meters_per_pixel = 0.015;
    state->map_width_m = state->map->width * state->map_meters_per_pixel;
    state->map_height_m = state->map->height * state->map_meters_per_pixel;

//    covisible_pose_probability(state->map);

    accuracy(state, argc > 1 ? atoi(argv[1]) : 0);
//    point_decimation_speedup(state);
}
