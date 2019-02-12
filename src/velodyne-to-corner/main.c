/* Copyright (C) 2013-2019, The Regents of The University of Michigan.
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

/*$LICENSE*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

#include <lcm/lcm.h>

#include "lcmtypes/laser_t.h"
#include "lcmtypes/raw_t.h"
#include "lcmtypes/gps_t.h"
#include "lcmtypes/pose_t.h"

#include "common/config.h"
#include "common/color.h"
#include "common/doubles.h"
#include "common/floats.h"
#include "common/http_advertiser.h"
#include "common/image_u8.h"
#include "common/getopt.h"
#include "common/gridmap.h"
#include "common/gridmap_util.h"
#include "common/interpolator.h"
#include "common/math_util.h"
#include "common/time_util.h"
#include "common/zarray.h"
#include "common/zqueue.h"

#include "velodyne/april_velodyne.h"

#include "vx/vx.h"
#include "vx/webvx.h"

#include "contour.h"
#include "linefitter.h"
#include "lcmtypes/line_features_t.h"

#define SLAM_SLOPE to_radians(80)
#define MIN_RANGE 0.4
#define MAX_RANGE 50.0
#define MIN_RELATIVE_HEIGHT -0.5
#define MAX_RELATIVE_HEIGHT 10
#define MIN_RELATIVE_RANGE 0.1
#define NEAR_REGION_DIST 2
#define FEATURE_REGION_DIST 8



typedef struct point_accumulator {
    int64_t acc;
    float z_min;
    float z_max;
    float xy[2];
} point_accumulator_t;

typedef struct state {
    lcm_t* lcm;
    config_t *config;

    raw_t *msg;

    // viewer fields
    bool debug;
    vx_world_t* vw;
    webvx_t* webvx;
    zarray_t *pts;
    zarray_t *intensities;

    // Velodyne management
    april_velodyne_t *velo;
    zqueue_t* velodyne_msgs;
    pthread_mutex_t velodyne_msgs_mutex;

    char *map_channel;
    zarray_t *laser_points;

    contourExtractor_t *contourExtractor;
    lineFitter_t *lineFitter;
    double slope;
    double intercept;

} state_t;

void on_key_press(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
    state_t *state = user;

    //API KEY: press s to save landmarks into a file;
    if (ev->u.key.key_code == 'a') {
        state->slope += 0.1;
        printf("slope: %f, intercept: %f \n", state->slope, state->intercept);
    } else if (ev->u.key.key_code == 'd') {
        state->slope -= 0.1;
        printf("slope: %f, intercept: %f \n", state->slope, state->intercept);
    } else if (ev->u.key.key_code == 'w') {
        state->intercept += 0.1;
        printf("slope: %f, intercept: %f \n", state->slope, state->intercept);
    } else if (ev->u.key.key_code == 's') {
        state->intercept -= 0.1;
        printf("slope: %f, intercept: %f \n", state->slope, state->intercept);
    }
}

static int on_event(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
    state_t *state = impl;

    switch (ev->type) {
        case VX_EVENT_MOUSE_DOWN:
            break;
        case VX_EVENT_KEY_PRESSED:
            on_key_press(vl, ev, state);
            break;
    default:
            break;
    }
    return 0;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;
    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_set_world(vl, state->vw);

    double eye[3] = {0, -8, 20.0};
    double lookat[3] = {0, 0, 0};
    double up[3] = {0.0, 1.0, 0.0};
    vx_layer_set_elu(vl, eye, lookat, up, 0);
    vx_layer_add_event_handler(vl, on_event, 1, state);
}


void on_destroy_canvas(vx_canvas_t* vc, void* impl)
{
   printf("[INFO] on destroy canvas\n");
}

void signal_handler(int sig)
{
    exit(sig);
}

static void velodyne_callback(const lcm_recv_buf_t* rbuf,
                              const char* channel,
                              const raw_t* msg,
                              void* user)
{
    state_t* state = (state_t*)user;
    raw_t* msg_copy = raw_t_copy(msg);;

    pthread_mutex_lock(&state->velodyne_msgs_mutex);
    zqueue_push(state->velodyne_msgs, &msg_copy);
    while (zqueue_size(state->velodyne_msgs) > 90) {
        raw_t *vmsg;
        zqueue_pop(state->velodyne_msgs, &vmsg);
        raw_t_destroy(vmsg);
    }
    pthread_mutex_unlock(&state->velodyne_msgs_mutex);
}


uint64_t reverse(uint64_t orig)
{
    uint64_t ret = 0;
    ret |= orig & 1;
    for(int i = 0; i < 63; i++)
    {
        ret <<= 1;
        orig >>= 1;
        ret |= orig & 1;
    }
    return ret;
}

static void render_data(state_t *state)
{
    if (zarray_size(state->pts) <= 0)
        return;
    zarray_t *pts = state->pts;
    zarray_t *intensities = state->intensities;

    if (state->debug) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "3D point cloud");
        vx_resource_t *vr = vx_resource_make_attr_f32_copy((float*)pts->data,
                                                           3*zarray_size(pts),
                                                           3);
        vx_resource_t *vi = vx_resource_make_attr_f32_copy((float*)intensities->data,
                                                           zarray_size(intensities),
                                                           1);
        vx_buffer_add_back(vb,
                           vxo_points_pretty(vr, vi),
                           NULL);
        vx_buffer_swap(vb);
    }

    if (state->debug) {
        //vx_buffer_t *vb = vx_world_get_buffer(state->vw, "accumulated_points");
        zarray_t *pts = zarray_create(sizeof(float[3]));
        zarray_t *intensities = zarray_create(sizeof(float));
        if(1) {
            for(int i = 0; i < zarray_size(state->laser_points); i++) {
                point_accumulator_t point;
                zarray_get(state->laser_points, i, &point);
                if(sqrt(sq(point.xy[0]) + sq(point.xy[1])) > FEATURE_REGION_DIST) {
                    continue;
                }
                else if(sqrt(sq(point.xy[0]) + sq(point.xy[1])) > NEAR_REGION_DIST) {
                    if(point.z_max - point.z_min < 1)
                        continue;
                } else {
                    //if(point.z_max - point.z_min < 0.1)
                    //continue;
                    if(point.z_max < 0.6)
                        continue;
                }
                float xyz[3] = {point.xy[0], point.xy[1], 0};
                zarray_add(pts, xyz);
                //float intensity = point.z_max;
                float intensity = (point.z_max - point.z_min);
                zarray_add(intensities, &intensity);
            }
        } else {
            for(int i = 0; i < 20; i++) {
                float x = 0.1 * i;
                float y = state->slope * x + state->intercept;
                float z = 0;
                float xyz[3] = {x, y, z};
                zarray_add(pts, xyz);
            }
        }

        /* vx_resource_t *vr = vx_resource_make_attr_f32_copy((float*)pts->data, */
        /*                                                    3*zarray_size(pts), */
        /*                                                    3); */
        /* vx_resource_t *vi = vx_resource_make_attr_f32_copy((float*)intensities->data, */
        /*                                                    zarray_size(intensities), */
        /*                                                    1); */
        /* vx_buffer_add_back(vb, */
        /*                    vxo_points_pretty(vr, vi, 3), */
        /*                    NULL); */
        /* vx_buffer_swap(vb); */


        if(zarray_size(pts)){
            //render contours
            state->contourExtractor->extract(state->contourExtractor, pts);
            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "contours");
            int n = zarray_size(state->contourExtractor->contours);
            for(int i = 0; i< zarray_size(state->contourExtractor->contours); i++) {
                zarray_t *contour;
                zarray_get(state->contourExtractor->contours, i, &contour);
                float hue = (360.0)*((float)(reverse(i+2))/(UINT64_MAX));
                float saturation = 1.0;
                float value = (i==n) ? 0.75 : 0.45;
                float rgb[3];
                HSVtoRGB(&rgb[0], &rgb[1], &rgb[2], hue, saturation, value);
                float rgba[4] = { rgb[0], rgb[1], rgb[2], 1};
                vx_resource_t *vr = vx_resource_make_attr_f32_copy((float*)contour->data,
                                                                   3*zarray_size(contour),
                                                                   3);
                vx_buffer_add_back(vb,
                                   vxo_points(vr, rgba, 4),
                                   NULL);
            }
            vx_buffer_swap(vb);
            if(1) {
                //render lines
                state->lineFitter->extract(state->lineFitter, state->contourExtractor->contours);
                //printf("Num of lines: %d\n", zarray_size(state->lineFitter->lineFeatures));
                vx_buffer_t *vb = vx_world_get_buffer(state->vw, "lines");
                lineFeature_t line;
                lineFeature_t lineNext;
                float *color;
                int skip = 3;
                zarray_t *lines = zarray_create(sizeof(float[3]));
                for(int i = 0; i< zarray_size(state->lineFitter->lineFeatures); i++) {
                    color = vx_white;
                    zarray_get(state->lineFitter->lineFeatures, i, &line);
                    //TODO: Should not use min, it should mod.
                    for(int j = i+1; j < i+1+skip; j++) {
                        int idx_j = j % zarray_size(state->lineFitter->lineFeatures);
                        zarray_get(state->lineFitter->lineFeatures, idx_j, &lineNext);
                        //check normal
                        if(fabs(mod2pi(lineNext.normal - line.normal)) < M_PI /2 + to_radians(5) &&
                           fabs(mod2pi(lineNext.normal - line.normal)) > M_PI /2 - to_radians(5)) {
                            //check distances
                            if(floats_distance(line.p2, lineNext.p1, 2) < 0.5) {
                                //TODO: We should check with same i, what is the best corner.
                                //printf("i - j : %d - %d, %f \n", i, idx_j, to_degrees(mod2pi(lineNext.normal - line.normal)));
                                color = vx_blue;
                                float corner[2];
                                if(intersectionWith(&line.line2D, &lineNext.line2D, corner)) {
                                    vx_buffer_add_back(vb,
                                                       vxo_matrix_translate(corner[0], corner[1], 0.02),
                                                       vxo_matrix_scale(0.3),
                                                       vxo_square_solid(vx_green),
                                                       NULL);

                                    float corner_theta[3] = {corner[0], corner[1], atan2(corner[1], corner[0])};
                                    zarray_add(lines, corner_theta);
                                }
                            }
                        }
                    }
                    float line_points[6] = {line.p1[0], line.p1[1], 0, line.p2[0], line.p2[1], 0};
                    vx_resource_t *vr = vx_resource_make_attr_f32_copy(line_points,
                                                                       6,
                                                                       3);
                    vx_buffer_add_back(vb,
                                       vxo_line_strip(vr, color, 7),
                                       vxo_chain(
                                           vxo_matrix_translate(line.p1[0], line.p1[1], 0.01),
                                           vxo_text(VXO_TEXT_ANCHOR_CENTER,
                                                    "<<#000000, sansserif-0.25>>%d", i),
                                           NULL),
                                       NULL);
                }

                vx_buffer_swap(vb);
                state->lineFitter->clear(state->lineFitter);

                line_features_t line_features = {0};
                line_features.utime = state->msg->utime;
                line_features.lines_data_length = zarray_size(lines) * 3;
                line_features.lines = (float*) lines->data;
                //landmark select
                line_features_t_publish(state->lcm, "LOCAL_LINE_FEATURES", &line_features);
                printf("--------------------publish corner features:%d\n", zarray_size(lines));
                zarray_destroy(lines);
            }
            state->contourExtractor->clear(state->contourExtractor);
        }
        zarray_destroy(pts);
        zarray_destroy(intensities);
    }
    zarray_clear(pts);
    zarray_clear(intensities);
}


static void on_slice(const zarray_t *pts,
                     const zarray_t *ranges,
                     const zarray_t *intensities,
                     uint8_t mode,
                     void *user)
{
    state_t *state = user;

    if (state->debug) {
        zarray_add_all(state->pts, pts);

        for (int i = 0; i < zarray_size(intensities); i++) {
            uint8_t intensity = 0;
            zarray_get(intensities, i, &intensity);
            float fintensity = intensity/255.0f;
            zarray_add(state->intensities, &fintensity);
        }
    }

    float d[3];
    point_accumulator_t point;
    point.acc = 0;
    point.z_min = 10000;
    point.z_max = -10000;
    for (int laseridx = 0; laseridx < zarray_size(pts)-1; laseridx++) {
        float *xyz = (float*)pts->data + 3*laseridx;
        float *next_xyz = (float*)pts->data + 3*(laseridx+1);

        float range = ((float*)ranges->data)[laseridx];
        float next_range = ((float*)ranges->data)[laseridx+1];

        // Skip null range points
        if (range <= MIN_RANGE || next_range > MAX_RANGE)
            continue;

        // Skip points outside of useful height range
        if (next_xyz[2] < MIN_RELATIVE_HEIGHT || next_xyz[2] > MAX_RELATIVE_HEIGHT)
            continue;

        for (int i = 0; i < 3; i++)
            d[i] = (next_xyz[i] - xyz[i]);

        float xy2 = d[0]*d[0] + d[1]*d[1];
        float slope_rad = acos(sqrt(xy2)/sqrt(xy2+d[2]*d[2]));
        // Detects objects exceeding max slope, in addition to some
        // heuristic methods for detecting dropoffs.

        if (slope_rad > SLAM_SLOPE || slope_rad < M_PI - SLAM_SLOPE) {
            // If the distance is very small, then we can consider it may hit on a same objects.
            if(xy2 < MIN_RELATIVE_RANGE) {
                if(point.acc == 0) {
                    point.xy[0] = (xyz[0] + next_xyz[0]) / 2;
                    point.xy[1] = (xyz[1] + next_xyz[1]) / 2;
                    point.acc = 2;
                    if(xyz[2] > point.z_max) {
                        point.z_max = xyz[2];
                    }
                    if(xyz[2] < point.z_min) {
                        point.z_min = xyz[2];
                    }
                    if(next_xyz[2] > point.z_max) {
                        point.z_max = next_xyz[2];
                    }
                    if(next_xyz[2] < point.z_min) {
                        point.z_min = next_xyz[2];
                    }
                } else {
                    point.xy[0] *= point.acc;
                    point.xy[1] *= point.acc;
                    point.acc++;
                    point.xy[0] = (point.xy[0] + next_xyz[0]) / point.acc;
                    point.xy[1] = (point.xy[1] + next_xyz[1]) / point.acc;
                    if(next_xyz[2] > point.z_max) {
                        point.z_max = next_xyz[2];
                    }
                    if(next_xyz[2] < point.z_min) {
                        point.z_min = next_xyz[2];
                    }
                }
            } else {
                //Different objects
                zarray_add(state->laser_points, &point);
                point.acc = 0;
                point.z_min = 10000;
                point.z_max = -10000;
            }
        }
    }
    if(point.acc) {
        zarray_add(state->laser_points, &point);
    }
}

static void on_sweep(void *user)
{
    state_t *state = user;

    // Debugging
    render_data(state);
    zarray_clear(state->laser_points);
}

void *process_velodyne_data(void* arg)
{
    state_t *state = arg;

    int delay = 0; // Delay several messages

    double xform_[16];
    float xform[16];

    if (1) {
        const zarray_t *xform_xyz = config_require_doubles(state->config, "velodyne.xyz_m");
        const zarray_t *xform_rpy = config_require_doubles(state->config, "velodyne.rpy_deg");

        double *xyz_ = (double*)xform_xyz->data;
        double *rpy_ = (double*)xform_rpy->data;
        double xyzrpy[6] = { xyz_[0],
                             xyz_[1],
                             xyz_[2],
                             to_radians(rpy_[0]),
                             to_radians(rpy_[1]),
                             to_radians(rpy_[2]) };

        doubles_xyzrpy_to_mat44(xyzrpy, xform_);
        for (int i = 0; i < 16; i++)
            xform[i] = (float)xform_[i];
    }

    while (1) {
        pthread_mutex_lock(&state->velodyne_msgs_mutex);
        if (zqueue_size(state->velodyne_msgs) <= delay) {
            pthread_mutex_unlock(&state->velodyne_msgs_mutex);
            timeutil_usleep(10000);
            continue;
        }
        zqueue_pop(state->velodyne_msgs, &state->msg);
        pthread_mutex_unlock(&state->velodyne_msgs_mutex);

        float M[16];
        float XYT[16];
        float fxyt[3] = {0, 0, 0};
        floats_xyt_to_mat44(fxyt, XYT);
        floats_mat_AB(XYT, 4, 4,
                      xform, 4, 4,
                      M, 4, 4);

        april_velodyne_on_packet(state->velo,
                                 state->msg->buf,
                                 state->msg->len,
                                 M,
                                 on_slice,
                                 on_sweep,
                                 state);

        raw_t_destroy(state->msg);
    }
}

void init_state(state_t* state, getopt_t *gopt)
{
    state->debug = getopt_get_bool(gopt, "debug");
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(getopt_get_int(gopt, "port"),
                                       NULL,
                                       "index.html");
    printf("[INFO] viewer enabled on port %d\n", getopt_get_int(gopt, "port"));
    webvx_define_canvas(state->webvx, "velodyne_terrain_canvas",
                        on_create_canvas, on_destroy_canvas, state);

    if (!state->debug) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "debug-warn");
        vx_buffer_add_back(vb,
                           vxo_chain(vxo_matrix_scale(0.05),
                                     vxo_text(VXO_TEXT_ANCHOR_CENTER, "Please enable debug mode"),
                                     NULL),
                           NULL);
        vx_buffer_swap(vb);
    }

    state->pts = zarray_create(3*sizeof(float));
    state->intensities = zarray_create(1*sizeof(float));
    state->contourExtractor = contourExtractor_create();
    state->lineFitter = lineFitter_create();

    // Initialize LCM/message
    state->lcm = lcm_create(NULL);
    http_advertiser_create(state->lcm, getopt_get_int(gopt, "port"), "Velo2Corners", "Velodyne to corners process");
    state->map_channel = strdup(getopt_get_string(gopt, "map-channel"));
    state->laser_points = zarray_create(sizeof(point_accumulator_t));

    // Velodyne msg queue
    state->velodyne_msgs = zqueue_create(sizeof(raw_t*));
    zqueue_ensure_capacity(state->velodyne_msgs, 100);
    if(!state->lcm) {
        printf("[ERROR] impossible to initialize LCM environment... quitting!");
        exit(-1);
    }

    state->velo = april_velodyne_create();

    // Initialize robot-specific config
    state->config = config_create_path(getopt_get_string(gopt, "config"));
}

int main(int argc, char *argv[])
{
    setlinebuf(stderr);
    setlinebuf(stdout);

    signal(SIGINT, signal_handler);

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_bool(gopt, 'd', "debug", 0, "Debugging visualization");
    getopt_add_int(gopt, 'p', "port", "8899", "vx port");
    getopt_add_string(gopt, '\0', "lidar-channel", "VELODYNE_DATA", "Velodyne channel");

    if (!getopt_parse(gopt, argc, argv, 0)) {
        fprintf(stderr, "ERR: getopt_parse\n");
        return -1;
    }

    if (getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }

    // init state
    state_t* state = calloc(1, sizeof(state_t));
    state->slope = 0.3;
    state->intercept = -4.2;
    init_state(state, gopt);

    // create thread that processes velodyne point clouds
    pthread_t tid;
    int err = pthread_create(&tid, NULL, &process_velodyne_data, state);
    if(err != 0) {
        printf("[ERROR] can't create thread to process velodyne point clouds [%s]\n",
               strerror(err));
        return -1;
    }

    vx_buffer_add_back(vx_world_get_buffer(state->vw, "robot"),
                       vxo_robot_solid(vx_white),
                       NULL);
    vx_buffer_swap(vx_world_get_buffer(state->vw, "robot"));


    // subscribe to velodyne data channel
    raw_t_subscribe(state->lcm, getopt_get_string(gopt, "lidar-channel"), velodyne_callback, state);
    // run the loop
    while(1) {
        lcm_handle(state->lcm);
    }

    zqueue_destroy(state->velodyne_msgs);
    lcm_destroy(state->lcm);
    free(state);

    return 0;
}
