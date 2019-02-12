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
#include <stdlib.h>
#include <assert.h>
#include <signal.h>
#include <string.h>


#include "april_graph/april_graph.h"

#include "common/doubles.h"
#include "common/floats.h"
#include "common/getopt.h"
#include "common/config.h"
#include "common/rand_util.h"
#include "common/time_util.h"
#include "common/string_util.h"
#include "common/zarray.h"
#include "common/geo_image.h"
#include "common/gps_linearization.h"
#include "common/interpolator.h"
#include "common/stype_lcm.h"
#include "common/global_map.h"
#include "common/gridmap.h"
#include "common/gridmap_util.h"
#include "common/http_advertiser.h"

#include "vx/vx.h"
#include "vx/webvx.h"
#include "vx/vxo_generic.h"

#include "lcm/lcm.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/gps_t.h"
#include "lcmtypes/global_world_state_t.h"
#include "lcmtypes/global_robot_state_t.h"
#include "lcmtypes/line_features_t.h"
#include "lcmtypes/lcmdoubles_t.h"


/**
 * Running program:
 ** ./FLAG-pf -m ../config/mission-gen.config -f ../resc/bbb_floorplan.pnm -i ../resc/flag_landmarks.csv --world ../resc/global_map.composite
 */

#define NUM_OF_PARTICLES 500 //Number of particles
double motion_noise[3] = { 0.1, 0.05, to_radians(0.1) };
double sensor_noise[2] = { 0.15, to_radians(5) };
typedef struct state state_t;
struct state{
    lcm_t *lcm;
    vx_world_t *vw;
    webvx_t *webvx;

    const char *input_file_path;

    zarray_t *landmarks;

    gps_lin_t *gps_lin;
    geo_image_t *geo_img;

    config_t *mission_config;
    vx_object_t *world_img;
    grid_map_t *gm;

    pthread_mutex_t mutex;
    pthread_mutex_t pose_lock;

    double lidar_to_img[3];

    interpolator_t *pose_interp;
    double xyt_local[3];
    double l2g[3];

    double initial_l2g[3];
    double initial_position[3];
    bool is_start;

    zarray_t *odom_poses;
    zarray_t *lidar_poses;
    zarray_t *flag_poses;

    zarray_t *line_features_queue;

    //particles
    double mu[3]; //or use mode
    double particles[NUM_OF_PARTICLES][3];
};

static void signal_handler(int signum)
{
    switch (signum) {
        case SIGINT:
            exit(1);    // XXX Cleanup later, if necessary
            break;
        default:
            break;
    }
}

static void redraw_satellite(state_t *state)
{
    if (state->geo_img == NULL) {
        printf("geo_img empty\n");
        return;
    }

    const matd_t *tr = geo_image_get_matrix(state->geo_img);

    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "satellite");

    vx_object_t *ob = vxo_image_saturated(vx_resource_make_texture_u8x3_copy(geo_image_get_image(state->geo_img),
                                                                             VX_TEXTURE_MAX_LINEAR),
                                          0.5);

    vx_buffer_add_back(vb,
                       vxo_depth_test(0,
                                      vxo_matrix_scale(0.96),
                                      vxo_chain(vxo_matrix_translate(0, 0, 0),
                                                vxo_matrix(tr->data),
                                                ob,
                                                NULL),
                                      NULL),

                       NULL);
    vx_buffer_swap(vb);
}

static void redraw_global_world(state_t *state)
{
    if (state->world_img == NULL) {
        return;
    }
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "map");
    vx_buffer_add_back(vb,
                       vxo_depth_test(0,
                                      vxo_matrix_xyt(state->lidar_to_img),
                                      vxo_chain(vxo_matrix_translate(state->gm->x0, state->gm->y0, 0),
                                                vxo_matrix_scale(state->gm->meters_per_pixel),
                                                state->world_img,
                                                NULL),
                                      NULL),
                       NULL);

    vx_buffer_swap(vb);

}

static void render_landmarks(state_t *state)
{
    vx_buffer_t* vb = vx_world_get_buffer(state->vw, "landmarks");
    float xy[2];
    for (int i = 0; i < zarray_size(state->landmarks); i++) {
        double _xy[2];
        zarray_get(state->landmarks, i, _xy);
        xy[0] = _xy[0];
        xy[1] = _xy[1];
        float *color = vx_white;
        vx_buffer_add_back(vb,
                           vxo_depth_test(0,
                                          vxo_matrix_translate(xy[0], xy[1], 0),
                                          vxo_matrix_scale(0.5),
                                          vxo_square_solid(color),
                                          NULL),
                           NULL);
    }
    vx_buffer_swap(vb);
}

static void render_poses(zarray_t *poses, vx_buffer_t *vb, float color[4])
{
    double pose[3];
    for(int i=0; i < zarray_size(poses); i++) {
        zarray_get(poses, i, pose);
        vx_buffer_add_back(vb,
                           vxo_depth_test(0,
                                          vxo_matrix_xyt(pose),
                                          vxo_robot_solid(color),
                                          NULL),
                           NULL);
    }
}

static void render_particles(state_t *state)
{
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "particles");
    for(int i = 0; i < NUM_OF_PARTICLES; i++) {
            vx_buffer_add_back(vb,
                               vxo_depth_test(0,
                                              vxo_matrix_xyt(state->particles[i]),
                                              vxo_matrix_scale(0.1),
                                              vxo_robot_solid(vx_cyan),
                                              NULL),
                               NULL);
    }
    vx_buffer_swap(vb);
}

static void* render_loop(void *user)
{
    state_t *state = (state_t*)user;
    timeutil_rest_t *rt = timeutil_rest_create();
    vx_buffer_t *vb_lidar = vx_world_get_buffer(state->vw, "Lidar-positions");
    vx_buffer_t *vb_odom = vx_world_get_buffer(state->vw, "Odom-positions");
    vx_buffer_t *vb_flag = vx_world_get_buffer(state->vw, "Flag-positions");
    while(1) {
        timeutil_sleep_hz(rt, 10);
        /*
        double xyt_global[3];
        doubles_xyt_mul(state->l2g, state->xyt_local, xyt_global);
        */
        render_particles(state);
        render_poses(state->lidar_poses, vb_lidar, vx_yellow);
        render_poses(state->odom_poses, vb_odom, vx_blue);
        render_poses(state->flag_poses, vb_flag, vx_red);

        vx_buffer_swap(vb_lidar);
        vx_buffer_swap(vb_odom);
        vx_buffer_swap(vb_flag);
    }
    return NULL;
}

void start_flag(state_t *state)
{
    pthread_mutex_lock(&state->mutex);

    doubles_xyt_mul(state->l2g, state->xyt_local, state->initial_position);
    state->is_start = true;
    memcpy(state->initial_l2g, state->l2g, sizeof(double)*3);

    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "initial-position");
    float rgba[4] = { 1, 1, 1, 1 };
    vx_buffer_add_back(vb,
                       vxo_depth_test(0,
                                      vxo_matrix_xyt(state->initial_position),
                                      vxo_robot_solid(rgba),
                                      NULL),
                       NULL);
    vx_buffer_swap(vb);

    zarray_clear(state->odom_poses);
    zarray_clear(state->lidar_poses);
    zarray_clear(state->flag_poses);

    zarray_add(state->odom_poses, state->initial_position);
    zarray_add(state->lidar_poses, state->initial_position);
    zarray_add(state->flag_poses, state->initial_position);

    //Initalize samples
    for(int i = 0; i < NUM_OF_PARTICLES; i++) {
        memcpy(state->particles[i], state->initial_position, sizeof(double)*3);
    }

    pthread_mutex_unlock(&state->mutex);

}

static void on_console_command(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user)
{
    state_t *state = user;
    zarray_t *toks = str_split(cmd, " ");

    if (zarray_size(toks) >= 1) {
        char *t0;
        zarray_get(toks, 0, &t0);
        if (!strcmp(t0, "start")) {
            vx_console_printf(vc, "Hello world");
            start_flag(state);
            goto command_cleanup;
        }
        vx_console_printf(vc, "unknown command '%s'", cmd);

    }
  command_cleanup:
    zarray_vmap(toks, free);
    zarray_destroy(toks);
}

static zarray_t* on_console_tab(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user)
{
    zarray_t *commands = zarray_create(sizeof(char*));
    char *cmds[] = { "start",
                     NULL };

    for (int i = 0; cmds[i] != NULL; i++) {
        if (!strncmp(cmds[i], cmd, strlen(cmd))) {
            char *s = strdup(cmds[i]);
            zarray_add(commands, &s);
        }
    }

    return commands;

}

static int mouse_down(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
    return 0;
}

static int on_event(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{
    state_t *state = impl;

    switch (ev->type) {
        case VX_EVENT_MOUSE_DOWN:
            return mouse_down(vl, ev, state);
            break;
        case VX_EVENT_KEY_PRESSED:
            if(ev->u.key.key_code == 's') {
                start_flag(state);
            }
            break;
        default:
            break;
    }
    return 0;
}

static void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;
    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_set_world(vl, state->vw);
    vx_console_t *vx_console = vx_console_create(vx_world_get_buffer(state->vw, "console"),
                                                 on_console_command,
                                                 on_console_tab,
                                                 state);

    vx_console_setup_layer(vx_console, vl);
    vx_layer_add_event_handler(vl, on_event, 1, state);


    int order = -10;
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "satellite"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "map"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "particles"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "Odom-positions"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "Lidar-positions"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "Flag-positions"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "corners"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "landmarks"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "initial-position"), order++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "console"), order++);
}

static void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    fprintf(stderr,"ON DESTROY CANVAS\n");
}

static void on_pose(const lcm_recv_buf_t *rbuf,
                    const char *channel, const pose_t *msg, void *userdata)
{
    state_t *state = (state_t*)userdata;
    pthread_mutex_lock(&state->pose_lock);
    doubles_quat_xyz_to_xyt(msg->orientation,
                            msg->pos,
                            state->xyt_local);
    interpolator_add(state->pose_interp, msg);
    pthread_mutex_unlock(&state->pose_lock);
}

static void on_l2g(const lcm_recv_buf_t *rbuf,
                    const char *channel, const lcmdoubles_t *msg, void *userdata)
{
    state_t *state = (state_t*)userdata;
    if(!state->is_start) {
        printf("Get l2g, wait for starting signal....\n");
    }
    pthread_mutex_lock(&state->mutex);
    doubles_xyt_mul(state->lidar_to_img, msg->data, state->l2g);
    pthread_mutex_unlock(&state->mutex);
}

static void floats_to_doubles(float *f, double *d, int len)
{
    for(int i = 0; i<len; i++) {
        d[i] = f[i];
    }
}

static void on_line_features(const lcm_recv_buf_t *rbuf, const char *channel,
                             const line_features_t *msg, void *user)
{
    state_t *state = user;
    pthread_mutex_lock(&state->mutex);
    line_features_t *line_f = line_features_t_copy(msg);
    zarray_add(state->line_features_queue, &line_f);
    pthread_mutex_unlock(&state->mutex);
}

static double gaussian_sample(double mu, double variance)
{
    return mu + sqrt(variance) * randf_normal();
}

static double normpdf_W(double v, double mu, double variance)
{
    return -sq(v-mu) / 2 / variance;
}

static void resample(state_t *state, double w[])
{
    double c = w[0];
    double r = 1.0 / NUM_OF_PARTICLES * randf_uniform(0, 1);
    double particles[NUM_OF_PARTICLES][3];
    memcpy(particles, state->particles, sizeof(double)*NUM_OF_PARTICLES*3);
    int k = 0;
    for (int i = 0; i < NUM_OF_PARTICLES; i++) {
        double U = r + (double)i / NUM_OF_PARTICLES;
        while (U>c) {
            k++;
            c +=  w[k];
        }
        //XXX??? : Will this be a problem, should I have a temperory holder for particles state?
        //memcpy(state->particles[i], state->particles[k], sizeof(double)*3);
        memcpy(state->particles[i], particles[k], sizeof(double)*3);
    }
}

static int find_landmark(state_t *state, double _xy[2], double threshold)
{
    double dist_thresh = threshold;
    int32_t closest_id = -1;
    double closest_dist = dist_thresh;

    double xy[2];
    for (int i = 0; i < zarray_size(state->landmarks); i++) {
        zarray_get(state->landmarks, i, xy);
        double dist = doubles_distance(_xy, xy, 2);
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_id = i;
        }
    }

    return closest_id;
}

void line_feature_process(state_t *state, line_features_t *line_f)
{
    pose_t pose_local;
    int ret;
    lcmdoubles_t pose;
    pthread_mutex_lock(&state->pose_lock);
    if ((ret = interpolator_get(state->pose_interp,
                               line_f->utime,
                               &pose_local)))
    {
        if(ret == -1) {
            printf("No POSE (single side)\n");
            printf("time diff: %f ms \n", (line_f->utime - pose_local.utime) * 1E-3);
        }
        if(ret == -2) {
            printf("No POSE(double side)\n");
        }
        pthread_mutex_unlock(&state->pose_lock);
        return;
    }
    pthread_mutex_unlock(&state->pose_lock);

    double xyt_local[3];
    doubles_quat_xyz_to_xyt(pose_local.orientation,
                            pose_local.pos,
                            xyt_local);

    double lidar_pose[3];
    doubles_xyt_mul(state->l2g, xyt_local, lidar_pose);

    double odom_pose[3];
    doubles_xyt_mul(state->initial_l2g, xyt_local, odom_pose);

    //check movement
    double odom_tmp[3];
    zarray_get(state->odom_poses, zarray_size(state->odom_poses)-1, odom_tmp);
    /* pf updates*/
    double z[3];
    /* node = lastnode + z */
    doubles_xyt_inv_mul(odom_tmp, odom_pose, z);
    //TODO: sampling should be based on the distance moved not fixed covariance.
    if(doubles_magnitude(z, 2) < 0.5) {
        //Move very little, just return;
        return;
    }
    double *motion_std = motion_noise;
    double sampled_xyt[3];

    double W[NUM_OF_PARTICLES] ={ 0 };
    double w[NUM_OF_PARTICLES] ={ 0 };
    double w_max = -HUGE;

    //double mu_sum[2] = {0.0};
    //double cos_sum = 0.0f;
    //double sin_sum = 0.0f;

    for (int k = 0; k < NUM_OF_PARTICLES; k++) {
        double motion[3] = { z[0] - gaussian_sample(0, pow(motion_std[0], 2)),
                             z[1] - gaussian_sample(0, pow(motion_std[1], 2)),
                             z[2] - gaussian_sample(0, pow(motion_std[2], 2)) };
        doubles_xyt_mul(state->particles[k], motion, sampled_xyt);
        memcpy(state->particles[k], sampled_xyt, sizeof(double)*3);

        //mu_sum[0] += state->particles[k][0];
        //mu_sum[1] += state->particles[k][1];
        //cos_sum += cos(state->particles[k][2]);
        //sin_sum += sin(state->particles[k][2]);
    }
    //state->mu[0] = mu_sum[0] / NUM_OF_PARTICLES;
    //state->mu[1] = mu_sum[1] / NUM_OF_PARTICLES;
    //state->mu[2] = atan2(sin_sum, cos_sum);

    //double flag_pose[3];
    if(!line_f->lines_data_length) {
        double mu[3];
        doubles_xyt_mul(state->mu, z, mu);
        memcpy(state->mu, mu, sizeof(double)*3);
        goto line_feature_cleanup;
    }

    for (int k = 0; k < NUM_OF_PARTICLES; k++) {
        w[k] = 1.0;
        /* data association here */
        {
            double xy[2] = { 0 };
            double min_dist = DBL_MAX;
            int closest_landmark_id = -1;
            for (int i = 0; i < line_f->lines_data_length / 3; i++) {
                //transform local points to global
                double local_corner[3];
                double global_corner_position[2];
                floats_to_doubles(&line_f->lines[3*i], local_corner, 3);
                doubles_xyt_transform_xy(state->particles[k], local_corner, global_corner_position);

                /* particles data association radius */
                double threshold = 0.2;
                int landmark_idx =  find_landmark(state, global_corner_position, threshold);
                if (landmark_idx == -1) {
                    continue;
                }
                double landmark[2];
                zarray_get(state->landmarks, landmark_idx, landmark);
                //XXX: For now, we only use the closest landmark as a feature for data asscocaition
                double dist = doubles_distance(global_corner_position, landmark, 2);
                if(dist < min_dist){
                    closest_landmark_id = landmark_idx;
                    min_dist = dist;
                    xy[0] = global_corner_position[0];
                    xy[1] = global_corner_position[1];
                }
            }
            /* no associated measurements */
            if (closest_landmark_id == -1) {
                //XXX: why?
                double delta_z[2] = { 50, M_PI };
                w[k] += normpdf_W(delta_z[0], 0, sensor_noise[0]);
                w[k] += normpdf_W(delta_z[1], 0, sensor_noise[1]);
            } else {
                /* only x and y distance */
                double landmark[2];
                zarray_get(state->landmarks, closest_landmark_id, landmark);
                /* double zhat[2] = { landmark[0] - state->particles[k][0], */
                /*                    landmark[1] - state->particles[k][1] }; //here are the expectations */
                /* double z[2] = { xy[0] - global_pose[0], */
                /*                 xy[1] - global_pose[1] }; //here are the observations */

                /* range and bearing */
                double zhat[2] = { doubles_distance(landmark, state->particles[k], 2),
                                   atan2(landmark[1] - state->particles[k][1], landmark[0] - state->particles[k][0]) };
                double z[2] = { doubles_distance(xy, state->particles[k], 2),
                                atan2(xy[1] - state->particles[k][1], xy[0] - state->particles[k][0]) }; //observations

                double delta_z[2] = { z[0] - zhat[0], mod2pi(z[1] - zhat[1])};
                w[k] += normpdf_W(delta_z[0], 0, sensor_noise[0]);
                w[k] += normpdf_W(delta_z[1], 0, sensor_noise[1]);
            }
        }
        if (w[k] > w_max) {
            w_max = w[k];
        }
    }

    double W_sum = 0;
    for (int k = 0; k < NUM_OF_PARTICLES; k++){
        W_sum += exp(w[k] - w_max);
    }
    double w_sum = log(W_sum);
    double W_sum_check = 0.0;
    double largest_W = -HUGE;
    int largest_W_idx = -1;

    for (int k = 0; k < NUM_OF_PARTICLES; k++) {
        W[k] = exp(w[k] - (w_max + w_sum));
        W_sum_check += W[k];
        if (W[k] > largest_W) {
            largest_W = W[k];
            largest_W_idx = k;
            //use mode instead of mu
        }
    }
    memcpy(state->mu, state->particles[largest_W_idx], sizeof(double)*3);
    resample(state, W);

    /* double corner[2]; */
    /* vx_buffer_t *vb = vx_world_get_buffer(state->vw, "corners"); */
    /* for(int i = 0; i < line_f->lines_data_length / 3; i++) { */
    /*     double local_corners[3]; */
    /*     floats_to_doubles(&line_f->lines[3*i], local_corners, 3); */
    /*     if(doubles_magnitude(local_corners, 2) < 1) */
    /*         continue; */
    /*     doubles_xyt_transform_xy(global_pose, local_corners, corner); */
    /*     vx_buffer_add_back(vb, */
    /*                        vxo_depth_test(0, */
    /*                                       vxo_matrix_translate(corner[0], corner[1], 0.02), */
    /*                                       vxo_matrix_scale(0.3), */
    /*                                       vxo_square_solid(vx_green), */
    /*                                       NULL), */
    /*                        NULL); */
    /* } */
    /* vx_buffer_swap(vb); */

  line_feature_cleanup:
    //check movement
    //publish states

    pose.utime = line_f->utime;
    pose.ndata = 3;
    pose.data = lidar_pose;
    zarray_add(state->lidar_poses, lidar_pose);
    lcmdoubles_t_publish(state->lcm, "FLAG.LIDAR-POSE", &pose);

    pose.data = odom_pose;
    zarray_add(state->odom_poses, odom_pose);
    lcmdoubles_t_publish(state->lcm, "FLAG.ODOM-POSE", &pose);

    pose.data = state->mu;
    lcmdoubles_t_publish(state->lcm, "FLAG.FLAG-POSE", &pose);
    zarray_add(state->flag_poses, state->mu);

    double particles[NUM_OF_PARTICLES*3];
    for(int i=0; i<NUM_OF_PARTICLES; i++) {
        memcpy(&particles[i*3], state->particles[i], sizeof(double)*3);
    }
    pose.ndata = NUM_OF_PARTICLES*3;
    pose.data = particles;
    lcmdoubles_t_publish(state->lcm, "FLAG.PARTICLES", &pose);
}

void *line_f_thread(void *user)
{
    state_t *state = user;

    int low_water = 1;
    int high_water = 10;

    while (1) {
        timeutil_usleep(100000);
        pthread_mutex_lock(&state->mutex);
        // Don't get too far behind
        while (zarray_size(state->line_features_queue) > high_water) {
            line_features_t *l_f = NULL;
            zarray_get(state->line_features_queue, 0, &l_f);
            zarray_remove_index(state->line_features_queue, 0, 0);
            line_features_t_destroy(l_f);
        }

        // Wait if no data
        if (zarray_size(state->line_features_queue) < low_water) {
            pthread_mutex_unlock(&state->mutex);
            continue;
        }

        line_features_t *l_f = NULL;
        zarray_get(state->line_features_queue, 0, &l_f);
        zarray_remove_index(state->line_features_queue, 0, 0);
        pthread_mutex_unlock(&state->mutex);
        if(state->is_start)
            line_feature_process(state, l_f);
        line_features_t_destroy(l_f);
    }

    return NULL;
}

vx_object_t* make_gws_from_file(state_t *state, const char *path)
{
    global_world_state_t *gws = NULL;
    vx_object_t *vxo = NULL;

    FILE *fp = fopen(path, "r");
    fseek(fp, 0L, SEEK_END);
    long len = ftell(fp);
    fseek(fp, 0L, SEEK_SET);

    uint8_t *buf = malloc(len);
    size_t sz = fread(buf, 1, len, fp);

    if (sz != len)
        goto cleanup;

    uint32_t pos = 0;
    gws = stype_decode_object(buf, &pos, len, NULL);
    printf("NFO: Loaded global_world_state_t from %s - %ld bytes\n", path, len);


    grid_map_t *gm = NULL;
    switch (gws->global_map.encoding) {
        case GRID_MAP_T_ENCODING_NONE:
            gm = gridmap_copy(&gws->global_map);
            break;
        case GRID_MAP_T_ENCODING_GZIP:
            gm = grid_map_t_decode_gzip(&gws->global_map);
            break;
        default:
            printf("ERR: encoding type %d not supported\n", gws->global_map.encoding);
            exit(1);
    }


    image_u8_t *im = image_u8_create(gm->width, gm->height);
    for (int y = 0; y < gm->height; y++)
        memcpy(&im->buf[y*im->stride], &gm->data[y*gm->width], gm->width);

    vx_resource_t *tex = vx_resource_make_texture_u8_copy(im, 0);
    image_u8_destroy(im);

    vxo = vxo_image_tile(tex,
                         vx_nada,
                         vx_gray,
                         vx_maroon,
                         vx_orange);
    state->gm = gm;
    global_world_state_t_destroy(gws);

  cleanup:
    free(buf);
    fclose(fp);
    return vxo;
}

void read_landmarks(state_t *state)
{
    FILE *f = fopen(state->input_file_path, "r");

    float x;
    float y;
    if(f) {
        while (fscanf(f, "%f,%f", &x, &y) != EOF) {
            double *xy = (double*) calloc(2, sizeof(float));
            xy[0] = x;
            xy[1] = y;
            zarray_add(state->landmarks, xy);
        }
        printf("Read in %d landmarks \n", zarray_size(state->landmarks));
        fclose(f);
    }
}

void setup_geo(state_t *state, getopt_t *gopt)
{
    state->gps_lin = gps_lin_create();
    double latlon_deg[2] = {0, 0};
    const char *sitename = NULL;
    if (state->mission_config) {
        sitename = config_get_string(state->mission_config, "geo.site", NULL);
    } else {
        printf("No config provided. Operating without site\n");
    }

    const char *image = NULL;
    if (sitename) {
        char *sitepath = sprintf_alloc("geo.%s", sitename);
        char *linpath = sprintf_alloc("%s.linpt", sitepath);
        char *imgpath = sprintf_alloc("%s.image", sitepath);

        printf("loading site %s\n", sitename);
        if (getopt_was_specified(gopt, "image")) {
            image = getopt_get_string(gopt, "image");
        }
        if (config_get_doubles(state->mission_config, linpath, NULL)) {
            config_require_doubles_len(state->mission_config, linpath, latlon_deg, 2);
            printf("%s latlon [%3.6f, %3.6f]\n", sitename, latlon_deg[0], latlon_deg[1]);
        }

        free(sitepath);
        free(linpath);
        free(imgpath);
    }

    gps_lin_setup(state->gps_lin, latlon_deg, (double[]){0,0,0}, utime_now());
    if (image) {
        state->geo_img = geo_image_create(image, state->gps_lin);
        geo_image_update_lin(state->geo_img, state->gps_lin);
    }
}

int main(int argc, char *argv[])
{

    setlinebuf(stderr);
    setlinebuf(stdout);

    stype_register_basic_types();
    stype_register(STYPE_FROM_LCM("grid_map_t", grid_map_t));
    stype_register(STYPE_FROM_LCM("global_world_state_t", global_world_state_t));
    stype_register(STYPE_FROM_LCM("global_robot_state_t", global_robot_state_t));

    getopt_t *gopt = getopt_create();
    getopt_add_int(gopt, 'p', "port", "8891", "jsvx server port");
    getopt_add_string(gopt, 'i', "in-file", "", "input feature file");
    getopt_add_string(gopt, 'f', "image", "", "satellite image file path");
    getopt_add_string(gopt, '\0', "world", "", "global world image file path");
    getopt_add_string(gopt, 'm', "mission-config", "", "Config file for mission");

    if (!getopt_parse(gopt, argc, argv, 1)) {
        getopt_do_usage(gopt);
        return 1;
    }
    signal(SIGINT, signal_handler);
    state_t *state = (state_t*) calloc(1, sizeof(state_t));
    //HARD CODE for evaluation against lidar
    state->lidar_to_img[0] =  -6.600000;
    state->lidar_to_img[1] =  -9.000000;
    state->lidar_to_img[2] =  3.193953;

    state->mission_config = config_create_path(getopt_get_string(gopt, "mission-config"));
    if (!state->mission_config) {
        printf("ERR: bad mission config %s\n", getopt_get_string(gopt, "mission-config"));
        return -1;
    }
    state->lcm = lcm_create(NULL);

    // vx satellite image
    setup_geo(state, gopt);
    // vx canvas initialization
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(getopt_get_int(gopt, "port"),
                                       NULL,
                                       "index.html");

    //each element contains (x,y) position of the landmark in global frame.
    state->landmarks = zarray_create(2*sizeof(double));
    webvx_define_canvas(state->webvx,
                        "flag-map-generator-Canvas",
                        on_create_canvas,
                        on_destroy_canvas,
                        state);
    redraw_satellite(state);

    state->input_file_path = getopt_get_string(gopt, "in-file");
    if (strlen(state->input_file_path)) {
        read_landmarks(state);
        render_landmarks(state);
    }

    if (strlen(getopt_get_string(gopt, "world"))) {
        printf("Load world file from: %s\n", getopt_get_string(gopt, "world"));
        state->world_img = make_gws_from_file(state, getopt_get_string(gopt, "world"));
        redraw_global_world(state);
    }

    //Initilize variables
    state->odom_poses = zarray_create(sizeof(double[3]));
    state->lidar_poses = zarray_create(sizeof(double[3]));
    state->flag_poses = zarray_create(sizeof(double[3]));
    state->line_features_queue = zarray_create(sizeof(line_features_t*));

    pthread_mutex_init(&state->mutex, NULL);
    pthread_mutex_init(&state->pose_lock, NULL);

    state->pose_interp = interpolator_create(sizeof(pose_t), offsetof(pose_t, utime), 5.0, 100E3);
    interpolator_add_field(state->pose_interp, INTERPOLATOR_DOUBLE_LINEAR,
                           3, offsetof(pose_t, pos));
    interpolator_add_field(state->pose_interp, INTERPOLATOR_DOUBLE_QUAT,
                           4, offsetof(pose_t, orientation));


    line_features_t_subscribe(state->lcm,"LOCAL_LINE_FEATURES", on_line_features, state);
    pose_t_subscribe(state->lcm, "POSE", on_pose, state);
    lcmdoubles_t_subscribe(state->lcm, "L2G_SCANMATCH", on_l2g, state);

    http_advertiser_create(state->lcm, getopt_get_int(gopt, "port"),
                           "FLAG localization", "FLAG localization");



    pthread_t render_thread;
    pthread_create(&render_thread, NULL, render_loop, state);

    pthread_t thread;
    pthread_create(&thread, NULL, line_f_thread, state);

    while (1) {
        lcm_handle(state->lcm);
    }

    lcm_destroy(state->lcm);
}
