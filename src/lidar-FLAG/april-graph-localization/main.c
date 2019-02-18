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
 ** ./FLAG--graph-localization -m ../config/mission-gen.config -f ../resc/bbb_floorplan.pnm -i ../resc/flag_landmarks.csv --world ../resc/global_map.composite
 */
#define NUM_MIN 3
#define DIST_ERR_THRES 0.1
#define D_THRESHOLD 0.5
double motion_noise[3] = { 0.1, 0.05, to_radians(0.1) };
typedef struct {
    april_graph_t *graph;
    int nlandmark;
    zarray_t *factor_hypotheses;

    int vote_thres;
    double dist_err_tolerance;
    zarray_t *graduate_factors;
} graph_cut_t;

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
    april_graph_t *graph;
    zarray_t *line_features_queue;

    graph_cut_t *cut;
};

graph_cut_t *graph_cut_create(april_graph_t *g, int nlandmarks,
                              int vote_thres, double dist_err_tollerance)
{
    graph_cut_t *cut = calloc(1, sizeof(graph_cut_t));
    cut->graph = g;
    cut->nlandmark = nlandmarks;
    cut->factor_hypotheses = zarray_create(sizeof(zarray_t*));
    cut->vote_thres = vote_thres;
    cut->dist_err_tolerance = dist_err_tollerance;
    cut->graduate_factors = zarray_create(sizeof(april_graph_factor_t*));
    for(int i=0; i<nlandmarks; i++) {
        zarray_t *landmark_factors = zarray_create(sizeof(april_graph_factor_t*));
        zarray_add(cut->factor_hypotheses, &landmark_factors);
    }
    return cut;
}

bool check_consistency(april_graph_t *graph, april_graph_factor_t *fa, april_graph_factor_t *fb, double thres)
{
    april_graph_node_t *na;
    april_graph_node_t *nb;
    zarray_get(graph->nodes, fa->nodes[1], &na);
    zarray_get(graph->nodes, fb->nodes[1], &nb);
    double pa[2];
    double pb[2];
    doubles_xyt_transform_xy(na->state, fa->u.common.z, pa);
    doubles_xyt_transform_xy(nb->state, fb->u.common.z, pb);
    if(doubles_distance(pa, pb, 2) < thres) {
        return true;
    }
    return false;
}

void graph_cut_add(graph_cut_t *cut, april_graph_factor_t* factor)
{
    assert(factor->type == APRIL_GRAPH_FACTOR_XYT_TYPE);
    zarray_t *landmark_factors;
    //first node should always be landmark node
    assert(factor->nodes[0] < cut->nlandmark);
    zarray_get(cut->factor_hypotheses, factor->nodes[0], &landmark_factors);
    if(!zarray_size(landmark_factors)) {
        zarray_add(landmark_factors, &factor);
        return;
    }
    zarray_add(landmark_factors, &factor);
    int *votes = calloc(zarray_size(landmark_factors), sizeof(int));
    //XXX: Instead of check most recent one, we recheck consistency of all before
    /* april_graph_factor old_factor; */
    /* for(int i=0 ; i<zarray_size(landmark_factors); i++) { */
    /*     zarray_get(landmark_factors, i, &old_factor); */
    /*     //check consistency between old factor and factor */
    /* } */
    april_graph_factor_t *fa;
    april_graph_factor_t *fb;
    for(int i=0 ; i<zarray_size(landmark_factors); i++) {
        for(int j=i+1 ; j<zarray_size(landmark_factors); j++) {
            zarray_get(landmark_factors, i, &fa);
            zarray_get(landmark_factors, j, &fb);
            //check consistency between old factor and factor
            if(check_consistency(cut->graph, fa, fb, cut->dist_err_tolerance)) {
                //printf("Vote++\n");
                votes[i]++;
                votes[j]++;
            }
        }
    }
    int idx = 0;
    int loop_idx = 0;
    while(loop_idx < zarray_size(landmark_factors)) {
        zarray_get(landmark_factors, loop_idx, &fa);
        if(votes[idx] >= cut->vote_thres) {
            //printf("Graduate!\n");
            zarray_add(cut->graduate_factors, &fa);
            zarray_remove_index(landmark_factors, loop_idx, 0);
        } else {
            loop_idx++;
        }
        idx++;
    }
    free(votes);
}

void graph_cut_destroy(graph_cut_t* cut)
{
    if(cut == NULL)
        return;
    zarray_t *landmark_factors;
    april_graph_factor_t *factor;
    for(int i = 0; i < cut->nlandmark; i++) {
        zarray_get(cut->factor_hypotheses, i, &landmark_factors);
        for(int j=0; j<zarray_size(landmark_factors); j++) {
            zarray_get(landmark_factors, j, &factor);
            //XXX: Destroy is not clean
            factor->destroy(factor);
        }
        zarray_destroy(landmark_factors);
    }
    zarray_destroy(cut->factor_hypotheses);
}

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

static void render_graph(state_t *state, vx_buffer_t *vb, float color[4])
{
    for(int i=0; i<state->graph->nodes->size; i++ ) {
        april_graph_node_t *node;
        zarray_get(state->graph->nodes, i, &node);
        if(i < state->landmarks->size) {
            //landmark node
            vx_buffer_add_back(vb,
                               vxo_depth_test(0,
                                              vxo_matrix_translate(node->state[0], node->state[1], 0),
                                              vxo_matrix_scale(0.5),
                                              vxo_square_solid(vx_green),
                                              NULL),
                               NULL);

        }
        else {
            vx_buffer_add_back(vb,
                               vxo_depth_test(0,
                                              vxo_matrix_xyt(node->state),
                                              vxo_robot_solid(color),
                                              NULL),
                               NULL);
        }
    }
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
        render_graph(state, vb_flag, vx_red);
        render_poses(state->lidar_poses, vb_lidar, vx_yellow);
        render_poses(state->odom_poses, vb_odom, vx_blue);

        vx_buffer_swap(vb_lidar);
        vx_buffer_swap(vb_odom);
        vx_buffer_swap(vb_flag);
    }
    return NULL;
}

static void load_landmark_nodes(state_t *state)
{
    for(int i = 0; i < zarray_size(state->landmarks); i++) {
        double xy[2];
        zarray_get(state->landmarks, i, xy);
        //add xy node
        april_graph_node_t *node = april_graph_node_xy_create(xy, xy, xy);
        zarray_add(state->graph->nodes, &node);
        //add xypos factor
        matd_t *W = matd_create(2, 2);
        MATD_EL(W, 0, 0) = 1.0 / pow(0.1, 2);
        MATD_EL(W, 1, 1) = 1.0 / pow(0.1, 2);
        april_graph_factor_t *factor = april_graph_factor_xypos_create(i, xy, NULL, W);
        april_graph_factor_attr_put(factor, NULL, "type", strdup("landmark"));
        zarray_add(state->graph->factors, &factor);
        matd_destroy(W);
    }
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
    //XXX: Memory leak here;
    april_graph_destroy(state->graph);
    state->graph = april_graph_create();
    graph_cut_destroy(state->cut);
    state->cut = graph_cut_create(state->graph, zarray_size(state->landmarks), NUM_MIN, DIST_ERR_THRES);
    load_landmark_nodes(state);

    zarray_add(state->odom_poses, state->initial_position);
    zarray_add(state->lidar_poses, state->initial_position);

    //add a new node into aprilgraph;
    april_graph_node_t *node = april_graph_node_xyt_create(state->initial_position,
                                                           state->initial_position,
                                                           state->initial_position);
    zarray_add(state->graph->nodes, &node);
    //add a xytpos factor
    matd_t *W = matd_create(3, 3);
    MATD_EL(W, 0, 0) = 1.0 / pow(0.01, 2);
    MATD_EL(W, 1, 1) = 1.0 / pow(0.01, 2);
    MATD_EL(W, 2, 2) = 1.0 / pow(0.001, 2);
    april_graph_factor_t *factor = april_graph_factor_xytpos_create(zarray_size(state->graph->nodes)-1,
                                                                    state->initial_position, NULL, W);
    april_graph_factor_attr_put(factor, NULL, "type", strdup("Initial"));
    zarray_add(state->graph->factors, &factor);
    matd_destroy(W);

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
        if (!strcmp(t0, "save")) {
            vx_console_printf(vc, "Save");
            april_graph_save(state->graph, "/tmp/FLAG.graph");
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

static void optimize_cholesky(state_t *state)
{
    //int64_t utime0 = utime_now();
    struct april_graph_cholesky_param chol_param;
    april_graph_cholesky_param_init(&chol_param);
    chol_param.tikhanov = 1.0E-6;
    april_graph_cholesky(state->graph, &chol_param);
    //int64_t utime1 = utime_now();
    //printf("optimized using cholesky (%.3f s)\n", (utime1 - utime0) / 1.0E6);
}

static int find_landmark(state_t *state, double _xy[3], double threshold)
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

static void on_line_features(const lcm_recv_buf_t *rbuf, const char *channel,
                             const line_features_t *msg, void *user)
{
    state_t *state = user;
    pthread_mutex_lock(&state->mutex);
    line_features_t *line_f = line_features_t_copy(msg);
    zarray_add(state->line_features_queue, &line_f);
    pthread_mutex_unlock(&state->mutex);
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
    if(doubles_magnitude(z, 2) < 0.4) {
        //Move very little, just return;
        return;
    }
    april_graph_node_t *last_node;
    int last_node_idx = zarray_size(state->graph->nodes)-1;
    zarray_get(state->graph->nodes, last_node_idx, &last_node);
    double nstate[3];
    doubles_xyt_mul(last_node->state, z, nstate);
    april_graph_node_t *new_node = april_graph_node_xyt_create(nstate, nstate, nstate);
    zarray_add(state->graph->nodes, &new_node);

    //add odometry factor
    matd_t *W = matd_create(3, 3);
    MATD_EL(W, 0, 0) = 1.0 / pow(motion_noise[0], 2);
    MATD_EL(W, 1, 1) = 1.0 / pow(motion_noise[1], 2);
    MATD_EL(W, 2, 2) = 1.0 / pow(motion_noise[2], 2);

    april_graph_factor_t *factor = april_graph_factor_xyt_create(last_node_idx, zarray_size(state->graph->nodes)-1,
                                                                 z, NULL, W);
    april_graph_factor_attr_put(factor, NULL, "type", strdup("odom"));
    zarray_add(state->graph->factors, &factor);

    //double flag_pose[3];
    if(!line_f->lines_data_length) {
        goto line_feature_cleanup;
    }

    //Data association here
    for (int i = 0; i < line_f->lines_data_length / 3; i++) {
        //transform local points to global
        double local_corner[3];
        double global_corner_position[2];
        floats_to_doubles(&line_f->lines[3*i], local_corner, 3);
        doubles_xyt_transform_xy(new_node->state, local_corner, global_corner_position);
        const double d_threshold = D_THRESHOLD;
        int idx = find_landmark(state, global_corner_position, d_threshold);
        if(idx != -1) {
            //TODO: Instead of directly adding a factor, we add a potential factor, if enough observations are consistent,
            //we will add a factor. Instead of use graph-cut to check consistency, maybe it is easiser use a simpler version of loopval.
            //graph_cut_create(graph, num_of_landmark_nodes);
            //graph_cut_add(factor);
            //april_graph_factor_t *factor = graph_cut_graduate();
            //add r factor
            const int choose = 1;
            if(choose == 0) {
                matd_t *W1 = matd_identity(1);
                MATD_EL(W1, 0, 0) = 1.0 / pow(0.05, 2);
                matd_t *W2 = matd_op("0.01*M", W1);
                double z = doubles_magnitude(local_corner, 2);
                april_graph_factor_t *factor1 = april_graph_factor_r_create(idx, zarray_size(state->graph->nodes)-1, &z, &z, W1);
                april_graph_factor_t *factor2 = april_graph_factor_r_create(idx, zarray_size(state->graph->nodes)-1, &z, &z, W2);

                april_graph_factor_t *maxfactor = april_graph_factor_max_create(
                    (april_graph_factor_t*[]) { factor1, factor2 },
                    (double[]) { 1, 100 },
                    2);
                zarray_add(state->graph->factors, &maxfactor);
                matd_destroy(W1);
                matd_destroy(W2);
            }
            else if(choose == 1) {
                matd_t *W = matd_identity(3);
                april_graph_factor_t *factor = april_graph_factor_xyt_create(idx, zarray_size(state->graph->nodes)-1,
                                                                             local_corner,
                                                                             NULL,
                                                                             W);
                matd_destroy(W);
                graph_cut_add(state->cut, factor);
                for(int i =0; i<zarray_size(state->cut->graduate_factors); i++) {
                    zarray_get(state->cut->graduate_factors, i, &factor);
                    double z =  doubles_magnitude(factor->u.common.z, 2);
                    matd_t *W1 = matd_identity(1);
                    MATD_EL(W1, 0, 0) = 1.0 / pow(0.05, 2);
                    matd_t *W2 = matd_op("0.01*M", W1);
                    april_graph_factor_t *factor1 = april_graph_factor_r_create(factor->nodes[0], factor->nodes[1], &z, &z, W1);
                    april_graph_factor_t *factor2 = april_graph_factor_r_create(factor->nodes[0], factor->nodes[1], &z, &z, W2);

                    april_graph_factor_t *maxfactor = april_graph_factor_max_create(
                        (april_graph_factor_t*[]) { factor1, factor2 },
                        (double[]) { 1, 100 },
                        2);

                    zarray_add(state->graph->factors, &maxfactor);
                    //XXX: Mem leak
                    factor->destroy(factor);
                    matd_destroy(W1);
                    matd_destroy(W2);
                }
                zarray_clear(state->cut->graduate_factors);
            }
        }
    }

  line_feature_cleanup:
    optimize_cholesky(state);
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

    pose.data = new_node->state;
    lcmdoubles_t_publish(state->lcm, "FLAG.FLAG-POSE", &pose);
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
    state->graph = april_graph_create();
    load_landmark_nodes(state);
    state->odom_poses = zarray_create(sizeof(double[3]));
    state->lidar_poses = zarray_create(sizeof(double[3]));
    state->line_features_queue = zarray_create(sizeof(line_features_t*));

    state->cut = graph_cut_create(state->graph, zarray_size(state->landmarks), NUM_MIN, DIST_ERR_THRES);

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
