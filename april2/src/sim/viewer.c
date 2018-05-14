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
#include <math.h>
#include <unistd.h>
#include <float.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#include "vx/vx.h"
#include "vx/webvx.h"

#define SERVO_OFFSET_US 0

#include "common/time_util.h"
#include "common/rand_util.h"
#include "common/matd.h"
#include "common/matd_coords.h"
#include "common/stype.h"
#include "common/stype_lcm.h"
#include "common/interpolator.h"
#include "common/doubles.h"
#include "common/image_u8.h"
#include "common/config.h"
#include "common/config_util.h"

#include "lcmtypes/laser_t.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/dynamixel_status_t.h"

typedef struct state state_t;
struct state
{
    int robot_id;

	lcm_t *lcm;
	vx_world_t *vw;
	webvx_t *webvx;

	int debug;

	interpolator_t *pose_interp;
	interpolator_t *dynamixel_status_interp;

    zarray_t *lasers; // delayed laser_ts

    zarray_t *laser_poses; // the set of scans that we're plotting

    int64_t pose_utime; // newest pose
    int64_t dynamixel_status_utime; // newest dynamixel status

    config_t *config;
};

struct laser_pose
{
    // interpolated position
  	dynamixel_status_t dynstatus;
  	pose_t pose;

  	int npoints;
    float *points; // 3 floats per point.
};

void check_ready_lasers(state_t *state);

void on_laser(const lcm_recv_buf_t *rbuf,
              const char *channel, const laser_t *_msg, void *userdata)
{
	state_t *state = userdata;

    // we put laser data into a waiting queue until we know that we
    // have dynamixel and pose data that will allow the interpolator
    // to succeed.
	laser_t *msg = laser_t_copy(_msg);
	zarray_add(state->lasers, &msg);

	if (zarray_size(state->lasers) > 100) {
	  laser_t *old;
	  zarray_get(state->lasers, 0, &old);
	  zarray_remove_index(state->lasers, 0, 0);
	  laser_t_destroy(old);
	  printf("Laser queue has grown too large-- discarding one; no dynamixel/pose data?\n");
	}
}

void on_pose(const lcm_recv_buf_t *rbuf,
             const char *channel, const pose_t *msg, void *userdata)
{
	state_t *state = userdata;

	interpolator_add(state->pose_interp, msg);
	state->pose_utime = msg->utime;

    check_ready_lasers(userdata);
}


void on_dynamixel_status(const lcm_recv_buf_t *rbuf,
                         const char *channel, const dynamixel_status_t *msg, void *userdata)
{
	state_t *state = userdata;

	interpolator_add(state->dynamixel_status_interp, msg);
	state->dynamixel_status_utime = msg->utime;

    check_ready_lasers(userdata);

}

int on_event(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
	state_t *state = user;
	return 0;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
	state_t *state = impl;

	vx_layer_t *vl = vx_canvas_get_layer(vc, "default");

	vx_layer_set_world(vl, state->vw);

	vx_layer_add_event_handler(vl, on_event, 0, state);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
	state_t *state = impl;

	printf("ON DESTROY CANVAS\n");
}

void check_ready_lasers(state_t *state)
{
	for (int i = 0; i < zarray_size(state->lasers); i++) {
		laser_t *laser;
		zarray_get(state->lasers, i, &laser);

		if (laser->utime < state->pose_utime &&
			laser->utime < state->dynamixel_status_utime) {

			struct laser_pose laserpose;
			memset(&laserpose, 0, sizeof(struct laser_pose));

            // do we have valid data?
			if (!interpolator_get(state->dynamixel_status_interp, laser->utime + SERVO_OFFSET_US, &laserpose.dynstatus) &&
				!interpolator_get(state->pose_interp, laser->utime, &laserpose.pose)) {

                // Now, project the scan into local coordinates as
                // best as we can, using all of the available
                // roll/pitch/yaw information. (In handle_sweep, we
                // will project all of these points into the frame of
                // a "reference pose".)
				laserpose.points = malloc(3 * sizeof(float) * laser->nranges);
				laserpose.npoints = 0;

                // account for position of LIDAR with respect to robot origin
                double Tmount[16];
                config_util_require_mat(state->config, "HOKUYO", Tmount);

                // account for orientation of the servo
                double Tservo[16];
                doubles_angleaxis_to_mat44((double[]) { -laserpose.dynstatus.position_radians, 0, 1, 0 }, Tservo);

                // account for position and orientation of the robot
                double Tpose[16];
                doubles_quat_xyz_to_mat44(laserpose.pose.orientation, laserpose.pose.pos, Tpose);

                double T[16];
                doubles_mat_ABC(Tpose, 4, 4,
                                Tmount, 4, 4,
                                Tservo, 4, 4,
                                T, 4, 4);

                for (int i = 0; i < laser->nranges; i++) {

                    if (laser->ranges[i] < 0)
                        continue;

                    double theta = laser->rad0 + laser->radstep*i;
                    double s = sin(theta), c = cos(theta);

                    double lxyzh[4] = { laser->ranges[i]*c, laser->ranges[i]*s, 0, 1 };
                    double gxyzh[4];

                    doubles_mat_Ab(T, 4, 4, lxyzh, 4, gxyzh, 4);
                    for (int i = 0; i < 3; i++)
                        laserpose.points[3*laserpose.npoints + i] = gxyzh[i];

                    laserpose.npoints++;
                }
            }

            zarray_remove_index(state->lasers, i, 1);
            laser_t_destroy(laser);
            i--;
        }
    }
}

int main(int argc, char *argv[])
{
	setlinebuf(stderr);
	setlinebuf(stdout);

	state_t *state = calloc(1, sizeof(state_t));
    state->robot_id = 2;
	state->lcm = lcm_create(NULL);
	state->vw = vx_world_create();
	state->webvx = webvx_create_server(2000, NULL, "index.html");
    state->config = config_create_path(str_expand_envs("$HOME/ebolson/sidpackbot/robot.config"));

	state->pose_interp = interpolator_create(sizeof(pose_t), offsetof(pose_t, utime), 5.0, 100E3);
	interpolator_add_field(state->pose_interp, INTERPOLATOR_DOUBLE_LINEAR, 3, offsetof(pose_t, pos));
	interpolator_add_field(state->pose_interp, INTERPOLATOR_DOUBLE_QUAT, 4, offsetof(pose_t, orientation));

	state->dynamixel_status_interp = interpolator_create(sizeof(dynamixel_status_t), offsetof(dynamixel_status_t, utime), 5.0, 100E3);
	interpolator_add_field(state->dynamixel_status_interp, INTERPOLATOR_DOUBLE_RADIANS, 1, offsetof(dynamixel_status_t, position_radians));
	interpolator_add_field(state->dynamixel_status_interp, INTERPOLATOR_DOUBLE_LINEAR, 1, offsetof(dynamixel_status_t, speed));

	state->lasers = zarray_create(sizeof(laser_t*));
	state->laser_poses = zarray_create(sizeof(struct laser_pose*));

	webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);

	if (1) {
		vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
		vx_buffer_add_back(vb,
                           vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                           NULL);
		vx_buffer_swap(vb);
	}

	laser_t_subscribe(state->lcm, "HOKUYO_LIDAR", on_laser, state);
	pose_t_subscribe(state->lcm, "POSE", on_pose, state);
	dynamixel_status_t_subscribe(state->lcm, "DYNAMIXEL_STATUS_3", on_dynamixel_status, state);

	while (1)
		lcm_handle(state->lcm);

	return 0;
}
