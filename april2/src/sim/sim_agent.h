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

#ifndef _SIM_AGENT_H

#include "common/zarray.h"

#include "common/mesh_model.h"
#include "lcmtypes/pose_t.h"
#include "sim.h"

typedef struct sim_agent sim_agent_t;
struct sim_agent {
    int64_t time;       // What is the current simulation time
                        // this is updated before calling sense/move/manip
    int64_t prev_time;  // What is the current simulation time

    int     id;

    int sim_count;

    pose_t  pose;

    void (*sense)(void * impl);
    void (*move)(void * impl);
    void (*manipulate)(void * impl);
    void (*render)(void * impl);

    sim_world_t * world;
    sim_object_t * so_self;
    zarray_t * so_self_ignore;

    void * sense_state;
    void * move_state;
    void * manipulate_state;
    void * impl;
};

#endif
