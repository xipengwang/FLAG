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

#ifndef GRIDMAP_UTIL_H
#define GRIDMAP_UTIL_H

#include <lcm/lcm.h>

#include "lcmtypes/grid_map_t.h"
#include "common/color.h"

#ifdef __cplusplus
extern "C"{
#endif


// Returns a new grid map that is in ENCODING NONE
// user is responsible for calling "grid_map_t_destroy()" on resulting pointer
grid_map_t * grid_map_t_decode_gzip(const grid_map_t * orig);
grid_map_t * grid_map_t_encode_gzip(const grid_map_t * orig);

// This function checks for collision of car with the current dilated gridmap
// The gridmap should be dilated by WIDTH/2, where WIDTH is width of the car
// Returns 1 if Safe, 0 if finds collision
//int gridmap_check_collision (const grid_map_t *gm, waypoint_t *waypoint);

grid_map_t *grid_map_t_copy_attributes(const grid_map_t *orig);

int gridmap_write_to_file  (const grid_map_t *gm, const char *filename);
int gridmap_write_to_text_file (const grid_map_t *gm, const char *filename);
grid_map_t * gridmap_read_from_file (const char *filename);

// get a constant sized sub gridmap.  Origin is in meters, xy size of submap in pixels
grid_map_t *gridmap_submap (const grid_map_t *gm, double origin[2], double xy[2], uint8_t default_fill);
grid_map_t *gridmap_prune (const grid_map_t *gm);

//xyh -> {x-meters, y-meters, h-radians}
grid_map_t *gridmap_transform (const grid_map_t *gm, double xyh[3]);

void gridmap_publish_encoded (lcm_t *lcm, const char *channel, const grid_map_t *gm);

grid_map_t *gridmap_decode_and_copy (const grid_map_t *orig);


// If the gridmap is encoded, decode it, and resize the orig.data accordingly
//#include "lcmtypes/waypoint_t.h"
void gridmap_decode_in_place (grid_map_t *orig);
void gridmap_encode_in_place (grid_map_t *orig);

// Return a gridmap with the maximum difference between a cell and it's neighbors
grid_map_t *gridmap_max_diff (const grid_map_t *gm);

/**
 * Return a gridmap made from pnm file located at filename. Returns NULL if
 * the file couldn't be found.
 * x0: x location in meters of pixel (0, 0)
 * y0: y location in meters of pixel (0, 0)
 * meters_per_pixel: self-explanatory?
 */
grid_map_t *grid_map_from_pnm(const char *filename, double x0, double y0, double meters_per_pixel);

// copies data from src into dest
// dest and src MUST have the same meters_per_pixel, MUST be grid-aligned, and the
// bounding box of src must be contained in the bounding box of src
// returns 0 on success, 1 otherwise
int gridmap_cpy_data (grid_map_t *dest, const grid_map_t *src);

int gridmap_zcache_save (const void *tile, const char *filename, const void *user);
void *gridmap_zcache_load (const char *filename, const void *user);
void *gridmap_zcache_copy (const void *tile);
void gridmap_zcache_destroy (void * tile);

#ifdef __cplusplus
}
#endif

#endif //DNGV_UTIL_GRID_MAP_UTIL_H
