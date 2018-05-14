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

#ifndef _GRIDMAP_H
#define _GRIDMAP_H

/** A rudimentary gridmap class. Please note the conventions listed here!

    Pixel coordinates are obtained via this meters->pixels transformation:
        ix = (int) ((px - x0) / metersPerPixel)
        iy = (int) ((py - y0) / metersPerPixel)

    Pixel centers in meters are obtained via this pixels->meters transformation:
        px = x0 + (ix + 0.5) * metersPerPixel
        py = y0 + (iy + 0.5) * metersPerPixel

    The methods gridmap_get_index_[xy]() and gridmap_get_meters() are provided
    as examples. Efficiency-conscious users might re-implement these methods to
    eliminate error checking.
**/

#include <stdint.h>
#include <stdbool.h>

#include "lcmtypes/grid_map_t.h"
#include "common/image_u8.h"
#include "common/image_u8x4.h"

#ifdef __cplusplus
extern "C" {
#endif

enum GRID_VAL {
 GRID_VAL_TRAVERSABLE   = 0,
 GRID_VAL_OBSTACLE      = 1,
 GRID_VAL_SLAMMABLE     = 2,
 GRID_VAL_UNKNOWN       = 3,
 GRID_VAL_OTHER         = 4,
 GRID_VAL_CHECKED       = 5,
};


// create a grid_map_t that must be destroyed by the user
grid_map_t *gridmap_make_meters(double x0, double y0, double sizex, double sizey,
                               double meters_per_pixel, uint8_t default_fill);

// create a grid_map_t that must be destroyed by the user
grid_map_t *gridmap_make_pixels(double x0, double y0, int32_t width, int32_t height,
                               double meters_per_pixel, uint8_t default_fill,
                               uint8_t round_up_dimensions);

// create a grid_map_t that must be destroyed by the user
grid_map_t *gridmap_copy(const grid_map_t *gm);

void gridmap_destroy(grid_map_t *gridmap);

void gridmap_fill(grid_map_t *gm, uint8_t v);
// XXX Use fill (0) instead void gridmap_clear(grid_map_t *gm);

// Returns true iff (x,y) is within the gridmap bounds
static inline bool gridmap_is_pos_in_map(const grid_map_t *gm, const double x, const double y)
{
    return x > gm->x0 &&
           y > gm->y0 &&
           x < gm->x0 + gm->width*gm->meters_per_pixel &&
           y < gm->y0 + gm->height*gm->meters_per_pixel;
}

void gridmap_set_value(grid_map_t *gm, double x, double y, uint8_t v);
void gridmap_set_value_index(grid_map_t *gm, int32_t ix, int32_t iy, uint8_t v);
void gridmap_set_value_index_safe(grid_map_t *gm, int32_t ix, int32_t iy, uint8_t v);

uint8_t gridmap_get_value(const grid_map_t *gm, double x, double y);
uint8_t gridmap_get_value_safe(const grid_map_t *gm, double x, double y, uint8_t def);
uint8_t gridmap_get_value_index(const grid_map_t *gm, int32_t ix, int32_t iy);
uint8_t gridmap_get_value_index_safe(const grid_map_t *gm, int32_t ix, int32_t iy, uint8_t def);

uint8_t gridmap_get_max_value(const grid_map_t *gm);
uint8_t gridmap_get_min_value(const grid_map_t *gm);

// get the array index for the input coordinate specified in meters.
// returns -1 if the coordinate is out of bounds
int32_t gridmap_get_index_x(const grid_map_t *gm, double x);
int32_t gridmap_get_index_y(const grid_map_t *gm, double y);

//No error checking
int32_t gridmap_get_index(const grid_map_t *gm, double x, double y);

image_u8x4_t *gridmap_make_image_u8x4(const grid_map_t *gm, const uint8_t gray_alpha,
                                      const uint8_t rgba_obstacle[4], const uint8_t rgba_unknown[4]);
image_u8x4_t *gridmap_make_image_u8x4_raw(const grid_map_t *gm, const uint8_t gray_alpha);

int8_t gridmap_get_pixel_center(const grid_map_t *gm, int32_t ix, int32_t iy, double *x, double *y);

// set x and y to the position in meters of the index specified. if ix or iy
// are out of bounds, x and y are not set. returns 1 on error
int gridmap_get_meters(const grid_map_t *gm, int ix, int iy, double *x, double *y);

// Crop/resize gridmap
grid_map_t *gridmap_resize_pixels(const grid_map_t *gm, int xmin, int ymin, int _width,
                                  int _height, int default_fill, bool round_up_dimensions);
grid_map_t *gridmap_crop(const grid_map_t *gm, bool round_up_dimensions, uint8_t ignore_val);

grid_map_t *gridmap_crop_pixels(const grid_map_t *gm, int xmin, int ymin,
                                int _width, int _height,
                                bool round_up_dimensions);

grid_map_t *gridmap_crop_meters(const grid_map_t *gm, double xmin, double ymin,
                                double width, double height,
                                bool round_up_dimensions);

/**
 * Does the grid cell at (ix,iy) have a neighbor whose value is
 * v? ix, iy are in pixel coordinates.
 **/
int gridmap_has_neighbor(const grid_map_t* gm, int ix, int iy, uint8_t v);
/**
 * Set all neighbors of grid cells with value 'v' to v. Repeat
 * this process 'iterations' times.
 **/
void gridmap_dilate(grid_map_t* gm, uint8_t v, int iterations);
/**
 * Convolve gridmap with disc specified by radius.
 **/
void gridmap_convolve_centered_disc_max (grid_map_t *gm, int radius);
/**
 * Convolve gridmap with specified box filter.
 **/
void gridmap_convolve_separable_max (grid_map_t *gm, int dim, double *filter, double padding);

grid_map_t *gridmap_max_convolution(const grid_map_t *gm_in, int k);

grid_map_t *gridmap_decimate_max(const grid_map_t *gm, int k);

grid_map_t *gridmap_downsample_mean (const grid_map_t *gm); // downsample by doubling meter_per_pixel
grid_map_t *gridmap_downsample_min (const grid_map_t *gm); // downsample by doubling meter_per_pixel
grid_map_t *gridmap_downsample_max (const grid_map_t *gm); // downsample by doubling meter_per_pixel
void gridmap_downsample_min_and_max (const grid_map_t *gm, grid_map_t **gmin, grid_map_t **gmax); // downsample with 2 outputs

/*
 * Gridmap drawing functions
 */
void gridmap_draw_circle(grid_map_t *gm, double cx, double cy, double r, uint8_t fill);
void gridmap_draw_circle_max(grid_map_t *gm, double cx, double cy, double r, uint8_t fill);
void gridmap_draw_line(grid_map_t *gm, double xa, double ya, double xb, double yb, uint8_t fill);
void gridmap_draw_line_max(grid_map_t *gm, double xa, double ya, double xb, double yb, uint8_t fill);
void gridmap_draw_line_interpolate(grid_map_t *gm, double xa, double ya, double xb, double yb, int f0, int f1);
void gridmap_draw_rectangle(grid_map_t * gm, double cx, double cy, double x_size, double y_size, double theta, uint8_t fill);

/* Basic arithmetic */
void gridmap_max(const grid_map_t *in1, const grid_map_t *in2, grid_map_t *out);

/** Path Integrals
 * If negativeOn255 is set, -1 will be returned if the path
 * goes through a cell whose value is 255. */
double gridmap_evaluate_path(const grid_map_t *gm, const double* xys, int num_points, int negativeOn255);
double gridmap_evaluate_path_edge(const grid_map_t *gm, const double xy0[2], const double xy1[2], int negativeOn255);

/** Methods that can be easily ported from april.util.GridMap:
  *     - crop/resize/recenter
  *     - scale/add/subtract
  *     - filldown/fillup (flood fill operations)
  *     - [partly ported] draw dot/line/circle/rectangle (with and without lookup tables for cost functions)
  *     - decimate
  * and more
  */

// Sets all cells in the gridmap to the given value
static inline void gridmap_set_all(grid_map_t *gm, const uint8_t v) {
    for (int i = 0; i < gm->width; i++)
        for (int j = 0; j < gm->height; j++)
            gridmap_set_value_index(gm, i, j, v);
}


/* Returns true if xy0 and xy1 belong to the same connected component of the gridmap,
 * where connected components consist of non-255 pixels.
 * Returns false if any of the poses is off the map */
bool gridmap_are_poses_connected(const grid_map_t *gm, const double xy0[2], const double xy1[2]);

#ifdef __cplusplus
}
#endif

#endif
