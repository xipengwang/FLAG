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

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <limits.h>

#include "common/image_u8x4.h"
#include "common/time_util.h"
#include "common/math_util.h"
#include "common/zqueue.h"
#include "lcmtypes/grid_map_t.h"

#include "gridmap.h"

grid_map_t *gridmap_make_meters(double x0, double y0, double sizex, double sizey,
                                double meters_per_pixel, uint8_t default_fill)
{
    // this format guarantees we do not over-allocate unless necessary, i.e., fmod(size*, meters_per_pixel) != 0
    int32_t width  = (int32_t) (sizex / meters_per_pixel);
    int32_t height = (int32_t) (sizey / meters_per_pixel);
    if (sizex - width*meters_per_pixel > 1e-6) width++;
    if (sizey - height*meters_per_pixel > 1e-6) height++;

    return gridmap_make_pixels(x0, y0, width, height, meters_per_pixel, default_fill, 0);
}

grid_map_t *gridmap_make_pixels(double x0, double y0, int32_t width, int32_t height,
                                double meters_per_pixel, uint8_t default_fill,
                                uint8_t round_up_dimensions)
{
    grid_map_t *gm = malloc(sizeof(grid_map_t));
    //gm->utime = 0;
    gm->x0 = x0;
    gm->y0 = y0;
    gm->meters_per_pixel = meters_per_pixel;

    gm->width = width;
    gm->height = height;

    if (round_up_dimensions) {
        gm->width  += 4 - (gm->width%4);
        gm->height += 4 - (gm->height%4);
    }
    gm->datalen = gm->width * gm->height;
    gm->data = malloc(gm->datalen * sizeof(uint8_t));

    gm->encoding = GRID_MAP_T_ENCODING_NONE;
    // always fill because we used malloc
    gridmap_fill(gm, default_fill);

    return gm;
}

grid_map_t *gridmap_copy(const grid_map_t *gm)
{
    if (gm == NULL)
        return NULL;

    grid_map_t *copy = malloc(sizeof(grid_map_t));
    copy->utime             = gm->utime;
    copy->encoding          = gm->encoding;
    copy->x0                = gm->x0;
    copy->y0                = gm->y0;
    copy->meters_per_pixel  = gm->meters_per_pixel;
    copy->width             = gm->width;
    copy->height            = gm->height;
    copy->datalen           = gm->datalen;

    copy->data = malloc(copy->datalen);
    memcpy(copy->data, gm->data, copy->datalen * sizeof(*gm->data));

    return copy;
}

void gridmap_destroy(grid_map_t *gridmap)
{
    free(gridmap->data);
    free(gridmap);
}

void gridmap_fill(grid_map_t *gm, uint8_t v)
{
    assert(gm != NULL);
    assert(gm->data != NULL);

    memset(gm->data, v, gm->width*gm->height*sizeof(uint8_t));
}


void gridmap_set_value(grid_map_t *gm, double x, double y, uint8_t v)
{
    int32_t ix = (int32_t) ((x - gm->x0) / gm->meters_per_pixel);
    int32_t iy = (int32_t) ((y - gm->y0) / gm->meters_per_pixel);

    gridmap_set_value_index_safe(gm, ix, iy, v);
}

void gridmap_set_value_index(grid_map_t *gm, int32_t ix, int32_t iy, uint8_t v)
{
    gm->data[iy*gm->width + ix] = v;
}

void gridmap_set_value_index_safe(grid_map_t *gm, int32_t ix, int32_t iy, uint8_t v)
{
    if (ix < 0 || iy < 0 || ix >= gm->width || iy >= gm->height)
        return;

    gm->data[iy*gm->width + ix] = v;
}

uint8_t gridmap_get_value(const grid_map_t *gm, double x, double y)
{
    int32_t ix = (int32_t) ((x - gm->x0) / gm->meters_per_pixel);
    int32_t iy = (int32_t) ((y - gm->y0) / gm->meters_per_pixel);

    return gridmap_get_value_index_safe(gm, ix, iy, 0);
}

uint8_t gridmap_get_value_safe(const grid_map_t *gm, double x, double y, uint8_t def){
    int32_t ix = (int32_t) ((x - gm->x0) / gm->meters_per_pixel);
    int32_t iy = (int32_t) ((y - gm->y0) / gm->meters_per_pixel);

    return gridmap_get_value_index_safe(gm, ix, iy, def);

}

uint8_t gridmap_get_value_index(const grid_map_t *gm, int32_t ix, int32_t iy)
{
    return gm->data[iy*gm->width + ix];
}

uint8_t gridmap_get_value_index_safe(const grid_map_t *gm, int32_t ix, int32_t iy, uint8_t def)
{
    if (ix < 0 || iy < 0 || ix >= gm->width || iy >= gm->height)
        return def;

    return gm->data[iy*gm->width + ix];
}

uint8_t gridmap_get_max_value(const grid_map_t *gm)
{
    uint8_t max = 0x00;
    for (int i = 0; i < gm->datalen; i++)
        if (gm->data[i] > max)
            max = gm->data[i];
    return max;
}

uint8_t gridmap_get_min_value(const grid_map_t *gm)
{
    uint8_t min = 0xff;
    for (int i = 0; i < gm->datalen; i++)
        if (gm->data[i] < min)
            min = gm->data[i];
    return min;
}

int32_t gridmap_get_index(const grid_map_t *gm, double x, double y)
{
    int32_t ix = (int32_t) ((x - gm->x0) / gm->meters_per_pixel);
    int32_t iy = (int32_t) ((y - gm->y0) / gm->meters_per_pixel);

    return iy*gm->width + ix;
}

int32_t gridmap_get_index_x(const grid_map_t *gm, double x)
{
    int32_t ix = (int32_t) ((x - gm->x0) / gm->meters_per_pixel);

    if (ix >= 0 && ix < gm->width)
        return ix;

    return -1;
}

int32_t gridmap_get_index_y(const grid_map_t *gm, double y)
{
    int32_t iy = (int32_t) ((y - gm->y0) / gm->meters_per_pixel);

    if (iy >= 0 && iy < gm->height)
        return iy;

    return -1;
}

image_u8x4_t *gridmap_make_image_u8x4(const grid_map_t *gm, const uint8_t gray_alpha,
                                      const uint8_t rgba_obstacle[4], const uint8_t rgba_unknown[4])
{
    // only implemented for uncompressed gridmaps
    assert(gm->encoding == GRID_MAP_T_ENCODING_NONE);
    int      gmwidth  = gm->width;
    int      gmheight = gm->height;
    uint8_t *gmbuf    = gm->data;

    image_u8x4_t *imp = image_u8x4_create(gm->width, gm->height); // RGBA image
    image_u8x4_t im = *imp; // copy value of image struct

    for(int y = 0; y < gmheight; y++) {
        for(int x = 0; x < gmwidth; x++) {

            uint8_t cell = gmbuf[y*gmwidth + x];

            if (cell == 0xff) {
                im.buf[y*im.stride + 4*x + 0] = rgba_obstacle[0];
                im.buf[y*im.stride + 4*x + 1] = rgba_obstacle[1];
                im.buf[y*im.stride + 4*x + 2] = rgba_obstacle[2];
                im.buf[y*im.stride + 4*x + 3] = rgba_obstacle[3];
            }
            else if (cell == 0xfe) {
                im.buf[y*im.stride + 4*x + 0] = rgba_unknown[0];
                im.buf[y*im.stride + 4*x + 1] = rgba_unknown[1];
                im.buf[y*im.stride + 4*x + 2] = rgba_unknown[2];
                im.buf[y*im.stride + 4*x + 3] = rgba_unknown[3];
            }
            else {
                im.buf[y*im.stride + 4*x + 0] = cell;
                im.buf[y*im.stride + 4*x + 1] = cell;
                im.buf[y*im.stride + 4*x + 2] = cell;
                im.buf[y*im.stride + 4*x + 3] = gray_alpha;
            }
        }
    }

    return imp;
}

image_u8x4_t *gridmap_make_image_u8x4_raw(const grid_map_t *gm, const uint8_t gray_alpha)
{
    // only implemented for uncompressed gridmaps
    assert(gm->encoding == GRID_MAP_T_ENCODING_NONE);
    int      gmwidth  = gm->width;
    int      gmheight = gm->height;
    uint8_t *gmbuf    = gm->data;
    image_u8x4_t *imp = image_u8x4_create(gm->width, gm->height); // RGBA image
    image_u8x4_t im = *imp; // copy value of image struct

    for(int y = 0; y < gmheight; y++) {
        for(int x = 0; x < gmwidth; x++) {
            uint8_t cell = gmbuf[y*gmwidth + x];
            im.buf[y*im.stride + 4*x + 0] = cell;
            im.buf[y*im.stride + 4*x + 1] = cell;
            im.buf[y*im.stride + 4*x + 2] = cell;
            im.buf[y*im.stride + 4*x + 3] = gray_alpha;
        }
    }
    return imp;
}

int8_t gridmap_get_pixel_center(const grid_map_t *gm, int32_t ix, int32_t iy, double *x, double *y){


    if(!gm){
        return 1;
    }
    if(!(ix >= 0 && ix < gm->width && iy >=0 && iy < gm->height)){
        return 1;
    }

    *x = gm->x0 + (ix + 0.5) * gm->meters_per_pixel;
    *y = gm->y0 + (iy + 0.5) * gm->meters_per_pixel;
    return 0;
}


grid_map_t *gridmap_resize_pixels(const grid_map_t *gm, int xmin, int ymin, int _width,
                                  int _height, int default_fill, bool round_up_dimensions)
{
    grid_map_t *gm_new = malloc(sizeof(grid_map_t));

    gm_new->utime = gm->utime;
    gm_new->x0 = gm->x0 + xmin * gm->meters_per_pixel;
    gm_new->y0 = gm->y0 + ymin * gm->meters_per_pixel;
    gm_new->width =  _width;
    gm_new->height = _height;
    gm_new->meters_per_pixel = gm->meters_per_pixel;
    gm_new->encoding = gm->encoding;

    if (round_up_dimensions) {
        // round up to multiple of four (necessary for OpenGL happiness)
        gm_new->width += (4 - (gm_new->width % 4)) % 4; // final mod ensures we don't add 4 needlessly
        gm_new->height += (4 - (gm_new->height% 4)) % 4;
    }

    gm_new->datalen = gm_new->width * gm_new->height;
    gm_new->data = malloc(gm_new->datalen * sizeof(uint8_t));

    // crawl the new gm and insert old values where applicable
    for (int y = 0; y < gm_new->height; y++) {
        for (int x = 0; x < gm_new->width; x++) {
            if (y + ymin >= 0 && y + ymin < gm->height &&
                x + xmin >= 0 && x + xmin < gm->width)
                gm_new->data[y * gm_new->width + x] = gm->data[(y+ymin)*gm->width + (x+xmin)];
            else
                gm_new->data[y * gm_new->width + x] = default_fill;
        }
    }
    return gm_new;
}


// Return a gridmap that contains all of the non-zero pixels, but
// is (potentially) smaller than the original
grid_map_t *gridmap_crop(const grid_map_t *gm, bool round_up_dimensions, uint8_t ignore_val)
{
    int xmin = INT_MAX;
    int xmax = -1;
    int ymin = INT_MAX;
    int ymax = -1;

    // find bounding box. (This is a bit inefficient... perhaps it
    // would be faster if we worked in from the edges.)
    for (int y = 0; y < gm->height; y++) {
        for (int x = 0; x < gm->width; x++) {
            if (gm->data[y * gm->width + x] != ignore_val) {
                xmin = fmin(xmin, x);
                xmax = fmax(xmax, x);
                ymin = fmin(ymin, y);
                ymax = fmax(ymax, y);
            }
        }
    }

    if (xmax < xmin) {
        xmin = 0;
        ymin = 0;
        xmax = 0;
        ymax = 0;
    }

    return gridmap_resize_pixels(gm, xmin, ymin, xmax-xmin+1, ymax-ymin+1, 0, round_up_dimensions);
}

grid_map_t *gridmap_crop_meters(const grid_map_t *gm, double xmin, double ymin,
                                double width, double height,
                                bool round_up_dimensions)
{
    return gridmap_crop_pixels(gm,
                               (int) ((xmin - gm->x0) / gm->meters_per_pixel),
                               (int) ((ymin - gm->y0) / gm->meters_per_pixel),
                               (int) (width / gm->meters_per_pixel),
                               (int) (height / gm->meters_per_pixel),
                               round_up_dimensions);
}

grid_map_t *gridmap_crop_pixels(const grid_map_t *gm, int xmin, int ymin,
                                int _width, int _height,
                                bool round_up_dimensions)
{
    xmin = max(0, xmin);
    ymin = max(0, ymin);

    _width = max(0, _width);
    _height = max(0, _height);

    return gridmap_resize_pixels(gm, xmin, ymin, _width, _height,
                                 0, round_up_dimensions);
}


int gridmap_get_meters(const grid_map_t *gm, int ix, int iy, double *x, double *y)
{
    if (ix < 0 || iy < 0 || ix >= gm->width || iy >= gm->height)
        return 1;

    *x = gm->x0 + (ix + 0.5) * gm->meters_per_pixel;
    *y = gm->y0 + (iy + 0.5) * gm->meters_per_pixel;

    return 0;
}

int gridmap_has_neighbor(const grid_map_t* gm, int ix, int iy, uint8_t v)
{
    int width = gm->width;
    int height = gm->height;
    if (iy > 0) {
        if (ix > 0)
            if (gm->data[(iy-1)*width+(ix-1)]==v)
                return 1;
        if (gm->data[(iy-1)*width+ix]==v)
            return 1;
        if (ix+1 < width)
            if (gm->data[(iy-1)*width+(ix+1)]==v)
                return 1;
    }

    if (ix > 0)
        if (gm->data[iy*width+ix-1]==v)
            return 1;
    if (ix+1 < width)
        if (gm->data[iy*width+ix+1]==v)
            return 1;

    if (iy+1 < height) {
        if (ix > 0)
            if (gm->data[(iy+1)*width+(ix-1)]==v)
                return 1;
        if (gm->data[(iy+1)*width+ix]==v)
            return 1;
        if (ix+1 < width)
            if (gm->data[(iy+1)*width+(ix+1)]==v)
                return 1;
    }
    return 0;
}

void gridmap_dilate(grid_map_t* gm, uint8_t v, int iterations)
{
    int height = gm->height;
    int width = gm->width;

    grid_map_t *bef = gm;
    grid_map_t *aft = gridmap_copy (gm);

    for (int iter = 0; iter < iterations; iter++) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (aft->data[y*width+x]==v)
                    continue;
                if (gridmap_has_neighbor(bef, x, y, v))
                    aft->data[y*width+x] = v;
            }
        }

        memcpy (bef->data, aft->data, aft->datalen * sizeof(*aft->data));
    }

    gridmap_destroy (aft);
}

void gridmap_convolve_centered_disc_max (grid_map_t *gm, int radius)
{
    grid_map_t *bef = gm;
    grid_map_t *aft = gridmap_copy (gm);

    for (int k = -radius; k <= radius; ++k) {
        for (int l = -radius; l <= radius; ++l) {
            // is it within radius?
            if (k*k + l*l > radius*radius)
                continue;

            int ymin = fmax (0, 0-k);
            int ymax = fmin (gm->height, gm->height-k);
            int xmin = fmax (0, 0-l);
            int xmax = fmin (gm->width, gm->width-l);

            for (int y = ymin; y < ymax; ++y) {
                int n = (y+k)*gm->width + (xmin+l);
                int o = y*gm->width + xmin;

                for (int x = xmin; x < xmax; ++x) {
                    aft->data[o] = (bef->data[n] > aft->data[o]) ? bef->data[n] : aft->data[o];
                    n++; o++;
                }
            }
        }
    }

    memcpy (bef->data, aft->data, aft->datalen * sizeof(*aft->data));
    gridmap_destroy (aft);
}

void gridmap_convolve_separable_max (grid_map_t *gm, int dim, double *filter, double padding)
{
    grid_map_t *bef = gm;
    grid_map_t *aft = gridmap_copy (gm);

    // convolve each row
    for (int row = 0; row < gm->height; ++row) {
        for (int i = 0; i < gm->width; ++i) {
            double val = 0;
            for (int j = 0; j < dim; ++j) {
                int k = i + j - floor (dim/2.0);

                if (k < 0 || k >= gm->width)
                    val = fmax (filter[j]*padding, val);
                else
                    val = fmax (filter[j]*bef->data[row*gm->width+k], val);
            }

            aft->data[row*gm->width + i] = (int8_t) val;
        }
    }
    memcpy (bef->data, aft->data, aft->datalen * sizeof(*aft->data));

    // convolve each col
    for (int col = 0; col < gm->width; ++col) {
        for (int i = 0; i < gm->height; ++i) {
            double val = 0;
            for (int j = 0; j < dim; ++j) {
                int k = i + j - floor (dim/2.0);

                if (k < 0 || k >= gm->height)
                    val = fmax (filter[j]*padding, val);
                else
                    val = fmax (filter[j]*bef->data[k*gm->width + col], val);
            }

            aft->data[i*gm->width + col] = (int8_t) val;
        }
    }

    memcpy (bef->data, aft->data, aft->datalen * sizeof(*aft->data));
    gridmap_destroy (aft);
}

static void max_convolution(uint8_t *in, int in_offset, int width, int k, uint8_t *out, int out_offset){

    int run_length = 0;
    int run_value  = 0;

    for(int x = 0; x < width; x++){
        int v = 0;
        int cnt = fmin(k, width - x);

        // if the last convolution step was all zeros, and the
        // right-most position is a zero, then we know the result
        // for this pixel will be zero.
        int right = in[in_offset + x + cnt - 1] & 0xff;

        if (right == run_value){
            run_length++;
        }
        else{
            run_length = 1;
            run_value = right;
        }

        if(run_length >= k){
            out[out_offset + x] = (uint8_t) run_value;
            continue;
        }

        //do the dumb convolution.
        for(int i = 0; i < cnt; i++){
            v = fmax(v, in[in_offset + x + i] & 0xff);
            if(v == 255){
                break;
            }
        }

        out[out_offset + x] = (uint8_t) v;
    }
}

grid_map_t * gridmap_max_convolution(const grid_map_t *gm_in, int k){


    grid_map_t *gm = gridmap_copy(gm_in);
    int width = gm_in->width;
    int height = gm_in->height;
    for(int y = 0; y < gm->height; y++){
        max_convolution(gm_in->data, y *width, width, k, gm->data, y * width);
    }

    uint8_t *tmp  = (uint8_t*)malloc(sizeof(uint8_t) * height);
    uint8_t *tmp2 = (uint8_t*)malloc(sizeof(uint8_t) * height);

    for(int x = 0; x < gm_in->width; x++){

        //copy column into 1d array for locality
        for(int y = 0; y < gm_in->height; y++){
            tmp[y] = gm->data[y * width + x];
        }
        max_convolution(tmp, 0, gm_in->height, k, tmp2, 0);

        //copy back
        for(int y = 0; y < height; y++){
            gm->data[y * width + x] = tmp2[y];
        }
    }

    free(tmp);
    free(tmp2);

    return gm;

}

grid_map_t *gridmap_decimate_max(const grid_map_t *gm, int factor)
{
    int new_width = (gm->width + factor - 1) / factor;
    int new_height = (gm->height + factor - 1) / factor + 1;

    grid_map_t *gm_out = gridmap_make_pixels(gm->x0, gm->y0, new_width,
                                             new_height,
                                             gm->meters_per_pixel * factor,
                                             0, true);
    // loop over input rows
    for (int iy = 0; iy < gm->height; iy++) {
        // which output row should this affect?
        int oy = iy/factor;

        // loop over input columns
        for (int ix = 0; ix < gm->width; ix+= factor) {
            // destination column?
            int ox = ix/factor;

            int maxv = 0;
            int maxdx = min(factor, gm->width - ox*factor);

            // loop over the input pixels that all map to (ox,oy)
            for (int dx = 0; dx < maxdx; dx++)
                maxv = max(maxv,
                           (gm->data[iy*gm->width + ox*factor + dx]&0xff));

            // update output column
            gm_out->data[oy*gm_out->width + ox] = (uint8_t) max((gm_out->data[oy*gm_out->width + ox]&0xff), maxv);
        }
    }

    return gm_out;
}

grid_map_t *gridmap_downsample_mean (const grid_map_t *gm)
{
    grid_map_t *downsampled = gridmap_make_pixels (gm->x0, gm->y0,
            gm->width/2, gm->height/2, gm->meters_per_pixel*2, 0, 0);

    double sum;
    for (int j = 0; j < downsampled->height; ++j) {
        for (int i = 0; i < downsampled->width; ++i) {
            uint8_t a = gm->data[(2*j + 0)*gm->width + (2*i + 0)];
            uint8_t b = gm->data[(2*j + 0)*gm->width + (2*i + 1)];
            uint8_t c = gm->data[(2*j + 1)*gm->width + (2*i + 0)];
            uint8_t d = gm->data[(2*j + 1)*gm->width + (2*i + 1)];

            sum = a;
            sum += b;
            sum += c;
            sum += d;

            downsampled->data[j*downsampled->width + i] = round( sum / 4.0 );
        }
    }

    return downsampled;
}

grid_map_t *gridmap_downsample_min (const grid_map_t *gm)
{
    grid_map_t *downsampled = gridmap_make_pixels (gm->x0, gm->y0,
            gm->width/2, gm->height/2, gm->meters_per_pixel*2, 0, 0);

    for (int j = 0; j < downsampled->height; ++j) {
        for (int i = 0; i < downsampled->width; ++i) {
            uint8_t a = gm->data[(2*j + 0)*gm->width + (2*i + 0)];
            uint8_t b = gm->data[(2*j + 0)*gm->width + (2*i + 1)];
            uint8_t c = gm->data[(2*j + 1)*gm->width + (2*i + 0)];
            uint8_t d = gm->data[(2*j + 1)*gm->width + (2*i + 1)];

            uint8_t minv = (a > b) ? b : a;
            minv = (minv > c) ? c : minv;
            minv = (minv > d) ? d : minv;

            downsampled->data[j*downsampled->width + i] = minv;
        }
    }

    return downsampled;
}

grid_map_t *gridmap_downsample_max (const grid_map_t *gm)
{
    grid_map_t *downsampled = gridmap_make_pixels (gm->x0, gm->y0,
            gm->width/2, gm->height/2, gm->meters_per_pixel*2, 0, 0);

    for (int j = 0; j < downsampled->height; ++j) {
        for (int i = 0; i < downsampled->width; ++i) {
            uint8_t a = gm->data[(2*j + 0)*gm->width + (2*i + 0)];
            uint8_t b = gm->data[(2*j + 0)*gm->width + (2*i + 1)];
            uint8_t c = gm->data[(2*j + 1)*gm->width + (2*i + 0)];
            uint8_t d = gm->data[(2*j + 1)*gm->width + (2*i + 1)];

            uint8_t maxv = (a < b) ? b : a;
            maxv = (maxv < c) ? c : maxv;
            maxv = (maxv < d) ? d : maxv;

            downsampled->data[j*downsampled->width + i] = maxv;
        }
    }

    return downsampled;
}

void gridmap_downsample_min_and_max (const grid_map_t *gm, grid_map_t **gmin, grid_map_t **gmax)
{
    grid_map_t *tmp_gmin = gridmap_make_pixels (gm->x0, gm->y0, gm->width/2, gm->height/2, gm->meters_per_pixel*2, 0, 0);
    grid_map_t *tmp_gmax = gridmap_make_pixels (gm->x0, gm->y0, gm->width/2, gm->height/2, gm->meters_per_pixel*2, 0, 0);

    for (int j = 0; j < tmp_gmin->height; ++j) {
        for (int i = 0; i < tmp_gmin->width; ++i) {
            uint8_t a = gm->data[(2*j + 0)*gm->width + (2*i + 0)];
            uint8_t b = gm->data[(2*j + 0)*gm->width + (2*i + 1)];
            uint8_t c = gm->data[(2*j + 1)*gm->width + (2*i + 0)];
            uint8_t d = gm->data[(2*j + 1)*gm->width + (2*i + 1)];

            uint8_t minv = (a > b) ? b : a;
            minv = (minv > c) ? c : minv;
            minv = (minv > d) ? d : minv;

            uint8_t maxv = (a < b) ? b : a;
            maxv = (maxv < c) ? c : maxv;
            maxv = (maxv < d) ? d : maxv;

            tmp_gmin->data[j*tmp_gmin->width + i] = minv;
            tmp_gmax->data[j*tmp_gmax->width + i] = maxv;
        }
    }

    *gmin = tmp_gmin;
    *gmax = tmp_gmax;
}

/*
 * Gridmap drawing functions
 */
void gridmap_draw_circle(grid_map_t *gm, double cx, double cy, double r, uint8_t fill)
{
    double pixelsPerMeter = 1.0 / gm->meters_per_pixel;

    float fcx = ((cx - gm->x0) * pixelsPerMeter);
    float fcy = ((cy - gm->y0) * pixelsPerMeter);

    // Convert relevant portion of GM to image struct
    image_u8_t im = { .width = gm->width,
                      .height = gm->height,
                      .stride = gm->width,  // Stride seems unused in grid_map_t
                      .buf = gm->data };
    image_u8_draw_circle(&im,fcx, fcy, r*pixelsPerMeter, fill);
}

void gridmap_draw_circle_max(grid_map_t *gm, double cx, double cy, double r, uint8_t fill)
{
    double pixelsPerMeter = 1.0 / gm->meters_per_pixel;

    int ix0 = (int) ((cx - r - gm->x0) * pixelsPerMeter);
    int ix1 = (int) ((cx + r - gm->x0) * pixelsPerMeter);
    int iy0 = (int) ((cy - r - gm->y0) * pixelsPerMeter);
    int iy1 = (int) ((cy + r - gm->y0) * pixelsPerMeter);

    for (int iy = iy0; iy <= iy1; iy++) {
        if (iy < 0 || iy >= gm->height)
            continue;

        for (int ix = ix0; ix <= ix1; ix++) {
            if (ix < 0 || ix >= gm->width)
                continue;

            double x = gm->x0 + (ix + .5)*gm->meters_per_pixel;
            double y = gm->y0 + (iy + .5)*gm->meters_per_pixel;

            double d2 = (x - cx)*(x-cx) + (y - cy)*(y - cy);
            if (d2 <= (r*r))
                gm->data[iy*gm->width + ix] = (uint8_t) fmax(fill & 0xff,
                                                             gm->data[iy*gm->width + ix] & 0xff);
        }
    }
}

void gridmap_draw_line(grid_map_t *gm, double xa, double ya, double xb, double yb, uint8_t fill)
{
    double dist = sqrt(sq(xb-xa) + sq(yb-ya));
    int nsteps = (int) (dist / gm->meters_per_pixel + 1);
    double pixelsPerMeter = 1.0 / gm->meters_per_pixel;

    for (int i = 0; i < nsteps; i++) {
        double alpha = ((double) i)/nsteps;
        double x = xa*alpha + xb*(1-alpha);
        double y = ya*alpha + yb*(1-alpha);

        int ix = (int) ((x - gm->x0) * pixelsPerMeter);
        int iy = (int) ((y - gm->y0) * pixelsPerMeter);

        if (ix >= 0 && ix < gm->width && iy >= 0 && iy < gm->height)
            gm->data[iy*gm->width + ix] = fill;
    }
}

void gridmap_draw_line_max(grid_map_t *gm, double xa, double ya, double xb, double yb, uint8_t fill)
{
    double dist = sqrt(sq(xb-xa) + sq(yb-ya));
    int nsteps = (int) (dist / gm->meters_per_pixel + 1);
    double pixelsPerMeter = 1.0 / gm->meters_per_pixel;

    for (int i = 0; i < nsteps; i++) {
        double alpha = ((double) i)/nsteps;
        double x = xa*alpha + xb*(1-alpha);
        double y = ya*alpha + yb*(1-alpha);

        int ix = (int) ((x - gm->x0) * pixelsPerMeter);
        int iy = (int) ((y - gm->y0) * pixelsPerMeter);

        if (ix >= 0 && ix < gm->width && iy >= 0 && iy < gm->height)
            gm->data[iy*gm->width + ix] = (uint8_t) fmax(gm->data[iy*gm->width+ix]&0xff, fill&0xff);
    }
}

void gridmap_draw_line_interpolate(grid_map_t *gm, double xa, double ya, double xb, double yb, int f0, int f1)
{
    double dist = sqrt(sq(xb-xa) + sq(yb-ya));
    int nsteps = (int) (dist / gm->meters_per_pixel + 1);
    double pixelsPerMeter = 1.0 / gm->meters_per_pixel;

    for (int i = 0; i < nsteps; i++) {
        double alpha = ((double) i)/nsteps;
        double x = xa*alpha + xb*(1-alpha);
        double y = ya*alpha + yb*(1-alpha);

        int ix = (int) ((x - gm->x0) * pixelsPerMeter);
        int iy = (int) ((y - gm->y0) * pixelsPerMeter);

        if (ix >= 0 && ix < gm->width && iy >= 0 && iy < gm->height) {
            int f = (int) (f0*alpha + f1*(1-alpha));
            gm->data[iy*gm->width + ix] = (uint8_t) f;
        }
    }
}

void gridmap_draw_rectangle(grid_map_t * gm, double cx, double cy, double x_size, double y_size, double theta, uint8_t fill)
{
    double ppm = 1.0 / gm->meters_per_pixel;

    double ux = cos(theta), uy = sin(theta);

    double x_bound = (x_size / 2.0 * fabs(ux) + y_size / 2.0 * fabs(uy));
    double y_bound = (x_size / 2.0 * fabs(uy) + y_size / 2.0 * fabs(ux));

    // lots of overdraw for high-aspect rectangles around 45 degrees.
    int ix0 = iclamp((int) ((cx - x_bound - gm->x0)*ppm), 0, gm->width - 1);
    int ix1 = iclamp((int) ((cx + x_bound - gm->x0)*ppm), 0, gm->width - 1);

    int iy0 = iclamp((int) ((cy - y_bound - gm->y0)*ppm), 0, gm->height - 1);
    int iy1 = iclamp((int) ((cy + y_bound - gm->y0)*ppm), 0, gm->height - 1);

    // Each pixel will be evaluated based on the distance to the
    // center of that pixel.
    double y = gm->y0 + (iy0+.5)*gm->meters_per_pixel;

    for (int iy = iy0; iy <= iy1; iy++) {

        double x = gm->x0 + (ix0+.5)*gm->meters_per_pixel;

        for (int ix = ix0; ix <= ix1; ix++) {

            // distances from query point to center of rectangle
            double dx = x - cx, dy = y - cy;

            // how long are the projections of the vector (dx,dy) onto the two principle
            // components of the rectangle? How much longer are they than the dimensions
            // of the rectangle?
            double c1 = fabs(dx * ux + dy * uy) - (x_size / 2);
            double c2 = fabs(- dx * uy + dy * ux) - (y_size / 2);

            // if the projection length is < 0, we're *inside* the rectangle.
            c1 = fmax(0, c1);
            c2 = fmax(0, c2);

            if(c1 == 0 && c2 == 0)
            {
                gm ->data[iy*gm->width + ix] = fill;
            }

            x += gm->meters_per_pixel;
        }

        y += gm->meters_per_pixel;
    }
}

void gridmap_max(const grid_map_t *in1, const grid_map_t *in2, grid_map_t *out)
{
    assert(in1->datalen == in2->datalen);
    for (int i = 0; i < in1->datalen; i++)
        out->data[i] = fmax(in1->data[i], in2->data[i]);
}

/* path integrals */
double gridmap_evaluate_path(const grid_map_t *gm, const double* xys, int num_points, int negativeOn255)
{
    assert(gm != NULL);
    double cost = 0;

    for (int i = 0; i < num_points - 1; i++) {
        double xy0[2], xy1[2];
        memcpy(xy0, &xys[2*i], 2*sizeof(double));
        memcpy(xy1, &xys[2*(i+1)], 2*sizeof(double));
        double thisCost = gridmap_evaluate_path_edge(gm, xy0, xy1, negativeOn255);
        if (thisCost < 0)
            return thisCost;
        cost += thisCost;
    }

    return cost;
}

double gridmap_evaluate_path_edge(const grid_map_t *gm, const double xy0[2], const double xy1[2], int negativeOn255)
{
    assert(gm != NULL);
    // we'll microstep at 0.25 pixels. this isn't exact but it's pretty darn close.
    double stepSize = gm->meters_per_pixel * 0.25;

    double dist = sqrt(sq(xy0[0]-xy1[0]) + sq(xy0[1]-xy1[1]));

    int nsteps = ((int) (dist / stepSize)) + 1;

    double cost = 0;

    for (int i = 0; i < nsteps; i++) {
        double alpha = ((double) i) / nsteps;
        double x = alpha*xy0[0] + (1-alpha)*xy1[0];
        double y = alpha*xy0[1] + (1-alpha)*xy1[1];

        int v = gridmap_get_value(gm, x, y);
        if (negativeOn255 && v==255)
            return -1;

        cost += v;
    }

    // normalize correctly for distance.
    return cost * dist / nsteps;
}

bool gridmap_are_poses_connected(const grid_map_t *gm, const double xy0[2], const double xy1[2])
{
    bool inmap0 = gridmap_is_pos_in_map(gm, xy0[0], xy0[1]);
    bool inmap1 = gridmap_is_pos_in_map(gm, xy1[0], xy1[1]);
    if (!inmap0 || !inmap1) return false;

    const int ii = gridmap_get_index_x(gm, xy0[0]);
    const int ij = gridmap_get_index_y(gm, xy0[1]);
    const int ti = gridmap_get_index_x(gm, xy1[0]);
    const int tj = gridmap_get_index_y(gm, xy1[1]);

    grid_map_t *visited = gridmap_copy(gm);
    gridmap_set_all(visited, 0x00);

    zqueue_t *zq = zqueue_create(sizeof(int[2]));
    int cel[2] = {ii, ij};
    zqueue_add_back(zq, &cel);

    while (zqueue_size(zq) > 0) {
        // pop next element in the queue
        zqueue_get(zq, 0, &cel);
        zqueue_remove_front(zq);

        // if it's the target cell, we are done
        if (cel[0] == ti && cel[1] == tj) {
            zqueue_destroy(zq);
            gridmap_destroy(visited);
            return true;
        }

        // mark it as visited
        gridmap_set_value_index(visited, cel[0], cel[1], 0x01);

        // add all unvisited neighbors (A8 connectivity, left to right, top to bottom)
        int nel[2];

        nel[0] = cel[0]-1;    nel[1] = cel[1]-1;
        if (gridmap_get_value_index_safe(gm, nel[0], nel[1], 0xff) != 0xff &&
            gridmap_get_value_index(visited, nel[0], nel[1]) == 0x00)
            zqueue_add_back(zq, &nel);

        nel[0] = cel[0]-1;    nel[1] = cel[1];
        if (gridmap_get_value_index_safe(gm, nel[0], nel[1], 0xff) != 0xff &&
            gridmap_get_value_index(visited, nel[0], nel[1]) == 0x00)
            zqueue_add_back(zq, &nel);

        nel[0] = cel[0]-1;    nel[1] = cel[1]+1;
        if (gridmap_get_value_index_safe(gm, nel[0], nel[1], 0xff) != 0xff &&
            gridmap_get_value_index(visited, nel[0], nel[1]) == 0x00)
            zqueue_add_back(zq, &nel);

        nel[0] = cel[0];      nel[1] = cel[1]-1;
        if (gridmap_get_value_index_safe(gm, nel[0], nel[1], 0xff) != 0xff &&
            gridmap_get_value_index(visited, nel[0], nel[1]) == 0x00)
            zqueue_add_back(zq, &nel);

        nel[0] = cel[0];      nel[1] = cel[1]+1;
        if (gridmap_get_value_index_safe(gm, nel[0], nel[1], 0xff) != 0xff &&
            gridmap_get_value_index(visited, nel[0], nel[1]) == 0x00)
            zqueue_add_back(zq, &nel);

        nel[0] = cel[0]+1;    nel[1] = cel[1]-1;
        if (gridmap_get_value_index_safe(gm, nel[0], nel[1], 0xff) != 0xff &&
            gridmap_get_value_index(visited, nel[0], nel[1]) == 0x00)
            zqueue_add_back(zq, &nel);

        nel[0] = cel[0]+1;    nel[1] = cel[1];
        if (gridmap_get_value_index_safe(gm, nel[0], nel[1], 0xff) != 0xff &&
            gridmap_get_value_index(visited, nel[0], nel[1]) == 0x00)
            zqueue_add_back(zq, &nel);

        nel[0] = cel[0]+1;    nel[1] = cel[1]+1;
        if (gridmap_get_value_index_safe(gm, nel[0], nel[1], 0xff) != 0xff &&
            gridmap_get_value_index(visited, nel[0], nel[1]) == 0x00)
            zqueue_add_back(zq, &nel);
    }

    zqueue_destroy(zq);
    gridmap_destroy(visited);
    return false;

}
