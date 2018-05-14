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
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "common/math_util.h"
#include "common/image_f32.h"
#include "common/zarray.h"

#include "shi_tomasi.h"

static zarray_t *convert_indexes_to_points(zarray_t *indexes, image_f32_t *im)
{
    zarray_t *points = zarray_create(2*sizeof(int32_t));
    int npoints = zarray_size(indexes);
    for (int i = 0; i < npoints; i++) {
        int32_t idx;
        zarray_get(indexes, i, &idx);

        int32_t xy[2] = {idx % im->stride, idx / im->stride};
        zarray_add(points, xy);
    }

    return points;
}

static bool in_image(image_f32_t *im, int32_t idx)
{
    int32_t x = idx % im->stride;
    if (x < 0 || x >= im->width)
        return false;

    int32_t y = idx / im->stride;
    if (y < 0 || y >= im->height)
        return false;

    return true;
}

static int32_t neighbor_search(image_f32_t *response, image_f32_t *visited,
                               int32_t idx, uint32_t max_idx, double threshold)
{
    if (in_image(response, idx) && visited->buf[idx] < 0.1) {
        visited->buf[idx] = 1.0;
        if (response->buf[idx] >= response->buf[max_idx])
            max_idx = idx;

        if (response->buf[idx] > threshold) {
            max_idx = neighbor_search(response, visited, idx + response->stride, max_idx, threshold);
            max_idx = neighbor_search(response, visited, idx - response->stride, max_idx, threshold);
            max_idx = neighbor_search(response, visited, idx + 1, max_idx, threshold);
            max_idx = neighbor_search(response, visited, idx - 1, max_idx, threshold);
        }
    }

    return max_idx;
}

static zarray_t *extract_corners(image_f32_t *response, double threshold)
{
    image_f32_t *visited = image_f32_create(response->width, response->height);
    zarray_t *corners = zarray_create(sizeof(int32_t));

    for (int y = 0; y < response->height; y++) {
        for (int x = 0; x < response->width; x++) {
            int32_t this_idx = y*response->stride + x;

            if (visited->buf[this_idx] < 0.1 &&
                response->buf[this_idx] > threshold) {
                int32_t max_idx = neighbor_search(response, visited, this_idx, this_idx, threshold);
                zarray_add(corners, &max_idx);
            }

            visited->buf[this_idx] = 1.0;
        }
    }

    image_f32_destroy(visited);

    return corners;
}

// Compute the minimum eigenvalue of the matrix [a b; b d]
static double compute_min_eig(double a, double b, double d)
{
    // 2a^2 + 4b^2 + 2d^2 - 2*(a+d)sqrt(a^2 - 2ad + d^2 + 4b^2)
    double SA = (a + d) * (a + d) * (a * a - 2 * a * d + d * d + 4 * b * b);
    assert (SA >= 0);
    SA = sqrt(SA);
    double SB = 2 * a * a + 4 * b * b + 2 * d * d - 2 * SA;
    if (SB < 0) // can be negative due to numerical precision.
        return 0.0;
    SB = sqrt(SB);
    return 0.5 * SB;
}

shi_tomasi_res_t *shi_tomasi_detect_corners(image_f32_t *im, int scale, double sigma, double threshold)
{
    int32_t width = im->width;
    int32_t stride = im->stride;
    int32_t height = im->height;

    image_f32_t *response = image_f32_create(width, height);
    image_f32_t *im_Ix2 = image_f32_create(width, height);
    image_f32_t *im_IxIy = image_f32_create(width, height);
    image_f32_t *im_Iy2 = image_f32_create(width, height);

    ///////////////////////////////////////////////
    // Compute gradients at every pixel
    for (int y = scale; y + scale < height; y++) {
        for (int x = scale; x + scale < width; x++) {
            int32_t this_idx = y*width + x;
            float east = im->buf[this_idx + scale];
            float west = im->buf[this_idx - scale];
            float north = im->buf[this_idx + scale*stride];
            float south = im->buf[this_idx - scale*stride];
            float Ix = east - west;
            float Iy = north - south;

            im_Ix2->buf[this_idx] = Ix*Ix;
            im_IxIy->buf[this_idx] = Ix*Iy;
            im_Iy2->buf[this_idx] = Iy*Iy;
        }
    }

    ///////////////////////////////////////////////
    // Convolve with gaussian. This makes it rotationally invariant.
    int k = ((int) max(3, 3*sigma)) | 1;
    image_f32_gaussian_blur(im_Ix2, sigma, k);
    image_f32_gaussian_blur(im_IxIy, sigma, k);
    image_f32_gaussian_blur(im_Iy2, sigma, k);

    ///////////////////////////////////////////////
    // Compute filter response.
    for (int y = 1; y + 1 < height; y++) {
        for (int x = 1; x + 1 < width; x++) {
            int this_idx = y*stride + x;
            double A = im_Ix2->buf[this_idx];
            double B = im_IxIy->buf[this_idx];
            double D = im_Iy2->buf[this_idx];

            // The ellipse (autocorrelation matrix) is [A B; C D]
            // = [A B; B D]. We're really interested in the
            // minimum eigenvalue, but this is slower to
            // compute. Instead, we can get some relevant
            // information by considering the determinant (product
            // of the eigenvalues) and trace (sum of the
            // eigenvalues).
            response->buf[this_idx] = compute_min_eig(A, B, D);
        }
    }

    image_f32_normalize(response);

    zarray_t *corner_indexes = extract_corners(response, threshold);
    zarray_t *corner_points = convert_indexes_to_points(corner_indexes, im);

    zarray_destroy(corner_indexes);

    //image_f32_destroy(response);
    image_f32_destroy(im_Ix2);
    image_f32_destroy(im_IxIy);
    image_f32_destroy(im_Iy2);

    shi_tomasi_res_t * res = calloc(1, sizeof(shi_tomasi_res_t));
    res->res_img = response;
    res->res_points = corner_points;
    return res;
}
void shi_tomasi_res_t_destroy(shi_tomasi_res_t * res)
{
    image_f32_destroy(res->res_img);
    zarray_destroy(res->res_points);
    free(res);
}
