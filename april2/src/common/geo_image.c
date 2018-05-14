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

#include "geo_image.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "common/matd.h"
#include "common/string_util.h"
#include "common/math_util.h"

struct tie_point
{
    double image[3]; // pixel coordinates
    double ll[3];    // lat lon el
    double xy[2];    // position in meters using linearization
};

struct geo_image
{
    image_u8x3_t * im;
    gps_lin_t * gps_lin;
    struct tie_point   tiepoints[4];
    matd_t * im2xy;
    matd_t * im2xy_inv;
};

void geo_image_destroy(geo_image_t * geo) {
    if(!geo) return;

    if(geo->im)
        image_u8x3_destroy(geo->im);
    if(geo->im2xy)
        matd_destroy(geo->im2xy);
    if(geo->im2xy_inv)
        matd_destroy(geo->im2xy_inv);
    free(geo);
}

geo_image_t * geo_image_create(const char * path, gps_lin_t * gps_lin)
{
    assert(path);
    assert(gps_lin);

    geo_image_t * geo = calloc(1, sizeof(geo_image_t));

    geo->im = image_u8x3_create_from_pnm(path);
    if(geo->im == NULL)
    {
        fprintf(stderr, "ERR: failed loading image %s\n", path);
        free(geo);
        return NULL;
    }

    char *wpath = sprintf_alloc("%sw", path);
    FILE * wfile = fopen(wpath, "r");
    if(wfile == NULL)
    {
        fprintf(stderr, "ERR failed opening geo_image %s\n",wpath);

    }
    for(int i = 0; i < 4; i++)
    {
        double elev, im3;
        if(6 != fscanf(wfile,
                       "%lf %lf %lf %lf %lf %lf",
                       &geo->tiepoints[i].image[0],
                       &geo->tiepoints[i].image[1],
                       &im3,
                       &geo->tiepoints[i].ll[1],
                       &geo->tiepoints[i].ll[0],
                       &elev))
        {
            fprintf(stderr, "ERR: failed reading line %d from %s\n", i, wpath);
            return NULL;
        }
    }

    geo_image_update_lin(geo, gps_lin);
    return geo;
}

void image2xy(geo_image_t * geo, const double ip[2], double xy[2])
{
    matd_t * imat = matd_create_data(4,1,(double []) { ip[0], ip[1], 0, 1});
    matd_t * out = matd_multiply(geo->im2xy, imat);

    xy[0] = MATD_EL(out,0,0);
    xy[1] = MATD_EL(out,1,0);

    matd_destroy(imat);
    matd_destroy(out);
}

void xy2image(geo_image_t * geo, const double xy[2], double ip[2])
{
    matd_t * imat = matd_create_data(4,1,(double []) { xy[0], xy[1], 0, 1});
    matd_t * out = matd_multiply(geo->im2xy_inv, imat);

    ip[0] = MATD_EL(out,0,0);
    ip[1] = MATD_EL(out,1,0);

    matd_destroy(imat);
    matd_destroy(out);
}


void geo_image_update_lin(geo_image_t * geo, gps_lin_t * lin)
{
    if(geo == NULL) return;

    if(lin != NULL)
        geo->gps_lin = lin;

    if (geo->gps_lin == NULL || gps_lin_get_setup_time(geo->gps_lin) == 0)
        return; //Not ready yet

    // fit a linear model that converts from pixel coordinates to meters
    // [ x ]   [ e ]   [ a b ] [ px ]
    // [ y ] = [ f ] + [ c d ] [ py ]
    //
    // (rearrange so that a,b,c,d,e,f are the unknowns, least squares solution)
    // [ x ]   [ px  py 0  0  1  0 ] [ a ]
    // [ y ] = [ 0   0  px py 0  1 ] [ b ]
    //                               [ c ]
    //                               [ d ]
    //                               [ e ]
    //                               [ f ]
    //
    // In matrix notation:
    // y = Jx

    matd_t * J = matd_create(8, 6);
    matd_t * y = matd_create(8, 1);

    for (int i = 0; i < 4; i++) {
        struct tie_point * tp = &geo->tiepoints[i];
        gps_lin_ll2xy(geo->gps_lin, tp->ll, tp->xy);


        matd_put(J,i*2 + 0, 0, tp->image[0]);
        matd_put(J,i*2 + 0, 1, tp->image[1]);
        matd_put(J,i*2 + 0, 4, 1);
        matd_put(J,i*2 + 1, 2, tp->image[0]);
        matd_put(J,i*2 + 1, 3, tp->image[1]);
        matd_put(J,i*2 + 1, 5, 1);

        matd_put(y,i*2 + 0, 0, tp->xy[0]);
        matd_put(y,i*2 + 1, 0, tp->xy[1]);
    }

    matd_t * Jt = matd_transpose(J);
    matd_t * JtJ = matd_multiply(Jt, J);
    matd_t * JtJi = matd_inverse(JtJ);
    matd_t * Jty = matd_multiply(Jt, y);
    matd_t * x = matd_multiply(JtJi, Jty);

    // I'm opting for a homogenous 2d transform
    // [ x ]   [ a b e ] [ px ]
    // [ y ] = [ c d f ] [ py ]
    // [ 1 ]   [ 0 0 1 ] [ 1  ]
    //
    matd_destroy(geo->im2xy);
    geo->im2xy = matd_create_data(4, 4, (double[]) {
        MATD_EL(x,0,0), MATD_EL(x,1,0), 0, MATD_EL(x,4,0),
        MATD_EL(x,2,0), MATD_EL(x,3,0), 0, MATD_EL(x,5,0),
        0,              0,              1,  0,
        0,              0,              0,  1});

    matd_destroy(J);
    matd_destroy(y);
    matd_destroy(Jt);
    matd_destroy(JtJ);
    matd_destroy(JtJi);
    matd_destroy(Jty);
    matd_destroy(x);

    matd_destroy(geo->im2xy_inv);
    geo->im2xy_inv = matd_inverse(geo->im2xy);

    // sanity check our projection matrix by testing the tiepoints.
    for (int i = 0; i < 4; i++) {
        struct tie_point * tp = &geo->tiepoints[i];
        double xy[2];
        image2xy(geo, tp->image, xy);
        double error = sqrt(sq(xy[0] - tp->xy[0]) + sq(xy[1] - tp->xy[1]));
        if (error > 0.00001) {
            fprintf(stderr, "WARNING: tie_point reprojection error of %15f m\n", error);
            fprintf(stderr, "[%f %f] != [%f %f]\n", xy[0], xy[1], tp->xy[0], tp->xy[1]);
        }
        double im[2];
        xy2image(geo, xy, im);
        error = sqrt(sq(im[0] - tp->image[0]) + sq(im[1] - tp->image[1]));
        if (error > 0.00001) {
            fprintf(stderr, "WARNING: tie_point reprojection error of %15f m\n", error);
            fprintf(stderr, "[%f %f] != [%f %f]\n", im[0], im[1], tp->image[0], tp->image[1]);
        }
    }
}


const image_u8x3_t * geo_image_get_image(geo_image_t * geo)
{
    return geo->im;
}

const matd_t *  geo_image_get_matrix(geo_image_t * geo)
{
    return geo->im2xy;
}

const gps_lin_t * geo_image_get_gps_lin(geo_image_t * geo)
{
    return geo->gps_lin;
}
