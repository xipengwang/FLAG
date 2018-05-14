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

#ifndef _GPS_LINEARIZATION_H
#define _GPS_LINEARIZATION_H

#include <stdint.h>

typedef struct gps_lin gps_lin_t;

gps_lin_t * gps_lin_create();
void gps_lin_free(gps_lin_t *gps);

void gps_lin_setup(gps_lin_t * gps, const double origin_deg[], double _xyt[], int64_t setup_time);
int64_t gps_lin_get_setup_time(gps_lin_t *gps);

void gps_lin_get_origin_latlon(gps_lin_t *gps, double * ll);

void gps_lin_ll2xy(const gps_lin_t * gps, const double latlon[], double xy[]);

void gps_lin_xy2ll(const gps_lin_t * gps, const double _xy[], double ll[]);

void gps_lin_destroy(gps_lin_t *gps);

#endif //_GPS_LINEARIZATION_H
