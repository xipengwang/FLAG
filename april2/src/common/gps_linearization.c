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

#include "gps_linearization.h"

#include "doubles.h"
#include "math_util.h"

// Data retrieved from http://en.wikipedia.org/wiki/Earth_radius on 6-27-15
static const double RAD_EQ_M = 6378137.0; // RADIUS_EQUATOR_METERS
static const double RAD_PO_M = 6356752.3; // RADIUS_POLAR_METERS

// indices
static const int LAT = 0;
static const int LON = 1;
static const int X = 0;
static const int Y = 1;

struct gps_lin
{
    int64_t setup_time;
    double lat0_deg, lon0_deg;
    double lat0_rad, lon0_rad;
    double radius_ns, radius_ew;

    double xyt[3];
};


/** Associate the GPS coordinate system at origin_deg facing north with
    the Euclidean coordinate system _xyt. **/
gps_lin_t * gps_lin_create()
{
    gps_lin_t * gps = calloc(1, sizeof(gps_lin_t));
    gps->lat0_deg = NAN;
    gps->lon0_deg = NAN;
    return gps;
}

void gps_lin_setup(gps_lin_t * gps, const double origin_deg[], double _xyt[], int64_t setup_time)
{
    if(gps->setup_time == setup_time) return;

    gps->setup_time = setup_time;
    gps->xyt[0] = _xyt[0];
    gps->xyt[1] = _xyt[1];
    gps->xyt[2] = _xyt[2];

    gps->lat0_deg = origin_deg[LAT];
    gps->lon0_deg = origin_deg[LON];

    const double a = RAD_EQ_M;
    const double b = RAD_PO_M;

    gps->lat0_rad = to_radians(gps->lat0_deg);
    gps->lon0_rad = to_radians(gps->lon0_deg);

    //Aproximating the two radii at this location:
    gps->radius_ns = sq(a*b)/pow(sq(a*cos(gps->lat0_rad)) +
                                 sq(b*sin(gps->lat0_rad)),1.5);

    gps->radius_ew = a*a / sqrt(sq(a*cos(gps->lat0_rad)) +
                                sq(b*sin(gps->lat0_rad)));

}

int64_t gps_lin_get_setup_time(gps_lin_t *gps)
{
    return gps->setup_time;
}

void gps_lin_destroy(gps_lin_t *gps)
{
    free(gps);
}

void gps_lin_get_origin_latlon(gps_lin_t *gps, double * ll)
{
    ll[LAT] = gps->lat0_deg;
    ll[LON] = gps->lon0_deg;
}

void gps_lin_ll2xy(const gps_lin_t * gps, const double latlon[2], double xy[2])
{
    const double latlon_rad [] = {
        to_radians(latlon[LAT]),
        to_radians(latlon[LON]) };
    const double d_rad[] = { latlon_rad[LAT] - gps->lat0_rad,
                             latlon_rad[LON] - gps->lon0_rad};
    const double result[] = {
        sin(d_rad[LON]) * gps->radius_ew * cos(gps->lat0_rad),
        sin(d_rad[LAT]) * gps->radius_ns };

    doubles_xyt_transform_xy(gps->xyt, result, xy);
}

void gps_lin_xy2ll(const gps_lin_t * gps, const double _xy[2], double ll[2])
{

    double xyt_inv[3];
    doubles_xyt_inv(gps->xyt, xyt_inv);
    double xy[2];
    doubles_xyt_transform_xy(xyt_inv, _xy, xy);
    double dlat_rad =
        asin(xy[Y] / gps->radius_ns);
    double dlon_rad =
        asin( (xy[X] / gps->radius_ew ) / cos(gps->lat0_rad) );
    ll[0] = to_degrees(dlat_rad + gps->lat0_rad);
    ll[1] = to_degrees(dlon_rad + gps->lon0_rad);
}

/** Old java serialization code, for future reference
void read(StructureReader ins) throws IOException
{
    double ll_deg[] = ins.readDoubles();
    double r[] = ins.readDoubles();
    this.xyt = ins.readDoubles();

    gps->lat0_deg = ll_deg[0];
    gps->lon0_deg = ll_deg[1];
    gps->lat0_rad = to_radians(gps->lat0_deg);
    gps->lon0_rad = to_radians(gps->lon0_deg);
    gps->radius_ns = r[0];
    gps->radius_ew = r[1];
}

void write(StructureWriter outs) throws IOException
{
    outs.writeDoubles(new double[] { gps->lat0_deg, gps->lon0_deg });
    outs.writeDoubles(new double[] { gps->radius_ns, gps->radius_ew });
    outs.writeDoubles(xyt);
}
 */
