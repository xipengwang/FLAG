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

#ifndef _GEO_IMAGE_H
#define _GEO_IMAGE_H
/** A geo-referenced image. It is specified by two files: an image
 * file (e.g., "foo.png") and a tiepoints file (e.g., "foo.pngw").
 * The tiepoints file contains data exactly equivalent to that found
 * in a GeoTIFF: a list of tiepoints. Each tiepoint is listed on a
 * line by itself, and has six fields:
 *
 * pixelx pixely pixelz longitude latitude altitude
 *
 * The pixelz and altitude fields are ignored and are typically
 * zero. GPS positions correspond to pixel *centers*.
 *
 * How to create your own GeoImage:
 *   Download and extract the worldwind jars from NASA:
 *   http://builds.worldwind.arc.nasa.gov/download-release.asp
 *   a) Add the jars to your classpath, in your .bashrc

 * Download the NASA Worldwind applet for saving geotiff images:
 *   mkdir -p ~/worldwind
 *   cd ~/worldwind
 *   wget http://worldwind.arc.nasa.gov/java/demos/worldwind.jar
 *   wget http://worldwind.arc.nasa.gov/java/demos/worldwindx.jar
 *   sudo apt-get install libjogl-java libjogl-jni
 * Now add the appropriate CLASSPATH entry to your .bashrc:
 *   export CLASSPATH=/usr/share/java/gluegen-rt.jar:/usr/share/java/jogl.jar:$CLASSPATH
 *   export CLASSPATH=$HOME/worldwind/worldwind.jar:$HOME/worldwind/worldwindx.jar:$CLASSPATH
 * Open a new terminal, and do:
 *   java gov.nasa.worldwindx.examples.ExportImageOrElevations  *   java worldwind
 *  1) Select and save the area you are trying to grab. Be sure to cycle through the layers
 *     to find the best imagery (MS is usually good) for your location
 *  2) convert foo.tif foo.png  (this may complain about some unknown fields, but ignore)
 *  3) java april.util.TIFF foo.tif | tail -n 4 > foo.pngw
 *
 * now pass the path for foo.png to an instance of this class
 **/

#include "common/gps_linearization.h"
#include "common/image_u8x3.h"
#include "common/matd.h"

typedef struct geo_image geo_image_t;

geo_image_t * geo_image_create(const char * path, gps_lin_t * gps_lin);

void geo_image_update_lin(geo_image_t * geo, gps_lin_t * lin);

const image_u8x3_t * geo_image_get_image(geo_image_t * geo);

const matd_t *  geo_image_get_matrix(geo_image_t * geo);

const gps_lin_t * geo_image_get_gps_lin();

void geo_image_get_xyt(const geo_image_t * geo, double *xyt);

void geo_image_destroy(geo_image_t * geo);

#endif //_GEO_IMAGE_H
