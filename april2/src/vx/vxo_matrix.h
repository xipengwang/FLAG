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

#ifndef _VXO_MATRIX_H
#define _VXO_MATRIX_H

// a vx object must incref any resources it uses.
vx_object_t *vxo_matrix(const double d[16]);
vx_object_t *vxo_matrix_scale(double s);
vx_object_t *vxo_matrix_xyt(const double xyt[3]);
vx_object_t *vxo_matrix_scale3(double sx, double sy, double sz);
vx_object_t *vxo_matrix_translate(double tx, double ty, double tz);
vx_object_t *vxo_matrix_rotatex(double rad);
vx_object_t *vxo_matrix_rotatey(double rad);
vx_object_t *vxo_matrix_rotatez(double rad);
vx_object_t *vxo_matrix_quat(double q[4]);
vx_object_t *vxo_matrix_quat_xyz(double q[4], double xyz[3]);
#endif
