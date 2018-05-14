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

#ifndef _ALIGNPOINTS2D_H
#define _ALIGNPOINTS2D_H

typedef struct alignpoints2d alignpoints2d_t;
struct alignpoints2d
{
    double Sax, Say, Sbx, Sby, Saxby, Saybx, Saxbx, Sayby;
    double N;
};

alignpoints2d_t *alignpoints2d_create();
void alignpoints2d_destroy(alignpoints2d_t *align);
void alignpoints2d_add(alignpoints2d_t *align, const double *a, const double *b);
void alignpoints2d_add_weighted(alignpoints2d_t *align, const double *a, const double *b, double weight);
void alignpoints2d_compute(alignpoints2d_t *align, double *xyt);
void alignpoints2d_reset(alignpoints2d_t *align);
#endif
