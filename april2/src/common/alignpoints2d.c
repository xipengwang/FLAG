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

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "alignpoints2d.h"

// compute the rigid-body transformation xyt that projects points as onto bs.
alignpoints2d_t *alignpoints2d_create()
{
    alignpoints2d_t *align = calloc(1, sizeof(alignpoints2d_t));

    return align;
}

void alignpoints2d_destroy(alignpoints2d_t *align)
{
    free(align);
}

void alignpoints2d_add(alignpoints2d_t *align, const double *a, const double *b)
{
    align->Sax += a[0];
    align->Say += a[1];
    align->Sbx += b[0];
    align->Sby += b[1];
    align->Saxby += a[0]*b[1];
    align->Saybx += a[1]*b[0];
    align->Saxbx += a[0]*b[0];
    align->Sayby += a[1]*b[1];

    align->N++;
}

void alignpoints2d_add_weighted(alignpoints2d_t *align, const double *a, const double *b, double weight)
{
    align->Sax += a[0]*weight;
    align->Say += a[1]*weight;
    align->Sbx += b[0]*weight;
    align->Sby += b[1]*weight;

    align->Saxby += a[0]*b[1]*weight;
    align->Saybx += a[1]*b[0]*weight;
    align->Saxbx += a[0]*b[0]*weight;
    align->Sayby += a[1]*b[1]*weight;

    align->N += weight;
}

void alignpoints2d_compute(alignpoints2d_t *align, double *xyt)
{
    double N = align->N;

    double axc = align->Sax / N;
    double ayc = align->Say / N;
    double bxc = align->Sbx / N;
    double byc = align->Sby / N;

    double M0 = align->Saxby - align->Sax*byc - align->Sby*axc + axc*byc*N -
        align->Saybx + align->Say*bxc + align->Sbx*ayc - ayc*bxc*N;
    double N0 = align->Saxbx - align->Sax*bxc - align->Sbx*axc + axc*bxc*N +
        align->Sayby - align->Say*byc - align->Sby*ayc + ayc*byc*N;

    double theta = atan2(M0, N0);

    double c = cos(theta), s = sin(theta);
    double dx = (bxc-axc) - c*axc + s*ayc + axc;
    double dy = (byc-ayc) - s*axc - c*ayc + ayc;

    xyt[0] = dx;
    xyt[1] = dy;
    xyt[2] = theta;
}

void alignpoints2d_reset(alignpoints2d_t *align)
{
    memset(align, 0, sizeof(alignpoints2d_t));
}
