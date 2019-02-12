/* Copyright (C) 2013-2019, The Regents of The University of Michigan.
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

/*$LICENSE*/

#pragma once

#include "common/zarray.h"
#include "common/floats.h"

typedef struct contourExtractor contourExtractor_t;
struct contourExtractor {
    /** How many points can we skip over in order to connect contours? **/
    int maxSkipPoints;
    /** If two points are adjacent (relative to scan order), and they
        are really close together, join them even if another point is
        closer.
    **/
    double adjacentAcceptDistance;

    /** When adding a point to a contour, it's never okay to add a
     * point farther than this away.
     **/
    double maxDistance;

    /** When starting a new contour, pretend that the last two points
     * were this far apart. This affects maxDistanceRatio, and
     * effectively limits our willingness to create contours that
     * contain only sparsely-connected points. **/
    double startContourMaxDistance;

    int minPointsPerContour;

    double maxDistanceRatio; // just big enough to allow for a missed return.
    double alwaysAcceptDistance;

    void (*extract)(contourExtractor_t* contourExtractor, zarray_t *points);
    void (*clear)(contourExtractor_t* contourExtractor);
    zarray_t* contours;

};

contourExtractor_t *contourExtractor_create();
void contourExtractor_destroy(contourExtractor_t *contourExtractor);
