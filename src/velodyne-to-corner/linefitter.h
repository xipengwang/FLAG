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

#include<stdbool.h>

#include "common/zarray.h"
#include "common/zmaxheap.h"
#include "common/floats.h"


typedef struct line2D {
    //slope
    float dx, dy;
    //a point it passes though
    float p[2];
}line2D_t;

typedef struct lineFeature lineFeature_t;
//2D
struct lineFeature {
    line2D_t line2D;

    //start/end points;
    float p1[2];
    float p2[2];
    int npoints;

    //Line normal in angle(radian);
    float normal;
};


typedef struct lineFitter lineFitter_t;
//2D
struct lineFitter {
    // mean distance^2 (in meters) from the fit line
    double errorThresh;

    // minimum number of points in the line
    int minPoints;

    // maximum gap (in meters) between any two points
    double maxSpan;

    /** Given a set of points that are radially ordered and physically
     * proximate, (i.e., contours), fit lines through consecutive
     * points.
     *
     * For normal estimation to work correctly, points should be
     * observed from approximately the origin.
     **/

    void (*extract)(lineFitter_t* lineFitter, zarray_t* contours);
    void (*clear)(lineFitter_t* lineFitter);
    zarray_t* lineFeatures; //lineFeature_t
};

typedef struct fitterSegment fitterSegment_t;
typedef struct join join_t;

struct fitterSegment {
    line2D_t line2D;
    float p1[2];
    float p2[2];
    int nPoints;

    // moments
    double mX, mY, mXX, mYY, mXY;

    // average error
    double error;

    // Index into the PointSet where to find these points.
    int pLo, pHi;

    // used by segment internally
    join_t *leftJoin, *rightJoin;

    // set when this linesegment is joined
    bool deleted;
    int id;
};

struct join {
    fitterSegment_t *leftSeg, *rightSeg;
    fitterSegment_t *leftrightSeg;
    bool deleted;
};


lineFitter_t *lineFitter_create();
void lineFitter_destroy(lineFitter_t *lineFitter);
int intersectionWith(line2D_t *la, line2D_t *lb, float inter[2]);
