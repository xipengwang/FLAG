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
