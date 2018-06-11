#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <float.h>

#include "contour.h"

void extract_contours(contourExtractor_t* contourExtractor, zarray_t *points);
void clear_contours(contourExtractor_t* contourExtractor);

typedef struct join {
    int a, b;
    float distance;
}join_t;

int join_compare(const void* _a, const void* _b)
{
    join_t *a = (join_t*)_a;
    join_t *b = (join_t*)_b;
    if(a->distance > b->distance) {
        return 1;
    }
    else if(a->distance == b->distance) {
        return 0;
    }
    else {
        return -1;
    }
}

contourExtractor_t *contourExtractor_create()
{
    contourExtractor_t *contourExtractor = calloc(1, sizeof(contourExtractor_t));
    /** How many points can we skip over in order to connect contours? **/
    contourExtractor->maxSkipPoints = 3;

    /** If two points are adjacent (relative to scan order), and they
        are really close together, join them even if another point is
        closer.
    **/
    contourExtractor->adjacentAcceptDistance = 0.05;

    /** When adding a point to a contour, it's never okay to add a
     * point farther than this away.
     **/
    contourExtractor->maxDistance = 5;

    /** When starting a new contour, pretend that the last two points
     * were this far apart. This affects maxDistanceRatio, and
     * effectively limits our willingness to create contours that
     * contain only sparsely-connected points. **/
    contourExtractor->startContourMaxDistance = 0.5;
    contourExtractor->maxDistanceRatio = 2.2; // just big enough to allow for a missed return.
    contourExtractor->alwaysAcceptDistance = 0.15;

    contourExtractor->contours = zarray_create(sizeof(zarray_t*));
    contourExtractor->extract = extract_contours;
    contourExtractor->clear = clear_contours;

    contourExtractor->minPointsPerContour = 5;

    return contourExtractor;
}

void contourExtractor_destroy(contourExtractor_t* contourExtractor)
{
    if(contourExtractor) {
        contourExtractor->clear(contourExtractor);
        zarray_destroy(contourExtractor->contours);
        free(contourExtractor);
    }
}

// points: (x,y, height)
void extract_contours(contourExtractor_t* contourExtractor, zarray_t *points)
{
    if(!points)
        return;
    if(zarray_size(points) == 0)
        return;

    int npoints = zarray_size(points);
    zarray_t *contours = contourExtractor->contours;
    zarray_t *joins = zarray_create(sizeof(join_t));
    float point_i[3];
    float point_j[3];
    for (int i = 0; i < npoints; i++) {
        for (int j = i + 1; j < min(npoints, i + contourExtractor->maxSkipPoints + 2); j++) {
            zarray_get(points, i, point_i);
            zarray_get(points, j, point_j);
            float distance = floats_distance(point_i, point_j, 2);

            // If the points are adjacent and their distance is
            // less than a threshold, always accept them.
            if (i+1 == j && distance < contourExtractor->adjacentAcceptDistance)
                distance = -1;

            if (distance < contourExtractor->maxDistance) {
                join_t join;
                join.a = i;
                join.b = j;
                join.distance = distance;
                zarray_add(joins, &join);
            }
        }
    }

    // Sort joins in order of least cost to maximum cost.
    zarray_sort(joins, join_compare);

    // Perform joins

    // Who is the left/right neighbor of each point?  If no neighbor, -1.
    int *left = calloc(npoints, sizeof(int));
    int *right = calloc(npoints, sizeof(int));

    for (int i = 0; i < npoints; i++) {
        left[i] = -1;
        right[i] = -1;
    }

    for (int joinidx = 0; joinidx < zarray_size(joins); joinidx++) {
        join_t join;
        zarray_get(joins, joinidx, &join);

        // Can't join two points that already have different neighbors.
        if (right[join.a] >=0 || left[join.b] >= 0)
            continue;

        // If the left or right point is already part of a
        // contour, what is the distance between the most recently
        // added point?  If the distance jumps suddenly, we may
        // not want to connect to this contour.
        float lastDistance = FLT_MAX;
        if (left[join.a] >= 0) {
            float point_a[3];
            float point_b[3];
            zarray_get(points, join.a, point_a);
            zarray_get(points, left[join.a], point_b);
            lastDistance = min(lastDistance, floats_distance(point_a, point_b, 2));

        }
        if (right[join.b] >= 0) {
            float point_a[3];
            float point_b[3];
            zarray_get(points, join.b, point_b);
            zarray_get(points, right[join.b], point_a);
            lastDistance = min(lastDistance, floats_distance(point_a, point_b, 2));

        }
        if (lastDistance == FLT_MAX) {
            lastDistance = contourExtractor->startContourMaxDistance;
        }

        double distanceRatio = join.distance / lastDistance;
        if (distanceRatio > contourExtractor->maxDistanceRatio && join.distance > contourExtractor->alwaysAcceptDistance)
            continue;

        // join the points.
        right[join.a] = join.b;
        left[join.b] = join.a;
    }


    for (int root = 0; root < npoints; root++) {
        // is this isn't left-most point in a chain, we've already extracted this contour.
        if (left[root] >=0)
            continue;

        zarray_t *contour = zarray_create(sizeof(float[3]));
        for (int child = root; child >= 0; child = right[child]) {
            float point[3];
            zarray_get(points, child, point);
            zarray_add(contour, point);
        }

        if (zarray_size(contour) >= contourExtractor->minPointsPerContour) {
            zarray_add(contours, &contour);
        } else {
            zarray_destroy(contour);
        }
    }

    zarray_destroy(joins);
    free(left);
    free(right);
}

void clear_contours(contourExtractor_t* contourExtractor)
{
    for(int i = 0; i < zarray_size(contourExtractor->contours); i++) {
        zarray_t *contour;
        zarray_get(contourExtractor->contours, i, &contour);
        zarray_destroy(contour);
    }
    zarray_clear(contourExtractor->contours);
}
