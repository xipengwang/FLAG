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

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <float.h>

#include "linefitter.h"


void extract_lines(lineFitter_t* lineFitter, zarray_t *contours);
void clear_lines(lineFitter_t* lineFitter);

lineFitter_t *lineFitter_create()
{
    lineFitter_t *lineFitter = calloc(1, sizeof(lineFitter_t));
    // mean distance^2 (in meters) from the fit line
    lineFitter->errorThresh = pow(0.03, 2);

    // minimum number of points in the line
    lineFitter->minPoints = 4;

    // maximum gap (in meters) between any two points
    lineFitter->maxSpan = 1.5;

    lineFitter->lineFeatures = zarray_create(sizeof(lineFeature_t));
    lineFitter->extract = extract_lines;
    lineFitter->clear = clear_lines;
    return lineFitter;
}

void lineFitter_destroy(lineFitter_t *lineFitter)
{
    if(lineFitter) {
        lineFitter->clear(lineFitter);
        zarray_destroy(lineFitter->lineFeatures);
        free(lineFitter);
    }
}

line2D_t create_line2D(float dx, float dy, float p[2])
{
    line2D_t line2D = {0};
    line2D.dx = dx;
    line2D.dy = dy;
    //a point it passes though
    line2D.p[0] = p[0];
    line2D.p[1] = p[1];
    return line2D;
}

void normalize_slope(line2D_t* line2D)
{
    //normalize slope
    double mag=sqrt(line2D->dx*line2D->dx+line2D->dy*line2D->dy);
    line2D->dx/=mag;
    line2D->dy/=mag;
}
void normlize_P(line2D_t* line2D)
{
    //normalize P
    // we already have a point (P) on the line, and we know the line vector U and it's perpendicular vector V:
    // so, P'=P.*V *V
    double dotprod=-line2D->dy*line2D->p[0] + line2D->dx*line2D->p[1];
    line2D->p[0] = -line2D->dy*dotprod;
    line2D->p[1] = line2D->dx*dotprod;
}
void set_p1(fitterSegment_t *S, float pin[3])
{
    line2D_t *line2D = &(S->line2D);
    double dotprod=pin[0]*line2D->dx + pin[1]*line2D->dy;
    S->p1[0] = line2D->p[0]+line2D->dx*dotprod;
    S->p1[1] = line2D->p[1]+line2D->dy*dotprod;
}
void set_p2(fitterSegment_t *S, float pin[3])
{
    line2D_t *line2D = &(S->line2D);
    double dotprod=pin[0]*line2D->dx + pin[1]*line2D->dy;
    S->p2[0] = line2D->p[0]+line2D->dx*dotprod;
    S->p2[1] = line2D->p[1]+line2D->dy*dotprod;
}

int intersectionWith(line2D_t *la, line2D_t *lb, float inter[2])
{
    double m00, m01, m10, m11;
    double i00, i01;
    //double i10, i11;
    double b00, b10;

    m00=la->dx;
    m01=-lb->dx;
    m10=la->dy;
    m11=-lb->dy;

    // determinant of m
    double det=m00*m11-m01*m10;

    // parallel lines?
    if (fabs(det)<0.0000000001){
        return 0;
    }

    // inverse of m
    i00=m11/det;
    //i11=m00/det;
    i01=-m01/det;
    //i10=-m10/det;

    b00=lb->p[0] - la->p[0];
    b10=lb->p[1] - la->p[1];

    double x00;
    //double x10;
    x00=i00*b00+i01*b10;
    //	x10=i10*b00+i11*b10;

    inter[0] = la->dx*x00+la->p[0];
    inter[1] = la->dy*x00+la->p[1];

    return 1;
}


void fitLine(fitterSegment_t *S, zarray_t *points)
{
    // estimate the new line
    double Cxx,Cyy,Cxy,Ex,Ey;

    // compute covariances and expectations
    Cxx = S->mXX/S->nPoints - pow(S->mX/S->nPoints,2);
    Cyy = S->mYY/S->nPoints - pow(S->mY/S->nPoints,2);
    Cxy = S->mXY/S->nPoints - (S->mX/S->nPoints)*(S->mY/S->nPoints);
    Ex  = S->mX/S->nPoints;
    Ey  = S->mY/S->nPoints;

    // find dominant direction via SVD
    if(1) {
        double phi=0.5*atan2(-2*Cxy,(Cyy-Cxx));
        //double rho=Ex*cos(phi) + Ey*sin(phi);
        // compute line parameters
        S->line2D = create_line2D(-sin(phi), cos(phi), (float[2]){Ex, Ey});
    } else {
        //This seems wrong from old Java code
        double theta = 0.5*atan2(Cxy, (Cxx-Cyy));
        S->line2D = create_line2D(cos(theta), sin(theta), (float[2]){Ex, Ey});
    }

    // compute the error
    S->error=0;
    line2D_t *line2D = &(S->line2D);
    normalize_slope(line2D);
    normlize_P(line2D);

    float pointLow[3];
    zarray_get(points, S->pLo, pointLow);
    float pointHigh[3];
    zarray_get(points, S->pHi, pointHigh);
    set_p1(S, pointLow);
    set_p2(S, pointHigh);

    for (int idx=S->pLo;idx<=S->pHi;idx++) {
        float p[3];
        zarray_get(points, idx, p);
        line2D_t radialLine = create_line2D(p[0], p[1], p);
        //line2D_t radialLine = create_line2D(0, 1, p);
        float inter[2];
        if(!intersectionWith(&radialLine, line2D, inter)) {
            S->error = FLT_MAX;
            break;
        } else {
            float d=floats_distance(inter, p, 2);
            S->error += d*d;
        }
    }
    S->error /= S->nPoints;
}

fitterSegment_t *fitterSegment_create(zarray_t *points, int i, int j)
{
    fitterSegment_t *fitterSegment = calloc(1, sizeof(fitterSegment_t));
    float a[3];
    float b[3];
    zarray_get(points, i, a);
    zarray_get(points, j, b);
    for (int idx=i; idx<=j; idx++)
    {
        float p[3];
        zarray_get(points, idx, p);
        //printf("p %f,%f\n", p[0], p[1]);
        fitterSegment->mX+=p[0];
        fitterSegment->mY+=p[1];
        fitterSegment->mXX+=p[0]*p[0];
        fitterSegment->mYY+=p[1]*p[1];
        fitterSegment->mXY+=p[0]*p[1];
    }
    fitterSegment->pLo=i;
    fitterSegment->pHi=j;
    fitterSegment->nPoints=j-i+1;
    fitterSegment->deleted = false;
    fitterSegment->leftJoin = NULL;
    fitterSegment->rightJoin = NULL;

    fitLine(fitterSegment, points);
    return fitterSegment;
}

fitterSegment_t* joinSegments(lineFitter_t *lineFitter, zarray_t *points,
                              fitterSegment_t *seg1, fitterSegment_t *seg2)
{
    fitterSegment_t *outseg = calloc(1, sizeof(fitterSegment_t));
    outseg->mX=seg1->mX+seg2->mX;
    outseg->mXX=seg1->mXX+seg2->mXX;
    outseg->mY=seg1->mY+seg2->mY;
    outseg->mYY=seg1->mYY+seg2->mYY;
    outseg->mXY=seg1->mXY+seg2->mXY;

    // correct our statistics by removing any extra copies of
    // points we might have due to the segments overlapping.
    // we reorder the points so that we have:
    //
    // small indexes --------> high indexes
    //
    // a------b        or     a------b
    //     c------d                     c------d
    //
    int a, b, c, d;
    if (seg1->pLo<seg2->pLo) {
        a=seg1->pLo;
        b=seg1->pHi;
        c=seg2->pLo;
        d=seg2->pHi;
    } else {
        a=seg2->pLo;
        b=seg2->pHi;
        c=seg1->pLo;
        d=seg1->pHi;
    }

    assert(a<=b);
    assert(c<=d);
    assert(a<=d);
    assert(a<=c);
    assert(b<=d);

    // remove twice-counted nodes
    int duplicatepoints=0;
    for (int idx=c;idx<=b;idx++) {
        assert(b==c);
        duplicatepoints++;

        float p[3];
        zarray_get(points, idx, p);

        outseg->mX-=p[0];
        outseg->mY-=p[1];
        outseg->mXX-=p[0]*p[0];
        outseg->mYY-=p[1]*p[1];
        outseg->mXY-=p[0]*p[1];
    }

    outseg->pLo=a;
    outseg->pHi=d;
    outseg->nPoints=seg1->nPoints+seg2->nPoints-duplicatepoints; //d-a+1;
    fitLine(outseg, points);

    // check for excessive join length
    // new edges are b -> max(b+1,c)
    int e=b;
    int f=max(b+1,c);

    float pe[3];
    float pf[3];
    zarray_get(points, e, pe);
    zarray_get(points, f, pf);
    if (floats_distance(pe, pf,2) > lineFitter->maxSpan) {
        outseg->error = FLT_MAX;
    }
    return outseg;
}

join_t *join_create(lineFitter_t* lineFitter, zarray_t *points, fitterSegment_t *lS, fitterSegment_t *rS)
{
    join_t *join = calloc(1, sizeof(join_t));
    join->leftSeg = lS;
    join->rightSeg = rS;
    join->leftrightSeg = joinSegments(lineFitter, points, lS, rS);
    join->deleted = false;
    return join;
}

float computeNormal(float p0[2], float p1[2])
{
    float dx = p0[0] - p1[0];
    float dy = p0[1] - p1[1];

    // compute angle of line segment perpendicular to the line OP-P
    float theta = atan2(-dx, dy);

    // make sure we get the angle that points towards the laser scanner
    // (pick the possibility that is closest to the projection through the origin)
    float dumbtheta = atan2(-p0[1], -p0[0]);
    float err = fabs(mod2pi(dumbtheta - theta));
    if (err > M_PI/2)
        theta += M_PI;

    theta = mod2pi(theta);
    return theta;
}

void extract_lines(lineFitter_t* lineFitter, zarray_t *contours)
{
    zarray_t *segments = zarray_create(sizeof(fitterSegment_t*));
    zmaxheap_t *joins = zmaxheap_create(sizeof(join_t*));
    for(int contour_idx = 0; contour_idx < zarray_size(contours); contour_idx++) {
        zarray_t *points;
        zarray_get(contours, contour_idx, &points);
        int npoints = zarray_size(points);
        for (int i = 0; i < npoints-1; i++) {
            float a[3];
            float b[3];
            zarray_get(points, i, a);
            zarray_get(points, i+1, b);
            if (floats_distance(a, b, 2) > lineFitter->maxSpan)
                continue;
            fitterSegment_t *fitterSegment = fitterSegment_create(points, i, i+1);
            fitterSegment->id = zarray_size(segments);
            //printf("dxy: %f, %f \n", fitterSegment->line2D.dx, fitterSegment->line2D.dy);
            //printf("error %f, %d \n", fitterSegment->error, fitterSegment->nPoints);
            zarray_add(segments, &fitterSegment);
        }
        for (int i = 0; i < zarray_size(segments); i++) {
            fitterSegment_t *segment;
            zarray_get(segments, i, &segment);
        }

        // create the joins
        for (int i = 0; i < zarray_size(segments) - 1; i++) {
            fitterSegment_t *leftSegment;
            fitterSegment_t *rightSegment;
            zarray_get(segments, i, &leftSegment);
            zarray_get(segments, i+1, &rightSegment);
            join_t *j = join_create(lineFitter, points, leftSegment, rightSegment);
            //we want first get segment with smallest error
            zmaxheap_add(joins, &j, -j->leftrightSeg->error);
            leftSegment->rightJoin = j;
            rightSegment->leftJoin = j;
        }

        // now join segments until we reach our error threshold.
        while(zmaxheap_size(joins) > 0) {
            join_t *j;
            zmaxheap_remove_max(joins, &j, NULL);
            if(j->deleted) {
                free(j->leftrightSeg);
                free(j);
                continue;
            }
            // if this join has too great error, we're done.
            if (j->leftrightSeg->error > lineFitter->errorThresh) {
                //Clear memory taken by joins later
                //A join that is not freed must be pointed by a join pointer in a segment.
                free(j->leftrightSeg);
                free(j);
                break;
            }
            // otherwise, this join is a keeper. We'll take
            // j.leftrightSeg (which is already the union of the left
            // and right segments) and create two new joins connecting
            // it to its neighbors.
            join_t *leftJoin = j->leftSeg->leftJoin;
            join_t *rightJoin = j->rightSeg->rightJoin;

            //put leftrightSeg pointer into segments at id position
            //Then the union left-right-seg becomes the left seg of this join.
            j->leftrightSeg->id = j->leftSeg->id;
            zarray_set(segments, j->leftSeg->id, &(j->leftrightSeg), NULL);
            //remove j;
            //The left seg is replaced by the leftrightSeg
            free(j->leftSeg);
            j->rightSeg->deleted = true;

            if(leftJoin != NULL) {
                leftJoin->deleted = true;
                join_t *newj = join_create(lineFitter, points,
                                           leftJoin->leftSeg, j->leftrightSeg);
                leftJoin->leftSeg->rightJoin = newj;
                j->leftrightSeg->leftJoin = newj;
                zmaxheap_add(joins, &newj, -newj->leftrightSeg->error);
            }

            if(rightJoin != NULL) {
                rightJoin->deleted = true;
                join_t *newj = join_create(lineFitter, points,
                                           j->leftrightSeg, rightJoin->rightSeg);
                rightJoin->rightSeg->leftJoin = newj;
                j->leftrightSeg->rightJoin = newj;
                zmaxheap_add(joins, &newj, -newj->leftrightSeg->error);
            }
            free(j);
        }

        /* for (int i = 0; i < zarray_size(segments); i++) { */
        /*     fitterSegment_t *segment; */
        /*     zarray_get(segments, i, &segment); */
        /*     printf("segment id: %d, i:%d \n", segment->id, i); */
        /* } */
        /* printf("Size:%d \n", zarray_size(segments)); */
        //reject bad segments
        for (int i = 0; i < zarray_size(segments); i++) {
            fitterSegment_t *segment;
            zarray_get(segments, i, &segment);
            if(!segment->deleted && segment->nPoints >= lineFitter->minPoints) {
                //generate line features
                lineFeature_t lineFeature;
                lineFeature.npoints = segment->nPoints;
                lineFeature.p1[0] = segment->p1[0];
                lineFeature.p1[1] = segment->p1[1];
                lineFeature.p2[0] = segment->p2[0];
                lineFeature.p2[1] = segment->p2[1];
                memcpy(&(lineFeature.line2D), &(segment->line2D), sizeof(line2D_t));
                lineFeature.normal = computeNormal(segment->p1, segment->p2);
                zarray_add(lineFitter->lineFeatures, &lineFeature);
            }
            assert(segment->id == i);
            free(segment);
        }
        zarray_clear(segments);
        //release memory of rest of joins
        while(zmaxheap_size(joins) > 0) {
            join_t *j;
            zmaxheap_remove_max(joins, &j, NULL);
            free(j->leftrightSeg);
            free(j);
        }
    }
    zarray_destroy(segments);
    zmaxheap_destroy(joins);
}

void clear_lines(lineFitter_t* lineFitter)
{
    zarray_clear(lineFitter->lineFeatures);
}
