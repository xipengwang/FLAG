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

#include "sim_triray.h"

#include "common/floats.h"
#include "common/doubles.h"
#include "common/matd.h"

#include "common/mesh_model.h"

struct triangle
{
    float p[3][3];
    float n[3];
    float n0; // n DOT p[0]

    // matrix transformation that projects points relative to p0 into
    // "triangle" coordinates... i.e., where p0 is at (0,0), p1 is at
    // (1,0) and p2 is at (0,1)
    float T[3][3];
};

static int triangle_contains_point(struct triangle *tri, const double q[3])
{
    double Tq[3] = { 0, 0, 0 };;

    // XXX unroll, do each test right after computing it.
    for (int i = 0; i < 3; i++)
        Tq[i] = tri->T[i][0] * (q[0] - tri->p[0][0]) +
            tri->T[i][1] * (q[1] - tri->p[0][1]) +
            tri->T[i][2] * (q[2] - tri->p[0][2]);

    return (Tq[0] >= 0) && (Tq[1] >= 0) &&
        (Tq[1] <= 1 - Tq[0]) && (Tq[0] <= 1);

}

static double triangle_ray_distance(struct triangle *tri, const double r[3], const double d[3], double max)
{
    // first, compute the point along the ray that lies within the plane of the triangle.

    //  (r + lambda * d) DOT tri.n = tri.n0
    // (lambda * d) DOT tri.n = tri.n0 - (r DOT tri.n);

    double nd[3] = { tri->n[0], tri->n[1], tri->n[2] };
    double lambda = (tri->n0 - doubles_dot(r, nd, 3)) / doubles_dot(d, nd, 3);
    if (lambda < 0)
        return max;

    if (lambda > max)
        return max;

    // the candidate point is r + lambda * d

    double q[3];
    for (int i = 0; i < 3; i++)
        q[i] = r[i] + d[i] * lambda;

    if (triangle_contains_point(tri, q))
        return lambda;

    return max;
}

static void triangle_init(struct triangle *tri, const float p0[3], const float p1[3], const float p2[3])
{
    memcpy(tri->p[0], p0, 3 * sizeof(float));
    memcpy(tri->p[1], p1, 3 * sizeof(float));
    memcpy(tri->p[2], p2, 3 * sizeof(float));

    float v0[3], v1[3];
    for (int i = 0; i < 3; i++) {
        v0[i] = tri->p[1][i] - tri->p[0][i];
        v1[i] = tri->p[2][i] - tri->p[0][i];
    }

    floats_cross_product(v0, v1, tri->n);
    floats_normalize(tri->n, 3, tri->n);

    tri->n0 = floats_dot(tri->p[0], tri->n, 3);

    // T * [ v0 v1 n ] = [ 1  0 0 ]
    //     [  |  | | ] = [ 0  1 0 ]
    //     [  |  | | ] = [ 0  0 1 ]

    matd_t *V = matd_create(3, 3);
    for (int i = 0; i < 3; i++) {
        MATD_EL(V, i, 0) = v0[i];
        MATD_EL(V, i, 1) = v1[i];
        MATD_EL(V, i, 2) = tri->n[i];
    }

    matd_t *T = matd_inverse(V);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            tri->T[i][j] = MATD_EL(T, i, j);

//    matd_print(T, "%15f");

    matd_destroy(T);
    matd_destroy(V);
}

sim_triray_t *sim_triray_create()
{
    sim_triray_t *str = calloc(1, sizeof(sim_triray_t));
    str->type = 0;
    str->u.tris.tris = zarray_create(sizeof(struct triangle));
    return str;
}

void sim_triray_destroy(sim_triray_t *str)
{
    if (!str)
        return;

    switch (str->type) {
        case 0: {
            zarray_destroy(str->u.tris.tris);
            break;
        }
        case 1: {
            zarray_destroy(str->u.strs.strs);
            break;
        }
    }

    free(str);
}

void sim_triray_add(sim_triray_t *str, const float p0[3], const float p1[3], const float p2[3])
{
    struct triangle tri;
    triangle_init(&tri, p0, p1, p2);
    assert(str->type == 0);
    zarray_add(str->u.tris.tris, &tri);
    str->sphere_valid = 0;
}

void sim_triray_add_mesh(sim_triray_t *str, mesh_model_t *model)
{
    for (int cidx = 0; cidx < zarray_size(model->chunks); cidx++) {
        struct mesh_model_chunk *chunk;
        zarray_get(model->chunks, cidx, &chunk);

        if (chunk->indices && zarray_size(chunk->indices) > 0) {
            assert(0);
        } else {
            for (int v = 0; v < zarray_size(chunk->vertices); v += 3) {
                float *a, *b, *c;
                zarray_get_volatile(chunk->vertices, v + 0, &a);
                zarray_get_volatile(chunk->vertices, v + 1, &b);
                zarray_get_volatile(chunk->vertices, v + 2, &c);
                sim_triray_add(str, a, b, c);
            }
        }
    }
}

// recursively update the bounding box. You must initialize mins/maxs
// somewhere (see compute_sphere)
void sim_tri_update_bounding_box(sim_triray_t *str, double mins[3], double maxs[3])
{
    switch (str->type) {
        case 0: {
            for (int i = 0; i < zarray_size(str->u.tris.tris); i++) {
                struct triangle *tri;
                zarray_get_volatile(str->u.tris.tris, i, &tri);

                for (int dim = 0; dim < 3; dim++) {
                    for (int vtx = 0; vtx < 3; vtx++) {
                        mins[dim] = fmin(mins[dim], tri->p[vtx][dim]);
                        maxs[dim] = fmax(maxs[dim], tri->p[vtx][dim]);
                    }
                }
            }
            break;
        }

        case 1: {
            for (int i = 0; i < zarray_size(str->u.strs.strs); i++) {
                sim_triray_t *thisstr;
                zarray_get(str->u.strs.strs, i, &thisstr);

                sim_tri_update_bounding_box(thisstr, mins, maxs);
            }
            break;
        }
    }
}

void sim_triray_split(sim_triray_t *str, const double max_sz, const int min_tris)
{
    assert(str->type == 0);

    if (zarray_size(str->u.tris.tris) < min_tris)
        return;

    // compute a bounding box
    double mins[3] = {  DBL_MAX,  DBL_MAX,  DBL_MAX };
    double maxs[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };

    sim_tri_update_bounding_box(str, mins, maxs);

    int worst_idx = -1;
    double worst_sz = 0;

    for (int i = 0; i < 3; i++) {
        double this_sz = maxs[i] - mins[i];
        if (this_sz >= worst_sz) {
            worst_idx = i;
            worst_sz = this_sz;
        }
    }

    // don't split?
    if (worst_sz < max_sz)
        return;

    // split along dimension "worst_dim"
    double thresh = (maxs[worst_idx] + mins[worst_idx]) / 2.0;

    sim_triray_t *left = sim_triray_create();
    sim_triray_t *right = sim_triray_create();

    for (int i = 0; i < zarray_size(str->u.tris.tris); i++) {
        struct triangle *tri;
        zarray_get_volatile(str->u.tris.tris, i, &tri);

        double coord = 0;
        for (int j = 0; j < 3; j++)
            coord += tri->p[j][worst_idx];
        coord /= 3.0;

        if (coord < thresh)
            zarray_add(left->u.tris.tris, tri);
        else
            zarray_add(right->u.tris.tris, tri);
    }

    if (zarray_size(left->u.tris.tris) == 0 ||
        zarray_size(right->u.tris.tris) == 0) {

        // we split for no gain (not handling this would cause infinite loop)
        sim_triray_destroy(left);
        sim_triray_destroy(right);
        return;
    }

    zarray_destroy(str->u.tris.tris);
    str->type = 1;
    str->u.strs.strs = zarray_create(sizeof(sim_triray_t*));
    zarray_add(str->u.strs.strs, &left);
    zarray_add(str->u.strs.strs, &right);

    sim_triray_split(left, max_sz, min_tris);
    sim_triray_split(right, max_sz, min_tris);
}

void sim_tri_compute_sphere(sim_triray_t *str)
{
    if (str->sphere_valid)
        return;

    // compute a bounding box
    double mins[3] = {  DBL_MAX,  DBL_MAX,  DBL_MAX };
    double maxs[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };

    sim_tri_update_bounding_box(str, mins, maxs);

    // fit a sphere.
    double hypot = 0;

    for (int dim = 0; dim < 3; dim++) {
        str->sphere_xyz[dim] = (mins[dim] + maxs[dim]) / 2.0;
        hypot += sq((maxs[dim] - mins[dim]) / 2);
    }

    str->sphere_r2 = hypot;
    str->sphere_valid = 1;
}

// dir must be UNIT vector
double sim_triray_cast(sim_triray_t *str, const double p[3], const double dir[3], double max)
{
    if (!str->sphere_valid)
        sim_tri_compute_sphere(str);

    if (1) {
        double sphere_vec[3]; // what direction from p to the sphere?
        for (int i = 0; i < 3; i++)
            sphere_vec[i] = str->sphere_xyz[i] - p[i];

        double dot = doubles_dot(sphere_vec, dir, 3);

        // how far do we miss the center of the sphere by?
        double miss[3];
        for (int i = 0; i < 3; i++)
            miss[i] = sphere_vec[i] - dot*dir[i];

        if (doubles_squared_magnitude(miss, 3) > str->sphere_r2)
            return max;
    }

    switch (str->type) {
        case 0: {
            for (int i = 0; i < zarray_size(str->u.tris.tris); i++) {
                struct triangle *tri;
                zarray_get_volatile(str->u.tris.tris, i, &tri);

                max = triangle_ray_distance(tri, p, dir, max);
            }

            break;
        }

        case 1: {
            for (int i = 0; i < zarray_size(str->u.strs.strs); i++) {
                sim_triray_t **thisstr;
                zarray_get_volatile(str->u.strs.strs, i, &thisstr);

                max = sim_triray_cast(*thisstr, p, dir, max);
            }

            break;
        }
    }

    return max;
}
