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

/*$LICENSE*/

#include <stdlib.h>
#include <string.h>

#include "scanmatch.h"
#include "common/zhash.h"
#include "common/zarray.h"
#include "common/math_util.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <assert.h>

static int __sm_slow_validation_warning = 0;

typedef struct int32x2 int32x2_t;
struct int32x2 {
    int32_t x, y;
};

//////////////////////////////////////////////////////////////////
// point hash, used in constructing sm_points_data
#define TNAME int32x2_hash
#define TKEYTYPE int32x2_t
#define TVALTYPE uint32_t
#define TKEYHASH(pk) ((uint32_t) ((pk)->x^(pk)->y))
#define TKEYEQUAL(pka, pkb) ((pka)->x == (pkb)->x && (pka)->y == (pkb)->y)
#include "common/thash_impl.h"
#undef TKEYEQUAL
#undef TKEYHASH
#undef TKEYTYPE
#undef TVALTYPE
#undef TNAME


/**
   Let us consider the problem of finding an integer-valued
   1D translation of 't*' such that

   t* = argmax_t \sum_i L(p_i + t);

   where L(...) is an arbitrary function mapping integer arguments to
   real values. We will actually generalize this somewhat more,
   letting the first element of L correspond to some point other than
   0. Let this offset be 'o', and we write:

   t* = argmax_t \sum_i L(p_i + t - o)                  [1]

   This problem extends naturally to 2D translations, but for
   simplicity, we will consider only the 1D version. We do NOT
   consider rotations.

   Let us consider the optimum translation t* over a small range of
   values T. Specifically, let us find the best translation in the
   set T:

   t \in T = [ D*j, D*j + D - 1 ]

   where j is an integer. In other words, we are searching over a
   range of "D" distinct values of t, where the range is "aligned" to
   a even multiple of D. (This alignment will be useful in the proof
   that follows...) Obviously, any search range for t can be enclosed
   by one or more such regions of size D.

   We now construct a "low resolution" version of L:

      L_D(y)   = max_x L(x)            x \in X = [D*y + o_d - o, D*y + 2*D - 2 + o_d - o ]

      such that o_d = D*k for some integer k.

   Claim:

      sum_i L_D(floor(p_i / D) + j - o_d / D) >= max_t sum_i L(p_i + t - o)     t \in T = [ D*j, D*j + D - 1 ]

   Proof:

      The claim is certainly true if the sum and max are interchanged
      (since now the RHS gets to pick a different t for every pixel---
      it can only get a better match!):

      sum_i L_D(floor(p_i / D) + j - o_d / D) >= sum_i max_t L(p_i + t - o)     t \in T = [ D*j, D*j + D - 1 ]

      And this will hold if the relationship is true for each individual point.

      L_D(floor(p_i / D) + j - o_d / D) >= max_t L(p_i + t - o)     t \in T = [ D*j, D*j + D - 1 ]

      [Note that the reverse arguments do not apply.] We will prove the final statement.

      Note that o_d / D = k, and move the "o" term into the range of
      T:

      L_D(floor(p_i / D) + j - k)       >= max_t L(p_i + t)       t \in T = [ D*j - o, D*j + D - 1 -o ]

      This holds if the range of values of L(x) used in the
      construction of L_D entirely contain the range of values in the
      RHS. This yields two constraints (checking that boundaries on
      both sides). First, that the left-most coordinate used in the
      construction of L_D is <= the left-most coordinate in T::

            D*(floor(p_i / D) + j - k) + o_d - o  <= p_i + D*j - o
        ==> D*floor(p_i / D) + D*j - D*k + o_d - o <= p_i + D*j - o
        ==> D*floor(p_i / D)  <= p_i     (true by def. of floor)

      and second:

            D*(floor(p_i / D) + j - k) + 2*D - 2 + o_d - o >= p_i + D*j + D - 1 - o
        ==> D*floor(p_i / D) + D*j - D*k + 2*D - 2 + o_d - o >= p_i + D*j + D - 1 - o
        ==> D*floor(p_i / D) + D - 1 >= p_i

        This range also makes sense: when working in an upper level of
        the pyramid, we map D different locations onto the same cell
        (with a worst-case error of (D-1) locations), and consider D
        different translations at once (with a maximum magnitude of
        (D-1).) These two uncertainties add, yielding a total
        positional uncertainty of 2*D - 2 locations.


   (Note: the conservatism of the construction of L_D (requiring a
   range of 2*D-2) stems from our desire to be able to decimate the
   points. If we are willing to individually transform every point
   (which comes at a substantial computational cost), we could reduce
   the range to D-1.

 Discussion

   If we compute L_D, we can replace "aligned" queries over T of size
   D with a single lookup in L_D, relying on L_D to conservatively
   bound (from above) the result of checking each translation in T
   individually. In an A*-style search, this conservative/optimistic
   bound serves as a good heuristic function that can quickly lead a
   search to the optimal value of t*.

   To use L_D, we must decompose ranges of translations into queries
   that span multiples of D. E.g., if D=8 and we wish to search the
   range of translations from [4, 33], we must perform queries for the
   following ranges: [4,7], [8,15], [16,23], [24,31], [32, 33]. (Note
   that it is conservative to enlarge these ranges, so we can use
   [0,7] instead of [4,7] and [32,39] instead of [32, 33]. This will
   be easier in practice.)

   Note also that the only form in which "p_i" appears is within a
   floor(p / D), which means that if two points p1 and p2 have
   floor(p1 / D) = floor(p2/ D), they will have identical values in
   the lookup table. These points can be merged so that we query L_D
   only once (scaling the result by the number of merged points).
   This means that not only are we decimating the images, but we are
   also decimating the points.

 Computing L_D

   We will build L_D by increasing D according to powers of two. The
   first L is the one provided by the user (the maximum resolution
   map). This corresponds to D=1.

   D = 1, 2D-2 = 0
   | 0   | 1   | 2   | 3   | 4   | 5   | 6   | 7   | 8   | 9    | 10    | 11    |  <-- L=L_1[0:11] (level 0)

   D = 2, 2D-2 = 2
   | 0,1,2     | 2,3,4     | 4,5,6     | 6,7,8     | 8,9,10     | 10,11,12      |  <-- L_2  (level 1)

   D = 4, 2D-2 = 6
   | 0,1,2,3,4,5,6         | 4,5,6,7,8,9,10        | 8,9,10,11,12,13,14         |  <-- L_4  (level 2)

   A useful recurrence is evident:

   LUT_{k+1}(i) = max( LUT_k(2*i+0, LUT_k(2*i+1), LUT_k(2*i+2)) )

   This corresponds to a max decimation with a max kernel of size 3,
   followed by decimation by a factor of 2. A bit of care must be
   exercised to ensure that each LUTD is aligned appropriately.

   Recall that each level of the LUT must be aligned to a multiple of
   D. When recursively computing the models, each level D>1 will have
   either o_d = o_(d/2), or o_d = o_(d/2) - d/2. I.e.---
   at D=1 (level 0), no special alignment is required (i.e., o_1 can
   be any value). At D=2, if o_1 is odd, it will be necessary to
   (conceptually) insert a dummy zero to the left of the image so that
   o_2 will be o_1 - 1.


 A note on biases.

   A systematic (and correctable) bias is introduced by using floor()
   instead of round() in the decimation of points. On average, points
   are subjected to a translation of -0.5 cells, even when t = 0. (If
   we used round(), the points [-0.5,0.5] would map to zero. Instead,
   the points [0, 1) map to zero. Thus input points are shifted lower
   by 0.5 more than they would if they were correctly rounded.)

   However, correct compensation for bias depends on the manner in
   which the model was constructed, as different methods for rendering
   the model can introduce potentially offsetting biases.

   If, for example, the model is generated by similarly using floor()
   (or, since image coordinates are always positive, by casting to
   "int"), the model will also appear to have been shifted by -0.5
   cells. In this case, both the model and points have identical
   shifts, and these biases cancel.

   However, if the model is generated using sub-pixel accurate
   rendering (i.e, by drawing fuzzy blobs whose center of mass is
   sub-pixel accurate), then the model has no bias. The biases do not
   cancel in this case, and thus the computed translations should be
   shifted by -0.5 to account for the bias in the points. (Why isn't
   it +0.5? The way to think about this is that the -0.5 bias in the
   points decimation is a translation that is being applied but not
   reported in blockx/blocky. Thus, to get the translation *actually*
   applied to the points, we need to include the -0.5 translation that
   results from the use of floor().

**/

static inline int32_t floorDiv(int32_t v, int32_t D)
{
    int32_t f = (int32_t) floor(((float) v) / D);
//    assert(v >= f*D && v <= (f*D+D-1));
    return f;
}

// every pixel with value 255 is added as a float[2] to the output
// array, translated and scaled according to (x0,y0) and
// meters_per_pixel, respectively.
zarray_t *sm_load_points_from_image(image_u8_t *im, float x0, float y0, float meters_per_pixel)
{
    zarray_t *points = zarray_create(sizeof(float[2]));

    for (int iy = 0; iy < im->height; iy++) {
        for (int ix = 0; ix < im->width; ix++) {
            if (im->buf[iy*im->stride + ix] == 255)
                zarray_add(points, (float[]) { x0 + ix*meters_per_pixel + .5, y0 + iy*meters_per_pixel + .5 });
        }
    }
    return points;
}

// padx/pady: number of zero columns/rows to "hallucinate" when decimating the input image.
static image_u8_t *decimate_image_pad(image_u8_t *in, int width, int height, int padx, int pady)
{
    image_u8_t *out = image_u8_create(width, height);

    // naive implementation. Note that the setting of padx/pady
    // "delay" the arrival of the input data, thus those values are
    // subtracted.
    for (int y = 0; y < out->height; y++) {
        for (int x = 0; x < out->width; x++) {
            uint8_t max = 0;

            for (int dy = 0; dy < 3; dy++) {
                int iy = 2*y - pady + dy;
                if (iy < 0 || iy >= in->height)
                    continue;

                for (int dx = 0; dx < 3; dx++) {
                    int ix = 2*x - padx + dx;
                    if (ix < 0 || ix >= in->width)
                        continue;

                    uint8_t v = in->buf[iy*in->stride + ix];
                    if (v > max)
                        max = v;
                }
            }

            out->buf[y*out->stride + x] = max;
        }
    }

    return out;
}

// it is an error to free a model that still belongs to search queries.
void sm_model_data_destroy(sm_model_data_t *model_data)
{
    for (int i = 0; i < zarray_size(model_data->models); i++) {
        sm_model_t *model;
        zarray_get(model_data->models, i, &model);
        image_u8_destroy(model->im);
        free(model);
    }

    zarray_destroy(model_data->models);
    free(model_data);
}

// the image becomes owned by the sm_model_data. Do not free it; it
// will be freed by sm_model_data_destroy
sm_model_data_t *sm_model_data_create_reference(image_u8_t *im,
                                                int32_t x0, int32_t y0,
                                                float meters_per_pixel, int nresolutions)
{
    // We create a list of L_Ds, with the first level being the
    // original image, the second L_2, the third L_4, etc.

    sm_model_data_t *model_data = calloc(1, sizeof(sm_model_data_t));
    model_data->meters_per_pixel = meters_per_pixel;
    model_data->models = zarray_create(sizeof(sm_model_t*));

    if (1) {
        sm_model_t *model = calloc(1, sizeof(sm_model_t));
        model->x0 = x0; // no alignment issues to worry about at the highest resolution; D=1
        model->y0 = y0;
        model->im = im;

        // add the highest-resolution model
        zarray_add(model_data->models, &model);
    }

    // now compute lower-resolution models
    for (int level = 1; level < nresolutions; level++) {

        int D = 1<<level;

        // have to move the origin back by as much as 2*D-2
        int32_t this_x0 = D*floorDiv(x0, D) - 2*D;
        int32_t this_y0 = D*floorDiv(y0, D) - 2*D;

        int32_t this_width = im->width / D + 4;
        int32_t this_height = im->height / D + 4;

//        printf("level %2d, x0: %4d, y0: %4d, width: %4d, height: %4d\n",
//               level, this_x0, this_y0, this_width, this_height);

        // does this image contain everything?
        assert(this_x0 + D*2 - 2 <= x0);
        assert(this_y0 + D*2 - 2 <= y0);
        assert(this_x0 + this_width*D - 1 >= x0 + im->width - 1);
        assert(this_y0 + this_height*D - 1 >= y0 + im->height - 1);

        image_u8_t *out = image_u8_create(this_width, this_height);
        for (int y = 0; y < this_height; y++) {
            for (int x = 0; x < this_width; x++) {
                uint8_t max = 0;
                for (int ty = 0; ty <= 2*D - 2; ty++) {
                    int py = D*y + this_y0 - y0 + ty;
                    if (py < 0 || py >= im->height)
                        continue;

                    for (int tx = 0; tx <= 2*D - 2; tx++) {
                        int px = D*x + this_x0 - x0 + tx;
                        if (px < 0 || px >= im->width)
                            continue;

                        uint8_t v = im->buf[py*im->stride + px];
                        if (v > max)
                            max = v;
                    }
                }

                out->buf[y*out->stride + x] = max;
            }
        }

        if (0) {
            char name[1024];
            snprintf(name, sizeof(name), "decimate-%d.pnm", level);
            image_u8_write_pnm(out, name);
        }

        sm_model_t *model = calloc(1, sizeof(sm_model_t));
        model->x0 = this_x0;
        model->y0 = this_y0;
        model->im = out;
        zarray_add(model_data->models, &model);
    }

    return model_data;
}

static int sm_model_data_identical(sm_model_data_t *ma, sm_model_data_t *mb)
{
    if (zarray_size(ma->models) != zarray_size(mb->models))
        return 0;

    for (int level = 0; level < zarray_size(ma->models); level++) {
        sm_model_t *a, *b;
        zarray_get(ma->models, level, &a);
        zarray_get(mb->models, level, &b);

        if (a->x0 != b->x0)
            return 0;

        if (a->y0 != b->y0)
            return 0;

        if (a->im->width != b->im->width)
            return 0;

        if (a->im->height != b->im->height)
            return 0;

        for (int y = 0; y < a->im->height; y++) {
            for (int x = 0; x < a->im->width; x++) {
                if (a->im->buf[y*a->im->stride+x] != b->im->buf[y*b->im->stride+x])
                    return 0;
            }
        }
    }

    return 1;
}

// the image becomes owned by the sm_model_data. Do not free it; it
// will be freed by sm_model_data_destroy
sm_model_data_t *sm_model_data_create(image_u8_t *im,
                                      int32_t x0, int32_t y0,
                                      float meters_per_pixel, int nresolutions)
{
    sm_model_data_t *model_data = calloc(1, sizeof(sm_model_data_t));
    model_data->meters_per_pixel = meters_per_pixel;
    model_data->models = zarray_create(sizeof(sm_model_t*));

    sm_model_t *model = calloc(1, sizeof(sm_model_t));
    model->x0 = x0; // no alignment issues to worry about at the highest resolution; D=1
    model->y0 = y0;
    model->im = im;

    // add the highest-resolution model
    zarray_add(model_data->models, &model);

    // now compute lower-resolution models
    for (int level = 1; level < nresolutions; level++) {
        sm_model_t *last;
        zarray_get(model_data->models, level-1, &last);

        int D = 1<<level;

        // have to move the origin back by as much as 2*D-2
        int32_t this_x0 = D*floorDiv(x0, D) - 2*D;
        int32_t this_y0 = D*floorDiv(y0, D) - 2*D;

        int32_t this_width = im->width / D + 4;
        int32_t this_height = im->height / D + 4;

        sm_model_t *model = calloc(1, sizeof(sm_model_t));
        model->x0 = this_x0;
        model->y0 = this_y0;
        int padx = (last->x0 - this_x0) / (D / 2);
        int pady = (last->y0 - this_y0) / (D / 2);

        model->im = decimate_image_pad(last->im, this_width, this_height, padx, pady);
        zarray_add(model_data->models, &model);
    }

    if (0) {
        sm_model_data_t *md = sm_model_data_create_reference(image_u8_copy(im),
                                                             x0, y0, meters_per_pixel, nresolutions);
        if (!sm_model_data_identical(md, model_data)) {
            printf("incorrect model\n");
            exit(0);
        }

        sm_model_data_destroy(md);
    }
    return model_data;
}

// it is an error to destroy a points_data object if there are still
// entries referring to it in the heap.
void sm_points_data_destroy(sm_points_data_t *points_data)
{
    sm_points_data_clear_cache(points_data);
    zarray_destroy(points_data->points);

    sm_points_record_hash_destroy(points_data->points_hash);
    free(points_data);
}

void sm_points_data_clear_cache(sm_points_data_t *points_data)
{
    sm_points_record_hash_iterator_t iter;
    sm_points_record_hash_iterator_init(points_data->points_hash, &iter);

    struct sm_points_record rec;
    zarray_t *points;

    while (sm_points_record_hash_iterator_next(&iter, &rec, &points)) {
        zarray_destroy(points);
    }

    sm_points_record_hash_clear(points_data->points_hash);
}


sm_points_data_t *sm_points_data_create(zarray_t *points)
{
    return sm_points_data_create_flags(points, 0);
}

sm_points_data_t *sm_points_data_create_flags(zarray_t *points, uint32_t flags)
{
    sm_points_data_t *points_data = calloc(1, sizeof(sm_points_data_t));
    points_data->flags = flags;
    points_data->points = points;
    points_data->points_hash = sm_points_record_hash_create();

    return points_data;
}

// OUTPUT: each element is an int32_t[3] { x, y, weight }
zarray_t *sm_points_data_get(sm_points_data_t *sm_data,
                             float meters_per_pixel, float rad, int level)
{
    int npoints = zarray_size(sm_data->points);

    struct sm_points_record smr = { .rad = rad, .level = level, .meters_per_pixel = meters_per_pixel };

    zarray_t *out_points;
    if (sm_points_record_hash_get(sm_data->points_hash, &smr, &out_points)) {
        return out_points;
    }

    out_points = zarray_create(sizeof(int32_t[3]));

    // we use a hash table whose key is the X,Y position so that we can
    // detect duplicates (and thus have fewer points.) The value is
    // the # of points (i.e. the weight).
    int32x2_hash_t *pthash = int32x2_hash_create_capacity(npoints);

    float c = cos(rad), s = sin(rad);

    for (int pointidx = 0; pointidx < npoints; pointidx++) {
        float *fp;
        zarray_get_volatile(sm_data->points, pointidx, &fp);

        float x = c*fp[0] - s*fp[1];
        float y = s*fp[0] + c*fp[1];

//      The code below used to be written this way. However, this
//      could cause subtle rounding problems?
//
//        p[0] = floorf(x / this_meters_per_pixel);
//        p[1] = floorf(y / this_meters_per_pixel);

        // The version below avoids the risk of different rounding as a
        // function of level; the floating-point portion is identical
        // for every level.
        int32x2_t xy = { .x = floorf(x / meters_per_pixel), floorf(y / meters_per_pixel) };
        xy.x = floorDiv(xy.x, 1 << level);
        xy.y = floorDiv(xy.y, 1 << level);

        uint32_t *count;

        if (int32x2_hash_get_volatile(pthash, &xy, &count)) {
            // increase weight of existing point
            (*count)++;
        } else {
            // add a new point
            uint32_t this_count = 1;
            int32x2_hash_put(pthash, &xy, &this_count, NULL, NULL);
        }
    }

    // now create a simple list of points by iterating over
    // the hash table.
    int32x2_hash_iterator_t zit;
    int32x2_hash_iterator_init(pthash, &zit);

    int32_t p[3];
    uint32_t count;

    int32_t total_count = 0;

    int32x2_t xy;
    while (int32x2_hash_iterator_next(&zit, &xy, &count)) {
        if (sm_data->flags & SM_POINTS_NODECIMATE) {
            // only here to measure impact of decimation in points
            for (int i = 0; i < count; i++) {
                p[2] = 1;
                zarray_add(out_points, &p);
                total_count ++;
            }
        } else {
            p[0] = xy.x;
            p[1] = xy.y;
            p[2] = count;
            zarray_add(out_points, &p);
            total_count += count;
        }
    }

    assert(total_count == zarray_size(sm_data->points));

    int32x2_hash_destroy(pthash);

    if (sm_points_record_hash_put(sm_data->points_hash, &smr, &out_points, NULL, NULL)) {
        assert(0);
    }

    return out_points;
}


void sm_search_destroy(sm_search_t *search)
{
    for (int i = 0; i < zarray_size(search->handles); i++) {
        sm_search_handle_t *handle;
        zarray_get(search->handles, i, &handle);
        free(handle->meaninfs);
        free(handle);
    }

    sm_search_record_heap_destroy(search->maxheap);
    zarray_destroy(search->handles);

    free(search);
/*
    // remove any handles that were not removed by the user.  NB: We
    // could probably deallocate these faster than removing them one
    // by one as we do here, since we go through some effort to
    // maintain the maxheap at every step.
    for (int i = 0; i < zarray_size(search->handles); i++) {
        sm_search_handle_t *handle;
        zarray_get(search->handles, i, &handle);
        sm_search_remove(search, handle);
    }

    // all handles should have been destroyed by now.
    assert(zmaxheap_size(search->maxheap)==0);

    zmaxheap_destroy(search->maxheap);
    zarray_destroy(search->handles);

    free(search);
 */
}

sm_search_t *sm_search_create()
{
    sm_search_t *search = calloc(1, sizeof(sm_search_t));

    search->maxheap = sm_search_record_heap_create();
    search->handles = zarray_create(sizeof(sm_search_handle_t*));

    return search;
}

// the points_data and model_data should be constructed with the same
// meters_per_pixel.
sm_search_handle_t *sm_search_add(sm_search_t *search,
                                  sm_points_data_t *points_data, sm_model_data_t *model_data,
                                  int32_t tx0, int32_t tx1, int32_t ty0, int32_t ty1,
                                  float rad0, float rad1, float radstep,
                                  float scale, float *mean, float *inf,
                                  float minscore_before_penalty)
{
    if (mean == NULL) {
        assert(inf == NULL);
    }

    if (inf == NULL) {
        assert(mean == NULL);
    }

    sm_search_handle_t *handle = calloc(1, sizeof(sm_search_handle_t));
    handle->points_data = points_data;
    handle->model_data = model_data;
    handle->scale = scale;
    handle->tx0 = tx0;
    handle->tx1 = tx1;
    handle->ty0 = ty0;
    handle->ty1 = ty1;
    handle->minscore_before_penalty = minscore_before_penalty;

    // what's the lowest-resolution data available?
    int max_level = zarray_size(model_data->models) - 1;

    assert(tx1 >= tx0);
    assert(ty1 >= ty0);
    assert(rad1 >= rad0);

    // which gridmap resolution should we start with?  (Start at the
    // coarsest resolution level that results in at least some branch
    // factor.)
    int maxt = imax(ty1 - ty0, tx1 - tx0);

    // Picking the top level has a pretty big impact on
    // performance. Intuitively, we want a small (but greater than 1)
    // branching factor, which argues for something like
    // log2(maxt). The -1 term below was found empirically on a small
    // example; it might not be the best choice. Note that the initial
    // level can actually be one level higher than our actual biggest
    // image, since we never actually evaluate the score at *this*
    // level. (similarly, we can't have level==0)
    int level = floor(logf(maxt)/logf(2)) - 1;
    level = imax(level, 1);
    level = imin(level, max_level);

    int D = 1 << level;

    // create an initial set of heap entries, one for each rotation.
    // Error tends to accumulate in rad as we iterate, thus we're
    // using double precision instead of single precision.
    // NB: nrads is an upper bound on the memory we'll actually use.
    int nrads = (rad1 - rad0) / radstep + 1;

    if (inf) {
        handle->meaninfs = calloc(nrads, sizeof(struct sm_meaninf));
        memcpy(handle->mean3, mean, 3*sizeof(float));
        memcpy(handle->inf33, inf, 9*sizeof(float));
    }

    int radidx = 0;
    for (double rad = rad0; rad <= rad1; rad += radstep, radidx++) {
        // We're going to compute the quadratic costs conditioned on
        // the specific value of rad.
        //
        // Our penalty is formulated as a quadratic loss in x, y, and
        // theta. Let us group error in x and y as e1, and error in
        // theta as e2. E.g., e1 = x - mean(x). We obtain:
        //
        // [e1]T [ A  B ] [e1]
        // [e2]  [ B' C ] [e2]
        //
        // Performing an LDU decomposition on [A B; B' C], and
        // expanding the [e1 e2] terms into L and U, we obtain:
        //
        // [ e1 + inv(A)*B*e2 ]T [ A       0         ] [ e1 + inv(A)*B*e2 ]
        // [       e2         ]  [ 0   C-B'*inv(A)*B ] [        e2        ]
        //
        // Note, if the original penalty corresponds to a multivariate
        // Gaussian distribution over x,y,t, this factorization
        // corresponds to p(xyt) = p(xy|t)p(t).
        //
        // Note that e2 is constant for all children with the same
        // theta.

        // pluck out B submatrix from information matrix

        struct sm_meaninf *meaninf = NULL;

        if (inf) {
            float B0 = inf[2];
            float B1 = inf[5];

            // pluck out A matrix. (note: symmetric)
            float A00 = inf[0];
            float A01 = inf[1];
            float A11 = inf[4];

            // need to invert A. (note: also symmetric)
            float Ainvdet = 1.0 / (A00*A11 - A01*A01);
            float iA00 = A11 * Ainvdet;
            float iA01 = -A01 * Ainvdet;
            float iA11 = A00 * Ainvdet;

            // pluck out C matrix (just a scalar for us)
            float C = inf[8];

            float e2 = mod2pi(rad - mean[2]);

            // Here's the p(e2)=p(t) penalty term.
            float rad_penalty = e2*(C - (B0*B0*iA00 + 2*B0*B1*iA01 + B1*B1*iA11))*e2;

            // Compute the conditional means for x and y.
            float u0 = mean[0] - e2*(B0*iA00+B1*iA01);
            float u1 = mean[1] - e2*(B0*iA01+B1*iA11);

            meaninf = &handle->meaninfs[radidx];
            meaninf->u0 = u0;
            meaninf->u1 = u1;
            meaninf->A00 = A00;
            meaninf->A01 = A01;
            meaninf->A11 = A11;
            meaninf->rad_penalty = rad_penalty;
        }

        // create an integral number of heap queries that are all
        // aligned to D boundaries.  This creates an invariant that
        // each heap entry will yield exactly 4 children.
        for (int blocky = floorDiv(ty0, D); D*blocky <= ty1; blocky ++) {
            for (int blockx = floorDiv(tx0, D); D*blockx <= tx1; blockx ++) {

                sm_search_record_t r = { .handle = handle,
                                         .level = level,
                                         .rad = rad,
                                         .blockx = blockx,
                                         .blocky = blocky,
                                         .meaninf = meaninf };

                sm_search_record_heap_add(search->maxheap, &r, HUGE);
            }
        }
    }

    zarray_add(search->handles, &handle);
    return handle;
}

// compute a lower bound of the chi2 of the quadratric
// (x-u)'*INF*(x-u) in the rectangular region.
static float evaluate_minimum_quadratic(float *mean, float *inf,
                                        float tx0, float tx1, float ty0, float ty1)
{
    // if the volume contains the mean, the minimum is zero.
    if (tx0 <= mean[0] && tx1 >= mean[0] && ty0 <= mean[1] && ty1 >= mean[1])
        return 0;

    float a = inf[0], b = inf[1], d = inf[3];

    // the minimum must now be along the perimeter. Try each of the
    // four segments in turn.

    // create a single-variable quadratic expression by fixing a
    // value, then minimizing the other variable.
    //
    // X^2 = [ex ey] [ a b; b d] [ex ey]'   (with e.g. ey = y - uy)
    //
    float minchi2 = HUGE;

    // XXX TODO We shouldn't have to evaluate all 4 edges.

    if (1) {
        // do a search along the Y-aligned line with a constant value
        // of x (x=tx0).
        float x = tx0;

        // compute ex (the error in x)
        float ex = x - mean[0];

        // differentiate with respect to y, set to zero, solve for ey.
        float ey = -b*ex / d;

        float y = ey + mean[1]; // compute the y.

        y = fmin(fmax(y, ty0), ty1); // clamp to [ty0, ty1]

        // recompute ey for the clamped y.
        ey = y - mean[1];

        // now, what is the value of the expression at this value of x and y?
        float chi2 = a*ex*ex + 2*b*ex*ey + d*ey*ey;

        minchi2 = fmin(chi2, minchi2);
    }

    if (1) {
        float x = tx1;
        float ex = x - mean[0];
        float ey = -b*ex / d;
        float y = ey + mean[1];
        y = fmin(fmax(y, ty0), ty1); // clamp to [ty0, ty1]
        ey = y - mean[1];
        float chi2 = a*ex*ex + 2*b*ex*ey + d*ey*ey;

/*        if (fabs(tx1-mean[0]) < fabs(tx0-mean[0])) {
            if (chi2 >= minchi2) {
                printf("%15f %15f; %15f %15f %15f\n", chi2, minchi2, mean[0], tx0, tx1);
            }
            assert(chi2 < minchi2);
        }
*/
        minchi2 = fmin(chi2, minchi2);
    }

    if (1) {
        float y = ty0;
        float ey = y - mean[1];
        float ex = -b*ey / a;
        float x = ex + mean[0];
        x = fmin(fmax(x, tx0), tx1); // clamp to [tx0, tx1]
        ex = x - mean[0];
        float chi2 = a*ex*ex + 2*b*ex*ey + d*ey*ey;
        minchi2 = fmin(chi2, minchi2);
    }

    if (1) {
        float y = ty1;
        float ey = y - mean[1];
        float ex = -b*ey / a;
        float x = ex + mean[0];
        x = fmin(fmax(x, tx0), tx1); // clamp to [tx0, tx1]
        ex = x - mean[0];
        float chi2 = a*ex*ex + 2*b*ex*ey + d*ey*ey;
        minchi2 = fmin(chi2, minchi2);
    }

    return minchi2;
}

// call this method to free matches returned by sm_search_run.
void sm_search_record_destroy(sm_search_record_t *r)
{
    free(r);
}

// expand the heap until a maximum-resolution match is found.  Returns
// a match (which you must destroy with sm_search_record_destroy) or
// NULL, if no match was found.
//
// we correct for the bias introduced by point cloud decimation, but assume
// that the model is unbiased. If your model is biased, you should correct
// the resulting transform.
sm_result_t *sm_search_run(sm_search_t *search)
{
    int debug = 0;
    int trace = 0;

    while (1) {
        sm_search_record_t r;
        float rscore = 0;

        if (!sm_search_record_heap_remove_max(search->maxheap, &r, &rscore)) {
            return NULL;
        }

        if (r.level == 0) {
            // we've found our solution
            sm_result_t *result = calloc(1, sizeof(sm_result_t));
            result->handle = r.handle;

            // compensate for systematic -0.5 bias resulting from
            // point cloud decimation and truncation. See comments
            // above.
            result->xyt[0] = (r.blockx - 0.5) * r.handle->model_data->meters_per_pixel;
            result->xyt[1] = (r.blocky - 0.5) * r.handle->model_data->meters_per_pixel;
            result->xyt[2] = r.rad;
            result->score = rscore;

            return result;
        }

        sm_search_handle_t *handle = r.handle;

        sm_model_t *parent_model;
        zarray_get(handle->model_data->models, r.level, &parent_model);

        // The object on the top of heap represents the best alignment
        // known so far. Our job is to expand that alignment into
        // higher-resolution child alignments.
        sm_model_t *child_model;
        int child_level = r.level - 1;

        zarray_get(handle->model_data->models, child_level, &child_model);

        if (trace)
            printf("par  level %2d, tx0 %4d, ty0 %4d, score: %8.5f\n",
                   r.level, r.blockx * (1<<r.level), r.blocky * (1<<r.level), rscore);

        zarray_t *points = sm_points_data_get(handle->points_data,
                                              handle->model_data->meters_per_pixel,
                                              r.rad, child_level);

        // how many level 0 pixels does each pixel at "child_level"
        // represent?
        int childD = 1 << child_level;

        image_u8_t *child_im = child_model->im;

        // split this parent node (r) into four children, and evaluate
        // them using the child image.
        for (int subblocky = 0; subblocky < 2; subblocky ++) {

            int child_blocky = 2*r.blocky + subblocky;

            // the parent block intersects the requested search area,
            // but some of the children blocks may not. Prune them
            // here. (XXX: Off by ones? I've made these conservative.)
            if (child_blocky*childD > handle->ty1)
                continue;
            if (child_blocky*childD + childD < handle->ty0)
                continue;

            for (int subblockx = 0; subblockx < 2; subblockx++) {

                int child_blockx = 2*r.blockx + subblockx;

                // again, prune children outside of the search area.
                if (child_blockx*childD > handle->tx1)
                    continue;
                if (child_blockx*childD + childD < handle->tx0)
                    continue;

                // Perform a scan match operation.
                int32_t score = 0;

                int npoints = zarray_size(points);

                int offx = child_blockx - child_model->x0 / childD;
                int offy = child_blocky - child_model->y0 / childD;

                for (int pidx = 0; pidx < npoints; pidx++) {
//                  int32_t *p;
//                  zarray_get_volatile(points, pidx, &p);
                    int32_t *p = &((int32_t*) points->data)[pidx*3];

//                  assert(child_model->x0 / childD == floorDiv(child_model->x0, childD));
//                  assert(child_model->y0 / childD == floorDiv(child_model->y0, childD));

                    // note: points are already floorDiv'd by childD.
                    int coordx = p[0] + offx; // child_blockx - child_model->x0 / childD;
                    if (coordx < 0 || coordx >= child_im->width)
                        continue;

                    int coordy = p[1] + offy; // child_blocky - child_model->y0 / childD;
                    if (coordy < 0 || coordy >= child_im->height)
                        continue;

                    score += p[2]*child_im->buf[coordy * child_im->stride + coordx];
                }

                // compute search ranges for the child node, clamping
                // at the originally-specified search bounds. (otherwise,
                // our search range tends to increase as we descend the tree
                // to higher-resolution gridmaps. ick!)
                sm_search_record_t child = { .handle = r.handle,
                                             .level = r.level - 1,
                                             .rad = r.rad,
                                             .meaninf = r.meaninf,
                                             .blockx = child_blockx,
                                             .blocky = child_blocky };

                double cscore = score * handle->scale;

                if (cscore < handle->minscore_before_penalty)
                    continue;

                // compute a penalty based on the prior.
                //
                if (child.meaninf != NULL) {
                    // Our penalty is formulated as a quadratic loss
                    // in x, y, and theta. Let us group error in x and
                    // y as e1, and error in theta as e2. E.g., e1 = x
                    // - mean(x). We obtain:
                    //
                    // [e1]T [ A  B ] [e1]
                    // [e2]  [ B' C ] [e2]
                    //
                    // Performing an LDU decomposition on [A B; B' C],
                    // and expanding the [e1 e2] terms into L and U,
                    // we obtain:
                    //
                    // [ e1 + inv(A)*B*e2 ]T [ A       0         ] [ e1 + inv(A)*B*e2 ]
                    // [       e2         ]  [ 0   C-B'*inv(A)*B ] [        e2        ]
                    //
                    // Note, if the original penalty corresponds to a
                    // multivariate Gaussian distribution over x,y,t,
                    // this factorization corresponds to p(xyt) =
                    // p(xy|t)p(t).
                    //
                    // Note that e2 is constant for all children with
                    // the same theta.

                    if (child.meaninf) {

                        if (child.level == 0)
                            assert(childD == 1);

                        // NB: Even at the lowest level of the
                        // pyramid, evaluate the prior over the full
                        // search space.

                        // XXX TODO refactor evaluate_minimum_quadratic so
                        // we don't have to repack these.
                        float modified_mean[] = { child.meaninf->u0, child.meaninf->u1 };
                        float modified_inf[] = { child.meaninf->A00, child.meaninf->A01,
                                                 child.meaninf->A01, child.meaninf->A11 };

                        double tx0 = handle->model_data->meters_per_pixel * child.blockx * childD;
                        double tx1 = handle->model_data->meters_per_pixel * (child.blockx + 1) * childD;

                        double ty0 = handle->model_data->meters_per_pixel * child.blocky * childD;
                        double ty1 = handle->model_data->meters_per_pixel * (child.blocky + 1) * childD;

                        float xy_penalty = evaluate_minimum_quadratic(modified_mean, modified_inf,
                                                                      tx0, tx1, ty0, ty1);

                        double penalty = child.meaninf->rad_penalty + xy_penalty;
                        cscore -= penalty;
                    }

/*
                    if (0) {
                        if (__sm_slow_validation_warning == 0) {
                            printf("NOTICE: Prior penalty validation enabled (will be slow).\n");
                            __sm_slow_validation_warning = 1;
                        }

                        // VERY EXPENSIVE VALIDATION TEST.
                        // validate our conservative penalty by
                        // exhaustively computing over whole window.
                        float min_penalty = HUGE;

                        for (int tx = child->tx0; tx <= child->tx1; tx++) {
                            for (int ty = child->ty0; ty <= child->ty1; ty++) {

                                float ex = tx - handle->mean3[0];
                                float ey = ty - handle->mean3[1];
                                float et = mod2pi(child.rad - handle->mean3[2]);

                                // child's information matrix is symmetrical.
                                float A = handle->inf33[0], B = handle->inf33[1], C = handle->inf33[2];
                                float                       D = handle->inf33[4], E = handle->inf33[5];
                                float                                             F = handle->inf33[8];

                                float this_penalty = ex*ex*A + 2*ex*ey*B + 2*ex*et*C + ey*ey*D + 2*ey*et*E + et*et*F;
                                min_penalty = fmin(min_penalty, this_penalty);
                            }
                        }

                        // NB: Don't freak out if the answer is
                        // different just due to numerical error.
                        if (penalty > min_penalty) {
                            double err = fabs((double) penalty - (double) min_penalty) / fmax(penalty, min_penalty);

                            if (err > 1E-5) {
                                // potential problem detected.
                                printf("level: %d,  %f > %f. rad=%f (err=%e) %s\n",
                                       child.level, penalty, min_penalty, child.rad, err,
                                       penalty <= min_penalty ? "" : "BAD");

                                assert(penalty <= min_penalty);
                            }
                        }
                    }
*/
                }

                if (trace)
                    printf(" add level %2d, tx0 %4d, ty0 %4d, score: %8.5f\n",
                           child.level, child.blockx * (1<<child.level), child.blocky * (1<<child.level), cscore);


                if (cscore > rscore) {
                    double err = ((double) rscore - (double) cscore) / cscore;
                    if (err < -1E-6) {
                        printf("child level: %d child score: %f, parent score: %f, err: %f, tx0 %d, ty0 %d, rad %6.3f\n",
                               child.level, cscore, rscore, err,
                               child.blockx * (1<<child.level), child.blocky * (1<<child.level), child.rad);

                        assert (cscore <= rscore);
                    }
                }

                sm_search_record_heap_add(search->maxheap, &child, cscore);
            }
        }
    }
}

void sm_result_destroy(sm_result_t *result)
{
    free(result);
}

void sm_search_remove(sm_search_t *search, sm_search_handle_t *handle)
{
    assert(0);
/*
    zmaxheap_iterator_t it;

    zmaxheap_iterator_init(search->maxheap, &it);

    sm_search_record_t *r;
    float score;

    while (zmaxheap_iterator_next(&it, &r, &score)) {
        if (r.handle == handle) {
            zmaxheap_iterator_remove(&it);
            free(r);
        }
    }

    zmaxheap_iterator_finish(&it);

    zarray_remove_value(search->handles, handle, 1);
    free(handle->meaninfs);
    free(handle);
*/
}

void sm_hillclimb_result_destroy(sm_hillclimb_result_t *res)
{
    free(res);
}

// we use the floating point versions of the points, so there is no
// bias resulting from points decimation. Correcting for bias
// resulting from the model is still the caller's job.
static double eval(sm_points_data_t *points_data, sm_model_data_t *model_data, double qxyt[3],
                   float scale, const float *mean, const float *inf,
                   double *_penalty)
{
    // get high-resolution model
    sm_model_t *model;
    zarray_get(model_data->models, 0, &model);
    image_u8_t *im = model->im;
    double meters_per_pixel = model_data->meters_per_pixel;

    int npoints = zarray_size(points_data->points);
    float *points = (float*) points_data->points->data;

    while (qxyt[2] > M_PI)
        qxyt[2] -= 2*M_PI;
    while (qxyt[2] < - M_PI)
        qxyt[2] += 2*M_PI;

    // try a new alignment at qxyt
    double c = cos(qxyt[2]), s = sin(qxyt[2]);
    double x0 = model->x0 * meters_per_pixel;
    double y0 = model->y0 * meters_per_pixel;

    int32_t iscore = 0;

    // we introduce a bit of bias below by truncating instead of
    // rounding. This bias introduces a net translation of -0.5. Here, we pre-compensate for that bias
    // by shifting points by +0.5. (However, since we *subtract* x0 and y0, we actually subtract here.
    x0 -= 0.5 * meters_per_pixel;
    y0 -= 0.5 * meters_per_pixel;

    for (int i = 0; i < npoints; i++) {
        float *xy = &points[2*i];

        float Tx = xy[0] * c - xy[1] * s + qxyt[0];

        // XXX We're doing lookups by truncating (casting to
        // int). This isn't *quite* right, since small negative values
        // (which should map to negative indices) will be truncated to
        // an index of zero, which may have a non-zero score. Should
        // have: int ix = floor(( Tx - x0) / meters_per_pixel), but
        // this isn't worth the computational cost.
        //
        // Note that if we used round(), we would have zero bias.  But
        // this simple integer truncation gives us a baked-in bias of
        // -0.5 cells.
        int ix = (Tx - x0) / meters_per_pixel;
        if (ix < 0 || ix >= im->width)
            continue;

        float Ty = xy[0] * s + xy[1] * c + qxyt[1];
        int iy = (Ty - y0) / meters_per_pixel;
        if (iy < 0 || iy >= im->height)
            continue;

        iscore += im->buf[iy * im->stride + ix];
    }

    double penalty = 0;

    assert(iscore >= 0);

    if (inf) {
        double res[3] = { qxyt[0] - mean[0], qxyt[1] - mean[1], mod2pi(qxyt[2] - mean[2]) };
        double inf_r[3] = { inf[0]*res[0] + inf[1]*res[1] + inf[2]*res[2],
                            inf[3]*res[0] + inf[4]*res[1] + inf[5]*res[2],
                            inf[6]*res[0] + inf[7]*res[1] + inf[8]*res[2] };

        penalty = (res[0]*inf_r[0] + res[1]*inf_r[1] + res[2]*inf_r[2]);
    }

    double score = scale*iscore - penalty;
    if (_penalty)
        *_penalty = penalty;

//    printf("HillClimb Eval %15f %15f %15f, %15f %15f\n", qxyt[0], qxyt[1], qxyt[2], score, penalty);
/*    if (score < 0)
        printf("%d %15f %15f %15f\n", iscore, scale, penalty, score);
        assert(score >= 0); */
    return score;
}

sm_hillclimb_result_t *sm_hillclimb(sm_points_data_t *points_data, sm_model_data_t *model_data, double _xyt0[3],
                                    sm_hillclimb_params_t *params,
                                    float scale, float *mean, float *inf)
{
    sm_hillclimb_result_t *result = calloc(1, sizeof(sm_hillclimb_result_t));

    double xyt0[3];
    memcpy(xyt0, _xyt0, 3*sizeof(double));

    double best_xyt[3] = { xyt0[0], xyt0[1], xyt0[2] };
    double best_penalty = 0;
    double best_score = eval(points_data, model_data, best_xyt, scale, mean, inf, &best_penalty);

    double step_sizes[3];
    memcpy(step_sizes, params->initial_step_sizes, sizeof(step_sizes));
    int step_size_shrinks = 0; // our count so far

    while (result->iters < params->maxiters) {
        int improved = 0;

        result->iters++;

        // we will attempt six refinements (one in each coordinate
        // direction), all starting from reference_xyt. (As opposed to
        // instantaneously accepting any step that improves noise.)
        double reference_xyt[3];
        memcpy(reference_xyt, best_xyt, sizeof(reference_xyt));

        // we will now try six steps with respect to reference_xyt
        for (int change_index = 0; change_index < 3; change_index ++) {
            for (int change_direction = -1; change_direction <= 1; change_direction += 2) {

                // what is our query xyt?
                double qxyt[3];
                memcpy(qxyt, reference_xyt, sizeof(qxyt));

                qxyt[change_index] += step_sizes[change_index] * change_direction;
                double this_penalty;
                double this_score = eval(points_data, model_data, qxyt, scale, mean, inf, &this_penalty);

                if (this_score > best_score) {
                    memcpy(best_xyt, qxyt, sizeof(best_xyt));
                    best_score = this_score;
                    best_penalty = this_penalty;
                    improved = 1;
                }
            } // for change_dir
        } // for change_index

        // If we're far from the minimum, it makes sense to try
        // another step with same (largish) step size parameter.
        if (improved) {
/*            printf("%5d %15f %15f %15f: %15f\n",
                   result->iters, best_xyt[0], best_xyt[1], best_xyt[2], (double) best_score);
*/
            // try the same step size again.
            continue;
        }

        if (step_size_shrinks == params->max_step_size_shrinks)
            break;

        for (int i = 0; i < 3; i++)
            step_sizes[i] *= params->step_size_shrink_factor;
        step_size_shrinks++;
    }

    // correct for bias resulting from us projecting using integer
    // cast instead of round.
//    double meters_per_pixel = model_data->meters_per_pixel;
    result->xyt[0] = best_xyt[0]; // - 0.5*meters_per_pixel;
    result->xyt[1] = best_xyt[1]; // - 0.5*meters_per_pixel;
    result->xyt[2] = best_xyt[2];
    result->score = best_score;
    result->penalty = best_penalty;

//    printf("****** %f\n", result->score);
//    assert(result->score >= 0);
    return result;
}

// A = BT

struct align2d
{
    double Sax, Say, Sbx, Sby, Saxby, Saybx, Saxbx, Sayby;
    double weight;
};

void align2d_init(struct align2d *a)
{
    memset(a, 0, sizeof(struct align2d));
}

// compute a rigid-body transformation that transforms (ax,ay) to align with (bx, by)
void align2d_update(struct align2d *a, double ax, double ay, double bx, double by, double weight)
{
    a->Sax += ax;
    a->Say += ay;
    a->Sbx += bx;
    a->Sby += by;
    a->Saxby += ax*by;
    a->Saybx += ay*bx;
    a->Saxbx += ax*bx;
    a->Sayby += ay*by;
    a->weight += weight;
}

void align2d_compute(struct align2d *a, double *xyt)
{
    double axc = a->Sax / a->weight;
    double ayc = a->Say / a->weight;
    double bxc = a->Sbx / a->weight;
    double byc = a->Sby / a->weight;

    double M = a->Saxby - a->Sax*byc - a->Sby*axc + axc*byc*a->weight -
        a->Saybx + a->Say*bxc + a->Sbx*ayc - ayc*bxc*a->weight;
    double N = a->Saxbx - a->Sax*bxc - a->Sbx*axc + axc*bxc*a->weight +
        a->Sayby - a->Say*byc - a->Sby*ayc + ayc*byc*a->weight;

    double theta = atan2(M, N);

    double c = cos(theta), s = sin(theta);
    double dx = (bxc-axc) - c*axc + s*ayc + axc;
    double dy = (byc-ayc) - s*axc - c*ayc + ayc;

    xyt[0] = dx;
    xyt[1] = dy;
    xyt[2] = theta;
}


// This is a symmetrical ICP rule that uses a weak marriage rule to
// improve consistency. Suppose point a (from set A) matches point b
// (from set B). We check that the point that b prefers (call it a')
// is within max_diff_distance_m of a.
//
// tthresh_m, radthresh determine when the optimization
// terminates. (But never to exceed maxiter iterations).
sm_icp_result_t sm_icp(zarray_t *pointsa, zarray_t *pointsb, double *xyt, sm_icp_params_t *params)
{
    int npointsa = zarray_size(pointsa);
    int npointsb = zarray_size(pointsb);

    float *pa = (void*) pointsa->data;
    float *pb = (void*) pointsb->data;

    // transformed b points
    float *Tpb = malloc(npointsb * sizeof(float) * 2);

    // for each transformed point b, which point (index) in a was
    // closest?
    int *aclosest = malloc(npointsa * sizeof(int));
    int *bclosest = malloc(npointsb * sizeof(int));

    struct sm_icp_result result;
    memset(&result, 0, sizeof(result));

    for (result.iters = 0; result.iters < params->maxiters; result.iters++) {
        double c = cos(xyt[2]), s = sin(xyt[2]);

        for (int ib = 0; ib < npointsb; ib++) {
            bclosest[ib] = -1;

            double bx = pb[ib*2 + 0];
            double by = pb[ib*2 + 1];

            double Tbx = bx * c - by * s + xyt[0];
            double Tby = bx * s + by * c + xyt[1];

            Tpb[2*ib + 0] = Tbx;
            Tpb[2*ib + 1] = Tby;

            double best_dist2 = HUGE;

            // find closest point in set a to each point in b (as
            // viewed under current xyt).
            for (int ia = 0; ia < npointsa; ia++) {
                double ax = pa[ia*2 + 0];
                double ay = pa[ia*2 + 1];

                double dist2 = sq(Tbx - ax) + sq(Tby-ay);
                if (dist2 < best_dist2) {
                    best_dist2 = dist2;
                    bclosest[ib] = ia;
                }
            }
        }

        // for all points in a, find closest point in Tpb
        for (int ia = 0; ia < npointsa; ia++) {
            aclosest[ia] = -1;
            double ax = pa[ia*2 + 0];
            double ay = pa[ia*2 + 1];

            double best_dist2 = HUGE;

            for (int ib = 0; ib < npointsb; ib++) {
                double Tbx = Tpb[2*ib + 0];
                double Tby = Tpb[2*ib + 1];

                double dist2 = sq(Tbx - ax) + sq(Tby-ay);
                if (dist2 < best_dist2) {
                    best_dist2 = dist2;
                    aclosest[ia] = ib;
                }
            }
        }

        // consider all the correspondences now.
        double Sax = 0, Say = 0, Sbx = 0, Sby = 0, Saxby = 0, Saybx = 0, Saxbx = 0, Sayby = 0;

        struct align2d a2d;
        align2d_init(&a2d);

        for (int ia = 0; ia < npointsa; ia++) {
            int ib = aclosest[ia];
            if (ib < 0)
                continue;

            double ax = pa[ia*2+0];
            double ay = pa[ia*2+1];
            double bx = pb[ib*2+0];
            double by = pb[ib*2+1];
            double Tbx = Tpb[ib*2+0];
            double Tby = Tpb[ib*2+1];

            // consider correspondence (ia, ib)
            double a2b_dist2 = sq(ax - Tbx) + sq(ay - Tby);

            int ia2 = bclosest[ib];
            double ax2 = pa[ia2*2+0];
            double ay2 = pa[ia2*2+1];

            double b2a_dist2 = sq(ax2 - Tbx) + sq(ay2 - Tby);

            if (sqrt(a2b_dist2) > params->max_dist_ratio*sqrt(b2a_dist2)) {
                aclosest[ia] = -1; // indicate correspondence rejected
                continue;
            }

            // compute correspondences using original point coordinates
            align2d_update(&a2d, bx, by, ax, ay, 1.0); //1.0 / sqrt(a2b_dist2));
        }

        for (int ib = 0; ib < npointsb; ib++) {
            int ia = bclosest[ib];
            if (ia < 0)
                continue;

            double ax = pa[ia*2+0];
            double ay = pa[ia*2+1];
            double bx = pb[ib*2+0];
            double by = pb[ib*2+1];
            double Tbx = Tpb[ib*2+0];
            double Tby = Tpb[ib*2+1];

            // consider correspondence (ia, ib)
            double b2a_dist2 = sq(ax - Tbx) + sq(ay - Tby);

            int ib2 = aclosest[ia];
            double Tbx2 = Tpb[ib2*2+0];
            double Tby2 = Tpb[ib2*2+1];

            double a2b_dist2 = sq(ax - Tbx2) + sq(ay - Tby2);

            if (sqrt(b2a_dist2) > params->max_dist_ratio*sqrt(a2b_dist2)) {
                bclosest[ib] = -1; // indicate correspondence rejected
                continue;
            }

            // compute correspondences using original point coordinates
            align2d_update(&a2d, bx, by, ax, ay, 1.0); //1.0 / sqrt(b2a_dist2));
        }

        double newxyt[3];
        align2d_compute(&a2d, newxyt);

        memcpy(result.xyt, xyt, sizeof(result.xyt));
        result.weight = a2d.weight;

        double dtrans2 = sq(xyt[0]-newxyt[0]) + sq(xyt[1]-newxyt[1]);
        double drad = xyt[2] - newxyt[2];
        if (drad < -M_PI)
            drad += 2*M_PI;
        if (drad > M_PI)
            drad -= 2*M_PI;
        drad = fabs(drad);

        memcpy(xyt, newxyt, sizeof(newxyt));

        // early termination?
        if (dtrans2 < sq(params->trans_thresh_m) && drad < params->rad_thresh)
            break;
    }

    free(Tpb);
    free(bclosest);
    free(aclosest);

    return result;
}
