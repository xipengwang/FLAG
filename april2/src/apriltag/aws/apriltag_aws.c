/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

   Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan. */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag36h10.h"
#include "apriltag/tag36artoolkit.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag25h7.h"

#include "common/image_u8.h"
#include "common/image_u8x4.h"
#include "common/zarray.h"
#include "common/getopt.h"
#include "common/pjpeg.h"
#include "common/json.h"

/* The state must be seeded so that it is not everywhere zero. */
static uint32_t xorshift128plus_32(uint64_t s[2])
{
    uint64_t x = s[0];
    uint64_t const y = s[1];
    s[0] = y;
    x ^= x << 23; // a
    s[1] = x ^ y ^ (x >> 17) ^ (y >> 26); // b, c

    uint64_t res64 = s[1] + y;
    uint32_t res32 = (res64 >> 32) ^ (res64);
    return res32;
}

void randf_normals(uint64_t state[2], float *out, int nout)
{
    int outpos = 0;

    while (outpos < nout) {

        float s = 0.0f;
        float u, v, R2;

        while (s == 0.0f || s >= 1.0f) {

            // Sample two values uniformly from (-1, +1)
            u = (1.0*xorshift128plus_32(state) / UINT32_MAX) * 2 - 1;
            v = (1.0*xorshift128plus_32(state) / UINT32_MAX) * 2 - 1;

            R2 = u*u + v*v;
            s = R2;
        }

        float factor = sqrt( (-2*log(s)) / (s) );

        out[outpos++] = u * factor;
        if (outpos < nout)
            out[outpos++] = v * factor;
    }
}


/*
static void add_dir_recursive(zarray_t *paths, const char *dirpath)
{
    DIR *d = opendir(dirpath);

    if (!d) {
        perror("opendir");
        return;
    }

    while (true) {
        struct dirent *ent = readdir(d);
        if (!ent)
            break;

        if (ent->d_name[0] == '.')
            continue;

        if (strstr(ent->d_name, "mosaic"))
            continue;

        char *p = sprintf_alloc("%s/%s", dirpath, ent->d_name);

        struct stat st;
        if (stat(p, &st)) {
            perror("stat");
            continue;
        }

        if (S_ISREG(st.st_mode))
            zarray_add(paths, &p);
        if (S_ISDIR(st.st_mode))
            add_dir_recursive(paths, p);
    }

    closedir(d);
}
*/

// Invoke:
//
// tagtest [options] input.pnm

int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 'i', "iters", "1", "Repeat processing on input set this many times");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input; negative sharpens");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");
    getopt_add_string(getopt, 'j', "json-output-path", "/tmp/out.json", "Write results to the path");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options] <input files>\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    const zarray_t *extra = getopt_get_extra_args(getopt);
    zarray_t *inputs = zarray_create(sizeof(char*));
    for (int i = 0; i < zarray_size(extra); i++) {
        const char *path;
        zarray_get(extra, i, &path);

        zarray_add(inputs, &path);
    }

    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11"))
        tf = tag36h11_create();
    else if (!strcmp(famname, "tag36h10"))
        tf = tag36h10_create();
    else if (!strcmp(famname, "tag36artoolkit"))
        tf = tag36artoolkit_create();
    else if (!strcmp(famname, "tag25h9"))
        tf = tag25h9_create();
    else if (!strcmp(famname, "tag25h7"))
        tf = tag25h7_create();
    else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    tf->black_border = getopt_get_int(getopt, "border");

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    // we will use the same noise field for every image. Precompute it
    // (because computing normals code is pretty slow...)
    int nnormals = 16*1024*1024;
    float *normals = calloc(nnormals, sizeof(float));
    if (1) {
        uint64_t state[] = { 12345, 678901 };
        randf_normals(state, normals, nnormals);
    }

    const double degrade0 = 0;
    const double degrade1 = 2;
    const double degrade_step = .1;
    const int    ndegrade = (degrade1 - degrade0) / degrade_step;

    json_object_t *results_array_json = json_array_create();

    char *path;
    zarray_get(inputs, 0, &path);

    image_u8_t *orig_im = NULL;
    if (str_ends_with(path, "pnm") || str_ends_with(path, "PNM"))
        orig_im = image_u8_create_from_pnm(path);
    else if (str_ends_with(path, "jpg") || str_ends_with(path, "JPG")) {
        int err = 0;
        pjpeg_t *pjpeg = pjpeg_create_from_file(path, 0, &err);
        if (pjpeg == NULL) {
            printf("pjpeg error %d\n", err);
            return 1;
        }

        orig_im = pjpeg_to_u8_baseline(pjpeg);
    }

    if (orig_im == NULL) {
        printf("couldn't load %s\n", path);
        exit(1);
    }

    for (int degrade_idx = 0; degrade_idx < ndegrade; degrade_idx++) {
        double degrade = degrade0 + degrade_step * degrade_idx;

        image_u8_t *im = image_u8_copy(orig_im);

        ////////////////////////////////////////////////////
        // degrade the image

        // resize the image
        if (1) {
            double scale = pow(0.4, degrade);

            int outw = im->width * scale;
            int outh = im->height * scale;

            image_u8_t *out = image_u8_create(outw, outh);

            for (int outy = 0; outy < outh; outy++) {
                int iny = im->height * (outy + .5) / outh;
                for (int outx = 0; outx < outw; outx++) {
                    int inx = im->width * (outx + .5) / outw;

                    out->buf[outy*out->stride + outx] = im->buf[iny*im->stride + inx];
                }
            }

            image_u8_destroy(im);
            im = out;
        }

        // blur.
        if (1) {
            double sigma = degrade * 4;
            image_u8_gaussian_blur(im, sigma, (int) ((3 * sigma + 3)) | 1);
        }

        // reduce dynamic range.
        if (1) {
            double range = pow(.5, degrade);

            assert(range <= 1);

            int w = im->width, h = im->height;

            for (int y = 0; y < h; y++) {
                for (int x = 0; x < w; x++) {
                    im->buf[y*im->stride + x] *= range;
                }
            }
        }

        // add noise
        if (1) {
            double sigma = 20 * degrade;

            int w = im->width, h = im->height;

            for (int y = 0; y < h; y++) {
                for (int x = 0; x < w; x++) {
                    double v = im->buf[y*im->stride + x];
                    v += sigma * normals[(y*im->width + x) % nnormals];
                    if (v < 0)
                        v = 0;
                    if ( v > 255)
                        v = 255;

                    im->buf[y*im->stride + x] = v;
                }
            }
        }

//                image_u8_write_pnm(im, "debug_degrade.pnm");

        //////////////////////////////////////////////
        // perform the detections

        int64_t utime0 = utime_now();

        zarray_t *detections = apriltag_detector_detect(td, im);

        int64_t utime1 = utime_now();

        json_object_t *degrade_json = json_hash_create();
        json_array_add(results_array_json, degrade_json);
        json_hash_add(degrade_json, "degrade", json_number_create(degrade));
        json_hash_add(degrade_json, "nquads", json_number_create(td->nquads));
        json_hash_add(degrade_json, "time", json_number_create((utime1 - utime0) / 1.0E6));

        json_object_t *detections_array_json = json_array_create();
        json_hash_add(degrade_json, "detections", detections_array_json);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            json_object_t *detection_json = json_hash_create();
            json_array_add(detections_array_json, detection_json);

            json_hash_add(detection_json, "id", json_number_create(det->id));
            json_hash_add(detection_json, "hamming", json_number_create(det->hamming));
//            json_hash_add(detection_json, "goodness", json_number_create(det->goodness));
            json_hash_add(detection_json, "margin", json_number_create(det->decision_margin));

            json_object_t *points_json = json_array_create();
            json_hash_add(detection_json, "p", points_json);
            for (int j = 0; j < 4; j++) {
                json_object_t *point_json = json_array_create();
                json_array_add(points_json, point_json);
                json_array_add(point_json, json_number_create(det->p[j][0]));
                json_array_add(point_json, json_number_create(det->p[j][1]));
            }
        }

        apriltag_detections_destroy(detections);

        image_u8_destroy(im);
    } // degrade

// don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(td);

    tag36h11_destroy(tf);

    json_object_write_to_file(results_array_json, getopt_get_string(getopt, "json-output-path"));
    json_object_destroy(results_array_json);

    return 0;
}
