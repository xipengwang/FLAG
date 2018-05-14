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

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <lcm/lcm.h>
#include "lcmtypes/image_t.h"
#include "lcmtypes/tag_detection_list_t.h"

#include "common/getopt.h"
#include "common/tic.h"
#include "common/image_convert.h"

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"

typedef struct
{
    lcm_t                  *lcm;
    char                   *image_channel;
    char                   *tag_channel;
    double                  max_fps;
    int64_t                 last_detection;

    apriltag_detector_t    *detector;

} tag_detector_t;

static void image_cb(const lcm_recv_buf_t *rbuf, const char *channel,
                     const image_t *msg, void *user)
{
    tag_detector_t *td = user;

    ////////////////////////////////////////
    // rate limit detections

    double time_since_last_detect = (msg->utime - td->last_detection)*1e-6;
    if (time_since_last_detect < 1.0/td->max_fps && // rate limiting
        time_since_last_detect > -1.0) // time warp detection (log playback)
    {
        return;
    }

    td->last_detection = msg->utime;

    ////////////////////////////////////////
    // debayer image

    tic_t tic = tic_begin();

    image_u8x4_t *im_u8x4 = image_t_to_image_u8x4(msg);
    image_u8_t *im = image_u8x4_to_u8(im_u8x4);
    //image_u8_t im_u8 = { .width = msg->width, .height = msg->height, .stride = msg->row_stride, .buf = msg->data };
    //image_u8_t *im = image_u8_copy(&im_u8);

    image_u8x4_destroy(im_u8x4);

    double debayer_s = toctic_s(&tic);

    ////////////////////////////////////////
    // detect

    zarray_t *detections = apriltag_detector_detect(td->detector, im);
    int ndetections = zarray_size(detections);

    // debugging
    if (0 && ndetections == 0) {
        char *path = sprintf_alloc("/tmp/IMAGE_%" PRId64 ".pnm", utime_now()); // blacklist-ignore
        image_u8_write_pnm(im, path);
        free(path);
    }

    ////////////////////////////////////////
    // publish on LCM

    tag_detection_list_t *detection_list = calloc(1, sizeof(tag_detection_list_t));
    detection_list->utime       = (int64_t) msg->utime;
    detection_list->ndetections = (int32_t) ndetections;
    detection_list->detections  = calloc(detection_list->ndetections, sizeof(tag_detection_t));

    for (int i = 0; i < ndetections; i++)
    {
        apriltag_detection_t *atd = NULL;
        zarray_get(detections, i, &atd);

        tag_detection_t *det = &detection_list->detections[i];

        det->tag_family_bit_width        = (int8_t)  atd->family->d;
        det->tag_family_min_hamming_dist = (int8_t)  atd->family->h;
        det->id                          = (int32_t) atd->id;
        det->hamming_dist                = (int8_t)  atd->hamming;
        det->goodness                    = (float)   atd->goodness;

        assert(atd->H->nrows == 3);
        assert(atd->H->ncols == 3);
        det->H[0][0]                     = (float)   atd->H->data[0*atd->H->ncols + 0];
        det->H[0][1]                     = (float)   atd->H->data[0*atd->H->ncols + 1];
        det->H[0][2]                     = (float)   atd->H->data[0*atd->H->ncols + 2];
        det->H[1][0]                     = (float)   atd->H->data[1*atd->H->ncols + 0];
        det->H[1][1]                     = (float)   atd->H->data[1*atd->H->ncols + 1];
        det->H[1][2]                     = (float)   atd->H->data[1*atd->H->ncols + 2];
        det->H[2][0]                     = (float)   atd->H->data[2*atd->H->ncols + 0];
        det->H[2][1]                     = (float)   atd->H->data[2*atd->H->ncols + 1];
        det->H[2][2]                     = (float)   atd->H->data[2*atd->H->ncols + 2];

        det->cxy[0]                      = (float)   atd->c[0];
        det->cxy[1]                      = (float)   atd->c[1];

        det->pxy[0][0]                   = (float)   atd->p[0][0];
        det->pxy[0][1]                   = (float)   atd->p[0][1];
        det->pxy[1][0]                   = (float)   atd->p[1][0];
        det->pxy[1][1]                   = (float)   atd->p[1][1];
        det->pxy[2][0]                   = (float)   atd->p[2][0];
        det->pxy[2][1]                   = (float)   atd->p[2][1];
        det->pxy[3][0]                   = (float)   atd->p[3][0];
        det->pxy[3][1]                   = (float)   atd->p[3][1];
    }

    // publish
    tag_detection_list_t_publish(td->lcm, td->tag_channel, detection_list);

    double detect_s = toctic_s(&tic);

    printf("%8.3f ms to debayer (%dx%d), %8.3f ms to detect (%d detections)\n",
           debayer_s*1e3, im->width, im->height,
           detect_s*1e3, ndetections);

    // clean up
    zarray_vmap(detections, apriltag_detection_destroy);
    zarray_destroy(detections);

    tag_detection_list_t_destroy(detection_list);

    image_u8_destroy(im);
}

static int args_invalid(getopt_t *gopt)
{
    if (getopt_get_bool(gopt, "help")) {
        fprintf(stderr, "Help requested\n");
        return 1;
    }

    if (strlen(getopt_get_string(gopt, "image-channel")) < 1) {
        fprintf(stderr, "Invalid image subscription channel\n");
        return 1;
    }

    if (getopt_get_double(gopt, "max-fps") < 0) {
        fprintf(stderr, "Invalid max framerate\n");
        return 1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    setlinebuf(stdout);
    setlinebuf(stderr);

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");
    getopt_add_string(gopt, 'i', "image-channel", "IMAGE", "Image channel");
    getopt_add_string(gopt, 'p', "tag-channel", "TAG_DETECTIONS", "Tag detection channel");
    getopt_add_double(gopt, 'f', "max-fps", "1", "Max framerate");

    if (!getopt_parse(gopt, argc, argv, 1) || args_invalid(gopt)) {
        fprintf(stderr, "Usage: %s [options]\n", argv[0]);
        getopt_do_usage(gopt);
        getopt_destroy(gopt);
        exit(1);
    }

    tag_detector_t *td = calloc(1, sizeof(tag_detector_t));
    td->lcm           = lcm_create(NULL);
    td->image_channel = strdup(getopt_get_string(gopt, "image-channel"));
    td->tag_channel   = strdup(getopt_get_string(gopt, "tag-channel"));
    td->max_fps       = getopt_get_double(gopt, "max-fps");

    td->detector = apriltag_detector_create();
    apriltag_detector_add_family(td->detector, tag36h11_create());  // Leak XXX
    //td->detector->debug = 1;
    td->detector->nthreads = 1;
    td->detector->quad_decimate = 2.0;
    td->detector->quad_sigma = 0.0;

    image_t_subscribe(td->lcm, td->image_channel, image_cb, td);

    while (1) {
        lcm_handle(td->lcm);
    }

    getopt_destroy(gopt);
    lcm_destroy(td->lcm);
    free(td->image_channel);
    free(td->tag_channel);
    apriltag_detector_destroy(td->detector);
    free(td);
}
