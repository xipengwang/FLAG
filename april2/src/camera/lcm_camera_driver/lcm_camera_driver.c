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
#include <string.h>

#include <lcm/lcm.h>
#include "lcmtypes/image_t.h"

#include "common/zarray.h"
#include "common/getopt.h"
#include "common/time_util.h"
#include "imagesource/image_source.h"

// returns non-zero on error
static int image_loop(lcm_t *lcm, const char *channel, double hz, image_source_t *isrc)
{
    int64_t last_print = utime_now();
    int frame_count = 0;

    while (1)
    {
        int64_t begin = utime_now();

        image_source_data_t frmd;

        if (isrc->get_frame(isrc, &frmd)) {
            fprintf(stderr, "Error: failed to get frame from image source\n");
            return 1;
        }

        image_t im;
        im.utime       = frmd.utime;
        im.width       = frmd.ifmt.width;
        im.height      = frmd.ifmt.height;
        im.row_stride  = frmd.ifmt.width;
        im.pixelformat = IMAGE_T_PIXEL_FORMAT_INVALID;
        im.size        = (int32_t) frmd.datalen;
        im.data        = (uint8_t*) frmd.data;
        im.nmetadata   = 0;
        im.metadata    = NULL;

        if (!strcmp("BAYER_RGGB", frmd.ifmt.format)) {
            im.pixelformat = IMAGE_T_PIXEL_FORMAT_BAYER_RGGB;
        } else if (!strcmp("BAYER_GBRG", frmd.ifmt.format)) {
            im.pixelformat = IMAGE_T_PIXEL_FORMAT_BAYER_GBRG;
        } else if (!strcmp("GRAY", frmd.ifmt.format)) {
            im.pixelformat = IMAGE_T_PIXEL_FORMAT_GRAY;
        } else if (!strcmp("YUYV", frmd.ifmt.format)) {
            im.pixelformat = IMAGE_T_PIXEL_FORMAT_YUYV;
        } else if (!strcmp("GRAY16", frmd.ifmt.format)) {
            im.pixelformat = IMAGE_T_PIXEL_FORMAT_BE_GRAY16;
            im.row_stride = 2*frmd.ifmt.width;
        } else {
            printf("UNKOWN FRAME TYPE '%s'\n", frmd.ifmt.format);
            assert(0);
        }


        image_t_publish(lcm, channel, &im);

        if (isrc->release_frame(isrc, &frmd)) {
            fprintf(stderr, "Error: failed to release frame from image source\n");
            return 1;
        }

        if (hz > 0.0) {
            timeutil_usleep((1000000/hz) - (utime_now() - begin));
        }

        frame_count++;
        if (begin - last_print > 1000000) {

            double seconds = (double)(begin - last_print) / (double)1000000;
            double fps = (double)frame_count / seconds;

            printf("fps: %f\r\n", fps);

            frame_count = 0;
            last_print = begin;
        }

    }

    return 0;
}

static int args_invalid(getopt_t *gopt)
{
    if (getopt_get_bool(gopt, "help")) {
        fprintf(stderr, "Help requested\n");
        return 1;
    }

    if (strlen(getopt_get_string(gopt, "channel")) < 1) {
        fprintf(stderr, "Invalid channel\n");
        return 1;
    }

    if (zarray_size(getopt_get_extra_args(gopt)) != 1) {
        const zarray_t *eargs = getopt_get_extra_args(gopt);
        fprintf(stderr, "Got %d extra args (1 required). Extra args:\n", zarray_size(eargs));
        for (int i = 0; i < zarray_size(eargs); i++) {
            char *arg = NULL;
            zarray_get(eargs, i, &arg);
            printf("   [%d] '%s'\n", i, arg);
        }

        return 1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    setlinebuf(stdout);
    setlinebuf(stderr);

    int exit_code = 0;
    lcm_t *lcm = lcm_create(NULL);
    image_source_t *isrc = NULL;

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string(gopt, '\0', "channel", "IMAGE", "image_t channel");
    getopt_add_double(gopt, '\0', "hz", "0", "Max. frames per second (0 for unlimited)");

    if (!getopt_parse(gopt, argc, argv, 1) || args_invalid(gopt)) {
        fprintf(stderr, "Usage: %s [options] <camera url>\n", argv[0]);
        getopt_do_usage(gopt);
        exit_code = 1;
        goto cleanup;
    }

    const char *url = NULL;
    zarray_get(getopt_get_extra_args(gopt), 0, &url);

    printf("Opening camera '%s'\n", url);

    isrc = image_source_open(url);
    if (isrc == NULL) {
        fprintf(stderr, "Error: failed to open image source\n");
        exit_code = 1;
        goto cleanup;
    }

    if (isrc->start(isrc)) {
        fprintf(stderr, "Error: failed to start image source\n");
        exit_code = 1;
        goto cleanup;
    }

    if (image_loop(lcm, getopt_get_string(gopt, "channel"), getopt_get_double(gopt, "hz"), isrc)) {
        exit_code = 1;
        // fall through to attempt stop
    }

    if (isrc->stop(isrc)) {
        fprintf(stderr, "Error: failed to stop image source\n");
        exit_code = 1;
        goto cleanup;
    }

cleanup:
    lcm_destroy(lcm);
    getopt_destroy(gopt);
    if (isrc != NULL) { isrc->close(isrc); }
    return exit_code;
}
