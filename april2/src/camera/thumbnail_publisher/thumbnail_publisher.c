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

#include "common/getopt.h"
#include "common/image_u8x4.h"
#include "common/tic.h"
#include "common/image_convert.h"

typedef struct _thumbnail_publisher thumbnail_publisher_t;
struct _thumbnail_publisher
{
    lcm_t  *lcm;
    char   *input_channel;
    char   *thumb_channel;
    int     jpeg_quality;
    double  max_fps;
    int64_t last_thumb;

    int64_t timing_sums[2];
    int     timing_count;
    int64_t timing_last_print;
};

static uint8_t *read_fully(const char *path, int64_t *_len)
{
    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        return NULL;
    }

    if (fseek(fp, 0L, SEEK_END)) {
        fprintf(stderr, "Error: fseek end failed\n");
        fclose(fp);
        return NULL;
    }

    int64_t file_length = ftell(fp);
    if (file_length < 0) {
        fprintf(stderr, "Error: ftell failed\n");
        fclose(fp);
        return NULL;
    }

    uint8_t *buf = malloc(file_length+1);
    memset(buf, 0, file_length+1);

    if (fseek(fp, 0L, SEEK_SET)) {
        fprintf(stderr, "Error: fseek start failed\n");
        fclose(fp);
        free(buf);
        return NULL;
    }

    size_t bytes_read = fread(buf, sizeof(uint8_t),
                              (size_t) (file_length+1), fp);
    if (bytes_read != file_length) {
        fprintf(stderr, "Error: fread failed (read %zu expected %" PRId64 ")\n",
                bytes_read, file_length);
        fclose(fp);
        free(buf);
        return NULL;
    }

    fclose(fp);

    *_len = file_length+1;
    return buf;
}

static void image_cb(const lcm_recv_buf_t *rbuf, const char *channel, const image_t *msg, void *userdata)
{
    thumbnail_publisher_t *tp = userdata;

    double time_since_last_thumb = (msg->utime - tp->last_thumb)*1e-6;
    if ((time_since_last_thumb < 1.0/tp->max_fps) && // recent thumb and
        (time_since_last_thumb > -1))                // not a time warp
    {
        return;
    }

    tp->last_thumb = msg->utime;

    char *orig_path = sprintf_alloc("/tmp/%s-ORIG.pnm", tp->thumb_channel);
    char *jpeg_path = sprintf_alloc("/tmp/%s-JPEG.jpg", tp->thumb_channel);

    // debayer image
    tic_t tic = tic_begin();
    image_u8x4_t *im = image_t_to_image_u8x4_half(msg);
    image_u8x4_write_pnm(im, orig_path);
    image_u8x4_destroy(im);
    tp->timing_sums[0] += toctic_us(&tic);

    // jpeg compress
    char *args = "";
    if (0)
        args = "-scale 50\%";
    char *cmd = sprintf_alloc("convert %s -quality %d %s %s\n",
                              orig_path, tp->jpeg_quality, args, jpeg_path);
    int system_result = system(cmd);
    free(cmd);

    if (system_result) {
        fprintf(stderr, "Error: image conversion system call failed\n");
        exit(1);
    }

    // read compressed image
    int64_t len = -1;
    uint8_t *buf = read_fully(jpeg_path, &len);
    tp->timing_sums[1] += toctic_us(&tic);

    // publish
    image_t out;
    out.utime       = msg->utime;
    out.width       = im->width;
    out.height      = im->height;
    out.row_stride  = out.width;
    out.pixelformat = IMAGE_T_PIXEL_FORMAT_MJPEG;
    out.size        = len;
    out.data        = buf;
    out.nmetadata   = 0;
    out.metadata    = NULL;

    image_t_publish(tp->lcm, tp->thumb_channel, &out);
    free(buf);

    // (potentially) print timing
    tp->timing_count++;

    double time_since_last_print = (msg->utime - tp->timing_last_print)*1e-6;
    if (time_since_last_print < -1 || // time warp
        time_since_last_print > 1)    // 1Hz printing
    {
        printf("Timing avgs (%d updates): de-Bayer %5.1f ms, jpeg %5.1f ms\n",
               tp->timing_count,
               tp->timing_sums[0]*1e-3/tp->timing_count,
               tp->timing_sums[1]*1e-3/tp->timing_count);

        tp->timing_sums[0]    = 0;
        tp->timing_sums[1]    = 0;
        tp->timing_count      = 0;
        tp->timing_last_print = msg->utime;
    }
}

static int args_invalid(getopt_t *gopt)
{
    if (getopt_get_bool(gopt, "help")) {
        fprintf(stderr, "Help requested\n");
        return 1;
    }

    if (strlen(getopt_get_string(gopt, "input-channel")) < 1) {
        fprintf(stderr, "Invalid subscription channel\n");
        return 1;
    }

    if (strlen(getopt_get_string(gopt, "thumb-channel")) < 1) {
        fprintf(stderr, "Invalid thumbnail channel\n");
        return 1;
    }

    if (getopt_get_int(gopt, "quality") < 0 ||
        getopt_get_int(gopt, "quality") > 100)
    {
        fprintf(stderr, "Invalid JPEG quality\n");
        return 1;
    }

    if (getopt_get_double(gopt, "max-fps") < 0) {
        fprintf(stderr, "Invalid max framerate\n");
        return 1;
    }

    if (zarray_size(getopt_get_extra_args(gopt)) != 0) {
        const zarray_t *eargs = getopt_get_extra_args(gopt);
        fprintf(stderr, "Got %d extra args (expected 0). Extra args:\n", zarray_size(eargs));
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

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");
    getopt_add_string(gopt, 'i', "input-channel", "IMAGE", "Input image channel");
    getopt_add_string(gopt, 't', "thumb-channel", "THUMBS", "Thumbnail channel");
    getopt_add_int(gopt, 'q', "quality", "15", "Thumbnail quality (0-100)");
    getopt_add_double(gopt, 'f', "max-fps", "30", "Max framerate");

    if (!getopt_parse(gopt, argc, argv, 1) || args_invalid(gopt)) {
        fprintf(stderr, "Usage: %s [options]\n", argv[0]);
        getopt_do_usage(gopt);
        getopt_destroy(gopt);
        exit(1);
    }

    thumbnail_publisher_t *tp = calloc(1, sizeof(thumbnail_publisher_t));
    tp->lcm           = lcm_create(NULL);
    tp->input_channel = strdup(getopt_get_string(gopt, "input-channel"));
    tp->thumb_channel = strdup(getopt_get_string(gopt, "thumb-channel"));
    tp->jpeg_quality  = getopt_get_int(gopt, "quality");
    tp->max_fps       = getopt_get_double(gopt, "max-fps");

    image_t_subscribe(tp->lcm, tp->input_channel, image_cb, tp);

    while (1) {
        lcm_handle(tp->lcm);
    }

    getopt_destroy(gopt);
    lcm_destroy(tp->lcm);
    free(tp->input_channel);
    free(tp->thumb_channel);
    free(tp);
}
