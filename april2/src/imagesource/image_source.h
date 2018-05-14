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

#ifndef _IMAGE_SOURCE_H
#define _IMAGE_SOURCE_H

#include <stdint.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "common/zarray.h"

#include "url_parser.h"

#define IMAGE_SOURCE_MAX_FORMAT_LENGTH 32

typedef struct image_source_format image_source_format_t;
struct image_source_format
{
    // dimensions of the image in pixels
    int   width, height;

    // human readable string. Use fourcc codes whenever possible
    char  format[IMAGE_SOURCE_MAX_FORMAT_LENGTH];
};

typedef struct image_source_data image_source_data_t;
struct image_source_data
{
    image_source_format_t ifmt;

    // acquisition time of the frame (to the best of the driver's
    // knowledge), relative to this computer's clock.
    uint64_t utime;

    // the image data.
    void *data;
    int  datalen;

    void *priv;
};

typedef struct image_source image_source_t;
struct image_source
{
    int   impl_type;
    void *impl;

    ///////////////////////////////////////////////////
    // Formats

    // how many formats are supported by this camera?
    int (*num_formats)(image_source_t *isrc);

    // user allocates an image_source_format and the driver fills it in.
    // note: formats can change over time, so this (at best) is a guess.
    void (*get_format)(image_source_t *isrc, int idx, image_source_format_t *fmt);

    // returns non-zero on error.
    int (*set_format)(image_source_t *isrc, int idx);

    // returns non-zero on error.
    int (*set_named_format)(image_source_t *isrc, const char *desired_format);

    // return the index of the current format.
    int (*get_current_format)(image_source_t *isrc);

    ///////////////////////////////////////////////////
    // Acquisition control

    // returns non-zero on error.
    int (*start)(image_source_t *isrc);

    // returns non-zero on error.
    int (*get_frame)(image_source_t *isrc, image_source_data_t *frmd);

    // returns non-zero on error.
    int (*release_frame)(image_source_t *isrc, image_source_data_t *frmd);

    // returns non-zero on error.
    int (*stop)(image_source_t *isrc);

    ///////////////////////////////////////////////////
    // Feature control

    int (*num_features)(image_source_t *isrc);
    const char* (*get_feature_name)(image_source_t *isrc, int idx);
    int (*is_feature_available)(image_source_t *isrc, int idx);

    // string is allocated by driver, to be freed by user. (The
    // options available for a feature can change depending on the
    // settings of other features.)
    // "b"  boolean
    // "i"  integer
    // "i,min,max"
    // "i,min,max,increment"
    // "f,min,max"
    // "f,min,max,increment"
    // "c,0=apple,3=banana,5=orange,"    <-- note: should be robust to extra commas
    char* (*get_feature_type)(image_source_t *isrc, int idx);

    double (*get_feature_value)(image_source_t *isrc, int idx);

    // returns non-zero on error
    int (*set_feature_value)(image_source_t *isrc, int idx, double v);

    void (*print_info)(image_source_t *isrc);

    int (*close)(image_source_t *isrc);
};

struct image_source_provider
{
  const char *protocol; // e.g. "file://"

  image_source_t* (*open)(url_parser_t *urlp);
  void (*enumerate)(zarray_t *urls);

  struct image_source_provider *next;
};

extern struct image_source_provider *image_source_provider_head;

image_source_t *image_source_open(const char *url);
zarray_t *image_source_enumerate();
void image_source_enumerate_free(zarray_t *urls);

#endif

