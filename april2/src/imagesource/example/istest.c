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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "common/zarray.h"

#include "image_source.h"

#define BUFSIZE (1024*1024)

int main(int argc, char *argv[])
{
    char *url = NULL;

    if (argc > 1) {
        // if URL is provided on command line, use that.
        url = argv[1];
    } else {
        // otherwise, show all cameras and use the first one.

        zarray_t *urls = image_source_enumerate();

        printf("Cameras:\n");
        for (int i = 0; i < zarray_size(urls); i++) {
            char *url;
            zarray_get(urls, i, &url);
            printf("  %3d: %s\n", i, url);
        }

        if (zarray_size(urls) == 0) {
            printf("Found no cameras.\n");
            return -1;
        }

        zarray_get(urls, 0, &url);
    }

    image_source_t *isrc = image_source_open(url);

    if (isrc == NULL) {
        printf("Error opening device.\n");
        return -1;
    }

    printf("Formats:\n");
    for (int i = 0; i < isrc->num_formats(isrc); i++) {
        image_source_format_t ifmt;
        isrc->get_format(isrc, i, &ifmt);
        printf("   %3d: %4d x %4d (%s)\n", i, ifmt.width, ifmt.height, ifmt.format);
    }

    if (1) {
        int res = isrc->start(isrc);
        printf("start: res = %d\n", res);
    }

    int nframes = 0;

//    setlinebuf(stdout);

    while(1) {
        image_source_data_t * frmd = calloc(1, sizeof(image_source_data_t));
        int res = isrc->get_frame(isrc, frmd);
        if (res < 0) {
            printf("get_frame fail: %d\n", res);
            continue;
        } else {
            nframes++;
        }

        printf("get_frame: res = %d count = %10d (%10d bytes)\r", res, nframes, frmd->datalen);
        fflush(stdout);
        isrc->release_frame(isrc, frmd);
    }
    return 0;
}

