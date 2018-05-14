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
#include <stdint.h>

#include <lcm/lcm.h>

#include "apriltag/apriltag.h"
#include "apriltag/tag36artoolkit.h"
#include "apriltag/tag36h10.h"
#include "apriltag/tag36h11.h"
#include "camera/camera.h"
#include "common/config.h"
#include "common/doubles.h"
#include "common/homography.h"
#include "common/http_advertiser.h"
#include "common/getopt.h"
#include "common/image_convert.h"
#include "common/image_u8.h"
#include "common/image_u8x3.h"
#include "common/tic.h"
#include "common/zarray.h"
#include "imagesource/image_convert.h"
#include "vx/vx.h"
#include "vx/webvx.h"


typedef struct {
    vx_world_t *vw;
    webvx_t *webvx;
    getopt_t *getopt;
    lcm_t *lcm;

    apriltag_detector_t *td;
    apriltag_family_t *tf;

    config_t *config;
    calibration_t *calib;
    double tagsize;

    bool spacebar_hit;
} state_t;


int on_event(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
    state_t *state = user;

    if (ev->type == VX_EVENT_KEY_PRESSED) {
        if (ev->u.key.key_code == ' ') {
            state->spacebar_hit = true;
            return 1;
        }
    }

    return 0;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;

    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");

    vx_layer_set_world(vl, state->vw);
    vx_layer_set_elu(vl, (double[]){0, -2, 1.65},
                         (double[]){0, 2, 0},
                         (double[]){0, .37, 0.927}, 0);

    vx_layer_add_event_handler(vl, on_event, 0, state);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{

}

int main(int argc, char *argv[])
{
    state_t *state = calloc(1, sizeof(state_t));

    state->getopt = getopt_create();

    getopt_add_bool(state->getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(state->getopt, 'd', "debug", 0,
                    "Enable debugging output (slow)");
    getopt_add_int(state->getopt, '\0', "border", "1",
                   "Set tag family border size");
    getopt_add_int(state->getopt, 't', "threads", "1",
                   "Use this many CPU threads");
    getopt_add_double(state->getopt, 'x', "decimate", "2.0",
                      "Decimate input image by this factor");
    getopt_add_double(state->getopt, 'b', "blur", "0.0",
                      "Apply low-pass blur to input");
    getopt_add_bool(state->getopt, '0', "refine-edges", 1,
                    "Spend more time trying to align edges of tags");
    getopt_add_bool(state->getopt, '1', "refine-decode", 0,
                    "Spend more time trying to decode tags");
    getopt_add_bool(state->getopt, '2', "refine-pose", 0,
                    "Spend more time trying to precisely localize tags");
    getopt_add_string(state->getopt, 'u', "url", "",
                      "Image source URL");
    getopt_add_int(state->getopt, 'p', "port", "2824",
                   "Port for jsvx");
    getopt_add_string(state->getopt, 'c', "config", "",
                      "Path to config file with camera calibration");
    getopt_add_double(state->getopt, 'f', "focal-length", "770",
                      "Focal length in px (ignored if --config is set)");
    getopt_add_double(state->getopt, 's', "size", "0.167",
                      "Tag size in meters");
    getopt_add_bool(state->getopt, '\0', "pause", 0,
                    "Pause between each frame");

    if (!getopt_parse(state->getopt, argc, argv, 1) ||
        getopt_get_bool(state->getopt, "help") ||
        strlen(getopt_get_string(state->getopt, "url")) == 0) {
        printf("Usage: %s [options] -u <imagesource url>\n", argv[0]);
        getopt_do_usage(state->getopt);
        exit(0);
    }

    int port = getopt_get_int(state->getopt, "port");
    state->lcm = lcm_create(NULL);
    http_advertiser_t * ha = http_advertiser_create(state->lcm, port,
                                                    "AprilTag Viewer",
                                                    "View decoded AprilTags");
    // Init GUI
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(port, NULL, "index.html");

    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas,
                        on_destroy_canvas, state);
    {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
        vx_buffer_add_back(vb,
                           vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                           NULL);
        vx_buffer_swap(vb);
    }
    {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "camera");
        vx_buffer_add_back(vb,
                           vxo_matrix_scale(0.15),
                           vxo_box_solid(vx_gray),
                           NULL);
        vx_buffer_swap(vb);
    }
    printf("started jsvx server on port %d\n",
        getopt_get_int(state->getopt, "port"));


    // Init tag detector
    state->tf = tag36h11_create();
    state->tf->black_border = getopt_get_int(state->getopt, "border");

    state->td = apriltag_detector_create();
    apriltag_detector_add_family(state->td, state->tf);
    state->td->quad_decimate = getopt_get_double(state->getopt, "decimate");
    state->td->quad_sigma = getopt_get_double(state->getopt, "blur");
    state->td->nthreads = getopt_get_int(state->getopt, "threads");
    state->td->debug = getopt_get_bool(state->getopt, "debug");
    state->td->refine_edges = getopt_get_bool(state->getopt, "refine-edges");
    state->td->refine_decode = getopt_get_bool(state->getopt, "refine-decode");
    state->td->refine_pose = getopt_get_bool(state->getopt, "refine-pose");
    state->tagsize = getopt_get_double(state->getopt, "size");

    // Init camera
    const char *url = getopt_get_string(state->getopt, "url");
    if (strlen(url) == 0) {
        zarray_t *urls = image_source_enumerate();
        if (zarray_size(urls) == 0) {
            printf("No image sources found\n");
            return 1;
        }
        printf("Image sources:\n");
        for (int i = 0; i < zarray_size(urls); i += 1) {
            const char *s;
            zarray_get(urls, i, &s);
            printf("    %s\n", s);
        }
    }
    printf("Using image source: %s\n", url);
    image_source_t *isrc = image_source_open(url);
    printf("Formats:\n");
    image_source_format_t ifmt;
    for (int i = 0; i < isrc->num_formats(isrc); i++) {
        isrc->get_format(isrc, i, &ifmt);
        printf("   %3d: %4d x %4d (%s)\n", i, ifmt.width, ifmt.height, ifmt.format);
    }
    isrc->get_format(isrc, 0, &ifmt);

    // Set some default camera parameters
    // (cx, cy will be set when we get know the image dimensions)
    rasterizer_t *rectifier = NULL;
    double fx = getopt_get_double(state->getopt, "focal-length"),
        fy = getopt_get_double(state->getopt, "focal-length"),
        cx = 0,
        cy = 0;

    // Try to load config
    const char *config_path = getopt_get_string(state->getopt, "config");
    if (strlen(config_path) > 0) {
        state->config = config_create_path(config_path);
        state->calib = calibration_load_config(state->config,
                "aprilCameraCalibration.camera0000");

        view_t *mirv = mirv_get_view(mirv_create(state->calib->view));
        rectifier = rasterizer_create_bilinear(state->calib->view, mirv);

        // Get rectified camera params
        matd_t *K = mirv->copy_intrinsics(mirv);
        fx = MATD_EL(K, 0, 0);
        fy = MATD_EL(K, 1, 1);
        cx = MATD_EL(K, 0, 2);
        cy = MATD_EL(K, 1, 2);

        matd_destroy(K);
        mirv->destroy(mirv);
    }

    // Image loop
    isrc->start(isrc);
    while (1) {
        image_source_data_t * frmd = calloc(1, sizeof(image_source_data_t));
        int res = isrc->get_frame(isrc, frmd);
        if (res < 0) {
            printf("get_frame fail: %d\n", res);
            break;
        }

        image_u8x3_t *color_img = image_convert_u8x3(frmd);

        image_u8_t *gray_img;
        if (color_img)
            gray_img = image_u8x3_to_u8(color_img);
        else
            gray_img = image_convert_u8(frmd);

        image_u8_t *rect_img;
        if (rectifier) {
            rect_img = rectifier->rasterize_u8(rectifier, gray_img);
            image_u8_destroy(gray_img);
        } else {
            rect_img = gray_img;

            cx = rect_img->width / 2;
            cy = rect_img->height / 2;
        }

        tic_t tic = tic_begin();
        zarray_t *detections = apriltag_detector_detect(state->td, rect_img);
        int64_t utime = toc_us(&tic);

        // Draw image (limit rendered size to maxw x maxh)
        const double maxw = 1000;
        const double maxh = 1000;
        double image_scale = fmin(1.0, fmin(maxh/rect_img->height,
                                            maxw/rect_img->width));
        {
            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "image");
            vx_buffer_set_draw_order(vb, 20);

            // Use color image if possible
            vx_object_t *im_obj;
            if (!rectifier && color_img)
                im_obj = vxo_image_u8x3(color_img, 0);
            else
                im_obj = vxo_image_u8(rect_img, 0);
            vx_buffer_add_back(vb,
                               vxo_pixcoords(VXO_PIXCOORDS_TOP_LEFT,
                                             VXO_PIXCOORDS_SCALE_MODE_ONE,
                                             vxo_matrix_scale(image_scale),
                                             vxo_matrix_scale3(1, -1, 1),
                                             im_obj,
                                             NULL),
                               NULL);
            vx_buffer_swap(vb);
        }

        // Draw tag detection outlines
        {
            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "detections");
            vx_buffer_set_draw_order(vb, 10);
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                vx_resource_t *xaxis = vx_resource_make_attr_f32_copy(
                    (float[]){det->p[0][0], det->p[0][1],
                            det->p[1][0], det->p[1][1]}, 4, 2);
                vx_resource_t *yaxis = vx_resource_make_attr_f32_copy(
                    (float[]){det->p[0][0], det->p[0][1],
                            det->p[3][0], det->p[3][1]}, 4, 2);
                vx_resource_t *border = vx_resource_make_attr_f32_copy(
                    (float[]){
                            det->p[1][0], det->p[1][1],
                            det->p[2][0], det->p[2][1],
                            det->p[3][0], det->p[3][1]}, 6, 2);

                double ymax = max(max(det->p[0][1], det->p[1][1]),
                                  max(det->p[2][1], det->p[3][1]));

                vx_buffer_add_back(
                    vb,
                    vxo_pixcoords(
                        VXO_PIXCOORDS_TOP_LEFT,
                        VXO_PIXCOORDS_SCALE_MODE_ONE,
                        vxo_matrix_scale(image_scale),
                        vxo_matrix_scale3(1, -1, 1), // Image Y coordinates negative in PIXCOORDS
                        vxo_line_strip(xaxis, vx_green, 2),
                        vxo_line_strip(yaxis, vx_red, 2),
                        vxo_line_strip(border, vx_blue, 2),
                        vxo_chain(vxo_matrix_translate(det->c[0], ymax+30, 0),
                                  vxo_matrix_scale3(1, -1, 1), // Flip text back over
                                  vxo_text(VXO_TEXT_ANCHOR_CENTER,
                                           "<<sansserif-24,center,#0099ff>>"
                                           "id %3d\n(err=%d)\n",
                                           det->id, det->hamming),
                                  NULL),
                        NULL),
                    NULL);
            }

            vx_buffer_swap(vb);
        }

        // Draw tags in 3d projection
        {
            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "tags");
            vx_buffer_set_draw_order(vb, 30);
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                // NOTE: Still have to pass in a negative fx,
                // due to the OpenGL conventions.
                // In homogeneous coordinates, [x y -z] results in
                // a sign flip when projected to image pixel coordinates
                // [u v] = [-x/z -y/z].
                // The sign flip in the u axis is undone by
                // passing a negative fx.
                // The y needs to be sign flipped because the
                // image v axis and the camera y axis are flipped.
                matd_t *M = homography_to_pose(det->H, -fx, fy, cx, cy);
                double scale = state->tagsize / 2.0;
                MATD_EL(M, 0, 3) *= scale;
                MATD_EL(M, 1, 3) *= scale;
                MATD_EL(M, 2, 3) *= scale;

                image_u8_t *tag_im = apriltag_to_image(det->family, det->id);

                // subtract out the width of the 1px white border
                double image_scale = state->tagsize / (tag_im->width - 2);
                vx_buffer_add_back(vb,
                                   vxo_matrix_rotatex(M_PI/2),
                                   vxo_matrix(M->data),
                                   vxo_matrix_scale(image_scale),
                                   vxo_matrix_translate(-tag_im->width/2,
                                                        -tag_im->width/2, 0),
                                   vxo_image_u8(tag_im, 0),
                                   // Put a black backing on the tags to give them a "front"
                                   vxo_matrix_translate(tag_im->width/2,
                                                        tag_im->width/2,
                                                        0.01),
                                   vxo_matrix_scale(tag_im->width),
                                   vxo_square_solid(vx_black),
                                   NULL);

                if (0) {
                    printf("Tag corners:\n");
                    for (int j = 0; j < 4; j += 1)
                        printf("%d: %15f %15f\n",
                                j, det->p[j][0], det->p[j][1]);
                    printf("Transform:\n");
                    matd_print(M, "%10f");
                    printf("RPY:\n");

                    double quat[4];
                    double rpy[3];
                    doubles_mat_to_quat(M->data, quat);
                    doubles_quat_to_rpy(quat, rpy);
                    doubles_print(rpy, 3, "%10f");
                    printf("\n");
                }

                matd_destroy(M);
                image_u8_destroy(tag_im);
            }

            vx_buffer_swap(vb);
        }

        // Draw elapsed time
        {
            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "time");
            vx_buffer_add_back(vb,
                               vxo_pixcoords(
                                   VXO_PIXCOORDS_BOTTOM_RIGHT,
                                   VXO_PIXCOORDS_SCALE_MODE_ONE,
                                   vxo_text(VXO_TEXT_ANCHOR_BOTTOM_RIGHT,
                                            "%.2f ms", utime/1000.0),
                                   NULL),
                               NULL);
            vx_buffer_swap(vb);
        }


        apriltag_detections_destroy(detections);
        image_u8_destroy(rect_img);
        image_u8x3_destroy(color_img);
        isrc->release_frame(isrc, frmd);

        if (getopt_get_bool(state->getopt, "pause")) {
            while (!state->spacebar_hit)
                timeutil_usleep(10000);
            state->spacebar_hit = false;
        }
    }

    apriltag_detector_destroy(state->td);
    tag36h11_destroy(state->tf);
    if (rectifier)
        rectifier->destroy(rectifier);
    http_advertiser_destroy(ha);
}
