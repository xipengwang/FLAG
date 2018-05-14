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

#include "vx.h"

#include "vxo_generic.h"

vx_object_t *vxo_image_u8(image_u8_t *im, uint32_t flags)
{
    assert(im != NULL);

    return vxo_image(vx_resource_make_texture_u8_copy(im, flags));
}

vx_object_t *vxo_image_u8x3(image_u8x3_t *im, uint32_t flags)
{
    assert(im != NULL);

    return vxo_image(vx_resource_make_texture_u8x3_copy(im, flags));
}

vx_object_t *vxo_image_u8x4(image_u8x4_t *im, uint32_t flags)
{
    return vxo_image(vx_resource_make_texture_u8x4_copy(im, flags));
}

vx_object_t *vxo_image(vx_resource_t *tex)
{
    float w = vx_resource_texture_get_width(tex), h = vx_resource_texture_get_height(tex);
    float tw = 1.0 * vx_resource_texture_get_bpp(tex) * w / vx_resource_texture_get_stride(tex), th = 1.0;
    vx_resource_t *position_resource = vx_resource_make_attr_f32_copy((float[]) { 0, 0,  w, 0,  0, h,  w, h }, 8, 2);
    vx_resource_t *texcoord_resource = vx_resource_make_attr_f32_copy((float[]) { 0, 0,  tw, 0,  0, th, tw, th }, 8, 2);

    vx_lock();

    static vx_resource_t *program_resource = NULL;

    if (program_resource == NULL) {

        char vertex_shader_src[] =
            "attribute vec2 aposition; \n"  \
            "attribute vec2 atexcoord; \n"  \
            "varying vec2 vtexcoord; \n"   \
            "uniform mat4 VX_P;\n"         \
            "uniform mat4 VX_V;\n"         \
            "uniform mat4 VX_M;\n"         \
            "void main(void) {\n"          \
            "  vtexcoord = atexcoord; \n " \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(aposition, 0, 1.0);\n" \
            "}";

        char fragment_shader_src[] =
            "precision mediump float; \n"   \
            "varying vec2 vtexcoord; \n"         \
            "uniform sampler2D texture; \n" \
            "void main(void) {\n"           \
            "  vec4 c = texture2D(texture, vtexcoord);\n" \
            "  gl_FragColor = vec4(c.r, c.g, c.b, c.a);\n" \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="aposition", .resource=position_resource },
                                  { .name="atexcoord", .resource=texcoord_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  { .name="texture", .resource = tex },
                                  { .name = NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_TRIANGLE_STRIP, .first = 0, .count = 4 },
                                  { .count = 0 }, });
}

vx_object_t *vxo_image_saturated(vx_resource_t *tex, float sat)
{
    float w = vx_resource_texture_get_width(tex), h = vx_resource_texture_get_height(tex);
    float tw = 1.0 * vx_resource_texture_get_bpp(tex) * w / vx_resource_texture_get_stride(tex), th = 1.0;
    vx_resource_t *position_resource = vx_resource_make_attr_f32_copy((float[]) { 0, 0,  w, 0,  0, h,  w, h }, 8, 2);
    vx_resource_t *texcoord_resource = vx_resource_make_attr_f32_copy((float[]) { 0, 0,  tw, 0,  0, th, tw, th }, 8, 2);

    vx_lock();

    static vx_resource_t *program_resource = NULL;

    if (program_resource == NULL) {

        char vertex_shader_src[] =
            "attribute vec2 aposition; \n"  \
            "attribute vec2 atexcoord; \n"  \
            "varying vec2 vtexcoord; \n"   \
            "uniform mat4 VX_P;\n"         \
            "uniform mat4 VX_V;\n"         \
            "uniform mat4 VX_M;\n"         \
            "void main(void) {\n"          \
            "  vtexcoord = atexcoord; \n " \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(aposition, 0, 1.0);\n" \
            "}";

        char fragment_shader_src[] =
            "precision mediump float; \n"   \
            "varying vec2 vtexcoord; \n"         \
            "uniform sampler2D texture; \n" \
            "uniform float sat; \n" \
            "void main(void) {\n"           \
            "  vec4 c = texture2D(texture, vtexcoord);\n" \
            "  gl_FragColor = vec4(sat*c.r, sat*c.g, sat*c.b, 1);\n" \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="sat", .nrows = 1, .ncols = 1, .data=&sat  },
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="aposition", .resource=position_resource },
                                  { .name="atexcoord", .resource=texcoord_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  { .name="texture", .resource = tex },
                                  { .name = NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_TRIANGLE_STRIP, .first = 0, .count = 4 },
                                  { .count = 0 }, });
}

vx_object_t *vxo_image_tile(vx_resource_t *tex, float rgba0[4], float rgba1[4], float rgba2[4], float rgba3[4])
{
    float w = vx_resource_texture_get_width(tex), h = vx_resource_texture_get_height(tex);
    float tw = 1.0 * vx_resource_texture_get_bpp(tex) * w / vx_resource_texture_get_stride(tex), th = 1.0;
    vx_resource_t *position_resource = vx_resource_make_attr_f32_copy((float[]) { 0, 0,  w, 0,  0, h,  w, h }, 8, 2);
    vx_resource_t *texcoord_resource = vx_resource_make_attr_f32_copy((float[]) { 0, 0,  tw, 0,  0, th, tw, th }, 8, 2);

    vx_lock();

    static vx_resource_t *program_resource = NULL;

    if (program_resource == NULL) {

        char vertex_shader_src[] =
            "attribute vec2 aposition; \n"  \
            "attribute vec2 atexcoord; \n"  \
            "varying vec2 vtexcoord; \n"   \
            "uniform mat4 VX_P;\n"         \
            "uniform mat4 VX_V;\n"         \
            "uniform mat4 VX_M;\n"         \
            "void main(void) {\n"          \
            "  vtexcoord = atexcoord; \n " \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(aposition, 0, 1.0);\n" \
            "}";

        char fragment_shader_src[] =
            "precision mediump float; \n"   \
            "varying vec2 vtexcoord; \n"         \
            "uniform sampler2D texture; \n" \
            "uniform vec4 rgba0, rgba1, rgba2, rgba3; \n" \
            "void main(void) {\n"           \
            "  vec4 c = texture2D(texture, vtexcoord);\n" \
            "  if (c.r < 0.5 / 255.0) gl_FragColor = rgba0;\n" \
            "  else if (c.r < 1.5 / 255.0) gl_FragColor = rgba1;\n" \
            "  else if (c.r < 2.5 / 255.0) gl_FragColor = rgba2;\n" \
            "  else if (c.r < 3.5 / 255.0) gl_FragColor = rgba3;\n"
            "  else gl_FragColor = vec4(0.0, 0.0, 0.0, 0);\n" \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="rgba0", .nrows = 4, .ncols = 1, .data = rgba0 },
                                  { .name="rgba1", .nrows = 4, .ncols = 1, .data = rgba1 },
                                  { .name="rgba2", .nrows = 4, .ncols = 1, .data = rgba2 },
                                  { .name="rgba3", .nrows = 4, .ncols = 1, .data = rgba3 },
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="aposition", .resource=position_resource },
                                  { .name="atexcoord", .resource=texcoord_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  { .name="texture", .resource = tex },
                                  { .name = NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_TRIANGLE_STRIP, .first = 0, .count = 4 },
                                  { .count = 0 }, });
}


// for every non-zero value of the texture, output rgba. Else, discard.
vx_object_t *vxo_image_mask(vx_resource_t *tex, float rgba[4])
{
    float w = vx_resource_texture_get_width(tex), h = vx_resource_texture_get_height(tex);
    float tw = 1.0 * vx_resource_texture_get_bpp(tex) * w / vx_resource_texture_get_stride(tex), th = 1.0;
    vx_resource_t *position_resource = vx_resource_make_attr_f32_copy((float[]) { 0, 0,  w, 0,  0, h,  w, h }, 8, 2);
    vx_resource_t *texcoord_resource = vx_resource_make_attr_f32_copy((float[]) { 0, 0,  tw, 0,  0, th, tw, th }, 8, 2);

    vx_lock();

    static vx_resource_t *program_resource = NULL;

    if (program_resource == NULL) {

        char vertex_shader_src[] =
            "attribute vec2 aposition; \n"  \
            "attribute vec2 atexcoord; \n"  \
            "varying vec2 vtexcoord; \n"   \
            "uniform mat4 VX_P;\n"         \
            "uniform mat4 VX_V;\n"         \
            "uniform mat4 VX_M;\n"         \
            "void main(void) {\n"          \
            "  vtexcoord = atexcoord; \n " \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(aposition, 0, 1.0);\n" \
            "}";

        char fragment_shader_src[] =
            "precision mediump float; \n"                    \
            "varying vec2 vtexcoord; \n"                     \
            "uniform sampler2D texture; \n"                  \
            "uniform vec4 rgba;\n"                           \
            "void main(void) {\n"                            \
            "  vec4 c = texture2D(texture, vtexcoord);\n"    \
            "  if (c.r == 0.0)\n"                            \
            "    discard;\n"                                 \
            "  else\n"                                       \
            "    gl_FragColor = rgba;\n"                        \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="rgba", .nrows = 4, .ncols = 1, .data = rgba },
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="aposition", .resource=position_resource },
                                  { .name="atexcoord", .resource=texcoord_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  { .name="texture", .resource = tex },
                                  { .name = NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_TRIANGLE_STRIP, .first = 0, .count = 4 },
                                  { .count = 0 }, });
}
