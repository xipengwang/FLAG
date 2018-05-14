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

#include "common/string_util.h"
#include "vxo_generic.h"

vx_object_t *vxo_points(vx_resource_t *verts, const float rgba[4], float pointSize)
{
    static vx_resource_t *program_resource = NULL;

    vx_lock();

    if (program_resource == NULL) {

        char *vertex_shader_src =
            "attribute vec3 position; \n"                               \
            "uniform mat4 VX_P;\n"                                      \
            "uniform mat4 VX_V;\n"                                      \
            "uniform mat4 VX_M;\n"                                      \
            "uniform float pointSize;\n"                                \
            "void main(void) {\n"                                       \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(position, 1.0);\n" \
            "  gl_PointSize = pointSize;\n"                             \
            "}";

        char *fragment_shader_src =
            "precision mediump float; \n"       \
            "uniform vec4 rgba; \n"
            "void main(void) {\n"                              \
            "  gl_FragColor = rgba;\n" \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="rgba", .nrows = 4, .ncols = 1,  .data = (float[]) { rgba[0], rgba[1], rgba[2], rgba[3] } },
                                  { .name="pointSize", .nrows = 1, .ncols = 1, .data = &pointSize },
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="position", .resource = verts },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  {.name=NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_POINTS, .first = 0, .count = verts->u.attr_f32.nelements / verts->u.attr_f32.dim },
                                  { .count = 0 }, });

}

vx_object_t *vxo_points_pretty(vx_resource_t *verts, vx_resource_t *intensities)
{
    static vx_resource_t *program_resource = NULL;

    vx_lock();

    if (program_resource == NULL) {

        char *vertex_shader_src =
            "attribute vec3 position; \n"                               \
            "attribute float intensity; \n"                             \
            "uniform mat4 VX_P;\n"                                      \
            "uniform mat4 VX_V;\n"                                      \
            "uniform mat4 VX_M;\n"                                      \
            "varying vec4 vposition; \n"                                \
            "varying float vintensity;\n"                               \
            "void main(void) {\n"                                       \
            "  vposition = VX_M * vec4(position, 1.0);\n "              \
            "  gl_Position = VX_P * VX_V * vposition;\n " \
            "  gl_PointSize = 3.0;\n" \
            "  vintensity = intensity; \n" \
            "}";

        // Fast HSV to RGB from http://lolengine.net/blog/2013/07/27/rgb-to-hsv-in-glsl
        char *fragment_shader_src =
            "precision mediump float; \n"                           \
            "varying vec4 vposition; \n"                            \
            "varying float vintensity; \n"                          \
            "void hsv2rgb(in vec3 c, out vec3 rgb) {\n"                              \
            "  vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);\n"    \
            "  vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);\n" \
            "  rgb = c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);\n" \
            "}\n"                                                   \
            "void main(void) {\n"                                   \
            "  float dist = sqrt(vposition.x*vposition.x + vposition.y*vposition.y);\n"   \
            "  float h = 0.7 + 0.10*vposition.z;\n"    \
            "  float v = 0.4 + 2.0*vintensity;\n" \
            "  float s = max(max(1.0-0.025*(dist-0.8), 0.1), v);\n"           \
            "  vec3 hsv = vec3(h, s, v);\n"     \
            "  vec3 rgb;\n"                     \
            "  hsv2rgb(hsv, rgb);\n"            \
            "  if( vintensity > 0.9 ) gl_FragColor = vec4(1.0,1.0,1.0,1.0);\n" \
            "  else gl_FragColor = vec4(rgb, 1.0);\n" \
            "}\n";

//            "  gl_FragColor = vec4(0.5*vposition.z, 2.0*vintensity, 0.1*dist, 1); "
//        "  gl_FragColor = vec4(5.0*vintensity, vposition.z / 2.0, (vposition.z - 0.1) * 100.0, 1); "

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="position", .resource = verts },
                                  { .name="intensity", .resource = intensities },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  {.name=NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_POINTS, .first = 0, .count = verts->u.attr_f32.nelements / verts->u.attr_f32.dim },
                                  { .count = 0 }, });

}
