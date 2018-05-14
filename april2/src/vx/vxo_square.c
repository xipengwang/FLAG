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

vx_object_t *vxo_square_solid(float rgba[4])
{
    static vx_resource_t *program_resource = NULL, *vertex_resource = NULL;

    vx_lock();

    if (program_resource == NULL) {

        char vertex_shader_src[] =
            "attribute vec2 position; \n"  \
            "uniform mat4 VX_P;\n"         \
            "uniform mat4 VX_V;\n"         \
            "uniform mat4 VX_M;\n"                                      \
            "void main(void) {\n"                                       \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(position, 0, 1.0);\n" \
            "}";

        char fragment_shader_src[] =
            "precision mediump float; \n"                     \
            "uniform vec4 rgba; \n"                           \
            "void main(void) {\n"                             \
            "  gl_FragColor = rgba;\n"                        \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal

        #define DEFAULT_LENGTH 1.0f
        #define DEFAULT_WIDTH 0.45f

        float verts[] = { -0.5f, -0.5f,
                          -0.5f,  0.5f,
                           0.5f, -0.5f,
                           0.5f,  0.5f,
                           0.5f,  0.5f};

        vertex_resource = vx_resource_make_attr_f32_copy(verts, 8, 2);
        vertex_resource->incref(vertex_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="rgba", .nrows = 4, .ncols = 1, .data = rgba },
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="position", .resource=vertex_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  {.name=NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_TRIANGLE_STRIP, .first = 0, .count = 4 },
                                  { .count = 0 }, });
}


vx_object_t *vxo_square_line(float rgba[4], float lineWidth)
{
    static vx_resource_t *program_resource = NULL;
    static vx_resource_t *verts_resource = NULL;

    vx_lock();

    if (program_resource == NULL) {

        char *vertex_shader_src =
            "attribute vec3 position; \n"                               \
            "uniform mat4 VX_P;\n"                                      \
            "uniform mat4 VX_V;\n"                                      \
            "uniform mat4 VX_M;\n"                                      \
            "void main(void) {\n"                                       \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(position, 1.0);\n" \
            "}";

        char *fragment_shader_src =
            "precision mediump float; \n"       \
            "uniform vec4 rgba; \n"
            "void main(void) {\n"                              \
            "  gl_FragColor = rgba;\n" \
            "}\n";

        float verts[] = {-0.5,-0.5,
                         -0.5, 0.5,
                          0.5, 0.5,
                          0.5,-0.5,
                         -0.5,-0.5};

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal

        verts_resource = vx_resource_make_attr_f32_copy(verts, 10, 2);
        verts_resource->incref(verts_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="rgba", .nrows = 4, .ncols = 1, .data = rgba },
                                  { .name="glLineWidth", .nrows = 1, .ncols = 1, .data = (float[]) { lineWidth } },
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="position", .resource = verts_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  {.name=NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_LINE_STRIP, .first = 0,
                                          .count = verts_resource->u.attr_f32.nelements / verts_resource->u.attr_f32.dim },
                                  { .count = 0 }, });


}
/*
vx_object_t *vxo_robots_xyt_solid(zarray_t *xyts, float rgba[4])
{

}
*/
