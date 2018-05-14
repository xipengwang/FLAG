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

#include <math.h>
#include "vx.h"

#include "common/string_util.h"
#include "vxo_generic.h"

vx_object_t *vxo_circle_solid(float rgba[4])
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

        zarray_t *v = zarray_create(sizeof(float[2]));
        int nverts = 100;
        float origin[2] = { 0, 0 };
        zarray_add(v, origin);

        for (int i = 0; i < nverts; i++) {
            double rad = i * 2 * M_PI / nverts;
            float xy[2] = { cos(rad), sin(rad) };
            zarray_add(v, xy);
        }

        float t[2];
        zarray_get(v, 1, t);
        zarray_add(v, t);

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal

        verts_resource = vx_resource_make_attr_f32_copy((float*) v->data, zarray_size(v)*2, 2);
        verts_resource->incref(verts_resource); // make immortal

        zarray_destroy(v);
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="rgba", .nrows = 4, .ncols = 1, .data = rgba },
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="position", .resource = verts_resource },
                                  { .name = NULL } },
                              (struct vxo_generic_texture[]) {
                                  {.name=NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_TRIANGLE_FAN, .first = 0,
                                          .count = verts_resource->u.attr_f32.nelements / verts_resource->u.attr_f32.dim },
                                  { .count = 0 }, });


}

vx_object_t *vxo_circle_line(float rgba[4], float lineWidth)
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

        zarray_t *v = zarray_create(sizeof(float[2]));
        int nverts = 100;

        for (int i = 0; i < nverts; i++) {
            double rad = i * 2 * M_PI / nverts;
            float xy[2] = { cos(rad), sin(rad) };
            zarray_add(v, xy);
        }

        float t[2];
        zarray_get(v, 0, t);
        zarray_add(v, t);

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal

        verts_resource = vx_resource_make_attr_f32_copy((float*) v->data, zarray_size(v)*2, 2);
        verts_resource->incref(verts_resource); // make immortal

        zarray_destroy(v);
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
