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

vx_object_t *vxo_grid(float rgba[4], float lineWidth)
{
    static vx_resource_t *program_resource = NULL;
    static vx_resource_t *verts_resource = NULL;
    vx_lock();

    if (verts_resource == NULL) {
        int radius = 100;
        int nlines = 2*(2*radius+1);
        float data[4*nlines];

        int pos = 0;
        for (int x = -radius; x <= radius; x++) {
            int y = sqrt(radius*radius - x*x);
            data[pos++] = x;
            data[pos++] = -y;
            data[pos++] = x;
            data[pos++] = y;

            assert(pos <= sizeof(data) / 4);
        }

        for (int x = -radius; x <= radius; x++) {
            int y = sqrt(radius*radius - x*x);
            data[pos++] = -y;
            data[pos++] = x;
            data[pos++] = y;
            data[pos++] = x;

            assert(pos <= sizeof(data) / 4);
        }

        assert(pos == 4*nlines);
        verts_resource = vx_resource_make_attr_f32_copy(data, pos, 2);
    }

    if (program_resource == NULL) {
        char *vertex_shader_src =
            "attribute vec2 position; \n"                               \
            "uniform vec3 VX_eye, VX_lookat;\n"                         \
            "uniform mat4 VX_P;\n"                                      \
            "uniform mat4 VX_V;\n"                                      \
            "uniform mat4 VX_M;\n"                                      \
            "uniform float lineWidth;\n"                                \
            "void main(void) {\n"                                       \
            "  float dist = distance(VX_eye, VX_lookat); \n"            \
            "  float scale = max(1.0, 0.1*dist);\n"                     \
            "  scale = pow(2.0, floor(log2(scale))); \n"                \
            "  float x = floor(VX_lookat[0] / scale) * scale +  scale * position.x; \n"         \
            "  float y = floor(VX_lookat[1] / scale) * scale + scale * position.y; \n"         \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(x, y, 0, 1.0);\n"      \
            "}";

//        abs(VX_VM[3][2]);\n"
        char *fragment_shader_src =
            "precision mediump float; \n"       \
            "uniform vec4 rgba; \n"             \
            "void main(void) {\n"               \
            "  gl_FragColor = rgba;\n"          \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="rgba", .nrows = 4, .ncols = 1, .data = rgba },
                                  { .name="glLineWidth", .nrows = 1, .ncols = 1, .data = (float[]) { lineWidth }},
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="position", .resource = verts_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  {.name=NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_LINES, .first = 0, .count = verts_resource->u.attr_f32.nelements / verts_resource->u.attr_f32.dim },
                                  { .count = 0 }, });

}
