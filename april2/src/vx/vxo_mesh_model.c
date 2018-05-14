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
#include <math.h>
#include <stdint.h>
#include "vx.h"
#include "vxo_generic.h"
#include "common/mesh_model.h"

vx_object_t *vxo_mesh_model_create(mesh_model_t *model)
{
    static vx_resource_t *texture_program_resource = NULL;
    static vx_resource_t *solid_program_resource = NULL;

    vx_lock();

    if (texture_program_resource == NULL) {

        /** m_ prefix: In model space (before transformation by VX_M)
            w_ prefix: In world space (the camera is not at the origin)
            e_ prefix: In eye space (after VX_V * VX_M)
         **/
        char vertex_shader_src[] =
            "#define PI 3.14159265358979323846264338\n"                 \
            "precision mediump float; \n"                               \
            "attribute vec3 m_xyz; \n"                                  \
            "attribute vec3 m_normal; \n"                               \
            "attribute vec2 st; \n"                                     \
            "uniform mat4 VX_P;\n"                                      \
            "uniform mat4 VX_V;\n"                                      \
            "uniform mat4 VX_M;\n"                                      \
            "varying vec3 e_xyz;\n"                                     \
            "varying vec3 e_normal;\n"                                  \
            "varying vec2 vst;\n"                                       \
            "\n"                                                        \
            "void main(void) {\n"                                       \
            "  mat4  VX_VM = VX_V*VX_M;\n"                              \
            "  mat3  VX_VM_R = mat3(VX_VM[0][0], VX_VM[0][1], VX_VM[0][2],\n" \
            "                       VX_VM[1][0], VX_VM[1][1], VX_VM[1][2],\n" \
            "                       VX_VM[2][0], VX_VM[2][1], VX_VM[2][2]);\n" \
            "  e_xyz = (VX_VM*vec4(m_xyz, 1.0)).xyz;\n"                 \
            "  e_normal = normalize(VX_VM_R*m_normal);\n"               \
            "  vst = st;\n"                                              \
            "  gl_Position = VX_P * vec4(e_xyz, 1.0);\n"                \
            "}";

        char fragment_shader_common[] =
            "precision mediump float; \n"                               \
            "uniform mat4 VX_V;\n"                                      \
            "varying vec3 e_xyz;\n"                                     \
            "varying vec3 e_normal;\n"                                  \
            "uniform vec4 rgba;\n"                                      \
            "uniform float roughness;\n"                                \
            "varying vec2 vst;\n"                                       \
            "vec3 scale(float alpha, vec3 v) {\n"                       \
            " return vec3(alpha*v.x, alpha*v.y, alpha*v.z);\n"          \
            "}\n"                                                       \
            "\n"                                                        \
            "vec3 do_lighting(vec3 e_light_pos, vec3 light_diff_rgb, vec3 light_spec_rgb, vec3 e_xyz, vec3 e_normal) {\n" \
            "  vec3 rgb = vec3(0.0, 0.0, 0.0);\n"                       \
            "  vec3 e_light_dir = normalize(e_light_pos - e_xyz);\n"    \
            "  vec3 e_eye_dir = normalize(-e_xyz); \n"                  \
            "  vec3 e_reflect = -reflect(e_light_dir, e_normal);\n"     \
            "\n"                                                        \
            "  float alpha = clamp(dot(e_light_dir, e_normal), 0.0, 1.0);\n" \
            "  rgb += scale(alpha, light_diff_rgb);\n"         \
            "\n"
            "  float beta = clamp(dot(e_reflect, e_eye_dir), 0.0, 1.0);\n" \
            "  beta = pow(beta, roughness);\n"                          \
            "  rgb += scale(beta, light_spec_rgb);\n"          \
            "\n"                                                        \
            "  return rgb;\n"                                           \
            "}\n"                                                       \
            "\n";

        char fragment_shader_texture_main[] =
            "uniform sampler2D texture;\n"                              \
            "void main(void) {\n"                                       \
            "  vec3 w_light_pos = vec3(15, 0, 15);\n"                   \
            "  vec3 light_diff_rgb = vec3(0.3, 0.3, 0.3);\n"            \
            "  vec3 light_spec_rgb = vec3(0.3, 0.3, 0.3);\n"            \
            "  vec4 out_rgba;\n"                                        \
            "    vec4 c = texture2D(texture, vst);\n"                   \
            "    if (c.a == 0.0) discard;\n"                          \
            "    out_rgba = vec4(c.r, c.g, c.b, c.a);\n"                \
            "\n"                                                        \
            "  vec3 e_light_pos = (VX_V * vec4(w_light_pos, 1.0)).xyz;\n" \
            "  out_rgba.xyz += do_lighting(e_light_pos, light_diff_rgb, light_spec_rgb, e_xyz, e_normal);\n" \
            "  gl_FragColor = out_rgba;\n"                              \
            "}\n";

        // XXX GROSS: Hard-coded light position.
        char fragment_shader_solid_main[] =
            "void main(void) {\n"                                       \
            "  vec3 w_light_pos = vec3(15, 0, 15);\n"                   \
            "  vec3 light_diff_rgb = vec3(0.3, 0.3, 0.3);\n"            \
            "  vec3 light_spec_rgb = vec3(0.3, 0.3, 0.3);\n"            \
            "  vec4 out_rgba;\n"                                        \
            "    out_rgba = rgba;\n"                                    \
            "\n"                                                        \
            "  vec3 e_light_pos = (VX_V * vec4(w_light_pos, 1.0)).xyz;\n" \
            "  out_rgba.xyz += do_lighting(e_light_pos, light_diff_rgb, light_spec_rgb, e_xyz, e_normal);\n" \
            "  gl_FragColor = out_rgba;\n"                              \
            "}\n";

        char *fragment_shader_texture_src = sprintf_alloc("%s\n%s", fragment_shader_common, fragment_shader_texture_main);
        texture_program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_texture_src);
        texture_program_resource->incref(texture_program_resource); // make immortal

        char *fragment_shader_solid_src =  sprintf_alloc("%s\n%s", fragment_shader_common, fragment_shader_solid_main);
        solid_program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_solid_src);
        solid_program_resource->incref(solid_program_resource); // make immortal

    }

    vx_unlock();

    vx_object_t *chain = vxo_chain(NULL, NULL);

    for (int chunkidx = 0; chunkidx < zarray_size(model->chunks); chunkidx++) {
        struct mesh_model_chunk *chunk;
        zarray_get(model->chunks, chunkidx, &chunk);

        vx_resource_t *xyz_resource = vx_resource_make_attr_f32_copy((float*) chunk->vertices->data, zarray_size(chunk->vertices) * 3, 3);
        vx_resource_t *normals_resource = vx_resource_make_attr_f32_copy((float*) chunk->normals->data, zarray_size(chunk->normals) * 3, 3);
        vx_resource_t *indices_resource = NULL;
        if (chunk->indices)
            indices_resource = vx_resource_make_idx_u16_copy((uint16_t*) chunk->indices->data, zarray_size(chunk->indices) * 3);

        vx_resource_t *tex_resource = NULL;

        vx_resource_t *texcoords_resource = NULL;
        if (chunk->pam) {
            tex_resource = vx_resource_make_texture_pam_copy(chunk->pam, VX_TEXTURE_WRAP);

            texcoords_resource = vx_resource_make_attr_f32_copy((float*) chunk->texcoords->data, zarray_size(chunk->texcoords) * 2, 2);
        }

        int nvertices = indices_resource ? 3 * zarray_size(chunk->indices) : zarray_size(chunk->vertices);

        vx_object_t *obj = vxo_generic_create(tex_resource ? texture_program_resource : solid_program_resource,
                                              (struct vxo_generic_uniformf[]) {
                                                  { .name="rgba", .nrows = 4, .ncols = 1, .data = chunk->rgba },
                                                  { .name="roughness", .nrows = 1, .ncols = 1, .data = (float[]) { chunk->roughness } },
                                                  { .name=NULL } },
                                              (struct vxo_generic_attribute[]) {
                                                  { .name="m_xyz", .resource = xyz_resource },
                                                  { .name="m_normal", .resource = normals_resource },
                                                  { .name=texcoords_resource ? "st" : NULL, .resource = texcoords_resource },
                                                  { .name=NULL } },
                                              (struct vxo_generic_texture[]) {
                                                  { .name= tex_resource ? "texture" : NULL, .resource = tex_resource },
                                                  { .name=NULL } },
                                              (struct vxo_generic_draw[]) {
                                                  { .command = VX_GL_TRIANGLES, .first = 0, .count = nvertices, .indices_resource = indices_resource },
                                                  { .count = 0 }, });

        vxo_chain_add(chain, obj);

    }

    return chain;
}
