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

// returns a value of v that is within 0.5 of ref.
static double mod1ref(double ref, double v)
{
    while (v - ref > .5)
        v -= 1;

    while (v - ref < -.5)
        v += 1;

    return v;
}

struct vertex_data
{
    float xyz[3], st[2];
};

static void normalize(double xyz[3])
{
    double mag2 = 0;
    for (int i = 0; i < 3; i++)
        mag2 += xyz[i]*xyz[i];
    double norm = 1.0 / sqrt(mag2);
    for (int i = 0; i < 3; i++)
        xyz[i] *= norm;
}

static double dist2(double *a, float *b, int len)
{
    double mag2 = 0;

    for (int i = 0; i < len; i++)
        mag2 += (a[i] - b[i]) * (a[i] - b[i]);

    return mag2;
}

static uint16_t find_or_add_vertex(zarray_t *vertex_datas, double xyz[3], double st[2])
{
    // is this vertex already in our list somewhere?
    double eps = 0.001;
    double eps2 = eps * eps;

    // XXX Slow
    for (int i = 0; i < zarray_size(vertex_datas); i++) {
        struct vertex_data *vtest;
        zarray_get_volatile(vertex_datas, i, &vtest);

        if (dist2(xyz, vtest->xyz, 3) < eps2 && dist2(st, vtest->st, 2) < eps2)
            return i;
    }

    assert(zarray_size(vertex_datas)+1 <= 65535);

    struct vertex_data newvd = { .xyz = { xyz[0], xyz[1], xyz[2] }, .st = { st[0], st[1] } };

    zarray_add(vertex_datas, &newvd);
    return zarray_size(vertex_datas) - 1;
}

static void compute_texture_coordinates(const double xyz[3], double st[2])
{
    st[0] = atan2(xyz[1], xyz[0]) / (2 * M_PI);
    st[1] = acos(xyz[2]) / M_PI;
}

static void recurse(zarray_t *vertex_datas, zarray_t *tris, double xyza[3], double xyzb[3], double xyzc[3], int depth)
{
    if (depth == 0) {
        double sta[2], stb[2], stc[2];

        compute_texture_coordinates(xyza, sta);
        compute_texture_coordinates(xyzb, stb);
        compute_texture_coordinates(xyzc, stc);

        // fix up texture coordinates so that where the texture wraps
        // around we don't try to insert an entire earth's worth of
        // terrain.
        for (int i = 0; i < 2; i++) {
            stb[0] = mod1ref(sta[0], stb[0]);
            stc[0] = mod1ref(sta[0], stc[0]);
        }

        uint16_t *tri = (uint16_t[]) { find_or_add_vertex(vertex_datas, xyza, sta),
                                       find_or_add_vertex(vertex_datas, xyzb, stb),
                                       find_or_add_vertex(vertex_datas, xyzc, stc) };


        zarray_add(tris, tri);

        return;
    }

    double *xyzd = (double[]) { xyza[0] + xyzb[0], xyza[1] + xyzb[1], xyza[2] + xyzb[2] };
    double *xyze = (double[]) { xyzb[0] + xyzc[0], xyzb[1] + xyzc[1], xyzb[2] + xyzc[2] };
    double *xyzf = (double[]) { xyza[0] + xyzc[0], xyza[1] + xyzc[1], xyza[2] + xyzc[2] };

    normalize(xyzd);
    normalize(xyze);
    normalize(xyzf);

    recurse(vertex_datas, tris, xyza, xyzd, xyzf, depth - 1);
    recurse(vertex_datas, tris, xyzd, xyzb, xyze, depth - 1);
    recurse(vertex_datas, tris, xyzd, xyze, xyzf, depth - 1);
    recurse(vertex_datas, tris, xyzf, xyze, xyzc, depth - 1);
}

/** In this implementation, the only data passed to the shader are the
 * texture coordinates. These texture coordinates can be trivially
 * converted into latitude and longitude, but are adjusted so that the
 * triangles render textures properly across the longitude=PI
 * boundary. The shader computes lat/lon from the shader coordinates,
 * then computes xyz.
 **/
vx_object_t *vxo_sphere_textured(vx_resource_t *texture_resource)
{
    static vx_resource_t *program_resource = NULL;
    static vx_resource_t *sts_resource = NULL;
    static vx_resource_t *tris_resource = NULL;
    static int nvertices = 0, napositions = 0;

    vx_lock();

    if (program_resource == NULL) {

        zarray_t *vertex_datas = zarray_create(sizeof(struct vertex_data));
        zarray_t *tris = zarray_create(sizeof(uint16_t[3])); // indices

        const double v = sqrt(3) / 3;
        double *xyza = (double[]) {  v,  v,  v };
        double *xyzb = (double[]) { -v, -v,  v };
        double *xyzc = (double[]) { -v,  v, -v };
        double *xyzd = (double[]) {  v, -v, -v };

        int depth = 4;

        recurse(vertex_datas, tris, xyza, xyzc, xyzb, depth);
        recurse(vertex_datas, tris, xyza, xyzb, xyzd, depth);
        recurse(vertex_datas, tris, xyza, xyzd, xyzc, depth);
        recurse(vertex_datas, tris, xyzb, xyzc, xyzd, depth);

        zarray_t *sts = zarray_create(sizeof(float[2]));

        for (int i = 0; i < zarray_size(vertex_datas); i++) {
            struct vertex_data *vd;
            zarray_get_volatile(vertex_datas, i, &vd);

            float *st = (float[]) { vd->st[0],
                                    vd->st[1] };

            zarray_add(sts, st);
        }

        nvertices = zarray_size(tris) * 3;

        char vertex_shader_src[] =
            "#define PI 3.14159265358979323846264338\n" \
            "attribute vec2 st; \n"  \
            "varying vec2 vtexcoord; \n"   \
            "uniform mat4 VX_P;\n"         \
            "uniform mat4 VX_V;\n"         \
            "uniform mat4 VX_M;\n"         \
            "void main(void) {\n"          \
            "  float lat = (PI / 2.0) - PI * st.y; \n "  \
            "  float lon = 2.0 * PI * st.x; \n "  \
            "  vtexcoord = st.xy; \n" \
            "  float r = cos(lat); \n " \
            "  vec3 xyz = vec3(r*cos(lon), r*sin(lon), sin(lat)); \n " \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(xyz, 1.0);\n" \
            "}";

        char fragment_shader_src[] =
            "precision mediump float; \n"   \
            "varying vec2 vtexcoord; \n"         \
            "uniform sampler2D texture; \n" \
            "void main(void) {\n"           \
            "  vec4 c = texture2D(texture, vtexcoord);\n" \
            "  gl_FragColor = vec4(c.r, c.g, c.b, 1);\n" \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal

        sts_resource = vx_resource_make_attr_f32_copy((float*) sts->data, zarray_size(sts)*2, 2);
        sts_resource->incref(sts_resource); // make immortal

        tris_resource = vx_resource_make_idx_u16_copy((uint16_t*) tris->data, nvertices);
        tris_resource->incref(tris_resource);
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="st", .resource = sts_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  { .name="texture", .resource=texture_resource },
                                  {.name=NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_TRIANGLES, .first = 0, .count = nvertices, .indices_resource = tris_resource },
                                  { .count = 0 }, });

}


/** In this implementation, the only data passed to the shader are the
 * texture coordinates. These texture coordinates can be trivially
 * converted into latitude and longitude, but are adjusted so that the
 * triangles render textures properly across the longitude=PI
 * boundary. The shader computes lat/lon from the shader coordinates,
 * then computes xyz.
 **/
vx_object_t *vxo_sphere_solid(float rgba[])
{
    static vx_resource_t *program_resource = NULL;
    static vx_resource_t *sts_resource = NULL;
    static vx_resource_t *tris_resource = NULL;
    static int nvertices = 0, napositions = 0;

    vx_lock();

    if (program_resource == NULL) {

        zarray_t *vertex_datas = zarray_create(sizeof(struct vertex_data));
        zarray_t *tris = zarray_create(sizeof(uint16_t[3])); // indices

        const double v = sqrt(3) / 3;
        double *xyza = (double[]) {  v,  v,  v };
        double *xyzb = (double[]) { -v, -v,  v };
        double *xyzc = (double[]) { -v,  v, -v };
        double *xyzd = (double[]) {  v, -v, -v };

        int depth = 4;

        recurse(vertex_datas, tris, xyza, xyzc, xyzb, depth);
        recurse(vertex_datas, tris, xyza, xyzb, xyzd, depth);
        recurse(vertex_datas, tris, xyza, xyzd, xyzc, depth);
        recurse(vertex_datas, tris, xyzb, xyzc, xyzd, depth);

        zarray_t *sts = zarray_create(sizeof(float[2]));

        for (int i = 0; i < zarray_size(vertex_datas); i++) {
            struct vertex_data *vd;
            zarray_get_volatile(vertex_datas, i, &vd);

            float *st = (float[]) { vd->st[0],
                                    vd->st[1] };

            zarray_add(sts, st);
        }

        nvertices = zarray_size(tris) * 3;

        /** m_ prefix: In model space (before transformation by VX_M)
            w_ prefix: In world space (the camera is not at the origin)
            e_ prefix: In eye space (after VX_V * VX_M)

            For a sphere, m_normal==m_xyz. For more general geometry,
            we'll have to pass that in as an attribute.
         **/
        char vertex_shader_src[] =
            "#define PI 3.14159265358979323846264338\n"  \
            "precision mediump float; \n"                \
            "attribute vec2 st; \n"                      \
            "uniform mat4 VX_P;\n"                       \
            "uniform mat4 VX_V;\n"                       \
            "uniform mat4 VX_M;\n"                       \
            "varying vec3 e_xyz;\n"                      \
            "varying vec3 e_normal;\n"                   \
            "\n"                                         \
            "void main(void) {\n"                        \
            "  float lat = (PI / 2.0) - PI * st.y; \n "  \
            "  float lon = 2.0 * PI * st.x; \n "                       \
            "  float r = cos(lat); \n "                                 \
            "  vec3  m_xyz = vec3(r*cos(lon), r*sin(lon), sin(lat)); \n " \
            "  vec3  m_normal = m_xyz;\n"                               \
            "\n"                                                        \
            " // generic code below\n"                                  \
            "  mat4  VX_VM = VX_V*VX_M;\n"                              \
            "  mat3  VX_VM_R = mat3(VX_VM[0][0], VX_VM[0][1], VX_VM[0][2],\n" \
            "                       VX_VM[1][0], VX_VM[1][1], VX_VM[1][2],\n" \
            "                       VX_VM[2][0], VX_VM[2][1], VX_VM[2][2]);\n" \
            "  e_xyz = (VX_VM*vec4(m_xyz, 1.0)).xyz;\n"                   \
            "  e_normal = normalize(VX_VM_R*m_normal);\n"          \
            "  gl_Position = VX_P * vec4(e_xyz, 1.0);\n"                \
            "}";

        char fragment_shader_src[] =
            "precision mediump float; \n"                               \
            "uniform mat4 VX_V;\n"                                      \
            "varying vec3 e_xyz;\n"                                     \
            "varying vec3 e_normal;\n"                                  \
            "uniform vec4 rgba;\n"                                      \
            "uniform float shininess;\n"                                \
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
            "  beta = pow(beta, shininess);\n"                          \
            "  rgb += scale(beta, light_spec_rgb);\n"          \
            "\n"                                                        \
            "  return rgb;\n"                                           \
            "}\n"                                                       \
            "\n"                                                        \
            "void main(void) {\n"                                       \
            "  vec3 w_light_pos = vec3(15, 0, 15);\n"                   \
            "  vec3 light_diff_rgb = vec3(0.0, 0.5, 0.0);\n"            \
            "  vec3 light_spec_rgb = vec3(0.0, 0.0, 1.0);\n"            \
            "  vec4 out_rgba = rgba;\n"                                 \
            "\n"
            "  vec3 e_light_pos = (VX_V * vec4(w_light_pos, 1.0)).xyz;\n"     \
            "  out_rgba.xyz += do_lighting(e_light_pos, light_diff_rgb, light_spec_rgb, e_xyz, e_normal);\n" \
            "  gl_FragColor = out_rgba;\n"                              \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal

        sts_resource = vx_resource_make_attr_f32_copy((float*) sts->data, zarray_size(sts)*2, 2);
        sts_resource->incref(sts_resource); // make immortal

        tris_resource = vx_resource_make_idx_u16_copy((uint16_t*) tris->data, nvertices);
        tris_resource->incref(tris_resource);
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="rgba", .nrows = 4, .ncols = 1, .data = rgba },
                                  { .name="shininess", .nrows = 1, .ncols = 1, .data = (float[]) { 20.0 } },
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="st", .resource = sts_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  {.name=NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_TRIANGLES, .first = 0, .count = nvertices, .indices_resource = tris_resource },
                                  { .count = 0 }, });

}
