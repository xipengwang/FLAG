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

#include "sim.h"
#include "sim_agent.h"

#include <pthread.h>

#include "common/doubles.h"
#include "common/floats.h"
#include "common/matd.h"
#include "common/string_util.h"
#include "common/config.h"
#include "common/workerpool.h"
#include "common/time_util.h"
#include "common/mesh_model.h"
#include "lcmtypes/laser_t.h"

// The point and direction should be transformed according to inv(T) by the caller.
static double triray_cast(sim_object_t *obj, const double p[3], const double dir[3], double max)
{
    return sim_triray_cast(obj->triray, p, dir, max);
}

/////////////////////////////////////////////////////////////
// sim_object box

static void sim_object_box_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj)
{
    const sim_object_t *so = obj;

    for (int i = 0; i < 16; i++)
        encode_f64(data, datapos, so->T[i]);

    for (int i = 0; i < 16; i++)
        encode_f32(data, datapos, so->u.box.M[i]);

    for (int i = 0; i < 4; i++)
        encode_f32(data, datapos, so->u.box.rgba[i]);

    encode_f32(data, datapos, so->u.box.roughness);
}

static void *sim_object_box_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    double T[16];
    for (int i = 0; i < 16; i++)
        T[i] = decode_f64(data, datapos, datalen);

    float M[16];
    for (int i = 0; i < 16; i++)
        M[i] = decode_f32(data, datapos, datalen);

    float rgba[4];
    for (int i = 0; i < 4; i++)
        rgba[i] = decode_f32(data, datapos, datalen);

    float roughness = decode_f32(data, datapos, datalen);

    return sim_object_box_create(T, M, rgba, roughness);
}

const static stype_t sim_object_box_stype = { .name = "sim_object_t:box",
                                              .encode = sim_object_box_encode,
                                              .decode = sim_object_box_decode,
                                              .copy = NULL,
                                              .destroy = NULL,
                                              .impl = NULL };

sim_object_t *sim_object_box_create(const double T[16], // position (always rigid)
                                            const float  M[16],  // model transform (can be non-rigid)
                                            const float rgba[4], float roughness)
{
    sim_object_t *so = calloc(1, sizeof(sim_object_t));
    so->type = SIM_OBJECT_BOX;
    so->stype = &sim_object_box_stype;

    memcpy(so->T, T, 16 * sizeof(double));

    so->mesh_version = 0;
    so->mesh_nversions = 1;
    so->mesh = mesh_model_create();
    memcpy(so->u.box.M, M, 16 * sizeof(float));
    memcpy(so->u.box.rgba, rgba, 4 * sizeof(float));
    so->u.box.roughness = roughness;

    struct mesh_model_chunk *chunk = mesh_model_chunk_create_box();
    memcpy(chunk->rgba, rgba, 4 * sizeof(float));
    chunk->roughness = roughness;

    if (T) {
        double dM[16];
        for (int i = 0; i < 16; i++)
            dM[i] = M[i];
        mesh_model_chunk_transform(chunk, dM);
    }

    zarray_add(so->mesh->chunks, &chunk);

    so->triray = sim_triray_create();
    sim_triray_add_mesh(so->triray, so->mesh);
    so->ray_cast = triray_cast;
    return so;
}

/////////////////////////////////////////////////////////////
// sim_object surface
/*
   0---1---2---3      coordinate of pixel ix, iy:
    \ / \ / \ / \     x = x0 + ix*pitch + (iy&1)*pitch/2
     4---5---6--7     y = y0 + iy*pitch*sqrt(3)/2
    / \ / \ / \ /
   8---9--10--11      most nodes result in two triangles
    \ / \ / \ / \     even y: (n, n+1, n-width), (n, n+width, n+1)
    12--13--14-15     odd  y: (n, n+1, n-width+1), (n, n+width+1, n+1)
*/

static void sim_object_surface_rebuild_mesh(sim_object_t *so)
{
    mesh_model_destroy(so->mesh);

    zarray_t *vertices = zarray_create(sizeof(float[3]));
    zarray_t *indices = zarray_create(sizeof(int[3]));

    int width = so->u.surface.width, height = so->u.surface.height;
    double pitch_x = so->u.surface.pitch;
    double pitch_y = so->u.surface.pitch * 0.866025403784439;
    double x0 = 0, y0 = 0;

    for (int iy = 0; iy < height; iy++) {
        for (int ix = 0; ix < width; ix++) {

            float f[3] = { x0 + ix*pitch_x + (iy&1)*pitch_x/2,
                           y0 + iy*pitch_y,
                           so->u.surface.zs[iy*width+ix] };
            zarray_add(vertices, f);

            if (ix + 1 < width && iy > 0) {
                int tri[3] = { iy*width + ix, iy*width + ix + 1, (iy-1)*width + ix + (iy & 1) };
                zarray_add(indices, tri);
            }

            if (ix + 1 < width && iy + 1 < height) {
                int tri[3] = { iy*width + ix, (iy+1)*width + ix + (iy & 1), iy*width + ix + 1};
                zarray_add(indices, tri);
            }
        }
    }

    zarray_t *normals = zarray_create(sizeof(float[3]));
    for (int i = 0; i < zarray_size(vertices); i++) {
        float n[3] = { 0, 0, 0 };
        zarray_add(normals, n);
    }

    for (int i = 0; i < zarray_size(indices); i++) {
        int *tri;
        zarray_get_volatile(indices, i, &tri);

        float *v[3];
        for (int j = 0; j < 3; j++)
            zarray_get_volatile(vertices, tri[j], &v[j]);

        // compute the unit normal for this triangle
        float d0[3], d1[3];
        floats_subtract(v[1], v[0], 3, d0);
        floats_subtract(v[2], v[0], 3, d1);
        floats_normalize(d0, 3, d0);
        floats_normalize(d1, 3, d1);

        float thisn[3];
        floats_cross_product(d0, d1, thisn);

        // sum the normals over every vertex.
        for (int j = 0; j < 3; j++) {
            float *n;
            zarray_get_volatile(normals, tri[j], &n);
            floats_add(n, thisn, 3, n);
        }
    }

    // explode those vertices into a non-indexed mesh
    struct mesh_model_chunk *chunk = mesh_model_chunk_create();
    memcpy(chunk->rgba, so->u.surface.rgba, 4 * sizeof(float));
    chunk->roughness = so->u.surface.roughness;

    for (int i = 0; i < zarray_size(indices); i++) {
        int *tri;
        zarray_get_volatile(indices, i, &tri);

        for (int j = 0; j < 3; j++) {
            float *v;
            zarray_get_volatile(vertices, tri[j], &v);
            zarray_add(chunk->vertices, v);

            float *n;
            zarray_get_volatile(normals, tri[j], &n);
            floats_normalize(n, 3, n);
            zarray_add(chunk->normals, n);
        }
    }

    // enable this if you want to see the individual faces, without
    // the smoothed normals computed above.
    if (0)
        mesh_model_chunk_normals_from_faces(chunk);

    zarray_destroy(vertices);
    zarray_destroy(indices);
    zarray_destroy(normals);

    mesh_model_t *mm = mesh_model_create();
    zarray_add(mm->chunks, &chunk);
    so->mesh = mm;
    so->mesh_version ++;

    sim_triray_destroy(so->triray);
    so->triray = sim_triray_create();
    sim_triray_add_mesh(so->triray, so->mesh);
    sim_triray_split(so->triray, 0.0, 4);
    so->ray_cast = triray_cast;
}

void sim_object_surface_mound(sim_object_t *so, double x, double y, double dheight, double range)
{
    int width = so->u.surface.width, height = so->u.surface.height;
    double pitch_x = so->u.surface.pitch;
    double pitch_y = so->u.surface.pitch * 0.866025403784439;
    double x0 = 0, y0 = 0;

    for (int iy = 0; iy < height; iy++) {

        for (int ix = 0; ix < width; ix++) {

            float xyz[3] = { x0 + ix*pitch_x + (iy&1)*pitch_x/2,
                           y0 + iy*pitch_y,
                           so->u.surface.zs[iy*width+ix] };

            double dist2 = sq(xyz[0] - x) + sq(xyz[1] - y);

            so->u.surface.zs[iy*width + ix] += dheight * exp(-dist2 / range);
        }
    }

    sim_object_surface_rebuild_mesh(so);
}

static void sim_object_surface_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj)
{
    const sim_object_t *so = obj;

    for (int i = 0; i < 16; i++)
        encode_f64(data, datapos, so->T[i]);

    encode_f64(data, datapos, so->u.surface.pitch);
    encode_u32(data, datapos, so->u.surface.width);
    encode_u32(data, datapos, so->u.surface.height);
    for (int i = 0; i < so->u.surface.width * so->u.surface.height; i++)
        encode_f32(data, datapos, so->u.surface.zs[i]);

    for (int i = 0; i < 4; i++)
        encode_f32(data, datapos, so->u.surface.rgba[i]);

    encode_f32(data, datapos, so->u.surface.roughness);
}

static void *sim_object_surface_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    double T[16];
    for (int i = 0; i < 16; i++)
        T[i] = decode_f64(data, datapos, datalen);

    double pitch = decode_f64(data, datapos, datalen);

    uint32_t width = decode_u32(data, datapos, datalen);
    uint32_t height = decode_u32(data, datapos, datalen);

    float *zs = calloc(sizeof(float), width * height);
    for (int i = 0; i < width * height; i++)
        zs[i] = decode_f32(data, datapos, datalen);

    float rgba[4];
    for (int i = 0; i < 4; i++)
        rgba[i] = decode_f32(data, datapos, datalen);

    float roughness = decode_f32(data, datapos, datalen);

    return sim_object_surface_create_dims(T, pitch, width, height, zs, rgba, roughness);
}

const static stype_t sim_object_surface_stype = { .name = "sim_object_t:surface",
                                              .encode = sim_object_surface_encode,
                                              .decode = sim_object_surface_decode,
                                              .copy = NULL,
                                              .destroy = NULL,
                                              .impl = NULL };


// pitch: distance along triangle. We own the resulting zs.
sim_object_t *sim_object_surface_create_dims(const double T[16], double pitch, int width, int height, float *zs,
                                             const float rgba[4], float roughness)
{
    sim_object_t *so = calloc(1, sizeof(sim_object_t));
    so->type = SIM_OBJECT_SURFACE;
    so->stype = &sim_object_surface_stype;

    memcpy(so->T, T, 16 * sizeof(double));

    so->mesh_version = 0;
    so->mesh_nversions = -1;
    so->mesh = mesh_model_create();

    so->u.surface.pitch = pitch;
    so->u.surface.width = width;
    so->u.surface.height = height;
    so->u.surface.zs = zs;

    memcpy(so->u.surface.rgba, rgba, 4 * sizeof(float));
    so->u.surface.roughness = roughness;

    sim_object_surface_rebuild_mesh(so);

    return so;
}

sim_object_t *sim_object_surface_create(const double xy0[2], const double xy1[2], double pitch, double z,
                                        const float rgba[4], float roughness)
{
    int width = (xy1[0] - xy0[0]) / pitch + 1;
    int height = (xy1[1] - xy0[1]) / pitch + 1;
    float *zs = calloc(sizeof(float), width * height);
    for (int i = 0; i < width * height; i++)
        zs[i] = z;

    double T[16];
    doubles_mat44_translate((double[]) { xy0[0], xy0[1], 0 }, T);

    doubles_print_mat(T, 4, 4, "%15f");

    return sim_object_surface_create_dims(T, pitch, width, height, zs, rgba, roughness);
}

static void sim_world_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj)
{
    sim_world_t *sw = (sim_world_t*) obj;
    encode_u32(data, datapos, zarray_size(sw->objects));

    for (int i = 0; i < zarray_size(sw->objects); i++) {
        sim_object_t *so;
        zarray_get(sw->objects, i, &so);
        stype_encode_object(data, datapos, so->stype, so);
    }
}

static void *sim_world_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    sim_world_t *sw = sim_world_create();

    int nobjects = decode_u32(data, datapos, datalen);

    for (int i = 0; i < nobjects; i++) {
        sim_object_t *so = stype_decode_object(data, datapos, datalen, NULL);
        zarray_add(sw->objects, &so);
    }

    return sw;
}

const static stype_t sim_world_stype = { .name = "sim_world_t",
                                         .encode = sim_world_encode,
                                         .decode = sim_world_decode,
                                         .copy = NULL,
                                         .destroy = NULL,
                                         .impl = NULL };
sim_world_t *sim_world_create()
{
    sim_world_t *sw = calloc(1, sizeof(sim_world_t));
    pthread_mutex_init(&sw->mutex, NULL);
    sw->stype = &sim_world_stype;
    sw->objects = zarray_create(sizeof(sim_object_t*));
    sw->agents = zarray_create(sizeof(sim_agent_t*));
    return sw;
}

void * sim_world_run_thread(void * impl)
{
    sim_world_t * sw = impl;
    int nproc = workerpool_get_nprocs();

    sw->running = 1;
    int running = sw->running;

    sw->paused = 0;
    int paused = sw->paused;

    int64_t time = utime_now();

    workerpool_t * wp = workerpool_create(nproc);

    while(running)
    {
        while(!paused)
        {
            pthread_mutex_lock(&sw->mutex);

            time += 10*1000;

            sim_agent_t * ag;

            //sense
            for(int i = 0; i < zarray_size(sw->agents); i++)
            {
                zarray_get(sw->agents, i, &ag);
                ag->prev_time = ag->time;
                ag->time = time;
                ag->sim_count++;
                workerpool_add_task(wp, ag->sense, ag);
            }
            workerpool_run(wp);

            //Manipulate
            for(int i = 0; i < zarray_size(sw->agents); i++)
            {
                zarray_get(sw->agents, i, &ag);
                ag->time = time;
                workerpool_add_task(wp, ag->manipulate, ag);
            }
            workerpool_run_single(wp);

            //Move
            for(int i = 0; i < zarray_size(sw->agents); i++)
            {
                zarray_get(sw->agents, i, &ag);
                ag->time = time;
                workerpool_add_task(wp, ag->move, ag);
            }
            workerpool_run(wp);

            paused = sw->paused;
            pthread_mutex_unlock(&sw->mutex);

            usleep(10*1000);
        }
        usleep(100*1000);
        pthread_mutex_lock(&sw->mutex);
        running = sw->running;
        paused = sw->paused;
        pthread_mutex_unlock(&sw->mutex);
    }

    workerpool_destroy(wp);

    return NULL;
}



sim_world_t *sim_world_create_from_file(const char * path)
{

    if(str_ends_with(path, ".world"))
    {
        fprintf(stderr, "Loading a .world file\n");
        return stype_read_file(path);
    }
    if(str_ends_with(path, ".config"))
    {
        fprintf(stderr, "Loading a .config file\n");
        config_t * cf = config_create_path(path);
        if(!cf)
            return NULL;

        sim_world_t *sw = sim_world_create();
        int n_obj = 0;
        char key[32] = "sim_object0.";
        config_set_prefix(cf, key);
        while(config_has_key(cf, "pos"))
        {

            double rgba[4];
            config_require_doubles_len(cf, "rgba", rgba, 4);

            float rgbaf[4];
            rgbaf[0] = rgba[0];
            rgbaf[1] = rgba[1];
            rgbaf[2] = rgba[2];
            rgbaf[3] = rgba[3];

            double xyzrpy[6];
            config_require_doubles_len(cf, "pos", xyzrpy, 3);

            double dims[3];
            config_require_doubles_len(cf, "dims", dims, 3);

            config_require_doubles_len(cf, "rpy", &xyzrpy[3], 3);

            double shine = config_get_double(cf, "shine", 0.2);

            double T[16];
            doubles_xyzrpy_to_mat44(xyzrpy, T);
            float M[16];
            floats_mat44_identity(M);
            M[0] = dims[0];
            M[5] = dims[1];
            M[10] = dims[2];

            sim_object_t * so = sim_object_box_create(T, M, rgbaf, shine);

            zarray_add(sw->objects, &so);

            n_obj++;
            sprintf(key, "sim_object%d.", n_obj);
            config_set_prefix(cf, key);

        }
        config_set_prefix(cf, NULL);
        fprintf(stderr, "Read %d objects\n", n_obj);

        return sw;

    }
    fprintf(stderr, "Unknown file type %s\n", path);
    return NULL;
}


void sim_stype_init()
{
    stype_register(&sim_world_stype);
    stype_register(&sim_object_box_stype);
    stype_register(&sim_object_surface_stype);
}

double sim_world_ray_cast(sim_world_t *sw, const double p[3], const double dir[3], double max,
                          sim_object_t **ignores, int nignores, sim_object_t **out_object)
{
    for (int idx = 0; idx < zarray_size(sw->objects); idx++) {
        sim_object_t *so;
        zarray_get(sw->objects, idx, &so);

        int ignore = 0;
        for (int i = 0; i < nignores; i++)
            if (so == ignores[i])
                ignore = 1;

        if (ignore)
            continue;

        if (!so->ray_cast) {
            printf("type %s doesn't support ray_cast\n", so->stype->name);
            continue;
        }

        double Tp[3];
        double Tdir[3];

        doubles_mat44_inv_transform_xyz(so->T, p, Tp);
        doubles_mat44_inv_rotate_vector(so->T, dir, Tdir);

        double oldmax = max;
        max = so->ray_cast(so, Tp, Tdir, max);
        if (max < oldmax && out_object)
            *out_object = so;
    }

    return max;
}
