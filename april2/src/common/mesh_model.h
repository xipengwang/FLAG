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

#ifndef _MESH_MODEL_H
#define _MESH_MODEL_H

#include "common/zarray.h"
#include "common/pam.h"

// this mesh model data type is used as an internal representation by
// rwx and obj parsers
typedef struct mesh_model_chunk mesh_model_chunk_t;
struct mesh_model_chunk
{
    zarray_t *vertices;  // float[3]
    zarray_t *normals;   // float[3], same length as vertices. Never null.
    zarray_t *texcoords; // float[2], same length as vertices OR null.
    zarray_t *indices;   // uint16_t[3], if NULL, then just use the vertices directly

    float rgba[4]; // material color and transparency

    float reflectivity;
    float roughness;

    pam_t *pam; // the texture, if one was specified

    char *name; // can be NULL. Purely for human use.

    void *user; // a free pointer for use by the user. Not used by mesh_model.
};

typedef struct mesh_model mesh_model_t;
struct mesh_model
{
    zarray_t *chunks; // struct mesh_model_chunk*

    void *user;
};

struct mesh_model_create_params
{
    // default color (used if materials are not specified)
    float rgba[4];

    // reduce # of GL programs by merging chunks with the same material
    int combine_materials;
};

mesh_model_t *mesh_model_create();

void mesh_model_create_params_init(struct mesh_model_create_params *params);
mesh_model_t *mesh_model_create_from_obj(const char *objpath);
mesh_model_t *mesh_model_create_from_obj_params(const char *objpath, const struct mesh_model_create_params *params);
mesh_model_t *mesh_model_create_with_mtl(const char *objpath, char *mtlpath);
mesh_model_t *mesh_model_create_with_mtl_from_obj_params(const char *objpath, char *mtlpath,  const struct mesh_model_create_params *params);


// void mesh_model_paint(mesh_model_t *model, float rgba[]);

void mesh_model_normals_from_faces(mesh_model_t *model);
void mesh_model_destroy(mesh_model_t *model);

struct mesh_model_chunk *mesh_model_chunk_create();
struct mesh_model_chunk *mesh_model_chunk_create_box();

struct mesh_model_chunk *mesh_model_chunk_create_sphere(int depth);

void mesh_model_chunk_normals_from_faces(struct mesh_model_chunk *chunk);

void mesh_model_transform(mesh_model_t *model, const double M[16]);
void mesh_model_chunk_transform(struct mesh_model_chunk *chunk, const double M[16]);

int mesh_model_save_obj(mesh_model_t *model, const char *path);

#endif
