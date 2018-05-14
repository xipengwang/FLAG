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
#include "common/zhash.h"
#include "common/zarray.h"
#include "common/floats.h"
#include <common/string_util.h>

#include "mesh_model.h"
#include "common/pam.h"
#include "common/image_u8x3.h"
#include "common/doubles.h"
#include "common/string_util.h"

// only handles ordinary decimal numbers, of the form:
// -?[0-9.]+(e-?[0-9]+)?
// No careful rounding, no overflow handling.
// Breaks for exponential representations, as well.
double fast_strtod(const char *s, char *ignored)
{
    int pos = 0;

    int64_t v = 0;
    int decimal_digits = 0; // how many digits after decimal?

    if (1) {
        int neg = 0;

        if (s[pos] == '-') {
            neg = 1;
            pos++;
        }

        while (s[pos] >= '0' && s[pos] <= '9') {
            v = (v * 10) + (s[pos] - '0');
            pos++;
        }

        if (s[pos] == '.') {
            pos++;
            int p0 = pos;
            while (s[pos] >= '0' && s[pos] <= '9') {
                v = (v * 10) + (s[pos] - '0');
                pos++;
            }
            decimal_digits = pos - p0;
        }

        if (neg)
            v = -v;
    }

    int exp = 0;

    if (1) {
        if (s[pos] == 'e' || s[pos] == 'E') {
            pos++;

            int neg = 0;
            if (s[pos] == '-') {
                neg = 1;
                pos++;
            }

            while (s[pos] >= '0' && s[pos] <= '9') {
                exp = (exp*10) + (s[pos] - '0');
                pos++;
            }

            if (neg)
                exp = -exp;
        }
    }

//    printf("%20s %20d %5d %5d\n", s, v, decimal_digits, exp);
    return v * pow(10, exp - decimal_digits);
}

// does sepchars contain 'c'?
static int fast_split_contains(const char *sepchars, char c)
{
    for (; *sepchars != 0; sepchars++)
        if (*sepchars == c)
            return 1;
    return 0;
}

// returns the number of tokens written to 'toks', but never more than
// 'maxtoks'. If 'zerolength' is true, then every occurrence of a
// sepchar will generate a new token; if it is false, then zero-length
// tokens will be removed.
int fast_split(char *buf, char **toks, int maxtoks, char *sepchars, int zerolength)
{
    int ntoks = 0;
    int pos = 0;

    // remove any trailing newline characters
    int len = strlen(buf);
    while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r')) {
        buf[len-1] = 0;
        len--;
    }

    while (ntoks < maxtoks) {
        // skip leading/multiple separators
        while (!zerolength && fast_split_contains(sepchars, buf[pos]))
            pos++;

        // are we done?
        if (buf[pos] == 0)
            break;

        // new token
        toks[ntoks] = &buf[pos];

        // how long is this token?
        int toklen = 0;
        while (toks[ntoks][toklen] != 0 && !fast_split_contains(sepchars, toks[ntoks][toklen]))
            toklen++;

        // only keep empty tokens.
        if (toklen > 0 || zerolength)
            ntoks++;

        pos += toklen;
        if (buf[pos] == 0)
            break;

        // zero-terminate
        buf[pos] = 0;
        pos++;
    }

    if (ntoks > 0) {

    }

    return ntoks;
}

mesh_model_t *mesh_model_create()
{
    mesh_model_t *model = calloc(1, sizeof(mesh_model_t));
    model->chunks = zarray_create(sizeof(struct mesh_model_chunk*));
    return model;
}

void mesh_model_create_params_init(struct mesh_model_create_params *params)
{
    params->rgba[0] = 0.5;
    params->rgba[1] = 0.5;
    params->rgba[2] = 0.5;
    params->rgba[3] = 1;

    params->combine_materials = 1;
}

// We will create one mesh_model_chunk for each specified material. If
// multiple OBJ chunks use the same material, they will be combined
// together (this reduces the number of GL programs required to render
// the object, but the semantic organization of the model is lost)

// OBJ is a bit annoying in that triangles are specified as
// arbitrary combinations of vertex and normals, something that is
// awkward with OpenGL. We maintain our own database of vertices
// and normals, and generate mesh data only once the faces are
// specified.  This leads to a "fully exploded" mesh, in which we
// don't use indices at all, since it is hard (or inconvenient) to
// keep track of which vertex/normal combinations are used.
mesh_model_t *mesh_model_create_from_obj(const char *objpath)
{
    struct mesh_model_create_params params;
    mesh_model_create_params_init(&params);
    return mesh_model_create_from_obj_params(objpath, &params);
}

mesh_model_t *mesh_model_create_from_obj_params(const char *objpath, const struct mesh_model_create_params *params)
{
    int warn_map_ka_not_implemented = 0;
    int warn_tr_not_implemented = 0;
    int warn_ke_not_implemented = 0;

    FILE *objfile = fopen(objpath, "r");
    if (objfile == NULL)
        return NULL;

    FILE *mtlfile = NULL;

    // mtls values stores chunks but without any vertex information. We copy these
    // when we parse an obj.
    zhash_t *mtls = zhash_create(sizeof(char*), sizeof(struct mesh_model_chunk*),
                                 zhash_str_hash, zhash_str_equals);

    mesh_model_t *model = calloc(1, sizeof(mesh_model_t));
    model->chunks = zarray_create(sizeof(struct mesh_model_chunk*));

    // parse materials
    if (1) {
        char *mtlpath = strdup(objpath);
        int len = strlen(mtlpath);
        mtlpath[len-3] = 'm';
        mtlpath[len-2] = 't';
        mtlpath[len-1] = 'l';

        mtlfile = fopen(mtlpath, "r");
        if (mtlfile) {
            char buf[1024];

            struct mesh_model_chunk *chunk = NULL;

            char *toks[16];

            while (fgets(buf, sizeof(buf), mtlfile)) {

                int ntoks = fast_split(buf, toks, sizeof(toks) / sizeof(char*), " \t", 0);

                if (ntoks == 0)
                    continue;

                // ignore comments
                if (toks[0][0] == '#')
                    continue;

                // convert tok0 to lower case.
                for (int pos = 0; toks[0][pos] != 0; pos++) {
                    if (toks[0][pos] >= 'A' && toks[0][pos] <= 'Z')
                        toks[0][pos] += 'a' - 'A';
                }

                if (!strcmp(toks[0], "newmtl")) {
                    assert(ntoks == 2);

                    char *tok1 = strdup(toks[1]);

                    chunk = calloc(1, sizeof(struct mesh_model_chunk));
                    if (zhash_put(mtls, &tok1, &chunk, NULL, NULL)) {
                        // duplicate material. unhandled
                        printf("warning: duplicate material %s\n", tok1);
                       // assert(0);
                    }

                    // set defaults
                    chunk->roughness = 20.0;
                    chunk->reflectivity = 0;
                    chunk->rgba[0] = params->rgba[0];
                    chunk->rgba[1] = params->rgba[1];
                    chunk->rgba[2] = params->rgba[2];
                    chunk->rgba[3] = params->rgba[3];

                    chunk->vertices = zarray_create(sizeof(float[3]));
                    chunk->normals = zarray_create(sizeof(float[3]));
                    chunk->indices = NULL; // we explode all triangles

                    zarray_add(model->chunks, &chunk);

                    continue;
                }

                if (!chunk) {
                    // they're specifying a material property but haven't given
                    // a newmtl directive.
                    assert(0);
                    continue;
                }

                if (!strcmp(toks[0], "ns")) {
                    assert(ntoks == 2);

                    chunk->roughness = fast_strtod(toks[1], NULL);
                    continue;
                }

                if (!strcmp(toks[0], "d")) {
                    assert(ntoks == 2);
                    chunk->rgba[3] = fast_strtod(toks[1], NULL);
                    continue;
                }

                if (!strcmp(toks[0], "illum")) {
                    assert(ntoks == 2);
                   // assert(atoi(tok1) == 2); // other formats unimplemented
                    continue;
                }

                if (!strcmp(toks[0], "kd")) {
                    assert(ntoks == 4);

                    for (int i = 0; i < 3; i++)
                        chunk->rgba[i] = fast_strtod(toks[i+1], NULL);

                    continue;
                }

                if (!strcmp(toks[0], "ka")) {
                    assert(ntoks == 4);

                    continue;
                }

                if (!strcmp(toks[0], "ks")) {
                    assert(ntoks == 4);

                    continue;
                }

                if (!strcmp(toks[0], "map_d")) {
                    if (chunk->pam) {
                        // try to merge
                        if (ntoks == 1) {
                            printf("mesh_model: Missing texture\n");
                            continue;
                        }

                        if (ntoks != 2) {
                            printf("mesh_model ntoks !=2, ntoks = %d\n", ntoks);
                            for (int i = 0; i < ntoks; i++)
                                printf("%s ", toks[i]);
                            printf("\n");
                            assert(0);
                        }

                        char *path = strdup(objpath);
                        char *lastslash = strrchr(path, '/');
                        if (lastslash) {
                            lastslash[0] = 0;
                        } else {
                            free(path);
                            path = strdup(".");
                        }

                        char impath[2048];
                        sprintf(impath, "%s/%s", path, toks[1]);

                        int len = strlen(impath);
                        impath[len - 3] = 'p';
                        impath[len - 2] = 'a';
                        impath[len - 1] = 'm';

                        int w = chunk->pam->width, h = chunk->pam->height;

                        pam_t *alpha = pam_create_from_file(impath);
                        if (!alpha) {
                            printf("mesh_model.c: failed to load %s\n", impath);
                            continue;
                        }

                        pam_t *old = chunk->pam;
                        chunk->pam = pam_convert(old, PAM_RGB_ALPHA);

                        assert(w == alpha->width);
                        assert(h == alpha->height);

                        int d = alpha->depth;

                        for (int y = 0; y < h; y++) {
                            for (int x = 0; x < w; x++) {

                                int a = alpha->data[y*d*w + d*x + d - 1];
                                chunk->pam->data[y*4*w + x*4 + 3] = a;
                            }
                        }

                        pam_destroy(old);
                        pam_destroy(alpha);
                        assert(chunk->pam);
                    } else {
                        printf("mesh_model warning: unsupported separate depth channel. Merge into alpha image. See april2/src/common/example/pam_merge.\n");
                    }
                    continue;
                }

                if (!strcmp(toks[0], "map_ka")) {
                    if (!warn_map_ka_not_implemented)
                        printf("mesh_model.c: material property 'map_ka' not implemented\n");
                    warn_map_ka_not_implemented++;

                    continue;
                }

                if (!strcmp(toks[0], "tr")) {
                    if (!warn_tr_not_implemented)
                        printf("mesh_model.c: material property 'tr' not implemented\n");
                    warn_tr_not_implemented++;

                    continue;
                }

                if (!strcmp(toks[0], "ke")) {
                    if (!warn_tr_not_implemented)
                        printf("mesh_model.c: material property 'ke' not implemented\n");
                    warn_ke_not_implemented++;

                    continue;
                }

                if (!strcmp(toks[0], "map_kd")) {
                    if (ntoks == 1) {
                        printf("mesh_model: Missing texture\n");
                        continue;
                    }

                    if (ntoks != 2) {
                        printf("mesh_model ntoks !=2, ntoks = %d\n", ntoks);
                        for (int i = 0; i < ntoks; i++)
                            printf("%s ", toks[i]);
                        printf("\n");
                        assert(0);
                    }

                    char *path = strdup(objpath);
                    char *lastslash = strrchr(path, '/');
                    if (lastslash) {
                        lastslash[0] = 0;
                    } else {
                        free(path);
                        path = strdup(".");
                    }

                    char impath[2048];
                    sprintf(impath, "%s/%s", path, toks[1]);

                    int len = strlen(impath);
                    impath[len - 3] = 'p';
                    impath[len - 2] = 'a';
                    impath[len - 1] = 'm';

                    chunk->pam = pam_create_from_file(impath);
                    if (!chunk->pam) {
                        printf("failed to load texture %s\n", impath);
                    }
                    assert(chunk->pam);

                    chunk->texcoords = zarray_create(sizeof(float[2]));

                    continue;
                }

                if (!strcmp(toks[0], "bump")) {
                    // bump map not implemented
                    continue;
                }

                if (!strcmp(toks[0], "ni")) {
                    continue;
                }

                if (!strcmp(toks[0], "tf")) {
                    continue;
                }

                printf("unknown material property: '%s'\n", toks[0]);
            }

            fclose(mtlfile);
        }
    }

    // parse the obj itself
    struct mesh_model_chunk *chunk = NULL;

    zarray_t *vertices = zarray_create(sizeof(float[3]));
    zarray_t *normals = zarray_create(sizeof(float[3]));
    zarray_t *texcoords = zarray_create(sizeof(float[2]));

    if (1) {
        char buf[1024];
        char *toks[128];
        while (fgets(buf, sizeof(buf), objfile)) {

            int ntoks = fast_split(buf, toks, sizeof(toks) / sizeof(char*), " \t", 0);

            if (ntoks == 0)
                continue;

            // ignore comments
            if (toks[0][0] == '#')
                continue;

            if (!strcmp(toks[0], "mtllib")) {
                // we derive the MTL file name automatically from the
                // obj filename. Ignore this directive.
                continue;
            }

            if (!strcmp(toks[0], "o"))
                continue;

            if (!strcmp(toks[0], "g"))
                continue;

            if (!strcmp(toks[0], "usemtl")) {
                assert(ntoks == 2);

                if (!zhash_get(mtls, &toks[1], &chunk)) {
                    printf("usemtl '%s', but no material known of that type\n", toks[1]);
                    assert(0);
                }

                // always split different usemtls into different
                // chunks? Useful for debugging the different
                // components of a model, but probably a performance
                // loss.
                if (!params->combine_materials) {
                    if (zarray_size(chunk->vertices)) {
                        struct mesh_model_chunk *c = calloc(1, sizeof(struct mesh_model_chunk));
                        memcpy(c->rgba, chunk->rgba, 4 * sizeof(float));
                        c->roughness = chunk->roughness;
                        c->reflectivity = c->reflectivity;
                        if (chunk->pam)
                            c->pam = pam_copy(chunk->pam);
                        if (chunk->name)
                            c->name = strdup(chunk->name);
                        c->vertices = zarray_create(sizeof(float[3]));
                        c->normals = zarray_create(sizeof(float[3]));
                        chunk = c;
                        zarray_add(model->chunks, &chunk);
                    }
                }

                continue;
            }

            if (!strcmp(toks[0], "v")) {
                assert(ntoks == 4);

                float v[3];
                for (int i = 0; i < 3; i++)
                    v[i] = fast_strtod(toks[i+1], NULL);

                zarray_add(vertices, v);

                continue;
            }

            if (!strcmp(toks[0], "vn")) {
                assert(ntoks == 4);

                float v[3];
                for (int i = 0; i < 3; i++) {
                    v[i] = fast_strtod(toks[i+1], NULL);
                }

                floats_normalize(v, 3, v);

                zarray_add(normals, v);

                continue;
            }

            if (!strcmp(toks[0], "vt")) {
                assert(ntoks >= 3); // ignore extra texture entries

                float v[2];
                for (int i = 0; i < 2; i++)
                    v[i] = fast_strtod(toks[i+1], NULL);

                v[1] *= -1;

                zarray_add(texcoords, v);

                continue;
            }

            // if (!strcmp(toks[0], "g")) {
            //     // named groups. (This isn't quite right?)
            //     if (chunk->name)
            //         free(chunk->name);
            //     chunk->name = strdup(toks[1]);
            //     continue;
            // }

            if (!strcmp(toks[0], "s")) {
                // smoothing groups not implemented
                printf("smoothing groups not implemented\n");
                continue;
            }

            if (!strcmp(toks[0], "f")) {
                if (chunk == NULL) {
                    printf("ignoring data for unspecified material\n");
                    continue;
                }

                int sz = ntoks - 1;

                int vertidx[sz];
                int texidx[sz];
                int normidx[sz];

                zarray_t *vert_set = zarray_create(sizeof(int));

                for (int i = 0; i < sz; i++) {
                    char *parts[3];
                    int nparts = fast_split(toks[i+1], parts, 3, "/", 1);

                    assert(nparts > 0);

                    // if no texture is present (t = "x//y"), then tidx will be -1.
                    vertidx[i] = atoi(parts[0]) - 1;
                    texidx[i] = (nparts > 1) ? atoi(parts[1]) - 1 : -1;
                   // texidx[i] = atoi(parts[1]) - 1;
                    normidx[i] = (nparts > 2) ? atoi(parts[2]) - 1 : -1;
                    zarray_add(vert_set, &i);
                }

                int bad = 0;
                for (int i = 0; i < sz; i++) {
                    for (int j = i + 1; j < sz; j++)
                        if (vertidx[i] == vertidx[j])
                            bad = 1;
                }
                if (bad)
                    continue;

                // tesselate this polygon (stupidly).
                // fan tesselation
                // 0, 1, 2
                // 0, 2, 3
                // 0, 3, 4,
                // etc...
                if (1 || sz == 3) {
                    for (int j = 2; j < sz; j++) {
                        for (int i = 0; i < 3; i++) {
                            int idx;
                            if (i == 0)
                                idx = 0;
                            if (i == 1)
                                idx = j - 1;
                            if (i == 2)
                                idx = j;

                            float *vertex;
                            zarray_get_volatile(vertices, vertidx[idx], &vertex);
                            zarray_add(chunk->vertices, vertex);

                            float *normal;
                            zarray_get_volatile(normals, normidx[idx], &normal);
                            zarray_add(chunk->normals, normal);

                            if (chunk->pam) {
                                float *texcoord;
                                zarray_get_volatile(texcoords, texidx[idx], &texcoord);
                                zarray_add(chunk->texcoords, texcoord);
                            }
                        }
                    }
                }

                zarray_destroy(vert_set);

                continue;
            }

            printf("unknown line: %d '%s'\n", ntoks, toks[0]);

        }

        printf("input v %d, nv %d, nt %d\n",
                zarray_size(vertices), zarray_size(normals), zarray_size(texcoords));
    }

    zarray_destroy(vertices);
    zarray_destroy(normals);
    zarray_destroy(texcoords);

    return model;
}

mesh_model_t *mesh_model_create_with_mtl(const char *objpath, char *mtlpath)
{
    struct mesh_model_create_params params;
    mesh_model_create_params_init(&params);
    return mesh_model_create_with_mtl_from_obj_params(objpath, mtlpath, &params);
}


mesh_model_t *mesh_model_create_with_mtl_from_obj_params(const char *objpath, char *mtlpath,  const struct mesh_model_create_params *params)
{
    int warn_smoothing_groups_not_implemented = 0;

    FILE *objfile = fopen(objpath, "r");
    if (objfile == NULL)
        return NULL;

    FILE *mtlfile = NULL;

    // mtls values stores chunks but without any vertex information. We copy these
    // when we parse an obj.
    zhash_t *mtls = zhash_create(sizeof(char*), sizeof(struct mesh_model_chunk*),
                                 zhash_str_hash, zhash_str_equals);

    mesh_model_t *model = calloc(1, sizeof(mesh_model_t));
    model->chunks = zarray_create(sizeof(struct mesh_model_chunk*));

    // parse materials
    if (1) {
        // char *mtlpath = strdup(objpath);
        // int len = strlen(mtlpath);
        // mtlpath[len-3] = 'm';
        // mtlpath[len-2] = 't';
        // mtlpath[len-1] = 'l';

        mtlfile = fopen(mtlpath, "r");
        if (mtlfile) {
            char buf[1024];

            struct mesh_model_chunk *chunk = NULL;

            char *toks[16];

            while (fgets(buf, sizeof(buf), mtlfile)) {

                int ntoks = fast_split(buf, toks, sizeof(toks) / sizeof(char*), " ", 0);

                if (ntoks == 0)
                    continue;

                // ignore comments
                if (toks[0][0] == '#')
                    continue;

                for (int pos = 0; toks[0][pos] != 0; pos++) {
                    if (toks[0][pos] >= 'A' && toks[0][pos] <= 'Z')
                        toks[0][pos] += 'a' - 'A';
                }

                if (!strcmp(toks[0], "newmtl")) {
                    assert(ntoks == 2);

                    char *tok1 = strdup(toks[1]);

                    chunk = calloc(1, sizeof(struct mesh_model_chunk));
                    if (zhash_put(mtls, &tok1, &chunk, NULL, NULL)) {
                        // duplicate material. unhandled
                        printf("warning: duplicate material %s\n", tok1);
                       // assert(0);
                    }

                    // set defaults
                    chunk->reflectivity = 20.0;
                    chunk->rgba[0] = params->rgba[0];
                    chunk->rgba[1] = params->rgba[1];
                    chunk->rgba[2] = params->rgba[2];
                    chunk->rgba[3] = params->rgba[3];

                    chunk->vertices = zarray_create(sizeof(float[3]));
                    chunk->normals = zarray_create(sizeof(float[3]));
                    chunk->indices = NULL; // we explode all triangles

                    zarray_add(model->chunks, &chunk);

                    continue;
                }

                if (!chunk) {
                    // they're specifying a material property but haven't given
                    // a newmtl directive.
                    assert(0);
                    continue;
                }

                if (!strcmp(toks[0], "ns")) {
                    assert(ntoks == 2);

                    chunk->reflectivity = fast_strtod(toks[1], NULL);
                    continue;
                }

                if (!strcmp(toks[0], "d")) {
                    assert(ntoks == 2);
                    chunk->rgba[3] = fast_strtod(toks[1], NULL);
                    continue;
                }

                if (!strcmp(toks[0], "illum")) {
                    assert(ntoks == 2);
                   // assert(atoi(tok1) == 2); // other formats unimplemented
                    continue;
                }

                if (!strcmp(toks[0], "kd")) {
                    assert(ntoks == 4);

                    for (int i = 0; i < 3; i++)
                        chunk->rgba[i] = fast_strtod(toks[i+1], NULL);

                    continue;
                }

                if (!strcmp(toks[0], "ka")) {
                    assert(ntoks == 4);

                    continue;
                }

                if (!strcmp(toks[0], "ks")) {
                    assert(ntoks == 4);

                    continue;
                }

                if (!strcmp(toks[0], "map_kd")) {
                    if (ntoks == 1) {
                        printf("mesh_model: Missing texture\n");
                        continue;
                    }

                    assert(ntoks == 2);

                    char *path = strdup(objpath);
                    char *lastslash = strrchr(path, '/');
                    if (lastslash) {
                        lastslash[0] = 0;
                    } else {
                        free(path);
                        path = strdup(".");
                    }

                    char impath[2048];
                    sprintf(impath, "%s/Texture/%s", path, toks[1]);

                    int len = strlen(impath);
                    impath[len - 3] = 'p';
                    impath[len - 2] = 'a';
                    impath[len - 1] = 'm';

                    chunk->pam = pam_create_from_file(impath);
                    if (!chunk->pam) {
                        printf("failed to load texture %s\n", impath);
                    }
                    assert(chunk->pam);

                    chunk->texcoords = zarray_create(sizeof(float[2]));

                    continue;
                }

                if (!strcmp(toks[0], "bump")) {
                    // bump map not implemented
                    continue;
                }

                if (!strcmp(toks[0], "ni")) {
                    continue;
                }

                if (!strcmp(toks[0], "tf")) {
                    continue;
                }

                printf("unknown material property: %s\n", toks[0]);
            }

            fclose(mtlfile);
        }

        printf("materials: %d\n", zhash_size(mtls));
    }

    // parse the obj itself
    struct mesh_model_chunk *chunk = NULL;

    zarray_t *vertices = zarray_create(sizeof(float[3]));
    zarray_t *normals = zarray_create(sizeof(float[3]));
    zarray_t *texcoords = zarray_create(sizeof(float[2]));

    if (1) {
        char buf[1024];
        char *toks[128];
        while (fgets(buf, sizeof(buf), objfile)) {

            int ntoks = fast_split(buf, toks, sizeof(toks) / sizeof(char*), " ", 0);

            if (ntoks == 0)
                continue;

            // ignore comments
            if (toks[0][0] == '#')
                continue;

            if (!strcmp(toks[0], "mtllib")) {
                // we derive the MTL file name automatically from the
                // obj filename. Ignore this directive.
                continue;
            }

            if (!strcmp(toks[0], "o"))
                continue;

            if (!strcmp(toks[0], "usemtl")) {
                assert(ntoks == 2);

                if (!zhash_get(mtls, &toks[1], &chunk)) {
                    printf("usemtl '%s', but no material known of that type\n", toks[1]);
                    assert(0);
                }

                // always split different usemtls into different
                // chunks? Useful for debugging the different
                // components of a model, but probably a performance
                // loss.
                if (!params->combine_materials) {
                    if (zarray_size(chunk->vertices)) {
                        struct mesh_model_chunk *c = calloc(1, sizeof(struct mesh_model_chunk));
                        memcpy(c->rgba, chunk->rgba, 4 * sizeof(float));
                        c->reflectivity = chunk->reflectivity;
                        if (chunk->pam)
                            c->pam = pam_copy(chunk->pam);
                        if (chunk->name)
                            c->name = strdup(chunk->name);
                        c->vertices = zarray_create(sizeof(float[3]));
                        c->normals = zarray_create(sizeof(float[3]));
                        chunk = c;
                        zarray_add(model->chunks, &chunk);
                    }
                }

                continue;
            }

            if (!strcmp(toks[0], "v")) {
                assert(ntoks == 4);

                float v[3];
                for (int i = 0; i < 3; i++)
                    v[i] = fast_strtod(toks[i+1], NULL);

                zarray_add(vertices, v);

                continue;
            }

            if (!strcmp(toks[0], "vn")) {
                assert(ntoks == 4);

                float v[3];
                for (int i = 0; i < 3; i++) {
                    v[i] = fast_strtod(toks[i+1], NULL);
                }

                floats_normalize(v, 3, v);

                zarray_add(normals, v);

                continue;
            }

            if (!strcmp(toks[0], "vt")) {
                assert(ntoks >= 3); // ignore extra texture entries

                float v[2];
                for (int i = 0; i < 2; i++)
                    v[i] = fast_strtod(toks[i+1], NULL);

                v[1] *= -1;

                zarray_add(texcoords, v);

                continue;
            }

            // if (!strcmp(toks[0], "g")) {
            //     // named groups. (This isn't quite right?)
            //     if (chunk->name)
            //         free(chunk->name);
            //     chunk->name = strdup(toks[1]);
            //     continue;
            // }

            if (!strcmp(toks[0], "s")) {
                // smoothing groups not implemented
                if (!warn_smoothing_groups_not_implemented)
                    printf("mesh_model.c: smoothing groups not implemented\n");
                warn_smoothing_groups_not_implemented++;

                continue;
            }

            if (!strcmp(toks[0], "f")) {
                if (chunk == NULL) {
                    printf("mesh_model.c: ignoring data for unspecified material\n");
                    continue;
                }

                int sz = ntoks - 1;

                int vertidx[sz];
                int texidx[sz];
                int normidx[sz];

                zarray_t *vert_set = zarray_create(sizeof(int));

                for (int i = 0; i < sz; i++) {
                    char *parts[3];
                    int nparts = fast_split(toks[i+1], parts, 3, "/", 1);

                    assert(nparts > 0);

                    // if no texture is present (t = "x//y"), then tidx will be -1.
                    // negative indices can be used too.
                    vertidx[i] = -1;
                    texidx[i] = -1;
                    normidx[i] = -1;

                    vertidx[i] = atoi(parts[0]);
                    if (vertidx[i] < 0)
                        vertidx[i] += zarray_size(vertices);
                    else
                        vertidx[i] -= 1;

                    if (nparts > 1) {
                        texidx[i] = atoi(parts[1]);
                        if (texidx[i] < 0)
                            texidx[i] += zarray_size(texcoords);
                        else
                            texidx[i] -= 1;
                    }

                    if (nparts > 2) {
                        normidx[i] = atoi(parts[2]);
                        if (normidx[i] < 0)
                            normidx[i] += zarray_size(normals);
                        else
                            normidx[i] -= 1;
                    }

                    zarray_add(vert_set, &i);
                }

                int bad = 0;
                for (int i = 0; i < sz; i++) {
                    for (int j = i + 1; j < sz; j++)
                        if (vertidx[i] == vertidx[j])
                            bad = 1;
                }
                if (bad)
                    continue;

                // tesselate this polygon (stupidly).
                // fan tesselation
                // 0, 1, 2
                // 0, 2, 3
                // 0, 3, 4,
                // etc...
                if (1 || sz == 3) {
                    for (int j = 2; j < sz; j++) {
                        for (int i = 0; i < 3; i++) {
                            int idx;
                            if (i == 0)
                                idx = 0;
                            if (i == 1)
                                idx = j - 1;
                            if (i == 2)
                                idx = j;

                            float *vertex;
                            zarray_get_volatile(vertices, vertidx[idx], &vertex);
                            zarray_add(chunk->vertices, vertex);

                            float *normal;
                            zarray_get_volatile(normals, normidx[idx], &normal);
                            zarray_add(chunk->normals, normal);

                            if (chunk->pam) {
                                float *texcoord;
                                zarray_get_volatile(texcoords, texidx[idx], &texcoord);
                                zarray_add(chunk->texcoords, texcoord);
                            }
                        }
                    }
                }

                zarray_destroy(vert_set);

                continue;
            }

            printf("mesh_model: unknown line: %d '%s'\n", ntoks, toks[0]);

        }

        printf("input nv %d, nn %d, nt %d\n",
                zarray_size(vertices), zarray_size(normals), zarray_size(texcoords));
    }

    zarray_destroy(vertices);
    zarray_destroy(normals);
    zarray_destroy(texcoords);

    return model;
}


// Paint a mesh model with a single color
// void mesh_model_paint(mesh_model_t *model, float rgba[])
// {
//     //printf("%s painting chunks %d \n", __func__, zarray_size(model->chunks));
//     // zarray_t *chunks_array = zarray_create(sizeof(struct mesh_model_chunk));

//     struct mesh_model_chunk *chunk = NULL;//calloc(1, sizeof(struct mesh_model_chunk));
//     for(int i=0; i<zarray_size(model->chunks); i++){
//         zarray_get_volatile(model->chunks, i, &chunk);
//         printf("%s before %f %f %f \n", __func__, chunk->rgba[0],chunk->rgba[1],chunk->rgba[2]);
//         memcpy(chunk->rgba, rgba, 4 * sizeof(float));
//         // zarray_add(chunks_array, chunk);
//         // chunk->reflectivity = 10;
//         printf("%s after %f %f %f \n", __func__, chunk->rgba[0],chunk->rgba[1],chunk->rgba[2]);
//     }
//     // zarray_destroy(model->chunks);
//     // model->chunks = chunks_array;

// }

// duplicate vertices and normals such that no indices are required.
void mesh_model_chunk_explode(struct mesh_model_chunk *chunk)
{
    assert(0); // not implemented
}

// must be exploded first
void mesh_model_chunk_normals_from_faces(struct mesh_model_chunk *chunk)
{
    if (chunk->indices)
        mesh_model_chunk_explode(chunk);

    assert((zarray_size(chunk->vertices) % 3) == 0);

    if (zarray_size(chunk->vertices) != zarray_size(chunk->normals)) {
        zarray_destroy(chunk->normals);
        chunk->normals = zarray_create(sizeof(float[3]));
        float n[3] = { 0, 0, 0 };
        for (int i = 0; i < zarray_size(chunk->vertices); i++)
            zarray_add(chunk->normals, n);
    }

    int ntris = zarray_size(chunk->vertices) / 3;
    for (int triidx = 0; triidx < ntris; triidx++) {
        float *v[3];
        for (int i = 0; i < 3; i++) {
            zarray_get_volatile(chunk->vertices, 3*triidx + i, &v[i]);
        }

        float d1[3], d2[3];
        for (int i = 0; i < 3; i++) {
            d1[i] = v[1][i] - v[0][i];
            d2[i] = v[2][i] - v[0][i];
        }

        float this_normal[3];
        floats_cross_product(d1, d2, this_normal);
        floats_normalize(this_normal, 3, this_normal);

/*        for (int i = 0; i < 3; i++)
            floats_print(v[i], 3, "%15f");
        floats_print(this_normal, 3, "%15f");
*/
        for (int i = 0; i < 3; i++)
            zarray_set(chunk->normals, 3*triidx + i, this_normal, NULL);
    }
}

// recomputes normals for each vertex by making them normal to the
// face. The model will be exploded first if necessary. This is useful
// if the model has bad normals and you don't know what else to do!
void mesh_model_normals_from_faces(mesh_model_t *model)
{
    for (int chunkidx = 0; chunkidx < zarray_size(model->chunks); chunkidx++) {
        struct mesh_model_chunk *chunk;
        zarray_get(model->chunks, chunkidx, &chunk);
        mesh_model_chunk_normals_from_faces(chunk);
    }
}

void mesh_model_destroy(mesh_model_t *model)
{
    if (!model)
        return;

    for (int chunkidx = 0; chunkidx < zarray_size(model->chunks); chunkidx++) {
        struct mesh_model_chunk *chunk;
        zarray_get(model->chunks, chunkidx, &chunk);

        zarray_destroy(chunk->vertices);
        zarray_destroy(chunk->normals);
        zarray_destroy(chunk->texcoords);
        zarray_destroy(chunk->indices);

        if (chunk->pam)
            pam_destroy(chunk->pam);

        if (chunk->name)
            free(chunk->name);

        free(chunk);
    }

    zarray_destroy(model->chunks);

    free(model);
}


///////////////////////////////////////////////////////
struct mesh_model_chunk *mesh_model_chunk_create()
{
    struct mesh_model_chunk *chunk = calloc(1, sizeof(struct mesh_model_chunk));
    chunk->vertices = zarray_create(sizeof(float[3]));
    chunk->normals = zarray_create(sizeof(float[3]));
    chunk->texcoords = zarray_create(sizeof(float[2]));

    chunk->rgba[0] = 0.5;
    chunk->rgba[1] = 0.5;
    chunk->rgba[2] = 0.5;
    chunk->rgba[3] = 1.0;

    chunk->roughness = 20.0;
    chunk->reflectivity = 0;

    return chunk;
}

struct mesh_model_chunk *mesh_model_chunk_create_box()
{
    struct mesh_model_chunk *chunk = mesh_model_chunk_create();

    float tri_verts[] =  {
        // Bottom Face
        +0.5f,+0.5f,-0.5f, // 2
        -0.5f,-0.5f,-0.5f, // 0
        -0.5f,+0.5f,-0.5f, // 1

        -0.5f,-0.5f,-0.5f, // 0
        +0.5f,+0.5f,-0.5f, // 2
        +0.5f,-0.5f,-0.5f, // 3

        // Top
        -0.5f,+0.5f,+0.5f, // 5
        -0.5f,-0.5f,+0.5f, // 4
        +0.5f,+0.5f,+0.5f, // 6

        +0.5f,+0.5f,+0.5f, // 6
        -0.5f,-0.5f,+0.5f, // 4
        +0.5f,-0.5f,+0.5f, // 7

        // Front
        +0.5f,+0.5f,+0.5f, // 6
        +0.5f,-0.5f,+0.5f, // 7
        +0.5f,-0.5f,-0.5f, // 3

        +0.5f,+0.5f,+0.5f, // 6
        +0.5f,-0.5f,-0.5f, // 3
        +0.5f,+0.5f,-0.5f, // 2

        // Back
        -0.5f,-0.5f,+0.5f, // 4
        -0.5f,+0.5f,+0.5f, // 5
        -0.5f,+0.5f,-0.5f, // 1

        -0.5f,-0.5f,+0.5f, // 4
        -0.5f,+0.5f,-0.5f, // 1
        -0.5f,-0.5f,-0.5f, // 0

        // Right
        +0.5f,-0.5f,+0.5f, // 7
        -0.5f,-0.5f,+0.5f, // 4
        -0.5f,-0.5f,-0.5f, // 0

        +0.5f,-0.5f,+0.5f, // 7
        -0.5f,-0.5f,-0.5f, // 0
        +0.5f,-0.5f,-0.5f, // 3

        // Left
        -0.5f,+0.5f,+0.5f, // 5
        +0.5f,+0.5f,+0.5f, // 6
        +0.5f,+0.5f,-0.5f, // 2

        -0.5f,+0.5f,+0.5f, // 5
        +0.5f,+0.5f,-0.5f, // 2
        -0.5f,+0.5f,-0.5f, // 1
    };

    float tri_norms[] =  {
        // Bottom
        +0.0f,+0.0f,-1.0f,
        +0.0f,+0.0f,-1.0f,
        +0.0f,+0.0f,-1.0f,
        +0.0f,+0.0f,-1.0f,
        +0.0f,+0.0f,-1.0f,
        +0.0f,+0.0f,-1.0f,

        // Top
        +0.0f,+0.0f,+1.0f,
        +0.0f,+0.0f,+1.0f,
        +0.0f,+0.0f,+1.0f,
        +0.0f,+0.0f,+1.0f,
        +0.0f,+0.0f,+1.0f,
        +0.0f,+0.0f,+1.0f,

        // Front
        +1.0f,+0.0f,+0.0f,
        +1.0f,+0.0f,+0.0f,
        +1.0f,+0.0f,+0.0f,
        +1.0f,+0.0f,+0.0f,
        +1.0f,+0.0f,+0.0f,
        +1.0f,+0.0f,+0.0f,

        // Back
        -1.0f,+0.0f,+0.0f,
        -1.0f,+0.0f,+0.0f,
        -1.0f,+0.0f,+0.0f,
        -1.0f,+0.0f,+0.0f,
        -1.0f,+0.0f,+0.0f,
        -1.0f,+0.0f,+0.0f,

        // Right
        +0.0f,-1.0f,+0.0f,
        +0.0f,-1.0f,+0.0f,
        +0.0f,-1.0f,+0.0f,
        +0.0f,-1.0f,+0.0f,
        +0.0f,-1.0f,+0.0f,
        +0.0f,-1.0f,+0.0f,

        // Left
        +0.0f,+1.0f,+0.0f,
        +0.0f,+1.0f,+0.0f,
        +0.0f,+1.0f,+0.0f,
        +0.0f,+1.0f,+0.0f,
        +0.0f,+1.0f,+0.0f,
        +0.0f,+1.0f,+0.0f
    };

    assert(sizeof(tri_verts) == sizeof(tri_norms));

    for (int i = 0; i < sizeof(tri_verts)/sizeof(float[3]); i++) {
        zarray_add(chunk->vertices, &tri_verts[3*i]);
        zarray_add(chunk->normals, &tri_norms[3*i]);
    }

    return chunk;
}

void mesh_model_chunk_transform(struct mesh_model_chunk *chunk, const double M[16])
{
    for (int i = 0; i < zarray_size(chunk->vertices); i++) {
        float *f;
        zarray_get_volatile(chunk->vertices, i, &f);

        double in[3], out[3];
        for (int i = 0; i < 3; i++)
            in[i] = f[i];

        doubles_mat44_transform_xyz(M, in, out);

/*        printf("\n");
        doubles_print(in, 3, "%15f");
        doubles_print(out, 3, "%15f");
*/
        for (int i = 0; i < 3; i++)
            f[i] = out[i];
    }

    for (int i = 0; i < zarray_size(chunk->normals); i++) {
        float *f;
        zarray_get_volatile(chunk->normals, i, &f);

        double in[3], out[3];
        for (int i = 0; i < 3; i++)
            in[i] = f[i];

        doubles_mat44_rotate_vector(M, in, out);
        for (int i = 0; i < 3; i++)
            f[i] = out[i];
    }
}

void mesh_model_transform(mesh_model_t *model, const double M[16])
{
    for (int chunkidx = 0; chunkidx < zarray_size(model->chunks); chunkidx++) {
        struct mesh_model_chunk *chunk;
        zarray_get(model->chunks, chunkidx, &chunk);
        mesh_model_chunk_transform(chunk, M);
    }
}

int mesh_model_save_obj(mesh_model_t *model, const char *_objpath)
{
    char *rootname = NULL;
    char *dir = NULL; // what folder is path in?
    char *objpath, *mtlpath;

    if (!str_ends_with(_objpath, ".obj")) {
        objpath = sprintf_alloc("%s.obj", _objpath);
        mtlpath = sprintf_alloc("%s.mtl", _objpath);
    } else {
        objpath = strdup(_objpath);
        mtlpath = strdup(_objpath);
        int len = strlen(objpath);
        mtlpath[len - 3] = 'm';
        mtlpath[len - 2] = 't';
        mtlpath[len - 1] = 'l';
    }

    if (1) {
        dir = strdup(objpath);
        char *slash = strrchr(dir, '/');
        if (slash)
            *slash = 0;
        else
            dir[0] = 0;
    }

    if (1) {
        // if path = /abc/def/ghi.obj, derive rootname as 'ghi'

        const char *start = strrchr(objpath, '/');
        if (start == NULL)
            rootname = strdup(objpath); // no forward slash? start at beginning.
        else
            rootname = strdup(&start[1]); // skip last forward slash

        char *suffix = strrchr(rootname, '.');
        if (suffix)
            *suffix = 0;
    }

    FILE *obj = fopen(objpath, "w+");
    FILE *mtl = fopen(mtlpath, "w+");

    fprintf(obj, "mtllib %s\n", mtlpath);

    int v_offset = 0, t_offset = 0;

    for (int chunkidx = 0; chunkidx < zarray_size(model->chunks); chunkidx++) {
        struct mesh_model_chunk *chunk;
        zarray_get(model->chunks, chunkidx, &chunk);

        fprintf(obj, "usemtl mtl-%s-%d\n", rootname, chunkidx);

        for (int i = 0; i < zarray_size(chunk->vertices); i++) {
            float *f;
            zarray_get_volatile(chunk->vertices, i, &f);
            fprintf(obj, "v %.16g %.16g %.16g\n", f[0], f[1], f[2]);
        }

        assert(zarray_size(chunk->vertices) == zarray_size(chunk->normals));

        if (chunk->normals) {

            for (int i = 0; i < zarray_size(chunk->normals); i++) {
                float *f;
                zarray_get_volatile(chunk->normals, i, &f);
                fprintf(obj, "vn %.16g %.16g %.16g\n", f[0], f[1], f[2]);
            }
        }

        if (chunk->texcoords) {
            for (int i = 0; i < zarray_size(chunk->texcoords); i++) {
                float *f;
                zarray_get_volatile(chunk->texcoords, i, &f);
                fprintf(obj, "vt %.16g %.16g\n", f[0], -f[1]);
            }
        }

        if (chunk->indices) {
            // maybe it'd be better to output the vertex/normal/texcoords
            // as we traverse the triangles.

            for (int triidx = 0; triidx < zarray_size(chunk->indices); triidx++) {
                uint16_t *tri;
                zarray_get(chunk->indices, triidx, &tri);

                fprintf(obj, "f ");
                for (int i = 0; i < 3; i++) {
                    fprintf(obj, "%d/", v_offset + tri[i] + 1);
                    if (chunk->pam)
                        fprintf(obj, "%d", t_offset + tri[i] + 1);
                    fprintf(obj, "/%d ", v_offset + tri[i] + 1);
                }
                fprintf(obj, "\n");
            }

            // untested!
            assert(0);

        } else {
            assert(zarray_size(chunk->vertices) % 3 == 0);

            for (int i = 0; i < zarray_size(chunk->vertices); i += 3) {
                fprintf(obj, "f ");
                for (int j = 1; j <= 3; j++) {
                    fprintf(obj, "%d/", i + v_offset + j);

                    if (chunk->pam)
                        fprintf(obj, "%d", i + t_offset + j);

                    fprintf(obj, "/%d ", i + v_offset + j);
                }
                fprintf(obj, "\n");
            }
        }

        v_offset += zarray_size(chunk->vertices);
        if (chunk->texcoords)
            t_offset += zarray_size(chunk->texcoords);

        fprintf(mtl, "newmtl mtl-%s-%d\n", rootname, chunkidx);
        fprintf(mtl, "Kd %.16g %.16g %.16g\nd %.16g\n",
                chunk->rgba[0], chunk->rgba[1], chunk->rgba[2], chunk->rgba[3]);
        if (chunk->pam) {
            char full_pampath[1024];
            char partial_pampath[1024];
            sprintf(full_pampath, "%s/%s-%d.pam", dir, rootname, chunkidx);
            sprintf(partial_pampath, "%s-%d.pam", rootname, chunkidx);

            fprintf(mtl, "map_kd %s\n", partial_pampath);
            pam_write_file(chunk->pam, full_pampath);
        }
    }

    fclose(obj);
    fclose(mtl);

    free(rootname);
    free(dir);
    free(objpath);
    free(mtlpath);
    return 0;

}

//////////////////////////////////////////////////////////////////
// sphere stuff

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


mesh_model_chunk_t *mesh_model_chunk_create_sphere(int depth)
{
    struct mesh_model_chunk *chunk = mesh_model_chunk_create();
    chunk->indices = zarray_create(sizeof(uint16_t[3]));

    zarray_t *vertex_datas = zarray_create(sizeof(struct vertex_data));
    zarray_t *tris = zarray_create(sizeof(uint16_t[3])); // indices

    const double v = sqrt(3) / 3;
    double *xyza = (double[]) {  v,  v,  v };
    double *xyzb = (double[]) { -v, -v,  v };
    double *xyzc = (double[]) { -v,  v, -v };
    double *xyzd = (double[]) {  v, -v, -v };

    recurse(vertex_datas, tris, xyza, xyzc, xyzb, depth);
    recurse(vertex_datas, tris, xyza, xyzb, xyzd, depth);
    recurse(vertex_datas, tris, xyza, xyzd, xyzc, depth);
    recurse(vertex_datas, tris, xyzb, xyzc, xyzd, depth);

    for (int i = 0; i < zarray_size(vertex_datas); i++) {
        struct vertex_data *vd;
        zarray_get_volatile(vertex_datas, i, &vd);

        zarray_add(chunk->vertices, vd->xyz);
        zarray_add(chunk->normals, vd->xyz);
        zarray_add(chunk->texcoords, vd->st);
    }

    for (int i = 0; i < zarray_size(tris); i++) {
        uint16_t *tri;
        zarray_get_volatile(tris, i, &tri);
        zarray_add(chunk->indices, tri);
    }

//    nvertices = zarray_size(tris) * 3;
    zarray_destroy(vertex_datas);
    zarray_destroy(tris);

    return chunk;
}
