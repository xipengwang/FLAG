#include <assert.h>
#include <math.h>
#include "global_map.h"
#include "common/zhash.h"
#include "common/image_u8x3.h"
#include "common/image_u8.h"
#include "common/gridmap.h"
#include "common/math_util.h"
#include "common/time_util.h"

#define PAD_M 5

struct global_map {
    zhash_t * gms;
    image_u8x3_t * map3;

    //Be VERY carfeul with these two, I'm sharing
    //the data buf between them
    image_u8_t * out_map;
    grid_map_t * out_gm;
    int64_t last_update_utime;
    bool updated;

    double x0, y0, meters_per_pixel;
};

global_map_t * global_map_create(double meters_per_pixel)
{
    global_map_t * glm = calloc(1,sizeof(global_map_t));

    glm->gms = zhash_create(sizeof(grid_map_t*), sizeof(double[3]),
                            zhash_ptr_hash, zhash_ptr_equals);

    glm->map3    = image_u8x3_create_alignment(16,16,16);

    glm->out_map = image_u8_create_alignment(16,16,16);

    glm->out_gm  = calloc(1,sizeof(grid_map_t));

    glm->x0 = 0;
    glm->y0 = 0;
    glm->meters_per_pixel = meters_per_pixel;

    return glm;
}

void global_map_destroy(global_map_t * glm)
{
    zhash_destroy(glm->gms);
    image_u8x3_destroy(glm->map3);
    image_u8_destroy(glm->out_map);
    free(glm->out_gm);

    free(glm);
}

void ensure_index(global_map_t * glm, int * ix, int * iy) {

    int PPM = 1.0/glm->meters_per_pixel;
    int PAD = PPM * PAD_M; // this is a gross fixed padding
    image_u8x3_t * map3 = glm->map3;

    int upx = 0, downx = 0, upy = 0, downy = 0;

    if (*ix < 0) {
        downx = -(*ix) + PAD;
        downx -= downx % PPM;
    } else if (*ix >= map3->width) {
        upx = (*ix - map3->width) + PAD;
        upx -= upx % PPM;
    }
    if (*iy < 0) {
        downy = -(*iy) + PAD;
        downy -= downy % PPM;
    } else if (*iy >= map3->height) {
        upy = (*iy - map3->height) + PAD;
        upy -= upy % PPM;
    }

    if(!upx && !downx && !upy && !downy)
        return;

    int new_width = downx + map3->width + upx;
    int new_height = downy + map3->height + upy;
    image_u8x3_t * new3 = image_u8x3_create_alignment(new_width, new_height, new_width);

    assert (new3->width == new3->stride/3);
    assert (new3->width && new3->height);

    for(int y = 0; y < map3->height; y++) {
        memcpy(&new3->buf[(downy+y)*new3->stride + 3*downx],
               &map3->buf[y*map3->stride],
               map3->stride);
    }

    glm->map3 = new3;
    glm->x0 -= (glm->meters_per_pixel * downx);
    glm->y0 -= (glm->meters_per_pixel * downy);

    image_u8x3_destroy(map3);
}


static void map_add(global_map_t * glm, const grid_map_t * gm, double * xyt)
{
    glm->updated = true;
    glm->last_update_utime = utime_now();
    double c = cos(xyt[2]), s = sin(xyt[2]);

    for(int iy = 0; iy <= gm->height; iy += gm->height){
        for (int ix = 0; ix <= gm->width; ix += gm->width) {
            double x = gm->x0 + ix*gm->meters_per_pixel;
            double y = gm->y0 + iy*gm->meters_per_pixel;

            double tx = x*c - y*s + xyt[0];
            double ty = x*s + y*c + xyt[1];

            int out_ix = (tx - glm->x0) / glm->meters_per_pixel;
            int out_iy = (ty - glm->y0) / glm->meters_per_pixel;

            ensure_index(glm, &out_ix, &out_iy);
        }
    }

    for (int iy = 0; iy < gm->height; iy++) {
        for (int ix = 0; ix < gm->width; ix++) {
            uint8_t v = gm->data[iy*gm->width + ix];

            if (v == GRID_VAL_UNKNOWN) continue;
            if (v == GRID_VAL_OTHER) continue;

            double x = gm->x0 + ix*gm->meters_per_pixel;
            double y = gm->y0 + iy*gm->meters_per_pixel;

            double tx = x*c - y*s + xyt[0];
            double ty = x*s + y*c + xyt[1];

            int out_ix = (tx - glm->x0) / glm->meters_per_pixel;
            int out_iy = (ty - glm->y0) / glm->meters_per_pixel;

            if (v == GRID_VAL_TRAVERSABLE &&
                glm->map3->buf[out_iy*glm->map3->stride + 3*out_ix + 0] < 255)
                glm->map3->buf[out_iy*glm->map3->stride + 3*out_ix + 0] += 1;
            else if (v == GRID_VAL_OBSTACLE &&
                     glm->map3->buf[out_iy*glm->map3->stride + 3*out_ix + 1] < 255)
                glm->map3->buf[out_iy*glm->map3->stride + 3*out_ix + 1] += 1;
            else if (v == GRID_VAL_SLAMMABLE &&
                     glm->map3->buf[out_iy*glm->map3->stride + 3*out_ix + 2] < 255)
                glm->map3->buf[out_iy*glm->map3->stride + 3*out_ix + 2] += 1;
        }
    }
}

static void map_remove(global_map_t * glm, const grid_map_t * gm, double * xyt)
{
    glm->updated = true;
    image_u8x3_t * map3 = glm->map3;

    double c = cos(xyt[2]), s = sin(xyt[2]);
    for (int iy = 0; iy < gm->height; iy++) {
        for (int ix = 0; ix < gm->width; ix++) {
            uint8_t v = gm->data[iy*gm->width + ix];

            if (v == GRID_VAL_UNKNOWN) continue;
            if (v == GRID_VAL_OTHER) continue;

            double x = gm->x0 + ix*gm->meters_per_pixel;
            double y = gm->y0 + iy*gm->meters_per_pixel;

            double tx = x*c - y*s + xyt[0];
            double ty = x*s + y*c + xyt[1];

            int out_ix = (tx - glm->x0) / glm->meters_per_pixel;
            int out_iy = (ty - glm->y0) / glm->meters_per_pixel;

            if(out_ix < 0 || out_ix >= map3->width
               || out_iy < 0 || out_iy >= map3->height)
                continue;

            if (v == GRID_VAL_TRAVERSABLE &&
                map3->buf[out_iy*map3->stride + 3*out_ix + 0] > 0)
                map3->buf[out_iy*map3->stride + 3*out_ix + 0] -= 1;
            else if (v == GRID_VAL_OBSTACLE &&
                     map3->buf[out_iy*map3->stride + 3*out_ix + 1] > 0)
                map3->buf[out_iy*map3->stride + 3*out_ix + 1] -= 1;
            else if (v == GRID_VAL_SLAMMABLE &&
                     map3->buf[out_iy*map3->stride + 3*out_ix + 2] > 0)
                map3->buf[out_iy*map3->stride + 3*out_ix + 2] -= 1;
        }
    }
}

void global_map_add_gridmap(global_map_t * glm, const grid_map_t * gm, double * xyt)
{
    assert(!zhash_contains(glm->gms,&gm));

    zhash_put(glm->gms, &gm, xyt, NULL, NULL);

    map_add(glm, gm, xyt);
}

void global_map_move_gridmap(global_map_t * glm, const grid_map_t * gm, double * xyt)
{
    double old_xyt[3];
    assert(zhash_get(glm->gms, &gm, old_xyt));

    map_remove(glm, gm, old_xyt);

    map_add(glm, gm, xyt);

    zhash_put(glm->gms, &gm, xyt, NULL, NULL);
}

void global_map_remove_gridmap(global_map_t * glm, const grid_map_t * gm)
{
    double old_xyt[3];
    assert(zhash_get(glm->gms, &gm, old_xyt));

    map_remove(glm, gm, old_xyt);

    zhash_remove(glm->gms, &gm, NULL, NULL);
}

void global_map_get_xyt(global_map_t * glm, const grid_map_t * gm, double *xyt)
{
    assert(zhash_get(glm->gms, &gm, xyt));
}

const grid_map_t * global_map_get_map(global_map_t * glm)
{
    if (glm->updated) {
        glm->updated = false;
        glm->out_gm->utime = glm->last_update_utime;
        if (glm->out_map)
            image_u8_destroy(glm->out_map);
        glm->out_map = image_u8_create_alignment(glm->map3->width,
                                                 glm->map3->height,
                                                 glm->map3->width);

        for (int y = 0; y < glm->map3->height; y++) {
            for (int x = 0; x < glm->map3->width; x++) {
                uint8_t vtr = glm->map3->buf[glm->map3->stride*y + 3*x + 0];
                uint8_t vob = glm->map3->buf[glm->map3->stride*y + 3*x + 1];
                uint8_t vsl = glm->map3->buf[glm->map3->stride*y + 3*x + 2];

                if(vsl + vob > vtr)
                {
                    if(vob > vsl)
                        glm->out_map->buf[glm->out_map->stride*y + x] = GRID_VAL_OBSTACLE;
                    else if ( vsl > 0 )
                        glm->out_map->buf[glm->out_map->stride*y + x] = GRID_VAL_SLAMMABLE;
                }
                else if(vtr > 0)
                    glm->out_map->buf[glm->out_map->stride*y + x] = GRID_VAL_TRAVERSABLE;
                else
                    glm->out_map->buf[glm->out_map->stride*y + x] = GRID_VAL_UNKNOWN;
            }
        }
    }

    glm->out_gm->x0 = glm->x0;
    glm->out_gm->y0 = glm->y0;
    glm->out_gm->meters_per_pixel = glm->meters_per_pixel;
    glm->out_gm->width = glm->out_map->width;
    glm->out_gm->height = glm->out_map->height;
    glm->out_gm->datalen = glm->out_map->height * glm->out_map->width;
    glm->out_gm->data = glm->out_map->buf;

    return glm->out_gm;
}

grid_map_t * global_map_get_map_copy(global_map_t * glm)
{
    return gridmap_copy(global_map_get_map(glm));
}

void global_map_crop(global_map_t * glm)
{
    int PPM = 1.0/glm->meters_per_pixel;
    int PAD = PPM * PAD_M;

    image_u8x3_t *map3 = glm->map3;
    int minx = map3->width;
    int maxx = 0;
    int miny = map3->height;
    int maxy = 0;

    if (map3->width == 0 || map3->height == 0)
        return;

    for (int y = 0; y < map3->height; y++) {
        for (int x = 0; x < map3->width; x++) {
            uint8_t *pix = &map3->buf[y*map3->stride + 3*x];

            if (!(pix[0] || pix[1] || pix[2]))
                continue;

            minx = min(minx, x);
            miny = min(miny, y);

            maxx = max(maxx, x);
            maxy = max(maxy, y);
        }
    }

    if (minx > maxx || miny > maxy)
        return;

    // Retain padding
    minx = max(0, minx - PAD);
    miny = max(0, miny - PAD);
    maxx = min(map3->width-1, maxx+PAD);
    maxy = min(map3->height-1, maxy+PAD);

    int new_width = maxx - minx + 1;
    int new_height = maxy - miny + 1;

    int dw = map3->width - new_width;
    int dh = map3->height - new_height;

    // Nothing to do. Map is as small as reasonable
    if (dw <= 0 && dh <= 0)
        return;

    glm->updated = true;
    image_u8x3_t *new3 = image_u8x3_create_alignment(new_width, new_height, new_width);
    assert (new3->width == new3->stride/3);

    for (int y = 0; y < new3->height; y++) {
        int idx0 = y*new3->stride;
        int idx1 = (y+miny)*map3->stride + 3*minx;
        memcpy(&new3->buf[idx0],
               &map3->buf[idx1],
               new3->stride);
    }

    glm->map3 = new3;
    glm->x0 += (glm->meters_per_pixel * minx);
    glm->y0 += (glm->meters_per_pixel * miny);

    image_u8x3_destroy(map3);
}

