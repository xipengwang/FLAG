#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#include "common/image_u8x3.h"
#include "common/image_u32.h"
#include "common/unionfind.h"

struct edge
{
    uint32_t ida;
    uint32_t idb;
    uint32_t weight;
};

static int rgb_error(image_u8x3_t *im,
                       int x0, int y0,
                       int x1, int y1)
{
    int rgberr = 0;
    for (int c = 0; c < 3; c++) {
        int v0 = im->buf[y0*im->stride + 3*x0 + c];
        int v1 = im->buf[y1*im->stride + 3*x1 + c];
        rgberr += (v0-v1)*(v0-v1);
    }

    return 100 * sqrtf(rgberr);
}

static inline void add_edge(struct edge *edges, int *nedges, image_u8x3_t *im,
                            int x0, int y0,
                            int x1, int y1)
{
    edges[*nedges].ida = y0 * im->width + x0;
    edges[*nedges].idb = y1 * im->width + x1;
    edges[*nedges].weight = rgb_error(im, x0, y0, x1, y1);
    (*nedges)++;

}

// min_size >= 1
// higher k = join more
image_u32_t *fhsegment_u8x3(image_u8x3_t *im, double k, int min_size)
{
    int w = im->width, h = im->height;

    struct edge *edges = calloc(sizeof(struct edge), 4*w*h);
    int nedges = 0;

    for (int y = 1; y + 1 < h; y++) {
        for (int x = 1; x + 1 < w; x++) {
            add_edge(edges, &nedges, im, x, y, x + 1, y);
            add_edge(edges, &nedges, im, x, y, x + 1, y + 1);
            add_edge(edges, &nedges, im, x, y, x, y + 1);
            add_edge(edges, &nedges, im, x, y, x + 1, y - 1);
        }
    }

    float *thresh = malloc(sizeof(float) * w * h);
    for (int i = 0; i < w*h; i++)
        thresh[i] = k;


    // process edges in order of increasing weight. Sort them using
    // counting sort.
    if (1) {
        int maxerr = 100 * sqrt(3 * 255 * 255) + 1;
        int *counts = calloc(sizeof(int), maxerr);
        for (int i = 0; i < nedges; i++)
            counts[edges[i].weight]++;

        // accumulate
        for (int i = 1; i < maxerr; i++)
            counts[i] += counts[i - 1];

        struct edge *new_edges = calloc(sizeof(struct edge), nedges);

        for (int i = 0; i < nedges; i++) {
            int w = edges[i].weight;
            counts[w]--;
            new_edges[counts[w]] = edges[i];
        }

        free(edges);
        edges = new_edges;
    }

    // connect the components
    unionfind_t *uf = unionfind_create(w*h);

    for (int i = 0; i < nedges; i++) {
        uint32_t ida = unionfind_get_representative(uf, edges[i].ida);
        uint32_t idb = unionfind_get_representative(uf, edges[i].idb);
        if (ida == idb)
            continue;

        if (edges[i].weight < thresh[ida] && edges[i].weight < thresh[idb]) {
            uint32_t root = unionfind_connect(uf, ida, idb);

            int sz = unionfind_get_set_size(uf, root);
            thresh[root] = edges[i].weight + k / (sz); // XXX classic FH is just sz
        }
    }

    // enforce minimum component size. Join components that are too
    // small by using best edges first.
    if (min_size > 1) {
        for (int i = 0; i < nedges; i++) {
            if (unionfind_get_set_size(uf, edges[i].ida) < min_size ||
                unionfind_get_set_size(uf, edges[i].idb) < min_size)
                unionfind_connect(uf, edges[i].ida, edges[i].idb);
        }

    }

    image_u32_t *out = image_u32_create(w, h);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            out->buf[y*out->stride + x] = unionfind_get_representative(uf, y*w + x);

    free(edges);
    unionfind_destroy(uf);
    free(thresh);

    return out;
}
