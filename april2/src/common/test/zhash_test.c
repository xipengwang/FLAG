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
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include "common/zhash.h"
#include "common/rand_util.h"

struct entry {
    char *key;
    char *value;
};

int main(int argc, char *argv[])
{

    if(1)
    {
        zhash_t * zh = zhash_create(sizeof(char*), sizeof(zarray_t*), zhash_str_hash, zhash_str_equals);

        char *key;
        zarray_t *vals;

        char keys[10][10] = {
            "alpha",
            "beta",
            "gamma",
            "delta",
            "epsilon",
            "zeta",
            "eta",
            "theta",
            "iota",
            "kappa",
        };

        for(int i = 0; i < 10; i++)
        {
            key = strdup(keys[i]);
            vals = zarray_create(sizeof(char*));
            zhash_put(zh, &key, &vals, NULL, NULL);
        }

        zhash_iterator_t vit;
        zhash_iterator_init(zh, &vit);
        while (zhash_iterator_next(&vit, &key, &vals)) {
            zhash_iterator_remove(&vit);
            zarray_vmap(vals, free);
            zarray_destroy(vals);
            free(key);
        }
        assert(zhash_size(zh) == 0);
        zhash_destroy(zh);
    }

    zhash_t *zh = zhash_create(sizeof(char*), sizeof(char*),
                               zhash_str_hash, zhash_str_equals);

    zarray_t *arr = zarray_create(sizeof(struct entry));

    int nextid = 0;

    while (1) {

        if (1) {
            for (int i = 0; i < zarray_size(arr); i++) {
                struct entry *e;
                zarray_get_volatile(arr, i, &e);

                char *v;
                zhash_get(zh, &e->key, &v);
                assert(!strcmp(v, e->value));
            }
        }

        if (1) {
            // check that iterator returns every pair exactly once
            uint8_t present[zarray_size(arr)];
            memset(present, 0, zarray_size(arr));

            zhash_iterator_t zit;
            zhash_iterator_init(zh, &zit);

            char *k, *v;
            while (zhash_iterator_next(&zit, &k, &v)) {
                int found = 0;
                for (int i = 0; i < zarray_size(arr); i++) {
                    struct entry *e;
                    zarray_get_volatile(arr, i, &e);
                    if (!strcmp(e->key, k)) {
                        present[i]++;
                        found++;
                    }
                }
                assert(found == 1);
            }

            for (int i = 0; i < zarray_size(arr); i++) {
                if (present[i] != 1) {
                    printf("present[%d] = %d\n", i, present[i]);
                    assert(0);
                }
            }
        }

//        printf("%d %d\n", zhash_size(zh), zarray_size(arr));
        if (zhash_size(zh) != zarray_size(arr)) {
            printf("%d %d\n", zhash_size(zh), zarray_size(arr));
            assert(0);
        }

        double uniform = randf_uniform(0,1);

        uniform -= 0.1;
        if (uniform < 0) {
            char buf[1024];
            sprintf(buf, "K%d", nextid);
            char *key = strdup(buf);

            sprintf(buf, "V%d", nextid);
            char *value = strdup(buf);

            printf("insert %s => %s\n", key, value);
            struct entry e = { .key = key, .value = value };
            zarray_add(arr, &e);

            if (zhash_put(zh, &key, &value, NULL, NULL))
                assert(0);

            nextid++;
            continue;
        }

        uniform -= 0.05;
        if (uniform < 0 && zarray_size(arr) > 0) {
            int idx = (int) (randf_uniform(0,1) * zarray_size(arr));

            struct entry *e;
            zarray_get_volatile(arr, idx, &e);
            printf("remove %s\n", e->key);

            char *oldkey, *oldvalue;
            if (!zhash_remove(zh, &e->key, &oldkey, &oldvalue))
                assert(0);
            assert(e->key == oldkey);
            assert(e->value == oldvalue);
            zarray_remove_index(arr, idx, 1);
            continue;
        }

        uniform -= 0.10;
        if (uniform < 0) {
            zhash_iterator_t zit;
            zhash_iterator_init(zh, &zit);

            char *k, *v;
            while (zhash_iterator_next(&zit, &k, &v)) {

                if (randf_uniform(0,1) < 0.2) {
                    int found = -1;
                    for (int i = 0; i < zarray_size(arr); i++) {
                        struct entry *e;
                        zarray_get_volatile(arr, i, &e);
                        if (!strcmp(e->key, k)) {
                            found = i;
                        }
                    }
                    assert(found >= 0);

                    printf("removing iterator\n");
                    zhash_iterator_remove(&zit);
                    zarray_remove_index(arr, found, 1);
                }
            }
            continue;
        }

        uniform -= 0.01;
        if (uniform < 0) {
            printf("copy\n");
            zh = zhash_copy(zh);
        }
    }
}
