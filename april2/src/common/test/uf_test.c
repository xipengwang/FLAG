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

#include <stdlib.h>
#include <assert.h>

#include "common/unionfind.h"
#include "common/zarray.h"

typedef struct
{
    int a;
    int b;
}edge_t;


void special_test()
{

    unionfind_t *uf = unionfind_create(3);

    unionfind_connect(uf,1,0);

    assert(unionfind_get_representative(uf, 0) ==
           unionfind_get_representative(uf, 1));

    unionfind_destroy(uf);
}
int main()
{
    special_test();

    int sz = 10000;

    unionfind_t *uf = unionfind_create(sz);

    zarray_t * edges = zarray_create(sizeof(edge_t));
    for (int i = 0; i < 1000; i++) {
        edge_t e = {.a = rand()%sz,
                    .b = rand()%sz};
        zarray_add(edges, &e);

        unionfind_connect(uf, e.a, e.b);

        // validate all edges so far:
        for (int j = 0; j < zarray_size(edges); j++) {
            edge_t g;
            zarray_get(edges, j, &g);

            int repa = unionfind_get_representative(uf, g.a);
            int repb = unionfind_get_representative(uf, g.b);

            assert(repa == repb);


            int sza = unionfind_get_set_size(uf, g.a);
            int szb = unionfind_get_set_size(uf, g.b);

            assert(sza == szb);
        }
    }
    zarray_destroy(edges);
    unionfind_destroy(uf);
}
