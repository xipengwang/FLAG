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

/** Usage:

// what is the base name of the type to create?
// This prefix will be used for all functions. A "_t" will be appended for the type name.
#define TNAME string_array

// What is the type to be contained in the type? Note, void* won't work because
// we can't automatically construct a pointer-to-void* by adding an asterisk.
#define TVALTYPE char*

#include "common/tarray_impl.h"

#undef TNAME
#undef TVALTYPE

 **/

#include <stdlib.h>
#include <string.h>

#define TRRFN(root, suffix) root ## _ ## suffix
#define TRFN(root, suffix) TRRFN(root, suffix)
#define TFN(suffix) TRFN(TNAME, suffix)

#define TTYPENAME TFN(t)

typedef struct TTYPENAME TTYPENAME;
struct TTYPENAME
{
    TVALTYPE *data;
    int alloc;
    int size;
};

static inline TTYPENAME *TFN(create)()
{
    TTYPENAME *za = calloc(1, sizeof(TTYPENAME));
    return za;
}

static inline int TFN(size)(TTYPENAME *za)
{
    return za->size;
}

static inline void TFN(ensure_capacity)(TTYPENAME *za, int capacity)
{
    if (capacity <= za->alloc)
        return;

    while (za->alloc < capacity) {
        za->alloc *= 2;
        if (za->alloc < 8)
            za->alloc = 8;
    }

    za->data = realloc(za->data, za->alloc * sizeof(TVALTYPE));
}

static inline TTYPENAME *TFN(create_capacity)(int capacity)
{
    TTYPENAME *za = calloc(1, sizeof(TTYPENAME));
    TFN(ensure_capacity)(za, capacity);
    return za;
}

static inline void TFN(destroy)(TTYPENAME *za)
{
    if (!za)
        return;
    free(za->data);
    free(za);
}

static inline void TFN(get)(TTYPENAME *za, int idx, TVALTYPE *p)
{
    *p = za->data[idx];
}

static inline void TFN(get_volatile)(TTYPENAME *za, int idx, TVALTYPE **p)
{
    *p = &za->data[idx];
}

static inline void TFN(add)(TTYPENAME *za, TVALTYPE *p)
{
    TFN(ensure_capacity)(za, za->size + 1);
    za->data[za->size] = *p;
    za->size++;
}

#undef TRRFN
#undef TRFN
#undef TFN

#undef TTYPENAME
