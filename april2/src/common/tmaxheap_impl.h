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
#include <float.h>

#define TRRFN(root, suffix) root ## _ ## suffix
#define TRFN(root, suffix) TRRFN(root, suffix)
#define TFN(suffix) TRFN(TNAME, suffix)

#define TTYPENAME TFN(t)

struct TFN(entry)
{
    TVALTYPE value;
    float    priority;
};

typedef struct TTYPENAME TTYPENAME;
struct TTYPENAME
{
    struct TFN(entry) *entries;
    int       alloc;
    int       size;
};

static inline TTYPENAME *TFN(create)()
{
    TTYPENAME *heap = calloc(1, sizeof(TTYPENAME));
    return heap;
}

static inline void TFN(destroy)(TTYPENAME *heap)
{
    if (!heap)
        return;

    free(heap->entries);
    free(heap);
    return;
}

static inline int TFN(size)(TTYPENAME *heap)
{
    return heap->size;
}

static inline void TFN(swap)(TTYPENAME *heap, int a, int b)
{
    struct TFN(entry) t = heap->entries[a];
    heap->entries[a] = heap->entries[b];
    heap->entries[b] = t;
}

static inline void TFN(ensure_capacity)(TTYPENAME *heap, int capacity)
{
    if (heap->alloc >= capacity)
        return;

    int newcap = heap->alloc;
    while (newcap < capacity) {
        newcap *= 2;
        if (newcap < 8)
            newcap = 8;
    }

    heap->entries = realloc(heap->entries, newcap * sizeof(struct TFN(entry)));
    heap->alloc = newcap;
}

static inline void TFN(add)(TTYPENAME *heap, TVALTYPE *value, float priority)
{
    TFN(ensure_capacity)(heap, heap->size + 1);

    int idx = heap->size;
    heap->entries[idx].value = *value;
    heap->entries[idx].priority = priority;

    heap->size++;

    while (idx > 0) {
        int parent = (idx - 1) / 2;

        // we're done!
        if (heap->entries[parent].priority >= priority)
            break;

        // else, swap and recurse upwards
        TFN(swap)(heap, idx, parent);
        idx = parent;
    }
}

static inline int TFN(remove_index)(TTYPENAME *heap, int idx, TVALTYPE *value, float *priority)
{
    if (idx >= heap->size)
        return 0;

    if (value)
        *value = heap->entries[idx].value;
    if (priority)
        *priority = heap->entries[idx].priority;

    heap->size--;

    // If this element is already the last one, then there's nothing
    // for us to do.
    if (idx == heap->size)
        return 1;

    // copy last element to first element. (which probably upsets
    // the heap property).
    heap->entries[idx] = heap->entries[heap->size];

    // now fix the heap. Note, as we descend, we're "pushing down"
    // the same node the entire time. Thus, while the index of the
    // parent might change, the parent_priority doesn't.
    int parent = idx;
    float parent_priority = heap->entries[idx].priority;

    // descend, fixing the heap.
    while (parent < heap->size) {

        int left = 2*parent + 1;
        int right = left + 1;

        float left_priority = (left < heap->size) ? heap->entries[left].priority : -FLT_MAX;
        float right_priority = (right < heap->size) ? heap->entries[right].priority : -FLT_MAX;

        // put the biggest of (parent, left, right) as the parent.

        // already okay?
        if (parent_priority >= left_priority && parent_priority >= right_priority)
            break;

        // if we got here, then one of the children is bigger than the parent.
        if (left_priority >= right_priority) {
            TFN(swap)(heap, parent, left);
            parent = left;
        } else {
            TFN(swap)(heap, parent, right);
            parent = right;
        }
    }

    return 1;
}

static inline int TFN(remove_max)(TTYPENAME *heap, TVALTYPE *p, float *v)
{
    return TFN(remove_index)(heap, 0, p, v);
}

#undef TRRFN
#undef TRFN
#undef TFN

#undef TTYPENAME
