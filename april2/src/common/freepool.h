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

#ifndef _FREEPOOL_H
#define _FREEPOOL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct freepool freepool_t;

freepool_t *freepool_create();

// destroy should be a function of one argument; it will be passed
// 'p' when the freepool is destroyed.
// returns 'p', so it can be used inline
void *freepool_add(freepool_t *fp, void *p, void (*destroy)());

void freepool_destroy(freepool_t *fp);

#ifdef __cplusplus
}
#endif

#endif
