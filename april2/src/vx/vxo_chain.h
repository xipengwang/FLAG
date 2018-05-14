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

#ifndef _VXO_CHAIN_H
#define _VXO_CHAIN_H

// Specifies the origin of the coordinate space. Note that X goes
// right, Y goes up. So VXO_PIXCOORDS_TOP_LEFT puts the origin at the
// top left corner, and you'll need negative Y coordinates to see
// anything.
enum {
    VXO_PIXCOORDS_TOP_LEFT, VXO_PIXCOORDS_TOP, VXO_PIXCOORDS_TOP_RIGHT,
    VXO_PIXCOORDS_LEFT, VXO_PIXCOORDS_CENTER, VXO_PIXCOORDS_CENTER_ROUND, VXO_PIXCOORDS_RIGHT,
    VXO_PIXCOORDS_BOTTOM_LEFT, VXO_PIXCOORDS_BOTTOM, VXO_PIXCOORDS_BOTTOM_RIGHT
};

enum {
    // one unit to one pixel
    VXO_PIXCOORDS_SCALE_MODE_ONE = 0,

    // horiz and vert are scaled so that a value of 1 is the width of the screen.
    VXO_PIXCOORDS_SCALE_MODE_WIDTH = 1,

    // horiz and vert are scaled so that a value of 1 is the height of the screen.
    VXO_PIXCOORDS_SCALE_MODE_HEIGHT = 2,

    // horiz and vert are scaled so that a value of 1 is the min(width, height) of the screen.
    VXO_PIXCOORDS_SCALE_MODE_MIN = 3,

    // horiz and vert are scaled so that a value of 1 is the max(width, height) of the screen.
    VXO_PIXCOORDS_SCALE_MODE_MAX = 4
};

// the vx_chain becomes the owner of the objects, and will call their
// destructors when the chain is destroyed.
vx_object_t *vxo_chain(vx_object_t *first, ...)  __attribute__ ((__sentinel__));
vx_object_t *vxo_pixcoords(int origin, int scale_mode, ...) __attribute__ ((__sentinel__));
vx_object_t *vxo_depth_test(int enable, ...) __attribute__ ((__sentinel__));

void vxo_chain_add(vx_object_t *chain, vx_object_t *obj);

#endif
