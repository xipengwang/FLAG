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

#ifndef __VX_TEXT_H__
#define __VX_TEXT_H__

enum {
    VXO_TEXT_ANCHOR_TOP_LEFT = 1000, VXO_TEXT_ANCHOR_TOP_LEFT_ROUND,
    VXO_TEXT_ANCHOR_TOP, VXO_TEXT_ANCHOR_TOP_ROUND,
    VXO_TEXT_ANCHOR_TOP_RIGHT,VXO_TEXT_ANCHOR_TOP_RIGHT_ROUND,
    VXO_TEXT_ANCHOR_LEFT, VXO_TEXT_ANCHOR_LEFT_ROUND,
    VXO_TEXT_ANCHOR_CENTER, VXO_TEXT_ANCHOR_CENTER_ROUND,
    VXO_TEXT_ANCHOR_RIGHT, VXO_TEXT_ANCHOR_RIGHT_ROUND,
    VXO_TEXT_ANCHOR_BOTTOM_LEFT, VXO_TEXT_ANCHOR_BOTTOM_LEFT_ROUND,
    VXO_TEXT_ANCHOR_BOTTOM, VXO_TEXT_ANCHOR_BOTTOM_ROUND,
    VXO_TEXT_ANCHOR_BOTTOM_RIGHT, VXO_TEXT_ANCHOR_BOTTOM_RIGHT_ROUND };

enum {
    VXO_TEXT_JUSTIFY_LEFT = 2000,
    VXO_TEXT_JUSTIFY_RIGHT,
    VXO_TEXT_JUSTIFY_CENTER
};

enum {
    VXO_TEXT_PLAIN = 0,
    VXO_TEXT_ITALIC = 1,
    VXO_TEXT_BOLD = 2
};

typedef struct vxo_text vxo_text_t;
struct vxo_text
{
    vx_object_t  vxo; // must be first element.
    vx_object_t *chain;
    double       total_width;
    double       total_height;
};

/**
 * Create a text object. Pass in a string with markup in <<double-brackets>>.
 *
 * Format options:
 *    - Separate multiple options with commas ',' and no spaces
 *    - Colors. Specify in #RRGGBB or #RRGGBBAA format
 *    - Fonts. Options: 'serif', 'sansserif', 'monospaced'. Modify with '-bold', or '-italic'
 *    - Alignment. Options: 'left', 'right', 'center'
 *
 * Example:
 *    "<<monospaced-bold-16,left,#ff0000>>Example string\nSecond line"
 *
 * specifies font "monospaced-bold" scaled to 16 units high. (default is 18 units).
 */
vx_object_t *vxo_text(int anchor, const char *fmt, ...);

/**
 * Get the width or height from a text object returned by vxo_text_create()
 */
double vxo_text_get_width(vx_object_t *vo);
double vxo_text_get_height(vx_object_t *vo);

#endif
