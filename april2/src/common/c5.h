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

#ifndef _C5_H
#define _C5_H

#include <stdint.h>

#define C5_PAD 64

/** note that input and output buffers must be at least C5_PAD bytes longer than
    otherwise required.
**/
uint32_t uc5_length(const uint8_t *_in, int _inlen);

void uc5(const uint8_t *_in, int _inlen, uint8_t *_out, int *_outlen);

void c5(const uint8_t *_in, int _inlen, uint8_t *_out, int *_outlen);

#endif
