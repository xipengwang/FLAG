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

#ifndef _RS_H
#define _RS_H

typedef struct rs rs_t;
struct rs
{
    uint32_t nbits; // must be less than 8.
    uint32_t N;     // N = 1 << nbits.

    uint32_t poly;  // no implicitly set bits; 0x3 = x + 1.
    uint16_t *M;    // N*N multiplication table.
    uint16_t *inv;  // inverse lookup.

    uint32_t datasz, encsz;
    uint16_t *E;     // encsz x datasz encoding table
};

////////////////////////////////////////////////////////////////////
// nbits: number of bits per word. (e.g. 8). Practical upper-limit is
// 12 due to large memory usage.
//
// Note that the implementation passes around data in uint16_ts; this
// allows nbits to be greater than 8. (With nbits=8, the amount of
// redundancy is limited due to the corresponding limit of encsz.)
//
// poly: The polynomial to use, which must be irreducible. If you
// specify 0, a hard-coded polynomial is used.
//
// datasz: How many data words are encoded at once?
//
// encsz: How many encoded words are produced from datasz? (must be
// greater than or equal to datasz). Cannot be more than 2^nbits - 1.
rs_t *rs_create(int nbits, int poly, int datasz, int encsz);

// Used for debugging; verifies that the polynomial seems to be
// irreducible.
int rs_validate(rs_t *rs);

// Encode!
void rs_encode(rs_t *rs, const uint16_t *data, uint16_t *enc);

void rs_destroy(rs_t *rs);

typedef struct rs_decoder rs_erasure_decoder_t;
struct rs_decoder
{
    rs_t *rs;
    uint16_t *D; // datasz X datasz
};

////////////////////////////////////////////////////////////////////
// Decoder designed for erasures. (Error correction not supported.)
//
// encidx: Which encoded word indices were received? Must be datasz
// elements. Any subset of datasz elements of encoded data will result
// in successful decoding.
rs_erasure_decoder_t *rs_erasure_decoder_create(rs_t *rs, const uint16_t *encidx);

// Decode the data...
void rs_erasure_decoder_decode(rs_erasure_decoder_t *rsd, const uint16_t *enc, uint16_t *data);

void rs_erasure_decoder_destroy(rs_erasure_decoder_t *rsd);


// Berlekamp-Welch decoder. You pass it 'sz' encoded words whose
// indices are given by encidx. The data is contained in enc, and
// decoded data is returned in data.
//
// Returns the number of non-zero terms in the remainder of Q(x)/E(x),
// which is zero in the case of a successful decode.
//
// Should be able to recover from up to (sz-datasz)/2 errors.
//
// NB: This decoder is *thousands of times* slower than the erasure
// decoder.
int rs_bw_decoder(rs_t *rs, const uint16_t *encidx, int sz, const uint16_t *enc, uint16_t *data);

#endif
