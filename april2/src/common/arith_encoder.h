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

#ifndef _ARITH_ENCODER_H
#define _ARITH_ENCODER_H

typedef struct arith_model arith_model_t;
struct arith_model {
    /** The cumulative frequency of symbols less than v. If
     * sym==nsyms, the model should return the maximum cumulative
     * frequency (identical to get_cumulative(largest_symbol + 1). In
     * other words, the range for symbol v is getCumulative(v) to
     * getCumulative(v+1). As a consequence, get_cumulative(0) should
     * be 0.
     *
     * getCumulative(nsyms) must be no larger than 0x40000000.
     **/
    uint32_t (*get_cumulative)(arith_model_t *model, int sym);

    /** This is the inverse of get_cumulative(). **/
    int (*lookup)(arith_model_t *model, uint32_t v);

    /** Indicates to the model that we've emitted a symbol, and the
     * model should potentially update. This function is not actually
     * called by the arith_encode or _decode functions; the user must
     * call it between symbols. **/
    void (*update)(arith_model_t *model, int sym);

    void (*destroy)(arith_model_t *model);

    // how many symbols are used?
    int nsyms;

    // for use by the model
    void *user;
};

typedef struct arith_encoder arith_encoder_t;
typedef struct arith_decoder arith_decoder_t;

arith_encoder_t *arith_encoder_create();
int arith_encoder_write(arith_encoder_t *ae, arith_model_t *model, int sym);
void arith_encoder_finish(arith_encoder_t *ae, uint8_t **outdata, int *outdata_len);
void arith_encoder_destroy(arith_encoder_t *ae);

arith_decoder_t *arith_decoder_create(uint8_t *data, int datalen);
int arith_decoder_read(arith_decoder_t *ad, arith_model_t *model);
void arith_decoder_destroy(arith_decoder_t *ae);

arith_model_t *arith_model_order0_create(int nsyms);

// provide your own histogram, which the model "borrows".
arith_model_t *arith_model_fixed_create(uint32_t *chist, int nsyms);

#endif
