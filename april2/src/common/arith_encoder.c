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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

#include "arith_encoder.h"

typedef struct bit_output_stream bit_output_stream_t;
typedef struct bit_input_stream bit_input_stream_t;

struct arith_encoder
{
    bit_output_stream_t *bouts;

    uint64_t L; // the bounds of the number we are encoding, inclusive.
    uint64_t H; // e.g., [L, H].
    uint32_t overflow;
};

struct arith_decoder
{
    bit_input_stream_t *bins;

    // L, H, and V jointly describe the floating point number for the
    // subsequent symbols:
    //
    // (L-V) / (H-L)
    //
    // Bits are shifted out of V in powers of two so that no error is
    // accumulated due to divisions. These bits are shifted out *exactly*
    // the same way as the encoder writes bits out.
    uint64_t L;
    uint64_t H;
    uint64_t V;
};

/////////////////////////////////////////////////////////////
struct bit_output_stream
{
    uint8_t *bytes;
    int nbytes;
    int capacity;

    uint8_t bits;
    int nbits;
};

bit_output_stream_t *bit_output_stream_create()
{
    bit_output_stream_t *bouts = calloc(1, sizeof(bit_output_stream_t));

    return bouts;
}

void bit_output_stream_write(bit_output_stream_t *bouts, int b)
{
//    printf("out %d\n", b);

    assert(b >= 0 && b <= 1);

    // append a bit, LSB first
    if (b)
        bouts->bits |= (1 << bouts->nbits);

    bouts->nbits++;

    // flush a byte?
    if (bouts->nbits == 8) {

        // grow the byte buffer
        if (bouts->nbytes == bouts->capacity) {
            bouts->capacity *= 2;
            if (bouts->capacity < 128)
                bouts->capacity = 128;
            bouts->bytes = realloc(bouts->bytes, bouts->capacity);
        }

        // append the byte
        bouts->bytes[bouts->nbytes++] = bouts->bits;

        // reset for the next bit(s)
        bouts->bits = 0;
        bouts->nbits = 0;
    }
}

void bit_output_stream_finish(bit_output_stream_t *bouts)
{
    // while a partial byte is in the buffer, keep adding zeros.
    while (bouts->nbits)
        bit_output_stream_write(bouts, 0);
}

void bit_output_stream_destroy(bit_output_stream_t *bouts)
{
    if (!bouts)
        return;

    free(bouts->bytes);
    free(bouts);
}

/////////////////////////////////////////////////////////////
// hallucinates an infinite # of zeros at EOF
struct bit_input_stream
{
    uint8_t *bytes;
    int pos;
    int nbytes;

    uint8_t bits;
    int nbits;
};

// we are only borrowing the data; user still owns it.
bit_input_stream_t *bit_input_stream_create(uint8_t *data, int datalen)
{
    bit_input_stream_t *bins = calloc(1, sizeof(bit_input_stream_t));
    bins->bytes = data;
    bins->nbytes = datalen;
    bins->nbits = 0;
    return bins;
}

int bit_input_stream_read(bit_input_stream_t *bins)
{
    // refill buffer?
    if (bins->nbits == 0) {
        if (bins->pos < bins->nbytes) {
            bins->bits = bins->bytes[bins->pos++];
            bins->nbits = 8;
        } else {
            // EOF. hallucinate zeros.
            bins->bits = 0;
            bins->nbits = 8;
        }
    }

    // retrieve LSB first
    int b = (bins->bits >> (8 - bins->nbits)) & 1;
    bins->nbits--;

    return b;
}

void bit_input_stream_destroy(bit_input_stream_t *bins)
{
    if (!bins)
        return;
    free(bins);
}

/////////////////////////////////////////////////////////////
// a simple model based on a 0-th order statistics. Not intended for
// serious use; just an example of the usage of arith_encoder.
// EOF symbol = 256
static uint32_t order0_get_cumulative(arith_model_t *model, int sym)
{
    uint32_t *chist = (uint32_t*) model->user;

    return chist[sym];
}

static int order0_lookup(arith_model_t *model, uint32_t v)
{
    uint32_t *chist = (uint32_t*) model->user;

    // terrible implementation. Use binary search or something.
    for (int i = 1; i <= model->nsyms; i++)
        if (v < chist[i])
            return i - 1;

    printf("order0_lookup: %d, %d\n", v, chist[model->nsyms]);
    assert(0);
    return -1;
}

static void order0_update(arith_model_t *model, int sym)
{
    uint32_t *chist = (uint32_t*) model->user;
    for (int i = sym + 1; i <= model->nsyms; i++)
        chist[i]++;
}

static void order0_destroy(arith_model_t *model)
{
    free(model->user);
    free(model);
}

arith_model_t *arith_model_order0_create(int nsyms)
{
    arith_model_t *model = calloc(1, sizeof(arith_model_t));
    model->get_cumulative = order0_get_cumulative;
    model->lookup = order0_lookup;
    model->update = order0_update;
    model->destroy = order0_destroy;
    model->nsyms = nsyms;

    model->user = calloc(model->nsyms + 1, sizeof(uint32_t)); // 4GB max
    uint32_t *chist = (uint32_t*) model->user;

    for (int i = 1; i <= model->nsyms; i++)
        chist[i] = i;

    return model;
}

/////////////////////////////////////////////////////////////
// a simple model based on a 0-th order statistics. Not intended for
// serious use; just an example of the usage of arith_encoder.
static uint32_t fixed_get_cumulative(arith_model_t *model, int sym)
{
    uint32_t *chist = (uint32_t*) model->user;

    return chist[sym];
}

static int fixed_lookup(arith_model_t *model, uint32_t v)
{
    uint32_t *chist = (uint32_t*) model->user;

    // terrible implementation. Use binary search or something.
    for (int i = 1; i <= model->nsyms; i++)
        if (v < chist[i])
            return i - 1;

    printf("%d\n", v);
    assert(0);
    return -1;
}

static void fixed_update(arith_model_t *model, int sym)
{
    return;
}

static void fixed_destroy(arith_model_t *model)
{
    free(model);
}

arith_model_t *arith_model_fixed_create(uint32_t *chist, int nsyms)
{
    arith_model_t *model = calloc(1, sizeof(arith_model_t));
    model->get_cumulative = fixed_get_cumulative;
    model->lookup = fixed_lookup;
    model->update = fixed_update;
    model->destroy = fixed_destroy;
    model->nsyms = nsyms;
    model->user = chist;

    return model;
}

/////////////////////////////////////////////////////////////
arith_encoder_t *arith_encoder_create()
{
    arith_encoder_t *ae = calloc(1, sizeof(arith_encoder_t));
    ae->L = 0;
    ae->H = 0xffffffff;
    ae->overflow = 0;
    ae->bouts = bit_output_stream_create();
    return ae;
}

static void write_bit_with_overflow(arith_encoder_t *ae, int bit)
{
    bit_output_stream_write(ae->bouts, bit);

//    if (ae->overflow)
//        printf("OVERFLOW %d\n", ae->overflow);

    for (; ae->overflow > 0; ae->overflow--)
        bit_output_stream_write(ae->bouts, 1 - bit);

}

/*
  One symbol at a time, the encoder scales the range [L, H] according
  to the span of the symbol. This span is then "reduced" by shifting
  out bits that have been resolved.

  The decoder will extract the same symbol and perform exactly the
  same scaling and bitwise reductions. Thus the decoder computes the
  exact same [L, H] bounds as the encoder.

  The main difference is that the decoder has access to "future bits"
  as well. However, this doesn't affect the symbol currently being
  decoded.
*/
int arith_encoder_write(arith_encoder_t *ae, arith_model_t *model, int sym)
{
    assert(sym < model->nsyms);

    int64_t S0 = model->get_cumulative(model, sym);
    int64_t S1 = model->get_cumulative(model, sym + 1);
    int64_t SN = model->get_cumulative(model, model->nsyms);

    // Lp: Add one to make the range narrower (conservative rounding).
    uint32_t Lp = ae->L + S0 * ((uint64_t) ae->H - ae->L) / SN + 1;

    // Hp: our upper range is anything lower than what chist[sym+1] would map to.
    uint32_t Hp = ae->L + S1 * ((uint64_t) ae->H - ae->L) / SN - 1;

//    printf("[%08x, %08x] ==> [%08x, %08x]\n", ae->L, ae->H, Lp, Hp);

    ae->L = Lp;
    ae->H = Hp;

    // Now, shift out bits that have been determined. We must maintain
    // a minimum level of precision, i.e., H-L >= 0x40000000.
    while (1) {
        if (ae->L >= 0x80000000) {
            // both L and H begin with a one.
            write_bit_with_overflow(ae, 1);
            ae->L -= 0x80000000;
            ae->H -= 0x80000000;
        } else if (ae->H < 0x80000000) {
            // both L and H begin with a zero.
            write_bit_with_overflow(ae, 0);
        } else if (ae->L >= 0x40000000 && ae->H <= 0xbfffffff) {
            // Here's the tricky case... L and H are quite close
            // together, and we don't know whether a small increment
            // to L will cause the bits to roll over (so that L looks
            // like 0b10aaaaaa) or not.  But we still need to maintain
            // enough bits of precision, and all of these carry bits
            // will ultimately flip the same way. So we remove the bit
            // and make a note of the bit we must eventually output.
            // L = 0b01aaaaa
            // H = 0b10bbbbb
            ae->overflow++;
            ae->L -= 0x40000000;
            ae->H -= 0x40000000;
        } else {
            // In any other case, H-L >= 0x40000000 and no bits
            // can be shifted out.
            break;
        }

        // Shift the bits, updating the lower and upper bounds.
        ae->L = 2*ae->L;
        ae->H = 2*ae->H + 1;
    }

    assert(ae->H - ae->L >= 0x40000000);

    // wrote 1 byte.
    return 1;
}

void arith_encoder_finish(arith_encoder_t *ae, uint8_t **outdata, int *outdata_len)
{
    // need to output enough bits to unique encode a value between L
    // and H. If we stopped now and L is > 0, then we will have
    // encoded too small a number.
    for (uint32_t bit = 0x80000000; bit != 0; bit >>= 1) {
        if (ae->L & bit) {
            // if this bit of L is high, we must write 1 (can't write
            // anything bigger than 1!)
            write_bit_with_overflow(ae, 1);
        } else {
            // if this bit of L is low, then we might be able to write
            // a value between L and H by writing a one. This would
            // require fewer bits than writing out all of L.
            if (1 && ae->L + bit < ae->H) {
                write_bit_with_overflow(ae, 1);
                break;
            } else {
                write_bit_with_overflow(ae, 0);
            }
        }
    }

    if (ae->overflow)
        write_bit_with_overflow(ae, 0);

    bit_output_stream_finish(ae->bouts);

    // steal the data from the bouts.
    *outdata = ae->bouts->bytes;
    *outdata_len = ae->bouts->nbytes;

    ae->bouts->bytes = NULL;
    ae->bouts->nbytes = 0;
}

void arith_encoder_destroy(arith_encoder_t *ae)
{
    if (!ae)
        return;

    bit_output_stream_destroy(ae->bouts);
    free(ae);
}

/////////////////////////////////////////////////////////////
arith_decoder_t *arith_decoder_create(uint8_t *data, int datalen)
{
    arith_decoder_t *ad = calloc(1, sizeof(arith_decoder_t));
    ad->bins = bit_input_stream_create(data, datalen);
    ad->L = 0;
    ad->H = 0xffffffff;

    // load the first 32 bits in... Input stream will hallucinate 0s
    // if necessary.
    for (int i = 0; i < 32; i++)
        ad->V = (ad->V << 1) | bit_input_stream_read(ad->bins);
    return ad;
}

int arith_decoder_read(arith_decoder_t *ad, arith_model_t *model)
{
    // our number is (V-L)*0xffffffff / (H-L)

    // what is our current value, scaled to [0, SN]?
    int32_t SN = model->get_cumulative(model, model->nsyms);

    // The computation of SV is where precision becomes critical. We must
    uint32_t SV = ((uint64_t) ad->V - ad->L) * SN / (ad->H - ad->L);

    int sym = model->lookup(model, SV);
    int64_t S0 = model->get_cumulative(model, sym);
    int64_t S1 = model->get_cumulative(model, sym + 1);

    // Lp: Add one to make the range narrower (conservative rounding).
    uint32_t Lp = ad->L + S0 * ((uint64_t) ad->H - ad->L) / SN + 1;
    // Hp: our upper range is anything lower than what chist[sym+1] would map to.
    uint32_t Hp = ad->L + S1 * ((uint64_t) ad->H - ad->L) / SN - 1;

//    printf("[%08x, %08x], V=%08x ==> [%08x, %08x]\n", ad->L, ad->H, ad->V, Lp, Hp);

    ad->L = Lp;
    ad->H = Hp;

    while (1) {
        if (ad->L >= 0x80000000) {
            // discard a 1 bit
            ad->L -= 0x80000000;
            ad->H -= 0x80000000;
            ad->V -= 0x80000000;
        } else if (ad->H < 0x80000000) {
            // discard a 0 bit
        } else if (ad->L >= 0x40000000 && ad->H <= 0xbfffffff) {
            // discard an overflow bit
            ad->L -= 0x40000000;
            ad->H -= 0x40000000;
            ad->V -= 0x40000000;
        } else {
            break;
        }

        ad->L = 2*ad->L;
        ad->H = 2*ad->H + 1;
        ad->V = 2*ad->V + bit_input_stream_read(ad->bins);
    }

    return sym;
}

void arith_decoder_destroy(arith_decoder_t *ae)
{
    if (!ae)
        return;

    bit_input_stream_destroy(ae->bins);
    free(ae);
}
