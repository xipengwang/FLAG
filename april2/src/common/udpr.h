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

#ifndef _UDPR_H
#define _UDPR_H

#include <stdint.h>
#include "common/zarray.h"
#include "common/zhash.h"
#include "common/net_util.h"

///////////////////////////////////////////////////////////////////////
// UDPR_SEND
///////////////////////////////////////////////////////////////////////
typedef struct udpr_send_params udpr_send_params_t;
struct udpr_send_params
{
    // should be smaller than MTU - ~64
    int fragment_size;

    // after this many ms of not receiving an ACK, we will tickle the receiver
    // with a REQUEST_ACK message
    int ack_timeout_ms;

    // [0, 1]. When non-zero, messages will be probabilistically dropped
    float drop_rate;

    // how long to sleep after sending a packet
    uint32_t send_sleep_us;

    int verbosity;
};

// initializes parameters to reasonable defaults
void udpr_send_params_init(udpr_send_params_t *params);


// addr: to whom should we be sending? We will create an ephemeral UDP
// socket; we must be able to receive on that ephemeral port too.
//

// returns 0 on success

int udpr_send(udpr_send_params_t *params,
              const struct sockaddr_in *addr,
              const void *_data, const int datalen, const uint8_t guid[32]);

///////////////////////////////////////////////////////////////////////
// UDPR_RECEIVER
///////////////////////////////////////////////////////////////////////
typedef struct udpr_receiver_params udpr_receiver_params_t;
struct udpr_receiver_params
{
    // when a message becomes this old (whether or not it was fully
    // received) it is discarded. If we receive another fragment, we will
    // initiate a new receive operation for it.

    // expire messages that are incomplete (and possibly still in
    // progress). Usually make this big (as long as your worst case
    // transmission delay).
    uint32_t expire_incomplete_ms;

    // expire messages that are complete. Make this long enough to ensure
    // that any fragments trickling through the system finish.
    uint32_t expire_complete_ms;

    // [0, 1]. When non-zero, messages will be probabilistically dropped
    float drop_rate;

    // how long to sleep after sending a packet
    uint32_t send_sleep_us;

    // controls how accurately we can pre-emptively send ACKs.
    uint32_t poll_timeout_ms;

    // send an ACK even without a REQUEST ACK.
    uint32_t unacked_fragments_max;

    int verbosity;
};

typedef struct udpr_receiver_data udpr_receiver_data_t;
struct udpr_receiver_data
{
    uint64_t utime; // of first fragment
    uint8_t  guid[32];

    void *data;
    int   datalen;

    struct   sockaddr_in sender_addr;

    //////////////////////////////////////
    // these are for internal bookkeeping.
    uint32_t fragment_size;
    uint32_t nfragments;
    uint32_t nfragments_received;
    uint8_t  *received;

    uint32_t sequence_number; // our outgoing sequence numbers

    // sequence number from sender attached to most recently received
    // fragment
    uint32_t last_sequence_number;

    uint32_t last_echoed_sequence_number;

    // At which time should we next send an ack?
    uint32_t unacked_fragments;
};

// The receiver, unlike the sender, is persistent. This is to solve
// the "last FIN" problem--- if the receiver's last FIN is lost, the
// sender doesn't know when the transmission is finished. We solve
// that by having the receiver operate on a background thread which
// can provide replacement FINs. (Note: in our case, a FIN is an ACK
// with all the bits set.)
//
typedef struct udpr_receiver udpr_receiver_t;
struct udpr_receiver
{
    udpr_receiver_params_t params;

    int fd;
    pthread_mutex_t mutex;
    pthread_cond_t cond;

    zarray_t *datas; // udpr_receiver_data_t*
};

// initializes parameters to reasonable defaults
void udpr_receiver_params_init(udpr_receiver_params_t *params);

udpr_receiver_t *udpr_receiver_create(udpr_receiver_params_t *params, int port);

void udpr_receiver_data_destroy(udpr_receiver_data_t *h);

// block until data is available. Caller should call
// udpr_receiver_data_destroy() on the result.
udpr_receiver_data_t *udpr_receiver_get(udpr_receiver_t *urr);

#endif
