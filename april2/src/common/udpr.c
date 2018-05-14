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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <pthread.h>

#include "common/net_util.h"
#include "common/encode_bytes.h"
#include "common/sha1.h"
#include "common/rand_util.h"
#include "common/zarray.h"
#include "common/zhash.h"
#include "common/time_util.h"
#include "common/zset.h"

#include "udpr.h"

// verbosity guide:
//   0: fatal errors only
//   1: packets and errors
//   2: fragments and warnings
//   3: anything

#define UDPR_MAGIC_SEND 0x5716eda52205779aULL
#define UDPR_MAGIC_ACK  0x58287687aba3857aULL

#define UDPR_REQUEST_ACK_FLAG 1

/*
  Basic approach:

  - The sender drives the process: they begin by sending all of fragments.

  - The receiver responds with an ACK containing a bit mask of which
  fragments have been received.

  - If the sender doesn't receive an ACK, it retransmits a fragment,
  which is sufficient to retrigger the receiver to send its ACK
  (with the received fragments bitmask)

  =====

  - The receiver implements a delayed ACK: it will not immediately send
  an ACK (except when the last fragment of a packet is received), but
  will wait for a short timeout to occur. This is to prevent an ACK
  storm.

*/

static void print_guid(const uint8_t *p)
{
    for (int i = 0; i < 16; i++)
        printf("%02x", p[i]);
}

///////////////////////////////////////////////////////////////////////
// UDPR_SEND
///////////////////////////////////////////////////////////////////////

// initializes parameters to reasonable defaults
void udpr_send_params_init(udpr_send_params_t *params)
{
    memset(params, 0, sizeof(udpr_send_params_t));
    params->fragment_size = 1024; // 2048;
    params->ack_timeout_ms = 200;
    params->drop_rate = 0;
    params->send_sleep_us = 1e3;
}

///////////////////////////////////////////////////////////////////////
// addr: to whom should we be sending? We will create an ephemeral UDP
// socket; we must be able to receive on that ephemeral port too.
//

// returns 0 on success

int udpr_send(udpr_send_params_t *params,
              const struct sockaddr_in *addr,
              const void *_data, const int datalen, const uint8_t guid[32])
{
    const uint8_t *data = (const uint8_t*) _data;

    int res = 0;

    int fd = udp_socket_create(NULL, 0);
    if (fd < 0)
        return -1; // only place where we can return and not goto cleanup

    // fragment status: if UINT32_MAX, the packet has been ack'd.
    // otherwise, it is the sequence number that we last transmitted
    // containing this fragment.

    // We should only retransmit a fragment if the last sequence
    // number we transmitted that fragment with is OLDER than the last
    // received sequence number. Initializing to zero will force a
    // retransmission.
    uint32_t nfragments = (datalen + params->fragment_size - 1) / params->fragment_size;
    if (nfragments == 0)
        nfragments = 1;

    uint32_t nfragments_received = 0;
    uint32_t *fragment_status = calloc(nfragments, sizeof(uint32_t));

    // count our outgoing packets
    uint32_t sequence_number = 1;

    // this is the receiver's echoed version of our sequence number
    uint32_t echoed_sequence_number = 0;

    // this is the maximum received receiver's sequence number
    uint32_t rx_sequence_number = 0;

    // which fragment will we try to send next?
    uint32_t next_frag_send_idx = 0;

    uint64_t last_ack_utime = 0;

    // keep sending until we know the receiver has received everything.
    while (nfragments_received < nfragments) {

        int request_ack = 1;

        // (re-)transmit everything that we believe the receiver needs.
        for (int fragcounter = 0; fragcounter < nfragments; fragcounter++) {

            next_frag_send_idx  = (next_frag_send_idx + 1) % nfragments;
            uint32_t fragidx = next_frag_send_idx;

//            printf("frag %d %08x %08x %08x\n", fragidx, fragment_status[fragidx], echoed_sequence_number, sequence_number);

            // we've re-transmitted this packet more recently than the
            // last ACK, don't retransmit again.

            // Note that this covers the case where status is UINT32_MAX
            if (fragment_status[fragidx] > echoed_sequence_number)
                continue;

            ////////////////////////////////////////////////////////////
            // Transmit DATA fragment
            uint8_t fragment[params->fragment_size + 64];

            if (params->verbosity >= 2) {
                printf("%15.3f ", utime_now() / 1.0E6);
                print_guid(guid);
                printf(" fragment %4d / %4d\n", fragidx, nfragments);
            }

            // packet header format
            // 0:  [ TX-magic sequence, 8 bytes]
            // 8:  [ sequence number of transmitter, 4 bytes]
            // 12: [ last seq number from receiver, 4 bytes]
            // 16: [ flags, 4 bytes]
            // 20: [ guid, 32 bytes ]
            // 52: [ total message length, 4 bytes]
            // 56: [ fragmentation size, 4 bytes ] (nominal fragment size)
            // 60: [ fragment index, 4 bytes]
            // 64: [ this fragment data... ]
            uint32_t outpos = 0;
            encode_u64(fragment, &outpos, UDPR_MAGIC_SEND);
            encode_u32(fragment, &outpos, sequence_number);
            fragment_status[fragidx] = sequence_number;
            sequence_number++;
//            printf("SEND SEQ %08x\n", sequence_number);

            encode_u32(fragment, &outpos, rx_sequence_number);
            encode_u32(fragment, &outpos, 0);
            encodeN(fragment, &outpos, guid, 32);
            encode_u32(fragment, &outpos, datalen);
            encode_u32(fragment, &outpos, params->fragment_size);
            encode_u32(fragment, &outpos, fragidx);

            int this_fragment_size = datalen - params->fragment_size * fragidx;
            if (this_fragment_size > params->fragment_size)
                this_fragment_size = params->fragment_size;

            encodeN(fragment, &outpos,
                    &data[params->fragment_size * fragidx],
                    this_fragment_size);

            ssize_t res = sendto(fd,
                                 fragment,
                                 outpos,
                                 0,
                                 (struct sockaddr*) addr,
                                 sizeof(struct sockaddr_in));

            if (params->send_sleep_us > 0)
                usleep(params->send_sleep_us);

            // If there's an ACK waiting for us, stop sending...
            if (1) {
                struct pollfd pollfds[1] = { { .fd = fd,
                                               .events = POLLIN,
                                               .revents = 0 } };

                int res = poll(pollfds, 1, 0);
                if (res == 1) {
                    request_ack = 0;
                    break;
                }
            }
        }

        // are we done transmitting?
        for (int fragidx = 0; fragidx < nfragments; fragidx++) {

            if (fragment_status[fragidx] > echoed_sequence_number)
                continue;
            request_ack = 0;
        }

        if ((utime_now() - last_ack_utime) / 1000 < params->ack_timeout_ms) {
            request_ack = 0;
        }

        // request an ACK, we've transmitted everything we expect to
        // transmit. We can't do anything sane until we hear back from
        // the receiver.
        if (request_ack) {
            // just like a DATA fragment, but without the data. Needs to include
            // enough information for the receiver to create a data buffer.

            // 0:  [ TX-magic sequence, 8 bytes]
            // 8:  [ sequence number of transmitter, 4 bytes]
            // 12: [ last seq number from receiver, 4 bytes]
            // 16: [ flags, 4 bytes]
            // 20: [ guid, 32 bytes ]
            // 52: [ total message length, 4 bytes]
            // 56: [ fragmentation size, 4 bytes ] (nominal fragment size)
            // 60: 0xffffffff

            if (params->verbosity >= 2) {
                printf("%15.3f ", utime_now() / 1.0E6);
                print_guid(guid);
                printf(" REQUEST ACK\n");
            }

            // Transmit REQUEST_ACK fragment
            uint8_t fragment[64];

            uint32_t outpos = 0;
            encode_u64(fragment, &outpos, UDPR_MAGIC_SEND);
            encode_u32(fragment, &outpos, sequence_number);
            sequence_number++;
            encode_u32(fragment, &outpos, rx_sequence_number);
            encode_u32(fragment, &outpos, UDPR_REQUEST_ACK_FLAG);
            encodeN(fragment, &outpos, guid, 32);
            encode_u32(fragment, &outpos, datalen);
            encode_u32(fragment, &outpos, params->fragment_size);
            encode_u32(fragment, &outpos, 0xffffffff);

            ssize_t res = sendto(fd,
                                 fragment,
                                 outpos,
                                 0,
                                 (struct sockaddr*) addr,
                                 sizeof(struct sockaddr_in));

            if (res < 0)
                perror("sendto");
        }

        // NOW: we have (always?) transmitted all of the
        // packets which we believe the receiver requires. Some, of course, may
        // be lost in transit. We want to find out what happened.

        // Keep reading incoming messages until it blocks (and times
        // out). We loop so that if there are multiple ACKs incoming,
        // we can flush them.

        // We only block the first time around the loop. In subsequent
        // loops, we do not block.
        int poll_timeout_ms = params->ack_timeout_ms;

        while (nfragments_received < nfragments) {
            struct pollfd pollfds[1] = { { .fd = fd,
                                           .events = POLLIN,
                                           .revents = 0 } };

            int res = poll(pollfds, 1, poll_timeout_ms);
            poll_timeout_ms = 0;

            // error?
            if (res < 0) {
                res = -1;
                goto cleanup;
            }

            // if we timeout. We will only an REQUEST_ACK.
            if (res == 0)
                break;

            if (res == 1) {

                // XXX check size.
                uint8_t rx_data[65536];
                struct sockaddr rx_src_addr;
                socklen_t rx_src_addrlen = sizeof(rx_src_addr);

                ssize_t rx_len = recvfrom(fd, rx_data, sizeof(rx_data), 0,
                                          &rx_src_addr, &rx_src_addrlen);

                if (rx_len > 0 && randf_uniform(0, 1) < params->drop_rate) {
                    if (params->verbosity >= 3) {
                        printf("%15.3f DROP\n", utime_now() / 1.0E6);
                        usleep(params->ack_timeout_ms);
                    }
                    continue;
                }

                // ACK packet format
                // 0:  [ ACK-magic sequence, 8 bytes]
                // 8:  [ sequence number of transmitter, 4 bytes]
                // 12: [ last seq number from receiver, 4 bytes]
                // 16: [ flags, 4 bytes]
                // 20: [ guid, 32 bytes]
                // 52: [ total message length, 4 bytes]
                // 56: [ fragmentation size, 4 bytes ] (nominal fragment size)
                // 60: [ rx flags, 1 bit per fragment]
                if (rx_len < 0) {
                    printf("err\n");
                    continue;
                }

                int rx_len_check = 60 + (nfragments + 7) / 8;
                if (rx_len != rx_len_check) {
                    printf("bad rx len: %d %d\n", (int) rx_len, rx_len_check);
                    continue;
                }

                uint32_t rx_pos = 0;
                uint64_t rx_magic = decode_u64(rx_data, &rx_pos, rx_len);
                if (rx_magic != UDPR_MAGIC_ACK) {
                    printf("bad checksum\n");
                    continue;
                }

                uint32_t this_rx_sequence_number = decode_u32(rx_data, &rx_pos, rx_len);
                if (this_rx_sequence_number <= rx_sequence_number) {
                    printf("old ACK\n");
                    continue;
                }
                rx_sequence_number = this_rx_sequence_number;

                echoed_sequence_number = decode_u32(rx_data, &rx_pos, rx_len);

                uint32_t rx_flags = decode_u32(rx_data, &rx_pos, rx_len);

                // XXX TODO check that receiver was the desired receiver

                uint8_t rx_guid[32];
                decodeN(rx_data, &rx_pos, rx_len, rx_guid, 32);
                if (memcmp(rx_guid, guid, 32)) {
                    printf("bad rx guid\n");
                    continue;
                }

                uint32_t rx_datalen = decode_u32(rx_data, &rx_pos, rx_len);
                uint32_t rx_fragment_size = decode_u32(rx_data, &rx_pos, rx_len);

                if (rx_datalen != datalen || rx_fragment_size != params->fragment_size) {
                    printf("bad rx contents\n");
                    continue;
                }

                // recompute by going over the ACK
                nfragments_received = 0;

                for (int fragidx = 0; fragidx < nfragments; fragidx++) {
                    int byteidx = fragidx / 8;
                    int bitidx = fragidx & 7;

                    int v = (rx_data[rx_pos + byteidx] >> bitidx) & 1;

                    // did we previously have this fragment marked as
                    // received, but now the sender is saying they don't have
                    // it. This can happen if the receiver restarts, but shouldn't
                    // happen otherwise.
                    if (fragment_status[fragidx] == UINT32_MAX && !v) {
                        fragment_status[fragidx] = 0;

                        if (params->verbosity >= 3)
                            printf("unreceived flag\n");
                    }

                    if (v) {
                        nfragments_received++;
                        fragment_status[fragidx] = UINT32_MAX;
                    }
                }

                last_ack_utime = utime_now();

                if (params->verbosity >= 2) {
                    printf("%15.3f ", utime_now() / 1.0E6);
                    print_guid(rx_guid);
                    printf(" RX ACK   %4d / %4d\n", nfragments_received, nfragments);
                }
            }
        }
    }

    if (params->verbosity >= 1) {
        printf("%15.3f ", utime_now() / 1.0E6);
        print_guid(guid);
        printf(" SENT %4d / %4d\n", sequence_number, nfragments);
    }

  cleanup:
    free(fragment_status);
    close(fd);
    return res;
}

///////////////////////////////////////////////////////////////////////
// UDPR_RECEIVER
///////////////////////////////////////////////////////////////////////
void udpr_receiver_params_init(udpr_receiver_params_t *params)
{
    memset(params, 0, sizeof(udpr_receiver_params_t));
    params->expire_incomplete_ms = 90E3;
    params->expire_complete_ms = 5E3;
    params->drop_rate = 0;
    params->send_sleep_us = 0;
    params->unacked_fragments_max = 10;
    params->poll_timeout_ms = 5;

}

void udpr_receiver_data_destroy(udpr_receiver_data_t *h)
{
    free(h->data);
    free(h->received);
    free(h);
}

static udpr_receiver_data_t *udpr_receiver_data_copy(udpr_receiver_data_t *in)
{
    udpr_receiver_data_t *out = calloc(1, sizeof(udpr_receiver_data_t));
    out->utime = in->utime;
    memcpy(out->guid, in->guid, 32);
    out->datalen = in->datalen;

    out->data = malloc(out->datalen);
    memcpy(out->data, in->data, in->datalen);

    memcpy(&out->sender_addr, &in->sender_addr, sizeof(struct sockaddr_in));
    out->fragment_size = in->fragment_size;
    out->nfragments = in->nfragments;
    out->nfragments_received = in->nfragments_received;

    out->received = malloc(out->nfragments);
    memcpy(out->received, in->received, in->nfragments);

    return out;
}

static uint32_t guid_hash(const void *_a)
{
    const uint8_t *a = ((const uint8_t*) _a);
    return (a[0] << 24) + (a[1] << 16) + (a[2] << 8) + (a[3]);
}

static int guid_equals(const void *_a, const void *_b)
{
    const uint8_t *a = ((const uint8_t*) _a);
    const uint8_t *b = ((const uint8_t*) _b);

    return !memcmp(a, b, 32);
}

static void udpr_receiver_send_ack(udpr_receiver_t *urr,
                                   udpr_receiver_data_t *h)
{
    uint8_t ack[65536];
    uint32_t ack_pos = 0;
    encode_u64(ack, &ack_pos, UDPR_MAGIC_ACK);
    encode_u32(ack, &ack_pos, h->sequence_number);
    h->sequence_number++;
    encode_u32(ack, &ack_pos, h->last_sequence_number);
    encode_u32(ack, &ack_pos, 0); // flags
    encodeN(ack, &ack_pos, h->guid, 32);
    encode_u32(ack, &ack_pos, h->datalen);
    encode_u32(ack, &ack_pos, h->fragment_size);

    uint8_t b[(h->nfragments+7)/8];
    memset(b, 0, sizeof(b));

    for (int fragidx = 0; fragidx < h->nfragments; fragidx++) {
        int byteidx = fragidx / 8;
        int bitidx = fragidx & 7;

        if (h->received[fragidx])
            b[byteidx] |= (1 << bitidx);
    }

    encodeN(ack, &ack_pos, b, sizeof(b));

    ssize_t res = sendto(urr->fd, ack, ack_pos, 0,
                         (struct sockaddr*) &h->sender_addr,
                         sizeof(struct sockaddr_in));
    if (res < 0)
        perror("sendto");

    if (urr->params.send_sleep_us > 0)
        usleep(urr->params.send_sleep_us);

    if (urr->params.verbosity >= 2) {
        printf("%15.3f ", utime_now() / 1.0E6);
        print_guid(h->guid);
        printf(" Send ACK %d / %d\n", h->nfragments_received, h->nfragments);
    }

    h->unacked_fragments = 0;
}

static void *udpr_receiver_thread(void *_user)
{
    udpr_receiver_t *urr = (udpr_receiver_t*) _user;

    // hash stores the packets currently being received
    // maps GUID -> data_t*
    zhash_t *hash = zhash_create(32, sizeof(udpr_receiver_data_t*),
                                 guid_hash, guid_equals);


    uint64_t next_check_utime = 0;

    while (1) {
        uint64_t now = utime_now();

        // cleanup old udpr_receiver_data
        if (now > next_check_utime) {

            zhash_iterator_t zit;
            zhash_iterator_init(hash, &zit);

            uint8_t iguid[32];
            udpr_receiver_data_t *ih;

            while (zhash_iterator_next(&zit, iguid, &ih)) {

                // should we send an ack?
                if (ih->nfragments_received < ih->nfragments &&
                    ih->unacked_fragments > urr->params.unacked_fragments_max) {

                    udpr_receiver_send_ack(urr, ih);
                }

                // should we garbage collect this record?
                uint64_t dt_ms = (now - ih->utime) / 1000;

                int gc = 0;
                if (ih->nfragments_received == ih->nfragments &&
                    dt_ms > urr->params.expire_complete_ms)
                    gc = 1;

                if (ih->nfragments_received < ih->nfragments &&
                    dt_ms > urr->params.expire_incomplete_ms)
                    gc = 1;

                if (gc) {

                    zhash_iterator_remove(&zit);

                    printf("%15.3f ", utime_now() / 1.0E6);
                    print_guid(ih->guid);
                    printf(" expire %s\n", ih->nfragments == ih->nfragments_received ? "complete" : "INCOMPLETE");

                    udpr_receiver_data_destroy(ih);
                }
            }

            next_check_utime = utime_now() + 1E3;
        }

        struct pollfd pollfds[1] = { { .fd = urr->fd,
                                       .events = POLLIN,
                                       .revents = 0 } };

        int res = poll(pollfds, 1, urr->params.poll_timeout_ms);
        if (res == 0)
            continue;

        // XXX check size.
        uint8_t fragment[65536];
        struct sockaddr this_sender_addr;
        socklen_t this_sender_addr_len = sizeof(this_sender_addr);

        ssize_t fragment_len = recvfrom(urr->fd, fragment, sizeof(fragment), 0,
                                        &this_sender_addr, &this_sender_addr_len);

        if (fragment_len > 0 && randf_uniform(0, 1) < urr->params.drop_rate) {
            if (urr->params.verbosity >= 3) {
                printf("%15.3f DROP\n", utime_now() / 1.0E6);
                usleep(urr->params.poll_timeout_ms);
            }

            continue;
        }

        uint32_t fragment_pos = 0;
        uint64_t magic = decode_u64(fragment, &fragment_pos, fragment_len);
        if (magic != UDPR_MAGIC_SEND) {
            printf("bad magic %"PRIx64"\n", magic);
            continue;
        }

        uint32_t this_sequence_number = decode_u32(fragment, &fragment_pos, fragment_len);
        uint32_t this_echoed_sequence_number = decode_u32(fragment, &fragment_pos, fragment_len);
        uint32_t this_flags = decode_u32(fragment, &fragment_pos, fragment_len);
        uint8_t  this_guid[32];
        decodeN(fragment, &fragment_pos, fragment_len, this_guid, 32);
        uint32_t this_datalen = decode_u32(fragment, &fragment_pos, fragment_len);
        uint32_t this_fragment_size = decode_u32(fragment, &fragment_pos, fragment_len);
        uint32_t this_fragment_idx = decode_u32(fragment, &fragment_pos, fragment_len);

        udpr_receiver_data_t *h = NULL;

        if (!zhash_get(hash, &this_guid, &h)) {
            // create a new data object
            h = calloc(1, sizeof(udpr_receiver_data_t));
            h->utime = utime_now();
            memcpy(h->guid, this_guid, 32);
            h->data = calloc(1, this_datalen);
            h->datalen = this_datalen;
            h->fragment_size = this_fragment_size;
            h->nfragments = (h->datalen + h->fragment_size - 1) / h->fragment_size;
            if (h->nfragments == 0)
                h->nfragments = 1;
            h->nfragments_received = 0;
            h->received = calloc(1, h->nfragments);
            memcpy(&h->sender_addr, &this_sender_addr, sizeof(struct sockaddr_in));
            h->sequence_number = 1;

            if (zhash_put(hash, h->guid, &h, NULL, NULL)) {
                assert(0);
            }

        } else {
            // verify compatibility with our existing data object
            if (h->datalen != this_datalen ||
                h->fragment_size != this_fragment_size ||
                memcmp(h->guid, this_guid, 32) ||
                memcmp(&h->sender_addr, &this_sender_addr, sizeof(struct sockaddr_in))) {
                printf("mismatched sender data. XXX handle this better.\n");
                // if this sender is legit, it's probably going to get stuck now
                continue;
            }
        }

        h->last_sequence_number = this_sequence_number;
        h->last_echoed_sequence_number = this_echoed_sequence_number;

        int send_ack = 0;

        // Is this a data fragment?
        if (this_fragment_idx != UINT32_MAX) {
            // okay, looks good. let's receive it.
            uint32_t this_fragment_offset = this_fragment_size * this_fragment_idx;
            int this_fragment_len = this_datalen - this_fragment_offset;
            if (this_fragment_len > this_fragment_size)
                this_fragment_len = this_fragment_size;

            if (this_fragment_offset + this_fragment_len > this_datalen) {
                printf("data fragment too big\n");
                continue;
            }

            if (h->data)
                decodeN(fragment, &fragment_pos, fragment_len,
                        &((uint8_t*) h->data)[this_fragment_offset], this_fragment_len);

            if (urr->params.verbosity >= 2) {
                printf("%15.3f ", utime_now() / 1.0E6);
                print_guid(h->guid);
                printf(" fragment %s %4d / %4d %s\n",
                       h->received[this_fragment_idx] ? "OLD" : "new",
                       h->nfragments_received, h->nfragments,
                       this_flags & UDPR_REQUEST_ACK_FLAG ? "REQUEST_ACK": "");
            }

            // is this a fragment that we had not previously received?
            if (!h->received[this_fragment_idx]) {
                h->received[this_fragment_idx] = 1;
                h->nfragments_received++;
                h->unacked_fragments++;

                if (h->nfragments_received == h->nfragments) {
                    send_ack = 1;

                    udpr_receiver_data_t *copy = udpr_receiver_data_copy(h);
                    pthread_mutex_lock(&urr->mutex);
                    zarray_add(urr->datas, &copy);
                    pthread_cond_broadcast(&urr->cond);
                    pthread_mutex_unlock(&urr->mutex);

                    // we're done with the data, free it now.
                    free(h->data);
                    h->data = NULL;

                    if (urr->params.verbosity >= 1) {
                        printf("%15.3f ", utime_now() / 1.0E6);
                        print_guid(h->guid);
                        printf(" RECV\n");
                    }
                }
            }
        }

        if (this_flags & UDPR_REQUEST_ACK_FLAG) {
            send_ack = 1;
            printf("%15.3f ", utime_now() / 1.0E6);
            print_guid(h->guid);
            printf(" received REQUEST_ACK\n");
        }

        if (send_ack) {
            udpr_receiver_send_ack(urr, h);
        }

    }

    return NULL;
}

udpr_receiver_t *udpr_receiver_create(udpr_receiver_params_t *params, int port)
{
    int fd = udp_socket_listen(port);
    if (fd < 0)
        return NULL;

    udpr_receiver_t *urr = calloc(1, sizeof(udpr_receiver_t));
    urr->params = *params;
    urr->fd = fd;
    urr->datas = zarray_create(sizeof(udpr_receiver_data_t*));

    pthread_mutex_init(&urr->mutex, NULL);
    pthread_cond_init(&urr->cond, NULL);
    pthread_t t;
    pthread_create(&t, NULL, udpr_receiver_thread, urr);

    return urr;
}

// block until data is available. Caller should call
// udpr_receiver_data_destroy() on the result.
udpr_receiver_data_t *udpr_receiver_get(udpr_receiver_t *urr)
{
    pthread_mutex_lock(&urr->mutex);

    while (zarray_size(urr->datas) == 0)
        pthread_cond_wait(&urr->cond, &urr->mutex);

    udpr_receiver_data_t *data;
    zarray_get(urr->datas, 0, &data);
    zarray_remove_index(urr->datas, 0, 0);

    pthread_mutex_unlock(&urr->mutex);

    return data;
}
