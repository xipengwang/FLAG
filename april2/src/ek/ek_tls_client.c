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
#include <assert.h>
#include <time.h>
#include <string.h>
#include <stdio.h>

#include "common/zarray.h"
#include "ek/ek_tls.h"
#include "ek/ek_asn.h"

#define TIMEOUT_MS 10000

enum { STATE_INITIAL = 1, STATE_SERVER_DONE, STATE_APPLICATION_DATA };

static void buffer_append(uint8_t **_buf, int *bufpos, const void *s, int slen)
{
    uint8_t *buf = *_buf;

    buf = realloc(buf, *bufpos + slen);
    *_buf = buf;

    memcpy(&buf[*bufpos], s, slen);
    (*bufpos) += slen;
}

int client_conn_on_record(ek_tls_conn_t *conn)
{
    int ret = 0;
    ek_tls_client_t *client = conn->client;

    //////////////////////////////////////////////////
    // handle the packet
    // XXX length validation
    uint8_t *record = conn->record;
    uint8_t record_type = record[0];
    uint8_t record_version_major = record[1];
    uint8_t record_version_minor = record[2];
    uint16_t record_len = (record[3] << 8) + record[4] + 5;
    int record_pos = 5;

    if (record_version_major != conn->negotiated_version_major ||
        record_version_minor < conn->negotiated_version_minor)
        goto teardown;

    switch (record_type) {
        case 0x16: {
            // Handshake

            while (record_pos < record_len) {

                // XXX length validation
                uint8_t *handshake = &record[record_pos];
                uint8_t handshake_type = handshake[0];
                uint32_t handshake_len = 4 +
                    (handshake[1] << 16) +
                    (handshake[2] << 8) +
                    handshake[3];
                int handshake_pos = 4;

                switch (handshake_type) {

                    case 2: { // ServerHello
                        EK_TLSDBG("RX ServerHello");

                        buffer_append(&conn->headers, &conn->headers_len, handshake, handshake_len);

                        int requested_version_major = handshake[handshake_pos++];
                        int requested_version_minor = handshake[handshake_pos++];

                        if (requested_version_major != 3 || requested_version_minor != 3) {
                            EK_TLSDBG("bad TLS version\n");
                            return -1;
                        }

                        conn->negotiated_version_major = requested_version_major;
                        conn->negotiated_version_minor = requested_version_minor;

                        conn->server_random_len = 32;
                        conn->server_random = malloc(conn->server_random_len);
                        memcpy(conn->server_random, &handshake[handshake_pos], conn->server_random_len);
                        handshake_pos += conn->server_random_len;

                        int session_id_len = handshake[handshake_pos++];
                        // ignore it. (XXX, untested?)
                        handshake_pos += session_id_len;

                        int ciphersuite_id = (handshake[handshake_pos] << 8) + (handshake[handshake_pos + 1]);
                        handshake_pos += 2;

                        conn->pending_ciphersuite = NULL;

                        for (int j = 0; ek_tls_ciphersuites[j].name != NULL; j++) {
                            if (ek_tls_ciphersuites[j].id == ciphersuite_id) {
                                if (!ek_tls_ciphersuites[j].preference) {
                                    EK_TLSDBG("Server returned unpreferred ciphersuite %04x\n", ciphersuite_id);
                                    return -1;
                                }
                                conn->pending_ciphersuite = &ek_tls_ciphersuites[j];
                                EK_TLSDBG("negotiated cipher suite %s", conn->pending_ciphersuite->name);
                            }
                        }

                        if (conn->pending_ciphersuite == NULL) {
                            EK_TLSDBG("Failure to get pending ciphersuite");
                            return -1;
                        }

                        int compression_method = handshake[handshake_pos++];
                        if (compression_method != 0) {
                            EK_TLSDBG("Server returned unpreferred compression method\n");
                            return -1;
                        }

                        break;
                    }

                    case 11: { // Certificate
                        EK_TLSDBG("RX Certificate");

                        buffer_append(&conn->headers, &conn->headers_len, handshake, handshake_len);

                        int cert_list_len = (handshake[handshake_pos + 0] << 16) +
                            (handshake[handshake_pos + 1] << 8) +
                            (handshake[handshake_pos + 2] << 0);
                        handshake_pos += 3;
                        int cert_list_end = handshake_pos + cert_list_len;

                        if (cert_list_end != handshake_len) {
                            EK_TLSDBG("Bad total certificate length %d %d\n", cert_list_end, handshake_len);
                            return -1;
                        }

                        while (handshake_pos < cert_list_end) {
                            int this_cert_len = (handshake[handshake_pos + 0] << 16) +
                                (handshake[handshake_pos + 1] << 8) +
                                (handshake[handshake_pos + 2] << 0);
                            handshake_pos += 3;

                            int this_cert_end = handshake_pos + this_cert_len;

                            if (this_cert_end <= handshake_len) {

                                int err;
                                ek_asn_result_t *cert = ek_asn_result_create_from_der(&handshake[handshake_pos],
                                                                                this_cert_len,
                                                                                ek_asn_type_create_x509_cacert(),
                                                                                &err);

                                if (!err) {
                                    if (!conn->server_certificates)
                                        conn->server_certificates = zarray_create(sizeof(ek_asn_result_t*));
                                    zarray_add(conn->server_certificates, &cert);
                                } else {
                                    EK_TLSDBG("failed to parse server certificate");
                                    return -1;
                                }

                                handshake_pos += this_cert_len;


                            } else {
                                EK_TLSDBG("Bad certificate length %d", this_cert_len);
                                return -1;
                            }
                        }

                        if (handshake_pos != handshake_len) {
                            EK_TLSDBG("Certificate read error");
                            return -1;
                        }

                        break;
                    }

                        // ServerKeyExchange*

                        // CertificateRequest*

                    case 14: { // ServerDone
                        EK_TLSDBG("RX ServerDone");

                        buffer_append(&conn->headers, &conn->headers_len, handshake, handshake_len);

                        // nothing in this message!
                        conn->state = STATE_SERVER_DONE;
                        break;
                    }

                    case 20: {
                        EK_TLSDBG("RX Finished");

                        ek_hash_algorithm_t *alg = &conn->cipherparams->finished_hash_algorithm;
                        uint8_t digest[alg->digest_size];
                        ek_hash_state_t state;
                        alg->init(&state);
                        alg->update(&state, conn->headers, conn->headers_len);
                        alg->final(&state, digest);

                        uint8_t vd[12];
                        conn->cipherparams->prf_algorithm.prf(&conn->cipherparams->prf_algorithm,
                                                              conn->cipherparams->master_secret,
                                                              conn->cipherparams->master_secret_len,
                                                              "server finished",
                                                              digest, alg->digest_size, vd, sizeof(vd));

                        if (memcmp(vd, &handshake[handshake_pos], 12)) {
                            EK_TLSDBG("Bad verify");
                            goto teardown;
                        }

                        conn->state = STATE_APPLICATION_DATA;
                        break;
                    }

                    case 12: {
                        EK_TLSDBG("RX ServerKeyExchange -- not implemented");
                        goto teardown;
                        break;
                    }

                    case 13: {
                        EK_TLSDBG("RX CertificateRequest");

                        buffer_append(&conn->headers, &conn->headers_len, handshake, handshake_len);

                        ////////////////
                        int types_len = handshake[handshake_pos++];
                        int types_end = handshake_pos + types_len;
                        while (handshake_pos < types_end && handshake_pos < handshake_len) {
//                            printf("type: %d\n", handshake[handshake_pos++]);
                            handshake_pos++;
                        }


                        ////////////////
                        int sighashes_len = (handshake[handshake_pos + 0] << 8) +
                            (handshake[handshake_pos + 1]);
                        handshake_pos += 2;
                        int sighashes_end = handshake_pos + sighashes_len;

                        while (handshake_pos < sighashes_end && handshake_pos < handshake_len) {
                            int hashalg = handshake[handshake_pos++];
                            int sigalg = handshake[handshake_pos++];

//                            printf("sig %d, hash %d\n", sigalg, hashalg);
                        }

                        ////////////////
                        int authorities_len = (handshake[handshake_pos + 0] << 8) +
                            (handshake[handshake_pos + 1]);
                        handshake_pos += 2;
                        int authorities_end = handshake_pos + authorities_len;

                        while (handshake_pos < authorities_len && handshake_pos < handshake_len) {
                            int this_len = (handshake[handshake_pos + 0] << 8) +
                                (handshake[handshake_pos + 1]);
                            handshake_pos += 2;
                            int this_end = handshake_pos + this_len;

                            printf("authority: (not printing)\n");
                            while (handshake_pos < this_end && handshake_pos < handshake_len)
                                handshake_pos++;
//                            printf("%c", handshake[handshake_pos++]);
                        }

                        conn->cert_requested = 1;

                        // XXX maybe we should pick which certificate we
                        // will respond with here. But e can't send it
                        // since it will corrupt the hashes.
                        break;
                    }

                    default: {
                        EK_TLSDBG("RX Unimplemented handshake message %d", handshake_type);

                        break;
                    }
                }

                record_pos += handshake_len;
            }
            break;
        }

        case 0x14: {  // ChangeCipherSpec
            EK_TLSDBG("RX ChangeCipherSpec");
            conn->received_change_cipher_spec = 1;
            break;
        }

        case 0x17: {
            EK_TLSDBG("RX Application Data (%d)", record_len - record_pos);
            if (conn->state != STATE_APPLICATION_DATA)
                goto teardown;

            conn->app_data = &conn->record[record_pos];
            conn->app_data_len = record_len - record_pos;
            conn->app_data_pos = 0;
            ret = 0;

            break;
        }

        case 0x15: {
            EK_TLSDBG("TLS Alert");
            goto teardown;
        }

        default: {
            EK_TLSDBG("RX Unimplemented record");
            ek_print_buffer(conn->record, conn->record_len);
            goto teardown;
        }

    }

    return ret;

  teardown: ;
    ret = -1;
    return ret;

}

void client_conn_destroy(ek_tls_conn_t *conn)
{
    if (!conn)
        return;

    conn->tcp_stream->destroy(conn->tcp_stream);
    free(conn->tls_stream);

    free(conn->rx_buffer);

    // XXX secure free?
    free(conn->server_random);
    free(conn->client_random);

    free(conn->headers);

    ek_tls_cipherparams_destroy(conn->cipherparams);
    ek_tls_cipherparams_destroy(conn->pending_cipherparams);

    free(conn);
}

ek_tls_client_t *ek_tls_client_create()
{
    ek_tls_client_t *client = calloc(1, sizeof(ek_tls_client_t));
    return client;
}


ek_tls_conn_t *ek_tls_client_conn_create(ek_tls_client_t *client, estream_t *tcp_stream)
{
    ek_tls_conn_t *conn = calloc(1, sizeof(ek_tls_conn_t));

    conn->type = EK_TLS_CONN_CLIENT;
    conn->client = client;
    conn->tcp_stream = tcp_stream;

    conn->rx_alloc = 16384 + 2048 + 5;
    conn->rx_buffer = malloc(conn->rx_alloc);
    conn->rx_len = 0;

    conn->tls_stream = ek_tls_stream_create(conn);

    conn->on_record = client_conn_on_record;
    conn->destroy = client_conn_destroy;

    conn->state = STATE_INITIAL;

    conn->negotiated_version_major = 3; // ???
    conn->negotiated_version_minor = 1;

    conn->client_random_len = 32;
    conn->client_random = ek_random_buffer_create(conn->client_random_len);
    if (1) {
        // adjust first four bytes
        int32_t t = time(NULL);
        conn->client_random[0] = (t >> 24) % 256;
        conn->client_random[1] = (t >> 16) % 256;
        conn->client_random[2] = (t >> 8) % 256;
        conn->client_random[3] = (t >> 0) % 256;
    }

    // send CLIENT_HELLO
    if (1) {
        uint8_t response[32768];
        int response_pos = 0;

        response[response_pos++] = 1; // ClientHello
        response[response_pos++] = 0; // len (will be filled in)
        response[response_pos++] = 0;
        response[response_pos++] = 0;

        // client version
        response[response_pos++] = 3; // suggest TLS 1.2
        response[response_pos++] = 3; // suggest TLS 1.2

        // client random
        memcpy(&response[response_pos], conn->client_random, conn->client_random_len);
        response_pos += conn->client_random_len;

        // no session resume token
        response[response_pos++] = 0;

        // cipher suites
        int ncsuites = 0;
        for (int j = 0; ek_tls_ciphersuites[j].name != NULL; j++) {
            if (ek_tls_ciphersuites[j].preference)
                ncsuites++;
        }
        response[response_pos++] = (2*ncsuites) >> 8;
        response[response_pos++] = (2*ncsuites) & 255;

        for (int j = 0; ek_tls_ciphersuites[j].name != NULL; j++) {
            if (ek_tls_ciphersuites[j].preference) {
                response[response_pos++] = ek_tls_ciphersuites[j].id >> 8;
                response[response_pos++] = ek_tls_ciphersuites[j].id & 255;
            }
        }

        // compression
        response[response_pos++] = 1; // one compression method
        response[response_pos++] = 0; // ... NULL

        // extensions length
//        response[response_pos++] = 0;
//        response[response_pos++] = 0;

        // compute length
        response[1] = ((response_pos - 4) >> 16) & 0xff;
        response[2] = ((response_pos - 4) >> 8) & 0xff;
        response[3] = ((response_pos - 4) >> 0) & 0xff;

        // write a handshake record
        buffer_append(&conn->headers, &conn->headers_len, response, response_pos);

        ek_tls_conn_write_record(conn, 0x16, response, response_pos);
        EK_TLSDBG("TX ClientHello");
    }

    int timeout_ms = 5000;

    while (conn->state != STATE_SERVER_DONE) {
        if (ek_tls_conn_read_record(conn, timeout_ms))
            goto teardown;

        conn->on_record(conn);
    }

    // XXX Make sure that this certificate satisfies the request
    // XXX Don't send certificate if not solicited.
    if (conn->cert_requested && client->ident) {
        EK_TLSDBG("TX Certificate");

        uint8_t response[32768];
        int response_pos = 0;

        response[response_pos++] = 11; // Certificate
        response[response_pos++] = 0;  // length (will be filled in)
        response[response_pos++] = 0;
        response[response_pos++] = 0;

        int sz = client->ident->cert->datalen;

        // certificate LIST length; we have only one cert
        response[response_pos++] = ((sz+3) >> 16) % 256;
        response[response_pos++] = ((sz+3) >> 8) % 256;
        response[response_pos++] = ((sz+3) >> 0) % 256;

        // send the certificate (its length followed by data)
        response[response_pos++] = (sz >> 16) % 256;
        response[response_pos++] = (sz >> 8) % 256;
        response[response_pos++] = (sz >> 0) % 256;

        memcpy(&response[response_pos], client->ident->cert->data, client->ident->cert->datalen);
        response_pos += client->ident->cert->datalen;

        // compute length
        response[1] = ((response_pos - 4) >> 16) & 0xff;
        response[2] = ((response_pos - 4) >> 8) & 0xff;
        response[3] = ((response_pos - 4) >> 0) & 0xff;

        // write a handshake record
        buffer_append(&conn->headers, &conn->headers_len, response, response_pos);
        ek_tls_conn_write_record(conn, 0x16, response, response_pos);
    }

    EK_TLSDBG("SERVER DONE READ LOOP EXITING");

    // generate premaster secret
    int premaster_secret_len = 48;
    uint8_t *premaster_secret = ek_random_buffer_create(premaster_secret_len);
    premaster_secret[0] = 3; // first two bytes should be version.
    premaster_secret[1] = 3; // should be latest version offered by client.

    // send ClientKeyExchange
    if (1) {
        EK_TLSDBG("TX ClientKeyExchange");

        uint8_t response[32768];
        int response_pos = 0;

        response[response_pos++] = 16; // ClientKeyExchange
        response[response_pos++] = 0; // len (will be filled in)
        response[response_pos++] = 0;
        response[response_pos++] = 0;

        // get the public key of the server
        if (!conn->server_certificates) {
            EK_TLSDBG("Cannot complete ClientKeyExchange due to missing server certificate");
            goto teardown;
        }

        ek_asn_result_t *cert;
        zarray_get(conn->server_certificates, 0, &cert);

        struct ek_asn_element *el;
        if (ek_asn_result_find(cert, "certificate[0].tbsCertificate.subjectPublicKeyInfo.subjectPublicKey", &el)) {
            EK_TLSDBG("Failed to get subjectPublicKey");
            goto teardown;
        }

        // subjectPublicKey is a BIT STRING type; the first byte
        // is the number of bits prepended to the string. We only
        // handle the zero case.
        if (el->payload_len < 1 || el->payload[0] != 0) {
            EK_TLSDBG("invalid padding byte in certificate subjectPublicKey");
            ek_asn_result_destroy(cert);
            goto teardown;
        }

        // point to the actual octet data...
        int err;
        ek_asn_result_t *pubkey = ek_asn_result_create_from_der(el->payload + 1, el->payload_len - 1,
                                                          ek_asn_type_create_x509_rsapublic(),
                                                          &err);

        if (err) {
            EK_TLSDBG("error retrieving public key from server certificate");
            goto teardown;
        }

        struct ek_asn_element *modulus_el, *pubexp_el;
        if (ek_asn_result_find(pubkey, "pubkey.modulus", &modulus_el) ||
            ek_asn_result_find(pubkey, "pubkey.publicExponent", &pubexp_el)) {
            EK_TLSDBG("error retrieving public key from server certificate");
            goto teardown;
        }

        ek_bigint_t *modulus = ek_bigint_create_from_bytes(modulus_el->payload, modulus_el->payload_len);
        ek_bigint_t *pubexp = ek_bigint_create_from_bytes(pubexp_el->payload, pubexp_el->payload_len);

        // no padding bytes can be zero!
        int padding_len = 512; // enough for 4096 bit RSA
        uint8_t *padding = calloc(1, padding_len);
        for (int i = 0; i < padding_len; i++)
            while (!padding[i])
                ek_random_buffer(&padding[i], 1);

        uint8_t *out = calloc(1, 512);
        int out_len;
        if (ek_rsa_pkcs_15_encrypt(modulus, pubexp, padding, premaster_secret, premaster_secret_len, out, &out_len)) {
            EK_TLSDBG("rsa encryption failed");
            goto teardown;
        }

        response[response_pos++] = out_len >> 8;
        response[response_pos++] = out_len & 0xff;
        memcpy(&response[response_pos], out, out_len);
        response_pos += out_len;

        // compute length
        response[1] = ((response_pos - 4) >> 16) & 0xff;
        response[2] = ((response_pos - 4) >> 8) & 0xff;
        response[3] = ((response_pos - 4) >> 0) & 0xff;

        buffer_append(&conn->headers, &conn->headers_len, response, response_pos);
        ek_tls_conn_write_record(conn, 0x16, response, response_pos);
    }

        // send CertificateVerify*
    if (conn->cert_requested && client->ident) {

        EK_TLSDBG("TX CertificateVerify");

        uint8_t response[32768];
        int response_pos = 0;

        response[response_pos++] = 15; // CertificateVerify
        response[response_pos++] = 0; // len (will be filled in)
        response[response_pos++] = 0;
        response[response_pos++] = 0;

        response[response_pos++] = 2; // hashalgorithm
        response[response_pos++] = 1; // signature algorithm (rsa)

        uint8_t sig[4096];
        int siglen;

        ek_rsa_ssa_pkcs_15_sign_sha1(client->ident->rsa.modulus, client->ident->rsa.privexp,
                                     conn->headers, conn->headers_len, sig, &siglen);

        response[response_pos++] = (siglen >> 8) & 0xff;
        response[response_pos++] = (siglen >> 0) & 0xff;
        memcpy(&response[response_pos], sig, siglen);
        response_pos += siglen;

//        int structlenoff = response_pos;
//        response[response_pos++] = 0; // length of struct (2 + siglen)
//        response[response_pos++] = 0;


//        response[structlenoff + 0] = ((siglen + 4) >> 8) & 0xff;
//        response[structlenoff + 1] = ((siglen + 4) >> 0) & 0xff;

        // compute length
        response[1] = ((response_pos - 4) >> 16) & 0xff;
        response[2] = ((response_pos - 4) >> 8) & 0xff;
        response[3] = ((response_pos - 4) >> 0) & 0xff;

        buffer_append(&conn->headers, &conn->headers_len, response, response_pos);
        ek_tls_conn_write_record(conn, 0x16, response, response_pos);
    }

    // send ChangeCipherSpec
    if (1) {
        EK_TLSDBG("TX ChangeCipherSpec");

        uint8_t response[32768];
        int response_pos = 0;

        response[response_pos++] = 1;

        ek_tls_conn_write_record(conn, 0x14, response, response_pos);

        // XXX free old suite
        conn->cipherparams = ek_tls_create_ciphersuite_parameters(conn->pending_ciphersuite,
                                                                  conn->client_random, conn->client_random_len,
                                                                  conn->server_random, conn->server_random_len,
                                                                  premaster_secret, premaster_secret_len);
        conn->ciphersuite = conn->pending_ciphersuite;
        conn->sent_change_cipher_spec = 1;
    }

    // send Finished
    if (1) {
        EK_TLSDBG("TX Finished");

        uint8_t response[32768];
        int response_pos = 0;

        response[response_pos++] = 20; // Finished
        response[response_pos++] = 0; // len (will be filled in)
        response[response_pos++] = 0;
        response[response_pos++] = 0;

        if (1) {
            ek_hash_algorithm_t *alg = &conn->cipherparams->finished_hash_algorithm;
            uint8_t digest[alg->digest_size];
            ek_hash_state_t state;
            alg->init(&state);
            alg->update(&state, conn->headers, conn->headers_len);
            alg->final(&state, digest);

            uint8_t vd[12];
            conn->cipherparams->prf_algorithm.prf(&conn->cipherparams->prf_algorithm,
                                                  conn->cipherparams->master_secret,
                                                  conn->cipherparams->master_secret_len,
                                                  "client finished",
                                                  digest, alg->digest_size, vd, sizeof(vd));

            memcpy(&response[response_pos], vd, sizeof(vd));
            response_pos += sizeof(vd);
        }

        // compute length
        response[1] = ((response_pos - 4) >> 16) & 0xff;
        response[2] = ((response_pos - 4) >> 8) & 0xff;
        response[3] = ((response_pos - 4) >> 0) & 0xff;

        buffer_append(&conn->headers, &conn->headers_len, response, response_pos);
        ek_tls_conn_write_record(conn, 0x16, response, response_pos);
    }

    while (conn->state != STATE_APPLICATION_DATA) {
        if (ek_tls_conn_read_record(conn, timeout_ms))
            goto teardown;

        conn->on_record(conn);
    }

    return conn;

  teardown: ;

    return NULL;
}
