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
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include "ek_tls.h"
#include "common/io_util.h"
#include "common/estream.h"
#include "ek_asn.h"

// what message are we expecting next? (Note: some "unexpected"
// messages are okay). For example, the CLIENT can skip
// _CLIENT_CERTIFICATE depending on what we send in the server
// handshake.
enum { STATE_CLIENT_HELLO = 1, STATE_CLIENT_CERTIFICATE, STATE_CLIENT_KEY_EXCHANGE,
       STATE_CLIENT_CHANGE_CIPHERSPEC, STATE_CLIENT_FINISHED,
       STATE_CLIENT_CERTIFICATE_VERIFY,
       STATE_APPLICATION_DATA };

static void buffer_append(uint8_t **_buf, int *bufpos, const void *s, int slen)
{
    uint8_t *buf = *_buf;

    buf = realloc(buf, *bufpos + slen);
    *_buf = buf;

    memcpy(&buf[*bufpos], s, slen);
    (*bufpos) += slen;
}

int server_conn_on_record(ek_tls_conn_t *conn)
{
    int ret = 0;
    ek_tls_server_t *server = conn->server;

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
        record_version_minor < conn->negotiated_version_minor) {
        EK_TLSDBG("Record version isn't negotiated version");
        goto teardown;
    }

    switch (record_type) {
        case 0x14: {
            // ChangeCipherSpec
            EK_TLSDBG("RX ChangeCipherSpec");

            if (conn->state != STATE_CLIENT_CHANGE_CIPHERSPEC &&
                conn->state != STATE_CLIENT_CERTIFICATE_VERIFY) {
                EK_TLSDBG("Bad state change: in state %d", conn->state);
                goto teardown;
            }

            if (record_len - record_pos != 1) {
                EK_TLSDBG("bad ChangeCipherSpec length");
                goto teardown;
            }

            if (record[5] != 1) {
                EK_TLSDBG("bad ChangeCipherSpec contents");
                goto teardown;
            }

            if (conn->pending_ciphersuite == NULL) {
                EK_TLSDBG("no pending ciphersuite");
                goto teardown;
            }

            conn->ciphersuite = conn->pending_ciphersuite;
            conn->pending_ciphersuite = NULL;

            conn->cipherparams = conn->pending_cipherparams;
            conn->pending_cipherparams = NULL;

            conn->sequence_number_rx = 0; // Not sure about this.
            conn->sequence_number_tx = 0;

            conn->state = STATE_CLIENT_FINISHED;

            conn->received_change_cipher_spec = 1;
            break;
        }

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

                // XXX Multiple handshake messages can be inside one record.

                switch (handshake_type) {

                    case 1: {
                        EK_TLSDBG("RX ClientHello");
                        if (conn->state != STATE_CLIENT_HELLO)
                            goto teardown;

                        // update handshake message log. (RESET?)
                        if (conn->headers) {
                            free(conn->headers);
                            conn->headers = NULL;
                            conn->headers_len = 0;
                        }

                        buffer_append(&conn->headers, &conn->headers_len, handshake, handshake_len);

                        ////////////////////////////////////////////////
                        // TLS 1.2 or bust.
                        uint8_t request_version_major = handshake[handshake_pos++];
                        uint8_t request_version_minor = handshake[handshake_pos++];

                        if (request_version_major != 3 || request_version_minor != 3)
                            goto teardown;

                        conn->negotiated_version_major = 3;
                        conn->negotiated_version_minor = 3;

                        ////////////////////////////////////////////////
                        // store client_random
                        conn->client_random_len = 32;
                        conn->client_random = malloc(conn->client_random_len);
                        memcpy(conn->client_random, &handshake[handshake_pos], conn->client_random_len);
                        handshake_pos += conn->client_random_len;

                        ////////////////////////////////////////////////
                        // no session resume (for now)
                        uint8_t session_id_length = handshake[handshake_pos++];
                        if (session_id_length > 0)
                            goto teardown;

                        ////////////////////////////////////////////////
                        // pick a cipher suite
                        int cipher_suites_length = (handshake[handshake_pos]<<8) + handshake[handshake_pos+1];
                        handshake_pos += 2;
                        int cipher_suites_end = handshake_pos + cipher_suites_length;

                        if (conn->pending_ciphersuite != NULL)
                            goto teardown;

                        // look at the advertised modes and pick the one we like the best.
                        while (handshake_pos < cipher_suites_end) {

                            int id = (handshake[handshake_pos + 0] << 8) + handshake[handshake_pos + 1];
                            handshake_pos += 2;

                            const struct ek_tls_ciphersuite *cs = NULL;
                            for (int j = 0; ek_tls_ciphersuites[j].name != NULL; j++) {
                                if (ek_tls_ciphersuites[j].id == id) {
                                    cs = &ek_tls_ciphersuites[j];

                                    // never use this ciphersuite?
                                    if (cs->preference == 0)
                                        continue;

                                    if (conn->pending_ciphersuite == NULL || cs->preference < conn->pending_ciphersuite->preference)
                                        conn->pending_ciphersuite = cs;
                                    break;
                                }
                            }

//                        printf("cipher suite: %04x %s\n", id, cs ? cs->name : "(unknown)");
                        }

                        if (conn->pending_ciphersuite == NULL) {
                            EK_TLSDBG("No acceptable ciphersuites");
                            goto teardown;
                        }

                        EK_TLSDBG("negotiated cipher suite %s", conn->pending_ciphersuite->name);
                        handshake_pos += cipher_suites_length;

                        ////////////////////////////////////////////////
                        // "negotiate" no compression.
                        uint8_t compression_methods_length = handshake[handshake_pos];
                        handshake_pos += 1;

                        if (0) {
                            for (int i = 0; i < compression_methods_length; i+=1) {
                                printf("compression method: %02x\n", handshake[handshake_pos+i]);
                            }
                        }

                        handshake_pos += compression_methods_length;

                        // Now, send ServerHello, Certificate*,
                        // ServerKeyExchange*, CertificateRequest*,
                        // ServerHelloDone
                        //

                        if (1) { // ServerHello
                            EK_TLSDBG("TX ServerHello");

                            // the payload, sans record header.
                            uint8_t response[32768];
                            int response_pos = 0;

                            response[response_pos++] = 2; // ServerHello
                            response[response_pos++] = 0; // length (will be filled in)
                            response[response_pos++] = 0;
                            response[response_pos++] = 0;

                            // server version
                            response[response_pos++] = 3; // suggest TLS 1.2
                            response[response_pos++] = 3; // suggest TLS 1.2

                            // server random
                            memcpy(&response[response_pos], conn->server_random, conn->server_random_len);
                            response_pos += conn->server_random_len;

                            // no session id
                            response[response_pos++] = 0;

                            // select a cipher suite
                            response[response_pos++] = conn->pending_ciphersuite->id >> 8;
                            response[response_pos++] = conn->pending_ciphersuite->id & 0xff;

                            // select a compression method (none)
                            response[response_pos++] = 0;

                            // compute length
                            response[1] = ((response_pos - 4) >> 16) & 0xff;
                            response[2] = ((response_pos - 4) >> 8) & 0xff;
                            response[3] = ((response_pos - 4) >> 0) & 0xff;

                            // write a handshake record
                            buffer_append(&conn->headers, &conn->headers_len, response, response_pos);
                            ek_tls_conn_write_record(conn, 0x16, response, response_pos);
                        }

                        if (1) { // Certificate
                            EK_TLSDBG("TX Certificate");

                            uint8_t response[32768];
                            int response_pos = 0;

                            response[response_pos++] = 11; // Certificate
                            response[response_pos++] = 0;  // length (will be filled in)
                            response[response_pos++] = 0;
                            response[response_pos++] = 0;

                            int sz = server->ident->cert->datalen;

                            // certificate LIST length; we have only one cert
                            response[response_pos++] = ((sz+3) >> 16) % 256;
                            response[response_pos++] = ((sz+3) >> 8) % 256;
                            response[response_pos++] = ((sz+3) >> 0) % 256;

                            // send the certificate (its length followed by data)
                            response[response_pos++] = (sz >> 16) % 256;
                            response[response_pos++] = (sz >> 8) % 256;
                            response[response_pos++] = (sz >> 0) % 256;

                            memcpy(&response[response_pos], server->ident->cert->data, server->ident->cert->datalen);
                            response_pos += server->ident->cert->datalen;

                            // compute length
                            response[1] = ((response_pos - 4) >> 16) & 0xff;
                            response[2] = ((response_pos - 4) >> 8) & 0xff;
                            response[3] = ((response_pos - 4) >> 0) & 0xff;

                            // write a handshake record
                            buffer_append(&conn->headers, &conn->headers_len, response, response_pos);
                            ek_tls_conn_write_record(conn, 0x16, response, response_pos);
                        }

                        // Don't send ServerKeyExchange* or
                        // CertificateRequest*; not implemented.

                        if (1) {
                            EK_TLSDBG("TX ServerDone");
                            uint8_t response[32768];
                            int response_pos = 0;

                            response[response_pos++] = 14; // ServerDone
                            response[response_pos++] = 0;  // length (will be filled in)
                            response[response_pos++] = 0;
                            response[response_pos++] = 0;

                            // compute length
                            response[1] = ((response_pos - 4) >> 16) & 0xff;
                            response[2] = ((response_pos - 4) >> 8) & 0xff;
                            response[3] = ((response_pos - 4) >> 0) & 0xff;

                            // write a handshake record
                            buffer_append(&conn->headers, &conn->headers_len, response, response_pos);
                            ek_tls_conn_write_record(conn, 0x16, response, response_pos);
                        }

                        conn->state = STATE_CLIENT_CERTIFICATE;
                        break;
                    }

                    case 16: {
                        EK_TLSDBG("RX ClientKeyExchange");
                        if (conn->state != STATE_CLIENT_CERTIFICATE &&
                            conn->state != STATE_CLIENT_KEY_EXCHANGE)
                            goto teardown;

                        buffer_append(&conn->headers, &conn->headers_len, handshake, handshake_len);

                        // allocate worst-case rsa key size / 8, (4096 bit)
                        uint8_t premaster_secret[256];
                        int premaster_secret_len = 0;

                        // RSA-encrypted premaster secret
                        if (1) {
                            int enc_len = (handshake[handshake_pos] << 8) + handshake[handshake_pos+1];
                            handshake_pos += 2;

                            if (enc_len != 256) {
                                EK_TLSDBG("Bad RSA encrypted message length");
                                goto teardown;
                            }

                            int res;
                            if (server->ident->rsa.prime1) {
                                res = ek_rsa_pkcs_15_decrypt_crt(server->ident->rsa.modulus,
                                                                 server->ident->rsa.prime1, server->ident->rsa.prime2,
                                                                 server->ident->rsa.exponent1, server->ident->rsa.exponent2,
                                                                 server->ident->rsa.coefficient,
                                                                 &handshake[handshake_pos],
                                                                 premaster_secret, &premaster_secret_len);
                            } else {
                                res = ek_rsa_pkcs_15_decrypt_privexp(server->ident->rsa.modulus, server->ident->rsa.privexp,
                                                                     &handshake[handshake_pos],
                                                                     premaster_secret, &premaster_secret_len);
                            }


                            if (res || premaster_secret_len != 48) {
                                EK_TLSDBG("RSA decryption error %d\n", res);
                                goto teardown;
                            }

                            if (premaster_secret[0] != 0x03 || premaster_secret[1] != 0x03) {
                                EK_TLSDBG("invalid TLS version in pre-master secret\n");
                                goto teardown;
                            }
                        }

                        /////////////////////////////////////////////////////////
                        // create ciphersuite parameters
                        struct ek_tls_cipherparams *params = ek_tls_create_ciphersuite_parameters(conn->pending_ciphersuite,
                                                                                                  conn->client_random, conn->client_random_len,
                                                                                                  conn->server_random, conn->server_random_len,
                                                                                                  premaster_secret, premaster_secret_len);
                        if (!params)
                            goto teardown;

                        // XXX free old suite
                        conn->pending_cipherparams = params;

                        conn->state = STATE_CLIENT_CERTIFICATE_VERIFY;
                        break;
                    }

                    case 20: {
                        EK_TLSDBG("RX Finished");
                        if (conn->state != STATE_CLIENT_FINISHED) {
                            EK_TLSDBG("Wrong state");
                            goto teardown;
                        }

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

                            if (handshake_len - 4 != sizeof(vd)) {
                                EK_TLSDBG("bad digest length %d %d\n", record_len, alg->digest_size);
                                goto teardown;
                            }

                            if (memcmp(vd, &handshake[4], sizeof(vd))) {
                                printf("bad verify\n");
                                goto teardown;
                            }
                        }

                        buffer_append(&conn->headers, &conn->headers_len, handshake, handshake_len);

                        if (1) { // send ChangeCipherSpec
                            EK_TLSDBG("TX ChangeCipherSpec");
                            uint8_t response[65536];
                            int response_pos = 0;
                            response[response_pos++] = 1;

                            ek_tls_conn_write_record(conn, 0x14, response, response_pos);
                        }

                        conn->sent_change_cipher_spec = 1;

                        if (1) { // send Finished
                            EK_TLSDBG("TX Finished");
                            uint8_t response[65536];
                            int response_pos = 0;

                            response[response_pos++] = 20; // Finished
                            response[response_pos++] = 0; // length (will be filled in)
                            response[response_pos++] = 0;
                            response[response_pos++] = 0;

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

                            memcpy(&response[response_pos], vd, 12);
                            response_pos += 12;

                            // compute length
                            response[1] = ((response_pos - 4) >> 16) & 0xff;
                            response[2] = ((response_pos - 4) >> 8) & 0xff;
                            response[3] = ((response_pos - 4) >> 0) & 0xff;

                            buffer_append(&conn->headers, &conn->headers_len, response, response_pos);

                            // write a handshake record
                            ek_tls_conn_write_record(conn, 0x16, response, response_pos);
                        }

                        conn->state = STATE_APPLICATION_DATA;

                        ret = 0;
                        break;
                    }

                    default: {
                        EK_TLSDBG("Unsupported handshake message: %d", handshake_type);
                        goto teardown;
                    }
                }

                record_pos += handshake_len;
            }
            break;
        } // end Handshake

        case 0x17: {
            EK_TLSDBG("RX Application Data");
            if (conn->state != STATE_APPLICATION_DATA)
                goto teardown;

            conn->app_data = &conn->record[record_pos];
            conn->app_data_len = record_len - record_pos;
            conn->app_data_pos = 0;
            ret = 0;

            break;
        }

        case 0x18: {
            EK_TLSDBG("RX Heartbeat");
            break;
        }

        default:
            goto teardown;
    }

    return ret;

  teardown: ;
    ret = -1;
    return ret;

}

void server_conn_destroy(ek_tls_conn_t *conn)
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

ek_tls_server_t *ek_tls_server_create(ek_identity_t *ident)
{
    ek_tls_server_t *server = calloc(1, sizeof(ek_tls_server_t));
    server->ident = ident;
    return server;
}

/*
ek_tls_server_t *ek_tls_server_create(const char *privkeyder_path, const char *cacertder_path)
{
    ek_tls_server_t *server = calloc(1, sizeof(ek_tls_server_t));

    // retreive keys from privkey
    if (1) {
        uint8_t *buf;
        long bufsz;

        if (ioutils_read_file(privkeyder_path, (char**) &buf, &bufsz, 64*1024)) {
            printf("couldn't read %s\n", privkeyder_path);
            free(server);
            return NULL;
        }

        int error;
        ek_asn_result_t *res = ek_asn_result_create_from_der(buf, bufsz, ek_asn_type_create_x509_privkey(), &error);
        if (!res) {
            printf("couldn't parse privkey.der\n");
            return NULL;
        }

        struct pair {
            const char *name;
            ek_bigint_t **dest;
        };

        struct pair *pairs = (struct pair[]) { { .name = "privkey.modulus", .dest = &server->rsa_modulus },
                                               { .name = "privkey.privateExponent", .dest = &server->rsa_privexp },
                                               { .name = "privkey.prime1", .dest = &server->rsa_prime1 },
                                               { .name = "privkey.prime2", .dest = &server->rsa_prime2 },
                                               { .name = "privkey.exponent1", .dest = &server->rsa_exponent1 },
                                               { .name = "privkey.exponent2", .dest = &server->rsa_exponent2 },
                                               { .name = "privkey.coefficient", .dest = &server->rsa_coefficient },
                                               { .name = NULL } };

        for (int i = 0; pairs[i].name != NULL; i++) {
            struct ek_asn_element *el;
            if (ek_asn_result_find(res, pairs[i].name, &el)) {
                printf("Unable to retrieve keys from privkey file: %s\n", pairs[i].name);
                return NULL;
            }

            *pairs[i].dest = ek_bigint_create_from_bytes(el->payload, el->payload_len);
        }

        ek_asn_result_destroy(res);
    }

    // Certificate*
    if (1) {
        char *cacert;
        long cacert_len;

        if (ioutils_read_file(cacertder_path, &cacert, &cacert_len, 64*1024)) {
            printf("failed to open certificate: %s\n", cacertder_path);
            return NULL;
        }
        server->certificate_len = cacert_len;
        server->certificate = (uint8_t*) cacert;
    }


    return server;
}

*/

ek_tls_conn_t *ek_tls_server_conn_create(ek_tls_server_t *server,
                                         estream_t *tcp_stream)
{
    ek_tls_conn_t *conn = calloc(1, sizeof(ek_tls_conn_t));

    conn->type = EK_TLS_CONN_SERVER;
    conn->server = server;
    conn->tcp_stream = tcp_stream;

    conn->rx_alloc = 16384 + 2048 + 5;
    conn->rx_buffer = malloc(conn->rx_alloc);
    conn->rx_len = 0;

    conn->tls_stream = ek_tls_stream_create(conn);

    conn->on_record = server_conn_on_record;
    conn->destroy = server_conn_destroy;

    conn->state = STATE_CLIENT_HELLO;

    conn->server_random_len = 32;
    conn->server_random = ek_random_buffer_create(conn->server_random_len);
    if (1) {
        // adjust first four bytes
        int32_t t = time(NULL);
        conn->server_random[0] = (t >> 24) % 256;
        conn->server_random[1] = (t >> 16) % 256;
        conn->server_random[2] = (t >> 8) % 256;
        conn->server_random[3] = (t >> 0) % 256;
    }

    // we don't actually support this version, but we'll up-negotiate
    // right away.
    conn->negotiated_version_major = 3;
    conn->negotiated_version_minor = 1;

    return conn;
}
