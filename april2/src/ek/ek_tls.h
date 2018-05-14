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

#ifndef _EK_TLS_H
#define _EK_TLS_H

#include <stdint.h>

#include "ek.h"
#include "common/estream.h"
#include "common/zarray.h"

#define EK_TLSDBG(...) if (0) { fprintf(stdout, "TLSDBG: "); fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); }

struct ek_tls_ciphersuite
{
    uint16_t id;
    const char *name;

    // translate 'name' to the algorithm name recognized by
    // ek_hash_algorithm_get(). e.g. "SHA1"
    const char *mac_name;

    // always SHA256 for defined TLS 1.2 modes (so far)
    const char *prf_name;

    // the hash function used in computing FINISHED messages (almost always SHA256)
    const char *finished_name;

    // e.g. "AES128". We only support CBC modes.
    const char *block_cipher_name;

    int preference; // zero=disallowed, otherwise lower numbers "win". 1 is max preference.
};

struct ek_tls_cipherparams
{
    uint8_t *master_secret;
    int master_secret_len;

    uint8_t *client_write_mac_key;
    int client_write_mac_key_len;

    uint8_t *server_write_mac_key;
    int server_write_mac_key_len;

    uint8_t *client_write_key;
    int client_write_key_len;

    uint8_t *server_write_key;
    int server_write_key_len;

    uint8_t *client_write_IV; // only used for AEAD
    int client_write_IV_len;

    uint8_t *server_write_IV; // only used for AEAD
    int server_write_IV_len;

    ek_hash_algorithm_t mac_algorithm;

    ek_prf_algorithm_t prf_algorithm;

    ek_hash_algorithm_t finished_hash_algorithm;

    ek_block_cipher_t block_cipher;
    ek_block_cipher_keyexp_t client_write_keyexp, server_write_keyexp;
};

extern const struct ek_tls_ciphersuite ek_tls_ciphersuites[];

/////////////////////////////////////////////////////////
// ek_tls goals are to make as few assumptions about the underlying
// transport and platform threading models in order to make it
// possible to implement TLS on embedded devices.
//
// This TLS implementation implements an estream_t assuming an
// underlying TCP estream_t. This is done in such a way that no
// additional threads are required: writes are pushed directly as TLS
// records. Reads are implemented by buffering one record in the
// connection object; if a read can be satisfied from this buffer,
// we're done. Otherwise, we perform a blocking read to the underlying
// TCP estream to read a record.

typedef struct ek_tls_server ek_tls_server_t;
struct ek_tls_server
{
    ek_identity_t *ident;
//    ek_bigint_t *rsa_modulus;
//    ek_bigint_t *rsa_privexp;

    // if available, we'll use these for faster decryption operations
//    ek_bigint_t *rsa_prime1, *rsa_prime2, *rsa_exponent1, *rsa_exponent2, *rsa_coefficient;

//    uint8_t *certificate; // in DER form. XXX we'll have to support multiple certs sometime.
//    int certificate_len;
};

// a client capable of making various requests to a server. This
// object exists as a way of providing certificates and other
// credentials.
typedef struct ek_tls_client ek_tls_client_t;
struct ek_tls_client
{
    // if NULL, we won't send a certificate.
    ek_identity_t *ident;

//    uint8_t *certificate; // in DER form. XXX we'll have to support multiple certs sometime.
//    int certificate_len;

//    ek_bigint_t *rsa_modulus; // used for CertificateVerify
//    ek_bigint_t *rsa_privexp, *rsa_pubexp;
};

enum { EK_TLS_CONN_SERVER = 1, EK_TLS_CONN_CLIENT = 2 };

typedef struct ek_tls_conn ek_tls_conn_t;
struct ek_tls_conn
{
    int type; // EK_TLS_CONN_SERVER, EK_TLS_CONN_CLIENT
    int state;

    ek_tls_server_t   *server;
    ek_tls_client_t   *client;

    estream_t *tcp_stream; // our connection to the underlying TCP stream
    estream_t *tls_stream; // the stream we provide to the application

    int cert_requested; // used by clients, set to 1 if CertificateRequest is rx'd.

    // this is called internally when a new record has been read
    // in. clients and servers have different implementations.
    int (*on_record)(ek_tls_conn_t *conn);
    void (*destroy)(ek_tls_conn_t *conn);

    uint8_t *client_random;
    int client_random_len;

    uint8_t *server_random;
    int server_random_len;

    uint8_t *headers; // a log of all headers, used to generate Finished messages.
    int headers_len;

    const struct ek_tls_ciphersuite *ciphersuite; // if NULL, then no encryption or MAC, etc.
    struct ek_tls_cipherparams *cipherparams;

    const struct ek_tls_ciphersuite *pending_ciphersuite;
    struct ek_tls_cipherparams *pending_cipherparams;

    uint64_t sequence_number_rx;
    uint64_t sequence_number_tx;

    uint8_t negotiated_version_major;
    uint8_t negotiated_version_minor;

    int sent_change_cipher_spec, received_change_cipher_spec;

    // This is where a raw record is read in and potentially decrypted
    uint8_t *rx_buffer;
    int      rx_alloc; // how big is rx_buffer
    int      rx_len;

    // this points into rx_buffer. It's not necessarily the beginning due to decryption offset
    uint8_t *record;
    int      record_len;

    // this points into rx_buffer.
    uint8_t *app_data;       // this is decrypted application data
    int      app_data_len;
    int      app_data_pos;

    zarray_t *server_certificates; // asn_result_t*. can be NULL (only used by client connections)

    zarray_t *client_certificates; // asn_result_t*. can be NULL (only used by server connections)

    // for use by anyone outside ek_tls
    void    *user;
    int     user_fd; // doesn't have to be an fd... just user storage.
};

///////////////////////////////////////////////////////
// User API
///////////////////////////////////////////////////////
ek_tls_server_t *ek_tls_server_create(ek_identity_t *ident);

// establish a new connection. If all you care about is the
// tls_stream, you can ignore the conn_t itself. tls_stream->destroy()
// will clean up the ek_tls_conn_t. Do not call ek_tls_conn->destroy() yourself.
ek_tls_conn_t *ek_tls_server_conn_create(ek_tls_server_t *server,
                                         estream_t *tcp_stream);

///////////////////////////////////////////////////////
ek_tls_client_t *ek_tls_client_create();

// establish a new connection. If all you care about is the
// tls_stream, you can ignore the conn_t itself. tls_stream->destroy()
// will clean up the ek_tls_conn_t. Do not call ek_tls_conn->destroy() yourself.
ek_tls_conn_t *ek_tls_client_conn_create(ek_tls_client_t *client, estream_t *tcp_stream);










///////////////////////////////////////////////////////
// Internal functions
///////////////////////////////////////////////////////

void ek_tls_cipherparams_destroy(struct ek_tls_cipherparams *params);

// read a TLS record from the underlying TCP stream and store it in
// conn->rx_buffer. Decrypts it using the current connection
// parameters and updates the pointer conn->record.
int ek_tls_conn_read_record(ek_tls_conn_t *conn, int timeout_ms);

// write a record. The response is the message itself WITHOUT the record header.
//
// CAUTION: You must call with response containing enough room for the MAC plus padding.
//
// returns non-zero on error
int ek_tls_conn_write_record(ek_tls_conn_t *conn, uint8_t type, uint8_t *response, int response_len);

estream_t *ek_tls_stream_create(ek_tls_conn_t *conn);


struct ek_tls_cipherparams *ek_tls_create_ciphersuite_parameters(const struct ek_tls_ciphersuite *ciphersuite,
                                                                 const uint8_t *client_random, int client_random_len,
                                                                 const uint8_t *server_random, int server_random_len,
                                                                 const uint8_t *premaster_secret, int premaster_secret_len);

#endif
