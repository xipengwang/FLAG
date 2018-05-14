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
#include <string.h>
#include <assert.h>

#include "ek_tls.h"

const struct ek_tls_ciphersuite ek_tls_ciphersuites[] = {
    { .name = "TLS_NULL_WITH_NULL_NULL",                .id = 0x0000 },
    { .name = "TLS_RSA_WITH_NULL_MD5",                  .id = 0x0001 },
    { .name = "TLS_RSA_WITH_NULL_SHA",                  .id = 0x0002 },
    { .name = "TLS_RSA_WITH_RC4_128_MD5",               .id = 0x0004 },
    { .name = "TLS_RSA_WITH_RC4_128_SHA",               .id = 0x0005 },
    { .name = "TLS_RSA_WITH_IDEA_CBC_SHA",              .id = 0x0007 },
    { .name = "TLS_RSA_WITH_DES_CBC_SHA",               .id = 0x0009 },
    { .name = "TLS_RSA_WITH_3DES_EDE_CBC_SHA",          .id = 0x000A },
    { .name = "TLS_DH_DSS_WITH_DES_CBC_SHA",            .id = 0x000C },
    { .name = "TLS_DH_DSS_WITH_3DES_EDE_CBC_SHA",       .id = 0x000D },
    { .name = "TLS_DH_RSA_WITH_DES_CBC_SHA",            .id = 0x000F },
    { .name = "TLS_DH_RSA_WITH_3DES_EDE_CBC_SHA",       .id = 0x0010 },
    { .name = "TLS_DHE_DSS_WITH_DES_CBC_SHA",           .id = 0x0012 },
    { .name = "TLS_DHE_DSS_WITH_3DES_EDE_CBC_SHA",      .id = 0x0013 },
    { .name = "TLS_DHE_RSA_WITH_DES_CBC_SHA",           .id = 0x0015 },
    { .name = "TLS_DHE_RSA_WITH_3DES_EDE_CBC_SHA",      .id = 0x0016 },
    { .name = "TLS_DH_anon_WITH_RC4_128_MD5",           .id = 0x0018 },
    { .name = "TLS_DH_anon_WITH_DES_CBC_SHA",           .id = 0x001A },
    { .name = "TLS_DH_anon_WITH_3DES_EDE_CBC_SHA",      .id = 0x001B },
    { .name = "TLS_RSA_EXPORT_WITH_RC4_40_MD5",         .id = 0x0003 },
    { .name = "TLS_RSA_EXPORT_WITH_RC2_CBC_40_MD5",     .id = 0x0006 },
    { .name = "TLS_RSA_EXPORT_WITH_DES40_CBC_SHA",      .id = 0x0008 },
    { .name = "TLS_DH_DSS_EXPORT_WITH_DES40_CBC_SHA",   .id = 0x000B },
    { .name = "TLS_DH_RSA_EXPORT_WITH_DES40_CBC_SHA",   .id = 0x000E },
    { .name = "TLS_DHE_DSS_EXPORT_WITH_DES40_CBC_SHA",  .id = 0x0011 },
    { .name = "TLS_DHE_RSA_EXPORT_WITH_DES40_CBC_SHA",  .id = 0x0014 },
    { .name = "TLS_DH_anon_EXPORT_WITH_RC4_40_MD5",     .id = 0x0017 },
    { .name = "TLS_DH_anon_EXPORT_WITH_DES40_CBC_SHA",  .id = 0x0019 },
    { .name = "TLS_KRB5_WITH_DES_CBC_SHA",              .id = 0x001E },
    { .name = "TLS_KRB5_WITH_3DES_EDE_CBC_SHA",         .id = 0x001F },
    { .name = "TLS_KRB5_WITH_RC4_128_SHA",              .id = 0x0020 },
    { .name = "TLS_KRB5_WITH_IDEA_CBC_SHA",             .id = 0x0021 },
    { .name = "TLS_KRB5_WITH_DES_CBC_MD5",              .id = 0x0022 },
    { .name = "TLS_KRB5_WITH_3DES_EDE_CBC_MD5",         .id = 0x0023 },
    { .name = "TLS_KRB5_WITH_RC4_128_MD5",              .id = 0x0024 },
    { .name = "TLS_KRB5_WITH_IDEA_CBC_MD5",             .id = 0x0025 },
    { .name = "TLS_KRB5_EXPORT_WITH_DES_CBC_40_SHA",    .id = 0x0026 },
    { .name = "TLS_KRB5_EXPORT_WITH_RC2_CBC_40_SHA",    .id = 0x0027 },
    { .name = "TLS_KRB5_EXPORT_WITH_RC4_40_SHA",        .id = 0x0028 },
    { .name = "TLS_KRB5_EXPORT_WITH_DES_CBC_40_MD5",    .id = 0x0029 },
    { .name = "TLS_KRB5_EXPORT_WITH_RC2_CBC_40_MD5",    .id = 0x002A },
    { .name = "TLS_KRB5_EXPORT_WITH_RC4_40_MD5",        .id = 0x002B },
    { .name = "TLS_DH_DSS_WITH_AES_128_CBC_SHA",        .id = 0x0030 },
    { .name = "TLS_DH_RSA_WITH_AES_128_CBC_SHA",        .id = 0x0031 },
    { .name = "TLS_DHE_DSS_WITH_AES_128_CBC_SHA",       .id = 0x0032 },
    { .name = "TLS_DHE_RSA_WITH_AES_128_CBC_SHA",       .id = 0x0033 },
    { .name = "TLS_DH_anon_WITH_AES_128_CBC_SHA",       .id = 0x0034 },
    { .name = "TLS_DH_DSS_WITH_AES_256_CBC_SHA",        .id = 0x0036 },
    { .name = "TLS_DH_RSA_WITH_AES_256_CBC_SHA",        .id = 0x0037 },
    { .name = "TLS_DHE_DSS_WITH_AES_256_CBC_SHA",       .id = 0x0038 },
    { .name = "TLS_DHE_RSA_WITH_AES_256_CBC_SHA",       .id = 0x0039 },
    { .name = "TLS_DH_anon_WITH_AES_256_CBC_SHA",       .id = 0x003A },

    { .name = "TLS_RSA_WITH_AES_128_CBC_SHA",           .id = 0x002F,
      .preference = 1, .mac_name = "SHA1", .prf_name = "TLS_12_SHA256",
      .finished_name = "SHA256", .block_cipher_name = "AES128"},

    { .name = "TLS_RSA_WITH_AES_128_CBC_SHA256",        .id = 0x003c,
      .preference = 2, .mac_name = "SHA256", .prf_name = "TLS_12_SHA256",
      .finished_name = "SHA256", .block_cipher_name = "AES128" },
    { .name = NULL, .id = 0x0000 },

    { .name = "TLS_RSA_WITH_AES_256_CBC_SHA",           .id = 0x0035,
      .preference = 3, .mac_name = "SHA1", .prf_name = "TLS_12_SHA256",
      .finished_name = "SHA256", .block_cipher_name = "AES256"},
};


void ek_tls_cipherparams_destroy(struct ek_tls_cipherparams *params)
{
    if (!params)
        return;

    free(params->master_secret);
    free(params->client_write_mac_key);
    free(params->server_write_mac_key);
    free(params->client_write_key);
    free(params->server_write_key);
    free(params->client_write_IV);
    free(params->server_write_IV);

    free(params);
}

// read a TLS record from the underlying TCP stream and store it in
// conn->rx_buffer. Decrypts it using the current connection
// parameters and updates the pointer conn->record.
int ek_tls_conn_read_record(ek_tls_conn_t *conn, int timeout_ms)
{
    // otherwise, we need to read a frame from the underlying stream
    if (5 != estream_read_fully_timeout(conn->tcp_stream, conn->rx_buffer, 5, timeout_ms))
        return -1;

    int length = (conn->rx_buffer[3] << 8) + (conn->rx_buffer[4]);
    conn->rx_len = length + 5;

    if (conn->rx_len > conn->rx_alloc)
        return -1;

    if (length != estream_read_fully_timeout(conn->tcp_stream, &conn->rx_buffer[5], length, timeout_ms))
        return -1;

    //////////////////////////////////////////////////
    // decrypt using current ciphersuite/cipherparams
    uint8_t *rx_buffer = conn->rx_buffer;
    int rx_len = conn->rx_len;

    if (!conn->received_change_cipher_spec) {
        conn->record = conn->rx_buffer;
        conn->record_len = rx_len;
        return 0;
    }

    // contents: [Record Header (5)] [IV] [[Enc: [Data] [MAC] [Padding] ]]
    //           -------------------rx_buffer, rx_len --------------------
    //                              -------record, record_len-------------
    //                                            -mod-
    // (mod = modified_record_len)
    int iv_length = conn->cipherparams->block_cipher.block_size;
    int mac_length = conn->cipherparams->mac_algorithm.digest_size;

    int payload_len = rx_len - 5 - iv_length;
    if (payload_len < 0 || ((payload_len % conn->cipherparams->block_cipher.block_size) != 0)) {
        EK_TLSDBG("invalid ciphertext payload length: %d", payload_len);
        return -1;
    }

    ek_block_cipher_decrypt_cbc(&conn->cipherparams->block_cipher,
                                conn->type == EK_TLS_CONN_CLIENT ?
                                  &conn->cipherparams->server_write_keyexp :
                                  &conn->cipherparams->client_write_keyexp,
                                &rx_buffer[5], // iv
                                &rx_buffer[5 + iv_length], rx_len - 5 - iv_length);

    // check padding. There is always at least one byte of
    // padding, and the padding consists of the number of
    // bytes of padding minus one.
    int padding_byte = rx_buffer[rx_len - 1];

    // verify that the padding is intact
    // XXX beware padding oracle attacks?
    if (rx_len - 1 - padding_byte < 0) {
        EK_TLSDBG("bad padding byte %02x\n", padding_byte);
        ek_print_buffer(&rx_buffer[5], rx_len - 5);
        return -1;
    }

    for (int i = 0; i < padding_byte + 1; i++) { // check padding_byte+1 bytes.
        if (rx_buffer[rx_len - 1 - i] != padding_byte) {
            EK_TLSDBG("Bad padding at %d, pad byte %02x\n", i, padding_byte);
            ek_print_buffer(rx_buffer, rx_len);
            return -1;
        }
    }

    // create a new valid plaintext record by copying the header
    // just before the unencrypted data (overwriting part over the
    // IV). The decrypted data starts at &record[iv_length + 5],
    // so the new header should start 5 bytes before that.

    // adjust length. (What is the length of the plaintext payload within the record?)
    int modified_record_len = rx_len - iv_length - mac_length - padding_byte - 1 - 5;

    rx_buffer[iv_length + 0] = rx_buffer[0]; // content_type
    rx_buffer[iv_length + 1] = rx_buffer[1]; // major version
    rx_buffer[iv_length + 2] = rx_buffer[2]; // minor version
    rx_buffer[iv_length + 3] = (modified_record_len >> 8) & 0xff;
    rx_buffer[iv_length + 4] = (modified_record_len >> 0) & 0xff;

    uint8_t *record = &rx_buffer[iv_length];
    conn->record = record;
    conn->record_len = modified_record_len;

    // verify the MAC
    if (1) {
        ek_hmac_state_t hmac_state;
        ek_hmac_init(&hmac_state,
                     &conn->cipherparams->mac_algorithm,
                     conn->type == EK_TLS_CONN_CLIENT ?
                       conn->cipherparams->server_write_mac_key :
                       conn->cipherparams->client_write_mac_key,
                     conn->type == EK_TLS_CONN_CLIENT ?
                       conn->cipherparams->server_write_mac_key_len :
                       conn->cipherparams->client_write_mac_key_len);

        ek_hmac_update(&hmac_state, (uint8_t[]) {
                (conn->sequence_number_rx >> 56) & 0xff,
                    (conn->sequence_number_rx >> 48) & 0xff,
                    (conn->sequence_number_rx >> 40) & 0xff,
                    (conn->sequence_number_rx >> 32) & 0xff,
                    (conn->sequence_number_rx >> 24) & 0xff,
                    (conn->sequence_number_rx >> 16) & 0xff,
                    (conn->sequence_number_rx >> 8) & 0xff,
                    (conn->sequence_number_rx >> 0) & 0xff },
            8);

        ek_hmac_update(&hmac_state, record, modified_record_len + 5);

        uint8_t hmac[conn->cipherparams->mac_algorithm.digest_size];
        ek_hmac_final(&hmac_state, hmac);

        conn->sequence_number_rx ++;

        // the rx'd MAC is after our rx'd data.
        if (memcmp(hmac, &record[5 + modified_record_len],
                   conn->cipherparams->mac_algorithm.digest_size)) {
            EK_TLSDBG("MAC error");
            return -1;
        }
    }

    return 0;
}


// write a record. The response is the message itself WITHOUT the record header.
//
// You must call with response containing enough room for the MAC plus padding.
//
// returns non-zero on error
int ek_tls_conn_write_record(ek_tls_conn_t *conn, uint8_t type, uint8_t *response, int response_len)
{
    uint8_t record_header[5];
    record_header[0] = type;
    record_header[1] = conn->negotiated_version_major;
    record_header[2] = conn->negotiated_version_minor;
    record_header[3] = (response_len >> 8) & 0xff;
    record_header[4] = (response_len >> 0) & 0xff;

    if (!conn->sent_change_cipher_spec) {
        int res = conn->tcp_stream->writev_fully(conn->tcp_stream, 2,
                                                 (const void*[]) { record_header, response },
                                                 (uint64_t[])   { 5, response_len});
        if (res < 0)
            return -1;

    } else {
        assert(conn->cipherparams);

        int block_size = conn->cipherparams->block_cipher.block_size;
        int digest_size = conn->cipherparams->mac_algorithm.digest_size;

        uint8_t *iv = ek_random_buffer_create(block_size);
        uint8_t hmac[digest_size];

        if (1) {
            // compute and append the hmac
            uint64_t sequence_number = conn->sequence_number_tx;
            conn->sequence_number_tx++;

            ek_hmac_state_t hmac_state;
            ek_hmac_init(&hmac_state,
                         &conn->cipherparams->mac_algorithm,
                         conn->type == EK_TLS_CONN_CLIENT ?
                           conn->cipherparams->client_write_mac_key :
                           conn->cipherparams->server_write_mac_key,
                         conn->type == EK_TLS_CONN_CLIENT ?
                           conn->cipherparams->client_write_mac_key_len :
                           conn->cipherparams->server_write_mac_key_len);

            ek_hmac_update(&hmac_state, (uint8_t[]) {
                    (sequence_number >> 56) & 0xff,
                        (sequence_number >> 48) & 0xff,
                        (sequence_number >> 40) & 0xff,
                        (sequence_number >> 32) & 0xff,
                        (sequence_number >> 24) & 0xff,
                        (sequence_number >> 16) & 0xff,
                        (sequence_number >> 8) & 0xff,
                        (sequence_number >> 0) & 0xff }, 8);

            // at this point, the record_header still encodes the plaintext length
            ek_hmac_update(&hmac_state, record_header, 5);
            ek_hmac_update(&hmac_state, response, response_len);

            ek_hmac_final(&hmac_state, hmac);

            // append
            for (int i = 0; i < digest_size; i++)
                response[response_len++] = hmac[i];
        }

        if (1) {
            // pad
            int padding_size = block_size - ((response_len + 1) % block_size);
            for (int i = 0; i < padding_size + 1; i++)
                response[response_len++] = padding_size;
        }

        ek_block_cipher_encrypt_cbc(&conn->cipherparams->block_cipher,
                                    conn->type == EK_TLS_CONN_CLIENT ?
                                      &conn->cipherparams->client_write_keyexp :
                                      &conn->cipherparams->server_write_keyexp,
                                    iv,
                                    response,
                                    response_len);

        record_header[3] = ((block_size + response_len) >> 8) & 0xff;
        record_header[4] = ((block_size + response_len) >> 0) & 0xff;

        int res = conn->tcp_stream->writev_fully(conn->tcp_stream, 3,
                                                 (const void*[]) { record_header, iv, response },
                                                 (uint64_t[]) { 5, block_size, response_len });


        free(iv);
        if (res < 0)
            return -1;
    }

    return 0;
}

////////////////////////////////////////////////////////////
// estream_t functions
////////////////////////////////////////////////////////////
static int tls_conn_write_application_data(ek_tls_conn_t *conn, const void *_buf, uint64_t buf_len)
{
    const uint8_t *buf = _buf;
    int max_record_size = 8192; // we could go 2^14
    uint8_t response[max_record_size + 1024]; // extra space for ek_tls_conn_write_record()

    for (uint64_t offset = 0; offset < buf_len; offset += max_record_size) {
        int this_len = buf_len - offset;
        if (this_len > max_record_size)
            this_len = max_record_size;

        memcpy(response, &buf[offset], this_len);

        int res = ek_tls_conn_write_record(conn, 0x17, response, this_len);
        if (res < 0)
            return res;
    }

    return 0;
}

static int tls_stream_writev_fully(estream_t *es, int vlen, const void **bufs, uint64_t *lens)
{
    ek_tls_conn_t *conn = es->user;
    int total_written = 0;

    for (int vidx = 0; vidx < vlen; vidx++) {

        if (tls_conn_write_application_data(conn, bufs[vidx], lens[vidx]) < 0)
            return -1;

        total_written += lens[vidx];
    }

    return total_written;
}


static int tls_stream_read_timeout(estream_t *es, void *buf, int maxsize, int timeout_ms)
{
    ek_tls_conn_t *conn = es->user;

    while (conn->app_data_pos >= conn->app_data_len) {
        int res = ek_tls_conn_read_record(conn, timeout_ms);
        if (res) {
            EK_TLSDBG("TLS conn err %d; timeout_ms %d", res, timeout_ms);
            // XXX close the connection
            return res;
        }

        res = conn->on_record(conn);
        if (res)
            return res;
    }

    int len = maxsize;
    int maxlen = conn->app_data_len - conn->app_data_pos;
    if (len > maxlen)
        len = maxlen;
    memcpy(buf, &conn->app_data[conn->app_data_pos], len);
    conn->app_data_pos += len;

    return len;
}

static int tls_stream_close(estream_t *stream)
{
    ek_tls_conn_t *conn = stream->user;
    conn->tcp_stream->close(conn->tcp_stream);

    stream->closed = conn->tcp_stream->closed;
    return 0;
}

static void tls_stream_destroy(estream_t *stream)
{
    if (!stream->closed)
        tls_stream_close(stream);

    ek_tls_conn_t *conn = stream->user;
    conn->destroy(conn);
}

estream_t *ek_tls_stream_create(ek_tls_conn_t *conn)
{
    estream_t *tls_stream;

    tls_stream = calloc(1, sizeof(estream_t));
    tls_stream->writev_fully = tls_stream_writev_fully;
    tls_stream->read_timeout = tls_stream_read_timeout;
    tls_stream->close = tls_stream_close;
    tls_stream->destroy = tls_stream_destroy;
    tls_stream->user = conn;

    return tls_stream;
}


struct ek_tls_cipherparams *ek_tls_create_ciphersuite_parameters(const struct ek_tls_ciphersuite *ciphersuite,
                                                                 const uint8_t *client_random, int client_random_len,
                                                                 const uint8_t *server_random, int server_random_len,
                                                                 const uint8_t *premaster_secret, int premaster_secret_len)
{
    // create ciphersuite parameters
    struct ek_tls_cipherparams *params = calloc(1, sizeof(struct ek_tls_cipherparams));

    if (ek_hash_algorithm_get(ciphersuite->mac_name, &params->mac_algorithm)) {
        EK_TLSDBG("Bad ciphersuite name %s", ciphersuite->mac_name);
        goto teardown;
    }

    if (ek_hash_algorithm_get(ciphersuite->finished_name, &params->finished_hash_algorithm))
        goto teardown;

    if (ek_prf_algorithm_get(ciphersuite->prf_name, &params->prf_algorithm))
        goto teardown;

    if (ek_block_cipher_get(ciphersuite->block_cipher_name, &params->block_cipher))
        goto teardown;

    params->client_write_mac_key_len = params->mac_algorithm.digest_size;
    params->server_write_mac_key_len = params->mac_algorithm.digest_size;
    params->client_write_key_len = params->block_cipher.key_size;
    params->server_write_key_len = params->block_cipher.key_size;
    params->client_write_IV_len = 0;
    params->server_write_IV_len = 0;

    /////////////////////////////////////////////////////////
    // create ciphersuite parameters
    if (1) {
        int secrets_len = client_random_len + server_random_len;
        uint8_t secrets[secrets_len];
        memcpy(secrets, client_random, client_random_len);
        memcpy(&secrets[client_random_len], server_random, server_random_len);

        params->master_secret_len = 48;
        params->master_secret = malloc(params->master_secret_len);

        params->prf_algorithm.prf(&params->prf_algorithm,
                                  premaster_secret, premaster_secret_len,
                                  "master secret",
                                  secrets, secrets_len,
                                  params->master_secret, params->master_secret_len);

//        printf("master secret:\n");
//        ek_print_buffer(params->master_secret, params->master_secret_len);

        // XXX recommended to clear premaster secret at this point.
        // memset(premaster_secret, 0, sizeof(premaster_secret));
    }

    if (1) {
        int secrets_len = client_random_len + server_random_len;
        uint8_t secrets[secrets_len];
        // NB: order of server/client randoms is backwards from above.
        memcpy(secrets, server_random, server_random_len);
        memcpy(&secrets[server_random_len], client_random, client_random_len);

        int prf_nbytes = params->client_write_mac_key_len +
            params->server_write_mac_key_len +
            params->client_write_key_len +
            params->server_write_key_len +
            params->client_write_IV_len +
            params->server_write_IV_len;

        uint8_t prf_data[prf_nbytes];
        params->prf_algorithm.prf(&params->prf_algorithm,
                                  params->master_secret, params->master_secret_len,
                                  "key expansion",
                                  secrets, secrets_len, prf_data, prf_nbytes);

        int pos = 0;
        params->client_write_mac_key = malloc(params->client_write_mac_key_len);
        memcpy(params->client_write_mac_key, &prf_data[pos], params->client_write_mac_key_len);
        pos += params->client_write_mac_key_len;

        params->server_write_mac_key = malloc(params->client_write_mac_key_len);
        memcpy(params->server_write_mac_key, &prf_data[pos], params->server_write_mac_key_len);
        pos += params->server_write_mac_key_len;

        params->client_write_key = malloc(params->client_write_key_len);
        memcpy(params->client_write_key, &prf_data[pos], params->client_write_key_len);
        pos += params->client_write_key_len;

        params->server_write_key = malloc(params->client_write_key_len);
        memcpy(params->server_write_key, &prf_data[pos], params->server_write_key_len);
        pos += params->server_write_key_len;

        params->client_write_IV = malloc(params->client_write_IV_len);
        memcpy(params->client_write_IV, &prf_data[pos], params->client_write_IV_len);
        pos += params->client_write_IV_len;

        params->server_write_IV = malloc(params->client_write_IV_len);
        memcpy(params->server_write_IV, &prf_data[pos], params->server_write_IV_len);
        pos += params->server_write_IV_len;

        params->block_cipher.keyexp(&params->block_cipher,
                                    params->client_write_key,
                                    &params->client_write_keyexp);

        params->block_cipher.keyexp(&params->block_cipher,
                                    params->server_write_key,
                                    &params->server_write_keyexp);
    }

    return params;

  teardown:
    return NULL;
}
