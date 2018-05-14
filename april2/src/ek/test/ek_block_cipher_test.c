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
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include "crypto.h"
#include "ek_block_cipher.h"
#include "aes.h"

int main(int argc, char *argv[])
{
    for (int keysize = 16; 0 && keysize <= 32; keysize += 8) {
        uint8_t key[keysize];
        for (int i = 0; i < keysize; i++)
            key[i] = i;

        ek_block_cipher_t cipher;
        ek_block_cipher_keyexp_t keyexp;

        const char *name = keysize == 16 ? "AES128" : (keysize == 24 ? "AES192" : "AES256");
        if (!ek_block_cipher_get(name, &cipher))
            assert(0);

        cipher.keyexp(&cipher, key, &keyexp);

        WORD ks[60];
        aes_key_setup(key, ks, keysize * 8);

        // these won't match due to our pre-computed transpose of the key schedule
/*        int sz = 4 * (keysize / 4 + 7);
        if (memcmp(ks, keyexp.u.aes.w, sizeof(uint32_t) * sz)) {
            for (int i = 0; i < 4 * (keysize / 4 + 7); i++)
                printf("%2d: %08x %08x\n", i, ks[i], keyexp.u.aes.w[i]);
        }
*/
        uint8_t in[16];
        for (int i = 0; i < 16; i++)
            in[i] = i * 16 + i;

        uint8_t out[16];
        ek_aes_encrypt(&cipher, &keyexp, in, out);

        uint8_t out2[16];
        aes_encrypt(in, out2, ks, 8*keysize);

        printf("US:\n");
        crypto_print_buffer(out, 16);

        printf("TRUTH:\n");
        crypto_print_buffer(out2, 16);

        printf("DECRYPTED:\n");
        uint8_t out3[16];

        aes_decrypt(out, out3, ks, 8*keysize);
        crypto_print_buffer(out3, 16);
    }

    if (1) {
        // aes-128-CBC test
        char *pt = strdup("This is a 48-byte message (exactly 3 AES blocks)");
        int pt_len = 48;
        uint8_t *key;
        int key_len;
        crypto_ascii_to_bytes("6c3ea0477630ce21a2ce334aa746c2cd", &key, &key_len);
        uint8_t *iv;
        int iv_len;
        crypto_ascii_to_bytes("c782dc4c098c66cbd9cd27d825682c81", &iv, &iv_len);
        uint8_t *ct;
        int ct_len;
        crypto_ascii_to_bytes("d0a02b3836451753d493665d33f0e8862dea54cdb293ABC7506939276772f8d5021c19216bad525c8579695d83ba2684", &ct, &ct_len);

        ek_block_cipher_t cipher;
        ek_block_cipher_get("AES128", &cipher);
        ek_block_cipher_keyexp_t keyexp;
        cipher.keyexp(&cipher, key, &keyexp);

        ek_block_cipher_encrypt_cbc(&cipher, &keyexp, iv, pt, pt_len);

/*        printf("TRUTH ciphertext:\n");
        crypto_print_buffer(ct, ct_len);
        printf("US ciphertext:\n");
        crypto_print_buffer(pt, pt_len);

        crypto_print_buffer(iv, iv_len);
*/
        ek_block_cipher_decrypt_cbc(&cipher, &keyexp, iv, pt, pt_len);

//        crypto_print_buffer(pt, pt_len);
        pt[48] = 0;
        printf("%s\n", pt);

        if (1) {
            WORD key_schedule[60];

            aes_key_setup(key, key_schedule, 128);
            char *tmp = malloc(1024);
            aes_decrypt_cbc(ct, ct_len, (void*) tmp, key_schedule, 128, iv);

            printf("decrypted:\n");
            crypto_print_buffer(tmp, ct_len);
            tmp[48] = 0;
            assert(!strcmp(tmp, "This is a 48-byte message (exactly 3 AES blocks)"));
        }
    }

    return 0;
}
