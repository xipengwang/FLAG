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
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include "../ek.h"

void hmac_test()
{
    struct testcase {
        char *keys;
        char *datas;
        char *truths;
    };

    struct testcase *testcases = (struct testcase[]) {
        { .keys   = "0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b",
          .datas  = "4869205468657265",
          .truths = "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7" },

        { .keys   = "4a656665",
          .datas  = "7768617420646f2079612077616e7420 666f72206e6f7468696e673f",
          .truths = "5bdcc146bf60754e6a042426089575c7 5a003f089d2739839dec58b964ec3843", },

        { .keys   = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaaaa",
          .datas  = "dddddddddddddddddddddddddddddddd dddddddddddddddddddddddddddddddd dddddddddddddddddddddddddddddddd dddd",
          .truths = "773ea91e36800e46854db8ebd09181a7 2959098b3ef8c122d9635514ced565fe", },

        { .keys   = "0102030405060708090a0b0c0d0e0f10 111213141516171819",
          .datas  = "cdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcd cdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcd cdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcd cdcd",
          .truths = "82558a389a443c0ea4cc819899f2083a 85f0faa3e578f8077a2e3ff46729665b", },

        { .keys   = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaa",
          .datas  = "54686973206973206120746573742075 73696e672061206c6172676572207468  616e20626c6f636b2d73697a65206b65 7920616e642061206c61726765722074 68616e20626c6f636b2d73697a652064 6174612e20546865206b6579206e6565 647320746f2062652068617368656420 6265666f7265206265696e6720757365 642062792074686520484d414320616c 676f726974686d2e",
          .truths = "9b09ffa71b942fcb27635fbcd5b0e944 bfdc63644f0713938a7f51535c3a35e2" },
        { .keys = NULL },

    };

    ek_hash_algorithm_t alg;
    if (ek_hash_algorithm_get("SHA256", &alg))
        exit(-1);

    for (int i = 0; testcases[i].keys != NULL; i++) {
        uint8_t *key;
        int key_len;
        ek_ascii_to_bytes(testcases[i].keys, &key, &key_len);
        uint8_t *data;
        int data_len;
        ek_ascii_to_bytes(testcases[i].datas, &data, &data_len);

        uint8_t *truehmac;
        int truehmac_len;
        ek_ascii_to_bytes(testcases[i].truths, &truehmac, &truehmac_len);

        uint8_t hmac[alg.digest_size];
        ek_hash_hmac(&alg, key, key_len, data, data_len, hmac);

        if (memcmp(truehmac, hmac, truehmac_len)) {
            printf("fail %d\n", i);

            ek_print_buffer(truehmac, truehmac_len);
            ek_print_buffer(hmac, 32);

            exit(-1);
        }
    }
}

void hash_test()
{
    struct testcase {
        char *alg;
        char *data;
        char *hashs;
    };

    struct testcase *testcases = (struct testcase[]) {
        { .alg = "SHA1",
          .data = "",
          .hashs = "da39a3ee5e6b4b0d3255bfef95601890afd80709" },
        { .alg = "SHA1",
          .data = "a",
          .hashs = "86f7e437faa5a7fce15d1ddcb9eaeaea377667b8" },
        { .alg = "SHA1",
          .data = "The quick brown fox jumps over the lazy dog",
          .hashs = "2fd4e1c67a2d28fced849ee1bb76e7391b93eb12" },
        { .alg = "SHA1",
          .data = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnop",
          .hashs = "47b172810795699fe739197d1a1f5960700242f1" },
        { .alg = "SHA1",
          .data = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
          .hashs = "84983e44 1c3bd26e baae4aa1 f95129e5 e54670f1" },
        { .alg = "SHA1",
          .data = "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu",
          .hashs = "a49b2446 a02c645b f419f995 b6709125 3a04a259" },

        { .alg = "SHA256",
          .data = "",
          .hashs = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855" },
        { .alg = "SHA256",
          .data = "a",
          .hashs = "ca978112ca1bbdcafac231b39a23dc4da786eff8147c4e72b9807785afee48bb" },
        { .alg = "SHA256",
          .data = "The quick brown fox jumps over the lazy dog",
          .hashs = "d7a8fbb307d7809469ca9abcb0082e4f8d5651e46d3cdb762d02d0bf37c9e592" },
        { .alg = "SHA256",
          .data = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnop",
          .hashs = "aa353e009edbaebfc6e494c8d847696896cb8b398e0173a4b5c1b636292d87c7" },
        { .alg = "SHA256",
          .data = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
          .hashs = "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1" },
        { .alg = "SHA256",
          .data = "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu",
          .hashs = "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1" },
        { .alg = "MD5",
          .data = "",
          .hashs = "d41d8cd98f00b204e9800998ecf8427e" },
        { .alg = "MD5",
          .data = "The quick brown fox jumps over the lazy dog",
          .hashs = "9e107d9d372bb6826bd81d3542a419d6" },
        { .alg = "MD5",
          .data = "The quick brown fox jumps over the lazy dog.",
          .hashs = "e4d909c290d0fb1ca068ffaddf22cbd0" },
        { .alg = NULL }
    };

    for (int i = 0; testcases[i].alg != NULL; i++) {
        ek_hash_algorithm_t alg;
        if (ek_hash_algorithm_get(testcases[i].alg, &alg))
            assert(0);

        uint8_t *hash;
        int hash_len;
        ek_ascii_to_bytes(testcases[i].hashs, &hash, &hash_len);
        assert(hash_len == alg.digest_size);

        ek_hash_state_t state;
        uint8_t digest[alg.digest_size];

        alg.init(&state);
        alg.update(&state, testcases[i].data, strlen(testcases[i].data));
        alg.final(&state, digest);

        if (memcmp(hash, digest, hash_len)) {
            printf("fail: %s(\"%s\")\n", testcases[i].alg, testcases[i].data);
            ek_print_buffer(hash, hash_len);
            ek_print_buffer(digest, hash_len);
            exit(-1);
        }
    }
}

int main(int argc, char *argv[])
{
    hash_test();
    printf("hash ok\n");
    hmac_test();
    printf("hmac ok\n");
}
