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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

#include "common/arith_encoder.h"

// this test encodes only 2 symbols, manipulating the probabilities
// and modeled probabilities over many orders of magnitude in order to
// conjure up failures.
void trial(int seed)
{
    srandom(seed);

    int symslen = seed < 2 ? 32768 : random() & 0x1ffff;
    int *syms = calloc(sizeof(int), symslen);

    switch (seed) {
        case 0:
            for (int i = 0; i < symslen; i++)
                syms[i] = 0;
            break;

        case 1:
            for (int i = 0; i < symslen; i++)
                syms[i] = 1;
            break;

        default: {
            uint32_t mask = 0xffffffff;
            uint32_t thresh = random() & mask;

            for (int i = 0; i < symslen; i++)
                syms[i] = (random() & mask) < thresh;

            break;
        }
    }

    uint32_t x = (random() & 0x1fffffff);
    uint32_t chist[] = { 0, 1, 2+x, 2+x+1 };

    uint8_t *compressed_data;
    int compressed_data_len;

    if (1) {
        arith_model_t *model = arith_model_fixed_create(chist, 3);
        arith_encoder_t *ae = arith_encoder_create();

        int N[model->nsyms];
        memset(N, 0, sizeof(N));

        for (int i = 0; i < symslen; i++) {
            N[syms[i]] ++;
            arith_encoder_write(ae, model, syms[i]);
            model->update(model, syms[i]);
        }
        arith_encoder_write(ae, model, 2); // EOF
        arith_encoder_finish(ae, &compressed_data, &compressed_data_len);

        double predicted_bits = 0;

        for (int i = 0; i < model->nsyms; i++) {
            // how many bits to encode a 0?
            double bps = -log((chist[i+1] - chist[i])*1.0 / chist[model->nsyms]) / log(2);
            predicted_bits += N[i] * bps;
        }

        // actual encoded size should be close to predicted
        // size. Empirically, actual size is usually between [1, 1.02]
        // times the predicted size.
        printf("seed %8d: compressed %8d symbols ==> %8d bytes [ %8d predicted ](%8.2f %%)\n",
               seed, symslen, compressed_data_len, (int) ceil(predicted_bits / 8.0), compressed_data_len*100.0 / (symslen / 8.0));

        arith_encoder_destroy(ae);
        model->destroy(model);
    }

    if (1) {
        arith_model_t *model = arith_model_fixed_create(chist, 3);
        arith_decoder_t *ad = arith_decoder_create(compressed_data, compressed_data_len);

        int bad = 0;

        int readpos;
        for (readpos = 0; 1; readpos++) {
            assert(readpos < symslen + 1); // remember EOF

            int sym = arith_decoder_read(ad, model);
            if (sym == 2)
                break;

            model->update(model, syms[readpos]);

            if (sym != syms[readpos]) {
                bad++;
                printf("BAD (%d, expected %d, got %d)\n", readpos, syms[readpos], sym);
            }
        }

        if (bad) {
            printf("BAD COUNT: %d\n", bad);
            exit(-1);
        }

        assert(readpos == symslen);
        arith_decoder_destroy(ad);
        model->destroy(model);
    }
    free(compressed_data);
    free(syms);
}

void do_file(char *path)
{
    printf("%-80s: ", path);
    fflush(NULL);

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        printf("open failed\n");
        return;
    }

    fseek(f, 0, SEEK_END);
    long symslen = ftell(f);
    if (symslen > 1E9) {
        printf("too big\n");
        fclose(f);
        return;
    }

    fseek(f, 0, SEEK_SET);
    uint8_t *syms = calloc(1, symslen);
    if (symslen != fread(syms, 1, symslen, f)) {
        printf("read err\n");
        return;
    }

    uint8_t *compressed_data;
    int compressed_data_len;

    if (1) {
        arith_model_t *model = arith_model_order0_create(257);
        arith_encoder_t *ae = arith_encoder_create();
        for (int i = 0; i < symslen; i++) {
            arith_encoder_write(ae, model, syms[i]);
            model->update(model, syms[i]);
        }

        arith_encoder_write(ae, model, 256); // EOF
        arith_encoder_finish(ae, &compressed_data, &compressed_data_len);

        printf("%8d ==> %8d (%8.2f %%)\n",
               (int) symslen, (int) compressed_data_len, compressed_data_len * 100.0 / symslen);

        arith_encoder_destroy(ae);
        model->destroy(model);
    }

    if (1) {
        arith_model_t *model = arith_model_order0_create(257);
        arith_decoder_t *ad = arith_decoder_create(compressed_data, compressed_data_len);

        int bad = 0;
        int readpos;
        for (readpos = 0; 1; readpos++) {
            assert(readpos < symslen + 1); // remember EOF

            int sym = arith_decoder_read(ad, model);
            if (sym == 256)
                break;

            model->update(model, sym);

            if (sym != syms[readpos]) {
                bad++;
                printf("BAD (%d, expected %d, got %d)\n", readpos, syms[readpos], sym);
            }
        }

        if (bad) {
            printf("BAD COUNT: %d\n", bad);
            exit(-1);
        }

        assert(readpos == symslen);
        model->destroy(model);
    }

    fclose(f);
    free(compressed_data);
    free(syms);
}



void recurse(char *path)
{
    struct stat st;
    stat(path, &st);

    if (S_ISDIR(st.st_mode)) {
        DIR *dir = opendir(path);
        if (dir == NULL)
            return;

        struct dirent *dirent;
        while ((dirent = readdir(dir)) != NULL) {
            char buf[strlen(path) + 257];
            if (!strcmp(dirent->d_name, ".") || !strcmp(dirent->d_name, ".."))
                continue;

            sprintf(buf, "%s/%s", path, dirent->d_name);
            recurse(buf);
        }
        closedir(dir);
    }

    if (S_ISREG(st.st_mode)) {
        do_file(path);
    }
}

int main(int argc, char *argv[])
{
    for (int argi = 1; argi < argc; argi++) {
        recurse(argv[argi]);
    }

    if (argc == 1) {
        for (int iter = 0; 1; iter++)
            trial(iter);
    }
}
