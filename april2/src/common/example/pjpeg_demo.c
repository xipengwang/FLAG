#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/pjpeg.h"

int main(int argc, char *argv[])
{
    for (int argi = 1; argi < argc; argi++) {
        const char *path = argv[argi];

        printf("processing %s\n", path);

        int err;
        pjpeg_t *pjpeg = pjpeg_create_from_file(path, PJPEG_STRICT, &err);
        if (err) {
            printf("error %d\n", err);
            continue;
        }

        if (1) {
            image_u8x3_t *im = pjpeg_to_u8x3_baseline(pjpeg);

            char *outpath = malloc(strlen(path) + 128);
            sprintf(outpath, "%s.pnm", path);
            image_u8x3_write_pnm(im, outpath);
            image_u8x3_destroy(im);
            free(outpath);
        }


        if (1) {
            image_u8_t *im = pjpeg_to_u8_baseline(pjpeg);

            char *outpath = malloc(strlen(path) + 128);
            sprintf(outpath, "%s.8.pnm", path);
            image_u8_write_pnm(im, outpath);
            image_u8_destroy(im);
            free(outpath);
        }
    }

    return 0;

}
