#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "common/pam.h"

int main(int argc, char *argv[])
{
    if (argc != 4) {
        printf("usage: %s <rgba.pam> <alpha.pam> <out.pam>\n", argv[0]);
        return -1;
    }

    pam_t *rgb = pam_create_from_file(argv[1]);
    pam_t *alpha = pam_create_from_file(argv[2]);

    printf("%d\n", rgb->type);
    assert(rgb->type == PAM_RGB || rgb->type == PAM_RGB_ALPHA);

    assert(rgb->width == alpha->width && rgb->height == alpha->height);

    pam_t *rgba = calloc(1, sizeof(pam_t));
    rgba->type = PAM_RGB_ALPHA;
    rgba->width = rgb->width;
    rgba->height = rgb->height;
    rgba->depth = 4;
    rgba->maxval = 255;

    rgba->datalen = 4 * rgba->width * rgba->height;
    rgba->data = malloc(rgba->datalen);

    for (int y = 0; y < rgba->height; y++) {
        for (int x = 0; x < rgba->width; x++){
            for (int c = 0; c < 3; c++)
                rgba->data[y*4*rgba->width + 4*x + c] = rgb->data[y*rgb->depth*rgba->width + rgb->depth*x + c];

            // use the last channel from the alpha
            rgba->data[y*4*rgba->width + 4*x + 3] = alpha->data[y*alpha->depth*rgba->width + alpha->depth*x + alpha->depth - 1];
        }
    }
    pam_write_file(rgba, argv[3]);
    return 0;
}
