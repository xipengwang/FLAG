#ifndef _IMAGE_U32
#define _IMAGE_U32

#include <stdint.h>
#include "common/image_types.h"

// NB: Please do not use this to store images. Use u8x3 instead!
// Otherwise, you'll end up in endian hell.
image_u32_t *image_u32_create_stride(int width, int height, int stride);

image_u32_t *image_u32_create(int width, int height);

void image_u32_destroy(image_u32_t *im);


#endif
