#include "image.h"

static const uint8_t img_blank_data[] ATTR_TO_FLASH = {0,0,0,0,0,0,0,0,0,0,0,0,0};

Image_description_t img_blank = {
    .width = 13,
    .height = 8,
    .data = img_blank_data
};
