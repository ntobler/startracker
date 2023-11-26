#include "image.h"

static uint8_t img_rpi_data[] = {0,130,117,201,57,42,206,42,57,201,117,130,0,0,1,6,9,11,21,20,21,11,9,6,1,0};

Image_description_t img_rpi = {
    .width = 13,
    .height = 13,
    .data = img_rpi_data
};
