#include "image.h"

static const uint8_t img_power_data[] ATTR_TO_FLASH = {96,96,96,96,240,8,4,4,4,4,4,252,144,144,144,0,0,0,0,0,1,2,2,2,2,2,3,0,0,0};

Image_description_t img_power = {
    .width = 15,
    .height = 13,
    .data = img_power_data
};
