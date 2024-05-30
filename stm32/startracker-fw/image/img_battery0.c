#include "image.h"

static const uint8_t img_battery0_data[] ATTR_TO_FLASH = {252,4,4,4,4,4,4,4,4,4,4,28,240,7,4,4,4,4,4,4,4,4,4,4,7,1};

Image_description_t img_battery0 = {
    .width = 13,
    .height = 13,
    .data = img_battery0_data
};
