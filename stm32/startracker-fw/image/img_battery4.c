#include "image.h"

static uint8_t img_battery4_data[] = {252,252,252,252,252,4,4,4,4,4,4,28,240,7,7,7,7,7,4,4,4,4,4,4,7,1};

Image_description_t img_battery4 = {
    .width = 13,
    .height = 13,
    .data = img_battery4_data
};
