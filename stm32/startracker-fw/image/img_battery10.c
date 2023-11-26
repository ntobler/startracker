#include "image.h"

static uint8_t img_battery10_data[] = {252,252,252,252,252,252,252,252,252,252,252,252,240,7,7,7,7,7,7,7,7,7,7,7,7,1};

Image_description_t img_battery10 = {
    .width = 13,
    .height = 13,
    .data = img_battery10_data
};
