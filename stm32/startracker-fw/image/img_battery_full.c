#include "image.h"

static uint8_t img_battery_full_data[] = {252,12,172,236,60,252,60,252,12,252,12,252,240,7,6,7,7,6,6,6,7,6,7,6,7,1};

Image_description_t img_battery_full = {
    .width = 13,
    .height = 13,
    .data = img_battery_full_data
};
