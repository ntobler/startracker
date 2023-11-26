#include "image.h"

static uint8_t img_motor_data[] = {240,12,2,250,17,33,33,17,250,2,12,240,0,0,3,4,5,8,8,8,8,5,4,3,0,0};

Image_description_t img_motor = {
    .width = 13,
    .height = 13,
    .data = img_motor_data
};
