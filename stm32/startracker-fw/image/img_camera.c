#include "image.h"

static const uint8_t img_camera_data[] ATTR_TO_FLASH = {0,248,8,8,8,8,8,8,8,8,248,224,240,248,0,3,2,2,2,2,2,2,2,2,3,0,1,3};

Image_description_t img_camera = {
    .width = 14,
    .height = 13,
    .data = img_camera_data
};
