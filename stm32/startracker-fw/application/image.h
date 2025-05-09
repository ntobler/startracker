/*
 * image.h
 *
 *  Created on: Nov 12, 2023
 *      Author: ftobler
 */

#ifndef IMAGE_H_
#define IMAGE_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define ATTR_TO_FLASH __attribute__((section(".rodata")))

typedef struct {
	uint8_t width;
	uint8_t height;
	const uint8_t* data;
} Image_description_t;


void ssd1306_DrawImage(uint8_t x, uint8_t y, Image_description_t* image);


#ifdef __cplusplus
}
#endif


#endif /* IMAGE_H_ */
