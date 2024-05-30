/*
 * image.c
 *
 *  Created on: Nov 12, 2023
 *      Author: ftobler
 */


#include "image.h"
#include "ssd1306.h"



void ssd1306_DrawImage(uint8_t x, uint8_t y, Image_description_t* image) {
	ssd1306_DrawBitmap(x, y, image->width, image->height, image->data);
}
