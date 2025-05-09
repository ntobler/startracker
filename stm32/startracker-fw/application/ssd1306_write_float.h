/*
 * ssd1306_write_float.h
 *
 *  Created on: Nov 12, 2023
 *      Author: ftobler
 */

#ifndef SSD1306_WRITE_FLOAT_H_
#define SSD1306_WRITE_FLOAT_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "ssd1306.h"

void ssd1306_write_float(float f, int digits, FontDef font);
void ssd1306_write_int(int num, FontDef font);
void float_to_str(char* buffer, float num, int precision);

#ifdef __cplusplus
}
#endif


#endif /* SSD1306_WRITE_FLOAT_H_ */
