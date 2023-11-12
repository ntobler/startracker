/*
 * ssd1306_write_float.cpp
 *
 *  Created on: Nov 12, 2023
 *      Author: ftobler
 */


#include "ssd1306_write_float.h"
#include "stdio.h"
#include "math.h"

void ssd1306_write_float(float f, int digits, FontDef font) {
	char buf[25];
	float_to_str(buf, f, digits);
	ssd1306_WriteString(buf, font);
}

void ssd1306_write_int(int num, FontDef font) {
	char buf[25];
	sprintf(buf, "%d", num);
	ssd1306_WriteString(buf, font);
}

void float_to_str(char* buffer, float num, int precision) {
    int intPart = (int)num;  // Extract integer part
    int fracPart = abs((int)((num - intPart) * pow(10, precision)));  // Extract fractional part

    // Convert integer part to string
    sprintf(buffer, "%d.", intPart);

    // Convert fractional part to string
    char fracBuffer[20];  // Adjust the size accordingly
    sprintf(fracBuffer, "%d", fracPart);

    // Append fractional part to the buffer with proper precision
    int fracLen = strlen(fracBuffer);
    int i;
    for (i = 0; i < precision - fracLen; ++i) {
        strcat(buffer, "0");
    }
    strcat(buffer, fracBuffer);
}
