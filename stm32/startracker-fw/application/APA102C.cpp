/*
 * APA102C.cpp
 *
 *  Created on: Nov 24, 2023
 *      Author: ftobler
 */

#include "APA102C.h"

/**
 * brightness goes from 0..7
 */
void writeColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness, GPIO_TypeDef* GPIO_CLK, uint16_t Pin_CLK, GPIO_TypeDef* GPIO_DAT, uint16_t Pin_DAT) {
	for (int i = 0; i < 32; i++) {
		HAL_GPIO_WritePin(GPIO_CLK, Pin_CLK, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIO_CLK, Pin_CLK, GPIO_PIN_RESET);
	}
	brightness |= 0xE0;
	for (int i = 0; i < 8; i++) {
		if (brightness & (0b10000000 >> i)) {
			HAL_GPIO_WritePin(GPIO_DAT, Pin_DAT, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIO_DAT, Pin_DAT, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(GPIO_CLK, Pin_CLK, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIO_CLK, Pin_CLK, GPIO_PIN_RESET);
	}
	for (int i = 0; i < 8; i++) {
		if (blue & (0b10000000 >> i)) {
			HAL_GPIO_WritePin(GPIO_DAT, Pin_DAT, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIO_DAT, Pin_DAT, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(GPIO_CLK, Pin_CLK, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIO_CLK, Pin_CLK, GPIO_PIN_RESET);
	}
	for (int i = 0; i < 8; i++) {
		if (green & (0b10000000 >> i)) {
			HAL_GPIO_WritePin(GPIO_DAT, Pin_DAT, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIO_DAT, Pin_DAT, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(GPIO_CLK, Pin_CLK, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIO_CLK, Pin_CLK, GPIO_PIN_RESET);
	}
	for (int i = 0; i < 8; i++) {
		if (red & (0b10000000 >> i)) {
			HAL_GPIO_WritePin(GPIO_DAT, Pin_DAT, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIO_DAT, Pin_DAT, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(GPIO_CLK, Pin_CLK, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIO_CLK, Pin_CLK, GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(GPIO_DAT, Pin_DAT, GPIO_PIN_RESET);
}
