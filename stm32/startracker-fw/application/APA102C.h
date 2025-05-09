/*
 * APA102C.h
 *
 *  Created on: Nov 24, 2023
 *      Author: ftobler
 */

#ifndef APA102C_H_
#define APA102C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32_hal.h"

void writeColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness, GPIO_TypeDef* GPIO_CLK, uint16_t Pin_CLK, GPIO_TypeDef* GPIO_DAT, uint16_t Pin_DAT);

#ifdef __cplusplus
}
#endif

#endif /* APA102C_H_ */
