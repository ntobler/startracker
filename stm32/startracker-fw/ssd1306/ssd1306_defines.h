/*
 * ssd1306_defines.h
 *
 *  Created on: 14/04/2018
 *  Update on: 10/04/2019
 *      Author: Andriy Honcharenko
 *      version: 2
 *
 *  Modify on: 06/11/2021
 *      Author: Roberto Benjami
 */

#ifndef SSD1306_DEFINES_H_
#define SSD1306_DEFINES_H_

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1306_I2C_PORT  hi2c3   // I2C port as defined in main generated by CubeMx (hi2c1 or hi2c2 or hi2c3)
#define SSD1306_ADDRESS    0x3C   // I2C address display
#define SSD1306_128X64            // SSD1306_128X32 or SSD1306_128X64
#define SSD1306_USE_DMA       1   // 0: not used I2C DMA mode, 1: used I2C DMA mode
#define SSD1306_CONTUPDATE    0   // 0: continue update mode disable, 1: continue update mode enable (only DMA MODE)

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_DEFINES_H_ */
