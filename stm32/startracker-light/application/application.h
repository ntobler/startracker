#ifndef APPLICATION_H_
#define APPLICATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ARM_MATH_DSP
#error "please define ARM_MATH_DSP to accelerate floating point math."
#endif

#include "stdint.h"
#include "stm32_hal.h"

void app_init(SPI_HandleTypeDef *led_spi, SPI_HandleTypeDef *gyro_spi,
              UART_HandleTypeDef *rpi_uart);
void app_systick_isr();
void app_gpio_isr();
void app_usasrt1_irq_handler();

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_H_ */
