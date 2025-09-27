/*
 * ledring.cpp
 *
 *  Created on: Sep 16, 2025
 *      Author: ntobler
 *
 *
 *  Drives LEDs with the TI LP5868 LED driver
 *
 */

#include "ledring.h"

#include <algorithm>
#include <numbers>

#include "arm_math.h"
#include "main.h"

enum Registers {
    CHIP_EN = 0x00,
    DC_CURRENT_8BIT = 0x0100,
    PWM_8BIT = 0x0200,
};

static void disable_led(uint8_t *dst, uint8_t cs_index, uint8_t sw_index) {
    uint8_t byte_index = sw_index * 3 + cs_index / 8;
    uint8_t bit_index = cs_index & 7;
    dst[byte_index] &= ~(0x01 << bit_index);
}

LedRing::LedRing(SPI_HandleTypeDef *hspi)
    : buffer_(), payload_(&buffer_[N_ADDRESS_BYTES]), hspi_(hspi), t_(0) {
    // Disable unpopulated leds
    // Enable all leds
    std::fill_n(payload_, N_LEDS / 8, 0xff);
    disable_led(payload_, 14, 0);
    disable_led(payload_, 15, 2);
    disable_led(payload_, 15, 3);
    disable_led(payload_, 15, 4);
    disable_led(payload_, 15, 5);
    disable_led(payload_, 15, 6);
    disable_led(payload_, 15, 7);
    send(Registers::CHIP_EN, 1);

    // Enable chip
    payload_[Registers::CHIP_EN] = 0x01;
    send(Registers::CHIP_EN, 1);
}

static size_t correct_index(size_t x) {
    size_t cs_index = 17 - (x / 8);
    size_t sw_index = x % 8;
    return sw_index * 18 + cs_index;
}

void LedRing::tick() {
    t_ += 0.01f;
    if (t_ > 1.0 - 0.01 / 2.0) {
        t_ = 0;
    }

    for (size_t i = 0; i < N_LEDS; i++) {
        float pos = (float)i;
        float value = arm_sin_f32(((pos / 20.0f) + t_) * 2.0f * std::numbers::pi) * 100.0f + 100.0f;
        size_t index = correct_index(i);
        payload_[index] = (uint8_t)value;
    }

    send(Registers::PWM_8BIT, N_LEDS);
}

void LedRing::send(uint16_t address, uint16_t size) {
    buffer_[0] = (uint8_t)address >> 2;
    buffer_[1] = (uint8_t)address << 6;
    while (hspi_->State != HAL_SPI_STATE_READY)
        ;
    HAL_GPIO_WritePin(SPI1_LED_CS_GPIO_Port, SPI1_LED_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(hspi_, &buffer_[0], N_ADDRESS_BYTES + size);
}

void LedRing::spi_tx_complete_callback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == hspi_->Instance) {
        HAL_GPIO_WritePin(SPI1_LED_CS_GPIO_Port, SPI1_LED_CS_Pin, GPIO_PIN_SET);
    }
}
