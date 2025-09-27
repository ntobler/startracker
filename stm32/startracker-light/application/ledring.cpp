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
    CHIP_EN = 0x000,
    DOT_ONOFF = 0x043,
    DOT_LOD = 0x065,
    DC_CURRENT_8BIT = 0x100,
    PWM_8BIT = 0x200,
};

static void disable_led(uint8_t *dst, uint8_t cs_index, uint8_t sw_index) {
    uint8_t byte_index = sw_index * 3 + cs_index / 8;
    uint8_t bit_index = cs_index & 7;
    dst[byte_index] &= ~(0x01 << bit_index);
}

void LedRing::read(uint16_t address, uint16_t len) {
    uint8_t data[512];
    data[0] = (uint8_t)(address >> 2);
    data[1] = (uint8_t)(address << 6);
    while ((hspi_->State != HAL_SPI_STATE_READY) || tx_busy_) {
        // spin until DMA transfer is done
        __NOP();
    }
    HAL_GPIO_WritePin(SPI1_LED_CS_GPIO_Port, SPI1_LED_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi_, data, data, len, 1000);
    HAL_GPIO_WritePin(SPI1_LED_CS_GPIO_Port, SPI1_LED_CS_Pin, GPIO_PIN_SET);
    __NOP();
}

LedRing::LedRing(SPI_HandleTypeDef *hspi)
    : buffer_(),
      payload_(&buffer_[N_ADDRESS_BYTES]),
      hspi_(hspi),
      t_(0),
      tx_busy_{false},
      pos_x_{0},
      pos_y_{0},
      pos_z_{0},
      progress_{0},
      mode_{LedMode::LEVEL_XY} {}

void LedRing::start() {
    // Set CS high
    HAL_GPIO_WritePin(SPI1_LED_CS_GPIO_Port, SPI1_LED_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_VSYNC_GPIO_Port, LED_VSYNC_Pin, GPIO_PIN_RESET);

    // Disable chip
    payload_[0] = 0x01;
    payload_[1] = (8 << 3) | (0 << 1);
    send(Registers::CHIP_EN, 2);
    wait_tx();

    // Disable unpopulated leds
    // Enable all leds
    std::fill_n(payload_, 24, 0xff);
    disable_led(payload_, 14, 0);
    disable_led(payload_, 15, 2);
    disable_led(payload_, 15, 3);
    disable_led(payload_, 15, 4);
    disable_led(payload_, 15, 5);
    disable_led(payload_, 15, 6);
    disable_led(payload_, 15, 7);
    send(Registers::DOT_ONOFF, 24);
    wait_tx();

    std::fill_n(payload_, N_LEDS, 0xA0);
    send(Registers::DC_CURRENT_8BIT, N_LEDS);
    wait_tx();

    std::fill_n(payload_, N_LEDS, 0x01);
    send(Registers::PWM_8BIT, N_LEDS);
    wait_tx();

    // Enable chip
    payload_[0] = 0x01;
    payload_[1] = (8 << 3) | (0 << 1);
    send(Registers::CHIP_EN, 2);
    wait_tx();
}

static size_t correct_index(size_t x) {
    size_t cs_index = 17 - (x / 8);
    size_t sw_index = x % 8;
    size_t index = sw_index * 18 + cs_index;
    if (index >= N_LEDS) {
        while (1)
            ;
    }
    return index;
}

constexpr float two_pi = 2.0 * std::numbers::pi;

void LedRing::tick() {
    t_ += 10;
    if (t_ >= 10000) {
        t_ = 0;
    }
    float t_float = ((float)t_) * 0.001f;

    constexpr float n_dots = 10.0f;

    const uint64_t data[] = {
        0b1000010001111110001000000001000000011110, 0b1000010001000000001000000001000000100001,
        0b1111110001111110001000000001000000100001, 0b1000010001000000001000000001000000100001,
        0b1000010001111110001111110001111110011110,
    };

    wait_tx();

    switch (mode_) {
        default:
        case OFF:
            std::fill_n(payload_, N_LEDS, 0x00);
            break;
        case SINE: {
            for (size_t i = 0; i < N_LEDS; i++) {
                float pos = ((float)i) * (1.0f / 144.0f);
                float angle = ((pos * n_dots) + (t_float * 0.5f)) * two_pi;
                float value = arm_sin_f32(fmodf(angle, two_pi)) * 0.5f + 0.5f;
                value = value * value * value * 255.0f;
                size_t index = correct_index(i);
                payload_[index] = ((uint8_t)value);
            }
        } break;
        case AMPLITUDE_SINE: {
            float angle = t_float * (0.5f * two_pi);
            float amplitude = arm_sin_f32(fmodf(angle, two_pi));
            for (size_t i = 0; i < N_LEDS; i++) {
                float pos = ((float)i) * (1.0f / 144.0f);
                float angle = (pos * n_dots) * two_pi;
                float value = arm_sin_f32(fmodf(angle, two_pi)) * 0.5f + 0.5f;
                value = std::max(0.0f, value * amplitude);
                value = value * value * value * 255.0f;
                size_t index = correct_index(i);
                payload_[index] = ((uint8_t)value);
            }
        } break;
        case DOT: {
            for (size_t i = 0; i < N_LEDS; i++) {
                float pos = ((float)i) * (n_dots / 144.0f);
                float angle = (pos + (t_float * 0.1f));
                float value = fmodf(angle, 1.0f);
                value = std::min(std::min(value, 1.0f - value) * (144.0f / n_dots), 1.0f);
                value = (1.0f - value);
                value = value * value * 255.0f;
                size_t index = correct_index(i);
                payload_[index] = ((uint8_t)value);
            }
        } break;
        case SEARCH: {
            float angle = t_float * (0.5f * two_pi);
            float x = arm_sin_f32(fmodf(angle, two_pi)) * 10.0f;

            for (size_t i = 0; i < N_LEDS; i++) {
                float pos;
                if (i < N_LEDS / 8 * 1) {
                    pos = 0.0;
                } else if (i < N_LEDS / 8 * 3) {
                    pos = ((float)(N_LEDS / 4));
                } else if (i < N_LEDS / 8 * 5) {
                    pos = ((float)(N_LEDS / 2));
                } else if (i < N_LEDS / 8 * 7) {
                    pos = ((float)((N_LEDS / 4) * 3));
                } else {
                    pos = ((float)N_LEDS);
                }

                float i_f = ((float)i);

                float pos1 = pos + x;
                float pos2 = pos - x;

                float value = std::min(std::abs(pos1 - i_f), std::abs(pos2 - i_f));
                value = std::min(value, 1.0f);
                value = (1.0f - value);
                value = value * value * 255.0f;
                size_t index = correct_index(i);
                payload_[index] = ((uint8_t)value);
            }
        } break;
        case PROGRESS: {
            float p = (progress_ * (((N_LEDS / 4) + 1) * 0.5)) - 1.0f;
            for (size_t i = 0; i < N_LEDS; i++) {
                float pos;
                if (i < N_LEDS / 8 * 1) {
                    pos = 0.0;
                } else if (i < N_LEDS / 8 * 3) {
                    pos = ((float)(N_LEDS / 4));
                } else if (i < N_LEDS / 8 * 5) {
                    pos = ((float)(N_LEDS / 2));
                } else if (i < N_LEDS / 8 * 7) {
                    pos = ((float)((N_LEDS / 4) * 3));
                } else {
                    pos = ((float)N_LEDS);
                }

                float i_f = ((float)i);
                float value = std::abs(pos - i_f) - p;
                value = std::min(std::max(value, 0.0f), 1.0f);
                value = (1.0f - value);
                value = value * value * 150.0f;
                size_t index = correct_index(i);
                payload_[index] = ((uint8_t)value);
            }
        } break;
        case LEVEL_XY: {
            float positions[4];
            float x = pos_x_ * 10.0f;
            float y = pos_y_ * 10.0f;
            positions[0] = x;
            positions[1] = ((float)(N_LEDS / 4)) + y;
            positions[2] = ((float)(N_LEDS / 2)) - x;
            positions[3] = ((float)((N_LEDS / 4) * 3)) - y;
            for (size_t i = 0; i < N_LEDS; i++) {
                float pos;
                float i_f = ((float)i);
                if (i < N_LEDS / 8 * 1) {
                    pos = positions[0];
                } else if (i < N_LEDS / 8 * 3) {
                    pos = positions[1];
                } else if (i < N_LEDS / 8 * 5) {
                    pos = positions[2];
                } else if (i < N_LEDS / 8 * 7) {
                    pos = positions[3];
                } else {
                    pos = positions[0];
                    i_f -= ((float)N_LEDS);
                }

                float value = std::abs(pos - i_f);
                value = std::min(value, 1.0f);
                value = (1.0f - value);
                value = value * value * 255.0f;
                size_t index = correct_index(i);
                payload_[index] = ((uint8_t)value);
            }
        } break;
        case LEVEL_XY2: {
            float x = pos_x_ * 0.5f;
            float y = -pos_y_ * 0.5f;
            for (size_t i = 0; i < N_LEDS; i++) {
                constexpr float f1 = 12.0f / 18.0f;
                constexpr float f2 = 6.0f / 18.0f;

                float ix = std::min(std::abs((int)((N_LEDS / 4 * 3) - i)),
                                    std::abs((int)((-(N_LEDS / 4) * 1) - i))) -
                           N_LEDS / 4;
                float iy = std::abs((int)((N_LEDS / 2) - i)) - N_LEDS / 4;

                ix = std::min(std::max(ix, -N_LEDS / 8 * f1), N_LEDS / 8 * f1);
                iy = std::min(std::max(iy * f2, -N_LEDS / 8 * f1), N_LEDS / 8 * f1);

                float value = ix * x + iy * y;
                value = std::min(std::max(value, 0.0f), 1.0f);
                value = value * 255.0f;
                size_t index = correct_index(i);
                payload_[index] = ((uint8_t)value);
            }
        } break;
        case MATRIX: {
            float pos = (-pos_z_ * 20.0f) + 2.5f;
            if ((pos < 0.0f) || (pos >= 4.999f)) {
                std::fill_n(payload_, N_LEDS, 0x00);
            } else {
                size_t z_index = (size_t)pos;
                uint64_t line = data[z_index];
                for (size_t i = 0; i < N_LEDS; i++) {
                    size_t index = correct_index(i);
                    if ((i >= 80) && (i < 80 + 40)) {
                        payload_[index] = line & (1 << (i - 80)) ? 255 : 0;
                    } else {
                        payload_[index] = 0;
                    }
                }
            }
        } break;
    }

    send(Registers::PWM_8BIT, N_LEDS);
}

void LedRing::wait_tx(void) {
    while ((hspi_->State != HAL_SPI_STATE_READY) || tx_busy_)
        ;
}

void LedRing::send(uint16_t address, uint16_t size) {
    buffer_[0] = (uint8_t)(address >> 2);
    buffer_[1] = (uint8_t)(address << 6) | 0x20;
    while ((hspi_->State != HAL_SPI_STATE_READY) || tx_busy_)
        ;
    tx_busy_ = true;
    HAL_GPIO_WritePin(SPI1_LED_CS_GPIO_Port, SPI1_LED_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(hspi_, &buffer_[0], N_ADDRESS_BYTES + size);
}

void LedRing::spi_tx_complete_callback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == hspi_->Instance) {
        HAL_GPIO_WritePin(SPI1_LED_CS_GPIO_Port, SPI1_LED_CS_Pin, GPIO_PIN_SET);
        tx_busy_ = false;
    }
}
