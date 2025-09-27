/*
 * ledring.h
 *
 *  Created on: Sep 16, 2025
 *      Author: ntobler
 */

#ifndef LEDRING_H_
#define LEDRING_H_

#include <stdint.h>

#include <array>
#include <memory>

#include "stm32_hal.h"

enum {
    N_ADDRESS_BYTES = 2,
    N_LEDS = 144,
};

enum LedMode {
    OFF,
    SINE,
    AMPLITUDE_SINE,
    SEARCH,
    PROGRESS,
    DOT,
    LEVEL_XY,
    LEVEL_XY2,
    MATRIX,
    N_MODES,
};

class LedRing {
   private:
    std::array<uint8_t, N_ADDRESS_BYTES + N_LEDS> buffer_;
    uint8_t *payload_;
    SPI_HandleTypeDef *hspi_;
    uint16_t t_;
    bool tx_busy_;
    float pos_x_;
    float pos_y_;
    float pos_z_;
    float progress_;
    LedMode mode_;

    void send(uint16_t address, uint16_t size);
    void wait_tx(void);
    void read(uint16_t address, uint16_t len);

   public:
    LedRing(SPI_HandleTypeDef *hspi);
    void start();
    void tick();
    void set_pos(float x, float y, float z) {
        pos_x_ = x;
        pos_y_ = y;
        pos_z_ = z;
    };
    void set_progress(float p) { progress_ = p; }
    void set_mode(LedMode mode) { mode_ = mode; }
    LedMode get_mode() const { return mode_; }
    void spi_tx_complete_callback(SPI_HandleTypeDef *hspi);
};

#endif /* LEDRING_H_ */
