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

class LedRing {
   private:
    std::array<uint8_t, N_ADDRESS_BYTES + N_LEDS> buffer_;
    uint8_t *payload_;
    SPI_HandleTypeDef *hspi_;
    float t_;

    void send(uint16_t address, uint16_t size);

   public:
    LedRing(SPI_HandleTypeDef *hspi);
    void tick();
    void spi_tx_complete_callback(SPI_HandleTypeDef *hspi);
};

#endif /* LEDRING_H_ */
