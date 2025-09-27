/*
 * gyro.cpp
 *
 *  Created on: Sep 17, 2025
 *      Author: ntobler
 */

#include "gyro.h"

#include "main.h"

constexpr float SCALE_RAD_S_PER_DIGIT = 1.0;
constexpr float DELTA_T = 0.001;

enum Registers {
    READ = 0x80,
    TEMP_DATA1 = 9,
};

Gyro::Gyro(SPI_HandleTypeDef *hspi)
    : buffer_(),
      write_index_{0},
      read_index_{0},
      hspi_(hspi),
      bias_filters_{{HpFilter{0.99f}, HpFilter{0.99f}, HpFilter{0.99f}}} {}

struct Vec3 {
    float x;
    float y;
    float z;
};

void Gyro::tick() {
    if (write_index_ == read_index_) return;

    DataPayload &payload = buffer_[read_index_];

    Vec3 raw{
        (float)(((uint16_t)payload.accel_x_lower) + (((uint16_t)payload.accel_x_higher) << 8)),
        (float)(((uint16_t)payload.accel_y_lower) + (((uint16_t)payload.accel_y_higher) << 8)),
        (float)(((uint16_t)payload.accel_z_lower) + (((uint16_t)payload.accel_z_higher) << 8)),
    };

    raw.x = bias_filters_[0].process(raw.x) * SCALE_RAD_S_PER_DIGIT;
    raw.y = bias_filters_[0].process(raw.y) * SCALE_RAD_S_PER_DIGIT;
    raw.z = bias_filters_[0].process(raw.z) * SCALE_RAD_S_PER_DIGIT;

    Quaternion q = Quaternion::from_angular_velocities(raw.x, raw.y, raw.z, DELTA_T);
    q_.multiply_right(q);

    read_index_ = (read_index_ + 1) % N_ELEMENTS;
}

void Gyro::fetch_next() {
    DataPayload *payload = &buffer_[write_index_];
    payload->address = Registers::TEMP_DATA1 + Registers::READ;
    HAL_GPIO_WritePin(SPI3_GYRO_CS_GPIO_Port, SPI3_GYRO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hspi_, &(payload->address), &(payload->address),
                                sizeof(DataPayload) - 1);
}

void Gyro::spi_tx_rx_complete_callback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == hspi_->Instance) {
        HAL_GPIO_WritePin(SPI3_GYRO_CS_GPIO_Port, SPI3_GYRO_CS_Pin, GPIO_PIN_SET);
        write_index_ = (write_index_ + 1) % N_ELEMENTS;
    }
}
