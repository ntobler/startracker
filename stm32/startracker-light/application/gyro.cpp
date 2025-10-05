/*
 * gyro.cpp
 *
 *  Created on: Sep 17, 2025
 *      Author: ntobler
 */

#include "gyro.h"

#include "main.h"

constexpr float SCALE_RAD_S_PER_DIGIT = (500.0 * std::numbers::pi) / ((1 << 15) * 180.0);
constexpr float SCALE_G_PER_DIGIT = (2.0) / ((1 << 15));
constexpr float SCALE_MPSS_PER_G = 9.81;
constexpr float SCALE_MPSS_PER_DIGIT = SCALE_G_PER_DIGIT * SCALE_MPSS_PER_G;
constexpr float DELTA_T = 0.005;

enum Registers {
    READ = 0x80,
    TEMP_DATA1 = 9,
    INT_CONFIG = 6,
    PWR_MGMT0 = 31,

    GYRO_CONFIG0 = 32,
    GYRO_UI_FS_SEL_500DPS = 2 << 5,
    GYRO_ODR_100HZ = 9,
    GYRO_ODR_200HZ = 8,

    ACCEL_CONFIG0 = 33,
    ACCEL_UI_FS_SEL_2G = 3 << 5,
    ACCEL_ODR_100HZ = 9,
    ACCEL_ODR_200HZ = 8,

    INT_SOURCE0 = 43,
    INT_SOURCE_DRDY_INT1_EN = 1 << 3,
};

Gyro::Gyro(SPI_HandleTypeDef *hspi)
    : started_{false},
      buffer_(),
      write_index_{0},
      read_index_{0},
      hspi_(hspi),
      gyro_scale_{0, 0, 0},
      gyro_bias_{0, 0, 0},
      gyro_{0, 0, 0},
      accel_{0, 0, 0},
      q_{},
      accel_bias_filters_{{HpFilter{0.999f}, HpFilter{0.999f}, HpFilter{0.999f}}},
      pos_filters_{{LpIntFilter{0.999f}, LpIntFilter{0.999f}, LpIntFilter{0.999f}}},
      pos_{0, 0, 0},
      id_{0},
      gyro_raw_history_{} {}

void Gyro::start() {
    uint8_t config;

    config = 0x12;  // Set both interrupt to pushpull
    write(Registers::INT_CONFIG, &config, 1);

    config = 0x8f;  // Power on gyro and accel in low noise mode
    write(Registers::PWR_MGMT0, &config, 1);

    // delay_us(200);  // needed after setting low noise mode

    config = Registers::GYRO_UI_FS_SEL_500DPS + Registers::GYRO_ODR_200HZ;
    write(Registers::GYRO_CONFIG0, &config, 1);
    config = Registers::ACCEL_UI_FS_SEL_2G + Registers::ACCEL_ODR_200HZ;
    write(Registers::ACCEL_CONFIG0, &config, 1);

    config = Registers::INT_SOURCE_DRDY_INT1_EN;
    write(Registers::INT_SOURCE0, &config, 1);

    started_ = true;
}

void Gyro::write(uint8_t address, const uint8_t *data, size_t len) {
    address &= ~READ;
    HAL_GPIO_WritePin(SPI3_GYRO_CS_GPIO_Port, SPI3_GYRO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi_, &address, 1, 1000);
    HAL_SPI_Transmit(hspi_, data, len, 1000);
    HAL_GPIO_WritePin(SPI3_GYRO_CS_GPIO_Port, SPI3_GYRO_CS_Pin, GPIO_PIN_SET);
}

void Gyro::tick() {
    if (write_index_ == read_index_) return;

    DataPayload &payload = buffer_[read_index_];

    id_++;

    gyro_.x = (float)((int16_t)(((uint16_t)payload.gyro_x_lower) +
                                (((uint16_t)payload.gyro_x_higher) << 8))) *
              SCALE_RAD_S_PER_DIGIT;
    gyro_.y = (float)((int16_t)(((uint16_t)payload.gyro_y_lower) +
                                (((uint16_t)payload.gyro_y_higher) << 8))) *
              SCALE_RAD_S_PER_DIGIT;
    gyro_.z = (float)((int16_t)(((uint16_t)payload.gyro_z_lower) +
                                (((uint16_t)payload.gyro_z_higher) << 8))) *
              SCALE_RAD_S_PER_DIGIT;

    accel_.x = (float)((int16_t)(((uint16_t)payload.accel_x_lower) +
                                 (((uint16_t)payload.accel_x_higher) << 8))) *
               SCALE_MPSS_PER_DIGIT;
    accel_.y = (float)((int16_t)(((uint16_t)payload.accel_y_lower) +
                                 (((uint16_t)payload.accel_y_higher) << 8))) *
               SCALE_MPSS_PER_DIGIT;
    accel_.z = (float)((int16_t)(((uint16_t)payload.accel_z_lower) +
                                 (((uint16_t)payload.accel_z_higher) << 8))) *
               SCALE_MPSS_PER_DIGIT;

    gyro_raw_history_[id_ % GYRO_RAW_HISTORY_LEN] = gyro_;

    gyro_.x *= 1.0 + gyro_scale_.x;
    gyro_.y *= 1.0 + gyro_scale_.y;
    gyro_.z *= 1.0 + gyro_scale_.z;

    gyro_.x += gyro_bias_.x;
    gyro_.y += gyro_bias_.y;
    gyro_.z += gyro_bias_.z;

    Quaternion q = Quaternion::from_angular_velocities(gyro_.x, gyro_.y, gyro_.z, DELTA_T);
    q_.multiply_right(q);
    q_.normalize();

    Vec3 filtered_accel;
    filtered_accel.x = accel_bias_filters_[0].process(accel_.x);
    filtered_accel.y = accel_bias_filters_[1].process(accel_.y);
    filtered_accel.z = accel_bias_filters_[2].process(accel_.z);

    pos_.x = pos_filters_[0].process(filtered_accel.x * DELTA_T);
    pos_.y = pos_filters_[1].process(filtered_accel.y * DELTA_T);
    pos_.z = pos_filters_[2].process(filtered_accel.z * DELTA_T);

    read_index_ = (read_index_ + 1) % N_ELEMENTS;
}

void Gyro::adjust(Quaternion &q, Vec3 &scale, Vec3 &bias, uint16_t t) {
    gyro_scale_ = scale;
    gyro_bias_ = bias;
    q_ = q;

    t = t % GYRO_RAW_HISTORY_LEN;

    while (t != id_ % GYRO_RAW_HISTORY_LEN) {
        Vec3 omega_raw = gyro_raw_history_[t];

        omega_raw.x *= 1.0 + gyro_scale_.x;
        omega_raw.y *= 1.0 + gyro_scale_.y;
        omega_raw.z *= 1.0 + gyro_scale_.z;

        omega_raw.x += gyro_bias_.x;
        omega_raw.y += gyro_bias_.y;
        omega_raw.z += gyro_bias_.z;

        Quaternion delta_q =
            Quaternion::from_angular_velocities(omega_raw.x, omega_raw.y, omega_raw.z, DELTA_T);
        q.multiply_right(delta_q);
        q_.normalize();

        t = (t + 1) % GYRO_RAW_HISTORY_LEN;
    }
}

void Gyro::get_xy_images(float &x, float &y) {
    // Rotate z vector with internal quaternion and return x and y images.
    float v[3] = {0.0, 0.0, 1.0};
    q_.rotate_vec(v);
    x = v[0];
    y = v[1];
}

void Gyro::fetch_next() {
    if (!started_) return;
    HAL_GPIO_WritePin(TP2_GPIO_Port, TP2_Pin, GPIO_PIN_SET);
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
        HAL_GPIO_WritePin(TP2_GPIO_Port, TP2_Pin, GPIO_PIN_RESET);
    }
}
