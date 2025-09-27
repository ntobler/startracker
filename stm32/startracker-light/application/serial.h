/*
 * Serialh
 *
 *  Created on: 17.10.2020
 *      Author: Florin
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "stdint.h"
#include "stm32_hal.h"

constexpr size_t BUFFER_LEN = 256;

typedef void (*serial_hook)();

constexpr uint16_t CRC16_INITIAL_VALUE = 0xFFFF;
constexpr uint16_t POLYNOMIAL_16 = 0xA001;

class Buffer {
   private:
    size_t head_;
    size_t tail_;
    uint8_t buf_[BUFFER_LEN];

   public:
    Buffer() : head_{0}, tail_{0} {};
    void clear() {
        head_ = 0;
        tail_ = 0;
    }
    uint8_t getByte() {
        uint8_t data = buf_[tail_];
        tail_ = (tail_ + 1) % BUFFER_LEN;
        return data;
    };
    uint32_t getAvailable() { return (BUFFER_LEN + head_ - tail_) % BUFFER_LEN; };
    void setByte(uint8_t data) {
        buf_[head_] = data;
        head_ = (head_ + 1) % BUFFER_LEN;
    };
};

/**
 * UART implementation for STM32G0 MCUs
 * the implementation uses a send and receive FIFO buffer
 */
class Serial {
   private:
    Buffer out;
    Buffer in;
    uint8_t txnComplete;
    void enableTx();

   public:
    UART_HandleTypeDef *huart;  // need access to this for error handling
    serial_hook hook_begin_transmission;
    serial_hook hook_byte_received;
    serial_hook hook_end_transmission;
    serial_hook hook_end_transmission_modbus;
    Serial();
    void usart1_isr();
    void init(UART_HandleTypeDef *handler);
    void initFlow(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
    uint32_t available();
    void flushRX();
    void flushTX();
    void print(const char *str);
    uint8_t read();
    uint32_t readBuf(uint8_t *buf, uint16_t len);
    void write_command(uint8_t cmd, const uint8_t *data, uint8_t len);
    void writeBuf(const uint8_t *buf, uint16_t len);
    void write(const uint8_t data);
};

enum Cmd {
    QUAT = 0,
    STARQUAT = 1,
    SHUT_DOWN_REQUEST = 2,
};

class PacketReader {
   private:
    uint8_t buffer_[64];
    size_t index_;
    uint16_t crc_;

   public:
    PacketReader() : index_{0}, crc_{} {};
    bool check_packet(Serial &serial);
    uint8_t cmd() const { return buffer_[0]; };
    uint8_t len() const { return buffer_[1]; };
    const uint8_t *payload() const { return &buffer_[2]; };
};

#endif /* SERIAL_H_ */
