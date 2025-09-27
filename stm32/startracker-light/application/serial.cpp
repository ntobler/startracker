/*
 * serial.cpp
 *
 *  Created on: Feb 3, 2022
 *      Author: cchtofl01
 */

#include "serial.h"

#include "stm32_hal.h"

void Serial::usart1_isr() {
    uint32_t sr = huart->Instance->SR;    // fired
    uint32_t cr1 = huart->Instance->CR1;  // enabled

    uint32_t errors = (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE);
    if (sr & errors) {
        // Must read DR to clear errors
        volatile uint32_t tmp = huart->Instance->DR;
        (void)tmp;
    }
    // UART receive interrupt
    if ((sr & USART_SR_RXNE) && (cr1 & USART_CR1_RXNEIE)) {
        uint8_t data = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
        in.setByte(data);
    }

    // UART Transmit Empty Interrupt
    if ((sr & USART_SR_TXE) && (cr1 & USART_CR1_TXEIE)) {
        // sent something
        uint8_t outAvail = out.getAvailable();
        if (outAvail == 0) {
            // Disable the UART Transmit Empty Interrupt
            __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

            // Enable the UART Transmit Complete Interrupt
            __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
        } else {
            huart->Instance->DR = out.getByte();
        }
    }

    // Transmit Complete Interrupt
    if ((sr & USART_SR_TC) && (cr1 & USART_CR1_TCIE)) {
        // transmission ended

        /* Disable the UART Transmit Complete Interrupt */
        __HAL_UART_DISABLE_IT(huart, UART_IT_TC);

        txnComplete = 1;
    }
}

/**
 * constructor. Make sure huart pointer is set to 0
 * so errors can be handled
 */
Serial::Serial() {
    huart = 0;
    hook_begin_transmission = 0;
    hook_byte_received = 0;
    hook_end_transmission = 0;
    hook_end_transmission_modbus = 0;
}

void Serial::init(UART_HandleTypeDef *handler) {
    huart = handler;
    in.clear();
    out.clear();

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;

    __disable_irq();
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(huart, UART_IT_PE);
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    __enable_irq();
}

uint32_t Serial::available() { return in.getAvailable(); }

void Serial::flushRX() { in.clear(); }

void Serial::flushTX() { out.clear(); }

void Serial::print(const char *str) {
    const uint8_t *ptr = (uint8_t *)str;
    while (*ptr != '\0') {
        out.setByte(*ptr);
        ptr++;
    }
    enableTx();
}

/**
 * unguarded about overflow!
 * use Serial_available!
 */
uint8_t Serial::read() { return in.getByte(); }

uint32_t Serial::readBuf(uint8_t *buf, uint16_t len) {
    uint16_t count = 0;
    while (in.getAvailable() && (count < len)) {
        buf[count] = in.getByte();
        count++;
    }
    return count;
}

void Serial::writeBuf(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        out.setByte(buf[i]);
    }
    enableTx();
}

static void update_crc16(uint16_t &crc, uint8_t data) {
#if OPTION__TableCRC == 1
    uint8_t crc_index;
    uint16_t crc_lookup;

    crc_index = crc ^ data;
    crc_lookup = crc_tab16[crc_index];

    crc = ((crc >> 8) ^ crc_lookup);
#else
    crc = crc ^ static_cast<uint16_t>(data);
    for (size_t bit = 0; bit < 8; bit++) {
        if (crc & 0x0001) {
            crc = (crc >> 1) ^ POLYNOMIAL_16;
        } else {
            crc = (crc >> 1);
        }
    }
#endif
}

void Serial::write_command(uint8_t cmd, const uint8_t *data, uint8_t len) {
    uint16_t crc = CRC16_INITIAL_VALUE;
    out.setByte(cmd);
    update_crc16(crc, cmd);
    out.setByte(len);
    update_crc16(crc, len);
    for (size_t i = 0; i < len; i++) {
        out.setByte(data[i]);
        update_crc16(crc, data[i]);
    }
    out.setByte(crc >> 8);
    out.setByte(crc & 0xff);
    enableTx();
}

void Serial::write(const uint8_t data) {
    out.setByte(data);
    enableTx();
}

void Serial::enableTx() {
    if (hook_begin_transmission) {
        hook_begin_transmission();
    }

    // Enable the Transmit Data Register Empty interrupt
    __disable_irq();
    __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);
    __enable_irq();

    txnComplete = 0;
}

bool PacketReader::check_packet(Serial &serial) {
    size_t available = serial.available();

    // If there are no new bytes, flush
    if (available == 0) {
        index_ = 0;
        crc_ = CRC16_INITIAL_VALUE;
        return false;
    }

    while (available--) {
        uint8_t data = serial.read();

        buffer_[index_] = data;
        if (index_ < 2 + static_cast<size_t>(len())) {
            if ((index_ == 2) && (len() > sizeof(buffer_) - 2)) {
                index_ = 0;
                crc_ = CRC16_INITIAL_VALUE;
            } else {
                update_crc16(crc_, data);
                index_++;
            }
        } else if (index_ == 2 + static_cast<size_t>(len())) {
            if (data != crc_ >> 8) {
                index_ = 0;
                crc_ = CRC16_INITIAL_VALUE;
            } else {
                index_++;
            }
        } else {
            if (data != (crc_ & 0xff)) {
                index_ = 0;
                crc_ = CRC16_INITIAL_VALUE;
            } else {
                index_ = 0;
                crc_ = CRC16_INITIAL_VALUE;
                return true;
            }
        }
    }
    return false;
}
