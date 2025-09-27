/*
 * serial.cpp
 *
 *  Created on: Feb 3, 2022
 *      Author: cchtofl01
 */

#include "serial.h"

#include "stm32_hal.h"

void Serial::usart1_isr() {
    uint32_t isrflags = READ_REG(huart->Instance->SR);
    uint32_t cr1its = READ_REG(huart->Instance->CR1);

    uint32_t errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE |
                                                 USART_SR_NE | USART_SR_IDLE));
    if (errorflags == 0 || 1) {
        // handle normal
        if (((isrflags & USART_SR_RXNE) != 0U) && ((cr1its & USART_CR1_RXNEIE) != 0U)) {
            uint8_t data = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
            in.setByte(data);
        }
    } else {
        // handle the errors (receive error)
    }

    if (((isrflags & USART_SR_TXE) != 0U) && ((cr1its & USART_CR1_TXEIE) != 0U)) {
        // sent something
        uint8_t outAvail = out.getAvailable();
        if (outAvail == 0) {
            /* Disable the UART Transmit Complete Interrupt */
            __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

            /* Enable the UART Transmit Complete Interrupt */
            __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
        } else {
            huart->Instance->DR = out.getByte();
        }
    }

    if (((isrflags & USART_SR_TC) != 0U) && ((cr1its & USART_CR1_TCIE) != 0U)) {
        // transmission ended

        /* Disable the UART Transmit Complete Interrupt */
        __HAL_UART_DISABLE_IT(huart, UART_IT_TC);

        txnComplete = 1;
    }
    __HAL_UART_DISABLE_IT(huart, USART_CR1_IDLEIE);
    __HAL_UART_DISABLE_IT(huart, USART_CR1_PEIE);

    //	huart->Instance->ICR = 0b11111111111111111111111111111111;   //clear all flags
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

    /* Enable the UART Parity Error interrupt and Data Register Not Empty interrupt */
    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
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

    /* Enable the Transmit Data Register Empty interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);

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
