#include "application.h"

#include <atomic>

#include "gyro.h"
#include "ledring.h"
#include "main.h"
#include "serial.h"
#include "usbd_cdc_if.h"

class Application {
   public:
    LedRing led_ring_;
    Gyro gyro_;
    Serial rpi_serial_;
    uint16_t ms_;
    std::atomic<uint32_t> event_flags_;
    inline static Application *instance_{0};
    Application(SPI_HandleTypeDef *led_spi, SPI_HandleTypeDef *gyro_spi,
                UART_HandleTypeDef *rpi_uart);
    void run();
    void timer_isr();
    static Application *instance() { return instance_; }
};

enum Events {
    TICK_1ms = 1,
    TICK_10ms = 2,
    TICK_100ms = 4,
    TICK_1000ms = 8,
    NEW_GYRO = 16,
};

void app_init(SPI_HandleTypeDef *led_spi, SPI_HandleTypeDef *gyro_spi,
              UART_HandleTypeDef *rpi_uart) {
    Application app(led_spi, gyro_spi, rpi_uart);
    app.run();
}

Application::Application(SPI_HandleTypeDef *led_spi, SPI_HandleTypeDef *gyro_spi,
                         UART_HandleTypeDef *rpi_uart)
    : led_ring_{led_spi}, gyro_{gyro_spi}, ms_{0}, event_flags_{0} {
    rpi_serial_.init(rpi_uart);
    instance_ = this;
}

void Application::run() {
    while (1) {
        uint32_t events = event_flags_.exchange(0, std::memory_order_relaxed);
        if (!events) continue;
        HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, GPIO_PIN_SET);
        if (events & Events::TICK_1ms) {
            gyro_.fetch_next();
        }
        if (events & Events::TICK_10ms) {
            led_ring_.tick();
        }
        if (events & Events::TICK_100ms) {
            char msg[] = "Hello over USB!\r\n";
            CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
        }
        if (events & Events::NEW_GYRO) {
            gyro_.tick();
        }
        HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, GPIO_PIN_RESET);
    }
}

void Application::timer_isr() {
    ms_ = ms_ >= 1000 - 1 ? 0 : ms_ + 1;
    uint32_t new_events = Events::TICK_1ms;
    new_events |= (ms_ % 10 == 0) * Events::TICK_10ms;
    new_events |= (ms_ % 100 == 0) * Events::TICK_100ms;
    new_events |= (ms_ == 0) * Events::TICK_1000ms;
    event_flags_.fetch_or(new_events, std::memory_order_relaxed);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {}

void app_systick_isr() {
    if (Application::instance()) {
        Application::instance()->timer_isr();
    }
}

extern "C" void app_usasrt1_irq_handler() {
    if (Application::instance()) {
        Application::instance()->rpi_serial_.usart1_isr();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (Application::instance()) {
        Application::instance()->gyro_.spi_tx_rx_complete_callback(hspi);
        Application::instance()->event_flags_.fetch_or(NEW_GYRO, std::memory_order_relaxed);
    }
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (Application::instance()) {
        Application::instance()->led_ring_.spi_tx_complete_callback(hspi);
    }
}
