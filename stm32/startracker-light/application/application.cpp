#include "application.h"

#include <atomic>

#include "gyro.h"
#include "ledring.h"
#include "main.h"
// #include "usbd_cdc_if.h"
#include "arm_math.h"
#include "button.h"
#include "rpi.h"

class Application {
   private:
    void check_packets();

   public:
    LedRing led_ring_;
    Gyro gyro_;
    Button power_button_;
    Serial rpi_serial_;
    PacketReader packet_reader_;
    Rpi rpi_;
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
    TICK_5ms = 32,
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
    : led_ring_{led_spi},
      gyro_{gyro_spi},
      power_button_{BUTTON_POWER_GPIO_Port, BUTTON_POWER_Pin, true},
      rpi_serial_{},
      packet_reader_{},
      rpi_{&gyro_, &rpi_serial_, &led_ring_},
      ms_{0},
      event_flags_{0} {
    instance_ = this;
    led_ring_.start();
    gyro_.start();
    rpi_serial_.init(rpi_uart);
    rpi_.start();

    HAL_GPIO_WritePin(TP4_GPIO_Port, TP4_Pin, GPIO_PIN_RESET);
}

void Application::run() {
    while (1) {
        uint32_t events = event_flags_.exchange(0, std::memory_order_relaxed);
        if (!events) continue;
        HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, GPIO_PIN_SET);
        if (events & Events::TICK_1ms) {
            uint8_t button_events = power_button_.tick();
            if (button_events & ButtonEvent::ShortClick) {
                rpi_.short_click();
            }
            if (button_events & ButtonEvent::LongClick) {
                rpi_.long_press();
            }
        }
        if (events & Events::TICK_5ms) {
            led_ring_.tick();
        }
        if (events & Events::TICK_10ms) {
            check_packets();
            rpi_.tick_10ms();
        }
        if (events & Events::TICK_100ms) {
        }
        if (events & Events::NEW_GYRO) {
            gyro_.tick();
            float x, y;
            gyro_.get_xy_images(x, y);
            if (std::isnan(x)) {
                led_ring_.set_mode(LedMode::DOT);
            }
            float z = gyro_.get_pos().z;
            led_ring_.set_pos(x, y, z);
        }
        if (events & Events::TICK_1000ms) {
        }
        HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, GPIO_PIN_RESET);
    }
}

void Application::check_packets() {
    if (packet_reader_.check_packet(rpi_serial_)) {
        const uint8_t *data = packet_reader_.payload();
        switch (packet_reader_.cmd()) {
            case Cmd::QUAT:
                // Command never sent by Rpi
                break;
            case Cmd::STARQUAT: {
                Quaternion q;
                std::memcpy(&q.q_, &data[0], sizeof(float) * 4);
                uint16_t id = *((uint16_t *)&data[16]);
                rpi_.star_quat(q, id);
            } break;
            case Cmd::SHUT_DOWN_REQUEST:
                // Command never sent by Rpi
                break;
            default:
                break;
        }
    }
}

void Application::timer_isr() {
    ms_ = ms_ >= 1000 - 1 ? 0 : ms_ + 1;
    uint32_t new_events = Events::TICK_1ms;
    new_events |= (ms_ % 5 == 0) * Events::TICK_5ms;
    new_events |= (ms_ % 10 == 0) * Events::TICK_10ms;
    new_events |= (ms_ % 100 == 0) * Events::TICK_100ms;
    new_events |= (ms_ == 0) * Events::TICK_1000ms;
    event_flags_.fetch_or(new_events, std::memory_order_relaxed);
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (Application::instance()) {
        if (GPIO_Pin == GYRO_INT1_Pin) {
            Application::instance()->gyro_.fetch_next();
        }
    }
}

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
