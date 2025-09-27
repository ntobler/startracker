/*
 * button.h
 *
 *  Created on: Sep 24, 2025
 *      Author: ntobler
 */

#ifndef BUTTON_H_
#define BUTTON_H_

enum ButtonEvent {
    Press = 1,
    Release = 2,
    ShortClick = 4,
    LongClick = 8,
};

constexpr uint16_t LONGCLICK = 1000;
constexpr uint16_t SHORTCLICK = 20;

class Button {
   private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
    bool inverted_;
    uint16_t counter_;

   public:
    Button(GPIO_TypeDef* port, uint16_t pin, bool inverted)
        : port_{port}, pin_{pin}, inverted_{inverted}, counter_{0} {}
    uint8_t tick() {
        uint8_t ret = 0;
        if (HAL_GPIO_ReadPin(port_, pin_) ^ inverted_) {
            if (counter_ < 0xffff) {
                counter_++;
            }
            if (counter_ == SHORTCLICK) {
                ret |= ButtonEvent::Press;
            } else if (counter_ == LONGCLICK) {
                ret |= ButtonEvent::LongClick;
            }
        } else {
            if (counter_ >= SHORTCLICK) {
                if (counter_ < LONGCLICK) {
                    ret |= ButtonEvent::ShortClick;
                }
                ret |= ButtonEvent::Release;
            }
            counter_ = 0;
        }
        return ret;
    }
};

#endif /* BUTTON_H_ */
