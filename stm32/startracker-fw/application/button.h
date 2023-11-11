/*
 * button.h
 *
 *  Created on: ~2017
 *      Author: ftobler
 */


#ifndef BUTTON_H_
#define BUTTON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stm32_hal.h"


typedef enum {
	BTN_NOTHING = 0,
	BTN_RISING = 1,
	BTN_FALLING = 2,
	BTN_LONG = 3
} Button_state_en;


class Button {
    private:
		uint16_t _pin;
		GPIO_TypeDef* _port;
        bool _oldState;
        bool _activeLevel;
        unsigned long _lastRising;
    public:
        Button_state_en event;
        Button(GPIO_TypeDef* port, uint16_t pin, bool activeLevel);
        void update();
};


#ifdef __cplusplus
}
#endif



#endif /* BUTTON_H_ */
