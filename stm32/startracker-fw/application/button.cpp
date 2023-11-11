/*
 * button.cpp
 *
 *  Created on: ~2017
 *      Author: ftobler
 */


#include "button.h"


Button::Button(GPIO_TypeDef* port, uint16_t pin, bool activeLevel) {
    _port = port;
    _pin = pin;
    _oldState = !activeLevel;
    _activeLevel = activeLevel;
    _lastRising = uwTick;
    event = BTN_NOTHING;
//    pinMode(_pin, INPUT_PULLUP);
}

void Button::update() {
	if (_port == 0) {
		event = BTN_NOTHING;
		return;
	}
//    bool newState = digitalRead(_pin);
    bool newState = HAL_GPIO_ReadPin(_port, _pin);
    if (_activeLevel) {
        newState = !newState;
    }
    if (newState != _oldState) {
        _oldState = newState;
        if (newState) {
            _lastRising = uwTick;
        }
        if (!newState) {
            if ((uwTick - _lastRising) > 1000) {
                event = BTN_LONG;
                return;
            }
        }
        event = newState ? BTN_RISING : BTN_FALLING;
        return;
    }
    event = BTN_NOTHING;
}


