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
    _long_executed = false;
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
            _long_executed = false;
        }
        if (_long_executed == false) {
            event = newState ? BTN_FALLING : BTN_RISING;
            return;
        }
    }
    if (newState && _long_executed == false) {
    	if ((uwTick - _lastRising) > 1000) {
			event = BTN_LONG;
			_long_executed = true;
			return;
		}
    }
    event = BTN_NOTHING;
}


