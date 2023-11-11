/*
 * ui.cpp
 *
 *  Created on: Nov 11, 2023
 *      Author: ftobler
 */

#include "ui.h"
#include "ssd1306.h"
#include "button.h"
#include "oled_demo.h"



static AbstractUI* active = 0;
static Splash splash;
static Home home;


Button button_up(blackpill_button_GPIO_Port, blackpill_button_Pin, 1);
Button button_down(0, 0, 0);
Button button_left(0, 0, 0);
Button button_right(0, 0, 0);


AbstractUI* Splash::update(Ui_event_en e) {
	if (e == SHOW) {
		timer = 0;
	    ssd1306_SetColor(Black);
	    ssd1306_Fill();
	    ssd1306_SetCursor(2, 18);
	    ssd1306_WriteString("Startracker", Font_11x18);
	    ssd1306_UpdateScreen();
		return 0;
	}

	timer++;
	if (timer > 20) {
		return &home;
	}
	return 0;

}

AbstractUI* Home::update(Ui_event_en e) {
	if (e == SHOW) {
	    ssd1306_Clear();
	    ssd1306_SetColor(White);
	    ssd1306_SetCursor(0, 0);
	    ssd1306_WriteString("Home", Font_7x10);
	    ssd1306_UpdateScreen();
		return 0;
	}
	if (button_up.event == BTN_FALLING) {
		ssd1306_demo_run();
	}
	return 0;
}


void ui_init() {
	  ssd1306_Init();
	  active = &splash;
	  active->update(SHOW);
}

void ui_update() {
	button_up.update();
	button_down.update();
	button_left.update();
	button_right.update();
	if (active) {
		AbstractUI* new_ui = active->update(UPDATE);
		if (new_ui) {
			active->update(HIDE);
			active = new_ui;
			active->update(SHOW);
		}
	}
}
