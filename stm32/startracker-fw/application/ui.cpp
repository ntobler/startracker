/*
 * ui.cpp
 *
 *  Created on: Nov 11, 2023
 *      Author: ftobler
 */

#include "ui.h"
#include "ssd1306.h"

static AbstractUI* active = 0;
static Splash splash;
static Home home;





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
	return 0;
}


void ui_init() {
	  ssd1306_Init();
	  active = &splash;
	  active->update(SHOW);
}

void ui_update() {
	if (active) {
		AbstractUI* new_ui = active->update(UPDATE);
		if (new_ui) {
			active->update(HIDE);
			active = new_ui;
			active->update(SHOW);
		}
	}
}
