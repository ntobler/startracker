/*
 * ui.cpp
 *
 *  Created on: Nov 11, 2023
 *      Author: ftobler
 */

#include "ui.h"
#include "ssd1306.h"
#include "ssd1306_write_float.h"
#include "button.h"
#include "oled_demo.h"
#include "stdio.h"
#include "image.h"
#include "imu.h"


static AbstractUI* active = 0;
static Splash splash;
static Home home;
static Menu menu;
static Bubble bubble;
static Shutdown shutdown;
static About about;
extern float position[3];
extern uint32_t tick;
extern Imu_data_t imu;


Button button_up(BUTTON_UP_GPIO_Port, BUTTON_UP_Pin, 1);
Button button_down(BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin, 1);
Button button_left(BUTTON_LEFT_GPIO_Port, BUTTON_LEFT_Pin, 1);
Button button_right(BUTTON_RIGHT_GPIO_Port, BUTTON_RIGHT_Pin, 1);


extern Image_description_t img_splash;
extern Image_description_t img_rpi;
extern Image_description_t img_camera;
extern Image_description_t img_motor;
extern Image_description_t img_power;
extern Image_description_t img_power_active;
extern Image_description_t img_ursa_maior;
extern Image_description_t img_battery0;
extern Image_description_t img_battery1;
extern Image_description_t img_battery2;
extern Image_description_t img_battery3;
extern Image_description_t img_battery4;
extern Image_description_t img_battery5;
extern Image_description_t img_battery6;
extern Image_description_t img_battery7;
extern Image_description_t img_battery8;
extern Image_description_t img_battery9;
extern Image_description_t img_battery10;
static Image_description_t* img_batterys [] = {
	&img_battery0,
	&img_battery1,
	&img_battery2,
	&img_battery3,
	&img_battery4,
	&img_battery5,
	&img_battery6,
	&img_battery7,
	&img_battery8,
	&img_battery9,
	&img_battery10,
};


AbstractUI* Splash::update(Ui_event_en e) {
	if (e == SHOW) {
		timer = 0;
		draw();
		return 0;
	}

	timer++;
	if (timer > 20) {
		return &home;
	}
	draw();
	return 0;

}

void Splash::draw() {
//    ssd1306_SetColor(Black);
//    ssd1306_Fill();
//    ssd1306_SetCursor(2, 18);
//    ssd1306_WriteString("Startracker", Font_11x18);
//    ssd1306_UpdateScreen();

	ssd1306_SetColor(White);
    ssd1306_Clear();
    ssd1306_DrawImage(0, 0, &img_splash);
    ssd1306_UpdateScreen();
}

AbstractUI* Home::update(Ui_event_en e) {
//	if (e == SHOW) {
//		draw();
//		return 0;
//	}
	draw();
	if (button_up.event == BTN_RISING) {
		return &menu;
	}
	return 0;
}




typedef enum YourEnum : uint8_t {
	ENUM_VALUE_1,
	ENUM_VALUE_2,
} YourEnum;

volatile static YourEnum yourEnum[100];

void Home::draw() {
    ssd1306_Clear();
    ssd1306_SetColor(White);
    int i = 0;

    ssd1306_SetCursor(0, i++*13);
//    ssd1306_WriteString("Home", Font_7x10);

//    ssd1306_SetCursor(0, i++*13);
//    ssd1306_WriteString("pos[0]=", Font_7x10);
//    ssd1306_write_float(position[0], 3, Font_7x10);
//
//    ssd1306_SetCursor(0, i++*13);
//    ssd1306_WriteString("pos[1]=", Font_7x10);
//    ssd1306_write_float(position[1], 3, Font_7x10);
//
//    ssd1306_SetCursor(0, i++*13);
//    ssd1306_WriteString("pos[2]=", Font_7x10);
//    ssd1306_write_float(position[2], 3, Font_7x10);
//    i = 4;
    ssd1306_SetCursor(0, i++*13);
    ssd1306_WriteString("tick=", Font_7x10);
    ssd1306_write_int(tick, Font_7x10);

//    i = 0;
//	ssd1306_FillRect(0, 15+i++*4, imu.raw.a[0]/128, 3);
//	ssd1306_FillRect(0, 15+i++*4, imu.raw.a[1]/128, 3);
//	ssd1306_FillRect(0, 15+i++*4, imu.raw.a[2]/128, 3);
//	ssd1306_FillRect(0, 15+i++*4, imu.raw.g[0]/128, 3);
//	ssd1306_FillRect(0, 15+i++*4, imu.raw.g[1]/128, 3);
//	ssd1306_FillRect(0, 15+i++*4, imu.raw.g[2]/128, 3);
//	ssd1306_FillRect(0, 15+i++*4, imu.raw.m[0]/4, 3);
//	ssd1306_FillRect(0, 15+i++*4, imu.raw.m[1]/4, 3);
//	ssd1306_FillRect(0, 15+i++*4, imu.raw.m[2]/4, 3);

	ssd1306_SetCursor(0, i++*13);
	ssd1306_WriteString("yaw=", Font_7x10);
	ssd1306_write_float(imu.fusion.yaw, 3, Font_7x10);

	ssd1306_SetCursor(0, i++*13);
	ssd1306_WriteString("pitch=", Font_7x10);
	ssd1306_write_float(imu.fusion.pitch, 3, Font_7x10);

	ssd1306_SetCursor(0, i++*13);
	ssd1306_WriteString("roll=", Font_7x10);
	ssd1306_write_float(imu.fusion.roll, 3, Font_7x10);


    i = 0;
    ssd1306_DrawImage(i++ * 20, 0, &img_rpi);
    ssd1306_DrawImage(i++ * 20, 0, &img_ursa_maior);
    ssd1306_DrawImage(i++ * 20, 0, img_batterys[(uwTick / 200) % 11]);
    ssd1306_DrawImage(i++ * 20, 0, uwTick % 1000 > 500 ? &img_power : &img_power_active);
    ssd1306_DrawImage(i++ * 20, 0, &img_motor);
    ssd1306_DrawImage(i++ * 20, 0, &img_camera);

    ssd1306_UpdateScreen();
}

extern Image_description_t img_rpi;
extern Image_description_t img_camera;
extern Image_description_t img_motor;
extern Image_description_t img_power;
extern Image_description_t img_ursa_maior;
extern Image_description_t img_battery;



AbstractUI* Menu::update(Ui_event_en e) {
	if (e == SHOW) {
		draw();
		return 0;
	}
/*
capture image
start tracking
home motors
power OFF
*/

	if (button_up.event == BTN_RISING) {
//		ssd1306_demo_run();
		pos = (pos + 1) % 8;
		//draw();
	}
	if (button_up.event == BTN_LONG) {
		if (pos == 0) {
			return &home;
		}
		if (pos == 1) {
			return &bubble;
		}
		if (pos == 6) {
			return &shutdown;
		}
		if (pos == 7) {
			return &about;
		}
	}
	draw();
	return 0;
}


void Menu::draw() {
	static const int num_entry = 8;
	static const char* entrys[num_entry] = {
			"home screen",
			"bubble level",
			"capture stars",
			"start tracking",
			"home motors",
			"view camera",
			"shutdown",
			"about",
	};

    ssd1306_Clear();
    ssd1306_SetColor(White);
    fpos = (fpos * 0.6f) + (pos * 0.4f);

    for (int i = 0; i < num_entry; i++) {
    	int y = (int)((i-fpos+2) * 13);
        ssd1306_SetCursor(0, y);

        if (i == pos) {
            ssd1306_SetColor(White);
            ssd1306_FillRect(0, y-1, 256, 13);
        }

        ssd1306_SetColor(i == pos ? Black : White);
        ssd1306_WriteString(entrys[i], Font_7x10);
    }

    ssd1306_UpdateScreen();
}


AbstractUI* Bubble::update(Ui_event_en e) {
	if (e == SHOW) {
		draw();
		return 0;
	}
	if (button_up.event == BTN_RISING) {
		return &menu;
	}
	draw();
	return 0;
}

void Bubble::draw() {
    ssd1306_Clear();

    ssd1306_SetColor(White);
//    ssd1306_DrawArc(64, 32, 80, 0, 360);
//    ssd1306_DrawArc(64, 32, 60, 0, 360);
//    ssd1306_DrawArc(64, 32, 40, 0, 360);
//    ssd1306_DrawArc(64, 32, 20, 0, 360);

    for (int x = 1; x < 8; x++) {
        for (int y = 0; y < 32; y++) {
        	ssd1306_DrawPixel(x*16, y*2);
        }
	}
    for (int y = 1; y < 4; y++) {
        for (int x = 0; x < 64; x++) {
        	ssd1306_DrawPixel(x*2, y*16);
        }
	}

//    int x = (((int)uwTick % 5000 - 2500) / 100.0f) + 64;
//    int y = (((int)uwTick % 5000 - 2500) / 100.0f) + 32;
    int x = imu.fusion.pitch + 64;
    int y = imu.fusion.roll + 32;
    const int r = 6;


//    ssd1306_DrawLine(x-r, y-1, x-1, y-r);
//    ssd1306_DrawLine(x-r, y+1, x-1, y+r);
//    ssd1306_DrawLine(x+r, y-1, x+1, y-r);
//    ssd1306_DrawLine(x+r, y+1, x+1, y+r);


    ssd1306_DrawLine(x-r, y-1, x-1, y-1);
    ssd1306_DrawLine(x-r, y+1, x-1, y+1);
    ssd1306_DrawLine(x+r, y-1, x+1, y-1);
    ssd1306_DrawLine(x+r, y+1, x+1, y+1);


    ssd1306_DrawLine(x-1, y-1, x-1, y-r);
    ssd1306_DrawLine(x-1, y+1, x-1, y+r);
    ssd1306_DrawLine(x+1, y-1, x+1, y-r);
    ssd1306_DrawLine(x+1, y+1, x+1, y+r);

//    ssd1306_FillCircle(x, y, r);
//    ssd1306_SetColor(Black);
//    ssd1306_DrawLine(x-r, y, x+r, y);
//    ssd1306_DrawLine(x, y-r, x, y+r);

    ssd1306_UpdateScreen();
}


AbstractUI* About::update(Ui_event_en e) {
	if (e == SHOW) {
		draw();
		return 0;
	}
	if (button_up.event == BTN_RISING) {
		return &menu;
	}
	return 0;
}

void About::draw() {
    ssd1306_Clear();
    ssd1306_SetColor(White);
    int i = 0;
    ssd1306_SetCursor(0, i++*12);
    ssd1306_WriteString("startracker by", Font_7x10);
    ssd1306_SetCursor(0, i++*12);
    ssd1306_WriteString("Nicolas Tobler", Font_7x10);
    ssd1306_SetCursor(0, i++*12);
    ssd1306_WriteString("Florin Tobler", Font_7x10);
    ssd1306_SetCursor(0, i++*12);
    ssd1306_WriteString("Lukas Barbisch", Font_7x10);
    ssd1306_SetCursor(0, i++*12);
    ssd1306_WriteString("2023", Font_7x10);
    ssd1306_UpdateScreen();
}

AbstractUI* Shutdown::update(Ui_event_en e) {
	if (e == SHOW) {
		draw();
		return 0;
	}
	if (button_up.event == BTN_RISING) {
		NVIC_SystemReset();
	}
	draw();
	return 0;
}

void Shutdown::draw() {
    ssd1306_Clear();

    static int pos = 0;
    pos += 10;
    ssd1306_DrawArc(64, 32, 20, pos % 360, 220 + (pos % 360));
    ssd1306_DrawArc(64, 32, 25, (pos*2) % 360, 120 + ((pos*2) % 360));


    ssd1306_SetColor(White);
    ssd1306_SetCursor(36, 27);
    ssd1306_WriteString("shutdown", Font_7x10);

    ssd1306_UpdateScreen();
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
