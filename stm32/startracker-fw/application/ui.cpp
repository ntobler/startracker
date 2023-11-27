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
#include "scheduler.h"
#include "control.h"
#include "motor.h"


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
extern Control_t control;
extern Motor_t motor;


Button button_down(BUTTON_UP_GPIO_Port, BUTTON_UP_Pin, 1);
Button button_up(BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin, 1);
Button button_right(BUTTON_LEFT_GPIO_Port, BUTTON_LEFT_Pin, 1);
Button button_left(BUTTON_RIGHT_GPIO_Port, BUTTON_RIGHT_Pin, 1);


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
extern Image_description_t img_battery_full;
extern Image_description_t img_blank;
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
static const char* entrys[MENU_COUNT] = {
		"rpi",
		"tracking",
	"trajectory0",
	"trajectory1",
	"trajectory2",
	"bubble level",
	"view camera",
	"shutdown",
	"about",
};

//start tracking => rpi not running
//stop tracking => rpi running

//---------------- => no trajectory yet
//start trajectory => trajectory calculated
//reset trajectory => trajectory calculated & running
//pause trajectory => trajectory calculated & running

//home motors => always

//disable motors => if running or homed

//bubble level

//view camera

//shutdown

//about

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
	if (e == SHOW) {
		return 0;
	}

	if (button_up.event == BTN_RISING ||
		button_left.event == BTN_RISING ||
		button_right.event == BTN_RISING ||
		button_down.event == BTN_RISING) {
		return &menu;
	}
	if (button_up.event == BTN_LONG) {
		return &shutdown;
	}
	if (button_down.event == BTN_LONG) {
		return &splash;
	}
	draw();
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
    int i = 1;  //first line is line 1 because line 0 is used by all the symbols


    ssd1306_SetCursor(0, i++*13);
    ssd1306_WriteString("bat=", Font_7x10);
    ssd1306_write_float(control.battery_voltage, 2, Font_7x10);
    ssd1306_WriteString("V ", Font_7x10);
    if (control.charger_done == 0 && control.charger_charging == 0) {
        ssd1306_write_int(control.charge_level * 10, Font_7x10);
        ssd1306_WriteString("%", Font_7x10);
    }




    i = 0;
    if (control.state == CONTROL_RPI_READY) {
        ssd1306_DrawImage(i++ * 20, 0, &img_rpi);
    }
    if (control.state == CONTROL_RPI_SHUTDOWN || control.state == CONTROL_RPI_BOOTING) {
        ssd1306_DrawImage(i++ * 20, 0, uwTick % 1000 > 500 ? &img_rpi : &img_blank);
    }
    if (motor.trajectory_ready) {
        ssd1306_DrawImage(i++ * 20, 0, &img_ursa_maior);
    }
    if (control.charger_done) {
        ssd1306_DrawImage(i++ * 20, 0, &img_battery_full);
        ssd1306_DrawImage(i++ * 20, 0, &img_power);
    } else if (control.charger_charging) {
        ssd1306_DrawImage(i++ * 20, 0, img_batterys[(uwTick / 200) % 11]);
        ssd1306_DrawImage(i++ * 20, 0, uwTick % 1000 > 500 ? &img_power : &img_power_active);
    } else {
        ssd1306_DrawImage(i++ * 20, 0, img_batterys[control.charge_level]);
    }
    if (motor.state != MOTOR_MODE_NONE) {
        ssd1306_DrawImage(i++ * 20, 0, &img_motor);
    }
//    ssd1306_DrawImage(i++ * 20, 0, &img_camera);

    ssd1306_UpdateScreen();
}





AbstractUI* Menu::update(Ui_event_en e) {
	if (e == SHOW) {
		return 0;
	}

	if (button_down.event == BTN_RISING) {
		pos++;
		if (pos > MENU_COUNT-1) {
			pos = MENU_COUNT-1;
		}
	}
	if (button_up.event == BTN_RISING) {
		pos--;
		if (pos < 0) {
			pos = 0;
		}
	}
	if (button_left.event == BTN_RISING) {
		return &home;
	}
	if (button_up.event == BTN_LONG || button_right.event == BTN_RISING) {
		if (pos == MENU_RPI) {
			if (control.state == CONTROL_RPI_SHUTDOWN || control.state == CONTROL_RPI_IDLE) {
				control_do(RPI_DO_BOOT);
			} else {
				control_do(RPI_DO_SHUTDOWN);
			}
		}
		if (pos == MENU_TRACKING) {
			control_do(RPI_DO_CALC);
		}
		if (pos == MENU_TRAJECTORY0) {
			if (motor.state == MOTOR_MODE_READY ||
				motor.state == MOTOR_MODE_RUNNING ||
				motor.state == MOTOR_MODE_PAUSE) {
				motor_do(MOTOR_DO_DISABLE);
			}
		}
		if (pos == MENU_TRAJECTORY1) {
			if (motor.state == MOTOR_MODE_NONE ||
				motor.state == MOTOR_MODE_READY) {
				motor_do(MOTOR_DO_HOME);
			}
			if (motor.state == MOTOR_MODE_RUNNING) {
			    motor_do(MOTOR_DO_STOP);
			}
			if (motor.state == MOTOR_MODE_PAUSE) {
			    motor_do(MOTOR_DO_STOP);
			}
		}
		if (pos == MENU_TRAJECTORY2) {
			if (motor.state == MOTOR_MODE_NONE ||
				motor.state == MOTOR_MODE_HOMING ||
				motor.state == MOTOR_MODE_READY) {
				motor_do(MOTOR_DO_START);
			}
			if (motor.state == MOTOR_MODE_RUNNING) {
			    motor_do(MOTOR_DO_PAUSE);
			}
			if (motor.state == MOTOR_MODE_PAUSE) {
			    motor_do(MOTOR_DO_RESUME);
			}
		}
		if (pos == MENU_BUBBLE) {
			return &bubble;
		}
		if (pos == MENU_SHUTDOWN) {
			return &shutdown;
		}
		if (pos == MENU_ABOUT) {
			return &about;
		}
	}


	switch (control.state) {
	case CONTROL_RPI_SHUTDOWN:
	case CONTROL_RPI_IDLE:
		entrys[MENU_RPI] = "boot raspi";
		break;
	case CONTROL_RPI_BOOTING:
	case CONTROL_RPI_READY:
		entrys[MENU_RPI] = "shutdown raspi";
		break;
	}


	switch (motor.state) {
	case MOTOR_MODE_NONE:
		entrys[MENU_TRAJECTORY0] = "(motor off)";
		entrys[MENU_TRAJECTORY1] = "home motors";
		entrys[MENU_TRAJECTORY2] = "trajectory run";
		break;
	case MOTOR_MODE_HOMING:
		entrys[MENU_TRAJECTORY0] = "(motor off)";
		entrys[MENU_TRAJECTORY1] = "(homing)";
		entrys[MENU_TRAJECTORY2] = "trajectory run";
		break;
	case MOTOR_MODE_READY:
		entrys[MENU_TRAJECTORY0] = "disable motors";
		entrys[MENU_TRAJECTORY1] = "home motors";
		entrys[MENU_TRAJECTORY2] = "trajectory run";
		break;
	case MOTOR_MODE_RUNNING:
		entrys[MENU_TRAJECTORY0] = "disable motors";
		entrys[MENU_TRAJECTORY1] = "trajectory stop";
		entrys[MENU_TRAJECTORY2] = "trajectory pause";
		break;
	case MOTOR_MODE_PAUSE:
		entrys[MENU_TRAJECTORY0] = "disable motors";
		entrys[MENU_TRAJECTORY1] = "trajectory stop";
		entrys[MENU_TRAJECTORY2] = "trajectory resume";
		break;
	}


	draw();
	return 0;
}


void Menu::draw() {


    ssd1306_Clear();
    ssd1306_SetColor(White);
    fpos = (fpos * 0.6f) + (pos * 0.4f);

    for (int i = 0; i < MENU_COUNT; i++) {
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
		return 0;
	}
	if (button_up.event == BTN_RISING ||
		button_left.event == BTN_RISING ||
		button_right.event == BTN_RISING ||
		button_down.event == BTN_RISING) {
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
	if (button_up.event == BTN_RISING ||
		button_left.event == BTN_RISING ||
		button_right.event == BTN_RISING ||
		button_down.event == BTN_RISING) {
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
	if (button_right.event == BTN_RISING) {

		HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, GPIO_PIN_RESET);
		scheduler_task_sleep(100);
		NVIC_SystemReset();
	}
	if (button_left.event == BTN_RISING) {
		return &menu;
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
