/*
 * application.cpp
 *
 *  Created on: Nov 7, 2023
 *      Author: cchtofl01
 */


#include "application.h"
#include "scheduler.h"
#include "ssd1306.h"
#include "string"
#include "serial.h"
#include "serial_vcp.h"

extern "C" {
#include "usbd_cdc_if.h"
}

#include "ui.h"

static uint32_t vpc_send_cb(uint8_t* buf, uint32_t len);

static uint8_t stack0[4096];
static uint8_t stack1[4096];
static uint8_t stack2[4096];
static void task0();
static void task1();
static void task2();

static void ssd1306_TestBorder();
static void ssd1306_TestFonts();
static void ssd1306_TestFPS();
static void ssd1306_TestLine();
static void ssd1306_TestRectangle();
static void ssd1306_TestCircle();
static void ssd1306_TestArc();
static void ssd1306_TestPolyline();

extern uint32_t os_started;
extern UART_HandleTypeDef huart1;

Serial serial_rpi;
SerialVCP serial_usb(vpc_send_cb);



void app_init() {
	serial_rpi.init(&huart1);
	__enable_irq();

	scheduler_addTask(0,  task0, stack0,  4096);
	scheduler_addTask(1,  task1, stack1,  4096);
	scheduler_addTask(2,  task2, stack2,  4096);
	os_started = 1;
	scheduler_join();

}

void uart_isr() {
	serial_rpi.ISR(&huart1);
}

void vcp_receive_cb(uint8_t* buf, uint32_t len) {
	serial_usb.vcp_receive_cb(buf, len);
}

void vcp_send_cmplt_cb() {
	serial_usb.vcp_send_cmplt_cb();
}

static uint32_t vpc_send_cb(uint8_t* buf, uint32_t len) {
	uint8_t res = CDC_Transmit_FS(buf, len);
	return res != USBD_OK;
}

static void task0() {
	while (1) {
		//IDLE TASK
//		if (serial_usb.available()) {
//			char c = serial_usb.read();
//			serial_usb.write(c);
//		}
		int avail = serial_usb.available();
		if (avail > 256) {
			avail = 256;
		}
		if (avail) {
			static uint8_t buf[256];
			serial_usb.readBuf(buf, avail);
			serial_usb.writeBuf(buf, avail);
			//scheduler_task_sleep(10);
		}
	}
}

static void task1() {

	ui_init();
	while (1) {
		ui_update();
		scheduler_task_sleep(50);
	}
////	__enable_irq();
//	  uint8_t orientation = 1;
//	  ssd1306_Init();
//
//	  while(1)
//	  {
//	    if(orientation == 1)
//	    {
//	      ssd1306_ResetOrientation();
//	    }
//	    else if(orientation == 2)
//	    {
//	      ssd1306_FlipScreenVertically();
//	    }
//	    else if(orientation == 3)
//	    {
//	      ssd1306_MirrorScreen();
//	    }
//	    else if(orientation == 4)
//	    {
//	      ssd1306_MirrorFlipScreen();
//	      orientation = 0;
//	    }
////	    orientation++;
//
//	    ssd1306_TestFPS();
//	    scheduler_task_sleep(3000);
//	    ssd1306_TestBorder();
//	    ssd1306_TestFonts();
//	    scheduler_task_sleep(3000);
//	    ssd1306_SetColor(Black);
//	    ssd1306_Fill();
//	    ssd1306_TestRectangle();
//	    ssd1306_TestLine();
//	    scheduler_task_sleep(3000);
//	    ssd1306_SetColor(Black);
//	    ssd1306_Fill();
//	    ssd1306_TestPolyline();
//	    scheduler_task_sleep(3000);
//	    ssd1306_SetColor(Black);
//	    ssd1306_Fill();
//	    ssd1306_TestArc();
//	    scheduler_task_sleep(3000);
//	    ssd1306_SetColor(Black);
//	    ssd1306_Fill();
//	    ssd1306_TestCircle();
//	    scheduler_task_sleep(3000);
//	  }
}


static void task2() {
	while (1) {
		scheduler_task_sleep(3000);
	}
}



static void ssd1306_TestBorder()
{
  ssd1306_SetColor(Black);
  ssd1306_Fill();

  uint32_t start = HAL_GetTick();
  uint32_t end = start;
  uint8_t x = 0;
  uint8_t y = 0;
  do
  {
    ssd1306_SetColor(Black);
    ssd1306_DrawPixel(x, y);

    if((y == 0) && (x < 127))
      x++;
    else if((x == 127) && (y < (SSD1306_HEIGHT-1)))
      y++;
    else if((y == (SSD1306_HEIGHT-1)) && (x > 0))
      x--;
    else
      y--;

    ssd1306_SetColor(White);
    ssd1306_DrawPixel(x, y);
    ssd1306_UpdateScreen();

    scheduler_task_sleep(5);
    end = HAL_GetTick();
  } while((end - start) < 8000);

  scheduler_task_sleep(1000);
}

static void ssd1306_TestFonts()
{
  uint8_t y = 0;
  ssd1306_SetColor(Black);
  ssd1306_Fill();

  ssd1306_SetColor(White);
  ssd1306_SetCursor(2, y);
  ssd1306_WriteString("Font 16x26", Font_16x26);
  y += 26;

  ssd1306_SetCursor(2, y);
  ssd1306_WriteString("Font 11x18", Font_11x18);
  y += 18;

  ssd1306_SetCursor(2, y);
  ssd1306_WriteString("Font 7x10", Font_7x10);
  y += 10;

  ssd1306_UpdateScreen();
}

static void ssd1306_TestFPS()
{
  ssd1306_SetColor(White);
  ssd1306_Fill();

  uint32_t start = HAL_GetTick();
  uint32_t end = start;
  int fps = 0;
  char message[] = "ABCDEFGHIJK";

  ssd1306_SetCursor(2, 0);
  ssd1306_SetColor(Black);
  ssd1306_WriteString("Testing...", Font_11x18);

  do
  {
    ssd1306_SetCursor(2, 18);
    ssd1306_WriteString(message, Font_11x18);
    ssd1306_UpdateScreen();

    char ch = message[0];
    memmove(message, message + 1, sizeof(message) - 2);
    message[sizeof(message) - 2] = ch;

    fps++;
    end = HAL_GetTick();
  } while((end - start) < 5000);

  scheduler_task_sleep(1000);

  char buff[64];
  fps = (float)fps / ((end - start) / 1000.0);
  snprintf(buff, sizeof(buff), "~%d FPS", fps);

  ssd1306_SetColor(White);
  ssd1306_Fill();
  ssd1306_SetCursor(2, 2);
  ssd1306_SetColor(Black);
  ssd1306_WriteString(buff, Font_11x18);
  ssd1306_UpdateScreen();
}

static void ssd1306_TestLine()
{
  ssd1306_SetColor(White);
  ssd1306_DrawLine(1, 1, SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1);
  ssd1306_DrawLine(SSD1306_WIDTH - 1, 1, 1, SSD1306_HEIGHT - 1);
  ssd1306_UpdateScreen();
  return;
}

static void ssd1306_TestRectangle()
{
  uint32_t delta;

  ssd1306_SetColor(White);
  for(delta = 0; delta < 5; delta ++) {
    ssd1306_DrawRect(1 + (5 * delta), 1 + (5 * delta) ,SSD1306_WIDTH-1 - (5 * delta), SSD1306_HEIGHT-1 - (5 * delta));
  }
  ssd1306_UpdateScreen();
  return;
}

static void ssd1306_TestCircle()
{
  uint32_t delta;

  ssd1306_SetColor(White);
  for(delta = 0; delta < 5; delta ++) {
    ssd1306_DrawCircle(20 * delta + 30, 15, 10);
  }
  ssd1306_UpdateScreen();
  return;
}

static void ssd1306_TestArc()
{
  ssd1306_SetColor(White);
  ssd1306_DrawArc(30, 30, 30, 20, 270);
  ssd1306_UpdateScreen();
  return;
}

static void ssd1306_TestPolyline()
{
  SSD1306_VERTEX loc_vertex[] =
  {
    {35, 40},
    {40, 20},
    {45, 28},
    {50, 10},
    {45, 16},
    {50, 10},
    {53, 16}
  };

  ssd1306_SetColor(White);
  ssd1306_Polyline(loc_vertex, sizeof(loc_vertex) / sizeof(loc_vertex[0]));
  ssd1306_UpdateScreen();
  return;
}
