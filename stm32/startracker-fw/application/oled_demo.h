/*
 * oled_demo.h
 *
 *  Created on: Nov 11, 2023
 *      Author: ftobler
 */

#ifndef OLED_DEMO_H_
#define OLED_DEMO_H_


#ifdef __cplusplus
extern "C" {
#endif

void ssd1306_demo_run();
void ssd1306_TestBorder();
void ssd1306_TestFonts();
void ssd1306_TestFPS();
void ssd1306_TestLine();
void ssd1306_TestRectangle();
void ssd1306_TestCircle();
void ssd1306_TestArc();
void ssd1306_TestPolyline();


#ifdef __cplusplus
}
#endif


#endif /* OLED_DEMO_H_ */
