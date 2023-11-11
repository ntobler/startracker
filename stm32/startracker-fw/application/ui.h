/*
 * ui.h
 *
 *  Created on: Nov 11, 2023
 *      Author: ftobler
 */

#ifndef UI_H_
#define UI_H_

typedef enum {
	SHOW,
	UPDATE,
	HIDE
} Ui_event_en;


class AbstractUI {
public:
	virtual AbstractUI* update(Ui_event_en e);
};


class Splash : public AbstractUI {
	int timer = 0;
public:
	AbstractUI* update(Ui_event_en e) override;
};


class Home : public AbstractUI {
public:
	AbstractUI* update(Ui_event_en e) override;
};


void ui_init();
void ui_update();


#endif /* UI_H_ */
