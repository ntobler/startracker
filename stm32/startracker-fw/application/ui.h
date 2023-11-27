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

typedef enum {
	MENU_RPI,
	MENU_TRACKING,
	MENU_TRAJECTORY1,
	MENU_TRAJECTORY2,
	MENU_BUBBLE,
	MENU_VIEW,
	MENU_SHUTDOWN,
	MENU_ABOUT,
	MENU_COUNT
} Menu_entry_id_en;


class AbstractUI {
public:
	virtual AbstractUI* update(Ui_event_en e);
};


class Splash : public AbstractUI {
	int timer = 0;
	void draw();
public:
	AbstractUI* update(Ui_event_en e) override;
};

class Home : public AbstractUI {
	void draw();
public:
	AbstractUI* update(Ui_event_en e) override;
};

class Menu : public AbstractUI {
	int pos = 0;
	float fpos = 0.0f;
	void draw();
public:
	AbstractUI* update(Ui_event_en e) override;
};

class Bubble : public AbstractUI {
	void draw();
public:
	AbstractUI* update(Ui_event_en e) override;
};

class Shutdown : public AbstractUI {
	void draw();
public:
	AbstractUI* update(Ui_event_en e) override;
};

class About : public AbstractUI {
	void draw();
public:
	AbstractUI* update(Ui_event_en e) override;
};


void ui_init();
void ui_update();


#endif /* UI_H_ */
