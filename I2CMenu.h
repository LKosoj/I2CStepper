#pragma once
#ifndef _I2CMenu_H
#define _I2CMenu_H

#define USE_SOFTWIRE_H_AS_PLAIN_INCLUDE
#ifndef SCL_PORT
#define SCL_PIN 2 //D2
#define SCL_PORT PORTD
#define SDA_PIN 0 //A0
#define SDA_PORT PORTC
#endif

#include <LiquidCrystal_I2C2.h>
#include <LiquidMenu.h>

#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

const char c_On[]  = "On ";
const char c_Off[]  = "Off";

enum FunctionTypes {
  increase = 1,
  decrease = 2,
};

LiquidCrystal_I2C2 lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

LiquidMenu menu(lcd);

byte multiplier;
byte m_cnt;
bool navigate;
byte oldS;


int get_stepper_state();
const char* get_stepper_state_c();
const char* get_mixer_pump_state();
bool set_mixer_pump_state(bool state);
float get_speed(void);
byte get_direction(void);
float get_stepper_time(void);
void start_stepper(bool from_int);
void stop_stepper();
uint32_t get_target_from_array(void);
uint16_t get_spd_stp(uint16_t spd);
void set_speed_to_array(uint16_t spd);
uint16_t get_speed_from_array(void);
void set_target_to_array(uint32_t target);
const char* get_rele_state2();
const char* get_rele_state3();
const char* get_rele_state4();
bool set_rele_state_to_array(byte r, bool s);

LiquidLine back_line(10, 1, "<BACK");

LiquidLine main_line1(0, 0, ">STP: ", get_stepper_time);
LiquidLine main_line2(0, 1, ">Pump: ", get_mixer_pump_state);
LiquidLine main_line3(0, 2, ">Rele2: ", get_rele_state2);
LiquidLine main_line4(0, 3, ">Rele3: ", get_rele_state3);
LiquidLine main_line5(0, 4, ">Rele4: ", get_rele_state4);
LiquidScreen main_screen(main_line1, main_line2);

LiquidLine stp_line_spd(0, 0, ">STP S: ", get_speed);
LiquidLine stp_line_dir(0, 1, ">STP D: ", get_direction);
LiquidLine stp_line_time(0, 2, ">STP T: ", get_stepper_time);
LiquidLine stp_line_start(0, 3, ">STP Start: ", get_stepper_state_c);
LiquidScreen stp_screen(stp_line_spd, stp_line_dir, stp_line_time, stp_line_start);


LiquidMenu main_menu(lcd);

const char* get_rele_state2() {
  if (bit_is_set(rele_state, 1)) return c_On;
  else return c_Off;
}

const char* get_rele_state3() {
  if (bit_is_set(rele_state, 2)) return c_On;
  else return c_Off;
}

const char* get_rele_state4() {
  if (bit_is_set(rele_state, 3)) return c_On;
  else return c_Off;
}

const char* get_mixer_pump_state() {
  if (bit_is_set(rele_state, 0)) return c_On;
  else return c_Off;
}

const char* get_stepper_state_c() {
  if (stepper_state) return c_On;
  else return c_Off;
}

int get_stepper_state() {
  if (stepper_state) return 1;
  else return 0;
}

// Used for attaching something to the lines, to make them focusable.
void blankFunction() {
  return;
}

void backFunction() {
  main_menu.change_screen(&main_screen);
  main_menu.update();
}

void spdIncFunction() {
  byte c = m_cnt;
  if (c < 3) c = 1;
  else c = c / 3;
  set_spd += 1 * multiplier * c;
  if (set_spd > STEPPER_MAX_SPEED) set_spd = STEPPER_MAX_SPEED;
  if (stepper_state) {
    uint16_t s = get_spd_stp(set_spd);
    set_speed_to_array(s);
  }
}

void spdDecFunction() {
  byte c = m_cnt;
  if (c < 3) c = 1;
  else c = c / 3;
  if (set_spd <= 1 * multiplier * c) set_spd = 1;
  else set_spd -= 1 * multiplier * c;
  if (stepper_state) {
    uint16_t s = get_spd_stp(set_spd);
    set_speed_to_array(s);
  }
}

void timeIncFunction() {
  byte c = m_cnt;
  if (c < 3) c = 1;
  else c = c / 3;
  set_time += 1 * multiplier * c;
  if (set_time > 60000) set_time = 60000;
}

void timeDecFunction() {
  byte c = m_cnt;
  if (c < 3) c = 1;
  else c = c / 3;
  if (set_time <= 1 * multiplier * c) set_time = 0;
  else set_time -= 1 * multiplier * c;
}

void dirFunction() {
  set_dir = !set_dir;
  stepper.reverse(set_dir);
}

void menu_init(void) {
  lcd.init();
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  lcd.backlight();
  lcd.clear();
  navigate = true;

  back_line.attach_function(1, backFunction);

  main_line1.attach_function(1, blankFunction);
  main_line2.attach_function(1, blankFunction);

  main_screen.add_line(main_line3);
  main_screen.add_line(main_line4);
  main_screen.add_line(main_line5);
  main_line3.attach_function(1, blankFunction);
  main_line4.attach_function(1, blankFunction);
  main_line5.attach_function(1, blankFunction);

  stp_screen.add_line(back_line);

  stp_line_spd.attach_function(increase, spdIncFunction);
  stp_line_spd.attach_function(decrease, spdDecFunction);

  stp_line_dir.attach_function(increase, backFunction);
  stp_line_dir.attach_function(decrease, backFunction);

  stp_line_time.attach_function(increase, timeIncFunction);
  stp_line_time.attach_function(decrease, timeDecFunction);

  stp_line_start.attach_function(increase, blankFunction);
  stp_line_start.attach_function(decrease, blankFunction);

  main_screen.set_displayLineCount(2);
  stp_screen.set_displayLineCount(2);

  main_menu.add_screen(main_screen);
  main_menu.add_screen(stp_screen);

  main_menu.update();
  main_menu.set_focusedLine(0);
}

void poll_menu(void) {
  bool updscreen = true;

  encoder.tick();

  if (encoder.isRight()) {
    multiplier = 1;
    m_cnt = 1;
    if (navigate) {
      main_menu.switch_focus();
    } else {
      if (!main_menu.is_callable(increase)) {
        updscreen = false;
        //main_menu.next_screen();
      } else {
        updscreen = false;
        main_menu.call_function(increase);
      }
    }
  } else if (encoder.isLeft()) {
    multiplier = 1;
    m_cnt = 1;
    if (navigate) {
      main_menu.switch_focus(false);
    } else {
      if (!main_menu.is_callable(decrease)) {
        updscreen = false;
        //main_menu.previous_screen();
      } else {
        updscreen = false;
        main_menu.call_function(decrease);
      }
    }
  } else if (encoder.isRightH()) {
    multiplier = 10;
    m_cnt++;
    updscreen = false;
    main_menu.call_function(increase);
  } else if (encoder.isLeftH()) {
    multiplier = 10;
    m_cnt++;
    updscreen = false;
    main_menu.call_function(decrease);
  } else if (encoder.isClick()) {
    //main_menu.switch_focus();
    if (main_menu.get_currentScreen() == &main_screen) {
      if (main_menu.get_focusedLine() == 0) {
        updscreen = false;
        main_menu.change_screen(&stp_screen);
        main_menu.set_focusedLine(0);
        main_menu.update();
      } else if (main_menu.get_focusedLine() >= 1 && main_menu.get_focusedLine() <= 4) {
        //Переключаем 4 реле
        updscreen = false;
        set_rele_state_to_array(main_menu.get_focusedLine(), !bit_is_set(rele_state, main_menu.get_focusedLine() - 1));
        main_menu.update();
      }
    } else if (main_menu.get_currentScreen() == &stp_screen) {
      if (main_menu.get_focusedLine() == 4) {
        updscreen = false;
        backFunction();
      } else if (main_menu.get_focusedLine() == 3) {
        if (get_stepper_state()) stop_stepper();
        else start_stepper(true);
      } else if (main_menu.get_focusedLine() == 1) {
        dirFunction();
      }
      else navigate = !navigate;
    }
    else navigate = !navigate;
  }
  byte currS = millis() / 1000;
  if (currS != oldS) {
    if (updscreen) main_menu.softUpdate();
    oldS = millis() / 1000;
  }
}


#endif
