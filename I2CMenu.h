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


void set_direction_to_array(byte dir);
int get_stepper_state();
const char* get_stepper_state_c();
const char* get_mixer_pump_state();
bool set_mixer_pump_state(bool state);
uint32_t get_speed(void);
byte get_direction(void);
uint32_t get_stepper_time(void);
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
const char* get_stp_type();
uint32_t get_stp_ml();
bool set_rele_state_to_array(byte r, bool s);
void write_config();

void(* resetFunc) (void) = 0;

//строки для меню
const char c_On[] PROGMEM =  "On ";
const char c_Off[] PROGMEM = "Off";
const char c_Mixer[] PROGMEM = "Mixer";
const char c_Pump[] PROGMEM =  "Pump ";
const char c_None[] PROGMEM =  "None ";

const char str_BACK[] = "<BACK";
const char str_STP[]  = "STP> ";
const char str_Pmp[]  = ">Pump: ";
const char str_R2[]  = ">Rele2: ";
const char str_R3[]  = ">Rele3: ";
const char str_R4[]  = ">Rele4: ";
const char str_SET[]  = "SETUP>";
const char str_STP_Spd[]  = "STP Spd: ";
const char str_STP_Dir[]  = "STP Dir: ";
const char str_STP_Time[]  = "STP Time: ";
const char str_STP_Ml[]  = "STP ML: ";
const char str_STP_Start[]  = "STP Start: ";
const char str_SET_Type[]  = "Type: ";
const char str_SET_Stp_Ml[]  = "STP/ML: ";
char* str_STP_Measure = str_STP_Time;

LiquidLine back_line(10, 6, str_BACK);

LiquidLine main_line1(0, 0, str_STP, get_stepper_time);
LiquidLine main_line2(0, 1, str_Pmp, get_mixer_pump_state);
LiquidLine main_line3(0, 2, str_R2, get_rele_state2);
LiquidLine main_line4(0, 3, str_R3, get_rele_state3);
LiquidLine main_line5(0, 4, str_R4, get_rele_state4);
LiquidLine main_line6(0, 5, str_SET);
LiquidScreen main_screen(main_line1, main_line2);

LiquidLine stp_line_spd(0, 0, str_STP_Spd, get_speed);
LiquidLine stp_line_dir(0, 1, str_STP_Dir, get_direction);
LiquidLine stp_line_time(0, 2, str_STP_Measure, get_stepper_time);
LiquidLine stp_line_start(0, 3, str_STP_Start, get_stepper_state_c);
LiquidScreen stp_screen(stp_line_spd, stp_line_dir, stp_line_time, stp_line_start);

LiquidLine setup_line1(0, 0, str_SET_Type, get_stp_type);
LiquidLine setup_line2(0, 1, str_SET_Stp_Ml, get_stp_ml);
LiquidScreen setup_screen(setup_line1, setup_line2, back_line);

LiquidMenu main_menu(lcd);

uint32_t get_stp_ml() {
  return I2CSTPSetup.StepperStepMl;
}

const char* get_c_ptr(const char* p_str) {
  static char buf_g[6];
  strcpy_P(buf_g, p_str);
  return  buf_g;
}

const char* get_stp_type() {
  if (I2CSTPSetup.Type == I2CMIXER) return get_c_ptr(c_Mixer);
  else if (I2CSTPSetup.Type == I2CPUMP) return get_c_ptr(c_Pump);
  else return get_c_ptr(c_None);
}

const char* get_rele_state2() {
  if (bit_is_set(rele_state, 1)) {
    return get_c_ptr(c_On);
  }
  else {
    return get_c_ptr(c_Off);
  }
}

const char* get_rele_state3() {
  if (bit_is_set(rele_state, 2)) return get_c_ptr(c_On);
  else return get_c_ptr(c_Off);
}

const char* get_rele_state4() {
  if (bit_is_set(rele_state, 3)) return get_c_ptr(c_On);
  else return get_c_ptr(c_Off);
}

const char* get_mixer_pump_state() {
  if (bit_is_set(rele_state, 0)) return get_c_ptr(c_On);
  else return get_c_ptr(c_Off);
}

const char* get_stepper_state_c() {
  if (stepper_state) return get_c_ptr(c_On);
  else return get_c_ptr(c_Off);
}

int get_stepper_state() {
  if (stepper_state) return 1;
  else return 0;
}

// Used for attaching something to the lines, to make them focusable.
void change_type() {
  I2CSTPSetup.Type++;
  if (I2CSTPSetup.Type > I2CPUMP ) I2CSTPSetup.Type = I2CMIXER;
}

// Used for attaching something to the lines, to make them focusable.
void blankFunction() {
  return;
}

//функция для возврата в основное меню
void backFunction() {
  //Если выходим из настроек, их нужно сохранить
  if (main_menu.get_currentScreen() == &setup_screen) {
    write_config();
  }
  main_menu.change_screen(&main_screen);
  main_menu.update();
}

//инкремент шагов на мл
void spd_ml_IncFunction() {
  byte c = m_cnt;
  if (c < 3) c = 1;
  else c = c / 3;
  I2CSTPSetup.StepperStepMl += 1 * multiplier * c;
}

//декремент шагов на мл
void spd_ml_DecFunction() {
  byte c = m_cnt;
  if (c < 3) c = 1;
  else c = c / 3;
  if (I2CSTPSetup.StepperStepMl <= 1 * multiplier * c) {
    I2CSTPSetup.StepperStepMl = 1;
  } else I2CSTPSetup.StepperStepMl -= 1 * multiplier * c;
}

//инкремент скорости шаговика
void spdIncFunction() {
  byte c = m_cnt;
  uint32_t s;
  if (c < 3) c = 1;
  else c = c / 3;
  set_spd += 1 * multiplier * c;
  if (set_spd > STEPPER_MAX_SPEED) set_spd = STEPPER_MAX_SPEED;
  if (stepper_state) {
    s = get_spd_stp(set_spd);
    set_speed_to_array(s);
  }
#ifdef __I2CStepper_DEBUG
  Serial.print(F("SSSSSetSpd = "));
  Serial.println(set_spd);
  Serial.print(F("Set spd = "));
  Serial.println(s);
  Serial.print(F("Get spd from array = "));
  Serial.println(get_speed_from_array());
#endif
}

//декремент скорости шаговика
void spdDecFunction() {
  byte c = m_cnt;
  uint32_t s;
  if (c < 3) c = 1;
  else c = c / 3;
  if (set_spd <= 1 * multiplier * c) set_spd = 1;
  else set_spd -= 1 * multiplier * c;
  if (stepper_state) {
    s = get_spd_stp(set_spd);
    set_speed_to_array(s);
  }
#ifdef __I2CStepper_DEBUG
  Serial.print(F("SSSSSetSpd = "));
  Serial.println(set_spd);
  Serial.print(F("Set spd = "));
  Serial.println(s);
  Serial.print(F("Get spd from array = "));
  Serial.println(get_speed_from_array());
#endif
}

//инкремент времени работы шаговика
void timeIncFunction() {
  byte c = m_cnt;
  uint32_t target;
  if (c < 3) c = 1;
  else c = c / 3;
  set_time += 1 * multiplier * c;
  if (set_time > 100000) set_time = 100000;
  last_set_time = set_time;

  if (stepper_state) {
    uint16_t spd = get_speed_from_array();
    target = (uint32_t)set_time * spd;
    set_target_to_array(target);
    byte savePrescale;
    //остановим первый таймер
    savePrescale = TCCR1B & (0b111 << CS10);
    TCCR1B &= ~(0b111 << CS10);
    stepper.setTarget((long)target + stepper.getCurrent());
    //продолжим первый таймер
    TCCR1B |= savePrescale;
  }
#ifdef __I2CStepper_DEBUG
  Serial.print(F("Set time = "));
  Serial.println(set_time);
  Serial.print(F("Set target = "));
  Serial.println(target);
  Serial.print(F("Get target from array = "));
  Serial.println(get_target_from_array());
#endif
}

//декремент времени работы шаговика
void timeDecFunction() {
  byte c = m_cnt;
  uint32_t target;
  if (c < 3) c = 1;
  else c = c / 3;
  if (set_time <= 1 * multiplier * c) set_time = 0;
  else set_time -= 1 * multiplier * c;
  last_set_time = set_time;

  if (stepper_state) {
    uint16_t spd = get_speed_from_array();
    target = (uint32_t)set_time * spd;
    set_target_to_array(target);
    byte savePrescale;
    //остановим первый таймер
    savePrescale = TCCR1B & (0b111 << CS10);
    TCCR1B &= ~(0b111 << CS10);
    stepper.setTarget((long)target + stepper.getCurrent());
    //продолжим первый таймер
    TCCR1B |= savePrescale;
  }
#ifdef __I2CStepper_DEBUG
  Serial.print(F("Set time = "));
  Serial.println(set_time);
  Serial.print(F("Set target = "));
  Serial.println(target);
  Serial.print(F("Get target from array = "));
  Serial.println(get_target_from_array());
#endif
}

//изменение направления вращения шаговика
void dirFunction() {
  set_dir = !set_dir;
  set_direction_to_array(set_dir);
}

//инициализация меню
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
  main_screen.add_line(main_line6);
  main_line3.attach_function(1, blankFunction);
  main_line4.attach_function(1, blankFunction);
  main_line5.attach_function(1, blankFunction);
  main_line6.attach_function(1, blankFunction);

  stp_screen.add_line(back_line);

  stp_line_spd.attach_function(increase, spdIncFunction);
  stp_line_spd.attach_function(decrease, spdDecFunction);

  stp_line_dir.attach_function(increase, backFunction);
  stp_line_dir.attach_function(decrease, backFunction);

  stp_line_time.attach_function(increase, timeIncFunction);
  stp_line_time.attach_function(decrease, timeDecFunction);

  stp_line_start.attach_function(increase, blankFunction);
  stp_line_start.attach_function(decrease, blankFunction);

  setup_line1.attach_function(increase, change_type);
  setup_line1.attach_function(decrease, change_type);
  setup_line2.attach_function(increase, spd_ml_IncFunction);
  setup_line2.attach_function(decrease, spd_ml_DecFunction);

  main_screen.set_displayLineCount(2);
  stp_screen.set_displayLineCount(2);
  setup_screen.set_displayLineCount(2);

  main_menu.add_screen(main_screen);
  main_menu.add_screen(stp_screen);
  main_menu.add_screen(setup_screen);

  main_menu.change_screen(&main_screen);
  main_menu.update();
  main_menu.set_focusedLine(0);
}

//функция для опроса состояния энкодера и обработки меню
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
      } else if (main_menu.get_focusedLine() == 5) {
        updscreen = false;
        main_menu.change_screen(&setup_screen);
        main_menu.set_focusedLine(0);
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
    } else if (main_menu.get_currentScreen() == &setup_screen) {
      if (main_menu.get_focusedLine() == 2) {
        updscreen = false;
        backFunction();
      }
      else navigate = !navigate;
    }
    else navigate = !navigate;
  }
  byte currS = millis() / 1000;
  if (currS != oldS) {
    if (updscreen) main_menu.softUpdate();
    oldS = millis() / 1000;
#ifdef __I2CStepper_DEBUG
    Serial.print(F("spd = "));
    Serial.print(get_speed_from_array());
    Serial.print(F("; target = "));
    Serial.println(get_target_from_array());
#endif
  }
}
#endif
