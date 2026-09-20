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

//#define LIQUIDMENU_DEBUG true

#include <LiquidCrystal_I2C2.h>
#include <LiquidMenu.h>
#include <avr/wdt.h>

namespace glyph {
extern uint8_t rightFocus[8];
extern uint8_t customFocus[8];
}

#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

enum FunctionTypes {
  increase = 1,
  decrease = 2,
};

LiquidCrystal_I2C2 lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

byte multiplier;
byte m_cnt;
bool navigate;
uint32_t oldS;


void set_motion_direction(byte dir);
int get_stepper_state();
const char* get_stepper_state_c();
const char* get_mixer_pump_state();
bool set_mixer_pump_state(bool state);
uint32_t get_speed(void);
byte get_direction(void);
uint32_t get_stepper_time(void);
void start_stepper(bool from_int);
void stop_stepper();
uint32_t get_motion_target(void);
uint16_t get_spd_stp(uint32_t spd);
void set_motion_speed(uint16_t spd);
uint16_t get_motion_speed(void);
void set_motion_target(uint32_t target);
uint32_t calc_target_from_time(uint32_t time_value, uint16_t spd);
const char* get_rele_state2();
const char* get_rele_state3();
const char* get_rele_state4();
const char* get_stp_type();
uint32_t get_stp_ml();
bool set_rele_state(byte r, bool s);
uint32_t get_max_user_speed(void);
uint32_t get_min_user_speed(void);
bool write_config();
uint8_t pause_stepper_timer(void);
void resume_stepper_timer(uint8_t prescale);
void apply_local_motion_settings(void);
bool v3_local_controls_locked(void);
void v3_local_stop(void);

#define STEPPER_STEP_ML_MIN 100UL

//строки для меню
const char c_On[] PROGMEM =  "On ";
const char c_Off[] PROGMEM = "Off";
const char c_Mixer[] PROGMEM = "Mixer";
const char c_Pump[] PROGMEM =  "Pump ";
const char c_Fill[] PROGMEM =  "Fill ";
const char c_None[] PROGMEM =  "None ";

const char str_BACK[] PROGMEM = "<BACK";
const char str_Pmp[]  PROGMEM = "Pump:";
const char str_R2[]  PROGMEM = "Rele2:";
const char str_R3[]  PROGMEM = "Rele3:";
const char str_R4[]  PROGMEM = "Rele4:";
const char str_SET[]  PROGMEM = "SETUP>";
const char str_STP_Spd[]  PROGMEM = "STP Spd:";
const char str_STP_Dir[]  PROGMEM = "STP Dir:";
const char str_STP_Start[] PROGMEM = "STP Start:";
const char str_SET_Type[] PROGMEM = "Type:";
const char str_SET_Stp_Ml[] PROGMEM = "STP/ML:";
const char str_SET_Address[] PROGMEM = "I2C Adr:";
//общий буфер текста меню: LiquidMenu печатает результат геттера сразу после вызова
static char menu_text[17];

const char* format_motion_line(bool submenu) {
  if (I2CSTPSetup.mode == I2CMIXER) {
    if (submenu) strcpy_P(menu_text, PSTR("STP T:"));
    else strcpy_P(menu_text, PSTR("STP Time:"));
    ultoa(get_stepper_time(), menu_text + strlen(menu_text), 10);
  } else if (I2CSTPSetup.mode == I2CFILLING) {
    strcpy_P(menu_text, PSTR("STP ML:"));
    ultoa(get_stepper_time(), menu_text + strlen(menu_text), 10);
  } else {
    strcpy_P(menu_text, PSTR("Continuous"));
  }
  if (submenu) {
    char* end = menu_text + strlen(menu_text);
    *end++ = '>';
    *end = '\0';
  }
  return menu_text;
}

const char* get_main_motion_line() {
  return format_motion_line(true);
}

const char* get_stp_motion_line() {
  return format_motion_line(false);
}

LiquidLine back_line(10, 6, str_BACK);

LiquidLine main_line1(0, 0, get_main_motion_line);
LiquidLine main_line2(0, 1, str_Pmp, get_mixer_pump_state);
LiquidLine main_line3(0, 2, str_R2, get_rele_state2);
LiquidLine main_line4(0, 3, str_R3, get_rele_state3);
LiquidLine main_line5(0, 4, str_R4, get_rele_state4);
LiquidLine main_line6(0, 5, str_SET);
LiquidScreen main_screen(main_line1, main_line2);

LiquidLine stp_line_spd(0, 0, str_STP_Spd, get_speed);
LiquidLine stp_line_dir(0, 1, str_STP_Dir, get_direction);
LiquidLine stp_line_time(0, 2, get_stp_motion_line);
LiquidLine stp_line_start(0, 3, str_STP_Start, get_stepper_state_c);
LiquidScreen stp_screen(stp_line_spd, stp_line_dir, stp_line_time, stp_line_start);

LiquidLine setup_line1(0, 0, str_SET_Type, get_stp_type);
LiquidLine setup_line2(0, 1, str_SET_Stp_Ml, get_stp_ml);
uint8_t get_i2c_address() { return v3_staging_config.address; }
LiquidLine setup_line3(0, 2, str_SET_Address, get_i2c_address);
LiquidScreen setup_screen(setup_line1, setup_line2, setup_line3, back_line);

//«грязный» флаг — пишем конфиг и перезагружаемся только если в setup что-то реально поменялось
bool setup_dirty = false;
struct SetupSnapshot {
  byte role;
  byte mode;
  uint32_t stepperStepMl;
};
SetupSnapshot setup_snapshot;

void snapshot_setup() {
  setup_snapshot.role = I2CSTPSetup.role;
  setup_snapshot.mode = I2CSTPSetup.mode;
  setup_snapshot.stepperStepMl = I2CSTPSetup.stepperStepMl;
}

LiquidMenu main_menu(lcd);

uint32_t get_stp_ml() {
  return I2CSTPSetup.stepperStepMl;
}

const char* get_c_ptr(const char* p_str) {
  strcpy_P(menu_text, p_str);
  return menu_text;
}

const char* get_stp_type() {
  if (I2CSTPSetup.mode == I2CMIXER) return get_c_ptr(c_Mixer);
  else if (I2CSTPSetup.mode == I2CPUMP) return get_c_ptr(c_Pump);
  else if (I2CSTPSetup.mode == I2CFILLING) return get_c_ptr(c_Fill);
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
  if (v3_local_controls_locked()) return;
  byte mode = i2cstepper_v3_address_is_mixer(v3_staging_config.address)
                  ? I2CMIXER
                  : (I2CSTPSetup.mode == I2CPUMP ? I2CFILLING : I2CPUMP);
  if (mode == I2CSTPSetup.mode) return;
  I2CSTPSetup.mode = mode;
  I2CSTPSetup.role = (mode == I2CMIXER) ? I2CMIXER : I2CPUMP;
  setup_dirty = true;
}

void change_address(bool increase) {
  if (v3_local_controls_locked()) return;
  uint8_t oldAddress = v3_staging_config.address;
  uint8_t newAddress = oldAddress;
  if (increase) {
    if (newAddress >= I2CSTEPPER_V3_ADDRESS_MAX) return;
    newAddress++;
  } else {
    if (newAddress <= I2CSTEPPER_V3_ADDRESS_MIN) return;
    newAddress--;
  }
  uint8_t newMode = 0;
  if (!i2cstepper_v3_mode_after_address_change(oldAddress, newAddress,
                                                I2CSTPSetup.mode, &newMode)) return;
  v3_staging_config.address = newAddress;
  I2CSTPSetup.mode = newMode;
  I2CSTPSetup.role = (newMode == I2CMIXER) ? I2CMIXER : I2CPUMP;
  setup_dirty = true;
}

void address_inc() { change_address(true); }
void address_dec() { change_address(false); }

// Used for attaching something to the lines, to make them focusable.
void blankFunction() {
  return;
}

static bool setup_changed(void) {
  return I2CSTPSetup.role != setup_snapshot.role ||
         I2CSTPSetup.mode != setup_snapshot.mode ||
         I2CSTPSetup.stepperStepMl != setup_snapshot.stepperStepMl;
}

void set_menu_editing(bool editing) {
  navigate = !editing;
  lcd.createChar(15, editing ? glyph::customFocus : glyph::rightFocus);
  main_menu.update();
}

void show_save_error() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Invalid config"));
  delay(1000);
  main_menu.update();
}

bool save_stp_value() {
  byte line = main_menu.get_focusedLine();
  uint32_t* value = NULL;
  if (line == 0) {
    value = I2CSTPSetup.mode == I2CMIXER ? &I2CSTPSetup.mixerRpm :
            (I2CSTPSetup.mode == I2CPUMP ? &I2CSTPSetup.pumpMlHour : &I2CSTPSetup.fillingMlHour);
  } else if (line == 2) {
    value = I2CSTPSetup.mode == I2CMIXER ? &I2CSTPSetup.mixerRunSec : &I2CSTPSetup.fillingMl;
  }
  if (value != NULL) *value = line == 0 ? set_spd : set_time;
  return write_config();
}

bool save_setup_value() {
  if (!write_config()) return false;
  snapshot_setup();
  setup_dirty = false;
  return true;
}

void reboot_after_save() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Saved. Reboot!"));
  delay(500);
  wdt_enable(WDTO_15MS);
  while (true) {
  }
}

//функция для возврата в основное меню
void backFunction() {
  if (v3_local_controls_locked()) return;
  //Если выходим из настроек и там реально что-то поменялось — сохранить и перезагрузиться (меняется I2C-адрес)
  if (main_menu.get_currentScreen() == &setup_screen) {
    if (setup_dirty || setup_changed()) {
      if (!write_config()) {
        show_save_error();
        return;
      }
      reboot_after_save();
    }
  }
  main_menu.change_screen(&main_screen);
  set_menu_editing(false);
}

void change_steps_per_ml(bool increase_value) {
  if (v3_local_controls_locked()) return;
  byte c = m_cnt;
  if (c < 3) c = 1;
  else c = c / 3;
  uint32_t delta = 1UL * multiplier * c;
  if (increase_value) {
    I2CSTPSetup.stepperStepMl += delta;
  } else if (I2CSTPSetup.stepperStepMl <= STEPPER_STEP_ML_MIN + delta) {
    I2CSTPSetup.stepperStepMl = STEPPER_STEP_ML_MIN;
  } else {
    I2CSTPSetup.stepperStepMl -= delta;
  }
  setup_dirty = true;
}

//инкремент шагов на 100 мл
void spd_ml_IncFunction() { change_steps_per_ml(true); }

//декремент шагов на 100 мл
void spd_ml_DecFunction() { change_steps_per_ml(false); }

void change_speed(bool increase_value) {
  if (v3_local_controls_locked()) return;
  byte c = m_cnt;
  uint32_t max_spd = get_max_user_speed();
  uint32_t min_spd = get_min_user_speed();
  if (c < 3) c = 1;
  else c = c / 3;
  uint32_t delta = 1UL * multiplier * c;
  if (increase_value) set_spd += delta;
  else if (set_spd <= min_spd + delta) set_spd = min_spd;
  else set_spd -= delta;
  if (set_spd > max_spd) set_spd = max_spd;
  if (set_spd < min_spd) set_spd = min_spd;
  if (stepper_state) {
    uint16_t s = get_spd_stp(set_spd);
    set_motion_speed(s);
    apply_local_motion_settings();
  }
#ifdef __I2CStepper_DEBUG
  Serial.print(F("SSSSSetSpd = "));
  Serial.println(set_spd);
  Serial.print(F("Set spd = "));
  if (stepper_state) Serial.println(get_spd_stp(set_spd));
  else Serial.println(F("n/a"));
  Serial.print(F("Get spd from array = "));
  Serial.println(get_motion_speed());
#endif
}

//инкремент скорости шаговика
void spdIncFunction() { change_speed(true); }

//декремент скорости шаговика
void spdDecFunction() { change_speed(false); }

void change_time(bool increase_value) {
  if (v3_local_controls_locked()) return;
  if (I2CSTPSetup.mode == I2CPUMP) return;
  byte c = m_cnt;
  if (c < 3) c = 1;
  else c = c / 3;
  uint32_t delta = 1UL * multiplier * c;
  if (increase_value) {
    set_time += delta;
    if (set_time > 100000) set_time = 100000;
  } else if (I2CSTPSetup.mode == I2CFILLING && set_time <= delta) {
    set_time = 1;
  } else if (set_time <= delta) {
    set_time = 0;
  } else {
    set_time -= delta;
  }
  set_time_initialized = true;
  last_set_time = set_time;

  if (stepper_state) {
    uint16_t spd = get_motion_speed();
    uint32_t target = calc_target_from_time(set_time, spd);
    set_motion_target(target);
    uint8_t saved_prescale = pause_stepper_timer();
    //target <= STEPPER_TARGET_LIMIT, поэтому сумма с текущей позицией считается без 64 бит
    int32_t current = stepper.getCurrent();
    uint32_t absolute_target;
    if (current < 0) {
      int32_t sum = (int32_t)target + current;
      absolute_target = sum < 0 ? 0 : (uint32_t)sum;
    } else {
      absolute_target = target + (uint32_t)current;
      if (absolute_target > STEPPER_TARGET_LIMIT) absolute_target = STEPPER_TARGET_LIMIT;
    }
    stepper.setTarget((long)absolute_target);
    resume_stepper_timer(saved_prescale);
  }
#ifdef __I2CStepper_DEBUG
  Serial.print(F("Set time = "));
  Serial.println(set_time);
  Serial.print(F("Set target = "));
  if (stepper_state) Serial.println(calc_target_from_time(set_time, get_motion_speed()));
  else Serial.println(F("n/a"));
  Serial.print(F("Get target from array = "));
  Serial.println(get_motion_target());
#endif
}

//инкремент времени работы шаговика
void timeIncFunction() { change_time(true); }

//декремент времени работы шаговика
void timeDecFunction() { change_time(false); }

//изменение направления вращения шаговика
void dirFunction() {
  if (v3_local_controls_locked()) return;
  set_dir = !set_dir;
  set_dir_initialized = true;
  set_motion_direction(set_dir);
  apply_local_motion_settings();
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

  stp_line_dir.attach_function(increase, blankFunction);
  stp_line_dir.attach_function(decrease, blankFunction);

  stp_line_time.attach_function(increase, timeIncFunction);
  stp_line_time.attach_function(decrease, timeDecFunction);

  stp_line_start.attach_function(increase, blankFunction);
  stp_line_start.attach_function(decrease, blankFunction);

  setup_line1.attach_function(increase, change_type);
  setup_line1.attach_function(decrease, change_type);
  setup_line2.attach_function(increase, spd_ml_IncFunction);
  setup_line2.attach_function(decrease, spd_ml_DecFunction);
  setup_line3.attach_function(increase, address_inc);
  setup_line3.attach_function(decrease, address_dec);

  main_screen.set_displayLineCount(2);
  stp_screen.set_displayLineCount(2);
  setup_screen.set_displayLineCount(2);

  back_line.set_asProgmem(1);
  main_line2.set_asProgmem(1);
  main_line3.set_asProgmem(1);
  main_line4.set_asProgmem(1);
  main_line5.set_asProgmem(1);
  main_line6.set_asProgmem(1);
  stp_line_spd.set_asProgmem(1);
  stp_line_dir.set_asProgmem(1);
  stp_line_start.set_asProgmem(1);
  setup_line1.set_asProgmem(1);
  setup_line2.set_asProgmem(1);
  setup_line3.set_asProgmem(1);

  
  main_menu.add_screen(main_screen);
  main_menu.add_screen(stp_screen);
  main_menu.add_screen(setup_screen);

  main_menu.change_screen(&main_screen);
  main_menu.update();
  main_menu.set_focusedLine(0);
  set_menu_editing(false);
  snapshot_setup();
  setup_dirty = false;
}

//функция для опроса состояния энкодера и обработки меню
void poll_menu(void) {
  bool updscreen = true;

  //Самовар управляет приводом: меню заблокировано, показываем ход работы и даём локальный STOP
  static bool remote_screen = false;
  if (v3_local_controls_locked()) {
    if (encoder.isClick()) {
      v3_local_stop();
      return;
    }
    uint32_t currS = millis() / 1000;
    if (!remote_screen || currS != oldS) {
      if (!remote_screen) {
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(F("Click = STOP"));
        remote_screen = true;
      }
      oldS = currS;
      lcd.setCursor(0, 0);
      byte len = lcd.print(format_motion_line(false));
      for (; len < LCD_COLUMNS; len++) lcd.write(' ');
    }
    return;
  }
  if (remote_screen) {
    remote_screen = false;
    main_menu.update();
  }

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
    if (!navigate) main_menu.call_function(increase);
  } else if (encoder.isLeftH()) {
    multiplier = 10;
    m_cnt++;
    updscreen = false;
    if (!navigate) main_menu.call_function(decrease);
  } else if (encoder.isClick()) {
    //main_menu.switch_focus();
    if (main_menu.get_currentScreen() == &main_screen) {
      if (main_menu.get_focusedLine() == 0) {
        updscreen = false;
        main_menu.change_screen(&stp_screen);
        main_menu.set_focusedLine(0);
        set_menu_editing(false);
      } else if (main_menu.get_focusedLine() >= 1 && main_menu.get_focusedLine() <= 4) {
        //Переключаем 4 реле
        updscreen = false;
        set_rele_state(main_menu.get_focusedLine(), !bit_is_set(rele_state, main_menu.get_focusedLine() - 1));
        main_menu.update();
      } else if (main_menu.get_focusedLine() == 5) {
        updscreen = false;
        setup_dirty = false;
        snapshot_setup();
        main_menu.change_screen(&setup_screen);
        main_menu.set_focusedLine(0);
        set_menu_editing(false);
      }
    } else if (main_menu.get_currentScreen() == &stp_screen) {
      if (main_menu.get_focusedLine() == 4) {
        updscreen = false;
        backFunction();
      } else if (main_menu.get_focusedLine() == 3) {
        if (get_stepper_state()) v3_local_stop();
        else start_stepper(true);
      } else if (main_menu.get_focusedLine() == 1) {
        dirFunction();
        updscreen = false;
        if (!save_stp_value()) show_save_error();
        else main_menu.update();
      } else if (main_menu.get_focusedLine() == 2 && I2CSTPSetup.mode == I2CPUMP) {
        updscreen = false;
        main_menu.update();
      } else if (navigate) {
        updscreen = false;
        set_menu_editing(true);
      } else {
        updscreen = false;
        if (!save_stp_value()) show_save_error();
        else set_menu_editing(false);
      }
    } else if (main_menu.get_currentScreen() == &setup_screen) {
      if (main_menu.get_focusedLine() == 3) {
        updscreen = false;
        backFunction();
      } else if (navigate) {
        updscreen = false;
        set_menu_editing(true);
      } else {
        updscreen = false;
        bool address_changed = v3_staging_config.address != v3_runtime_address;
        if (!save_setup_value()) {
          show_save_error();
        } else if (address_changed) {
          reboot_after_save();
        } else {
          set_menu_editing(false);
        }
      }
    }
  }
  uint32_t currS = millis() / 1000;
  if (currS != oldS) {
    if (((currS / 10) * 10) == currS) {
      main_menu.update();
    } else if (updscreen) {
      main_menu.softUpdate();
    }
    oldS = millis() / 1000;
#ifdef __I2CStepper_DEBUG
    Serial.print(F("set_time = "));
    Serial.print(get_stepper_time());
    Serial.print(F("; set_spd = "));
    Serial.print(get_speed());
    Serial.print(F("; spd = "));
    Serial.print(get_motion_speed());
    Serial.print(F("; Stepper spd = "));
    Serial.print(stepper.getSpeed());
    Serial.print(F("; Stepper Time = "));
    Serial.print(stepper.stepTime);
    Serial.print(F("; target = "));
    Serial.println(get_motion_target());
#endif
  }
}
#endif
