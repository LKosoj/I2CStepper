//TODO вынести создание строк меню в функцию инициализации и убрать строки в PROGMEM

#define LIQUIDMENU_LIBRARY LiquidCrystal_I2C2_LIBRARY
#define DisplayClass LiquidCrystal_I2C2
#define DRIVER_STEP_TIME 5

// Подключаем библиотеки:
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>                                     // подключаем библиотеку для работы с шиной I2C
#include <EEPROM.h>
#include <iarduino_I2C_connect.h>                     // подключаем библиотеку для соединения arduino по шине I2C

#include "PinChangeInterrupt.h"
#include <GyverEncoder.h>
#include <TimerOne.h>

#include "I2CStepper.h"

#include "I2CMenu.h"

#ifndef __I2CStepper_DEBUG
#define ARDUINOTRACE_ENABLE 0  // Disable all traces
#endif
#include <ArduinoTrace.h>

void read_config();
void write_config();

void isrENK() {
  encoder.tick();  // отработка в прерывании
}

void setup() {
  stepper.brake();                                    // тормозим шаговик
  stepper.disable();                                  // отключаем шаговик

  Serial.begin(115200);
  attachPCINT(digitalPinToPCINT(ENC_CLK), isrENK, CHANGE); //вешаем прерывания для обработки энкодера
  attachPCINT(digitalPinToPCINT(ENC_DT), isrENK, CHANGE);  //вешаем прерывания для обработки энкодера
  attachPCINT(digitalPinToPCINT(ENC_SW), isrENK, CHANGE);  //вешаем прерывания для обработки энкодера

  read_config();

  Wire2.begin();                                      // инициируем подключение к шине I2C в качестве мастера
  I2C2.begin(REG_Array);                              // инициируем возможность чтения/записи данных по шине I2C, из/в указываемый массив
  //  stepper.setRunMode(FOLLOW_POS);
  REG_Array[8] = 0;
  pinMode(MIXER_PUMP_PIN, OUTPUT);                    // используем ногу для вывода
  pinMode(RELE_PIN2, OUTPUT);                         // используем ногу для вывода
  pinMode(RELE_PIN3, OUTPUT);                         // используем ногу для вывода
  pinMode(RELE_PIN4, OUTPUT);                         // используем ногу для вывода

  menu_init();                                        // инициализуерм меню экрана

  Timer1.initialize(40);                              // инициализируем таймер для упарвления шаговиком
  Timer1.attachInterrupt(stp_tick);

  if (I2CSTPSetup.Type == I2CMIXER) {
    Serial.print(F("Mixer "));
  } else if (I2CSTPSetup.Type == I2CPUMP) {
    Serial.print(F("Pump "));
  }
  Serial.println(F("ready"));
#ifdef __I2CStepper_DEBUG
  set_spd = 490;
  set_time = 2000;
  last_set_time = set_time;
#endif
}

void stp_tick(void)
{
  stepper.tick();
}

//возвращаем время или миллилитры, оставшиеся до конца работы шаговика для отображения на экране
uint32_t get_stepper_time(void) {
  if (set_time == 0 || stepper_state) set_time = get_stepper_time_from_array();
  return (uint32_t)set_time;
}

//возвращаем время или миллилитры, оставшиеся до конца работы шаговика
uint16_t get_stepper_time_from_array(void) {
  float t;
  if (I2CSTPSetup.Type == I2CMIXER) {
    //если время
    float s = get_speed_from_array();
    if (s == 0) return 0;
    t = (float)get_target_from_array() / s;
  } else if (I2CSTPSetup.Type == I2CPUMP) {
    //если миллилитры
    t = (float)get_target_from_array() * 100 / I2CSTPSetup.StepperStepMl;
  }
  return t;
}

//возвращаем количество шагов
uint32_t get_target_from_array(void) {
  while (REG_Array[8] == 1) {
    delay(1);
  }
  uint32_t t = (uint32_t)REG_Array[3] << 24;
  t += (uint32_t)REG_Array[4] << 16;
  t += (uint32_t)REG_Array[5] << 8;
  t += (uint32_t)REG_Array[6];
  return t;
}

//сохраняем количество шагов шаговика в массив для обмена с Самоваром
void set_target_to_array(uint32_t target) {
  REG_Array[3] = target >> 24;
  REG_Array[4] = target >> 16;
  REG_Array[5] = target >> 8;
  REG_Array[6] = target;
}

//возвращаем скрость в шагах в секунду из скорости в оборотах/мин
uint16_t get_spd_stp(uint16_t spd) {
  float s;
  if (I2CSTPSetup.Type == I2CMIXER) {
    s = (float)spd * STEPPER_STEPS / 60;
  } else if (I2CSTPSetup.Type == I2CPUMP) {
    s = (float)spd * I2CSTPSetup.StepperStepMl / 3600;
  }
  return s;
}

//возвращаем скорость в оборотах/мин или литры в час
uint32_t get_speed(void) {
  if (I2CSTPSetup.Type == I2CMIXER) {
    //в об/мин
    if (set_spd == 0 || stepper_state) set_spd = ((float)get_speed_from_array() / STEPPER_STEPS * 60 + 0.4);
  } else if (I2CSTPSetup.Type == I2CPUMP) {
    //в миллилитрах в час
    if (set_spd == 0 || stepper_state) set_spd = ((float)get_speed_from_array() + 0.8) * 3600 / I2CSTPSetup.StepperStepMl;
  }
  return set_spd;
}

//получаем скорость в шагах в секунду из массива
uint16_t get_speed_from_array(void) {
  while (REG_Array[8] == 1) {
    delay(1);
  }
  uint16_t t = (uint16_t)REG_Array[0] << 8;
  t += (uint16_t)REG_Array[1];
  return t;
}

//сохраняем скорость в массив для обмена с Самоваром
void set_speed_to_array(uint16_t spd) {
  REG_Array[0] = spd >> 8;
  REG_Array[1] = spd;
}

//получаем направление движения для отображения на экране
byte get_direction(void) {
  if (set_dir == 0) set_dir = get_direction_from_array();
  return set_dir;
}

//получаем направление движения из массива
byte get_direction_from_array(void) {
  while (REG_Array[8] == 1) {
    delay(1);
  }
  return REG_Array[2];
}

//сохраняем направление движения в массив для обмена с Самоваром
void set_direction_to_array(byte dir) {
  REG_Array[2] = dir;
}

//сохраняем состояние насоса в массив для обмена с Самоваром
bool set_mixer_pump_state(bool state) {
  set_rele_state_to_array(1, state);
  return state;
}

//получаем состояние насоса из массива
byte get_mixer_pump_from_array(void) {
  return get_rele_state_from_array(1);
}

//получаем состояние реле по номеру из массива
bool get_rele_state_from_array(byte r) {
  while (REG_Array[8] == 1) {
    delay(1);
  }
  return bit_is_set(REG_Array[7], r - 1);
}

//сохраняем состояние реле по номеру в массив для обмена с Самоваром
bool set_rele_state_to_array(byte r, bool s) {
  while (REG_Array[8] == 1) {
    delay(1);
  }
  bitWrite(REG_Array[7], r - 1, s);
  bitWrite(rele_state, r - 1, s);
  digitalWrite(rele_pin[r - 1], s);
#ifdef __I2CStepper_DEBUG
  Serial.println(F("======================"));
  Serial.print(F("Toggle rele "));
  Serial.print(r);
  Serial.print(F("; set "));
  Serial.println(s);
  Serial.println(F("======================"));
#endif
  return s;
}

//запускаем шаговик. Есть два варианта - через меню и через установленные значения в массиве.
void start_stepper(bool from_int) {
  uint32_t target;
  uint16_t spd;
  byte dir;

  if (from_int) {
    spd = get_spd_stp(set_spd);
    if (I2CSTPSetup.Type == I2CMIXER) {
      target = (uint32_t)set_time * spd;
    }
    if (I2CSTPSetup.Type == I2CPUMP) {
      target = (uint32_t)set_time * I2CSTPSetup.StepperStepMl / 100;
    }
    dir = set_dir;
    set_speed_to_array(spd);
    set_direction_to_array(dir);
    set_target_to_array(target);
  } else {
    target = get_target_from_array();
    spd = get_speed_from_array();
    dir = get_direction_from_array();
  }
  last_dir = dir;

  if (spd == 0 || target == 0) return;

#ifdef __I2CStepper_DEBUG
  Serial.println(F("======================"));
  Serial.println(F("START STP"));
  Serial.println(spd);
  Serial.println(target);
  Serial.println(F("======================"));
#endif

  //  stepper.setRunMode(FOLLOW_POS);
  stepper.setAcceleration(spd / 10);

  //  stepper.setAcceleration(0);
  //  Serial.println("===No Acceleration====");

  stepper.enable();
  stepper.reverse(dir);
  stepper.setCurrent(0);
  stepper.setMaxSpeed(spd);
  stepper.setSpeed(spd, true);
  stepper.setTarget((long)target);
  curr_spd = spd;
}

//останавливаем шаговик
void stop_stepper() {
  set_target_to_array(0);

  stepper.brake();
  stepper.disable();
  stepper.setCurrent(0);

#ifdef __I2CStepper_DEBUG
  Serial.println(F("======================"));
  Serial.println(F("FINISH STP"));
  Serial.println(F("======================"));
#endif
}

//основной цикл
void loop() {
  //TRACE();
  uint32_t target;
  uint32_t t;
  uint16_t spd;

  //опрашиваем состояние энкодера и работаем с меню
  poll_menu();

  //проверяем статус реле и переключаем, если текущий статус отличается от установленного в массиве
  for (byte i = 0; i < 4; i++) {
    if (bit_is_set(rele_state, i) != bit_is_set(REG_Array[7], i)) {
      set_rele_state_to_array(i + 1, bit_is_set(REG_Array[7], i));
    }
  }

  //обрабатываем измененения статуса шаговика
  target = get_target_from_array();
  spd = get_speed_from_array();
  byte dir = get_direction_from_array();
  stepper_state = stepper.getState();

  t = 0;
  //шаговик крутит
  if (target > 0 && spd > 0) {
    if (!stepper_state) {
      //если в массиве указаны параметры скорости и времени и шаговик не крутит - значит пришли значения от Самовара, запустим
      start_stepper(false);
    } else {
      //шаговик крутится, но текущая скорость отличается от скорости в массиве, установим заданную в массиве
      if (spd != curr_spd) {
        stepper.setMaxSpeed(spd);
        t = stepper.getTarget();
        stepper.setSpeed(spd, true);
        curr_spd = spd;
        set_spd = get_speed();
#ifdef __I2CStepper_DEBUG
        Serial.print(F("On changed SSSSSetSpd = "));
        Serial.println(set_spd);
        Serial.print(F("Changed spd = "));
        Serial.println(spd);
#endif
      }
    }

    //если новое значение направления отличается от предыдущего - останавливаем шаговик и начинаем крутить в другую сторону
    if (dir != last_dir) {
      stepper.brake();
      stepper.reverse(dir);
      last_dir = dir;
      stepper.enable();
      stepper.setMaxSpeed(spd);
      stepper.setSpeed(spd, true);
    }

    //если режим - миксер и предыдущее оставшееся время отличается от текущего оставшегося времени больше, чем на 1 секунду в любую сторону,
    //значит это значение пришло от Самовара и нужно синхронизироваться с ним
    uint16_t set_time = get_stepper_time();
    if (I2CSTPSetup.Type == I2CMIXER && abs((float)set_time - (float)last_set_time) > 1) {
      target = (uint32_t)set_time * spd;
      t = (long)target + stepper.getCurrent();
    }

    //если режим - насос и предыдущий оставшийся объем отличается от текущего оставшегося больше, чем на количество отобранных миллилитров на этой скорости за секунду,
    //значит это значение пришло от Самовара и нужно синхронизироваться с ним
    //    if (I2CSTPSetup.Type == I2CPUMP && abs((float)set_time - (float)last_set_time) > 1) {
    //      target = (uint32_t)set_time * I2CSTPSetup.StepperStepMl / 100;
    //      byte savePrescale;
    //      //остановим первый таймер
    //      savePrescale = TCCR1B & (0b111 << CS10);
    //      TCCR1B &= ~(0b111 << CS10);
    //      stepper.setTarget((long)target + stepper.getCurrent());
    //      //продолжим первый таймер
    //      TCCR1B |= savePrescale;
    //    }

    if (t != 0) {
      byte savePrescale;
      //остановим первый таймер
      savePrescale = TCCR1B & (0b111 << CS10);
      TCCR1B &= ~(0b111 << CS10);
      stepper.setAcceleration(get_speed_from_array() / 10);
      stepper.setTarget(t);
      //продолжим первый таймер
      TCCR1B |= savePrescale;
    }

    last_set_time = set_time;

    uint32_t crnt_trg = (uint32_t)stepper.getTarget() - (uint32_t)stepper.getCurrent();
    //синхронизируем текущее положение шаговика с массивом для передачи данных Самовару
    target = crnt_trg;
    set_target_to_array(target);
    //stepper.setTarget((long)target);
    //}
    if (target == 0) {
      stop_stepper();
    }
  } else {
    if (stepper_state) {
      stop_stepper();
    }
  }
}

void read_config() {
  EEPROM.get(0, I2CSTPSetup);

  if (isnan(I2CSTPSetup.Type)) {
    I2CSTPSetup.Type = I2CMIXER;
    write_config();
  }

  if (I2CSTPSetup.StepperStepMl > 80000 || I2CSTPSetup.StepperStepMl < 100) {
    I2CSTPSetup.StepperStepMl = 4500;
    write_config();
  }

  Wire.begin(I2CSTPSetup.Type);                       // инициируем подключение к шине I2C в качестве ведомого (slave) устройства, с указанием своего адреса на шине.
  //устанавливаем в меню нужный тип изменерения
  if (I2CSTPSetup.Type == I2CMIXER) {
    str_STP_Measure = str_STP_Time;
  } else if (I2CSTPSetup.Type == I2CPUMP) {
    str_STP_Measure = str_STP_Ml;
  }
  main_menu.update();
}

void write_config() {
  EEPROM.put(0, I2CSTPSetup);
}
