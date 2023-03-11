#define I2CSTEPPER_VERSION 0.1

#define LIQUIDMENU_LIBRARY LiquidCrystal_I2C2_LIBRARY
#define DisplayClass LiquidCrystal_I2C2
#define DRIVER_STEP_TIME 5

// Подключаем библиотеки:
#include <Arduino.h>
#include <Wire.h>                                     // подключаем библиотеку для работы с шиной I2C
#include <iarduino_I2C_connect.h>                     // подключаем библиотеку для соединения arduino по шине I2C
#include <GyverStepper.h>
#include "PinChangeInterrupt.h"
#include <GyverEncoder.h>
#include <TimerOne.h>

// Пины для I2C Master - подключение экрана и потенциально других устройств
// SDA_PIN 0 //A0
// SCL_PIN 2 //D2

// Пины для I2C Slave - связь с Samovar
// SDA_PIN A4
// SCL_PIN A5

// Пины для шагового двигателя
#define STEPPER_STEP 3
#define STEPPER_DIR 4
#define STEPPER_EN 5

// Настройки для шагового двигателя
#define STEPPER_MS 2
#define STEPPER_STEPS (200 * STEPPER_MS) //количество шагов, 200 x MS
#define STEPPER_MAX_SPEED 1200


//Пины для реле
#define MIXER_PUMP_PIN 13 // RELE_PIN1
#define RELE_PIN2 10      // RELE_PIN2
#define RELE_PIN3 11      // RELE_PIN3
#define RELE_PIN4 12      // RELE_PIN4

// Пины для Encoder
#define ENC_CLK 7 //S2
#define ENC_DT 8  //S1
#define ENC_SW 9  //KEY

/*
   FREE PIN
   A1
   A2
   A3
   A6
   A7
   D6
*/


GStepper< STEPPER2WIRE> stepper(STEPPER_STEPS, STEPPER_STEP, STEPPER_DIR, STEPPER_EN);

// Объявляем переменные и константы:
iarduino_I2C_connect I2C2;                            // объект I2C2 для работы c библиотекой iarduino_I2C2_connect
byte                 REG_Array[9];                    // массив, данные которого будут доступны мастеру (для чтения/записи) по шине I2C
uint16_t             set_spd;                         // храним значение установленной скорости
volatile uint16_t    curr_spd;                        // храним предыдущую установленную скорость
uint16_t             set_time;                        // храним значение установленного времени
byte                 set_dir;                         // храним значение направления вращения шаговика
bool                 stepper_state;                   // храним статус шаговика
byte                 rele_state;                      // байт для статусов 4 реле
uint8_t              rele_pin[] = {MIXER_PUMP_PIN, RELE_PIN2, RELE_PIN3, RELE_PIN4};//описание пинов реле

Encoder encoder(ENC_CLK, ENC_DT, ENC_SW, TYPE2);

#include "I2CMenu.h"

void isrENK() {
  encoder.tick();  // отработка в прерывании
}

void setup() {
  Serial.begin(115200);
  attachPCINT(digitalPinToPCINT(ENC_CLK), isrENK, CHANGE); //вешаем прерывания для обработки энкодера
  attachPCINT(digitalPinToPCINT(ENC_DT), isrENK, CHANGE);  //вешаем прерывания для обработки энкодера
  attachPCINT(digitalPinToPCINT(ENC_SW), isrENK, CHANGE);  //вешаем прерывания для обработки энкодера

  Wire.begin(0x01);                                   // инициируем подключение к шине I2C в качестве ведомого (slave) устройства, с указанием своего адреса на шине.
  Wire2.begin();                                      // инициируем подключение к шине I2C в качестве мастера
  I2C2.begin(REG_Array);                              // инициируем возможность чтения/записи данных по шине I2C, из/в указываемый массив
  stepper.setRunMode(FOLLOW_POS);
  REG_Array[8] = 0;
  pinMode(MIXER_PUMP_PIN, OUTPUT);                    // используем ногу для вывода
  pinMode(RELE_PIN2, OUTPUT);                         // используем ногу для вывода
  pinMode(RELE_PIN3, OUTPUT);                         // используем ногу для вывода
  pinMode(RELE_PIN4, OUTPUT);                         // используем ногу для вывода

  menu_init();                                        // инициализуерм меню экрана

  stepper.brake();                                    // тормозим шаговик
  stepper.disable();                                  // отключаем шаговик

  Timer1.initialize(40);                              // инициализируем таймер для упарвления шаговиком
  Timer1.attachInterrupt(stp_tick);

  Serial.println("Ready");
}

void stp_tick(void)
{
  stepper.tick();
}

//возвращаем время, оставшееся до конца работы шаговика для отображения на экране
float get_stepper_time(void) {
  if (set_time == 0 || stepper_state) set_time = get_stepper_time_from_array();
  return (uint16_t)set_time;
}

//возвращаем время, оставшееся до конца работы шаговика
uint16_t get_stepper_time_from_array(void) {
  float t;
  float s = get_speed_from_array();
  if (s == 0) return 0;
  t = (float)get_target_from_array() / s;
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
  return (float)spd * STEPPER_STEPS / 60;
}

//возвращаем скорость в оборотах/мин из шагов в секунду
float get_speed(void) {
  if (set_spd == 0) set_spd = get_speed_from_array() / STEPPER_STEPS * 60;
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
  Serial.println("======================");
  Serial.print("Toggle rele ");
  Serial.print(r);
  Serial.print("; set ");
  Serial.println(s);
  Serial.println("======================");
  return s;
}

//запускаем шаговик. Есть два варианта - через меню и через установленные значения в массиве.
void start_stepper(bool from_int) {
  uint32_t target;
  uint16_t spd;
  byte dir;

  if (from_int) {
    spd = get_spd_stp(set_spd);
    target = (uint32_t)set_time * spd;
    dir = set_dir;
    set_speed_to_array(spd);
    set_direction_to_array(dir);
    set_target_to_array(target);
    set_spd = 0;
    set_time = 0;
  } else {
    target = get_target_from_array();
    spd = get_speed_from_array();
    dir = get_direction_from_array();
  }

  if (spd == 0 || target == 0) return;

  Serial.println("======================");
  Serial.println("START STP");
  Serial.println(spd);
  Serial.println(target);
  Serial.println("======================");

  stepper.setRunMode(FOLLOW_POS);
  stepper.setAcceleration(STEPPER_STEPS);

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
  Serial.println("======================");
  Serial.println("FINISH STP");
  Serial.println("======================");
}

//основной цикл
void loop() {
  uint32_t target;
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

  stepper_state = stepper.getState();

  if (target > 0 && spd > 0) {
    if (!stepper_state) {
      start_stepper(false);
    } else {
      if (spd != curr_spd) {
        stepper.setAcceleration(0);
        stepper.setRunMode(KEEP_SPEED);
        stepper.setMaxSpeed(spd);
        stepper.setSpeed(spd, false);
        curr_spd = spd;
        set_spd = (float)get_speed_from_array() / STEPPER_STEPS * 60;;
      }
    }
    uint32_t crnt_trg = (uint32_t)stepper.getTarget() - (uint32_t)stepper.getCurrent();
    if (target != crnt_trg) {
      target = crnt_trg;
      set_target_to_array(target);
    }
    if (target == 0) {
      stop_stepper();
    }
  } else {
    if (stepper_state) {
      stop_stepper();
    }
  }
}
