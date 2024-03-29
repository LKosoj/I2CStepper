#ifndef __I2CSTEPPER_H
#define __I2CSTEPPER_H

#define I2CSTEPPER_VERSION 0.4

//#define __I2CStepper_DEBUG

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
#define STEPPER_MAX_SPEED 80000


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

#define EEPROM_SIZE 200


// Объявляем переменные и константы:
struct SetupEEPROM {
  byte Type;
  uint32_t StepperStepMl;
};


enum I2CType {I2CMIXER = 1, I2CPUMP = 2};


//#define GS_FAST_PROFILE 10
//#define SMOOTH_ALGORITHM
#include <GyverStepper.h>
GStepper< STEPPER2WIRE> stepper(STEPPER_STEPS, STEPPER_STEP, STEPPER_DIR, STEPPER_EN); // объект для работы с шаговым двигателем

Encoder               encoder(ENC_CLK, ENC_DT, ENC_SW, TYPE2); // объект для работы с энкодером
iarduino_I2C_connect  I2C2;                            // объект I2C2 для работы c библиотекой iarduino_I2C2_connect
SetupEEPROM           I2CSTPSetup;                     // структура для хранения настроек
byte                  REG_Array[9];                    // массив, данные которого будут доступны мастеру (для чтения/записи) по шине I2C
uint16_t              set_spd;                         // храним значение установленной скорости
volatile uint16_t     curr_spd;                        // храним предыдущую установленную скорость
uint16_t              set_time;                        // храним значение установленного времени
uint16_t              last_set_time;                   // храним предудущее значение установленного времени
byte                  set_dir;                         // храним значение направления вращения шаговика
byte                  last_dir;                        // храним предыдущее значение направления вращения шаговика
bool                  stepper_state;                   // храним статус шаговика
byte                  rele_state;                      // байт для статусов 4 реле
uint8_t               rele_pin[] = {MIXER_PUMP_PIN, RELE_PIN2, RELE_PIN3, RELE_PIN4}; //описание пинов реле


#endif __I2CSTEPPER_H
