#ifndef __I2CSTEPPER_H
#define __I2CSTEPPER_H

#define I2CSTEPPER_VERSION 0.5

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
#define STEPPER_TARGET_LIMIT 2147483647UL


//Пины для реле
#define MIXER_PUMP_PIN 13 // RELE_PIN1
#define RELE_PIN2 10      // RELE_PIN2
#define RELE_PIN3 11      // RELE_PIN3
#define RELE_PIN4 12      // RELE_PIN4

// Пины для Encoder
#define ENC_CLK 7 //S2
#define ENC_DT 8  //S1
#define ENC_SW 9  //KEY

#ifndef EXT_SENSOR_PIN
#define EXT_SENSOR_PIN 6
#endif

#ifndef EXT_SENSOR_INPUT_MODE
#define EXT_SENSOR_INPUT_MODE INPUT_PULLUP
#endif

#define EEPROM_SIZE 200
#define I2CSTEPPER_EEPROM_MARKER 0x53
#define I2CSTEPPER_PROTO_VERSION 2
#define I2CSTEPPER_REG_COUNT 48

#define I2CSTEPPER_CAP_MIXER   0x01
#define I2CSTEPPER_CAP_PUMP    0x02
#define I2CSTEPPER_CAP_FILLING 0x04
#define I2CSTEPPER_CAP_RELAY   0x08
#define I2CSTEPPER_CAP_SENSOR  0x10

#define I2CSTEPPER_STATUS_RUNNING     0x01
#define I2CSTEPPER_STATUS_PAUSED      0x02
#define I2CSTEPPER_STATUS_SENSOR      0x04
#define I2CSTEPPER_STATUS_CALIBRATION 0x08
#define I2CSTEPPER_STATUS_ERROR       0x80

#define I2CSTEPPER_FLAG_REVERSE_AFTER_PAUSE 0x01
#define I2CSTEPPER_FLAG_SMOOTH_START        0x02
#define I2CSTEPPER_FLAG_DIRECTION           0x04

#define I2CSTEPPER_SENSOR_ACTIVE_HIGH 0x01
#define I2CSTEPPER_SENSOR_STOP        0x02
#define I2CSTEPPER_SENSOR_PUMP_PAUSE  0x04

enum I2CStepperRegister : uint8_t {
  REG_MAGIC = 0,
  REG_VERSION = 1,
  REG_CAPS = 2,
  REG_ROLE = 3,
  REG_MODE = 4,
  REG_COMMAND = 5,
  REG_COMMAND_SEQ = 6,
  REG_ACK_SEQ = 7,
  REG_STATUS = 8,
  REG_ERROR = 9,
  REG_RELAY_MASK = 10,
  REG_SENSOR_FLAGS = 11,
  REG_OPTION_FLAGS = 12,
  REG_MIXER_RPM_H = 13,
  REG_MIXER_RPM_L = 14,
  REG_MIXER_RUN_H = 15,
  REG_MIXER_RUN_L = 16,
  REG_MIXER_PAUSE_H = 17,
  REG_MIXER_PAUSE_L = 18,
  REG_PUMP_MLH_H = 19,
  REG_PUMP_MLH_L = 20,
  REG_PUMP_PAUSE_H = 21,
  REG_PUMP_PAUSE_L = 22,
  REG_FILL_ML_H = 23,
  REG_FILL_ML_L = 24,
  REG_FILL_MLH_H = 25,
  REG_FILL_MLH_L = 26,
  REG_STEPS_PER_ML_H = 27,
  REG_STEPS_PER_ML_L = 28,
  REG_REMAINING_3 = 29,
  REG_REMAINING_2 = 30,
  REG_REMAINING_1 = 31,
  REG_REMAINING_0 = 32,
  REG_CURRENT_SPEED_H = 33,
  REG_CURRENT_SPEED_L = 34,
  REG_LOCK = 47,
};

enum I2CStepperCommand : uint8_t {
  I2CSTEP_CMD_NONE = 0,
  I2CSTEP_CMD_APPLY = 1,
  I2CSTEP_CMD_START = 2,
  I2CSTEP_CMD_STOP = 3,
  I2CSTEP_CMD_SAVE = 4,
  I2CSTEP_CMD_CALIBRATE_START = 5,
  I2CSTEP_CMD_CALIBRATE_FINISH = 6,
  I2CSTEP_CMD_RELAY = 7,
};

enum I2CStepperError : uint8_t {
  I2CSTEP_ERR_NONE = 0,
  I2CSTEP_ERR_UNSUPPORTED_MODE = 1,
  I2CSTEP_ERR_BAD_CONFIG = 2,
  I2CSTEP_ERR_COMM_TIMEOUT = 3,
};


// Объявляем переменные и константы:
struct SetupEEPROM {
  byte marker;
  byte version;
  byte role;
  byte mode;
  uint16_t mixerRpm;
  uint16_t mixerRunSec;
  uint16_t mixerPauseSec;
  uint16_t pumpMlHour;
  uint16_t pumpPauseSec;
  uint16_t fillingMl;
  uint16_t fillingMlHour;
  uint16_t stepperStepMl;
  byte optionFlags;
  byte sensorFlags;
  byte relayMask;
};


enum I2CType {I2CMIXER = 1, I2CPUMP = 2, I2CFILLING = 3};


//#define GS_FAST_PROFILE 10
//#define SMOOTH_ALGORITHM
#include <GyverStepper.h>
GStepper< STEPPER2WIRE> stepper(STEPPER_STEPS, STEPPER_STEP, STEPPER_DIR, STEPPER_EN); // объект для работы с шаговым двигателем

Encoder               encoder(ENC_CLK, ENC_DT, ENC_SW, TYPE2); // объект для работы с энкодером
iarduino_I2C_connect  I2C2;                            // объект I2C2 для работы c библиотекой iarduino_I2C2_connect
SetupEEPROM           I2CSTPSetup;                     // структура для хранения настроек
volatile byte         REG_Array[I2CSTEPPER_REG_COUNT]; // массив регистров v2 для чтения/записи по шине I2C
uint32_t              set_spd;                         // храним значение установленной скорости
volatile uint16_t     curr_spd;                        // храним предыдущую установленную скорость
uint32_t              set_time;                        // храним значение установленного времени
uint32_t              last_set_time;                   // храним предудущее значение установленного времени
bool                  set_time_initialized;            // флаг инициализации set_time
byte                  set_dir;                         // храним значение направления вращения шаговика
bool                  set_dir_initialized;             // флаг инициализации set_dir
byte                  last_dir;                        // храним предыдущее значение направления вращения шаговика
bool                  stepper_state;                   // храним статус шаговика
byte                  rele_state;                      // байт для статусов 4 реле
byte                  last_applied_mask;                // маска, согласованная между слейвом и мастером в прошлой транзакции (для двустороннего обмена по реле)
uint8_t               rele_pin[] = {MIXER_PUMP_PIN, RELE_PIN2, RELE_PIN3, RELE_PIN4}; //описание пинов реле
uint32_t              pause_deadline_ms;
bool                  pause_phase;
byte                  command_seq_seen;
bool                  calibration_active;


#endif // __I2CSTEPPER_H
