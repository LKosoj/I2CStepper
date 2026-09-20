#ifndef __I2CSTEPPER_H
#define __I2CSTEPPER_H

#include <I2CStepperV3.h>

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
#define STEPPER_MAX_SPEED I2CSTEPPER_V3_MAX_SPEED_STEPS_PER_SEC
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
#define I2CSTEPPER_FLAG_REVERSE_AFTER_PAUSE 0x01
#define I2CSTEPPER_FLAG_SMOOTH_START        0x02
#define I2CSTEPPER_FLAG_DIRECTION           0x04

#define I2CSTEPPER_SENSOR_ACTIVE_HIGH 0x01
#define I2CSTEPPER_SENSOR_STOP        0x02
#define I2CSTEPPER_SENSOR_PUMP_PAUSE  0x04

// Объявляем переменные и константы:
struct SetupEEPROM {
  byte marker;
  byte version;
  byte role;
  byte mode;
  uint32_t mixerRpm;
  uint32_t mixerRunSec;
  uint32_t mixerPauseSec;
  uint32_t pumpMlHour;
  uint32_t pumpPauseSec;
  uint32_t fillingMl;
  uint32_t fillingMlHour;
  uint32_t stepperStepMl;
  byte optionFlags;
  byte sensorFlags;
  byte relayMask;
};


enum I2CType {I2CMIXER = 1, I2CPUMP = 2, I2CFILLING = 3};


//#define GS_FAST_PROFILE 10
#include <GyverStepper2.h>
GStepper2< STEPPER2WIRE> stepper(STEPPER_STEPS, STEPPER_STEP, STEPPER_DIR, STEPPER_EN); // объект для работы с шаговым двигателем

Encoder               encoder(ENC_CLK, ENC_DT, ENC_SW, TYPE2); // объект для работы с энкодером
SetupEEPROM           I2CSTPSetup;                     // структура для хранения настроек
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
uint8_t               rele_pin[] = {MIXER_PUMP_PIN, RELE_PIN2, RELE_PIN3, RELE_PIN4}; //описание пинов реле
uint32_t              pause_deadline_ms;
bool                  pause_phase;
byte                  command_seq_seen;
bool                  calibration_active;
bool                  v3_motion_continuous;
bool                  v3_mixer_deadline_active;
uint32_t              v3_mixer_deadline_ms;

I2CStepperV3Config    v3_active_config;
I2CStepperV3Config    v3_staging_config;
I2CStepperV3Motion    v3_staging_motion;
I2CStepperV3StatusSnapshot v3_status_snapshot;
uint8_t               v3_rx_config_a[I2CSTEPPER_V3_CONFIG_A_SIZE];
uint8_t               v3_rx_config_b[I2CSTEPPER_V3_CONFIG_B_SIZE];
uint8_t               v3_rx_motion[I2CSTEPPER_V3_MOTION_SIZE];
uint8_t               v3_rx_command[I2CSTEPPER_V3_COMMAND_SIZE];
volatile bool         v3_rx_config_a_pending;
volatile bool         v3_rx_config_b_pending;
volatile bool         v3_rx_motion_pending;
volatile bool         v3_rx_command_pending;
uint8_t               v3_read_register;
uint8_t               v3_identity_frame[I2CSTEPPER_V3_IDENTITY_SIZE];
uint8_t               v3_status_frame[I2CSTEPPER_V3_STATUS_SIZE];
uint8_t               v3_config_a_frame[I2CSTEPPER_V3_CONFIG_A_SIZE];
uint8_t               v3_config_b_frame[I2CSTEPPER_V3_CONFIG_B_SIZE];
uint8_t               v3_motion_frame[I2CSTEPPER_V3_MOTION_SIZE];
uint32_t              v3_last_sequence;
uint32_t              v3_last_heartbeat_ms;
uint8_t               v3_runtime_address;
uint8_t               v3_runtime_mode;
bool                  v3_remote_owner;
bool                  v3_movement_allowed;

enum {
  I2CSTEPPER_V3_GLOBAL_BYTES = sizeof(v3_active_config) + sizeof(v3_staging_config) +
    sizeof(v3_staging_motion) + sizeof(v3_status_snapshot) + sizeof(v3_rx_config_a) +
    sizeof(v3_rx_config_b) + sizeof(v3_rx_motion) + sizeof(v3_rx_command) +
    sizeof(v3_rx_config_a_pending) + sizeof(v3_rx_config_b_pending) +
    sizeof(v3_rx_motion_pending) + sizeof(v3_rx_command_pending) + sizeof(v3_read_register) +
    sizeof(v3_identity_frame) + sizeof(v3_status_frame) + sizeof(v3_config_a_frame) +
    sizeof(v3_config_b_frame) + sizeof(v3_motion_frame) + sizeof(v3_last_sequence) +
    sizeof(v3_last_heartbeat_ms) + sizeof(v3_runtime_address) + sizeof(v3_runtime_mode) + sizeof(v3_remote_owner) + sizeof(v3_movement_allowed) +
    sizeof(v3_motion_continuous) + sizeof(v3_mixer_deadline_active) + sizeof(v3_mixer_deadline_ms),
};
// v3 wire snapshots and four independent write slots are bounded well below Nano SRAM.
static_assert(I2CSTEPPER_V3_GLOBAL_BYTES <= 320U, "v3 globals exceed Nano RAM budget");


#endif // __I2CSTEPPER_H
