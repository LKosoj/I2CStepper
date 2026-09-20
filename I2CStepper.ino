#define LIQUIDMENU_LIBRARY LiquidCrystal_I2C2_LIBRARY
#define DisplayClass LiquidCrystal_I2C2
#define DRIVER_STEP_TIME 5

// Подключаем библиотеки:
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>                                     // подключаем библиотеку для работы с шиной I2C
#include <EEPROM.h>

#include "PinChangeInterrupt.h"
#include <GyverEncoder.h>
#include <avr/interrupt.h>

#include "I2CStepper.h"
#include "I2CStepperRuntime.h"
#include "StepperMath.h"

#include "I2CMenu.h"

#ifndef __I2CStepper_DEBUG
#define ARDUINOTRACE_ENABLE 0  // Disable all traces
#endif
#include <ArduinoTrace.h>

void read_config();
bool write_config();
uint32_t calc_target_from_time(uint32_t time_value, uint16_t spd);
void start_current_mode();
void finish_calibration();
bool external_sensor_active();
void update_runtime_state();
void v3_wire_receive(int count);
void v3_wire_request();
void v3_publish_frames();
void v3_process_receive();
void v3_local_stop();
bool v3_local_controls_locked();
static uint8_t lock_interrupts();
static void unlock_interrupts(uint8_t sreg);
static void stepper_snapshot_position_atomic(int32_t* current, int32_t* target_abs);
static void start_motion(uint16_t spd, uint32_t target, byte dir, bool continuous);
static void stop_motion(bool smooth);
static void finish_mixer_run_phase();
static void timer1_disarm();
static void timer1_schedule(uint32_t periodUs);
static bool v3_apply_staged_relay();
static bool v3_start_configured_motion();
static uint8_t v3_start_precondition_error();
static uint8_t v3_apply_address_error();
static bool v3_take_mailbox(const uint8_t* source, uint8_t size, volatile bool* pending,
                            uint8_t* destination);

static const uint16_t V3_EEPROM_OFFSET = 32;
static const uint8_t V3_EEPROM_HEADER_SIZE = 5;
static const uint8_t V3_EEPROM_PAYLOAD_SIZE = I2CSTEPPER_V3_CONFIG_A_SIZE + I2CSTEPPER_V3_CONFIG_B_SIZE;
static const uint8_t V3_EEPROM_SIZE = V3_EEPROM_HEADER_SIZE + V3_EEPROM_PAYLOAD_SIZE;

static uint8_t v3_capabilities() {
  return i2cstepper_v3_address_is_mixer(v3_runtime_address)
           ? (I2CSTEPPER_V3_CAP_MIXER | I2CSTEPPER_V3_CAP_RELAY | I2CSTEPPER_V3_CAP_SENSOR)
           : (I2CSTEPPER_V3_CAP_PUMP | I2CSTEPPER_V3_CAP_FILLING | I2CSTEPPER_V3_CAP_RELAY | I2CSTEPPER_V3_CAP_SENSOR);
}

static void v3_default_config(I2CStepperV3Config* config) {
  config->address = 1;
  config->mode = I2CSTEPPER_V3_MODE_MIXER;
  // Только для пустой или испорченной EEPROM. На уже настроенной плате плавный разгон берётся из
  // сохранённых настроек и меняется строкой Smooth в SETUP или галочкой в Самоваре.
  config->optionFlags = I2CSTEPPER_FLAG_SMOOTH_START;
  config->sensorFlags = I2CSTEPPER_SENSOR_STOP;
  config->relayMask = 0;
  config->mixerRpm = 20;
  config->mixerRunSec = 0;
  config->mixerPauseSec = 0;
  config->pumpMlHour = 100;
  config->pumpPauseSec = 0;
  config->fillingMl = 100;
  config->fillingMlHour = 100;
  config->stepsPerMl = 16000;
}

static I2CStepperV3EepromState v3_eeprom_read(I2CStepperV3Config* config) {
  uint8_t record[V3_EEPROM_SIZE];
  for (uint8_t i = 0; i < V3_EEPROM_SIZE; i++) record[i] = EEPROM.read(V3_EEPROM_OFFSET + i);
  I2CStepperV3EepromState state = i2cstepper_v3_eeprom_record_state(
      record, V3_EEPROM_SIZE, I2CSTEPPER_V3_MAGIC, I2CSTEPPER_V3_VERSION,
      V3_EEPROM_PAYLOAD_SIZE, V3_EEPROM_HEADER_SIZE);
  if (state != I2CSTEPPER_V3_EEPROM_VALID) return state;
  i2cstepper_v3_decode_config_a(&record[V3_EEPROM_HEADER_SIZE], config);
  i2cstepper_v3_decode_config_b(&record[V3_EEPROM_HEADER_SIZE + I2CSTEPPER_V3_CONFIG_A_SIZE], config);
  return I2CSTEPPER_V3_EEPROM_VALID;
}

static bool v3_eeprom_write(const I2CStepperV3Config& config) {
  uint8_t record[V3_EEPROM_SIZE];
  record[0] = I2CSTEPPER_V3_MAGIC;
  record[1] = I2CSTEPPER_V3_VERSION;
  record[2] = V3_EEPROM_PAYLOAD_SIZE;
  i2cstepper_v3_encode_config_a(&record[V3_EEPROM_HEADER_SIZE], &config);
  i2cstepper_v3_encode_config_b(&record[V3_EEPROM_HEADER_SIZE + I2CSTEPPER_V3_CONFIG_A_SIZE], &config);
  uint16_t crc = i2cstepper_v3_crc16(&record[V3_EEPROM_HEADER_SIZE], V3_EEPROM_PAYLOAD_SIZE);
  record[3] = (uint8_t)(crc >> 8);
  record[4] = (uint8_t)crc;
  for (uint8_t i = 0; i < V3_EEPROM_SIZE; i++) EEPROM.update(V3_EEPROM_OFFSET + i, record[i]);
  for (uint8_t i = 0; i < V3_EEPROM_SIZE; i++) {
    if (EEPROM.read(V3_EEPROM_OFFSET + i) != record[i]) return false;
  }
  return true;
}

void v3_wire_receive(int count) {
  if (count < 1 || count > (int)I2CSTEPPER_V3_WIRE_BUFFER_SIZE) {
    while (Wire.available()) Wire.read();
    return;
  }
  if (count == 1) {
    int value = Wire.read();
    if (value >= 0) v3_read_register = (uint8_t)value;
    while (Wire.available()) Wire.read();
    return;
  }
  int registerValue = Wire.read();
  if (registerValue < 0) return;
  const uint8_t reg = (uint8_t)registerValue;
  uint8_t* mailbox = 0;
  volatile bool* pending = 0;
  uint8_t payloadSize = 0;
  if (reg == I2CSTEPPER_V3_REG_CONFIG_A) {
    mailbox = v3_rx_config_a;
    pending = &v3_rx_config_a_pending;
    payloadSize = I2CSTEPPER_V3_CONFIG_A_SIZE;
  } else if (reg == I2CSTEPPER_V3_REG_CONFIG_B) {
    mailbox = v3_rx_config_b;
    pending = &v3_rx_config_b_pending;
    payloadSize = I2CSTEPPER_V3_CONFIG_B_SIZE;
  } else if (reg == I2CSTEPPER_V3_REG_MOTION) {
    mailbox = v3_rx_motion;
    pending = &v3_rx_motion_pending;
    payloadSize = I2CSTEPPER_V3_MOTION_SIZE;
  } else if (reg == I2CSTEPPER_V3_REG_COMMAND) {
    mailbox = v3_rx_command;
    pending = &v3_rx_command_pending;
    payloadSize = I2CSTEPPER_V3_COMMAND_SIZE;
  }
  if (!mailbox || count != (int)payloadSize + 1 || *pending) {
    while (Wire.available()) Wire.read();
    return;
  }
  for (uint8_t index = 0; index < payloadSize; index++) {
    if (!Wire.available()) return;
    mailbox[index] = (uint8_t)Wire.read();
  }
  if (Wire.available()) {
    while (Wire.available()) Wire.read();
    return;
  }
  v3_read_register = reg;
  *pending = true;
}

void v3_wire_request() {
  if (v3_read_register == I2CSTEPPER_V3_REG_IDENTITY) {
    Wire.write(v3_identity_frame, I2CSTEPPER_V3_IDENTITY_SIZE);
  } else if (v3_read_register == I2CSTEPPER_V3_REG_STATUS) {
    Wire.write(v3_status_frame, I2CSTEPPER_V3_STATUS_SIZE);
  } else if (v3_read_register == I2CSTEPPER_V3_REG_CONFIG_A) {
    Wire.write(v3_config_a_frame, I2CSTEPPER_V3_CONFIG_A_SIZE);
  } else if (v3_read_register == I2CSTEPPER_V3_REG_CONFIG_B) {
    Wire.write(v3_config_b_frame, I2CSTEPPER_V3_CONFIG_B_SIZE);
  } else if (v3_read_register == I2CSTEPPER_V3_REG_MOTION) {
    Wire.write(v3_motion_frame, I2CSTEPPER_V3_MOTION_SIZE);
  }
}

static void v3_set_result(uint32_t sequence, uint8_t result, uint8_t error) {
  i2cstepper_v3_acknowledge_sequence(sequence, &v3_status_snapshot.commandSeq,
                                      &v3_status_snapshot.ackSeq);
  v3_status_snapshot.commandResult = result;
  v3_status_snapshot.error = error;
}

static bool v3_config_valid(const I2CStepperV3Config& config) {
  if (!i2cstepper_v3_address_valid(config.address) ||
      !i2cstepper_v3_mode_supported(config.address, config.mode) ||
      config.stepsPerMl == 0 || (config.relayMask & 0xF0U) != 0) return false;
  uint64_t speed = 0;
  uint64_t target = 0;
  if (config.mode == I2CSTEPPER_V3_MODE_MIXER) {
    speed = stepper_rate_steps(config.mixerRpm, STEPPER_STEPS, 60U);
    if (config.mixerRunSec > 0) target = speed * config.mixerRunSec;
  } else if (config.mode == I2CSTEPPER_V3_MODE_PUMP) {
    speed = stepper_rate_steps(config.pumpMlHour, config.stepsPerMl, 3600U);
  } else {
    speed = stepper_rate_steps(config.fillingMlHour, config.stepsPerMl, 3600U);
    target = (uint64_t)config.fillingMl * config.stepsPerMl;
  }
  bool targetRequired = config.mode == I2CSTEPPER_V3_MODE_FILLING ||
                        (config.mode == I2CSTEPPER_V3_MODE_MIXER && config.mixerRunSec > 0);
  return i2cstepper_v3_derived_config_valid(speed, target, targetRequired,
                                             I2CSTEPPER_V3_MAX_SPEED_STEPS_PER_SEC,
                                             I2CSTEPPER_V3_TARGET_STEPS_MAX);
}

// Выставляет выводы реле по rele_state. Нужен везде, где меняется маска реле: иначе экран
// показывает On, а само реле остаётся выключенным.
static void write_relay_pins() {
  for (byte i = 0; i < 4; i++) digitalWrite(rele_pin[i], bit_is_set(rele_state, i));
}

static void v3_apply_to_runtime() {
  I2CSTPSetup.role = i2cstepper_v3_address_is_mixer(v3_active_config.address) ? I2CMIXER : I2CPUMP;
  I2CSTPSetup.mode = v3_active_config.mode;
  I2CSTPSetup.optionFlags = v3_active_config.optionFlags;
  I2CSTPSetup.sensorFlags = v3_active_config.sensorFlags;
  I2CSTPSetup.relayMask = v3_active_config.relayMask;
  I2CSTPSetup.mixerRpm = v3_active_config.mixerRpm;
  I2CSTPSetup.mixerRunSec = v3_active_config.mixerRunSec;
  I2CSTPSetup.mixerPauseSec = v3_active_config.mixerPauseSec;
  I2CSTPSetup.pumpMlHour = v3_active_config.pumpMlHour;
  I2CSTPSetup.pumpPauseSec = v3_active_config.pumpPauseSec;
  I2CSTPSetup.fillingMl = v3_active_config.fillingMl;
  I2CSTPSetup.fillingMlHour = v3_active_config.fillingMlHour;
  I2CSTPSetup.stepperStepMl = v3_active_config.stepsPerMl;
  rele_state = v3_active_config.relayMask;
  write_relay_pins();
  set_spd = I2CSTPSetup.mode == I2CMIXER ? I2CSTPSetup.mixerRpm :
            (I2CSTPSetup.mode == I2CPUMP ? I2CSTPSetup.pumpMlHour : I2CSTPSetup.fillingMlHour);
  set_time = I2CSTPSetup.mode == I2CMIXER ? I2CSTPSetup.mixerRunSec :
             (I2CSTPSetup.mode == I2CFILLING ? I2CSTPSetup.fillingMl : 0);
  last_set_time = set_time;
  set_dir = (I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_DIRECTION) ? 1 : 0;
  set_time_initialized = true;
  set_dir_initialized = true;
}

static void v3_publish_runtime_identity(I2CStepperV3Identity* identity) {
  if (!identity) return;
  identity->address = v3_runtime_address;
  identity->capabilities = v3_capabilities();
  v3_status_snapshot.address = v3_runtime_address;
  v3_status_snapshot.mode = v3_runtime_mode;
}

void v3_publish_frames() {
  I2CStepperV3Identity identity = {};
  v3_publish_runtime_identity(&identity);
  v3_status_snapshot.status = stepper.getState() ? I2CSTEPPER_V3_STATUS_RUNNING : 0;
  if (pause_phase) v3_status_snapshot.status |= I2CSTEPPER_V3_STATUS_PAUSED;
  if (external_sensor_active()) v3_status_snapshot.status |= I2CSTEPPER_V3_STATUS_SENSOR;
  if (calibration_active) v3_status_snapshot.status |= I2CSTEPPER_V3_STATUS_CALIBRATION;
  if (v3_status_snapshot.error != I2CSTEPPER_V3_ERR_NONE) v3_status_snapshot.status |= I2CSTEPPER_V3_STATUS_ERROR;
  v3_status_snapshot.currentSpeedStepsPerSec = stepper.getState()
                                               ? (uint32_t)abs((int32_t)stepper.getSpeed()) : 0;
  int32_t current = 0;
  int32_t target = 0;
  stepper_snapshot_position_atomic(&current, &target);
  v3_status_snapshot.remainingSteps = i2cstepper_v3_remaining_steps(v3_motion_continuous,
                                                                      current, target);
  uint8_t sreg = lock_interrupts();
  i2cstepper_v3_encode_identity(v3_identity_frame, &identity);
  i2cstepper_v3_encode_config_a(v3_config_a_frame, &v3_active_config);
  i2cstepper_v3_encode_config_b(v3_config_b_frame, &v3_active_config);
  i2cstepper_v3_encode_motion(v3_motion_frame, &v3_staging_motion);
  i2cstepper_v3_encode_status(v3_status_frame, &v3_status_snapshot);
  unlock_interrupts(sreg);
}

bool v3_local_controls_locked() {
  return i2cstepper_v3_local_controls_locked(v3_remote_owner, millis(), v3_last_heartbeat_ms,
                                             stepper_state || pause_phase || calibration_active);
}

void v3_local_stop() {
  stop_stepper();
  v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_LOCAL;
  v3_status_snapshot.stopEventSeq = i2cstepper_v3_sequence_next(v3_status_snapshot.stopEventSeq);
}

static void v3_stop_remote_for_timeout() {
  I2CStepperV3HeartbeatTimeoutAction action = i2cstepper_v3_heartbeat_timeout_action(
      stepper_state || stepper.getState(), pause_phase, calibration_active);
  stop_motion(false);
  for (byte i = 0; i < 4; i++) digitalWrite(rele_pin[i], LOW);
  rele_state = 0;
  v3_active_config.relayMask = 0;
  v3_staging_config.relayMask = 0;
  I2CSTPSetup.relayMask = 0;
  pause_phase = false;
  calibration_active = false;
  if (action == I2CSTEPPER_V3_TIMEOUT_STOP_AND_REPORT) {
    v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_HEARTBEAT;
    v3_set_result(v3_status_snapshot.commandSeq, I2CSTEPPER_V3_RESULT_FAILED,
                  I2CSTEPPER_V3_ERR_HEARTBEAT_TIMEOUT);
  }
}

static bool v3_motion_valid(bool finite) {
  return v3_staging_motion.direction <= 1 &&
         v3_staging_motion.speedStepsPerSec >= 1 &&
         v3_staging_motion.speedStepsPerSec <= I2CSTEPPER_V3_MAX_SPEED_STEPS_PER_SEC &&
         (!finite || (v3_staging_motion.targetSteps >= 1 &&
                      v3_staging_motion.targetSteps <= I2CSTEPPER_V3_TARGET_STEPS_MAX));
}

static bool v3_prepare_staged_config(I2CStepperV3Config* prepared) {
  if (!prepared) return false;
  *prepared = v3_staging_config;
  if (prepared->address != v3_active_config.address) {
    uint8_t mode = 0;
    if (!i2cstepper_v3_mode_after_address_change(v3_active_config.address, prepared->address,
                                                  v3_active_config.mode, &mode)) return false;
    prepared->mode = mode;
  }
  return v3_config_valid(*prepared);
}

static void v3_start_staged_motion(bool finite) {
  start_motion((uint16_t)v3_staging_motion.speedStepsPerSec,
               finite ? v3_staging_motion.targetSteps : 0,
               v3_staging_motion.direction, !finite);
}

static bool v3_start_configured_motion() {
  if (!v3_config_valid(v3_active_config)) return false;
  start_current_mode();
  return stepper_state;
}

static uint8_t v3_start_precondition_error() {
  if (!v3_movement_allowed) return I2CSTEPPER_V3_ERR_EEPROM_INVALID;
  return i2cstepper_v3_configured_start_ready(v3_runtime_address, v3_active_config.address)
           ? I2CSTEPPER_V3_ERR_NONE : I2CSTEPPER_V3_ERR_REBOOT_REQUIRED;
}

static uint8_t v3_apply_address_error() {
  return i2cstepper_v3_runtime_address_matches(v3_runtime_address, v3_staging_config.address)
           ? I2CSTEPPER_V3_ERR_NONE : I2CSTEPPER_V3_ERR_BAD_ADDRESS;
}

static void v3_claim_remote_ownership() {
  i2cstepper_v3_claim_remote_ownership(&v3_remote_owner, &v3_last_heartbeat_ms, millis());
}

static bool v3_apply_staged_relay() {
  if ((v3_staging_config.relayMask & 0xF0U) != 0) return false;
  v3_active_config.relayMask = v3_staging_config.relayMask;
  v3_staging_config.relayMask = v3_active_config.relayMask;
  I2CSTPSetup.relayMask = v3_active_config.relayMask;
  rele_state = v3_active_config.relayMask;
  write_relay_pins();
  v3_status_snapshot.generation++;
  v3_claim_remote_ownership();
  return true;
}

static bool v3_finish_calibration() {
  uint8_t saved_prescale = pause_stepper_timer();
  int32_t current = stepper.getCurrent();
  resume_stepper_timer(saved_prescale);
  stop_stepper();
  if (current <= 0) return false;
  uint32_t measured = (uint32_t)current / 100UL;
  if (measured == 0) return false;
  I2CStepperV3Config calibrated = v3_active_config;
  calibrated.stepsPerMl = measured;
  if (!v3_eeprom_write(calibrated)) return false;
  v3_active_config = calibrated;
  v3_staging_config = calibrated;
  v3_apply_to_runtime();
  v3_status_snapshot.generation++;
  return true;
}

static bool v3_take_mailbox(const uint8_t* source, uint8_t size, volatile bool* pending,
                            uint8_t* destination) {
  uint8_t sreg = lock_interrupts();
  if (!*pending) {
    unlock_interrupts(sreg);
    return false;
  }
  for (uint8_t index = 0; index < size; index++) destination[index] = source[index];
  *pending = false;
  unlock_interrupts(sreg);
  return true;
}

void v3_process_receive() {
  uint8_t packet[I2CSTEPPER_V3_CONFIG_A_SIZE];
  if (v3_take_mailbox(v3_rx_config_a, I2CSTEPPER_V3_CONFIG_A_SIZE,
                      &v3_rx_config_a_pending, packet)) {
    i2cstepper_v3_decode_config_a(packet, &v3_staging_config);
  }
  if (v3_take_mailbox(v3_rx_config_b, I2CSTEPPER_V3_CONFIG_B_SIZE,
                      &v3_rx_config_b_pending, packet)) {
    i2cstepper_v3_decode_config_b(packet, &v3_staging_config);
  }
  if (v3_take_mailbox(v3_rx_motion, I2CSTEPPER_V3_MOTION_SIZE,
                      &v3_rx_motion_pending, packet)) {
    i2cstepper_v3_decode_motion(packet, &v3_staging_motion);
  }
  if (v3_take_mailbox(v3_rx_command, I2CSTEPPER_V3_COMMAND_SIZE,
                      &v3_rx_command_pending, packet)) {
    I2CStepperV3CommandFrame command;
    i2cstepper_v3_decode_command(packet, &command);
    I2CStepperV3SequenceState seq = i2cstepper_v3_sequence_state(v3_last_sequence, command.commandSeq);
    if (!i2cstepper_v3_runtime_address_matches(v3_runtime_address, command.address)) {
      if (seq == I2CSTEPPER_V3_SEQUENCE_NEW) v3_last_sequence = command.commandSeq;
      v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_ADDRESS);
    } else if (seq == I2CSTEPPER_V3_SEQUENCE_INVALID || seq == I2CSTEPPER_V3_SEQUENCE_STALE) {
      v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_SEQUENCE);
    } else if (seq == I2CSTEPPER_V3_SEQUENCE_DUPLICATE) {
      return;
    } else {
      v3_last_sequence = command.commandSeq;
      v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_PENDING, I2CSTEPPER_V3_ERR_NONE);
      v3_publish_frames();
      if (command.command == I2CSTEPPER_V3_CMD_HEARTBEAT) {
        v3_claim_remote_ownership();
        v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_SUCCESS, I2CSTEPPER_V3_ERR_NONE);
      } else if (command.command == I2CSTEPPER_V3_CMD_APPLY || command.command == I2CSTEPPER_V3_CMD_SAVE) {
        I2CStepperV3Config prepared;
        if (!i2cstepper_v3_address_valid(v3_staging_config.address)) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_ADDRESS);
        } else if (command.command == I2CSTEPPER_V3_CMD_APPLY &&
                   v3_apply_address_error() != I2CSTEPPER_V3_ERR_NONE) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, v3_apply_address_error());
        } else if (v3_staging_config.address == v3_active_config.address &&
                   !i2cstepper_v3_mode_supported(v3_staging_config.address, v3_staging_config.mode)) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_UNSUPPORTED_MODE);
        } else if (!v3_prepare_staged_config(&prepared)) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_CONFIG);
        } else if (command.command == I2CSTEPPER_V3_CMD_SAVE && !v3_eeprom_write(prepared)) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_EEPROM_WRITE);
        } else {
          v3_active_config = prepared;
          v3_staging_config = prepared;
          if (prepared.address == v3_runtime_address) {
            v3_runtime_mode = prepared.mode;
          }
          v3_status_snapshot.generation++;
          if (prepared.address == v3_runtime_address) v3_apply_to_runtime();
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_SUCCESS, I2CSTEPPER_V3_ERR_NONE);
        }
      } else if (command.command == I2CSTEPPER_V3_CMD_START_CONFIGURED) {
        uint8_t precondition = v3_start_precondition_error();
        if (precondition != I2CSTEPPER_V3_ERR_NONE) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, precondition);
        } else if (!v3_start_configured_motion()) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_CONFIG);
        } else {
          v3_claim_remote_ownership();
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_SUCCESS, I2CSTEPPER_V3_ERR_NONE);
        }
      } else if (command.command == I2CSTEPPER_V3_CMD_START_FINITE ||
                 command.command == I2CSTEPPER_V3_CMD_START_CONTINUOUS) {
        bool finite = command.command == I2CSTEPPER_V3_CMD_START_FINITE;
        uint8_t precondition = v3_start_precondition_error();
        if (precondition != I2CSTEPPER_V3_ERR_NONE) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, precondition);
        } else if (v3_staging_motion.mode != v3_runtime_mode ||
                   !i2cstepper_v3_start_mode_supported(v3_runtime_mode, v3_staging_motion.mode,
                                                        command.command)) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_UNSUPPORTED_MODE);
        } else if (!v3_motion_valid(finite)) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_CONFIG);
        } else {
          v3_start_staged_motion(finite);
          v3_claim_remote_ownership();
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_SUCCESS, I2CSTEPPER_V3_ERR_NONE);
        }
      } else if (command.command == I2CSTEPPER_V3_CMD_STOP) {
        stop_stepper();
        v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_REMOTE;
        v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_SUCCESS, I2CSTEPPER_V3_ERR_NONE);
      } else if (command.command == I2CSTEPPER_V3_CMD_RELAY) {
        if (!v3_apply_staged_relay()) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_CONFIG);
        } else {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_SUCCESS, I2CSTEPPER_V3_ERR_NONE);
        }
      } else if (command.command == I2CSTEPPER_V3_CMD_CALIBRATE_START) {
        uint8_t precondition = v3_start_precondition_error();
        if (precondition != I2CSTEPPER_V3_ERR_NONE) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, precondition);
        } else if (i2cstepper_v3_address_is_mixer(v3_runtime_address)) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_UNSUPPORTED_MODE);
        } else if (stepper.getState() || calibration_active) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_CONFIG);
        } else {
          uint64_t speed = stepper_rate_steps(v3_active_config.pumpMlHour, v3_active_config.stepsPerMl, 3600U);
          if (speed < 1 || speed > I2CSTEPPER_V3_MAX_SPEED_STEPS_PER_SEC) {
            v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_CONFIG);
          } else {
            start_motion((uint16_t)speed, 0, 0, true);
            calibration_active = true;
            v3_claim_remote_ownership();
            v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_SUCCESS, I2CSTEPPER_V3_ERR_NONE);
          }
        }
      } else if (command.command == I2CSTEPPER_V3_CMD_CALIBRATE_FINISH) {
        if (!calibration_active) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_CONFIG);
        } else if (!v3_finish_calibration()) {
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_EEPROM_WRITE);
        } else {
          calibration_active = false;
          v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_SUCCESS, I2CSTEPPER_V3_ERR_NONE);
        }
      } else {
        v3_set_result(command.commandSeq, I2CSTEPPER_V3_RESULT_FAILED, I2CSTEPPER_V3_ERR_BAD_COMMAND);
      }
    }
  }
}

static uint8_t lock_interrupts() {
  uint8_t sreg = SREG;
  cli();
  return sreg;
}

static void unlock_interrupts(uint8_t sreg) {
  SREG = sreg;
}

uint8_t pause_stepper_timer() {
  uint8_t prescale = TCCR1B & (0b111 << CS10);
  TCCR1B &= ~(0b111 << CS10);
  TIMSK1 &= (uint8_t)~_BV(OCIE1A);
  return prescale;
}

void resume_stepper_timer(uint8_t prescale) {
  TCCR1B &= ~(0b111 << CS10);
  if (prescale) {
    TCCR1B |= prescale;
    TIMSK1 |= _BV(OCIE1A);
  }
}

static uint8_t timer1_clock_bits(uint16_t prescaler) {
  switch (prescaler) {
    case 1: return _BV(CS10);
    case 8: return _BV(CS11);
    case 64: return _BV(CS11) | _BV(CS10);
    case 256: return _BV(CS12);
    case 1024: return _BV(CS12) | _BV(CS10);
    default: return 0;
  }
}

static void timer1_disarm() {
  TIMSK1 &= (uint8_t)~_BV(OCIE1A);
  TCCR1B &= (uint8_t)~(0b111 << CS10);
}

static void timer1_schedule(uint32_t periodUs) {
  I2CStepperV3TimerPlan plan;
  if (!i2cstepper_v3_timer1_plan_16mhz(periodUs, &plan)) {
    timer1_disarm();
    return;
  }
  TCCR1A = 0;
  OCR1A = plan.ocr1a;
  TCNT1 = 0;
  TCCR1B = _BV(WGM12) | timer1_clock_bits(plan.prescaler);
  TIMSK1 |= _BV(OCIE1A);
}

static uint16_t stepper_acceleration_from_speed(uint16_t spd) {
  uint16_t acc = spd / 10;
  return acc == 0 ? 1 : acc;
}

static void stepper_snapshot_position_atomic(int32_t* current, int32_t* target_abs) {
  uint8_t saved_prescale = pause_stepper_timer();
  if (current) {
    *current = stepper.getCurrent();
  }
  if (target_abs) {
    *target_abs = stepper.getTarget();
  }
  resume_stepper_timer(saved_prescale);
}

void isrENK() {
  encoder.tick();  // отработка в прерывании
}

void setup() {
  stepper.brake();                                    // тормозим шаговик
  stepper.disable();                                  // отключаем шаговик

#ifdef __I2CStepper_DEBUG
  Serial.begin(115200);                               // порт нужен только отладке: в рабочей прошивке он не помещается во флеш
#endif
  attachPCINT(digitalPinToPCINT(ENC_CLK), isrENK, CHANGE); //вешаем прерывания для обработки энкодера
  attachPCINT(digitalPinToPCINT(ENC_DT), isrENK, CHANGE);  //вешаем прерывания для обработки энкодера
  attachPCINT(digitalPinToPCINT(ENC_SW), isrENK, CHANGE);  //вешаем прерывания для обработки энкодера

  read_config();

  Wire2.begin();                                      // инициируем подключение к шине I2C в качестве мастера
  //  stepper.setRunMode(FOLLOW_POS);
  pinMode(MIXER_PUMP_PIN, OUTPUT);                    // используем ногу для вывода
  pinMode(RELE_PIN2, OUTPUT);                         // используем ногу для вывода
  pinMode(RELE_PIN3, OUTPUT);                         // используем ногу для вывода
  pinMode(RELE_PIN4, OUTPUT);                         // используем ногу для вывода
  pinMode(EXT_SENSOR_PIN, EXT_SENSOR_INPUT_MODE);
  rele_state = I2CSTPSetup.relayMask & 0x0F;
  write_relay_pins();
  v3_publish_frames();

  menu_init();                                        // инициализуерм меню экрана

  TCCR1A = 0;
  TCCR1B = _BV(WGM12);
  TCNT1 = 0;
  timer1_disarm();

#ifdef __I2CStepper_DEBUG
  set_spd = 490;
  set_time = 2000;
  last_set_time = set_time;
#endif
}

ISR(TIMER1_COMPA_vect) {
  if (stepper.tickManual()) {
    timer1_schedule(stepper.getPeriod());
  } else {
    timer1_disarm();
  }
}

//возвращаем время или миллилитры, оставшиеся до конца работы шаговика для отображения на экране
uint32_t get_stepper_time(void) {
  if (!set_time_initialized || stepper_state) {
    set_time = get_stepper_time_from_motion();
    set_time_initialized = true;
  }
  return (uint32_t)set_time;
}

//возвращаем время или миллилитры, оставшиеся до конца работы шаговика
uint32_t get_stepper_time_from_motion(void) {
  uint32_t target = get_motion_target();
  uint16_t speed = get_motion_speed();

  if (I2CSTPSetup.mode == I2CMIXER) {
    //если время
    if (speed == 0) return 0;
    return (target + (speed / 2)) / speed;
  }

  if (I2CSTPSetup.mode == I2CPUMP || I2CSTPSetup.mode == I2CFILLING) {
    //если миллилитры
    if (I2CSTPSetup.stepperStepMl == 0) return 0;
    //округление к ближайшему без 64 бит: +1, если остаток не меньше половины делителя
    uint32_t step_ml = I2CSTPSetup.stepperStepMl;
    return target / step_ml + (target % step_ml >= step_ml - step_ml / 2 ? 1 : 0);
  }

  return 0;
}

//возвращаем количество шагов
uint32_t get_motion_target(void) {
  if (v3_motion_continuous) return 0;
  if (!stepper_state) return v3_staging_motion.targetSteps;
  int32_t current = 0;
  int32_t target = 0;
  stepper_snapshot_position_atomic(&current, &target);
  return i2cstepper_v3_remaining_steps(false, current, target);
}

//сохраняем количество шагов шаговика в массив для обмена с Самоваром
void set_motion_target(uint32_t target) {
  v3_staging_motion.targetSteps = target;
}

//возвращаем скрость в шагах в секунду из скорости в оборотах/мин
uint16_t get_spd_stp(uint32_t spd) {
  if (I2CSTPSetup.mode == I2CMIXER) {
    return stepper_speed_steps_mixer(spd, STEPPER_STEPS);
  } else if (I2CSTPSetup.mode == I2CPUMP || I2CSTPSetup.mode == I2CFILLING) {
    return stepper_speed_steps_pump(spd, I2CSTPSetup.stepperStepMl);
  } else {
    return 0;
  }
}

//минимальная user-скорость, при которой get_spd_stp() вернёт хотя бы 1 шаг/с
//(иначе старт шаговика не произойдёт, но пользователь видит «Start: On»)
uint32_t get_min_user_speed(void) {
  if (I2CSTPSetup.mode == I2CMIXER) {
    //(1 * STEPPER_STEPS + 30) / 60 ≥ 1 для STEPPER_STEPS ≥ 30, что всегда верно.
    return 1;
  }
  if (I2CSTPSetup.mode == I2CPUMP || I2CSTPSetup.mode == I2CFILLING) {
    if (I2CSTPSetup.stepperStepMl == 0) {
      return 1;
    }
    //(user * step_ml + 1800) / 3600 ≥ 1 ⟺ user ≥ ceil(1800 / step_ml)
    uint32_t step_ml = I2CSTPSetup.stepperStepMl;
    return (1800UL + step_ml - 1UL) / step_ml;
  }
  return 1;
}

uint32_t get_max_user_speed(void) {
  uint32_t max_spd = 0;
  if (I2CSTPSetup.mode == I2CMIXER) {
    max_spd = (uint32_t)(STEPPER_MAX_SPEED * 60UL) / STEPPER_STEPS;
  } else if (I2CSTPSetup.mode == I2CPUMP || I2CSTPSetup.mode == I2CFILLING) {
    if (I2CSTPSetup.stepperStepMl > 0) {
      max_spd = (STEPPER_MAX_SPEED * 3600UL) / I2CSTPSetup.stepperStepMl;
    }
  }

  if (max_spd > STEPPER_MAX_SPEED) {
    max_spd = STEPPER_MAX_SPEED;
  }

  return max_spd;
}

uint32_t calc_target_from_time(uint32_t time_value, uint16_t spd) {
  if (I2CSTPSetup.mode == I2CMIXER) {
    return stepper_target_from_time_mixer(time_value, spd, STEPPER_TARGET_LIMIT);
  }

  if (I2CSTPSetup.mode == I2CPUMP || I2CSTPSetup.mode == I2CFILLING) {
    return stepper_target_from_time_pump(time_value, I2CSTPSetup.stepperStepMl, STEPPER_TARGET_LIMIT);
  }

  return 0;
}

//возвращаем скорость в оборотах/мин или литры в час
uint32_t get_speed(void) {
  if (I2CSTPSetup.mode == I2CMIXER) {
    //в об/мин
    if (set_spd == 0 || stepper_state) set_spd = (get_motion_speed() * 60UL + STEPPER_STEPS / 2) / STEPPER_STEPS;
  } else if (I2CSTPSetup.mode == I2CPUMP || I2CSTPSetup.mode == I2CFILLING) {
    //в миллилитрах в час
    if (set_spd == 0 || stepper_state) {
      if (I2CSTPSetup.stepperStepMl == 0) {
        set_spd = 0;
      } else {
        set_spd = (get_motion_speed() * 3600UL + I2CSTPSetup.stepperStepMl / 2) / I2CSTPSetup.stepperStepMl;
      }
    }
  } else {
    set_spd = 0;
  }

  uint32_t max_spd = get_max_user_speed();
  if (set_spd > max_spd) {
    set_spd = max_spd;
    //Синхронизируем клэмп с массивом, чтобы шаговик и мастер видели ограниченное значение
    set_motion_speed(get_spd_stp(set_spd));
  }

  return set_spd;
}

//получаем скорость в шагах в секунду из массива
uint16_t get_motion_speed(void) {
  return v3_staging_motion.speedStepsPerSec;
}

//сохраняем скорость в массив для обмена с Самоваром
void set_motion_speed(uint16_t spd) {
  v3_staging_motion.speedStepsPerSec = spd;
}

//получаем направление движения для отображения на экране
byte get_direction(void) {
  if (!set_dir_initialized) {
    set_dir = get_motion_direction();
    set_dir_initialized = true;
  }
  return set_dir;
}

//получаем направление движения из массива
byte get_motion_direction(void) {
  return (I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_DIRECTION) ? 1 : 0;
}

//сохраняем направление движения в массив для обмена с Самоваром
void set_motion_direction(byte dir) {
  I2CSTPSetup.optionFlags = (I2CSTPSetup.optionFlags & (uint8_t)~I2CSTEPPER_FLAG_DIRECTION) |
                            (dir ? I2CSTEPPER_FLAG_DIRECTION : 0);
  v3_active_config.optionFlags = I2CSTPSetup.optionFlags;
  v3_staging_config.optionFlags = I2CSTPSetup.optionFlags;
  v3_staging_motion.direction = dir;
}

//сохраняем состояние насоса в массив для обмена с Самоваром
bool set_mixer_pump_state(bool state) {
  set_rele_state(1, state);
  return state;
}

//получаем состояние насоса из массива
bool get_rele_state(byte r) {
  if (r < 1 || r > 4) {
    return false;
  }
  return bitRead(v3_active_config.relayMask, r - 1);
}

//сохраняем состояние реле по номеру в массив для обмена с Самоваром
bool set_rele_state(byte r, bool s) {
  if (r < 1 || r > 4) {
    return false;
  }

  byte relay_mask = v3_active_config.relayMask;
  bitWrite(relay_mask, r - 1, s);
  v3_active_config.relayMask = relay_mask;
  v3_staging_config.relayMask = relay_mask;
  I2CSTPSetup.relayMask = relay_mask;
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

//запускаем шаговик из локального меню.
void start_stepper(bool from_int) {
  (void)from_int;
  if (!v3_movement_allowed) return;
  uint32_t target = 0;
  uint16_t spd = 0;
  byte dir = 0;
  bool continuous = I2CSTPSetup.mode == I2CPUMP ||
                    (I2CSTPSetup.mode == I2CMIXER && set_time == 0);

  if (from_int) {
    spd = get_spd_stp(set_spd);
    if (!continuous) {
      target = calc_target_from_time(set_time, spd);
      if (target == 0) return;
    }
    dir = set_dir;
    set_dir_initialized = true;
    set_motion_speed(spd);
    set_motion_direction(dir);
    set_motion_target(target);
  }
  if (!continuous && target > STEPPER_TARGET_LIMIT) {
    target = STEPPER_TARGET_LIMIT;
    set_motion_target(target);
  }
  last_dir = dir;

  if (spd == 0 || (!continuous && target == 0)) return;

  start_motion(spd, target, dir, continuous);

  //синхронизируем last_set_time, чтобы первая итерация loop не инициировала ложный time-sync
  last_set_time = get_stepper_time_from_motion();
}

//останавливаем шаговик
void stop_stepper() {
  stop_motion((I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_SMOOTH_START) != 0);
}

static void stop_motion(bool smooth) {
  set_motion_target(0);

  pause_stepper_timer();
  if (smooth && !v3_motion_continuous && stepper.getState()) {
    stepper.stop();
    timer1_schedule(stepper.getPeriod());
  } else {
    stepper.brake();
    stepper.disable();
    stepper.setCurrent(0);
    stepper.setTarget(0);
    timer1_disarm();
  }
  curr_spd = 0;
  stepper_state = false;
  pause_phase = false;
  calibration_active = false;
  v3_motion_continuous = false;
  v3_mixer_deadline_active = false;

#ifdef __I2CStepper_DEBUG
  Serial.println(F("======================"));
  Serial.println(F("FINISH STP"));
  Serial.println(F("======================"));
#endif
}

bool external_sensor_active() {
  bool active = digitalRead(EXT_SENSOR_PIN);
  if (!(I2CSTPSetup.sensorFlags & I2CSTEPPER_SENSOR_ACTIVE_HIGH)) {
    active = !active;
  }
  return active;
}

// Непрерывное вращение с плавным разгоном идёт в два этапа: сначала как ход к далёкой цели
// (разгон считает библиотека), затем finish_continuous_ramp() переводит мотор на постоянную скорость.
static void run_stepper(uint16_t spd, uint32_t target, bool continuous) {
  if (continuous && !(I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_SMOOTH_START)) {
    stepper.setSpeed((int32_t)spd);
  } else {
    stepper.setMaxSpeed(spd);
    stepper.setTarget((int32_t)(continuous ? STEPPER_TARGET_LIMIT : target));
  }
}

static void start_motion(uint16_t spd, uint32_t target, byte dir, bool continuous) {
  if (!v3_movement_allowed || spd == 0 || (!continuous && target == 0)) {
    v3_status_snapshot.error = I2CSTEPPER_V3_ERR_BAD_CONFIG;
    return;
  }
  if (!continuous && target > STEPPER_TARGET_LIMIT) target = STEPPER_TARGET_LIMIT;
  v3_staging_motion.mode = v3_runtime_mode;
  v3_staging_motion.direction = dir;
  v3_staging_motion.speedStepsPerSec = spd;
  v3_staging_motion.targetSteps = continuous ? 0 : target;
  v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_NONE;

  pause_stepper_timer();
  // Непрерывное вращение уже идёт в ту же сторону (Самовар меняет скорость повторным стартом):
  // не тормозим, иначе каждая смена скорости начинала бы разгон с нуля.
  if (!(continuous && v3_motion_continuous && dir == last_dir && stepper.getState())) {
    stepper.brake();
    stepper.setCurrent(0);
  }
  stepper.enable();
  stepper.reverse(dir);
  if (I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_SMOOTH_START) {
    stepper.setAcceleration(stepper_acceleration_from_speed(spd));
  } else {
    stepper.setAcceleration(0);
  }
  run_stepper(spd, target, continuous);
  curr_spd = 0;
  set_dir = dir;
  last_dir = dir;
  stepper_state = true;
  pause_phase = false;
  v3_motion_continuous = continuous;
  v3_mixer_deadline_active = false;
  set_motion_speed(spd);
  set_motion_target(continuous ? 0 : target);
  timer1_schedule(stepper.getPeriod());
  v3_status_snapshot.error = I2CSTEPPER_V3_ERR_NONE;
}

void apply_local_motion_settings() {
  if (!stepper_state || pause_phase || !stepper.getState()) return;
  uint16_t spd = get_spd_stp(set_spd);
  if (spd == 0) return;

  pause_stepper_timer();
  stepper.reverse(set_dir);
  run_stepper(spd, stepper.getTarget(), v3_motion_continuous);
  curr_spd = 0;
  set_motion_speed(spd);
  timer1_schedule(stepper.getPeriod());
}

static void finish_mixer_run_phase() {
  v3_mixer_deadline_active = false;
  pause_stepper_timer();
  stepper.brake();
  stepper.setTarget(stepper.getCurrent());
  stepper.disable();
  timer1_disarm();
  curr_spd = 0;
  v3_motion_continuous = false;

  if (I2CSTPSetup.mixerPauseSec > 0 && stepper_state) {
    pause_phase = true;
    pause_deadline_ms = millis() + (uint32_t)I2CSTPSetup.mixerPauseSec * 1000UL;
    if (I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_REVERSE_AFTER_PAUSE) {
      set_motion_direction(get_motion_direction() ? 0 : 1);
    }
  } else {
    stepper_state = false;
    v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_COMPLETE;
  }
}

void start_current_mode() {
  if (!v3_movement_allowed) return;

  byte dir = (I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_DIRECTION) ? 1 : 0;
  if (I2CSTPSetup.mode == I2CMIXER) {
    uint16_t spd = stepper_speed_steps_mixer(I2CSTPSetup.mixerRpm, STEPPER_STEPS);
    set_spd = I2CSTPSetup.mixerRpm;
    set_time = I2CSTPSetup.mixerRunSec;
    if (I2CSTPSetup.mixerRunSec == 0) {
      start_motion(spd, 0, dir, true);
    } else {
      uint32_t target = stepper_target_from_time_mixer(I2CSTPSetup.mixerRunSec, spd,
                                                         STEPPER_TARGET_LIMIT);
      start_motion(spd, target, dir, false);
      if (stepper_state) {
        v3_mixer_deadline_ms = millis() + (uint32_t)I2CSTPSetup.mixerRunSec * 1000UL;
        v3_mixer_deadline_active = true;
      }
    }
  } else if (I2CSTPSetup.mode == I2CPUMP) {
    uint16_t spd = stepper_speed_steps_pump(I2CSTPSetup.pumpMlHour, I2CSTPSetup.stepperStepMl);
    set_spd = I2CSTPSetup.pumpMlHour;
    set_time = 0;
    start_motion(spd, 0, dir, true);
  } else if (I2CSTPSetup.mode == I2CFILLING) {
    uint16_t spd = stepper_speed_steps_pump(I2CSTPSetup.fillingMlHour, I2CSTPSetup.stepperStepMl);
    uint32_t target = stepper_target_from_time_pump(I2CSTPSetup.fillingMl, I2CSTPSetup.stepperStepMl, STEPPER_TARGET_LIMIT);
    set_spd = I2CSTPSetup.fillingMlHour;
    set_time = I2CSTPSetup.fillingMl;
    start_motion(spd, target, dir, false);
  } else {
    v3_status_snapshot.error = I2CSTEPPER_V3_ERR_UNSUPPORTED_MODE;
  }
}

void finish_calibration() {
  uint8_t saved_prescale = pause_stepper_timer();
  uint32_t done = stepper.getCurrent();
  resume_stepper_timer(saved_prescale);
  stop_stepper();
  calibration_active = false;
  if (done > 0) {
    I2CSTPSetup.stepperStepMl = (uint16_t)(done / 100UL);
    if (I2CSTPSetup.stepperStepMl == 0) I2CSTPSetup.stepperStepMl = 1;
    v3_active_config.stepsPerMl = I2CSTPSetup.stepperStepMl;
    v3_staging_config.stepsPerMl = I2CSTPSetup.stepperStepMl;
  }
  v3_publish_frames();
}

// Второй этап плавного старта непрерывного вращения: разгон закончен (период шага дошёл
// до заданного) - переводим мотор с хода к далёкой цели на постоянную скорость.
static void finish_continuous_ramp() {
  if (!v3_motion_continuous || stepper.getStatus() != 1) return;
  uint8_t saved_prescale = pause_stepper_timer();
  if (stepper.getPeriod() <= 1000000UL / get_motion_speed()) {
    stepper.setSpeed((int32_t)get_motion_speed());
  }
  resume_stepper_timer(saved_prescale);
}

void update_runtime_state() {
  if (v3_mixer_deadline_active &&
      i2cstepper_v3_deadline_reached(millis(), v3_mixer_deadline_ms)) {
    finish_mixer_run_phase();
  }

  if (external_sensor_active()) {
    if ((I2CSTPSetup.sensorFlags & I2CSTEPPER_SENSOR_STOP) &&
        (stepper_state || pause_phase || calibration_active)) {
      stop_motion(false);
      v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_SENSOR;
    } else if (I2CSTPSetup.mode == I2CPUMP &&
               (I2CSTPSetup.sensorFlags & I2CSTEPPER_SENSOR_PUMP_PAUSE) &&
               I2CSTPSetup.pumpPauseSec > 0 && stepper_state && stepper.getState()) {
      if (!pause_phase) {
        pause_stepper_timer();
        stepper.brake();
        stepper.disable();
        timer1_disarm();
        pause_phase = true;
      }
      pause_deadline_ms = millis() + (uint32_t)I2CSTPSetup.pumpPauseSec * 1000UL;
    }
  }

  if (pause_phase && stepper_state &&
      i2cstepper_v3_deadline_reached(millis(), pause_deadline_ms)) {
    pause_phase = false;
    start_current_mode();
  }

  finish_continuous_ramp();

  if (!pause_phase && !stepper.getState() && stepper_state) {
    if (I2CSTPSetup.mode == I2CMIXER &&
        I2CSTPSetup.mixerRunSec > 0 &&
        I2CSTPSetup.mixerPauseSec > 0) {
      finish_mixer_run_phase();
    } else {
      if (I2CSTPSetup.mode == I2CFILLING) stepper.disable();
      stepper_state = false;
      v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_COMPLETE;
    }
  }
}


//основной цикл
void loop() {
  //TRACE();
  //опрашиваем состояние энкодера и работаем с меню
  poll_menu();
  v3_process_receive();
  if (i2cstepper_v3_heartbeat_expired(v3_remote_owner, millis(), v3_last_heartbeat_ms)) {
    v3_stop_remote_for_timeout();
    v3_remote_owner = false;
  }
  update_runtime_state();
  v3_publish_frames();
}

void read_config() {
  v3_movement_allowed = false;
  v3_status_snapshot = {};
  I2CStepperV3EepromState eepromState = v3_eeprom_read(&v3_active_config);
  if (eepromState == I2CSTEPPER_V3_EEPROM_VALID && !v3_config_valid(v3_active_config)) {
    eepromState = I2CSTEPPER_V3_EEPROM_CORRUPT;
  }
  bool hasV2Header = false;
  I2CStepperV2Config legacy;
  if (eepromState == I2CSTEPPER_V3_EEPROM_ABSENT) {
    EEPROM.get(0, legacy);
    hasV2Header = legacy.marker == I2CSTEPPER_V2_EEPROM_MARKER &&
                  legacy.version == I2CSTEPPER_V2_EEPROM_VERSION;
  }
  const uint8_t legacyType = EEPROM.read(0);
  I2CStepperV3BootAction bootAction = i2cstepper_v3_boot_action(
      eepromState, hasV2Header,
      legacyType == I2CSTEPPER_V2_ROLE_MIXER || legacyType == I2CSTEPPER_V2_ROLE_PUMP);
  if (bootAction == I2CSTEPPER_V3_BOOT_USE_V3) {
    v3_movement_allowed = true;
  } else if (bootAction == I2CSTEPPER_V3_BOOT_MIGRATE_V2) {
    if (!i2cstepper_v3_migrate_v2_config(&legacy, &v3_active_config) ||
        !v3_config_valid(v3_active_config)) {
      v3_default_config(&v3_active_config);
    }
    if (v3_eeprom_write(v3_active_config)) v3_movement_allowed = true;
  } else if (bootAction == I2CSTEPPER_V3_BOOT_MIGRATE_V1) {
    v3_default_config(&v3_active_config);
    v3_active_config.address = legacyType;
    v3_active_config.mode = legacyType == I2CSTEPPER_V2_ROLE_MIXER ? I2CSTEPPER_V3_MODE_MIXER
                                                                   : I2CSTEPPER_V3_MODE_PUMP;
    const uint32_t defaultStepsPerMl = v3_active_config.stepsPerMl;
    uint32_t legacyStepsPerMl = 0;
    EEPROM.get(1, legacyStepsPerMl);
    v3_active_config.stepsPerMl = legacyStepsPerMl;
    if (!v3_config_valid(v3_active_config)) v3_active_config.stepsPerMl = defaultStepsPerMl;
    if (v3_eeprom_write(v3_active_config)) v3_movement_allowed = true;
  } else if (bootAction == I2CSTEPPER_V3_BOOT_DEFAULTS) {
    v3_default_config(&v3_active_config);
    if (v3_eeprom_write(v3_active_config)) v3_movement_allowed = true;
  }
  if (!v3_movement_allowed) {
    v3_status_snapshot.error = I2CSTEPPER_V3_ERR_EEPROM_INVALID;
    return;
  }
  v3_staging_config = v3_active_config;
  v3_runtime_mode = v3_active_config.mode;
  v3_staging_motion.mode = v3_runtime_mode;
  v3_staging_motion.direction = 0;
  v3_staging_motion.speedStepsPerSec = 1;
  v3_staging_motion.targetSteps = 1;
  v3_runtime_address = v3_active_config.address;
  v3_apply_to_runtime();
  Wire.begin(v3_runtime_address);
  Wire.onReceive(v3_wire_receive);
  Wire.onRequest(v3_wire_request);
}

bool write_config() {
  I2CStepperV3Config saved = v3_active_config;
  saved.address = v3_staging_config.address;
  saved.mode = I2CSTPSetup.mode;
  saved.optionFlags = I2CSTPSetup.optionFlags;
  saved.sensorFlags = I2CSTPSetup.sensorFlags;
  saved.relayMask = I2CSTPSetup.relayMask;
  saved.mixerRpm = I2CSTPSetup.mixerRpm;
  saved.mixerRunSec = I2CSTPSetup.mixerRunSec;
  saved.mixerPauseSec = I2CSTPSetup.mixerPauseSec;
  saved.pumpMlHour = I2CSTPSetup.pumpMlHour;
  saved.pumpPauseSec = I2CSTPSetup.pumpPauseSec;
  saved.fillingMl = I2CSTPSetup.fillingMl;
  saved.fillingMlHour = I2CSTPSetup.fillingMlHour;
  saved.stepsPerMl = I2CSTPSetup.stepperStepMl;
  if (!v3_config_valid(saved) || !v3_eeprom_write(saved)) return false;
  v3_active_config = saved;
  v3_staging_config = saved;
  if (saved.address == v3_runtime_address) {
    v3_runtime_mode = saved.mode;
    v3_apply_to_runtime();
  } else {
    I2CSTPSetup.role = i2cstepper_v3_address_is_mixer(v3_runtime_address) ? I2CMIXER : I2CPUMP;
    I2CSTPSetup.mode = v3_runtime_mode;
  }
  v3_status_snapshot.generation++;
  return true;
}
