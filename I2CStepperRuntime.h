#ifndef __I2CSTEPPER_RUNTIME_H
#define __I2CSTEPPER_RUNTIME_H

#include <stdint.h>

enum I2CStepperV3EepromState : uint8_t {
  I2CSTEPPER_V3_EEPROM_ABSENT = 0,
  I2CSTEPPER_V3_EEPROM_VALID = 1,
  I2CSTEPPER_V3_EEPROM_CORRUPT = 2,
};

enum I2CStepperV3BootAction : uint8_t {
  I2CSTEPPER_V3_BOOT_USE_V3 = 0,
  I2CSTEPPER_V3_BOOT_MIGRATE_V2 = 1,
  I2CSTEPPER_V3_BOOT_DEFAULTS = 2,
  I2CSTEPPER_V3_BOOT_MIGRATE_V1 = 4,
};

enum I2CStepperV3HeartbeatTimeoutAction : uint8_t {
  I2CSTEPPER_V3_TIMEOUT_RELEASE_ONLY = 0,
  I2CSTEPPER_V3_TIMEOUT_STOP_AND_REPORT = 1,
};

static inline bool i2cstepper_v3_derived_config_valid(uint64_t speed, uint64_t target,
                                                        bool targetRequired, uint32_t maxSpeed,
                                                        uint32_t maxTarget) {
  return speed >= 1 && speed <= maxSpeed &&
         (!targetRequired || (target >= 1 && target <= maxTarget));
}

static inline bool i2cstepper_v3_start_allowed(bool movementAllowed, uint32_t speed,
                                                uint32_t target) {
  return movementAllowed && speed >= 1 && target >= 1;
}

static inline bool i2cstepper_v3_runtime_address_matches(uint8_t runtimeAddress,
                                                          uint8_t commandAddress) {
  return runtimeAddress == commandAddress;
}

static inline bool i2cstepper_v3_configured_start_ready(uint8_t runtimeAddress,
                                                         uint8_t persistedAddress) {
  return runtimeAddress == persistedAddress;
}

struct I2CStepperV3TimerPlan {
  uint16_t prescaler;
  uint16_t ocr1a;
};

static inline bool i2cstepper_v3_timer1_plan_16mhz(uint32_t periodUs,
                                                    I2CStepperV3TimerPlan* plan) {
  static const uint16_t prescalers[] = {1, 8, 64, 256, 1024};
  if (!plan) return false;
  if (periodUs < 20U) periodUs = 20U;

  for (uint8_t i = 0; i < sizeof(prescalers) / sizeof(prescalers[0]); i++) {
    uint32_t ticks = (periodUs * 16UL + prescalers[i] - 1U) / prescalers[i];
    if (ticks >= 1U && ticks <= 65536UL) {
      plan->prescaler = prescalers[i];
      plan->ocr1a = (uint16_t)(ticks - 1U);
      return true;
    }
  }
  return false;
}

static inline uint32_t i2cstepper_v3_remaining_steps(bool continuous, int32_t current,
                                                      int32_t target) {
  return !continuous && target > current ? (uint32_t)(target - current) : 0U;
}

static inline bool i2cstepper_v3_deadline_reached(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

static inline I2CStepperV3HeartbeatTimeoutAction i2cstepper_v3_heartbeat_timeout_action(
    bool running, bool paused, bool calibration) {
  return (running || paused || calibration) ? I2CSTEPPER_V3_TIMEOUT_STOP_AND_REPORT
                                            : I2CSTEPPER_V3_TIMEOUT_RELEASE_ONLY;
}

static inline void i2cstepper_v3_claim_remote_ownership(bool* remoteOwner,
                                                         uint32_t* lastHeartbeatMs,
                                                         uint32_t nowMs) {
  if (!remoteOwner || !lastHeartbeatMs) return;
  *remoteOwner = true;
  *lastHeartbeatMs = nowMs;
}

// Сколько Nano ждёт «пульс» Самовара, прежде чем остановить мотор и снять реле.
// 10 с: короткая занятость Самовара (запись файла, обмен с другим устройством) не должна
// останавливать процесс.
#define I2CSTEPPER_V3_HEARTBEAT_TIMEOUT_MS 10000UL

static inline bool i2cstepper_v3_heartbeat_expired(bool remoteOwner, uint32_t nowMs,
                                                    uint32_t lastHeartbeatMs) {
  return remoteOwner && (uint32_t)(nowMs - lastHeartbeatMs) > I2CSTEPPER_V3_HEARTBEAT_TIMEOUT_MS;
}

static inline void i2cstepper_v3_acknowledge_sequence(uint32_t sequence,
                                                       uint32_t* commandSequence,
                                                       uint32_t* ackSequence) {
  if (commandSequence) *commandSequence = sequence;
  if (ackSequence) *ackSequence = sequence;
}

static inline I2CStepperV3BootAction i2cstepper_v3_boot_action(
    I2CStepperV3EepromState state, bool hasV2Header, bool hasV1Type) {
  if (state == I2CSTEPPER_V3_EEPROM_VALID) return I2CSTEPPER_V3_BOOT_USE_V3;
  // Битая запись v3 и любые неизвестные данные заменяются настройками по умолчанию:
  // иначе Nano не выходит на шину и Samovar её не видит.
  if (state == I2CSTEPPER_V3_EEPROM_CORRUPT) return I2CSTEPPER_V3_BOOT_DEFAULTS;
  if (hasV2Header) return I2CSTEPPER_V3_BOOT_MIGRATE_V2;
  // Прошивки 0.1-0.4 хранили с нулевого байта {Type (1 мешалка / 2 насос), шагов на мл}.
  if (hasV1Type) return I2CSTEPPER_V3_BOOT_MIGRATE_V1;
  return I2CSTEPPER_V3_BOOT_DEFAULTS;
}

static inline uint16_t i2cstepper_v3_crc16(const uint8_t* data, uint8_t size) {
  uint16_t crc = 0xFFFFU;
  for (uint8_t i = 0; i < size; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static inline I2CStepperV3EepromState i2cstepper_v3_eeprom_record_state(
    const uint8_t* record, uint8_t size, uint8_t magic, uint8_t version,
    uint8_t payloadSize, uint8_t headerSize) {
  if (!record || size != (uint8_t)(headerSize + payloadSize)) return I2CSTEPPER_V3_EEPROM_CORRUPT;
  if (record[0] != magic && record[1] != version) return I2CSTEPPER_V3_EEPROM_ABSENT;
  if (record[0] != magic || record[1] != version || record[2] != payloadSize) {
    return I2CSTEPPER_V3_EEPROM_CORRUPT;
  }
  uint16_t stored = ((uint16_t)record[3] << 8) | record[4];
  return stored == i2cstepper_v3_crc16(&record[headerSize], payloadSize)
           ? I2CSTEPPER_V3_EEPROM_VALID
           : I2CSTEPPER_V3_EEPROM_CORRUPT;
}

#endif // __I2CSTEPPER_RUNTIME_H
