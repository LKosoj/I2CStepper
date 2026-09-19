#include <assert.h>
#include <stdint.h>

#include "I2CStepperRuntime.h"

static void make_valid_record(uint8_t* record) {
  record[0] = 0x53U;
  record[1] = 3U;
  record[2] = 37U;
  for (uint8_t i = 0; i < 37U; i++) record[5U + i] = (uint8_t)(i * 3U + 1U);
  uint16_t crc = i2cstepper_v3_crc16(&record[5], 37U);
  record[3] = (uint8_t)(crc >> 8);
  record[4] = (uint8_t)crc;
}

int main() {
  uint8_t record[42] = {};
  make_valid_record(record);
  assert(i2cstepper_v3_eeprom_record_state(record, 42U, 0x53U, 3U, 37U, 5U) ==
         I2CSTEPPER_V3_EEPROM_VALID);

  record[17] ^= 0x01U;
  assert(i2cstepper_v3_eeprom_record_state(record, 42U, 0x53U, 3U, 37U, 5U) ==
         I2CSTEPPER_V3_EEPROM_CORRUPT);
  make_valid_record(record);
  record[1] = 2U;
  assert(i2cstepper_v3_eeprom_record_state(record, 42U, 0x53U, 3U, 37U, 5U) ==
         I2CSTEPPER_V3_EEPROM_CORRUPT);
  record[0] = 0xFFU;
  record[1] = 0xFFU;
  assert(i2cstepper_v3_eeprom_record_state(record, 42U, 0x53U, 3U, 37U, 5U) ==
         I2CSTEPPER_V3_EEPROM_ABSENT);
  assert(i2cstepper_v3_boot_action(I2CSTEPPER_V3_EEPROM_VALID, false, false) ==
         I2CSTEPPER_V3_BOOT_USE_V3);
  assert(i2cstepper_v3_boot_action(I2CSTEPPER_V3_EEPROM_ABSENT, true, true) ==
         I2CSTEPPER_V3_BOOT_MIGRATE_V2);
  assert(i2cstepper_v3_boot_action(I2CSTEPPER_V3_EEPROM_CORRUPT, true, true) ==
         I2CSTEPPER_V3_BOOT_DEFAULTS);
  assert(i2cstepper_v3_boot_action(I2CSTEPPER_V3_EEPROM_ABSENT, false, true) ==
         I2CSTEPPER_V3_BOOT_MIGRATE_V1);
  assert(i2cstepper_v3_boot_action(I2CSTEPPER_V3_EEPROM_ABSENT, false, false) ==
         I2CSTEPPER_V3_BOOT_DEFAULTS);
  assert(!i2cstepper_v3_derived_config_valid(100U, 0U, true,
                                             18000U, 2147483647UL));
  assert(i2cstepper_v3_derived_config_valid(100U, 0U, false,
                                            18000U, 2147483647UL));
  assert(i2cstepper_v3_derived_config_valid(18000U, 2147483647ULL, true,
                                            18000U, 2147483647UL));
  assert(!i2cstepper_v3_derived_config_valid(18001U, 1U, true,
                                             18000U, 2147483647UL));
  assert(!i2cstepper_v3_start_allowed(false, 100U, 1U));
  assert(!i2cstepper_v3_start_allowed(true, 0U, 1U));
  assert(i2cstepper_v3_start_allowed(true, 100U, 1U));
  assert(i2cstepper_v3_heartbeat_timeout_action(false, false, false) ==
         I2CSTEPPER_V3_TIMEOUT_RELEASE_ONLY);
  assert(i2cstepper_v3_heartbeat_timeout_action(true, false, false) ==
         I2CSTEPPER_V3_TIMEOUT_STOP_AND_REPORT);
  assert(i2cstepper_v3_heartbeat_timeout_action(false, true, false) ==
         I2CSTEPPER_V3_TIMEOUT_STOP_AND_REPORT);
  bool remoteOwner = false;
  uint32_t lastHeartbeatMs = 0;
  i2cstepper_v3_claim_remote_ownership(&remoteOwner, &lastHeartbeatMs, 250U);
  assert(remoteOwner && lastHeartbeatMs == 250U);
  assert(!i2cstepper_v3_heartbeat_expired(true, 10250U, 250U));
  assert(i2cstepper_v3_heartbeat_expired(true, 10251U, 250U));
  assert(!i2cstepper_v3_heartbeat_expired(false, 10251U, 250U));
  uint8_t runtimeAddress = 1U;
  assert(i2cstepper_v3_runtime_address_matches(runtimeAddress, 1U));
  assert(!i2cstepper_v3_runtime_address_matches(runtimeAddress, 3U));
  assert(!i2cstepper_v3_runtime_address_matches(runtimeAddress, 2U));
  assert(i2cstepper_v3_configured_start_ready(runtimeAddress, 1U));
  assert(!i2cstepper_v3_configured_start_ready(runtimeAddress, 2U));
  runtimeAddress = 3U;  // simulated reboot after a same-parity SAVE
  assert(i2cstepper_v3_runtime_address_matches(runtimeAddress, 3U));
  runtimeAddress = 2U;  // simulated reboot after a cross-parity SAVE
  assert(i2cstepper_v3_runtime_address_matches(runtimeAddress, 2U));
  assert(i2cstepper_v3_configured_start_ready(runtimeAddress, 2U));
  uint32_t commandSeq = 0;
  uint32_t ackSeq = 0;
  i2cstepper_v3_acknowledge_sequence(0xFFFFFFFFUL, &commandSeq, &ackSeq);
  assert(commandSeq == 0xFFFFFFFFUL && ackSeq == 0xFFFFFFFFUL);

  I2CStepperV3TimerPlan plan = {};
  assert(i2cstepper_v3_timer1_plan_16mhz(1U, &plan));
  assert(plan.prescaler == 1U && plan.ocr1a == 319U);
  assert(i2cstepper_v3_timer1_plan_16mhz(1000U, &plan));
  assert(plan.prescaler == 1U && plan.ocr1a == 15999U);
  assert(i2cstepper_v3_timer1_plan_16mhz(500U, &plan));
  assert(plan.prescaler == 1U && plan.ocr1a == 7999U);
  assert(i2cstepper_v3_timer1_plan_16mhz(4096U, &plan));
  assert(plan.prescaler == 1U && plan.ocr1a == 65535U);
  assert(i2cstepper_v3_timer1_plan_16mhz(4097U, &plan));
  assert(plan.prescaler == 8U && plan.ocr1a == 8193U);
  assert(i2cstepper_v3_timer1_plan_16mhz(4194304UL, &plan));
  assert(plan.prescaler == 1024U && plan.ocr1a == 65535U);
  assert(!i2cstepper_v3_timer1_plan_16mhz(4194305UL, &plan));
  assert(i2cstepper_v3_remaining_steps(false, 7, 19) == 12U);
  assert(i2cstepper_v3_remaining_steps(false, 19, 7) == 0U);
  assert(i2cstepper_v3_remaining_steps(true, 7, 19) == 0U);
  assert(i2cstepper_v3_deadline_reached(5U, 0xFFFFFFFEUL));
  assert(!i2cstepper_v3_deadline_reached(0xFFFFFFFEUL, 5U));

  return 0;
}
