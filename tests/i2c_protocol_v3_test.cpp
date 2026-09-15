#include <assert.h>
#include <stdint.h>

#include <I2CStepperV3.h>
#include "I2CStepperRuntime.h"

int main() {
  assert(I2CSTEPPER_V3_MAGIC == 0x53U);
  assert(I2CSTEPPER_V3_VERSION == 3U);
  assert(I2CSTEPPER_V3_ADDRESS_MIN == 1U);
  assert(I2CSTEPPER_V3_ADDRESS_MAX == 10U);
  assert(I2CSTEPPER_V3_MAX_SPEED_STEPS_PER_SEC == 18000UL);
  assert(I2CSTEPPER_V3_REG_IDENTITY == 0x00U);
  assert(I2CSTEPPER_V3_REG_STATUS == 0x10U);
  assert(I2CSTEPPER_V3_REG_CONFIG_A == 0x30U);
  assert(I2CSTEPPER_V3_REG_CONFIG_B == 0x50U);
  assert(I2CSTEPPER_V3_REG_MOTION == 0x70U);
  assert(I2CSTEPPER_V3_REG_COMMAND == 0x80U);
  assert(I2CSTEPPER_V3_MODE_MIXER == 1U);
  assert(I2CSTEPPER_V3_MODE_PUMP == 2U);
  assert(I2CSTEPPER_V3_MODE_FILLING == 3U);
  assert(I2CSTEPPER_V3_CMD_NONE == 0U);
  assert(I2CSTEPPER_V3_CMD_APPLY == 1U);
  assert(I2CSTEPPER_V3_CMD_SAVE == 2U);
  assert(I2CSTEPPER_V3_CMD_START_FINITE == 3U);
  assert(I2CSTEPPER_V3_CMD_START_CONTINUOUS == 4U);
  assert(I2CSTEPPER_V3_CMD_STOP == 5U);
  assert(I2CSTEPPER_V3_CMD_RELAY == 6U);
  assert(I2CSTEPPER_V3_CMD_CALIBRATE_START == 7U);
  assert(I2CSTEPPER_V3_CMD_CALIBRATE_FINISH == 8U);
  assert(I2CSTEPPER_V3_CMD_HEARTBEAT == 9U);
  assert(I2CSTEPPER_V3_CMD_START_CONFIGURED == 10U);
  assert(I2CSTEPPER_V3_ERR_NONE == 0U);
  assert(I2CSTEPPER_V3_ERR_BAD_ADDRESS == 1U);
  assert(I2CSTEPPER_V3_ERR_UNSUPPORTED_MODE == 2U);
  assert(I2CSTEPPER_V3_ERR_BAD_CONFIG == 3U);
  assert(I2CSTEPPER_V3_ERR_BAD_COMMAND == 4U);
  assert(I2CSTEPPER_V3_ERR_HEARTBEAT_TIMEOUT == 5U);
  assert(I2CSTEPPER_V3_ERR_EEPROM_INVALID == 6U);
  assert(I2CSTEPPER_V3_ERR_EEPROM_WRITE == 7U);
  assert(I2CSTEPPER_V3_ERR_BAD_SEQUENCE == 8U);
  assert(I2CSTEPPER_V3_ERR_REBOOT_REQUIRED == 9U);
  assert(I2CSTEPPER_V3_RESULT_NONE == 0U);
  assert(I2CSTEPPER_V3_RESULT_PENDING == 1U);
  assert(I2CSTEPPER_V3_RESULT_SUCCESS == 2U);
  assert(I2CSTEPPER_V3_RESULT_FAILED == 3U);
  assert(I2CSTEPPER_V3_IDENTITY_SIZE == 8U);
  assert(I2CSTEPPER_V3_STATUS_SIZE == 32U);
  assert(I2CSTEPPER_V3_CONFIG_A_SIZE == 20U);
  assert(I2CSTEPPER_V3_CONFIG_B_SIZE == 17U);
  assert(I2CSTEPPER_V3_MOTION_SIZE == 10U);
  assert(I2CSTEPPER_V3_COMMAND_SIZE == 6U);
  assert(sizeof(I2CStepperV2Config) == 23U);
  assert(I2CSTEPPER_V3_STATUS_GENERATION_OFFSET == 8U);
  assert(I2CSTEPPER_V3_STATUS_COMMAND_SEQ_OFFSET == 12U);
  assert(I2CSTEPPER_V3_STATUS_ACK_SEQ_OFFSET == 16U);
  assert(I2CSTEPPER_V3_STATUS_STOP_EVENT_SEQ_OFFSET == 20U);
  assert(I2CSTEPPER_V3_STATUS_CURRENT_SPEED_OFFSET == 24U);
  assert(I2CSTEPPER_V3_STATUS_REMAINING_STEPS_OFFSET == 28U);
  assert(I2CSTEPPER_V3_CONFIG_A_PUMP_MLH_OFFSET == 16U);
  assert(I2CSTEPPER_V3_CONFIG_B_STEPS_PER_ML_OFFSET == 13U);
  assert(I2CSTEPPER_V3_MOTION_SPEED_OFFSET == 2U);
  assert(I2CSTEPPER_V3_MOTION_TARGET_OFFSET == 6U);
  assert(I2CSTEPPER_V3_COMMAND_SEQ_OFFSET == 2U);

  uint8_t value[4] = {0, 0, 0, 0};
  i2cstepper_v3_write_u32_be(value, 0x1234ABCDUL);
  assert(value[0] == 0x12U);
  assert(value[1] == 0x34U);
  assert(value[2] == 0xABU);
  assert(value[3] == 0xCDU);
  assert(i2cstepper_v3_read_u32_be(value) == 0x1234ABCDUL);

  I2CStepperV3Identity identity = {2U, I2CSTEPPER_V3_CAP_PUMP};
  uint8_t identityFrame[I2CSTEPPER_V3_IDENTITY_SIZE] = {0};
  i2cstepper_v3_encode_identity(identityFrame, &identity);
  const uint8_t expectedIdentity[] = {0x53U, 3U, 2U, 2U, 0U, 0U, 0U, 0U};
  for (uint8_t i = 0; i < I2CSTEPPER_V3_IDENTITY_SIZE; i++) assert(identityFrame[i] == expectedIdentity[i]);
  I2CStepperV3Identity decodedIdentity = {0, 0};
  assert(i2cstepper_v3_decode_identity(identityFrame, &decodedIdentity));
  assert(decodedIdentity.address == 2U && decodedIdentity.capabilities == I2CSTEPPER_V3_CAP_PUMP);
  identityFrame[I2CSTEPPER_V3_IDENTITY_VERSION_OFFSET] = 2U;
  assert(!i2cstepper_v3_decode_identity(identityFrame, &decodedIdentity));

  I2CStepperV3Config config = {
    2U, I2CSTEPPER_V3_MODE_FILLING, 2U, 4U, 9U,
    0x01020304UL, 0x05060708UL, 0x090A0B0CUL, 0x0D0E0F10UL,
    15UL, 321UL, 800UL, 0x12345678UL,
  };
  uint8_t configA[I2CSTEPPER_V3_CONFIG_A_SIZE] = {0};
  i2cstepper_v3_encode_config_a(configA, &config);
  const uint8_t expectedConfigA[] = {
    3U, 2U, 4U, 9U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U,
    9U, 10U, 11U, 12U, 13U, 14U, 15U, 16U,
  };
  for (uint8_t i = 0; i < I2CSTEPPER_V3_CONFIG_A_SIZE; i++) assert(configA[i] == expectedConfigA[i]);
  I2CStepperV3Config decodedConfig = {};
  i2cstepper_v3_decode_config_a(configA, &decodedConfig);
  assert(decodedConfig.mode == config.mode);
  assert(decodedConfig.optionFlags == config.optionFlags);
  assert(decodedConfig.sensorFlags == config.sensorFlags);
  assert(decodedConfig.relayMask == config.relayMask);
  assert(decodedConfig.mixerRpm == config.mixerRpm);
  assert(decodedConfig.mixerRunSec == config.mixerRunSec);
  assert(decodedConfig.mixerPauseSec == config.mixerPauseSec);
  assert(decodedConfig.pumpMlHour == config.pumpMlHour);

  uint8_t configB[I2CSTEPPER_V3_CONFIG_B_SIZE] = {0};
  i2cstepper_v3_encode_config_b(configB, &config);
  const uint8_t expectedConfigB[] = {
    2U, 0U, 0U, 0U, 15U, 0U, 0U, 1U, 0x41U,
    0U, 0U, 3U, 0x20U, 0x12U, 0x34U, 0x56U, 0x78U,
  };
  for (uint8_t i = 0; i < I2CSTEPPER_V3_CONFIG_B_SIZE; i++) assert(configB[i] == expectedConfigB[i]);
  i2cstepper_v3_decode_config_b(configB, &decodedConfig);
  assert(decodedConfig.address == config.address);
  assert(decodedConfig.pumpPauseSec == config.pumpPauseSec);
  assert(decodedConfig.fillingMl == config.fillingMl);
  assert(decodedConfig.fillingMlHour == config.fillingMlHour);
  assert(decodedConfig.stepsPerMl == config.stepsPerMl);

  I2CStepperV3Motion motion = {I2CSTEPPER_V3_MODE_FILLING, 1U, 0x00010000UL, 0x0001E240UL};
  uint8_t motionFrame[I2CSTEPPER_V3_MOTION_SIZE] = {0};
  i2cstepper_v3_encode_motion(motionFrame, &motion);
  const uint8_t expectedMotion[] = {3U, 1U, 0U, 1U, 0U, 0U, 0U, 1U, 0xE2U, 0x40U};
  for (uint8_t i = 0; i < I2CSTEPPER_V3_MOTION_SIZE; i++) assert(motionFrame[i] == expectedMotion[i]);
  I2CStepperV3Motion decodedMotion = {};
  i2cstepper_v3_decode_motion(motionFrame, &decodedMotion);
  assert(decodedMotion.mode == motion.mode);
  assert(decodedMotion.direction == motion.direction);
  assert(decodedMotion.speedStepsPerSec == motion.speedStepsPerSec);
  assert(decodedMotion.targetSteps == motion.targetSteps);

  I2CStepperV3CommandFrame command = {2U, I2CSTEPPER_V3_CMD_START_FINITE, 0x12345678UL};
  uint8_t commandFrame[I2CSTEPPER_V3_COMMAND_SIZE] = {0};
  i2cstepper_v3_encode_command(commandFrame, &command);
  const uint8_t expectedCommand[] = {2U, 3U, 0x12U, 0x34U, 0x56U, 0x78U};
  for (uint8_t i = 0; i < I2CSTEPPER_V3_COMMAND_SIZE; i++) assert(commandFrame[i] == expectedCommand[i]);
  I2CStepperV3CommandFrame decodedCommand = {};
  i2cstepper_v3_decode_command(commandFrame, &decodedCommand);
  assert(decodedCommand.address == command.address);
  assert(decodedCommand.command == command.command);
  assert(decodedCommand.commandSeq == command.commandSeq);

  I2CStepperV3CommandFrame configuredCommand = {
    2U, I2CSTEPPER_V3_CMD_START_CONFIGURED, 0x01020304UL,
  };
  i2cstepper_v3_encode_command(commandFrame, &configuredCommand);
  const uint8_t expectedConfiguredCommand[] = {2U, 10U, 1U, 2U, 3U, 4U};
  for (uint8_t i = 0; i < I2CSTEPPER_V3_COMMAND_SIZE; i++) {
    assert(commandFrame[i] == expectedConfiguredCommand[i]);
  }
  i2cstepper_v3_decode_command(commandFrame, &decodedCommand);
  assert(decodedCommand.address == configuredCommand.address);
  assert(decodedCommand.command == configuredCommand.command);
  assert(decodedCommand.commandSeq == configuredCommand.commandSeq);

  assert(i2cstepper_v3_address_valid(1U));
  assert(i2cstepper_v3_address_valid(10U));
  assert(!i2cstepper_v3_address_valid(0U));
  assert(!i2cstepper_v3_address_valid(11U));
  assert(i2cstepper_v3_address_is_mixer(1U));
  assert(!i2cstepper_v3_address_is_mixer(2U));
  assert(i2cstepper_v3_mode_supported(1U, I2CSTEPPER_V3_MODE_MIXER));
  assert(!i2cstepper_v3_mode_supported(1U, I2CSTEPPER_V3_MODE_PUMP));
  assert(i2cstepper_v3_mode_supported(2U, I2CSTEPPER_V3_MODE_PUMP));
  assert(i2cstepper_v3_mode_supported(2U, I2CSTEPPER_V3_MODE_FILLING));

  uint8_t changedMode = I2CSTEPPER_V3_MODE_MIXER;
  assert(i2cstepper_v3_mode_after_address_change(2U, 4U, I2CSTEPPER_V3_MODE_FILLING, &changedMode));
  assert(changedMode == I2CSTEPPER_V3_MODE_FILLING);
  assert(i2cstepper_v3_mode_after_address_change(2U, 1U, I2CSTEPPER_V3_MODE_FILLING, &changedMode));
  assert(changedMode == I2CSTEPPER_V3_MODE_MIXER);
  assert(i2cstepper_v3_mode_after_address_change(1U, 2U, I2CSTEPPER_V3_MODE_MIXER, &changedMode));
  assert(changedMode == I2CSTEPPER_V3_MODE_PUMP);
  assert(!i2cstepper_v3_mode_after_address_change(0U, 2U, I2CSTEPPER_V3_MODE_MIXER, &changedMode));
  assert(!i2cstepper_v3_mode_after_address_change(1U, 11U, I2CSTEPPER_V3_MODE_MIXER, &changedMode));
  assert(!i2cstepper_v3_mode_after_address_change(1U, 3U, I2CSTEPPER_V3_MODE_PUMP, &changedMode));

  I2CStepperV3Config addressReadback = {};
  addressReadback.address = 3U;
  addressReadback.mode = I2CSTEPPER_V3_MODE_MIXER;
  addressReadback.stepsPerMl = 123456UL;
  uint8_t addressEeprom[42] = {0x53U, 3U, 37U};
  i2cstepper_v3_encode_config_a(&addressEeprom[5], &addressReadback);
  i2cstepper_v3_encode_config_b(&addressEeprom[25], &addressReadback);
  uint16_t addressCrc = i2cstepper_v3_crc16(&addressEeprom[5], 37U);
  addressEeprom[3] = (uint8_t)(addressCrc >> 8);
  addressEeprom[4] = (uint8_t)addressCrc;
  assert(i2cstepper_v3_crc16(&addressEeprom[5], 37U) ==
         ((uint16_t)addressEeprom[3] << 8 | addressEeprom[4]));
  I2CStepperV3Config decodedAddressReadback = {};
  i2cstepper_v3_decode_config_b(&addressEeprom[25], &decodedAddressReadback);
  assert(decodedAddressReadback.address == 3U && decodedAddressReadback.stepsPerMl == 123456UL);
  uint8_t bootAddress = 1U;
  assert(i2cstepper_v3_runtime_address_matches(bootAddress, 1U));
  assert(!i2cstepper_v3_runtime_address_matches(bootAddress, decodedAddressReadback.address));
  bootAddress = decodedAddressReadback.address;
  assert(i2cstepper_v3_runtime_address_matches(bootAddress, 3U));

  addressReadback.address = 2U;
  addressReadback.mode = I2CSTEPPER_V3_MODE_PUMP;
  i2cstepper_v3_encode_config_a(&addressEeprom[5], &addressReadback);
  i2cstepper_v3_encode_config_b(&addressEeprom[25], &addressReadback);
  addressCrc = i2cstepper_v3_crc16(&addressEeprom[5], 37U);
  addressEeprom[3] = (uint8_t)(addressCrc >> 8);
  addressEeprom[4] = (uint8_t)addressCrc;
  i2cstepper_v3_decode_config_b(&addressEeprom[25], &decodedAddressReadback);
  assert(decodedAddressReadback.address == 2U);
  bootAddress = 1U;
  assert(i2cstepper_v3_runtime_address_matches(bootAddress, 1U));
  assert(!i2cstepper_v3_runtime_address_matches(bootAddress, decodedAddressReadback.address));
  bootAddress = decodedAddressReadback.address;
  assert(i2cstepper_v3_runtime_address_matches(bootAddress, 2U));

  assert(i2cstepper_v3_read_size_valid(1U));
  assert(i2cstepper_v3_read_size_valid(32U));
  assert(!i2cstepper_v3_read_size_valid(0U));
  assert(!i2cstepper_v3_read_size_valid(33U));
  assert(i2cstepper_v3_write_payload_size_valid(0U));
  assert(i2cstepper_v3_write_payload_size_valid(31U));
  assert(!i2cstepper_v3_write_payload_size_valid(32U));

  assert(i2cstepper_v3_sequence_state(0UL, 1UL) == I2CSTEPPER_V3_SEQUENCE_NEW);
  assert(i2cstepper_v3_sequence_state(42UL, 42UL) == I2CSTEPPER_V3_SEQUENCE_DUPLICATE);
  assert(i2cstepper_v3_sequence_state(42UL, 41UL) == I2CSTEPPER_V3_SEQUENCE_STALE);
  assert(i2cstepper_v3_sequence_state(42UL, 0UL) == I2CSTEPPER_V3_SEQUENCE_INVALID);
  assert(i2cstepper_v3_sequence_state(0xFFFFFFFFUL, 1UL) == I2CSTEPPER_V3_SEQUENCE_NEW);
  assert(i2cstepper_v3_sequence_state(1UL, 0xFFFFFFFFUL) == I2CSTEPPER_V3_SEQUENCE_STALE);
  assert(i2cstepper_v3_sequence_next(0xFFFFFFFFUL) == 1UL);
  uint32_t stopEventSeq = 0UL;
  stopEventSeq = i2cstepper_v3_sequence_next(stopEventSeq);
  assert(stopEventSeq == 1UL);
  assert(i2cstepper_v3_start_mode_supported(I2CSTEPPER_V3_MODE_MIXER,
                                             I2CSTEPPER_V3_MODE_MIXER,
                                             I2CSTEPPER_V3_CMD_START_FINITE));
  assert(i2cstepper_v3_start_mode_supported(I2CSTEPPER_V3_MODE_FILLING,
                                             I2CSTEPPER_V3_MODE_FILLING,
                                             I2CSTEPPER_V3_CMD_START_FINITE));
  assert(i2cstepper_v3_start_mode_supported(I2CSTEPPER_V3_MODE_PUMP,
                                             I2CSTEPPER_V3_MODE_PUMP,
                                             I2CSTEPPER_V3_CMD_START_CONTINUOUS));
  assert(i2cstepper_v3_start_mode_supported(I2CSTEPPER_V3_MODE_MIXER,
                                             I2CSTEPPER_V3_MODE_MIXER,
                                             I2CSTEPPER_V3_CMD_START_CONTINUOUS));
  assert(!i2cstepper_v3_start_mode_supported(I2CSTEPPER_V3_MODE_PUMP,
                                              I2CSTEPPER_V3_MODE_PUMP,
                                              I2CSTEPPER_V3_CMD_START_FINITE));
  assert(!i2cstepper_v3_start_mode_supported(I2CSTEPPER_V3_MODE_FILLING,
                                              I2CSTEPPER_V3_MODE_FILLING,
                                              I2CSTEPPER_V3_CMD_START_CONTINUOUS));
  assert(!i2cstepper_v3_start_mode_supported(I2CSTEPPER_V3_MODE_FILLING,
                                              I2CSTEPPER_V3_MODE_PUMP,
                                              I2CSTEPPER_V3_CMD_START_FINITE));
  assert(!i2cstepper_v3_start_mode_supported(I2CSTEPPER_V3_MODE_PUMP,
                                              I2CSTEPPER_V3_MODE_MIXER,
                                              I2CSTEPPER_V3_CMD_START_CONTINUOUS));
  assert(I2CSTEPPER_V3_ERR_UNSUPPORTED_MODE == 2U);
  stopEventSeq = i2cstepper_v3_sequence_next(0xFFFFFFFFUL);
  assert(stopEventSeq == 1UL);

  // Fixed v3 fixtures: register pointer is followed by the documented payload.
  const uint8_t mixerFinite[] = {
    I2CSTEPPER_V3_REG_MOTION, I2CSTEPPER_V3_MODE_MIXER, 0U,
    0U, 0U, 0x03U, 0xE8U,  // 1000 steps/s
    0U, 0U, 0xEAU, 0x60U,  // 60000 target steps
  };
  assert(i2cstepper_v3_read_u32_be(&mixerFinite[3]) == 1000UL);
  assert(i2cstepper_v3_read_u32_be(&mixerFinite[7]) == 60000UL);

  const uint8_t fillingFinite[] = {
    I2CSTEPPER_V3_REG_MOTION, I2CSTEPPER_V3_MODE_FILLING, 1U,
    0U, 0U, 0x07U, 0xD0U,  // 2000 steps/s
    0U, 0x01U, 0xE2U, 0x40U,  // 123456 target steps
  };
  assert(i2cstepper_v3_read_u32_be(&fillingFinite[3]) == 2000UL);
  assert(i2cstepper_v3_read_u32_be(&fillingFinite[7]) == 123456UL);

  const uint8_t pumpContinuous[] = {
    I2CSTEPPER_V3_REG_COMMAND, 2U, I2CSTEPPER_V3_CMD_START_CONTINUOUS,
    0U, 0U, 0U, 0x2AU,
  };
  assert(pumpContinuous[1] == 2U);
  assert(pumpContinuous[2] == I2CSTEPPER_V3_CMD_START_CONTINUOUS);
  assert(i2cstepper_v3_read_u32_be(&pumpContinuous[3]) == 42UL);

  const uint8_t mixerContinuous[] = {
    I2CSTEPPER_V3_REG_COMMAND, 1U, I2CSTEPPER_V3_CMD_START_CONTINUOUS,
    0U, 0U, 0U, 0x2CU,
  };
  assert(mixerContinuous[1] == 1U);
  assert(mixerContinuous[2] == I2CSTEPPER_V3_CMD_START_CONTINUOUS);
  assert(i2cstepper_v3_read_u32_be(&mixerContinuous[3]) == 44UL);

  const uint8_t configuredStart[] = {
    I2CSTEPPER_V3_REG_COMMAND, 2U, 10U, 0U, 0U, 0U, 45U,
  };
  assert(configuredStart[1] == 2U);
  assert(configuredStart[2] == 10U);
  assert(i2cstepper_v3_read_u32_be(&configuredStart[3]) == 45UL);

  const uint8_t pumpContinuousRetry[] = {
    I2CSTEPPER_V3_REG_COMMAND, 2U, I2CSTEPPER_V3_CMD_START_CONTINUOUS,
    0U, 0U, 0U, 0x2AU,
  };
  assert(i2cstepper_v3_sequence_state(42UL, i2cstepper_v3_read_u32_be(&pumpContinuousRetry[3])) == I2CSTEPPER_V3_SEQUENCE_DUPLICATE);

  const uint8_t calibrationFinish[] = {
    I2CSTEPPER_V3_REG_COMMAND, 2U, I2CSTEPPER_V3_CMD_CALIBRATE_FINISH,
    0U, 0U, 0U, 0x2BU,
  };
  assert(calibrationFinish[2] == I2CSTEPPER_V3_CMD_CALIBRATE_FINISH);
  assert(i2cstepper_v3_read_u32_be(&calibrationFinish[3]) == 43UL);

  const uint8_t calibrationFinishRetry[] = {
    I2CSTEPPER_V3_REG_COMMAND, 2U, I2CSTEPPER_V3_CMD_CALIBRATE_FINISH,
    0U, 0U, 0U, 0x2BU,
  };
  assert(i2cstepper_v3_sequence_state(43UL, i2cstepper_v3_read_u32_be(&calibrationFinishRetry[3])) == I2CSTEPPER_V3_SEQUENCE_DUPLICATE);

  const uint8_t badConfigStatus[] = {
    0x53U, 3U, 2U, 3U, 0x80U, 3U, 3U, 0U,
    0U, 0U, 0U, 9U, 0U, 0U, 0U, 44U,
    0U, 0U, 0U, 44U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
  };
  assert(badConfigStatus[0] == I2CSTEPPER_V3_MAGIC);
  assert(badConfigStatus[1] == I2CSTEPPER_V3_VERSION);
  assert(badConfigStatus[2] == 2U);
  assert(badConfigStatus[3] == I2CSTEPPER_V3_MODE_FILLING);
  assert(badConfigStatus[4] == I2CSTEPPER_V3_STATUS_ERROR);
  assert(badConfigStatus[5] == I2CSTEPPER_V3_RESULT_FAILED);
  assert(badConfigStatus[6] == I2CSTEPPER_V3_ERR_BAD_CONFIG);
  assert(badConfigStatus[7] == I2CSTEPPER_V3_STOP_NONE);
  assert(i2cstepper_v3_read_u32_be(&badConfigStatus[8]) == 9UL);
  assert(i2cstepper_v3_read_u32_be(&badConfigStatus[12]) == 44UL);
  assert(i2cstepper_v3_read_u32_be(&badConfigStatus[16]) == 44UL);
  assert(i2cstepper_v3_read_u32_be(&badConfigStatus[20]) == 0UL);
  assert(i2cstepper_v3_read_u32_be(&badConfigStatus[24]) == 0UL);
  assert(i2cstepper_v3_read_u32_be(&badConfigStatus[28]) == 0UL);
  I2CStepperV3StatusSnapshot decodedStatus = {};
  assert(i2cstepper_v3_decode_status(badConfigStatus, &decodedStatus));
  assert(decodedStatus.mode == I2CSTEPPER_V3_MODE_FILLING && decodedStatus.commandResult == I2CSTEPPER_V3_RESULT_FAILED);
  uint8_t encodedStatus[I2CSTEPPER_V3_STATUS_SIZE] = {0};
  i2cstepper_v3_encode_status(encodedStatus, &decodedStatus);
  for (uint8_t i = 0; i < I2CSTEPPER_V3_STATUS_SIZE; i++) assert(encodedStatus[i] == badConfigStatus[i]);
  encodedStatus[I2CSTEPPER_V3_STATUS_MAGIC_OFFSET] = 0U;
  assert(!i2cstepper_v3_decode_status(encodedStatus, &decodedStatus));

  const uint8_t rejectedAddressStatus[] = {
    I2CSTEPPER_V3_MAGIC, I2CSTEPPER_V3_VERSION, 2U, I2CSTEPPER_V3_MODE_PUMP,
    I2CSTEPPER_V3_STATUS_ERROR, I2CSTEPPER_V3_RESULT_FAILED,
    I2CSTEPPER_V3_ERR_BAD_ADDRESS, I2CSTEPPER_V3_STOP_NONE,
    0U, 0U, 0U, 9U, 0U, 0U, 0U, 44U,
    0U, 0U, 0U, 44U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
  };
  assert(rejectedAddressStatus[5] == I2CSTEPPER_V3_RESULT_FAILED);
  assert(i2cstepper_v3_read_u32_be(&rejectedAddressStatus[12]) == 44UL);
  assert(i2cstepper_v3_read_u32_be(&rejectedAddressStatus[16]) == 44UL);

  const uint8_t heartbeatTimeoutStatus[] = {
    0x53U, 3U, 2U, 2U, 0x80U, 3U, 5U, 5U,
    0U, 0U, 0U, 10U, 0U, 0U, 0U, 42U,
    0U, 0U, 0U, 42U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
  };
  assert(heartbeatTimeoutStatus[0] == I2CSTEPPER_V3_MAGIC);
  assert(heartbeatTimeoutStatus[1] == I2CSTEPPER_V3_VERSION);
  assert(heartbeatTimeoutStatus[2] == 2U);
  assert(heartbeatTimeoutStatus[3] == I2CSTEPPER_V3_MODE_PUMP);
  assert(heartbeatTimeoutStatus[4] == I2CSTEPPER_V3_STATUS_ERROR);
  assert(heartbeatTimeoutStatus[5] == I2CSTEPPER_V3_RESULT_FAILED);
  assert(heartbeatTimeoutStatus[6] == I2CSTEPPER_V3_ERR_HEARTBEAT_TIMEOUT);
  assert(heartbeatTimeoutStatus[7] == I2CSTEPPER_V3_STOP_HEARTBEAT);
  assert(i2cstepper_v3_read_u32_be(&heartbeatTimeoutStatus[8]) == 10UL);
  assert(i2cstepper_v3_read_u32_be(&heartbeatTimeoutStatus[12]) == 42UL);
  assert(i2cstepper_v3_read_u32_be(&heartbeatTimeoutStatus[16]) == 42UL);
  assert(i2cstepper_v3_read_u32_be(&heartbeatTimeoutStatus[20]) == 0UL);
  assert(i2cstepper_v3_read_u32_be(&heartbeatTimeoutStatus[24]) == 0UL);
  assert(i2cstepper_v3_read_u32_be(&heartbeatTimeoutStatus[28]) == 0UL);

  const uint8_t finiteModeMismatch[] = {
    I2CSTEPPER_V3_REG_MOTION, I2CSTEPPER_V3_MODE_PUMP, 0U,
    0U, 0U, 3U, 0xE8U, 0U, 0U, 0U, 100U,
  };
  i2cstepper_v3_decode_motion(&finiteModeMismatch[1], &decodedMotion);
  assert(decodedMotion.mode == I2CSTEPPER_V3_MODE_PUMP);
  assert(!i2cstepper_v3_start_mode_supported(I2CSTEPPER_V3_MODE_FILLING,
                                              decodedMotion.mode,
                                              I2CSTEPPER_V3_CMD_START_FINITE));

  const uint8_t continuousModeMismatch[] = {
    I2CSTEPPER_V3_REG_MOTION, I2CSTEPPER_V3_MODE_MIXER, 0U,
    0U, 0U, 3U, 0xE8U, 0U, 0U, 0U, 0U,
  };
  i2cstepper_v3_decode_motion(&continuousModeMismatch[1], &decodedMotion);
  assert(decodedMotion.mode == I2CSTEPPER_V3_MODE_MIXER);
  assert(!i2cstepper_v3_start_mode_supported(I2CSTEPPER_V3_MODE_PUMP,
                                              decodedMotion.mode,
                                              I2CSTEPPER_V3_CMD_START_CONTINUOUS));

  assert(I2CSTEPPER_V3_CMD_START_FINITE != I2CSTEPPER_V3_CMD_START_CONTINUOUS);
  assert(I2CSTEPPER_V3_CMD_STOP != I2CSTEPPER_V3_CMD_HEARTBEAT);

  const uint8_t mixerV2Eeprom[] = {
    0x53U, 2U, 1U, 1U, 0xB0U, 0x04U, 0x10U, 0x0EU, 0x3CU, 0U,
    0xF4U, 1U, 7U, 0U, 0xFAU, 0U, 0x58U, 2U, 0x80U, 0x3EU,
    2U, 2U, 5U,
  };
  const uint8_t pumpV2Eeprom[] = {
    0x53U, 2U, 2U, 2U, 20U, 0U, 0U, 0U, 0U, 0U,
    0xB8U, 0x0BU, 15U, 0U, 100U, 0U, 200U, 0U, 0xFFU, 0xFFU,
    3U, 5U, 10U,
  };
  const uint8_t fillingV2Eeprom[] = {
    0x53U, 2U, 2U, 3U, 25U, 0U, 0U, 0U, 0U, 0U,
    0xD0U, 7U, 30U, 0U, 0x41U, 1U, 0x20U, 3U, 0x39U, 0x30U,
    2U, 4U, 9U,
  };
  I2CStepperV2Config legacy;
  I2CStepperV3Config migrated;
  assert(i2cstepper_v3_decode_v2_config(mixerV2Eeprom, &legacy));
  assert(i2cstepper_v3_migrate_v2_config(&legacy, &migrated));
  assert(migrated.address == 1U && migrated.mode == I2CSTEPPER_V3_MODE_MIXER);
  assert(migrated.mixerRpm == 1200UL && migrated.mixerRunSec == 3600UL);
  assert(migrated.stepsPerMl == 16000UL && migrated.relayMask == 5U);

  assert(i2cstepper_v3_decode_v2_config(pumpV2Eeprom, &legacy));
  assert(i2cstepper_v3_migrate_v2_config(&legacy, &migrated));
  assert(migrated.address == 2U && migrated.mode == I2CSTEPPER_V3_MODE_PUMP);
  assert(migrated.pumpMlHour == 3000UL && migrated.stepsPerMl == 65535UL);

  assert(i2cstepper_v3_decode_v2_config(fillingV2Eeprom, &legacy));
  assert(i2cstepper_v3_migrate_v2_config(&legacy, &migrated));
  assert(migrated.address == 2U && migrated.mode == I2CSTEPPER_V3_MODE_FILLING);
  assert(migrated.fillingMl == 321UL && migrated.fillingMlHour == 800UL);
  assert(migrated.stepsPerMl == 12345UL);

  return 0;
}
