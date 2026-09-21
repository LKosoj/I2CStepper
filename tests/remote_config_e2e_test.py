#!/usr/bin/env python3
# Сквозная проверка: пакеты в том виде, как их шлёт Самовар (CONFIG_A, CONFIG_B, команда), проходят
# через настоящие v3_wire_receive и v3_process_receive из I2CStepper.ino. Смотрим, что флаг плавного
# старта и реле действительно доходят до настроек мотора и до выводов реле.
from i2c_stepper_v3_runtime_test import ROOT, compile_and_run, function_body

PROTOCOL = ROOT.parent / "Samovar" / "libraries" / "I2CStepperProtocol" / "src"

HARNESS = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <I2CStepperV3.h>
#include "I2CStepperRuntime.h"
#include "StepperMath.h"

typedef uint8_t byte;
#define LOW 0
#define STEPPER_STEPS 3200
#define bit_is_set(value, bit) (((value) & (1U << (bit))) != 0)
#define I2CSTEPPER_FLAG_SMOOTH_START 0x02
#define I2CSTEPPER_FLAG_DIRECTION 0x04
enum I2CType {I2CMIXER = 1, I2CPUMP = 2, I2CFILLING = 3};
struct SetupEEPROM {
  byte role, mode;
  uint32_t mixerRpm, mixerRunSec, mixerPauseSec, pumpMlHour, pumpPauseSec, fillingMl, fillingMlHour, stepperStepMl;
  byte optionFlags, sensorFlags, relayMask;
} I2CSTPSetup = {};

// Шина I2C: байты одного пакета от Самовара.
struct FakeWire {
  uint8_t data[40];
  int size = 0, pos = 0;
  int available() { return size - pos; }
  int read() { return pos < size ? data[pos++] : -1; }
} Wire;
struct FakeStepper { bool getState() { return false; } int32_t getCurrent() { return 0; } } stepper;

uint8_t v3_rx_config_a[I2CSTEPPER_V3_CONFIG_A_SIZE], v3_rx_config_b[I2CSTEPPER_V3_CONFIG_B_SIZE];
uint8_t v3_rx_motion[I2CSTEPPER_V3_MOTION_SIZE], v3_rx_command[I2CSTEPPER_V3_COMMAND_SIZE];
volatile bool v3_rx_config_a_pending, v3_rx_config_b_pending, v3_rx_motion_pending, v3_rx_command_pending;
bool v3_config_a_fresh = false, v3_config_b_fresh = false;
uint8_t v3_read_register = 0;
I2CStepperV3Config v3_active_config = {}, v3_staging_config = {};
I2CStepperV3Motion v3_staging_motion = {};
I2CStepperV3StatusSnapshot v3_status_snapshot = {};
uint8_t v3_runtime_address = 1, v3_runtime_mode = I2CSTEPPER_V3_MODE_MIXER;
uint32_t v3_last_sequence = 0;
bool calibration_active = false;
byte rele_state = 0, set_dir = 0;
uint8_t rele_pin[] = {13, 10, 11, 12};
uint8_t pins[14] = {};
uint32_t set_spd = 0, set_time = 0, last_set_time = 0;
bool set_time_initialized = false, set_dir_initialized = false;
uint8_t starts = 0;

uint8_t lock_interrupts() { return 0; }
void unlock_interrupts(uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t value) { pins[pin] = value; }
void v3_publish_frames() {}
void v3_claim_remote_ownership() {}
uint8_t v3_apply_address_error() { return I2CSTEPPER_V3_ERR_NONE; }
uint8_t v3_start_precondition_error() { return I2CSTEPPER_V3_ERR_NONE; }
bool v3_eeprom_write(const I2CStepperV3Config&) { return true; }
bool v3_start_configured_motion() { starts++; return true; }
void v3_start_staged_motion(bool) { starts++; }
void stop_stepper() {}
bool v3_finish_calibration() { return true; }
void start_motion(uint16_t, uint32_t, byte, bool) {}
bool v3_motion_valid(bool) { return true; }

@FIRMWARE@

static void deliver(uint8_t reg, const uint8_t* payload, uint8_t size) {
  Wire.data[0] = reg;
  memcpy(&Wire.data[1], payload, size);
  Wire.size = size + 1;
  Wire.pos = 0;
  v3_wire_receive(Wire.size);
}

// То же, что делает Самовар: CONFIG_A, CONFIG_B, затем команда с новым номером.
static void samovar_sends(const I2CStepperV3Config& config, uint8_t command) {
  uint8_t a[I2CSTEPPER_V3_CONFIG_A_SIZE] = {}, b[I2CSTEPPER_V3_CONFIG_B_SIZE] = {};
  uint8_t c[I2CSTEPPER_V3_COMMAND_SIZE] = {};
  i2cstepper_v3_encode_config_a(a, &config);
  i2cstepper_v3_encode_config_b(b, &config);
  I2CStepperV3CommandFrame frame = {};
  frame.address = 1;
  frame.command = command;
  frame.commandSeq = i2cstepper_v3_sequence_next(v3_last_sequence);
  i2cstepper_v3_encode_command(c, &frame);
  deliver(I2CSTEPPER_V3_REG_CONFIG_A, a, sizeof(a));
  deliver(I2CSTEPPER_V3_REG_CONFIG_B, b, sizeof(b));
  deliver(I2CSTEPPER_V3_REG_COMMAND, c, sizeof(c));
  v3_process_receive();
  assert(v3_status_snapshot.ackSeq == frame.commandSeq);
  assert(v3_status_snapshot.commandResult == I2CSTEPPER_V3_RESULT_SUCCESS);
}

int main() {
  I2CStepperV3Config config = {};
  config.address = 1;
  config.mode = I2CSTEPPER_V3_MODE_MIXER;
  config.optionFlags = I2CSTEPPER_FLAG_SMOOTH_START;
  config.mixerRpm = 20;
  config.pumpMlHour = 100;
  config.fillingMl = 100;
  config.fillingMlHour = 100;
  config.stepsPerMl = 16000;
  v3_active_config = v3_staging_config = config;
  v3_apply_to_runtime();
  assert(I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_SMOOTH_START);

  // Один новый блок нельзя смешивать со старой половиной конфигурации.
  config.mixerRpm = 100;
  uint8_t b[I2CSTEPPER_V3_CONFIG_B_SIZE] = {};
  uint8_t c[I2CSTEPPER_V3_COMMAND_SIZE] = {};
  i2cstepper_v3_encode_config_b(b, &config);
  I2CStepperV3CommandFrame incomplete = {1, I2CSTEPPER_V3_CMD_SAVE,
      i2cstepper_v3_sequence_next(v3_last_sequence)};
  i2cstepper_v3_encode_command(c, &incomplete);
  deliver(I2CSTEPPER_V3_REG_CONFIG_B, b, sizeof(b));
  deliver(I2CSTEPPER_V3_REG_COMMAND, c, sizeof(c));
  v3_process_receive();
  assert(v3_status_snapshot.commandResult == I2CSTEPPER_V3_RESULT_FAILED);
  assert(v3_status_snapshot.error == I2CSTEPPER_V3_ERR_BAD_CONFIG);
  assert(v3_active_config.mixerRpm == 20);

  // Галочку сняли: и «Применить», и «Сохранить» должны убрать флаг из рабочих настроек мотора.
  config.mixerRpm = 20;
  config.optionFlags = 0;
  samovar_sends(config, I2CSTEPPER_V3_CMD_SAVE);
  assert(!(I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_SMOOTH_START));
  config.optionFlags = I2CSTEPPER_FLAG_SMOOTH_START;
  samovar_sends(config, I2CSTEPPER_V3_CMD_APPLY);
  assert(I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_SMOOTH_START);
  config.optionFlags = 0;
  samovar_sends(config, I2CSTEPPER_V3_CMD_APPLY);
  assert(!(I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_SMOOTH_START));
  // Запуск «по настройкам» флаг не возвращает.
  samovar_sends(config, I2CSTEPPER_V3_CMD_START_CONFIGURED);
  assert(starts == 1 && !(I2CSTPSetup.optionFlags & I2CSTEPPER_FLAG_SMOOTH_START));

  // Реле 1 (вывод 13) и реле 3 (вывод 11) включаются командой RELAY и выключаются ею же.
  config.relayMask = 0x05;
  samovar_sends(config, I2CSTEPPER_V3_CMD_RELAY);
  assert(pins[13] == 1 && pins[10] == 0 && pins[11] == 1 && pins[12] == 0 && rele_state == 0x05);
  config.relayMask = 0;
  samovar_sends(config, I2CSTEPPER_V3_CMD_RELAY);
  assert(pins[13] == 0 && pins[11] == 0 && rele_state == 0);

  // Маска реле, пришедшая вместе с «Применить»/«Сохранить», тоже должна дойти до выводов.
  config.relayMask = 0x01;
  samovar_sends(config, I2CSTEPPER_V3_CMD_APPLY);
  assert(rele_state == 0x01);
  assert(pins[13] == 1);
  return 0;
}
'''


def main():
    firmware = "\n".join(
        signature + " {" + function_body(signature) + "}"
        for signature in (
            "static void v3_set_result(uint32_t sequence, uint8_t result, uint8_t error)",
            "static bool v3_config_valid(const I2CStepperV3Config& config)",
            "static void write_relay_pins()",
            "static void v3_apply_to_runtime()",
            "static bool v3_prepare_staged_config(I2CStepperV3Config* prepared)",
            "static bool v3_apply_staged_relay()",
            "static bool v3_take_mailbox(const uint8_t* source, uint8_t size, volatile bool* pending,\n"
            "                            uint8_t* destination)",
            "void v3_wire_receive(int count)",
            "void v3_process_receive()",
        ))
    source = HARNESS.replace("@FIRMWARE@", firmware)
    result = compile_and_run(source, PROTOCOL, "remote_config_e2e")
    assert result.returncode == 0, result.stdout + result.stderr
    for old, new, name in (
        ("const bool completeConfig = v3_config_a_fresh && v3_config_b_fresh;",
         "const bool completeConfig = true;", "missing config completeness check"),
        ("v3_config_a_fresh = true;", "v3_config_a_fresh = false;", "CONFIG_A freshness"),
        ("v3_config_b_fresh = true;", "v3_config_b_fresh = false;", "CONFIG_B freshness"),
    ):
        mutant = source.replace(old, new, 1)
        assert mutant != source, name + " mutation anchor missing"
        mutated = compile_and_run(mutant, PROTOCOL, "remote_config_e2e_mutated")
        assert mutated.returncode != 0, name + " mutation survived"


if __name__ == "__main__":
    main()
