#!/usr/bin/env python3
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "I2CStepper.ino").read_text()
MENU_SOURCE = (ROOT / "I2CMenu.h").read_text()


def function_body(signature):
    start = SOURCE.index(signature + " {")
    open_brace = SOURCE.index("{", start)
    depth = 0
    for index in range(open_brace, len(SOURCE)):
        if SOURCE[index] == "{":
            depth += 1
        elif SOURCE[index] == "}":
            depth -= 1
            if depth == 0:
                return SOURCE[open_brace + 1:index]
    raise AssertionError("unterminated function: " + signature)


def require(body, token):
    assert token in body, "missing " + token


def compile_and_run(source, protocol, name):
    compiler = shutil.which("g++")
    assert compiler is not None, "g++ is required"
    with tempfile.TemporaryDirectory(prefix="i2cstepper-v3-") as temporary:
        directory = Path(temporary)
        cpp = directory / (name + ".cpp")
        binary = directory / name
        cpp.write_text(source, encoding="utf-8")
        built = subprocess.run(
            [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
             "-I", str(protocol), "-I", str(ROOT), str(cpp), "-o", str(binary)],
            capture_output=True, text=True, check=False)
        assert built.returncode == 0, built.stdout + built.stderr
        return subprocess.run([str(binary)], capture_output=True, text=True, check=False)


def require_mutation_fails(source, protocol, name):
    result = compile_and_run(source, protocol, name)
    assert result.returncode != 0, name + " mutation survived"
    assert "Assertion" in result.stdout + result.stderr, name + " mutation failed outside a check"


def assert_mailbox_order(body):
    assert body.index("v3_rx_config_a") < body.index("v3_rx_config_b") < \
           body.index("v3_rx_motion") < body.index("v3_rx_command")
    assert body.index("i2cstepper_v3_decode_config_a") < \
           body.index("i2cstepper_v3_decode_config_b") < \
           body.index("i2cstepper_v3_decode_motion") < \
           body.index("i2cstepper_v3_decode_command")


def source_derived_production_harnesses():
    protocol = ROOT.parent / "Samovar" / "libraries" / "I2CStepperProtocol" / "src"
    assert protocol.is_dir(), "canonical protocol header is unavailable"

    config_body = function_body("static bool v3_config_valid(const I2CStepperV3Config& config)")
    config_definition = "static bool v3_config_valid(const I2CStepperV3Config& config) {" + config_body + "}"
    config_harness = r'''
#include <assert.h>
#include <stdint.h>

#include <I2CStepperV3.h>
#include "I2CStepperRuntime.h"
#include "StepperMath.h"
#define STEPPER_STEPS (200U * 2U)
@CONFIG_VALID@

static I2CStepperV3Config filling(uint32_t ml) {
  I2CStepperV3Config config = {};
  config.address = 2;
  config.mode = I2CSTEPPER_V3_MODE_FILLING;
  config.fillingMl = ml;
  config.fillingMlHour = 3600;
  config.stepsPerMl = 100;
  return config;
}

int main() {
  assert(v3_config_valid(filling(1)));
  assert(!v3_config_valid(filling(0)));
  I2CStepperV3Config mixer = filling(0);
  mixer.address = 1;
  mixer.mode = I2CSTEPPER_V3_MODE_MIXER;
  mixer.mixerRpm = 20;
  mixer.mixerRunSec = 0;
  assert(v3_config_valid(mixer));
  return 0;
}
'''.replace("@CONFIG_VALID@", config_definition)
    assert compile_and_run(config_harness, protocol, "config_valid").returncode == 0
    config_mutation = config_definition.replace(
        "speed, target, targetRequired,",
        "speed, target, static_cast<bool>((void)targetRequired, false),", 1)
    assert config_mutation != config_definition, "targetRequired mutation anchor is missing"
    require_mutation_fails(
        config_harness.replace(config_definition, config_mutation), protocol, "config_valid_mutated")

    precondition_body = function_body("static uint8_t v3_start_precondition_error()")
    precondition_definition = "static uint8_t v3_start_precondition_error() {" + precondition_body + "}"
    precondition_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>
#include "I2CStepperRuntime.h"

bool v3_movement_allowed = true;
uint8_t v3_runtime_address = 1U;
I2CStepperV3Config v3_active_config = {};

@PRECONDITION@

int main() {
  v3_active_config.address = 1U;
  assert(v3_start_precondition_error() == I2CSTEPPER_V3_ERR_NONE);
  v3_active_config.address = 2U;
  assert(v3_start_precondition_error() == I2CSTEPPER_V3_ERR_REBOOT_REQUIRED);
  v3_movement_allowed = false;
  assert(v3_start_precondition_error() == I2CSTEPPER_V3_ERR_EEPROM_INVALID);
  return 0;
}
'''.replace("@PRECONDITION@", precondition_definition)
    assert compile_and_run(precondition_harness, protocol, "start_precondition").returncode == 0
    precondition_mutation = precondition_definition.replace(
        "I2CSTEPPER_V3_ERR_NONE : I2CSTEPPER_V3_ERR_REBOOT_REQUIRED",
        "I2CSTEPPER_V3_ERR_NONE : I2CSTEPPER_V3_ERR_NONE", 1)
    assert precondition_mutation != precondition_definition, "start precondition mutation anchor is missing"
    require_mutation_fails(
        precondition_harness.replace(precondition_definition, precondition_mutation),
        protocol, "start_precondition_mutated")

    apply_error_body = function_body("static uint8_t v3_apply_address_error()")
    apply_error_definition = "static uint8_t v3_apply_address_error() {" + apply_error_body + "}"
    apply_error_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>
#include "I2CStepperRuntime.h"

uint8_t v3_runtime_address = 1U;
I2CStepperV3Config v3_staging_config = {};

@APPLY_ERROR@

int main() {
  v3_staging_config.address = 1U;
  assert(v3_apply_address_error() == I2CSTEPPER_V3_ERR_NONE);
  v3_staging_config.address = 2U;
  assert(v3_apply_address_error() == I2CSTEPPER_V3_ERR_BAD_ADDRESS);
  return 0;
}
'''.replace("@APPLY_ERROR@", apply_error_definition)
    assert compile_and_run(apply_error_harness, protocol, "apply_address").returncode == 0
    apply_error_mutation = apply_error_definition.replace(
        "I2CSTEPPER_V3_ERR_NONE : I2CSTEPPER_V3_ERR_BAD_ADDRESS",
        "I2CSTEPPER_V3_ERR_NONE : I2CSTEPPER_V3_ERR_NONE", 1)
    assert apply_error_mutation != apply_error_definition, "apply address mutation anchor is missing"
    require_mutation_fails(
        apply_error_harness.replace(apply_error_definition, apply_error_mutation),
        protocol, "apply_address_mutated")

    configured_start_body = function_body("static bool v3_start_configured_motion()")
    configured_start_definition = "static bool v3_start_configured_motion() {" + configured_start_body + "}"
    configured_start_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>

I2CStepperV3Config v3_active_config = {};
bool config_is_valid = true;
bool stepper_state = false;
uint8_t start_calls = 0;

static bool v3_config_valid(const I2CStepperV3Config&) { return config_is_valid; }
void start_current_mode() { start_calls++; stepper_state = true; }

@CONFIGURED_START@

int main() {
  config_is_valid = true;
  assert(v3_start_configured_motion());
  assert(start_calls == 1U && stepper_state);
  start_calls = 0;
  stepper_state = false;
  config_is_valid = false;
  assert(!v3_start_configured_motion());
  assert(start_calls == 0U && !stepper_state);
  return 0;
}
'''.replace("@CONFIGURED_START@", configured_start_definition)
    assert compile_and_run(configured_start_harness, protocol, "configured_start").returncode == 0
    configured_start_mutation = configured_start_definition.replace(
        "if (!v3_config_valid(v3_active_config))", "if (false && !v3_config_valid(v3_active_config))", 1)
    assert configured_start_mutation != configured_start_definition, "configured start mutation anchor is missing"
    require_mutation_fails(
        configured_start_harness.replace(configured_start_definition, configured_start_mutation),
        protocol, "configured_start_mutated")

    current_mode_body = function_body("void start_current_mode()")
    current_mode_definition = "void start_current_mode() {" + current_mode_body + "}"
    current_mode_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>
#include "StepperMath.h"

typedef uint8_t byte;
#define I2CMIXER 1U
#define I2CPUMP 2U
#define I2CFILLING 3U
#define STEPPER_STEPS 400U
#define STEPPER_TARGET_LIMIT 2147483647UL
#define I2CSTEPPER_FLAG_DIRECTION 0x04U

struct Setup {
  uint8_t optionFlags;
  uint8_t mode;
  uint32_t mixerRpm;
  uint32_t mixerRunSec;
  uint32_t pumpMlHour;
  uint32_t fillingMl;
  uint32_t fillingMlHour;
  uint32_t stepperStepMl;
} I2CSTPSetup = {};
bool v3_movement_allowed = true;
bool stepper_state = false;
bool v3_mixer_deadline_active = false;
I2CStepperV3StatusSnapshot v3_status_snapshot = {};
uint32_t v3_mixer_deadline_ms = 0;
uint32_t set_spd = 0;
uint32_t set_time = 0;
uint32_t fake_now = 1000;
uint32_t millis() { return fake_now; }
uint16_t recorded_speed = 0;
uint32_t recorded_target = 0;
uint8_t recorded_direction = 0;
bool recorded_continuous = false;
void start_motion(uint16_t speed, uint32_t target, byte direction, bool continuous) {
  recorded_speed = speed;
  recorded_target = target;
  recorded_direction = direction;
  recorded_continuous = continuous;
  stepper_state = true;
}
@CURRENT_MODE@

static void reset() {
  I2CSTPSetup = {};
  stepper_state = false;
  v3_mixer_deadline_active = false;
  recorded_speed = 0;
  recorded_target = 0;
  recorded_direction = 0;
  recorded_continuous = false;
}

int main() {
  reset();
  I2CSTPSetup.mode = I2CMIXER;
  I2CSTPSetup.mixerRpm = 60;
  start_current_mode();
  assert(recorded_speed == 400U && recorded_target == 0U && recorded_continuous);

  reset();
  I2CSTPSetup.mode = I2CMIXER;
  I2CSTPSetup.mixerRpm = 60;
  I2CSTPSetup.mixerRunSec = 2;
  I2CSTPSetup.optionFlags = I2CSTEPPER_FLAG_DIRECTION;
  start_current_mode();
  assert(recorded_speed == 400U && recorded_target == 800U && !recorded_continuous);
  assert(recorded_direction == 1U && v3_mixer_deadline_active && v3_mixer_deadline_ms == 3000U);

  reset();
  I2CSTPSetup.mode = I2CPUMP;
  I2CSTPSetup.pumpMlHour = 3600;
  I2CSTPSetup.stepperStepMl = 100;
  start_current_mode();
  assert(recorded_speed == 100U && recorded_target == 0U && recorded_continuous);

  reset();
  I2CSTPSetup.mode = I2CFILLING;
  I2CSTPSetup.fillingMlHour = 3600;
  I2CSTPSetup.fillingMl = 20;
  I2CSTPSetup.stepperStepMl = 100;
  start_current_mode();
  assert(recorded_speed == 100U && recorded_target == 2000U && !recorded_continuous);
  return 0;
}
'''.replace("@CURRENT_MODE@", current_mode_definition)
    assert compile_and_run(current_mode_harness, protocol, "current_mode").returncode == 0
    current_mode_mutation = current_mode_definition.replace(
        "I2CSTPSetup.mixerRunSec == 0", "false", 1)
    assert current_mode_mutation != current_mode_definition, "mixer configured-start mutation anchor is missing"
    require_mutation_fails(
        current_mode_harness.replace(current_mode_definition, current_mode_mutation),
        protocol, "current_mode_mutated")

    capabilities_body = function_body("static uint8_t v3_capabilities()")
    capabilities_definition = "static uint8_t v3_capabilities() {" + capabilities_body + "}"
    identity_body = function_body("static void v3_publish_runtime_identity(I2CStepperV3Identity* identity)")
    identity_definition = "static void v3_publish_runtime_identity(I2CStepperV3Identity* identity) {" + identity_body + "}"
    identity_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>
#include "I2CStepperRuntime.h"

I2CStepperV3Config v3_active_config = {};
I2CStepperV3StatusSnapshot v3_status_snapshot = {};
uint8_t v3_runtime_address = 1U;
uint8_t v3_runtime_mode = I2CSTEPPER_V3_MODE_MIXER;

@CAPABILITIES@
@IDENTITY@

int main() {
  // SAVE 1->2: persisted/readback config is Pump, physical runtime remains Mixer.
  v3_active_config.address = 2U;
  v3_active_config.mode = I2CSTEPPER_V3_MODE_PUMP;
  v3_runtime_mode = I2CSTEPPER_V3_MODE_MIXER;
  I2CStepperV3Identity identity = {};
  v3_publish_runtime_identity(&identity);
  assert(v3_active_config.address == 2U && v3_active_config.mode == I2CSTEPPER_V3_MODE_PUMP);
  assert(identity.address == 1U);
  assert(identity.capabilities == (I2CSTEPPER_V3_CAP_MIXER | I2CSTEPPER_V3_CAP_RELAY | I2CSTEPPER_V3_CAP_SENSOR));
  assert(v3_status_snapshot.address == 1U && v3_status_snapshot.mode == I2CSTEPPER_V3_MODE_MIXER);
  I2CStepperV3CommandFrame heartbeat = {1U, I2CSTEPPER_V3_CMD_HEARTBEAT, 7U};
  assert(i2cstepper_v3_runtime_address_matches(v3_runtime_address, heartbeat.address));
  heartbeat.address = 2U;
  assert(!i2cstepper_v3_runtime_address_matches(v3_runtime_address, heartbeat.address));

  // Simulated reboot adopts persisted Pump config and its physical address.
  v3_runtime_address = v3_active_config.address;
  v3_runtime_mode = v3_active_config.mode;
  v3_publish_runtime_identity(&identity);
  assert(identity.address == 2U);
  assert(identity.capabilities == (I2CSTEPPER_V3_CAP_PUMP | I2CSTEPPER_V3_CAP_FILLING |
                                  I2CSTEPPER_V3_CAP_RELAY | I2CSTEPPER_V3_CAP_SENSOR));
  assert(v3_status_snapshot.address == 2U && v3_status_snapshot.mode == I2CSTEPPER_V3_MODE_PUMP);
  heartbeat.address = 2U;
  assert(i2cstepper_v3_runtime_address_matches(v3_runtime_address, heartbeat.address));

  // SAVE 1->3 keeps Mixer role until reboot, while CONFIG readback is address 3.
  v3_active_config.address = 3U;
  v3_active_config.mode = I2CSTEPPER_V3_MODE_MIXER;
  v3_runtime_mode = I2CSTEPPER_V3_MODE_MIXER;
  v3_runtime_address = 1U;
  v3_publish_runtime_identity(&identity);
  assert(v3_active_config.address == 3U && v3_active_config.mode == I2CSTEPPER_V3_MODE_MIXER);
  assert(identity.address == 1U && v3_status_snapshot.address == 1U);
  assert(identity.capabilities == (I2CSTEPPER_V3_CAP_MIXER | I2CSTEPPER_V3_CAP_RELAY | I2CSTEPPER_V3_CAP_SENSOR));
  assert(v3_status_snapshot.mode == I2CSTEPPER_V3_MODE_MIXER);
  heartbeat.address = 1U;
  assert(i2cstepper_v3_runtime_address_matches(v3_runtime_address, heartbeat.address));
  return 0;
}
'''.replace("@CAPABILITIES@", capabilities_definition).replace("@IDENTITY@", identity_definition)
    assert compile_and_run(identity_harness, protocol, "runtime_identity").returncode == 0
    identity_mutation = identity_definition.replace("v3_runtime_mode", "v3_active_config.mode", 1)
    assert identity_mutation != identity_definition, "runtime mode mutation anchor is missing"
    require_mutation_fails(
        identity_harness.replace(identity_definition, identity_mutation), protocol, "runtime_identity_mutated")

    result_body = function_body("static void v3_set_result(uint32_t sequence, uint8_t result, uint8_t error)")
    result_definition = "static void v3_set_result(uint32_t sequence, uint8_t result, uint8_t error) {" + result_body + "}"
    timeout_body = function_body("static void v3_stop_remote_for_timeout()")
    timeout_definition = "static void v3_stop_remote_for_timeout() {" + timeout_body + "}"
    timeout_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>
#include "I2CStepperRuntime.h"

typedef uint8_t byte;
#define LOW 0
struct FakeStepper {
  bool running;
  bool getState() const { return running; }
} stepper = {};
bool stepper_state = false;
bool pause_phase = false;
bool calibration_active = false;
uint8_t rele_state = 0;
uint8_t rele_pin[] = {1, 2, 3, 4};
uint8_t relay_levels[4] = {};
uint8_t stop_calls = 0;
I2CStepperV3StatusSnapshot v3_status_snapshot = {};
I2CStepperV3Config v3_active_config = {};
I2CStepperV3Config v3_staging_config = {};
struct { uint8_t relayMask; } I2CSTPSetup = {};

void stop_motion(bool) { stop_calls++; }
void digitalWrite(uint8_t pin, uint8_t level) { relay_levels[pin - 1] = level; }

@SET_RESULT@
@TIMEOUT@

static void reset_state() {
  stepper = {};
  stepper_state = false;
  pause_phase = false;
  calibration_active = false;
  rele_state = 0x0FU;
  v3_active_config.relayMask = 0x0FU;
  v3_staging_config.relayMask = 0x0FU;
  I2CSTPSetup.relayMask = 0x0FU;
  stop_calls = 0;
  v3_status_snapshot = {};
  v3_status_snapshot.commandSeq = 41;
  v3_status_snapshot.commandResult = I2CSTEPPER_V3_RESULT_SUCCESS;
  for (uint8_t index = 0; index < 4; index++) relay_levels[index] = 1;
}

int main() {
  reset_state();
  v3_stop_remote_for_timeout();
  assert(stop_calls == 1 && rele_state == 0 && v3_active_config.relayMask == 0 &&
         v3_staging_config.relayMask == 0 && I2CSTPSetup.relayMask == 0);
  assert(v3_status_snapshot.commandResult == I2CSTEPPER_V3_RESULT_SUCCESS);
  assert(v3_status_snapshot.error == I2CSTEPPER_V3_ERR_NONE);
  assert(v3_status_snapshot.stopReason == I2CSTEPPER_V3_STOP_NONE);
  for (uint8_t index = 0; index < 4; index++) assert(relay_levels[index] == 0);

  reset_state();
  stepper_state = true;
  v3_stop_remote_for_timeout();
  assert(v3_status_snapshot.commandResult == I2CSTEPPER_V3_RESULT_FAILED);
  assert(v3_status_snapshot.error == I2CSTEPPER_V3_ERR_HEARTBEAT_TIMEOUT);
  assert(v3_status_snapshot.stopReason == I2CSTEPPER_V3_STOP_HEARTBEAT);
  assert(v3_status_snapshot.ackSeq == 41);

  reset_state();
  stepper.running = true;
  v3_stop_remote_for_timeout();
  assert(v3_status_snapshot.error == I2CSTEPPER_V3_ERR_HEARTBEAT_TIMEOUT);

  reset_state();
  pause_phase = true;
  v3_stop_remote_for_timeout();
  assert(v3_status_snapshot.error == I2CSTEPPER_V3_ERR_HEARTBEAT_TIMEOUT);

  reset_state();
  calibration_active = true;
  v3_stop_remote_for_timeout();
  assert(v3_status_snapshot.error == I2CSTEPPER_V3_ERR_HEARTBEAT_TIMEOUT);
  return 0;
}
'''.replace("@SET_RESULT@", result_definition).replace("@TIMEOUT@", timeout_definition)
    assert compile_and_run(timeout_harness, protocol, "timeout").returncode == 0
    timeout_mutation = timeout_definition.replace(
        "stepper_state || stepper.getState(),", "false,", 1)
    assert timeout_mutation != timeout_definition, "active timeout predicate mutation anchor is missing"
    require_mutation_fails(
        timeout_harness.replace(timeout_definition, timeout_mutation), protocol, "timeout_mutated")

    relay_body = function_body("static bool v3_apply_staged_relay()")
    relay_definition = "static bool v3_apply_staged_relay() {" + relay_body + "}"
    claim_body = function_body("static void v3_claim_remote_ownership()")
    claim_definition = "static void v3_claim_remote_ownership() {" + claim_body + "}"
    relay_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>
#include "I2CStepperRuntime.h"

typedef uint8_t byte;
#define LOW 0
#define bit_is_set(value, bit) (((value) & (1U << (bit))) != 0)
struct FakeStepper {
  bool running = false;
  bool getState() const { return running; }
} stepper;
I2CStepperV3Config v3_staging_config = {};
I2CStepperV3Config v3_active_config = {};
I2CStepperV3StatusSnapshot v3_status_snapshot = {};
struct { uint8_t relayMask; } I2CSTPSetup = {};
bool v3_remote_owner = false;
uint32_t v3_last_heartbeat_ms = 0;
bool stepper_state = false;
bool pause_phase = false;
bool calibration_active = false;
uint8_t rele_state = 0;
uint8_t rele_pin[] = {1, 2, 3, 4};
uint8_t relay_levels[4] = {};
uint8_t stop_calls = 0;
uint32_t fake_now = 100;
uint32_t millis() { return fake_now; }
void digitalWrite(uint8_t pin, uint8_t level) { relay_levels[pin - 1] = level; }
void stop_motion(bool) { stop_calls++; }

@CLAIM@
@SET_RESULT@
@RELAY@
@TIMEOUT@

int main() {
  v3_staging_config.relayMask = 0x0FU;
  v3_status_snapshot.commandSeq = 9U;
  assert(v3_apply_staged_relay());
  assert(v3_remote_owner && v3_last_heartbeat_ms == 100U && rele_state == 0x0FU &&
         v3_active_config.relayMask == 0x0FU && I2CSTPSetup.relayMask == 0x0FU);
  for (uint8_t index = 0; index < 4; index++) assert(relay_levels[index] == 1U);

  fake_now = 10101U;
  if (i2cstepper_v3_heartbeat_expired(v3_remote_owner, fake_now, v3_last_heartbeat_ms)) {
    v3_stop_remote_for_timeout();
    v3_remote_owner = false;
  }
  assert(!v3_remote_owner && stop_calls == 1U && rele_state == 0U &&
         v3_active_config.relayMask == 0U && v3_staging_config.relayMask == 0U &&
         I2CSTPSetup.relayMask == 0U);
  for (uint8_t index = 0; index < 4; index++) assert(relay_levels[index] == 0U);
  return 0;
}
'''.replace("@CLAIM@", claim_definition)
    relay_harness = relay_harness.replace("@SET_RESULT@", result_definition)
    relay_harness = relay_harness.replace("@RELAY@", relay_definition)
    relay_harness = relay_harness.replace("@TIMEOUT@", timeout_definition)
    assert compile_and_run(relay_harness, protocol, "relay_timeout").returncode == 0
    relay_mutation = relay_definition.replace("v3_claim_remote_ownership();",
                                              "if (false) v3_claim_remote_ownership();", 1)
    assert relay_mutation != relay_definition, "relay ownership mutation anchor is missing"
    require_mutation_fails(
        relay_harness.replace(relay_definition, relay_mutation), protocol, "relay_timeout_mutated")

    runtime_body = function_body("void update_runtime_state()")
    runtime_definition = "void update_runtime_state() {" + runtime_body + "}"
    runtime_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>
#include "I2CStepperRuntime.h"

typedef uint8_t byte;
#define I2CMIXER 1U
#define I2CPUMP 2U
#define I2CFILLING 3U
#define I2CSTEPPER_SENSOR_STOP 0x01U
#define I2CSTEPPER_SENSOR_PUMP_PAUSE 0x04U
struct FakeStepper {
  bool running = false;
  uint8_t disables = 0;
  bool getState() const { return running; }
  void brake() { running = false; }
  void disable() { disables++; }
} stepper;
struct Setup { uint8_t sensorFlags; uint8_t mode; uint32_t mixerRunSec; uint32_t mixerPauseSec; uint32_t pumpPauseSec; } I2CSTPSetup = {};
bool v3_mixer_deadline_active = false;
uint32_t v3_mixer_deadline_ms = 0;
bool stepper_state = false;
bool pause_phase = false;
bool calibration_active = false;
uint32_t pause_deadline_ms = 0;
bool sensor_active = false;
uint8_t stop_calls = 0;
I2CStepperV3StatusSnapshot v3_status_snapshot = {};
uint32_t millis() { return 100U; }
bool external_sensor_active() { return sensor_active; }
void finish_mixer_run_phase() {}
void stop_motion(bool) { stop_calls++; stepper_state = false; stepper.running = false; }
void pause_stepper_timer() {}
void timer1_disarm() {}
void start_current_mode() {}

@RUNTIME@

int main() {
  sensor_active = true;
  stepper_state = true;
  stepper.running = true;
  I2CSTPSetup.sensorFlags = I2CSTEPPER_SENSOR_STOP;
  update_runtime_state();
  assert(stop_calls == 1U && v3_status_snapshot.stopReason == I2CSTEPPER_V3_STOP_SENSOR);

  stop_calls = 0;
  stepper_state = true;
  pause_phase = true;
  calibration_active = false;
  update_runtime_state();
  assert(stop_calls == 1U && v3_status_snapshot.stopReason == I2CSTEPPER_V3_STOP_SENSOR);

  stop_calls = 0;
  stepper_state = false;
  pause_phase = false;
  calibration_active = true;
  update_runtime_state();
  assert(stop_calls == 1U && v3_status_snapshot.stopReason == I2CSTEPPER_V3_STOP_SENSOR);

  stop_calls = 0;
  stepper_state = false;
  pause_phase = false;
  calibration_active = false;
  v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_REMOTE;
  update_runtime_state();
  assert(stop_calls == 0U && v3_status_snapshot.stopReason == I2CSTEPPER_V3_STOP_REMOTE);
  v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_LOCAL;
  update_runtime_state();
  assert(stop_calls == 0U && v3_status_snapshot.stopReason == I2CSTEPPER_V3_STOP_LOCAL);

  sensor_active = false;
  stepper_state = true;
  stepper.running = false;
  I2CSTPSetup = {};
  I2CSTPSetup.mode = I2CPUMP;
  v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_NONE;
  update_runtime_state();
  assert(!stepper_state && stepper.disables == 0U &&
         v3_status_snapshot.stopReason == I2CSTEPPER_V3_STOP_COMPLETE);

  stepper_state = true;
  stepper.running = false;
  I2CSTPSetup.mode = I2CFILLING;
  v3_status_snapshot.stopReason = I2CSTEPPER_V3_STOP_NONE;
  update_runtime_state();
  assert(!stepper_state && stepper.disables == 1U &&
         v3_status_snapshot.stopReason == I2CSTEPPER_V3_STOP_COMPLETE);
  return 0;
}
'''.replace("@RUNTIME@", runtime_definition)
    assert compile_and_run(runtime_harness, protocol, "stop_reasons").returncode == 0
    sensor_mutation = runtime_definition.replace("I2CSTEPPER_V3_STOP_SENSOR",
                                                 "I2CSTEPPER_V3_STOP_COMPLETE", 1)
    assert sensor_mutation != runtime_definition, "sensor stop mutation anchor is missing"
    require_mutation_fails(
        runtime_harness.replace(runtime_definition, sensor_mutation), protocol, "stop_reasons_mutated")
    sensor_guard_mutation = runtime_definition.replace(
        "(stepper_state || pause_phase || calibration_active)", "false", 1)
    assert sensor_guard_mutation != runtime_definition, "sensor transition mutation anchor is missing"
    require_mutation_fails(
        runtime_harness.replace(runtime_definition, sensor_guard_mutation), protocol,
        "stop_transition_mutated")

    local_stop_body = function_body("void v3_local_stop()")
    local_stop_definition = "void v3_local_stop() {" + local_stop_body + "}"
    local_stop_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>

I2CStepperV3StatusSnapshot v3_status_snapshot = {};
uint8_t stop_calls = 0;
void stop_stepper() { stop_calls++; }

@LOCAL_STOP@

int main() {
  v3_local_stop();
  assert(stop_calls == 1U && v3_status_snapshot.stopReason == I2CSTEPPER_V3_STOP_LOCAL &&
         v3_status_snapshot.stopEventSeq == 1U);
  v3_local_stop();
  assert(stop_calls == 2U && v3_status_snapshot.stopEventSeq == 2U);
  return 0;
}
'''.replace("@LOCAL_STOP@", local_stop_definition)
    assert compile_and_run(local_stop_harness, protocol, "local_stop").returncode == 0
    local_stop_mutation = local_stop_definition.replace(
        "i2cstepper_v3_sequence_next(v3_status_snapshot.stopEventSeq)", "v3_status_snapshot.stopEventSeq", 1)
    assert local_stop_mutation != local_stop_definition, "local stop mutation anchor is missing"
    require_mutation_fails(
        local_stop_harness.replace(local_stop_definition, local_stop_mutation), protocol,
        "local_stop_mutated")

    finish_body = function_body("static void finish_mixer_run_phase()")
    finish_definition = "static void finish_mixer_run_phase() {" + finish_body + "}"
    current_mode_body = function_body("void start_current_mode()")
    current_mode_definition = "void start_current_mode() {" + current_mode_body + "}"
    direction_body = function_body("void set_motion_direction(byte dir)")
    direction_definition = "void set_motion_direction(byte dir) {" + direction_body + "}"
    direction_get_body = function_body("byte get_motion_direction(void)")
    direction_get_definition = "byte get_motion_direction(void) {" + direction_get_body + "}"
    reverse_runtime_body = function_body("void update_runtime_state()")
    reverse_runtime_definition = "void update_runtime_state() {" + reverse_runtime_body + "}"
    reverse_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>
#include "I2CStepperRuntime.h"
#include "StepperMath.h"

typedef uint8_t byte;
#define I2CMIXER 1U
#define I2CPUMP 2U
#define I2CFILLING 3U
#define STEPPER_STEPS 400U
#define STEPPER_TARGET_LIMIT 2147483647UL
#define I2CSTEPPER_FLAG_REVERSE_AFTER_PAUSE 0x01U
#define I2CSTEPPER_FLAG_DIRECTION 0x04U
#define I2CSTEPPER_SENSOR_STOP 0x02U
#define I2CSTEPPER_SENSOR_PUMP_PAUSE 0x04U
struct FakeStepper {
  bool running = false;
  int32_t current = 0;
  int32_t target = 0;
  void brake() { running = false; }
  void disable() {}
  void setTarget(int32_t value) { target = value; }
  int32_t getCurrent() const { return current; }
  bool getState() const { return running; }
} stepper;
struct Setup {
  uint8_t optionFlags;
  uint8_t sensorFlags;
  uint8_t mode;
  uint32_t mixerRpm;
  uint32_t mixerRunSec;
  uint32_t mixerPauseSec;
  uint32_t pumpMlHour;
  uint32_t pumpPauseSec;
  uint32_t fillingMl;
  uint32_t fillingMlHour;
  uint32_t stepperStepMl;
} I2CSTPSetup = {};
I2CStepperV3Config v3_active_config = {};
I2CStepperV3Config v3_staging_config = {};
I2CStepperV3Motion v3_staging_motion = {};
I2CStepperV3StatusSnapshot v3_status_snapshot = {};
bool v3_movement_allowed = true;
bool stepper_state = false;
bool pause_phase = false;
bool calibration_active = false;
bool v3_motion_continuous = false;
bool v3_mixer_deadline_active = false;
uint32_t v3_mixer_deadline_ms = 0;
uint32_t pause_deadline_ms = 0;
uint16_t curr_spd = 0;
uint32_t set_spd = 0;
uint32_t set_time = 0;
byte set_dir = 0;
uint32_t fake_now = 1000;
uint8_t started_direction = 0;
uint8_t start_calls = 0;
uint32_t millis() { return fake_now; }
bool external_sensor_active() { return false; }
void pause_stepper_timer() {}
void timer1_disarm() {}
void stop_motion(bool) {}
void start_motion(uint16_t, uint32_t, byte direction, bool) {
  started_direction = direction;
  start_calls++;
  stepper_state = true;
  stepper.running = true;
}

@SET_DIRECTION@
@GET_DIRECTION@
@FINISH@
@CURRENT_MODE@
@RUNTIME@

static void run_cycle(byte initial_direction) {
  I2CSTPSetup = {};
  I2CSTPSetup.mode = I2CMIXER;
  I2CSTPSetup.mixerRpm = 60U;
  I2CSTPSetup.mixerRunSec = 2U;
  I2CSTPSetup.mixerPauseSec = 3U;
  I2CSTPSetup.optionFlags = I2CSTEPPER_FLAG_REVERSE_AFTER_PAUSE |
                            (initial_direction ? I2CSTEPPER_FLAG_DIRECTION : 0U);
  v3_active_config = {};
  v3_staging_config = {};
  v3_staging_motion = {};
  v3_status_snapshot = {};
  stepper = {};
  stepper_state = false;
  pause_phase = false;
  calibration_active = false;
  v3_mixer_deadline_active = false;
  start_calls = 0;
  fake_now = 1000U;
  start_current_mode();
  assert(start_calls == 1U && started_direction == initial_direction &&
         v3_mixer_deadline_active && v3_mixer_deadline_ms == 3000U);
  fake_now = 3000U;
  update_runtime_state();
  assert(pause_phase && stepper_state && !stepper.running &&
         get_motion_direction() == (initial_direction ? 0U : 1U) && pause_deadline_ms == 6000U);
  fake_now = 6000U;
  update_runtime_state();
  assert(!pause_phase && start_calls == 2U &&
         started_direction == (initial_direction ? 0U : 1U));
}

int main() {
  run_cycle(0U);
  run_cycle(1U);
  return 0;
}
'''.replace("@SET_DIRECTION@", direction_definition).replace(
        "@GET_DIRECTION@", direction_get_definition).replace("@FINISH@", finish_definition).replace(
        "@CURRENT_MODE@", current_mode_definition).replace("@RUNTIME@", reverse_runtime_definition)
    assert compile_and_run(reverse_harness, protocol, "reverse_pause").returncode == 0
    reverse_mutation = finish_definition.replace(
        "set_motion_direction(get_motion_direction() ? 0 : 1);",
        "set_motion_direction(get_motion_direction());", 1)
    assert reverse_mutation != finish_definition, "reverse pause mutation anchor is missing"
    require_mutation_fails(
        reverse_harness.replace(finish_definition, reverse_mutation), protocol,
        "reverse_pause_mutated")

    type_body_start = MENU_SOURCE.index("void change_type() {")
    type_open = MENU_SOURCE.index("{", type_body_start)
    depth = 0
    type_body = None
    for index in range(type_open, len(MENU_SOURCE)):
        if MENU_SOURCE[index] == "{":
            depth += 1
        elif MENU_SOURCE[index] == "}":
            depth -= 1
            if depth == 0:
                type_body = MENU_SOURCE[type_open + 1:index]
                break
    assert type_body is not None, "unterminated change_type"
    type_definition = "void change_type() {" + type_body + "}"
    type_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>

typedef uint8_t byte;
#define I2CMIXER 1U
#define I2CPUMP 2U
#define I2CFILLING 3U
struct Setup { byte mode; byte role; } I2CSTPSetup = {};
I2CStepperV3Config v3_staging_config = {};
bool setup_dirty = false;
bool v3_local_controls_locked() { return false; }

@CHANGE_TYPE@

int main() {
  v3_staging_config.address = 2U;
  I2CSTPSetup.mode = I2CPUMP;
  change_type();  // increase action
  assert(I2CSTPSetup.mode == I2CFILLING && I2CSTPSetup.role == I2CPUMP && setup_dirty);
  setup_dirty = false;
  change_type();  // decrease action uses the same callback and must toggle back
  assert(I2CSTPSetup.mode == I2CPUMP && I2CSTPSetup.role == I2CPUMP && setup_dirty);
  v3_staging_config.address = 1U;
  I2CSTPSetup.mode = I2CMIXER;
  I2CSTPSetup.role = I2CMIXER;
  setup_dirty = false;
  change_type();
  assert(I2CSTPSetup.mode == I2CMIXER && I2CSTPSetup.role == I2CMIXER && !setup_dirty);
  return 0;
}
'''.replace("@CHANGE_TYPE@", type_definition)
    assert compile_and_run(type_harness, protocol, "menu_type").returncode == 0
    type_mutation = type_definition.replace(
        "(I2CSTPSetup.mode == I2CPUMP ? I2CFILLING : I2CPUMP)", "I2CPUMP", 1)
    assert type_mutation != type_definition, "menu type mutation anchor is missing"
    require_mutation_fails(
        type_harness.replace(type_definition, type_mutation), protocol, "menu_type_mutated")

    receive_body = function_body("void v3_wire_receive(int count)")
    receive_definition = "void v3_wire_receive(int count) {" + receive_body + "}"
    mailbox_harness = r'''
#include <assert.h>
#include <cstddef>
#include <stdint.h>
#include <vector>
#include <I2CStepperV3.h>

uint8_t v3_rx_config_a[I2CSTEPPER_V3_CONFIG_A_SIZE] = {};
uint8_t v3_rx_config_b[I2CSTEPPER_V3_CONFIG_B_SIZE] = {};
uint8_t v3_rx_motion[I2CSTEPPER_V3_MOTION_SIZE] = {};
uint8_t v3_rx_command[I2CSTEPPER_V3_COMMAND_SIZE] = {};
volatile bool v3_rx_config_a_pending = false;
volatile bool v3_rx_config_b_pending = false;
volatile bool v3_rx_motion_pending = false;
volatile bool v3_rx_command_pending = false;
uint8_t v3_read_register = 0;

struct FakeWire {
  std::vector<uint8_t> input;
  size_t offset = 0;
  int available() const { return offset < input.size() ? int(input.size() - offset) : 0; }
  int read() { return available() ? input[offset++] : -1; }
} Wire;

@RECEIVE@

static void set_input(const uint8_t* data, size_t size) {
  Wire.input.assign(data, data + size);
  Wire.offset = 0;
}

int main() {
  uint8_t configA[I2CSTEPPER_V3_CONFIG_A_SIZE + 1] = {I2CSTEPPER_V3_REG_CONFIG_A};
  uint8_t configB[I2CSTEPPER_V3_CONFIG_B_SIZE + 1] = {I2CSTEPPER_V3_REG_CONFIG_B};
  uint8_t motion[I2CSTEPPER_V3_MOTION_SIZE + 1] = {I2CSTEPPER_V3_REG_MOTION};
  uint8_t command[I2CSTEPPER_V3_COMMAND_SIZE + 1] = {I2CSTEPPER_V3_REG_COMMAND};
  for (uint8_t index = 1; index < sizeof(configA); index++) configA[index] = index;
  for (uint8_t index = 1; index < sizeof(configB); index++) configB[index] = index + 20U;
  for (uint8_t index = 1; index < sizeof(motion); index++) motion[index] = index + 40U;
  for (uint8_t index = 1; index < sizeof(command); index++) command[index] = index + 50U;

  // Samovar sends CONFIG_A, CONFIG_B, MOTION, COMMAND as a normal burst.
  set_input(configA, sizeof(configA)); v3_wire_receive(sizeof(configA));
  set_input(configB, sizeof(configB)); v3_wire_receive(sizeof(configB));
  set_input(motion, sizeof(motion)); v3_wire_receive(sizeof(motion));
  set_input(command, sizeof(command)); v3_wire_receive(sizeof(command));
  assert(v3_rx_config_a_pending && v3_rx_config_b_pending &&
         v3_rx_motion_pending && v3_rx_command_pending);
  for (uint8_t index = 0; index < I2CSTEPPER_V3_CONFIG_A_SIZE; index++) assert(v3_rx_config_a[index] == configA[index + 1]);
  for (uint8_t index = 0; index < I2CSTEPPER_V3_CONFIG_B_SIZE; index++) assert(v3_rx_config_b[index] == configB[index + 1]);
  for (uint8_t index = 0; index < I2CSTEPPER_V3_MOTION_SIZE; index++) assert(v3_rx_motion[index] == motion[index + 1]);
  for (uint8_t index = 0; index < I2CSTEPPER_V3_COMMAND_SIZE; index++) assert(v3_rx_command[index] == command[index + 1]);

  uint8_t overwritten[I2CSTEPPER_V3_CONFIG_A_SIZE + 1] = {I2CSTEPPER_V3_REG_CONFIG_A};
  for (uint8_t index = 1; index < sizeof(overwritten); index++) overwritten[index] = 0xEEU;
  set_input(overwritten, sizeof(overwritten));
  v3_wire_receive(sizeof(overwritten));
  assert(Wire.available() == 0 && v3_rx_config_a_pending);
  for (uint8_t index = 0; index < I2CSTEPPER_V3_CONFIG_A_SIZE; index++) assert(v3_rx_config_a[index] == configA[index + 1]);

  const uint8_t pointer[] = {I2CSTEPPER_V3_REG_STATUS};
  set_input(pointer, sizeof(pointer));
  v3_wire_receive(sizeof(pointer));
  assert(Wire.available() == 0 && v3_read_register == I2CSTEPPER_V3_REG_STATUS);
  assert(v3_rx_config_a_pending && v3_rx_config_b_pending &&
         v3_rx_motion_pending && v3_rx_command_pending);

  const uint8_t malformed[] = {1, 2, 3};
  set_input(malformed, sizeof(malformed));
  v3_wire_receive(I2CSTEPPER_V3_WIRE_BUFFER_SIZE + 1);
  assert(Wire.available() == 0 && v3_rx_config_a_pending && v3_rx_config_b_pending &&
         v3_rx_motion_pending && v3_rx_command_pending);
  return 0;
}
'''.replace("@RECEIVE@", receive_definition)
    assert compile_and_run(mailbox_harness, protocol, "mailbox").returncode == 0
    mailbox_mutation = receive_definition.replace("|| *pending)", "|| false)", 1)
    assert mailbox_mutation != receive_definition, "mailbox preservation mutation anchor is missing"
    require_mutation_fails(
        mailbox_harness.replace(receive_definition, mailbox_mutation), protocol, "mailbox_mutated")

    motion_body = function_body("static void start_motion(uint16_t spd, uint32_t target, byte dir, bool continuous)")
    motion_definition = "static void start_motion(uint16_t spd, uint32_t target, byte dir, bool continuous) {" + motion_body + "}"
    motion_harness = r'''
#include <assert.h>
#include <stdint.h>

#include <I2CStepperV3.h>

typedef uint8_t byte;
#define STEPPER_TARGET_LIMIT 2147483647UL
#define I2CSTEPPER_FLAG_SMOOTH_START 0x02U
struct FakeStepper {
  uint8_t reverseValue = 0;
  uint16_t acceleration = 0;
  uint16_t maxSpeed = 0;
  int32_t speed = 0;
  int32_t target = 0;
  void brake() {}
  void enable() {}
  void reverse(byte value) { reverseValue = value; }
  void setCurrent(int32_t) {}
  void setAcceleration(uint16_t value) { acceleration = value; }
  void setSpeed(int32_t value) { speed = value; }
  void setMaxSpeed(uint16_t value) { maxSpeed = value; }
  void setTarget(int32_t value) { target = value; }
  uint32_t getPeriod() const { return 1000U; }
} stepper;
struct { uint8_t optionFlags; } I2CSTPSetup = {};
bool v3_movement_allowed = true;
bool stepper_state = false;
bool pause_phase = false;
bool v3_motion_continuous = false;
bool v3_mixer_deadline_active = false;
uint8_t v3_runtime_mode = I2CSTEPPER_V3_MODE_FILLING;
I2CStepperV3Motion v3_staging_motion = {};
I2CStepperV3StatusSnapshot v3_status_snapshot = {};
uint16_t curr_spd = 0;
byte set_dir = 0;
byte last_dir = 0;
uint16_t stored_speed = 0;
uint32_t stored_target = 0;
uint32_t scheduled_period = 0;
void pause_stepper_timer() {}
uint16_t stepper_acceleration_from_speed(uint16_t speed) { return speed / 10U ? speed / 10U : 1U; }
void set_motion_speed(uint16_t speed) { stored_speed = speed; }
void set_motion_target(uint32_t target) { stored_target = target; }
void timer1_schedule(uint32_t period) { scheduled_period = period; }

@START_MOTION@

int main() {
  start_motion(1000U, 123456U, 1U, false);
  assert(stepper.maxSpeed == 1000U && stepper.target == 123456 && stepper.speed == 0);
  assert(stepper.reverseValue == 1U && !v3_motion_continuous);
  assert(stored_target == 123456U && stored_speed == 1000U && scheduled_period == 1000U);
  assert(v3_staging_motion.mode == I2CSTEPPER_V3_MODE_FILLING &&
         v3_staging_motion.direction == 1U &&
         v3_staging_motion.speedStepsPerSec == 1000U &&
         v3_staging_motion.targetSteps == 123456U);

  stepper = {};
  start_motion(2000U, 0U, 0U, true);
  assert(stepper.speed == 2000 && stepper.maxSpeed == 0U && stepper.target == 0);
  assert(v3_motion_continuous && stored_target == 0U && stored_speed == 2000U);
  assert(v3_staging_motion.speedStepsPerSec == 2000U && v3_staging_motion.targetSteps == 0U);

  const uint32_t nine_hours_at_max_speed = 18000UL * 9UL * 60UL * 60UL;
  assert(nine_hours_at_max_speed == 583200000UL);
  stepper = {};
  start_motion(18000U, 0U, 0U, true);
  assert(stepper.target == 0 && stepper.speed == 18000 && v3_motion_continuous);

  v3_status_snapshot.error = I2CSTEPPER_V3_ERR_NONE;
  start_motion(0U, 1U, 0U, false);
  assert(v3_status_snapshot.error == I2CSTEPPER_V3_ERR_BAD_CONFIG);
  v3_status_snapshot.error = I2CSTEPPER_V3_ERR_NONE;
  start_motion(10U, 0U, 0U, false);
  assert(v3_status_snapshot.error == I2CSTEPPER_V3_ERR_BAD_CONFIG);
  return 0;
}
'''.replace("@START_MOTION@", motion_definition)
    assert compile_and_run(motion_harness, protocol, "motion").returncode == 0
    motion_mutation = motion_definition.replace("if (continuous) {", "if (false) {", 1)
    assert motion_mutation != motion_definition, "continuous motion mutation anchor is missing"
    require_mutation_fails(
        motion_harness.replace(motion_definition, motion_mutation), protocol, "motion_mutated")

    stop_body = function_body("static void stop_motion(bool smooth)")
    stop_definition = "static void stop_motion(bool smooth) {" + stop_body + "}"
    stop_harness = r'''
#include <assert.h>
#include <stdint.h>

struct FakeStepper {
  bool running = false;
  uint8_t brakes = 0;
  uint8_t disables = 0;
  uint8_t stops = 0;
  int32_t current = 9;
  int32_t target = 17;
  void brake() { brakes++; running = false; }
  void disable() { disables++; }
  void stop() { stops++; }
  void setCurrent(int32_t value) { current = value; }
  void setTarget(int32_t value) { target = value; }
  bool getState() const { return running; }
  uint32_t getPeriod() const { return 1000U; }
} stepper;
bool v3_motion_continuous = false;
bool stepper_state = true;
bool pause_phase = true;
bool calibration_active = true;
bool v3_mixer_deadline_active = true;
uint16_t curr_spd = 123;
uint32_t stored_target = 1;
uint8_t timer_disarmed = 0;
uint32_t scheduled_period = 0;
void set_motion_target(uint32_t value) { stored_target = value; }
void pause_stepper_timer() {}
void timer1_schedule(uint32_t period) { scheduled_period = period; }
void timer1_disarm() { timer_disarmed++; }

@STOP_MOTION@

int main() {
  stop_motion(false);
  assert(stepper.brakes == 1U && stepper.disables == 1U && stepper.current == 0 && stepper.target == 0);
  assert(timer_disarmed == 1U && stored_target == 0U && !stepper_state && !pause_phase);

  stepper = {};
  stepper.running = true;
  v3_motion_continuous = false;
  stepper_state = true;
  pause_phase = true;
  calibration_active = true;
  v3_mixer_deadline_active = true;
  timer_disarmed = 0;
  scheduled_period = 0;
  stop_motion(true);
  assert(stepper.stops == 1U && stepper.brakes == 0U && stepper.disables == 0U);
  assert(scheduled_period == 1000U && timer_disarmed == 0U && !stepper_state && !pause_phase);
  return 0;
}
'''.replace("@STOP_MOTION@", stop_definition)
    assert compile_and_run(stop_harness, protocol, "stop").returncode == 0
    stop_mutation = stop_definition.replace(
        "if (smooth && !v3_motion_continuous && stepper.getState())",
        "if (smooth && false && !v3_motion_continuous && stepper.getState())", 1)
    assert stop_mutation != stop_definition, "smooth stop mutation anchor is missing"
    require_mutation_fails(
        stop_harness.replace(stop_definition, stop_mutation), protocol, "stop_mutated")

    direction_body = function_body("void set_motion_direction(byte dir)")
    direction_definition = "void set_motion_direction(byte dir) {" + direction_body + "}"
    direction_harness = r'''
#include <assert.h>
#include <stdint.h>
#include <I2CStepperV3.h>

typedef uint8_t byte;
#define I2CSTEPPER_FLAG_DIRECTION 0x04U
struct Setup { uint8_t optionFlags; } I2CSTPSetup = {0x03U};
I2CStepperV3Config v3_active_config = {};
I2CStepperV3Config v3_staging_config = {};
I2CStepperV3Motion v3_staging_motion = {};

@SET_DIRECTION@

int main() {
  set_motion_direction(1U);
  assert(I2CSTPSetup.optionFlags == 0x07U && v3_active_config.optionFlags == 0x07U &&
         v3_staging_config.optionFlags == 0x07U && v3_staging_motion.direction == 1U);
  set_motion_direction(0U);
  assert(I2CSTPSetup.optionFlags == 0x03U && v3_active_config.optionFlags == 0x03U &&
         v3_staging_config.optionFlags == 0x03U && v3_staging_motion.direction == 0U);
  return 0;
}
'''.replace("@SET_DIRECTION@", direction_definition)
    assert compile_and_run(direction_harness, protocol, "direction").returncode == 0
    direction_mutation = direction_definition.replace(
        "I2CSTPSetup.optionFlags & (uint8_t)~I2CSTEPPER_FLAG_DIRECTION", "0", 1)
    assert direction_mutation != direction_definition, "direction bit mutation anchor is missing"
    require_mutation_fails(
        direction_harness.replace(direction_definition, direction_mutation), protocol, "direction_mutated")


def main():
    assert "iarduino_I2C_connect" not in SOURCE
    assert "I2C2." not in SOURCE
    assert "Wire.end()" not in SOURCE
    assert "TimerOne" not in SOURCE
    assert "GyverStepper2.h" in (ROOT / "I2CStepper.h").read_text()

    timer_schedule = function_body("static void timer1_schedule(uint32_t periodUs)")
    require(timer_schedule, "i2cstepper_v3_timer1_plan_16mhz")
    require(timer_schedule, "OCR1A = plan.ocr1a")
    require(timer_schedule, "_BV(WGM12)")
    timer_isr = function_body("ISR(TIMER1_COMPA_vect)")
    require(timer_isr, "stepper.tickManual()")
    require(timer_isr, "stepper.getPeriod()")
    require(timer_isr, "timer1_disarm()")

    receive = function_body("void v3_wire_receive(int count)")
    request = function_body("void v3_wire_request()")
    for forbidden in ("EEPROM", "digitalWrite", "stepper.", "stop_stepper", "write_config"):
        assert forbidden not in receive
        assert forbidden not in request
    for mailbox in ("v3_rx_config_a", "v3_rx_config_b", "v3_rx_motion", "v3_rx_command"):
        require(receive, mailbox)
    require(receive, "while (Wire.available()) Wire.read()")
    require(receive, "if (count == 1)")
    require(receive, "mailbox[index] = (uint8_t)Wire.read()")
    require(receive, "*pending = true")
    assert "uint8_t received[" not in receive
    assert receive.rfind("*pending = true") > receive.rfind("mailbox[index] = (uint8_t)Wire.read()")
    require(request, "Wire.write(v3_status_frame, I2CSTEPPER_V3_STATUS_SIZE)")

    loop = function_body("void loop()")
    assert loop.index("v3_process_receive()") < loop.index("update_runtime_state()")
    assert "handle_command()" not in loop
    assert "sync_relays_from_array()" not in loop
    require(loop, "v3_stop_remote_for_timeout()")

    process = function_body("void v3_process_receive()")
    assert_mailbox_order(process)
    order_mutation = process.replace("v3_rx_config_a", "v3_rx_command", 1)
    try:
        assert_mailbox_order(order_mutation)
        raise AssertionError("mailbox order mutation survived")
    except AssertionError as error:
        assert str(error) != "mailbox order mutation survived"

    samovar_master = (ROOT.parent / "Samovar" / "I2CStepper.h").read_text()
    config_begin = samovar_master.index("inline bool i2c_stepper_write_config")
    config_end = samovar_master.index("inline bool i2c_stepper_write_motion", config_begin)
    config_sender = samovar_master[config_begin:config_end]
    assert config_sender.index("I2CSTEPPER_V3_REG_CONFIG_A") < \
           config_sender.index("I2CSTEPPER_V3_REG_CONFIG_B")
    command_begin = samovar_master.index("bool i2c_stepper_send_command")
    command_end = samovar_master.index("// Heartbeat is deliberately", command_begin)
    command_sender = samovar_master[command_begin:command_end]
    assert command_sender.index("i2c_stepper_write_block") < command_sender.index("vTaskDelay(10")
    assert command_sender.index("vTaskDelay(10") < command_sender.index("ackSeq == frame.commandSeq")

    timeout = function_body("static void v3_stop_remote_for_timeout()")
    require(timeout, "I2CSTEPPER_V3_ERR_HEARTBEAT_TIMEOUT")
    require(timeout, "I2CSTEPPER_V3_STOP_HEARTBEAT")
    require(timeout, "digitalWrite(rele_pin[i], LOW)")
    require(timeout, "v3_active_config.relayMask = 0")
    require(timeout, "v3_staging_config.relayMask = 0")
    require(timeout, "i2cstepper_v3_heartbeat_timeout_action")

    for signature in (
        "void start_stepper(bool from_int)",
        "static void start_motion(uint16_t spd, uint32_t target, byte dir, bool continuous)",
        "void start_current_mode()",
        "void start_calibration()",
    ):
        require(function_body(signature), "v3_movement_allowed")

    read_config = function_body("void read_config()")
    require(read_config, "v3_eeprom_read(&v3_active_config)")
    require(read_config, "i2cstepper_v3_migrate_v2_config")
    require(read_config, "I2CSTEPPER_V3_BOOT_MIGRATE_V1")
    require(read_config, "I2CSTEPPER_V3_BOOT_DEFAULTS")
    require(read_config, "i2cstepper_v3_boot_action")
    require(read_config, "I2CSTEPPER_V3_BOOT_MIGRATE_V2")
    require(read_config, "I2CSTEPPER_V3_ERR_EEPROM_INVALID")
    require(read_config, "Wire.onReceive(v3_wire_receive)")
    require(read_config, "Wire.onRequest(v3_wire_request)")
    assert read_config.index("v3_runtime_address = v3_active_config.address") < read_config.index("Wire.begin(v3_runtime_address)")

    command = function_body("void v3_process_receive()")
    require(command, "I2CSTEPPER_V3_ERR_BAD_SEQUENCE")
    require(command, "I2CSTEPPER_V3_CMD_START_FINITE")
    require(command, "I2CSTEPPER_V3_CMD_START_CONFIGURED")
    require(command, "v3_start_precondition_error()")
    require(command, "v3_apply_address_error()")
    require(command, "v3_start_configured_motion()")
    require(command, "i2cstepper_v3_start_mode_supported")
    require(command, "I2CSTEPPER_V3_CMD_CALIBRATE_FINISH")
    require(command, "I2CSTEPPER_V3_CMD_HEARTBEAT")
    require(command, "v3_eeprom_write(prepared)")
    require(command, "v3_prepare_staged_config")
    require(command, "i2cstepper_v3_runtime_address_matches(v3_runtime_address, command.address)")
    assert "Wire.begin" not in command
    assert command.index("I2CSTEPPER_V3_RESULT_PENDING") < command.index("v3_publish_frames()")
    calibration_start = command.index("I2CSTEPPER_V3_CMD_CALIBRATE_START")
    calibration_finish = command.index("I2CSTEPPER_V3_CMD_CALIBRATE_FINISH")
    calibration = command[calibration_start:calibration_finish]
    staged_start = command[command.index("I2CSTEPPER_V3_CMD_START_FINITE"):command.index("I2CSTEPPER_V3_CMD_STOP")]
    assert staged_start.index("v3_start_precondition_error()") < staged_start.index("i2cstepper_v3_start_mode_supported")
    assert calibration.index("v3_start_precondition_error()") < calibration.index("i2cstepper_v3_address_is_mixer")
    apply = command[command.index("I2CSTEPPER_V3_CMD_APPLY"):command.index("I2CSTEPPER_V3_CMD_START_CONFIGURED")]
    assert apply.index("v3_apply_address_error()") < apply.index("v3_prepare_staged_config")
    stop = command[command.index("I2CSTEPPER_V3_CMD_STOP"):command.index("I2CSTEPPER_V3_CMD_RELAY")]
    relay = command[command.index("I2CSTEPPER_V3_CMD_RELAY"):calibration_start]
    heartbeat = command[command.index("I2CSTEPPER_V3_CMD_HEARTBEAT"):command.index("I2CSTEPPER_V3_CMD_APPLY")]
    assert "v3_start_precondition_error" not in stop + relay + heartbeat
    assert calibration.index("calibration_active = true") < calibration.index("v3_claim_remote_ownership()")

    publish = function_body("void v3_publish_frames()")
    assert publish.index("lock_interrupts()") < publish.index("i2cstepper_v3_encode_status")
    assert publish.index("i2cstepper_v3_encode_status") < publish.index("unlock_interrupts(sreg)")
    require(publish, "v3_publish_runtime_identity(&identity)")
    runtime_identity = function_body("static void v3_publish_runtime_identity(I2CStepperV3Identity* identity)")
    require(runtime_identity, "identity->address = v3_runtime_address")
    require(runtime_identity, "v3_status_snapshot.address = v3_runtime_address")
    require(runtime_identity, "v3_status_snapshot.mode = v3_runtime_mode")

    eeprom = function_body("static bool v3_eeprom_write(const I2CStepperV3Config& config)")
    require(eeprom, "EEPROM.update")
    require(eeprom, "EEPROM.read(V3_EEPROM_OFFSET + i) != record[i]")

    menu = (ROOT / "I2CMenu.h").read_text()
    assert "LiquidLine setup_line3" in menu
    assert "void change_address(bool increase)" in menu
    assert "i2cstepper_v3_mode_after_address_change" in menu
    assert "Saved. Reboot!" in menu
    assert "saved.address--" not in SOURCE and "saved.address++" not in SOURCE
    poll_menu_start = menu.index("void poll_menu(void) {")
    poll_menu = menu[poll_menu_start:]
    assert poll_menu.count("v3_local_stop();") == 2
    assert "if (get_stepper_state()) v3_local_stop();" in poll_menu
    assert "if (get_stepper_state()) stop_stepper();" not in poll_menu
    assert "setup_line1.attach_function(increase, change_type);" in menu
    assert "setup_line1.attach_function(decrease, change_type);" in menu
    assert 'const char str_Pmp[]  PROGMEM = "Pump:";' in menu
    assert 'const char str_R2[]  PROGMEM = "Rele2:";' in menu
    assert "*end++ = '>';" in menu
    assert 'PSTR("Continuous")' in menu
    assert "void set_menu_editing(bool editing)" in menu
    assert "editing ? glyph::customFocus : glyph::rightFocus" in menu
    assert "bool save_stp_value()" in menu
    assert "&I2CSTPSetup.fillingMlHour" in menu
    assert "&I2CSTPSetup.fillingMl" in menu
    assert "*value = line == 0 ? set_spd : set_time" in menu
    assert "return write_config();" in menu
    assert "main_menu.get_focusedLine() == 2 && I2CSTPSetup.mode == I2CPUMP" in poll_menu
    assert "if (!navigate) main_menu.call_function(increase);" in poll_menu
    assert "if (!navigate) main_menu.call_function(decrease);" in poll_menu

    config_valid = function_body("static bool v3_config_valid(const I2CStepperV3Config& config)")
    require(config_valid, "targetRequired")
    require(config_valid, "i2cstepper_v3_derived_config_valid")
    result = function_body("static void v3_set_result(uint32_t sequence, uint8_t result, uint8_t error)")
    require(result, "i2cstepper_v3_acknowledge_sequence")
    runtime = function_body("void update_runtime_state()")
    require(runtime, "stepper_state && stepper.getState()")
    require(runtime, "pause_phase && stepper_state")
    mixer = function_body("void start_current_mode()")
    require(mixer, "I2CSTPSetup.mixerRunSec == 0")
    require(mixer, "start_motion(spd, 0, dir, true)")
    motion = function_body("static void start_motion(uint16_t spd, uint32_t target, byte dir, bool continuous)")
    require(motion, "v3_staging_motion.mode = v3_runtime_mode")
    require(motion, "v3_staging_motion.targetSteps = continuous ? 0 : target")
    require(motion, "I2CSTEPPER_V3_STOP_NONE")
    runtime = function_body("void update_runtime_state()")
    require(runtime, "I2CSTEPPER_V3_STOP_SENSOR")
    require(runtime, "I2CSTEPPER_V3_STOP_COMPLETE")
    require(runtime, "if (I2CSTPSetup.mode == I2CFILLING) stepper.disable()")
    apply_runtime = function_body("static void v3_apply_to_runtime()")
    require(apply_runtime, "I2CSTPSetup.fillingMlHour")
    require(apply_runtime, "I2CSTPSetup.fillingMl")
    require(apply_runtime, "I2CSTPSetup.pumpMlHour")
    menu = (ROOT / "I2CMenu.h").read_text()
    assert "apply_local_motion_settings();" in menu
    source_derived_production_harnesses()


if __name__ == "__main__":
    main()
