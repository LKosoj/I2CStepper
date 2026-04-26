# I2CStepper Hardware Test Checklist

Use this checklist after flashing firmware to validate real hardware behavior with Samovar integration.

## Preconditions

- Arduino Nano + LCD + encoder + stepper driver + 4 relays are wired as documented.
- I2CStepper and Samovar share common GND.
- Correct mode (`Mixer`, `Pump`, or `Fill`) is selected and device was rebooted after mode change.
- EEPROM `StepperStepMl` is configured for your pump calibration.
- If firmware was upgraded from protocol v1 to v2, verify mode and `STP/ML` after first boot because EEPROM defaults may be reinitialized.

## Stepper Safety Checks

1. Power on with no active command:
- Stepper is disabled.
- All relays are off.

2. Start from local menu:
- Start command begins movement.
- Stop command halts movement immediately.
- Remaining time/volume decrements consistently.

3. Direction change while running:
- Direction toggles without lockup.
- No uncontrolled speed jump.

4. Speed change while running:
- Speed is updated smoothly.
- Remaining value stays coherent with speed/mode.

## Relay Checks

1. Toggle each relay from menu:
- Only selected relay changes state.
- Displayed state matches physical relay output.

2. Toggle from Samovar over I2C:
- Local display follows remote state changes.

## I2C/Fail-Safe Checks

1. Run stepper, then force I2C lockup (disconnect SDA/SCL or hold bus):
- Controller enters fail-safe.
- Stepper stops.
- Relays go to OFF.

2. Restore bus:
- Device remains in fail-safe latch mode until encoder click confirmation.
- After click confirmation, new commands are accepted.

## Mode-Specific Validation

### Mixer mode
- Speed units: RPM.
- Time units: seconds.
- Target and remaining time match expected runtime.
- If run time and pause time are non-zero, movement enters pause after the run interval.
- If reverse-after-pause is enabled over I2C, direction changes after the pause cycle.

### Pump mode
- Speed units: ml/h.
- Pump keeps running until Stop command.
- Remaining register uses ml units, but Pump mode completion is not based on a requested volume.
- External sensor stop flag stops the pump.
- External sensor pause flag pauses the pump for configured `PumpPauseSec` when stop flag is disabled.
- Delivered volume is within your allowed tolerance after calibration.

### Fill mode
- Speed units: ml/h.
- Target and remaining units: ml.
- Stepper stops after configured fill volume.
- `Fill` uses the pump I2C role/address, not a separate I2C address.

## I2C Protocol v2 Checks

1. Read identity registers:
- `REG_MAGIC` is `0x53`.
- `REG_VERSION` is `2`.
- `REG_CAPS` matches selected role: Mixer role exposes Mixer + Relay + Sensor, Pump role exposes Pump + Fill + Relay + Sensor.

2. Command handshake:
- Write configuration registers and `REG_COMMAND`.
- Increment `REG_COMMAND_SEQ`.
- Device updates `REG_ACK_SEQ` to the same value.
- `REG_COMMAND` returns to `I2CSTEP_CMD_NONE`.
- `REG_ERROR` remains `I2CSTEP_ERR_NONE`.

3. Unsupported mode:
- In Mixer role, request `Fill` mode and send `APPLY`.
- Device rejects it with `I2CSTEP_ERR_UNSUPPORTED_MODE`.
- Existing motion does not start unexpectedly.

4. Relay command:
- Write `REG_RELAY_MASK`, send `I2CSTEP_CMD_RELAY`.
- Physical relays match lower four bits of the mask.

## Calibration Checks

1. Pump calibration start:
- Device is in Pump role.
- Send `I2CSTEP_CMD_CALIBRATE_START`.
- Pump starts and `I2CSTEPPER_STATUS_CALIBRATION` is set.

2. Pump calibration finish:
- Let the pump deliver exactly 100 ml.
- Send `I2CSTEP_CMD_CALIBRATE_FINISH`.
- Pump stops and calibration status clears.
- `REG_STEPS_PER_ML` is updated to measured steps / 100.
- Send `I2CSTEP_CMD_SAVE` if the new value must persist after reboot.
