# I2CStepper Hardware Test Checklist

Use this checklist after flashing firmware to validate real hardware behavior with Samovar integration.

## Preconditions

- Arduino Nano + LCD + encoder + stepper driver + 4 relays are wired as documented.
- I2CStepper and Samovar share common GND.
- Correct mode (`Mixer` or `Pump`) is selected and device was rebooted after mode change.
- EEPROM `StepperStepMl` is configured for your pump calibration.

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

### Pump mode
- Speed units: ml/h.
- Target/remaining units: ml.
- Delivered volume is within your allowed tolerance after calibration.
