# I2CStepper Hardware Test Checklist

Use this checklist after flashing firmware to validate real hardware behavior with Samovar integration.

## Preconditions

- Arduino Nano + LCD + encoder + stepper driver + 4 relays are wired as documented.
- I2CStepper and Samovar share common GND.
- Run the I2C, LCD and encoder together for several minutes after flashing: static v3 globals are compile-time limited, but AVR stack use from these libraries is measurable only on hardware.
- Correct mode (`Mixer`, `Pump`, or `Fill`) is selected and device was rebooted after mode change.
- EEPROM `StepperStepMl` is configured for your pump calibration.
- After an upgrade from v2, verify mode and `STP/ML` after the one-time v2-to-v3 migration. A nonblank corrupted EEPROM must show `EEPROM_INVALID` and must not move the motor.

## Stepper Safety Checks

1. Power on with no active command:
- Stepper is disabled.
- All relays are off.

2. Start from local menu:
- Start command begins movement.
- With smooth start disabled, Stop halts movement immediately; with smooth start enabled, a finite move decelerates before it stops.
- Remaining time/volume decrements consistently.

3. Direction change while running:
- Direction toggles without lockup.
- No uncontrolled speed jump.

4. Speed change while running:
- Speed is updated smoothly.
- Remaining value stays coherent with speed/mode.

5. Timer cadence:
- At 1000 and 2000 steps/s, verify stable motion without missed or doubled pulses.
- After a finite move completes or an immediate stop, verify that STEP pulses cease; Timer1 must be idle rather than ticking at a fixed background rate.

## Relay Checks

1. Toggle each relay from menu:
- Only selected relay changes state.
- Displayed state matches physical relay output.

2. Toggle from Samovar over I2C:
- Local display follows remote state changes.

## Mode-Specific Validation

### Mixer mode
- Speed units: RPM.
- Time units: seconds.
- Target and remaining time match expected runtime.
- If run time and pause time are non-zero, movement enters pause after the run interval.
- Run duration includes the acceleration interval. A run duration of zero is continuous until explicit STOP.
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

## I2C Protocol v3 Checks

- Read IDENTITY, STATUS, CONFIG_A, CONFIG_B and MOTION as complete frames; no read response exceeds 32 bytes.
- Write a register pointer plus at most 31 payload bytes. Confirm that an incomplete staging configuration does not change active CONFIG_A/B until `APPLY` or `SAVE`.
- Send CONFIG_A, CONFIG_B, MOTION and COMMAND without an intervening loop iteration. Confirm all four are consumed in that order. A second packet for an already pending register is discarded, preserving the first packet.
- Send heartbeat every 250 ms while the device is present, including idle. Confirm that local controls lock, local STOP still works once, and all remotely controlled relays deenergize after more than 1000 ms without heartbeat. During remote motion, pause or calibration STATUS reports `HEARTBEAT_TIMEOUT` and `STOP_HEARTBEAT`; idle expiry after successful remote STOP keeps that successful result.
- Change address with `SAVE`, verify the new address/configuration is persisted, then reboot before addressing Nano at the new physical I2C address.

1. Read IDENTITY:
- Write pointer `0x00`, then read all 8 bytes. They are `0x53`, version `3`, physical address, capabilities and four zero bytes.

2. Command handshake:
- Stage CONFIG_A (`0x30`) and CONFIG_B (`0x50`), then write COMMAND (`0x80`): address, command code and nonzero 32-bit sequence.
- Read all 32 STATUS bytes (`0x10`). `commandSeq` and `ackSeq` at offsets 12 and 16 equal the received sequence; offsets 5 and 6 report result and error. No command register is reset on v3.

3. Unsupported mode:
- At an odd address, stage Filling in CONFIG_A and send APPLY.
- STATUS reports `FAILED` with `UNSUPPORTED_MODE`; no motion starts.

4. Relay command:
- Stage the low four relay bits in CONFIG_A, then send COMMAND `RELAY`.
- Physical relays match the staged mask only after a successful STATUS result.
- Read CONFIG_A and confirm its active relay mask matches GPIO; power-cycle and confirm that this runtime-only relay change was not written to EEPROM.

5. Stop reasons:
- Force a sensor stop and confirm STATUS `stopReason=STOP_SENSOR`.
- Let a finite move finish and confirm `stopReason=STOP_COMPLETE`; begin another move and confirm it resets to `STOP_NONE`.

## Calibration Checks

1. Pump calibration start:
- Device is in Pump role.
- Send v3 COMMAND `CALIBRATE_START`.
- Pump starts and STATUS flag `CALIBRATION` is set.

2. Pump calibration finish:
- Let the pump deliver exactly 100 ml.
- Send v3 COMMAND `CALIBRATE_FINISH`.
- Pump stops and calibration status clears.
- Active CONFIG_B `stepsPerMl` is updated to measured steps / 100 and persists after readback verification.
