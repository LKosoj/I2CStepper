# I2CStepper

[Русская версия](README.md) | **English**

A sketch for Arduino Nano that drives a stepper motor and four relays. Its main purpose is to work together with Samovar as an expansion module.
The device can run standalone through a menu on a 16x2 LCD with a rotary encoder, or as an I2C slave for Samovar.

Relays:

- `MIXER_PUMP_PIN` - the first relay, usually an on/off pump or mixer.
- `RELE_PIN2`, `RELE_PIN3`, `RELE_PIN4` - additional outputs for peripherals.

Stepper motor modes:

- `Mixer` (`I2CMIXER`) - a mixer. Speed is set in revolutions per minute, run time in seconds. If the time is `0`, the motor keeps running until it is stopped explicitly. For cyclic operation a pause and an optional direction change after the pause are available.
- `Pump` (`I2CPUMP`) - a peristaltic pump. Speed is set in ml/h, the motor keeps running until a stop command. An external sensor can stop the pump or put it on pause if this is enabled by flags.
- `Fill` (`I2CFILLING`) - dosed filling through the pump. Speed is set in ml/h, target volume in ml; once the volume is reached the stepper stops.

When the speed is changed manually during motion, the target in steps does not change, so the displayed remaining time for `Mixer` or remaining volume for the pump modes is recalculated from the current speed and the remaining steps.

`Mixer` uses the `I2CMIXER` role/address, while `Pump` and `Fill` use the `I2CPUMP` role/address. The mode is selected in the `Setup` menu (`Mixer` / `Pump` / `Fill`). A short press after editing saves the value to EEPROM. Changing the I2C address reboots the device after saving so that the new address takes effect.

A detailed description of the screens and controls is in the [“Screens and controls”](#screens-and-controls) section.

For the pump modes `STP/ML` stores the number of steps per 1 ml. The I2C calibration commands are designed for pouring 100 ml: `CALIBRATE_START` starts the pump, `CALIBRATE_FINISH` stops it, recalculates `STP/ML` as steps travelled / 100 and saves the result to EEPROM with a readback check.

For `Mixer` the number of steps per revolution cannot be configured in the Nano menu or in Samovar - it is set in the sketch before flashing. In `I2CStepper.h` set `#define STEPPER_MS` to the microstepping (step division) selected by the jumpers on the motor driver: `1` - full step, `2` - 1/2, `8` - 1/8, `16` - 1/16 and so on. The default is `STEPPER_MS 2`, that is `STEPPER_STEPS = 200 × 2 = 400` steps per revolution. If the motor has a 0.9° step (400 full steps per revolution instead of 200), replace `200` in `STEPPER_STEPS` with `400`. If the value does not match the driver, the mixer will not turn at the requested speed: for example, with `STEPPER_MS 2` and a driver set to 1/8 it turns 4 times slower. The same number determines the mixer speed limit: 18000 steps per second is 2700 rpm at 400 steps per revolution and 675 rpm at 1600. The pump modes are not affected: there everything is defined by the `STP/ML` calibration.

In v3 Samovar sends a heartbeat every 250 ms to each Nano, including while idle. While the heartbeat is no older than 10,000 ms (10 s) and the drive is busy (motor, cycle pause, calibration), the local controls are locked except for the local STOP: instead of the menu the display shows the progress (`STP Time`/`STP ML`/`Continuous`) and the hint `Click = STOP`, and pressing the encoder stops the drive; while idle the menu and the encoder are available. When the heartbeat is lost the Nano always switches off the remotely controlled relays; `HEARTBEAT_TIMEOUT` is reported only if a remote motion, pause or calibration was in progress at that moment. A local start does not require a heartbeat.

EEPROM v3 stores CONFIG_A/B with CRC-16 and a readback check. The old v2 format is migrated once; settings from firmware 0.1–0.4 (role and `STP/ML` starting at byte zero) are migrated as well: role `1` gives address 1 (mixer), role `2` gives address 2 (pump); in all other cases (empty EEPROM, foreign data, a corrupted v3 record, a failed v2 migration) default values are written: address 1, mixer. The `EEPROM_INVALID` error with motion locked remains only when writing to EEPROM fails.

A compiler check limits the v3 static global data to 320 bytes. The actual stack headroom on a real Nano depends on the LCD library and interrupts, so after flashing a hardware scenario with LCD, I2C and the encoder working at the same time is required.

Current protocol specification: [`docs/i2c-protocol-v3.md`](docs/i2c-protocol-v3.md) (in Russian). v2 is kept only as a local EEPROM format for migration, without a wire protocol.

Support for other devices may be added in the future.

Use the libraries that ship with Samovar.

## Screens and controls

The display is 16 characters × 2 lines, so only two lines of a screen are visible at a time; the rest are scrolled with the encoder. There are three screens: the main screen, the stepper screen and the setup screen. A fourth, service screen appears on its own when the drive is controlled by Samovar.

### Encoder

| Action | In selection mode (arrow marker `←`) | In edit mode (square marker `▪`) |
|---|---|---|
| Rotate | move to the next/previous line | the value changes by 1 |
| Rotate with the button held | nothing | the value changes by 10; with prolonged rotation the step grows (20, 30, …) |
| Short press | the line's action: open a screen, toggle, enter edit mode | save the value to EEPROM (non-volatile memory) and return to line selection |

The marker is on the right of the active line. An arrow means you are selecting a line. A square means you are changing the value of that line.

The screen is redrawn once a second (values only) and completely once every 10 seconds.

The `Invalid config` message (shown for 1 second) means the value failed validation and was **not saved**: for example, the speed converted to steps is outside 1…18000 steps/s, or the volume gives too long a path. The menu stays in edit mode - correct the value and press again.

### Main screen

| Line | What it shows | Press |
|---|---|---|
| `STP T:120>` / `STP ML:250>` / `Continuous>` | A brief stepper state: in `Mixer` mode - run time in seconds, in `Fill` - volume in ml, in `Pump` - `Continuous`, that is unlimited operation. During motion the number counts down - it is the remainder. | Opens the stepper screen |
| `Pump:On/Off` | Relay 1 (`MIXER_PUMP_PIN`, pin 13) - an ordinary on/off pump or mixer | Toggles the relay |
| `Rele2:On/Off` | Relay 2 (pin 10) | Toggles the relay |
| `Rele3:On/Off` | Relay 3 (pin 11) | Toggles the relay |
| `Rele4:On/Off` | Relay 4 (pin 12) | Toggles the relay |
| `SETUP>` | - | Opens the setup screen |

Relays switch immediately. The toggle itself is not written to EEPROM, but the current relay state goes to EEPROM on any subsequent save (editing speed, time, direction, settings) and will then be restored after a reboot. Samovar sees the relay state and can change it with its own commands. When the link to Samovar is lost (no heartbeat - the keep-alive signal - for more than 10 s) all relays are switched off.

### Stepper screen

| Line | What it shows | Press / edit |
|---|---|---|
| `STP Spd:20` | Speed: in `Mixer` - revolutions per minute, in `Pump` and `Fill` - ml/h | Press to enter edit mode, rotate to change, press to save |
| `STP Dir:0` | Rotation direction: `0` or `1` | Each press changes the direction immediately and saves it; there is no edit mode |
| `STP Time:120` / `STP ML:250` / `Continuous` | `Mixer` - run time, s; `Fill` - volume, ml; `Pump` - continuous operation, nothing to set | `Mixer` and `Fill`: press - edit - press to save. `Pump`: a press does nothing |
| `STP Start:Off/On` | Whether the stepper is running | A press starts it, another press stops it |
| `<BACK` | - | Returns to the main screen |

Details:

- **Speed.** The limits are applied automatically. The minimum is the speed that gives at least 1 step per second. The maximum is 18000 steps per second: for `Mixer` this is 2700 rpm, for the pump it depends on `STP/ML` (for example, at 16000 steps/ml it is 4050 ml/h). It is saved as the speed of the current mode: `Mixer`, `Pump` and `Fill` have separate values.
- **Editing on the fly.** Speed and direction can be changed while the motor is running - the change is applied immediately. When the speed changes, the remaining path in steps stays the same, so the displayed remaining time (or ml) is recalculated. Time/volume can also be changed on the fly: a new remainder is set from the current moment.
- **Time (`Mixer`).** From 0 to 100000 s. `0` means “run until stopped manually”; the screen keeps showing `STP Time:0`.
- **Volume (`Fill`).** From 1 to 100000 ml. After the set volume has been dispensed the motor stops on its own.
- **Start.** The motor will not start if the speed gives 0 steps per second, if in `Fill` or in `Mixer` with a time the path comes out as zero, or if there was an EEPROM error at startup (motion is locked). The line then stays `Off`.
- **Smooth acceleration and deceleration.** Enabled with the “Плавный разгон и торможение” (smooth acceleration and deceleration) checkbox in Samovar: “Settings” → `I2CStepper` tab → “Save to Nano” (enabled by default; the Nano menu has no such option). When running to a set path (`Fill` and `Mixer` with a non-zero time) the motor accelerates for about 10 seconds and decelerates the same way. In continuous motion (`Pump`, `Mixer` with time 0) the motor accelerates just as smoothly but stops immediately: the STOP command must not lag. A speed change on the fly is also smooth upwards and instant downwards, and the motor does not stop for it. Without the checkbox every start and every stop happens immediately.
- **Stop.** A stop from the encoder is reported to Samovar as a “local STOP”.
- The pause between mixer cycles, the direction change after the pause and the reaction to an external sensor **cannot be configured** from the local menu - only from Samovar. If they are set, they also apply to a local start.

### Setup screen (`SETUP`)

| Line | What it shows | Edit |
|---|---|---|
| `Type:Mixer/Pump/Fill` | Stepper motor mode | Rotating in either direction switches `Pump` ↔ `Fill`. On an odd address the mode is always `Mixer` and cannot be changed |
| `STP/ML:16000` | How many motor steps correspond to 1 ml - the pump calibration | Step 1, with the button held - 10 and more. Minimum 100. Not used for `Mixer` |
| `I2C Adr:1` | The device address on the I2C bus for Samovar, from 1 to 10 | Rotation changes the address by 1 |
| `<BACK` | - | Returns to the main screen |

The address defines the device role: **odd addresses (1, 3, 5, 7, 9) are a mixer** (`Mixer`), **even ones (2, 4, 6, 8, 10) are a pump** (`Pump` or `Fill`). So to turn a pump into a mixer you change the address, not the type: on switching to an odd address the type becomes `Mixer` by itself, on switching to an even one - `Pump`. The other settings are preserved.

Saving:

- A press in edit mode saves the settings to EEPROM. The type and `STP/ML` take effect immediately.
- If the address was changed, `Saved. Reboot!` appears on the screen and the device reboots - the new address takes effect only after a reboot.
- If you leave through `<BACK` without saving the changes with a press, they are saved automatically, after which the device also reboots with the `Saved. Reboot!` message. If nothing was changed, `<BACK` simply returns to the main screen.

`STP/ML` does not have to be tuned by hand: Samovar can calibrate the pump by pouring 100 ml (see `CALIBRATE_START`/`CALIBRATE_FINISH` above).

### Remote control screen

Appears on its own when Samovar is connected and the drive is busy on its command (the motor is running, a mixer cycle pause or a calibration is in progress):

```
STP ML:232
Click = STOP
```

- The top line is the same as on the stepper screen: remaining time (`STP Time`), remaining volume (`STP ML`) or `Continuous`. It is updated once a second.
- The menu is locked during this time: rotating the encoder does nothing.
- **Pressing the encoder stops the drive.** Samovar receives a “stopped manually” mark (`STOP_LOCAL`) and cannot cancel this stop with the heartbeat; it can start the drive again only with a new command.
- When the job is finished, the drive is stopped or the link to Samovar has been lost for more than 10 s, the normal menu returns at the same place where it was.

While Samovar is connected but the drive is idle, the menu works as usual.

## Building with PlatformIO

The repository contains a `platformio.ini`.
The build requires a clone of `Samovar` with its `libraries` folder next to this project:

- `../Samovar/libraries`

Build:

```bash
pio run
```

## Checks

- CI: [`.github/workflows/ci.yml`](.github/workflows/ci.yml)
- Local host test of the math: `g++ -std=c++11 -Wall -Wextra -pedantic -I. tests/stepper_math_test.cpp -o /tmp/stepper_math_test && /tmp/stepper_math_test`
- Shared v3 header for Arduino IDE: `../Samovar/libraries/I2CStepperProtocol/src/I2CStepperV3.h` (the `I2CStepperProtocol` library).
- Local host test of the v3 protocol: `g++ -std=c++11 -Wall -Wextra -pedantic -I. -I../Samovar/libraries/I2CStepperProtocol/src tests/i2c_protocol_v3_test.cpp -o /tmp/i2c_protocol_v3_test && /tmp/i2c_protocol_v3_test`
- Local host test of EEPROM/mailbox v3: `g++ -std=c++11 -Wall -Wextra -pedantic -I. tests/i2c_stepper_runtime_test.cpp -o /tmp/i2c_stepper_runtime_test && /tmp/i2c_stepper_runtime_test`
- Source-level host test of the v3 runtime/Timer1: `python3 tests/i2c_stepper_v3_runtime_test.py`
- Smooth acceleration flag check against the real `GyverStepper2` library (needs the sibling `../Samovar` directory): `python3 tests/smooth_start_sim_test.py`
- Hardware checklist: [`docs/hardware-test-checklist.md`](docs/hardware-test-checklist.md) (in Russian)

Peripheral wiring:


    // Pins for I2C Master - the display and potentially other devices
    // SDA_PIN 0 //A0
    // SCL_PIN 2 //D2

    // Pins for I2C Slave - link to Samovar
    // SDA_PIN A4
    // SCL_PIN A5

    // Stepper motor pins
    #define STEPPER_STEP 3
    #define STEPPER_DIR 4
    #define STEPPER_EN 5

    // Relay pins
    #define MIXER_PUMP_PIN 13 // RELE_PIN1
    #define RELE_PIN2 10      // RELE_PIN2
    #define RELE_PIN3 11      // RELE_PIN3
    #define RELE_PIN4 12      // RELE_PIN4

    // Encoder pins
    #define ENC_CLK 7 //S2
    #define ENC_DT 8  //S1
    #define ENC_SW 9  //KEY
