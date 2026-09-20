# I2CStepper I2C protocol v3

## Scope

This is the only protocol for new I2CStepper and Samovar firmware. Protocol v2 is not accepted on the wire. EEPROM v2 migration is a local Nano boot operation, not a v2 wire fallback.

## Device address and type

The physical I2C slave address is `1..10`. An odd address is a Mixer; an even address is a Pump. A Pump also supports Filling. Type is not stored separately.

Every v3 command contains the target address in addition to the I2C slave address. Nano rejects a command whose address does not equal its own address. Addresses `1..7` are reserved by the I2C standard, but are intentionally part of the selected physical range.

When an address changes, only `SAVE` validates and persists it; `APPLY` with an address other than the current physical address is rejected with `BAD_ADDRESS`. The `Wire` slave keeps its old physical address until Nano reboot. Before reboot, CONFIG_A/B readback contains the new persisted configuration, while identity, status, capabilities and addressed commands remain on the old boot address and its old effective mode; Samovar must continue its heartbeat there until reboot. `START_FINITE`, `START_CONTINUOUS`, `START_CONFIGURED` and `CALIBRATE_START` are rejected with `REBOOT_REQUIRED` while an address change is pending reboot, before their mode or MOTION validation. STOP, HEARTBEAT and RELAY remain available on the old address. The local Setup screen edits this physical address explicitly in the `1..10` range and shows `Saved. Reboot!` only after a successful write. A change within the same parity retains the selected mode. Pump to Mixer selects Mixer; Mixer to Pump selects Pump. Other settings, including `stepsPerMl`, are retained. An invalid address or a mode not supported by its parity is rejected; it is never corrected silently.

## Wire transfer rules

Arduino Nano Wire has a 32-byte buffer. A write is one register pointer byte followed by no more than 31 payload bytes. A read response is no more than 32 bytes. Multi-byte fields are big-endian and must be read from one documented frame, never byte by byte from unrelated snapshots.

`Samovar/libraries/I2CStepperProtocol/src/I2CStepperV3.h` is the single shared v3 header. It defines semantic C++ objects and explicit byte-buffer encode/decode helpers for every v3 frame. The objects must never be copied to or from Wire with `memcpy`; only the named offset constants and codec helpers define the AVR/ESP/host-independent ABI.

The master writes a frame start register, then reads the whole frame from that register. Nano implements this only through Arduino `Wire` slave callbacks: `onReceive` copies a complete write directly into the pending mailbox for CONFIG_A (20), CONFIG_B (17), MOTION (10) or COMMAND (6), and `onRequest` returns a prebuilt frame. A full mailbox preserves its first packet and discards the later packet. `loop()` consumes pending mailboxes in the fixed CONFIG_A, CONFIG_B, MOTION, COMMAND order, so the standard four-frame burst is not mixed or dropped. EEPROM, relays, motor and command execution run later in `loop()`. The single COMMAND mailbox is sufficient because Samovar waits for its ACK/result before issuing its next command. Protocol v2 registers and `iarduino_I2C_connect` are not used on the wire.

Nano publishes status and active configuration from one immutable snapshot. `generation` is the active-configuration generation: it changes only when a new active configuration is published by `APPLY`, `SAVE`, successful calibration or a successful address configuration. Speed, remaining steps and other telemetry updates do not change it.

`config A` and `config B` read the active published configuration. Writes to those registers change only a separate staging configuration. To obtain a coherent active configuration, Samovar reads status `generation`, reads config A and B, then reads status `generation` again; it retries if the two generations differ. Staging writes alone do not change generation. A successful `APPLY` or `SAVE` publishes the new active configuration and advances generation.

## Registers and frames

| Start register | Size | Direction | Contents |
| --- | ---: | --- | --- |
| `0x00` identity | 8 | R | magic, version, address, capabilities, reserved bytes |
| `0x10` status | 32 | R | magic, version, address, mode, status, command result, error, stop reason, generation, commandSeq, ackSeq, stopEventSeq, current speed, remaining steps |
| `0x30` config A | 20 | R active / W staging | mode, option flags, sensor flags, relay mask, mixer RPM, mixer run/pause seconds, pump ml/h |
| `0x50` config B | 17 | R active / W staging | requested address, pump pause seconds, filling ml, filling ml/h, steps/ml |
| `0x70` motion | 10 | R/W staging | mode, direction, speed in steps/s, target in steps |
| `0x80` command | 6 | W | address, command, commandSeq |

`identity`: offsets `0..7` are magic, version, address, capabilities and four zero reserved bytes. `status`: offsets `0..7` are magic, version, address, mode, status, command result, error and stop reason; `8`, `12`, `16`, `20`, `24`, `28` start respectively generation, commandSeq, ackSeq, stopEventSeq, current speed and remaining steps.

`config A`: offsets `0..3` are mode, option flags, sensor flags and relay mask; offsets `4`, `8`, `12`, `16` start mixer RPM, mixer run seconds, mixer pause seconds and pump ml/h. `config B`: address at `0`; pump pause seconds, filling ml, filling ml/h and steps/ml start at `1`, `5`, `9`, `13`. `motion`: mode at `0`, direction at `1`, speed at `2`, target at `6`. `command`: address at `0`, code at `1`, commandSeq at `2`.

`magic = 0x53`; `version = 3`. `generation`, `commandSeq`, `ackSeq`, `stopEventSeq`, all speeds, times, volumes, target steps and `stepsPerMl` are unsigned 32-bit values. Relay mask uses low four bits. Status and capability flags keep their documented bit values in `I2CStepperV3.h`.

`currentSpeedStepsPerSec` and `remainingSteps` are always physical step values. The UI may derive seconds or millilitres for presentation, but must not overwrite them in the protocol.

## Value ranges

- address: `1..10`; a command address must equal the addressed Nano;
- mode: Mixer only for an odd address; Pump or Filling only for an even address;
- direction: `0` or `1`; relay mask: `0x00..0x0F`;
- finite motion: `speedStepsPerSec = 1..18000`, `targetSteps = 1..2147483647`;
- continuous motion: `speedStepsPerSec = 1..18000`; target is ignored and is not a sentinel;
- configuration speeds, durations and volumes: `0..4294967295`; `stepsPerMl = 1..4294967295`.

An out-of-range field makes `APPLY`, `SAVE`, `START_*`, `RELAY` or calibration fail with `BAD_ADDRESS`, `UNSUPPORTED_MODE` or `BAD_CONFIG`; neither runtime configuration nor EEPROM changes. `START_CONFIGURED` returns `REBOOT_REQUIRED` if saved and runtime addresses differ. Unknown command uses `BAD_COMMAND`.

Cross-field validation uses 64-bit intermediate values. For the selected applied mode Nano checks that the derived rate is nonzero and not above 18000 steps/s: Mixer uses `mixerRpm * STEPPER_STEPS / 60`; Pump/Filling uses `mlHour * stepsPerMl / 3600`. Filling also checks `fillingMl * stepsPerMl` is `1..2147483647`. `START_FINITE` separately checks its staged `targetSteps` in that range and `START_*` checks staged `speedStepsPerSec` in `1..18000`. A zero or overflowing derived rate/target is `BAD_CONFIG`; it is never truncated.

## Numeric ABI

The following literal values are ABI and are asserted by `tests/i2c_protocol_v3_test.cpp`: magic `0x53`, version `3`, maximum speed `18000` steps/s, registers `00/10/30/50/70/80`, commands `APPLY=1`, `START_FINITE=3`, `START_CONTINUOUS=4`, `CALIBRATE_FINISH=8`, `HEARTBEAT=9`, `START_CONFIGURED=10`, errors `BAD_SEQUENCE=8`, `REBOOT_REQUIRED=9`, results `PENDING=1`, `SUCCESS=2`, `FAILED=3`, and frame sizes `8/32/20/17/10/6`.

## Commands and atomic settings

| Code | Command | Meaning |
| ---: | --- | --- |
| 1 | `APPLY` | Validate all staging frames and atomically make them the current runtime configuration. |
| 2 | `SAVE` | Perform `APPLY`, then persist the valid configuration. |
| 3 | `START_FINITE` | Start the staged finite motion to its exact `targetSteps`. |
| 4 | `START_CONTINUOUS` | Start the staged continuous motion; `targetSteps` is ignored. |
| 5 | `STOP` | Stop the remote motion. |
| 6 | `RELAY` | Apply the staged relay mask. |
| 7 | `CALIBRATE_START` | Start pump calibration. |
| 8 | `CALIBRATE_FINISH` | Stop calibration, compute measured steps per 100 ml and persist `stepsPerMl`. |
| 9 | `HEARTBEAT` | Refresh remote ownership only. It never starts a motor or clears STOP. |
| 10 | `START_CONFIGURED` | Start the valid active saved configuration without reading MOTION or applying staging. |

`START_FINITE` and `START_CONTINUOUS` use the already applied configuration plus the separately staged motion frame. They never imply `APPLY`; a changed config frame requires an explicit successful `APPLY` or `SAVE` before START. A partial staging write changes neither live settings nor EEPROM.

`START_CONFIGURED` neither reads MOTION nor applies or saves staging. It uses only the current active saved Nano configuration and its existing Nano-side step calculations: Mixer is continuous when `mixerRunSec=0`, otherwise finite; Pump is continuous; Filling is finite. Its status and MOTION readback contain the resulting physical direction, speed and target. The command requires a valid configuration and equal runtime and saved address.

The staged `motion.mode` must exactly equal the active configuration mode. A mismatch is `UNSUPPORTED_MODE` for both `START_FINITE` and `START_CONTINUOUS`; Nano must not infer or substitute a mode.

| Command | Mixer | Pump | Filling | Additional requirement |
| --- | --- | --- | --- | --- |
| `APPLY` | yes | yes | yes | full valid staging configuration at the current runtime address |
| `SAVE` | yes | yes | yes | full valid staging configuration; a new address becomes effective only after reboot |
| `START_FINITE` | yes | no | yes | applied mode and valid staged finite motion |
| `START_CONTINUOUS` | yes | yes | no | applied mode and valid staged continuous motion |
| `START_CONFIGURED` | yes | yes | yes | valid active saved config; runtime address equals saved address |
| `STOP` | yes | yes | yes | none |
| `RELAY` | yes | yes | yes | relay capability and valid staged mask |
| `CALIBRATE_START`, `CALIBRATE_FINISH` | no | pump-capable address | pump-capable address | stopped device; calibration itself operates as Pump |
| `HEARTBEAT` | yes | yes | yes | any present Nano; it refreshes remote ownership even while idle |

### Command sequence and result

`commandSeq` is a nonzero 32-bit serial number. Nano retains the last processed sequence and cached result. A larger sequence in modulo-`2^32` order is new; `0` is invalid; an equal sequence is a duplicate and returns the cached result without executing again; a smaller or half-range-ambiguous sequence fails with `BAD_SEQUENCE`. The progression `0xFFFFFFFE`, `0xFFFFFFFF`, `1` is valid; zero is skipped after wrap.

On receipt of a new addressed command Nano publishes `PENDING`. On completion it publishes the same command sequence in both `commandSeq` and `ackSeq`, then `SUCCESS` or `FAILED` and the exact error. A rejected addressed command, including bad address, bad sequence, mode, configuration or pending reboot, also gets its received sequence in `ackSeq` and a `FAILED` result. Retried `START_*` and `CALIBRATE_FINISH` packets with the same sequence are duplicates, not second executions.

## Ownership and errors

Samovar sends heartbeat every 250 ms to every present Nano, including idle devices. A valid heartbeat holds remote ownership and keeps local controls locked while the drive is busy (motion, pause phase, calibration); when idle the local menu stays usable; after more than 10000 ms (10 s) without it, local controls unlock and Nano always deenergizes remotely controlled relays. Relay success updates active runtime CONFIG_A readback only; it does not write EEPROM. Timeout clears active, staging and runtime relay masks to zero together with GPIO. If remote-started motion, pause or calibration was active at that point, Nano also stops it, clears pause/calibration and publishes `HEARTBEAT_TIMEOUT` and `STOP_HEARTBEAT`. Idle-only ownership loss, including expiry after a successful remote STOP, preserves the successful command result and does not manufacture a timeout error. Locally started motion does not need heartbeat. Local STOP is always available, publishes `STOP_LOCAL`, and cannot be reversed by heartbeat or a paused-motion timer. A sensor stop publishes `STOP_SENSOR`; an ordinary finite completion publishes `STOP_COMPLETE`; each new start resets the reason to `STOP_NONE`.

`stopEventSeq` is a separate nonzero 32-bit event counter. It starts at zero (no local STOP observed), increments once for each local STOP event and is unchanged by ordinary status reads. Samovar sends one notification only when the counter advances. It uses the same modulo rule as `commandSeq`: `0xFFFFFFFF` advances to `1`; zero is never emitted as an event number.

Malformed address, unsupported mode, invalid configuration, unknown command, invalid EEPROM and EEPROM write failure use the error codes in `I2CStepperV3.h`. `STOP` reports its own result, not a stale prior error.

## EEPROM v2 migration

EEPROM v2 is decoded locally as exactly 23 bytes: marker, version, role, mode, eight little-endian `uint16_t` settings (`mixerRpm`, `mixerRunSec`, `mixerPauseSec`, `pumpMlHour`, `pumpPauseSec`, `fillingMl`, `fillingMlHour`, `stepperStepMl`), option flags, sensor flags and relay mask. Only marker `0x53` and version `2` are migratable.

The mapping is literal: v2 Mixer role becomes address `1`; v2 Pump role becomes address `2`; all settings and flags copy unchanged; `stepperStepMl` widens from `uint16_t` to `uint32_t`; Mixer accepts only Mixer mode, while Pump accepts Pump or Filling. The v3 EEPROM record begins at byte 32: magic `0x53`, version `3`, payload length `37`, CRC-16/CCITT (big-endian), then CONFIG_A (20 bytes) and CONFIG_B (17 bytes). Nano classifies that record as valid, absent, or corrupt before it considers migration. A record with either v3 magic or version but invalid header, CRC or config is corrupt and is replaced with defaults (address `1`, Mixer). Only an absent v3 record may use a valid v2 record; without one, a legacy 0.1–0.4 record (byte 0 = role `1`/`2`, bytes 1–4 = little-endian steps per ml) maps role to address and mode and keeps steps per ml when the resulting config is valid; anything else receives defaults. Nano writes every byte with `EEPROM.update` and reads every byte back before accepting `SAVE`, migration, default initialization or calibration. Unknown data and invalid v2 input receive defaults as well, so the Nano always joins the bus; `EEPROM_INVALID` with movement disabled remains only for an EEPROM write/readback failure. Executable Mixer, Pump and Filling v2 byte fixtures are in `tests/i2c_protocol_v3_test.cpp`.

## Fixed packet examples

Write examples include the first register-pointer byte. Status examples are 32-byte read responses after the master has already written the `0x10` pointer.

| Case | Bytes | Meaning |
| --- | --- | --- |
| Mixer finite motion | `70 01 00 00 00 03 E8 00 00 EA 60` | mode Mixer, direction 0, 1000 steps/s, target 60000 steps. |
| Filling finite motion | `70 03 01 00 00 07 D0 00 01 E2 40` | mode Filling, direction 1, 2000 steps/s, target 123456 steps. |
| Pump continuous start | `80 02 04 00 00 00 2A` | address 2, `START_CONTINUOUS`, sequence 42. |
| Mixer continuous start | `80 01 04 00 00 00 2C` | address 1, `START_CONTINUOUS`, sequence 44. |
| Start saved Nano configuration | `80 02 0A 00 00 00 2D` | address 2, `START_CONFIGURED`, sequence 45. |
| CONFIG_B with 32-bit calibration | `50 02 00 00 00 0F 00 00 01 41 00 00 03 20 12 34 56 78` | address 2, pump pause 15, fill 321 at 800 ml/h, `stepsPerMl=0x12345678`. |
| Oversized MOTION speed | `70 03 01 00 01 00 00 00 01 E2 40` | speed `65536` is big-endian and exceeds the allowed rate, therefore `BAD_CONFIG`. |
| Calibration finish | `80 02 08 00 00 00 2B` | address 2, `CALIBRATE_FINISH`, sequence 43. |
| Retry continuous start | `80 02 04 00 00 00 2A` | duplicate of sequence 42; it returns cached result and does not restart. |
| Retry calibration finish | `80 02 08 00 00 00 2B` | duplicate of sequence 43; it returns cached result and does not recalibrate. |
| Invalid finite status response | `53 03 02 03 80 03 03 00 00 00 00 09 00 00 00 2C 00 00 00 2C 00 00 00 00 00 00 00 00 00 00 00 00` | Filling at address 2, failed result, `BAD_CONFIG`; generation 9 and command/ack 44. |
| Heartbeat timeout status response | `53 03 02 02 80 03 05 05 00 00 00 0A 00 00 00 2A 00 00 00 2A 00 00 00 00 00 00 00 00 00 00 00 00` | Pump at address 2, failed result, heartbeat timeout and heartbeat stop reason. |
| Finite mode mismatch | `70 02 00 00 00 03 E8 00 00 00 64` | staged Pump motion with active Filling; `START_FINITE` returns `UNSUPPORTED_MODE`. |
| Continuous mode mismatch | `70 01 00 00 00 03 E8 00 00 00 00` | staged Mixer motion with active Pump; `START_CONTINUOUS` returns `UNSUPPORTED_MODE`. |

The executable counterparts are `tests/i2c_protocol_v3_test.cpp`; literal bytes and numeric ABI values are asserted independently of the named enums.

For the local Nano menu, Mixer with `mixerRunSec=0` and Pump use continuous motion until an explicit STOP. Mixer with a nonzero run duration measures each run phase from the start of acceleration; after that full elapsed interval it either enters its configured pause or completes.
