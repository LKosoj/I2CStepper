#!/usr/bin/env python3
# Проверка флага I2CSTEPPER_FLAG_SMOOTH_START на настоящей библиотеке GyverStepper2:
# функции запуска и остановки берутся из I2CStepper.ino, таймер шагов заменён виртуальным временем.
import shutil
import subprocess
import tempfile
from pathlib import Path

from i2c_stepper_v3_runtime_test import ROOT, function_body

LIBRARY = ROOT.parent / "Samovar" / "libraries" / "GyverStepper" / "src"

ARDUINO_H = r'''
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
typedef uint8_t byte;
#define OUTPUT 1
#define HIGH 1
#define LOW 0
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline void delayMicroseconds(uint32_t) {}
inline uint32_t micros() { return 0; }
'''

HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <Arduino.h>
#include <GyverStepper2.h>

#define STEPPER_TARGET_LIMIT 2147483647UL
#define I2CSTEPPER_FLAG_SMOOTH_START 0x02U
#define I2CSTEPPER_V3_ERR_NONE 0
#define I2CSTEPPER_V3_ERR_BAD_CONFIG 1
#define I2CSTEPPER_V3_STOP_NONE 0
GStepper2<STEPPER2WIRE> stepper(3200, 2, 3, 4);
struct { uint8_t optionFlags; } I2CSTPSetup = {};
struct { uint8_t mode, direction; uint32_t speedStepsPerSec, targetSteps; } v3_staging_motion = {};
struct { uint8_t error, stopReason; } v3_status_snapshot = {};
bool v3_movement_allowed = true, stepper_state, pause_phase, calibration_active;
bool v3_motion_continuous, v3_mixer_deadline_active;
uint8_t v3_runtime_mode = 0;
uint16_t curr_spd = 0;
byte set_dir = 0, last_dir = 0;
bool timer_armed = false;
uint8_t pause_stepper_timer() { return 0; }
void resume_stepper_timer(uint8_t) {}
void timer1_schedule(uint32_t) { timer_armed = true; }
void timer1_disarm() { timer_armed = false; }
uint16_t get_motion_speed() { return (uint16_t)v3_staging_motion.speedStepsPerSec; }
void set_motion_speed(uint16_t speed) { v3_staging_motion.speedStepsPerSec = speed; }
void set_motion_target(uint32_t) {}

@FIRMWARE@

// Крутит мотор untilUs микросекунд виртуального времени так же, как прерывание таймера и loop().
// Возвращает время, за которое пауза между шагами дошла до заданной (мотор вышел на полную скорость).
static uint32_t run_for(uint32_t untilUs, uint32_t fullPeriod, uint32_t* firstPeriod) {
  uint32_t now = 0, reached = 0;
  *firstPeriod = stepper.getPeriod();
  while (now < untilUs && stepper.getState()) {
    now += stepper.getPeriod();
    stepper.tickManual();
    finish_continuous_ramp();
    if (!reached && stepper.getPeriod() <= fullPeriod) reached = now;
  }
  return reached;
}

// Время от команды остановки до полной остановки мотора.
static uint32_t stop_time(bool smooth) {
  uint32_t now = 0;
  stop_motion(smooth);
  while (stepper.getState()) {
    now += stepper.getPeriod();
    stepper.tickManual();
  }
  return now;
}

// Проверяет один прогон на скорости spd: медленной, средней и предельной для прошивки.
static void check_speed(uint16_t spd) {
  const uint32_t fullPeriod = 1000000UL / spd;
  // Порядок выкл-вкл-выкл: проверяем и то, что после работы с разгоном флаг действительно отключается.
  for (uint8_t pass = 0; pass < 3; pass++) {
    const uint8_t smooth = pass & 1U;
    for (uint8_t continuous = 0; continuous < 2; continuous++) {
      I2CSTPSetup.optionFlags = smooth ? I2CSTEPPER_FLAG_SMOOTH_START : 0;
      start_motion(spd, 1000000UL, 0, continuous);
      uint32_t firstPeriod = 0;
      uint32_t ramp = run_for(15000000UL, fullPeriod, &firstPeriod);
      uint8_t status = stepper.getStatus();
      uint32_t stop = stop_time(smooth);
      printf("spd=%u smooth=%u continuous=%u first_period_us=%lu ramp_ms=%lu status=%u stop_ms=%lu\n",
             spd, smooth, continuous, (unsigned long)firstPeriod, (unsigned long)(ramp / 1000),
             status, (unsigned long)(stop / 1000));
      if (smooth) {
        // Старт с длинной паузы между шагами, выход на скорость примерно за 10 с.
        assert(firstPeriod > 10 * fullPeriod && ramp > 8000000UL && ramp < 12000000UL);
      } else {
        // Сразу полная скорость.
        assert(firstPeriod == fullPeriod && ramp <= fullPeriod);
      }
      // После разгона непрерывное вращение идёт в режиме постоянной скорости (3), конечный ход - к цели (1).
      assert(status == (continuous ? 3 : 1));
      // Плавно тормозит только конечный ход с флагом; остальное останавливается сразу.
      // На предельной скорости библиотеке не хватает точности счёта, торможение выходит короче (около 6 с).
      if (smooth && !continuous) assert(stop > 5000000UL && stop < 12000000UL);
      else assert(stop == 0);
    }
  }
}

int main() {
  check_speed(100);
  check_speed(2000);
  // Выше 6553 шаг/с путь торможения больше 32767 шагов: раньше здесь ломалась плавная остановка.
  check_speed(8000);
  check_speed(18000);
  return 0;
}
'''


def main():
    firmware = "\n".join(
        signature + " {" + function_body(signature) + "}"
        for signature in (
            "static uint16_t stepper_acceleration_from_speed(uint16_t spd)",
            "static void stop_motion(bool smooth)",
            "static void run_stepper(uint16_t spd, uint32_t target, bool continuous)",
            "static void start_motion(uint16_t spd, uint32_t target, byte dir, bool continuous)",
            "static void finish_continuous_ramp()",
        ))
    compiler = shutil.which("g++")
    assert compiler is not None, "g++ is required"
    with tempfile.TemporaryDirectory(prefix="i2cstepper-smooth-") as temporary:
        directory = Path(temporary)
        (directory / "Arduino.h").write_text(ARDUINO_H, encoding="utf-8")
        cpp = directory / "smooth.cpp"
        cpp.write_text(HARNESS.replace("@FIRMWARE@", firmware), encoding="utf-8")
        built = subprocess.run(
            [compiler, "-std=c++11", "-I", str(directory), "-I", str(LIBRARY),
             str(cpp), "-o", str(directory / "smooth")],
            capture_output=True, text=True, check=False)
        assert built.returncode == 0, built.stdout + built.stderr
        result = subprocess.run([str(directory / "smooth")], capture_output=True, text=True, check=False)
        print(result.stdout, end="")
        assert result.returncode == 0, result.stderr


if __name__ == "__main__":
    main()
