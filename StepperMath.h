#ifndef __STEPPER_MATH_H
#define __STEPPER_MATH_H

#include <stdint.h>

static inline uint32_t stepper_clamp_target(uint64_t target, uint32_t limit) {
  if (target > (uint64_t)limit) {
    return limit;
  }
  return (uint32_t)target;
}

// value * factor / divisor с округлением к ближайшему, без ограничения результата.
// noinline: 64-битная арифметика на AVR дорогая, держим её в одном экземпляре.
__attribute__((noinline)) static uint64_t stepper_rate_steps(uint32_t value, uint32_t factor, uint16_t divisor) {
  return ((uint64_t)value * factor + divisor / 2U) / divisor;
}

static inline uint16_t stepper_speed_steps_mixer(uint32_t user_speed, uint16_t stepper_steps) {
  uint64_t speed_steps = stepper_rate_steps(user_speed, stepper_steps, 60U);
  if (speed_steps > 65535ULL) {
    speed_steps = 65535ULL;
  }
  return (uint16_t)speed_steps;
}

// stepper_step_ml хранится в единицах "шагов на 1 мл".
// user_speed [мл/ч] * step_ml [шагов/мл] / 3600 с/ч = шагов/с.
static inline uint16_t stepper_speed_steps_pump(uint32_t user_speed, uint32_t stepper_step_ml) {
  if (stepper_step_ml == 0) {
    return 0;
  }
  uint64_t speed_steps = stepper_rate_steps(user_speed, stepper_step_ml, 3600U);
  if (speed_steps > 65535ULL) {
    speed_steps = 65535ULL;
  }
  return (uint16_t)speed_steps;
}

// a * b с ограничением сверху; noinline по той же причине, что и stepper_rate_steps.
__attribute__((noinline)) static uint32_t stepper_target_steps(uint32_t a, uint32_t b, uint32_t limit) {
  return stepper_clamp_target((uint64_t)a * b, limit);
}

static inline uint32_t stepper_target_from_time_mixer(uint32_t time_value, uint16_t speed_steps, uint32_t limit) {
  return stepper_target_steps(time_value, speed_steps, limit);
}

static inline uint32_t stepper_target_from_time_pump(uint32_t volume_ml, uint32_t stepper_step_ml, uint32_t limit) {
  return stepper_target_steps(volume_ml, stepper_step_ml, limit);
}

#endif // __STEPPER_MATH_H
