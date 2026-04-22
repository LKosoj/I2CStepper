#ifndef __STEPPER_MATH_H
#define __STEPPER_MATH_H

#include <stdint.h>

static inline uint32_t stepper_clamp_target(uint64_t target, uint32_t limit) {
  if (target > (uint64_t)limit) {
    return limit;
  }
  return (uint32_t)target;
}

static inline uint16_t stepper_speed_steps_mixer(uint32_t user_speed, uint16_t stepper_steps) {
  uint64_t speed_steps = ((uint64_t)user_speed * stepper_steps + 30ULL) / 60ULL;
  if (speed_steps > 65535ULL) {
    speed_steps = 65535ULL;
  }
  return (uint16_t)speed_steps;
}

// stepper_step_ml хранится в единицах "шагов на 100 мл".
// user_speed [мл/ч] * step_ml [шагов/100мл] / (3600 с/ч * 100 мл) = шагов/с.
static inline uint16_t stepper_speed_steps_pump(uint32_t user_speed, uint32_t stepper_step_ml) {
  if (stepper_step_ml == 0) {
    return 0;
  }
  uint64_t speed_steps = ((uint64_t)user_speed * stepper_step_ml + 180000ULL) / 360000ULL;
  if (speed_steps > 65535ULL) {
    speed_steps = 65535ULL;
  }
  return (uint16_t)speed_steps;
}

static inline uint32_t stepper_target_from_time_mixer(uint32_t time_value, uint16_t speed_steps, uint32_t limit) {
  return stepper_clamp_target((uint64_t)time_value * (uint64_t)speed_steps, limit);
}

static inline uint32_t stepper_target_from_time_pump(uint32_t volume_ml, uint32_t stepper_step_ml, uint32_t limit) {
  uint64_t target = ((uint64_t)volume_ml * (uint64_t)stepper_step_ml + 50ULL) / 100ULL;
  return stepper_clamp_target(target, limit);
}

#endif // __STEPPER_MATH_H
