#include <assert.h>
#include <stdint.h>

#include "StepperMath.h"

int main() {
  const uint32_t limit = 2147483647UL;

  assert(stepper_clamp_target(0ULL, limit) == 0UL);
  assert(stepper_clamp_target((uint64_t)limit, limit) == limit);
  assert(stepper_clamp_target((uint64_t)limit + 1ULL, limit) == limit);

  assert(stepper_speed_steps_mixer(0UL, 400U) == 0U);
  assert(stepper_speed_steps_mixer(60UL, 400U) == 400U);
  assert(stepper_speed_steps_mixer(600000UL, 400U) == 65535U);

  assert(stepper_speed_steps_pump(0UL, 4500UL) == 0U);
  assert(stepper_speed_steps_pump(3600UL, 4500UL) == 4500U);
  assert(stepper_speed_steps_pump(3600UL, 0UL) == 0U);
  assert(stepper_speed_steps_pump(99999999UL, 4500UL) == 65535U);

  assert(stepper_target_from_time_mixer(0UL, 100U, limit) == 0UL);
  assert(stepper_target_from_time_mixer(10UL, 100U, limit) == 1000UL);
  assert(stepper_target_from_time_mixer(limit, 2U, limit) == limit);

  assert(stepper_target_from_time_pump(0UL, 4500UL, limit) == 0UL);
  assert(stepper_target_from_time_pump(100UL, 4500UL, limit) == 4500UL);
  assert(stepper_target_from_time_pump(limit, 4500UL, limit) == limit);

  return 0;
}
