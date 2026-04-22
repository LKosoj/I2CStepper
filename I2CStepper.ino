#define LIQUIDMENU_LIBRARY LiquidCrystal_I2C2_LIBRARY
#define DisplayClass LiquidCrystal_I2C2
#define DRIVER_STEP_TIME 5

// Подключаем библиотеки:
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>                                     // подключаем библиотеку для работы с шиной I2C
#include <EEPROM.h>
#include <iarduino_I2C_connect.h>                     // подключаем библиотеку для соединения arduino по шине I2C

#include "PinChangeInterrupt.h"
#include <GyverEncoder.h>
#include <TimerOne.h>

#include "I2CStepper.h"
#include "StepperMath.h"

#include "I2CMenu.h"

#ifndef __I2CStepper_DEBUG
#define ARDUINOTRACE_ENABLE 0  // Disable all traces
#endif
#include <ArduinoTrace.h>

void read_config();
void write_config();
bool get_rele_state_from_array(byte r);
byte get_direction_from_array(void);
byte get_rele_mask_from_array(void);
void read_motion_from_array(uint32_t* target, uint16_t* speed, byte* dir);
uint32_t calc_target_from_time(uint32_t time_value, uint16_t spd);

static const uint16_t REG_READY_TIMEOUT_MS = 25;
static bool comm_timeout_error = false;
static bool comm_failsafe_latched = false;

enum RegisterIndex : uint8_t {
  REG_SPEED_MSB = 0,
  REG_SPEED_LSB = 1,
  REG_DIR = 2,
  REG_TARGET_3 = 3,
  REG_TARGET_2 = 4,
  REG_TARGET_1 = 5,
  REG_TARGET_0 = 6,
  REG_RELAY_MASK = 7,
  REG_READY = 8,
};

static volatile byte* reg_view() {
  return (volatile byte*)REG_Array;
}

//REG_READY выступает семафором: 1 — кто-то из участников (мастер или слейв) пишет в регистры.
//acquire_register_lock гарантирует, что слейв получает эксклюзивное владение без TOCTOU между проверкой
//и cli(): второй шаг проверяет ещё раз под cli(), и только тогда выставляет READY=1.
static bool acquire_register_lock() {
  const volatile byte* regs = reg_view();
  uint32_t start = millis();
  while (regs[REG_READY] == 1) {
    if ((uint32_t)(millis() - start) >= REG_READY_TIMEOUT_MS) {
      comm_timeout_error = true;
      return false;
    }
    delay(1);
  }

  uint8_t sreg = lock_interrupts();
  if (reg_view()[REG_READY] != 0) {
    //Мастер стартовал транзакцию между опросом и cli — считаем это таймаутом обмена.
    unlock_interrupts(sreg);
    comm_timeout_error = true;
    return false;
  }
  reg_view()[REG_READY] = 1;
  unlock_interrupts(sreg);
  return true;
}

static void release_register_lock() {
  reg_write_u8_atomic(REG_READY, 0);
}

static uint8_t lock_interrupts() {
  uint8_t sreg = SREG;
  cli();
  return sreg;
}

static void unlock_interrupts(uint8_t sreg) {
  SREG = sreg;
}

static uint8_t reg_read_u8_atomic(uint8_t index) {
  const volatile byte* regs = reg_view();
  uint8_t sreg = lock_interrupts();
  uint8_t value = regs[index];
  unlock_interrupts(sreg);
  return value;
}

static void reg_write_u8_atomic(uint8_t index, uint8_t value) {
  volatile byte* regs = reg_view();
  uint8_t sreg = lock_interrupts();
  regs[index] = value;
  unlock_interrupts(sreg);
}

static uint16_t reg_read_u16_atomic(uint8_t index_msb) {
  const volatile byte* regs = reg_view();
  uint8_t sreg = lock_interrupts();
  uint16_t value = ((uint16_t)regs[index_msb] << 8) | (uint16_t)regs[index_msb + 1];
  unlock_interrupts(sreg);
  return value;
}

static void reg_write_u16_atomic(uint8_t index_msb, uint16_t value) {
  volatile byte* regs = reg_view();
  uint8_t sreg = lock_interrupts();
  regs[index_msb] = value >> 8;
  regs[index_msb + 1] = value;
  unlock_interrupts(sreg);
}

static uint32_t reg_read_u32_atomic(uint8_t index_msb) {
  const volatile byte* regs = reg_view();
  uint8_t sreg = lock_interrupts();
  uint32_t value = ((uint32_t)regs[index_msb] << 24) |
                   ((uint32_t)regs[index_msb + 1] << 16) |
                   ((uint32_t)regs[index_msb + 2] << 8) |
                   (uint32_t)regs[index_msb + 3];
  unlock_interrupts(sreg);
  return value;
}

static void reg_write_u32_atomic(uint8_t index_msb, uint32_t value) {
  volatile byte* regs = reg_view();
  uint8_t sreg = lock_interrupts();
  regs[index_msb] = value >> 24;
  regs[index_msb + 1] = value >> 16;
  regs[index_msb + 2] = value >> 8;
  regs[index_msb + 3] = value;
  unlock_interrupts(sreg);
}

static void reg_read_motion_atomic(uint32_t* target, uint16_t* speed, byte* dir) {
  const volatile byte* regs = reg_view();
  uint8_t sreg = lock_interrupts();
  *speed = ((uint16_t)regs[REG_SPEED_MSB] << 8) | (uint16_t)regs[REG_SPEED_LSB];
  *dir = regs[REG_DIR];
  *target = ((uint32_t)regs[REG_TARGET_3] << 24) |
            ((uint32_t)regs[REG_TARGET_2] << 16) |
            ((uint32_t)regs[REG_TARGET_1] << 8) |
            (uint32_t)regs[REG_TARGET_0];
  unlock_interrupts(sreg);
}

static void reg_write_safe_defaults_atomic() {
  volatile byte* regs = reg_view();
  uint8_t sreg = lock_interrupts();
  regs[REG_SPEED_MSB] = 0;
  regs[REG_SPEED_LSB] = 0;
  regs[REG_DIR] = 0;
  regs[REG_TARGET_3] = 0;
  regs[REG_TARGET_2] = 0;
  regs[REG_TARGET_1] = 0;
  regs[REG_TARGET_0] = 0;
  regs[REG_RELAY_MASK] = 0;
  regs[REG_READY] = 0;
  unlock_interrupts(sreg);
}

uint8_t pause_stepper_timer() {
  uint8_t prescale = TCCR1B & (0b111 << CS10);
  TCCR1B &= ~(0b111 << CS10);
  return prescale;
}

void resume_stepper_timer(uint8_t prescale) {
  TCCR1B &= ~(0b111 << CS10);
  TCCR1B |= prescale;
}

static uint16_t stepper_acceleration_from_speed(uint16_t spd) {
  uint16_t acc = spd / 10;
  return acc == 0 ? 1 : acc;
}

static void stepper_snapshot_position_atomic(int32_t* current, int32_t* target_abs) {
  uint8_t saved_prescale = pause_stepper_timer();
  if (current) {
    *current = stepper.getCurrent();
  }
  if (target_abs) {
    *target_abs = stepper.getTarget();
  }
  resume_stepper_timer(saved_prescale);
}

static uint32_t u32_diff(uint32_t a, uint32_t b) {
  return (a > b) ? (a - b) : (b - a);
}

static void apply_comm_failsafe() {
  uint8_t saved_prescale = pause_stepper_timer();
  stepper.brake();
  stepper.disable();
  stepper.setCurrent(0);
  resume_stepper_timer(saved_prescale);

  for (byte i = 0; i < 4; i++) {
    digitalWrite(rele_pin[i], LOW);
  }
  reg_write_safe_defaults_atomic();

  //Отключаем TWI: onReceive/onRequest больше не будут дёргаться, мастер получает NACK,
  //регистры невозможно перетереть из аппаратного прерывания I2C параллельно нашей очистке.
  Wire.end();

  rele_state = 0;
  last_applied_mask = 0;
  set_spd = 0;
  curr_spd = 0;
  set_time = 0;
  last_set_time = 0;
  set_time_initialized = false;
  set_dir = 0;
  set_dir_initialized = false;
  stepper_state = false;
  comm_failsafe_latched = true;
}

bool is_comm_failsafe_latched(void) {
  return comm_failsafe_latched;
}

bool try_clear_comm_failsafe_latch(void) {
  //TWI был отключён в apply_comm_failsafe — мастер точно не пишет, лок не нужен. Просто чистим всё.
  reg_write_safe_defaults_atomic();
  comm_timeout_error = false;
  comm_failsafe_latched = false;
  set_time_initialized = false;
  set_dir_initialized = false;
  last_dir = 0;
  last_set_time = 0;
  last_applied_mask = 0;

  //Поднимаем TWI обратно с сохранённым slave-адресом. Wire сохраняет ранее зарегистрированные onReceive/onRequest.
  Wire.begin(I2CSTPSetup.Type);

  return true;
}

static bool process_comm_timeout_event() {
  if (!comm_timeout_error) {
    return false;
  }

  comm_timeout_error = false;
  if (!comm_failsafe_latched) {
    apply_comm_failsafe();
  }
  return true;
}

//Двусторонний обмен маской реле:
// - если в регистре появилось что-то новое относительно прошлой согласованной маски — считаем это командой мастера и применяем к HW;
// - если локальная rele_state отличается от регистра (ранее локально переключённый бит или не успевший распространиться клик из меню) — публикуем её для мастера.
//Это устраняет прежнее поведение, когда мастер затирал локальное переключение за одну итерацию loop.
static bool sync_relays_from_array() {
  byte mask_in_reg = get_rele_mask_from_array();
  if (process_comm_timeout_event()) {
    return false;
  }

  if (mask_in_reg != last_applied_mask) {
    //Мастер прислал новую команду — приводим HW и rele_state в соответствие.
    for (byte i = 0; i < 4; i++) {
      bool new_state = bit_is_set(mask_in_reg, i);
      if (bit_is_set(rele_state, i) != new_state) {
        digitalWrite(rele_pin[i], new_state);
      }
    }
    rele_state = mask_in_reg;
    last_applied_mask = mask_in_reg;
  } else if (rele_state != mask_in_reg) {
    //В регистре сейчас то же, что мастер согласовал в прошлый раз, но локально было переключение — опубликуем.
    if (acquire_register_lock()) {
      reg_write_u8_atomic(REG_RELAY_MASK, rele_state);
      release_register_lock();
      last_applied_mask = rele_state;
    }
  }

  return !process_comm_timeout_event();
}

static void update_stepper_motion(uint32_t target, uint16_t spd, byte dir) {
  stepper_state = stepper.getState();

  if (target == 0 || spd == 0) {
    if (stepper_state) {
      stop_stepper();
    }
    return;
  }

  if (!stepper_state) {
    //если в массиве указаны параметры скорости и времени и шаговик не крутит - значит пришли значения от Самовара, запустим
    start_stepper(false);
    return;
  }

  bool need_spd_change = (spd != curr_spd);
  bool need_dir_change = (dir != last_dir);
  bool need_set_target = false;
  int32_t new_target_abs = 0;

  //если режим - миксер и предыдущее оставшееся время отличается от текущего оставшегося времени больше, чем на 1 секунду в любую сторону,
  //значит это значение пришло от Самовара и нужно синхронизироваться с ним
  uint32_t remaining_time = get_stepper_time();
  if (I2CSTPSetup.Type == I2CMIXER && u32_diff(remaining_time, last_set_time) > 1UL) {
    int32_t current_pos = 0;
    stepper_snapshot_position_atomic(&current_pos, NULL);
    uint32_t offset = calc_target_from_time(remaining_time, spd);
    int64_t target_abs = (int64_t)offset + (int64_t)current_pos;
    if (target_abs < 0) {
      target_abs = 0;
    } else if ((uint64_t)target_abs > STEPPER_TARGET_LIMIT) {
      target_abs = STEPPER_TARGET_LIMIT;
    }
    new_target_abs = (int32_t)target_abs;
    need_set_target = true;
  }
  last_set_time = remaining_time;

  if (need_spd_change || need_dir_change || need_set_target) {
    uint8_t saved_prescale = pause_stepper_timer();
    if (need_dir_change) {
      stepper.brake();
      stepper.reverse(dir);
      last_dir = dir;
      stepper.enable();
    }
    if (need_spd_change || need_dir_change) {
      stepper.setMaxSpeed(spd);
      stepper.setSpeed(spd, true);
    }
    if (need_set_target) {
      stepper.setAcceleration(stepper_acceleration_from_speed(spd));
      stepper.setTarget(new_target_abs);
    }
    resume_stepper_timer(saved_prescale);

    if (need_spd_change) {
      curr_spd = spd;
      set_spd = get_speed();
#ifdef __I2CStepper_DEBUG
      Serial.print(F("On changed SSSSSetSpd = "));
      Serial.println(set_spd);
      Serial.print(F("Changed spd = "));
      Serial.println(spd);
#endif
    }
  }

  int32_t current_pos = 0;
  int32_t target_abs = 0;
  stepper_snapshot_position_atomic(&current_pos, &target_abs);
  int32_t crnt_trg = target_abs - current_pos;
  if (crnt_trg < 0) {
    crnt_trg = 0;
  }
  //синхронизируем текущее положение шаговика с массивом для передачи данных Самовару
  set_target_to_array((uint32_t)crnt_trg);
  if (crnt_trg == 0) {
    stop_stepper();
  }
}

static bool sync_stepper_motion_from_array() {
  uint32_t target = 0;
  uint16_t spd = 0;
  byte dir = 0;

  read_motion_from_array(&target, &spd, &dir);
  if (process_comm_timeout_event()) {
    return false;
  }

  update_stepper_motion(target, spd, dir);
  return !process_comm_timeout_event();
}

void isrENK() {
  encoder.tick();  // отработка в прерывании
}

void setup() {
  stepper.brake();                                    // тормозим шаговик
  stepper.disable();                                  // отключаем шаговик

  Serial.begin(115200);
  attachPCINT(digitalPinToPCINT(ENC_CLK), isrENK, CHANGE); //вешаем прерывания для обработки энкодера
  attachPCINT(digitalPinToPCINT(ENC_DT), isrENK, CHANGE);  //вешаем прерывания для обработки энкодера
  attachPCINT(digitalPinToPCINT(ENC_SW), isrENK, CHANGE);  //вешаем прерывания для обработки энкодера

  read_config();

  Wire2.begin();                                      // инициируем подключение к шине I2C в качестве мастера
  I2C2.begin(*(byte(*)[9]) & REG_Array);               // инициируем возможность чтения/записи данных по шине I2C, из/в указываемый массив
  //  stepper.setRunMode(FOLLOW_POS);
  reg_write_u8_atomic(REG_READY, 0);
  set_time_initialized = false;
  set_dir_initialized = false;
  comm_timeout_error = false;
  comm_failsafe_latched = false;
  pinMode(MIXER_PUMP_PIN, OUTPUT);                    // используем ногу для вывода
  pinMode(RELE_PIN2, OUTPUT);                         // используем ногу для вывода
  pinMode(RELE_PIN3, OUTPUT);                         // используем ногу для вывода
  pinMode(RELE_PIN4, OUTPUT);                         // используем ногу для вывода

  menu_init();                                        // инициализуерм меню экрана

  Timer1.initialize(40);                              // инициализируем таймер для упарвления шаговиком
  Timer1.attachInterrupt(stp_tick);

  if (I2CSTPSetup.Type == I2CMIXER) {
    Serial.print(F("Mixer "));
  } else if (I2CSTPSetup.Type == I2CPUMP) {
    Serial.print(F("Pump "));
  }
  Serial.println(F("ready"));
#ifdef __I2CStepper_DEBUG
  set_spd = 490;
  set_time = 2000;
  last_set_time = set_time;
#endif
}

void stp_tick(void)
{
  stepper.tick();
}

//возвращаем время или миллилитры, оставшиеся до конца работы шаговика для отображения на экране
uint32_t get_stepper_time(void) {
  if (!set_time_initialized || stepper_state) {
    set_time = get_stepper_time_from_array();
    set_time_initialized = true;
  }
  return (uint32_t)set_time;
}

//возвращаем время или миллилитры, оставшиеся до конца работы шаговика
uint32_t get_stepper_time_from_array(void) {
  uint32_t target = get_target_from_array();
  uint16_t speed = get_speed_from_array();

  if (I2CSTPSetup.Type == I2CMIXER) {
    //если время
    if (speed == 0) return 0;
    return (target + (speed / 2)) / speed;
  }

  if (I2CSTPSetup.Type == I2CPUMP) {
    //если миллилитры
    if (I2CSTPSetup.StepperStepMl == 0) return 0;
    return (uint32_t)(((uint64_t)target * 100ULL + (I2CSTPSetup.StepperStepMl / 2)) / I2CSTPSetup.StepperStepMl);
  }

  return 0;
}

//возвращаем количество шагов
uint32_t get_target_from_array(void) {
  if (!acquire_register_lock()) {
    return 0;
  }
  uint32_t value = reg_read_u32_atomic(REG_TARGET_3);
  release_register_lock();
  return value;
}

//сохраняем количество шагов шаговика в массив для обмена с Самоваром
void set_target_to_array(uint32_t target) {
  if (!acquire_register_lock()) {
    return;
  }
  reg_write_u32_atomic(REG_TARGET_3, target);
  release_register_lock();
}

//возвращаем скрость в шагах в секунду из скорости в оборотах/мин
uint16_t get_spd_stp(uint32_t spd) {
  if (I2CSTPSetup.Type == I2CMIXER) {
    return stepper_speed_steps_mixer(spd, STEPPER_STEPS);
  } else if (I2CSTPSetup.Type == I2CPUMP) {
    return stepper_speed_steps_pump(spd, I2CSTPSetup.StepperStepMl);
  } else {
    return 0;
  }
}

//минимальная user-скорость, при которой get_spd_stp() вернёт хотя бы 1 шаг/с
//(иначе старт шаговика не произойдёт, но пользователь видит «Start: On»)
uint32_t get_min_user_speed(void) {
  if (I2CSTPSetup.Type == I2CMIXER) {
    //(1 * STEPPER_STEPS + 30) / 60 ≥ 1 для STEPPER_STEPS ≥ 30, что всегда верно.
    return 1;
  }
  if (I2CSTPSetup.Type == I2CPUMP) {
    if (I2CSTPSetup.StepperStepMl == 0) {
      return 1;
    }
    //(user * step_ml + 180000) / 360000 ≥ 1 ⟺ user ≥ ceil(180000 / step_ml)
    uint32_t step_ml = I2CSTPSetup.StepperStepMl;
    return (180000UL + step_ml - 1UL) / step_ml;
  }
  return 1;
}

uint32_t get_max_user_speed(void) {
  uint32_t max_spd = 0;
  if (I2CSTPSetup.Type == I2CMIXER) {
    max_spd = (uint32_t)(65535UL * 60UL) / STEPPER_STEPS;
  } else if (I2CSTPSetup.Type == I2CPUMP) {
    if (I2CSTPSetup.StepperStepMl > 0) {
      max_spd = (uint32_t)(((uint64_t)65535ULL * 360000ULL) / I2CSTPSetup.StepperStepMl);
    }
  }

  if (max_spd > STEPPER_MAX_SPEED) {
    max_spd = STEPPER_MAX_SPEED;
  }

  return max_spd;
}

uint32_t calc_target_from_time(uint32_t time_value, uint16_t spd) {
  if (I2CSTPSetup.Type == I2CMIXER) {
    return stepper_target_from_time_mixer(time_value, spd, STEPPER_TARGET_LIMIT);
  }

  if (I2CSTPSetup.Type == I2CPUMP) {
    return stepper_target_from_time_pump(time_value, I2CSTPSetup.StepperStepMl, STEPPER_TARGET_LIMIT);
  }

  return 0;
}

//возвращаем скорость в оборотах/мин или литры в час
uint32_t get_speed(void) {
  if (I2CSTPSetup.Type == I2CMIXER) {
    //в об/мин
    if (set_spd == 0 || stepper_state) set_spd = ((float)get_speed_from_array() * 60.0f / STEPPER_STEPS + 0.5f);
  } else if (I2CSTPSetup.Type == I2CPUMP) {
    //в миллилитрах в час
    if (set_spd == 0 || stepper_state) {
      if (I2CSTPSetup.StepperStepMl == 0) {
        set_spd = 0;
      } else {
        set_spd = ((float)get_speed_from_array() * 360000.0f / I2CSTPSetup.StepperStepMl + 0.5f);
      }
    }
  } else {
    set_spd = 0;
  }

  uint32_t max_spd = get_max_user_speed();
  if (set_spd > max_spd) {
    set_spd = max_spd;
    //Синхронизируем клэмп с массивом, чтобы шаговик и мастер видели ограниченное значение
    set_speed_to_array(get_spd_stp(set_spd));
  }

  return set_spd;
}

//получаем скорость в шагах в секунду из массива
uint16_t get_speed_from_array(void) {
  if (!acquire_register_lock()) {
    return 0;
  }
  uint16_t value = reg_read_u16_atomic(REG_SPEED_MSB);
  release_register_lock();
  return value;
}

//сохраняем скорость в массив для обмена с Самоваром
void set_speed_to_array(uint16_t spd) {
  if (!acquire_register_lock()) {
    return;
  }
  reg_write_u16_atomic(REG_SPEED_MSB, spd);
  release_register_lock();
}

//получаем направление движения для отображения на экране
byte get_direction(void) {
  if (!set_dir_initialized) {
    set_dir = get_direction_from_array();
    set_dir_initialized = true;
  }
  return set_dir;
}

//получаем направление движения из массива
byte get_direction_from_array(void) {
  if (!acquire_register_lock()) {
    return set_dir;
  }
  byte value = reg_read_u8_atomic(REG_DIR);
  release_register_lock();
  return value;
}

//сохраняем направление движения в массив для обмена с Самоваром
void set_direction_to_array(byte dir) {
  if (!acquire_register_lock()) {
    return;
  }
  reg_write_u8_atomic(REG_DIR, dir);
  release_register_lock();
}

//сохраняем состояние насоса в массив для обмена с Самоваром
bool set_mixer_pump_state(bool state) {
  set_rele_state_to_array(1, state);
  return state;
}

//получаем состояние насоса из массива
byte get_mixer_pump_from_array(void) {
  return get_rele_state_from_array(1);
}

byte get_rele_mask_from_array(void) {
  if (!acquire_register_lock()) {
    return rele_state;
  }
  byte value = reg_read_u8_atomic(REG_RELAY_MASK);
  release_register_lock();
  return value;
}

//получаем состояние реле по номеру из массива
bool get_rele_state_from_array(byte r) {
  if (r < 1 || r > 4) {
    return false;
  }
  return bitRead(get_rele_mask_from_array(), r - 1);
}

//сохраняем состояние реле по номеру в массив для обмена с Самоваром
bool set_rele_state_to_array(byte r, bool s) {
  if (r < 1 || r > 4) {
    return false;
  }

  if (!acquire_register_lock()) {
    return false;
  }
  volatile byte* regs = reg_view();
  uint8_t sreg = lock_interrupts();
  byte relay_mask = regs[REG_RELAY_MASK];
  bitWrite(relay_mask, r - 1, s);
  regs[REG_RELAY_MASK] = relay_mask;
  //Обновляем только локальный бит: если мастер в этот же момент прислал другие биты, их применит sync_relays_from_array.
  bitWrite(rele_state, r - 1, s);
  unlock_interrupts(sreg);
  release_register_lock();

  digitalWrite(rele_pin[r - 1], s);
#ifdef __I2CStepper_DEBUG
  Serial.println(F("======================"));
  Serial.print(F("Toggle rele "));
  Serial.print(r);
  Serial.print(F("; set "));
  Serial.println(s);
  Serial.println(F("======================"));
#endif
  return s;
}

void read_motion_from_array(uint32_t* target, uint16_t* speed, byte* dir) {
  if (!acquire_register_lock()) {
    *speed = 0;
    *dir = set_dir;
    *target = 0;
    return;
  }
  reg_read_motion_atomic(target, speed, dir);
  release_register_lock();
}

//запускаем шаговик. Есть два варианта - через меню и через установленные значения в массиве.
void start_stepper(bool from_int) {
  uint32_t target = 0;
  uint16_t spd = 0;
  byte dir = 0;

  if (from_int) {
    spd = get_spd_stp(set_spd);
    target = calc_target_from_time(set_time, spd);
    if (target == 0) return;
    dir = set_dir;
    set_dir_initialized = true;
    set_speed_to_array(spd);
    set_direction_to_array(dir);
    set_target_to_array(target);
  } else {
    read_motion_from_array(&target, &spd, &dir);
    set_dir = dir;
    set_dir_initialized = true;
  }
  if (target > STEPPER_TARGET_LIMIT) {
    target = STEPPER_TARGET_LIMIT;
    set_target_to_array(target);
  }
  last_dir = dir;

  if (spd == 0 || target == 0) return;

#ifdef __I2CStepper_DEBUG
  Serial.println(F("======================"));
  Serial.println(F("START STP"));
  Serial.println(spd);
  Serial.println(target);
  Serial.println(F("======================"));
#endif

  //  stepper.setRunMode(FOLLOW_POS);
  uint8_t saved_prescale = pause_stepper_timer();
  stepper.setAcceleration(stepper_acceleration_from_speed(spd));

  //  stepper.setAcceleration(0);
  //  Serial.println("===No Acceleration====");

  stepper.enable();
  stepper.reverse(dir);
  stepper.setCurrent(0);
  stepper.setMaxSpeed(spd);
  stepper.setSpeed(spd, true);
  stepper.setTarget((long)target);
  curr_spd = spd;
  resume_stepper_timer(saved_prescale);

  //синхронизируем last_set_time, чтобы первая итерация loop не инициировала ложный time-sync
  last_set_time = get_stepper_time_from_array();
}

//останавливаем шаговик
void stop_stepper() {
  set_target_to_array(0);

  uint8_t saved_prescale = pause_stepper_timer();
  stepper.brake();
  stepper.disable();
  stepper.setCurrent(0);
  resume_stepper_timer(saved_prescale);

#ifdef __I2CStepper_DEBUG
  Serial.println(F("======================"));
  Serial.println(F("FINISH STP"));
  Serial.println(F("======================"));
#endif
}

//основной цикл
void loop() {
  //TRACE();
  //опрашиваем состояние энкодера и работаем с меню
  poll_menu();
  if (is_comm_failsafe_latched()) {
    process_comm_timeout_event();
    return;
  }

  if (process_comm_timeout_event()) {
    return;
  }

  if (!sync_relays_from_array()) {
    return;
  }
  sync_stepper_motion_from_array();
}

void read_config() {
  EEPROM.get(0, I2CSTPSetup);

  if (I2CSTPSetup.Type != I2CMIXER && I2CSTPSetup.Type != I2CPUMP) {
    I2CSTPSetup.Type = I2CMIXER;
    write_config();
  }

  if (I2CSTPSetup.StepperStepMl > 80000 || I2CSTPSetup.StepperStepMl < 100) {
    I2CSTPSetup.StepperStepMl = 4500;
    write_config();
  }

  Wire.begin(I2CSTPSetup.Type);                       // инициируем подключение к шине I2C в качестве ведомого (slave) устройства, с указанием своего адреса на шине.
}

void write_config() {
  EEPROM.put(0, I2CSTPSetup);
}
