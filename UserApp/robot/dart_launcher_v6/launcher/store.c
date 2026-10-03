/*
 * launcher/store.c — 掉电保存实现 (移植 dart_launcher_web_v5_HIK/dart_store.c)
 * 双 Bank 交替 + FNV-1a 校验; 保存时机: 变化后去抖; 电机静止优先。
 */
#include "store.h"

#include <stdint.h>

#include "bsp_flash.h"
#include "ctrl.h"
#include "main.h"
#include "motor.h"
#include "robot.h"
#include "servo.h"

#define BANK_A_ADDR ADDR_FLASH_SECTOR_10 /* 0x080C0000 */
#define BANK_B_ADDR ADDR_FLASH_SECTOR_11 /* 0x080E0000 */
#define CFG_MAGIC 0xD1A7B600u            /* v6 */
#define SAVE_DEBOUNCE_MS 300u
#define SAVE_FORCE_MS 2000u

typedef struct {
  float params[LM_COUNT][11];
  float zero[LM_COUNT];
  uint8_t reverse[LM_COUNT];
  uint8_t pad[4];
  float servo_std_deg;
  float servo_prep_deg;
  float servo_offset_deg;
  float spring_turns;
  int32_t yaw_mode;
  float yaw_aim_rpm;
} StoreData_s;

typedef struct {
  uint32_t magic;
  uint32_t seq;
  uint32_t crc;
  StoreData_s data;
} StoreBank_s;

static volatile uint8_t s_dirty = 0;
static volatile uint8_t s_force = 0;
static volatile uint32_t s_dirty_tick = 0;
static volatile uint8_t s_saved_flag = 0;
static uint32_t s_active_addr = 0;
static uint32_t s_seq = 0;

void StoreMarkDirty(void) {
  s_dirty = 1;
  s_dirty_tick = HAL_GetTick();
}
void StoreMarkDirtyNow(void) {
  s_dirty = 1;
  s_force = 1;
  s_dirty_tick = HAL_GetTick();
}
int StoreTakeSaved(void) {
  int v = s_saved_flag;
  s_saved_flag = 0;
  return v;
}

static uint32_t Crc(const StoreData_s* d) {
  const uint8_t* p = (const uint8_t*)d;
  uint32_t s = 0x811C9DC5u;
  for (uint32_t i = 0; i < sizeof(StoreData_s); i++) {
    s ^= p[i];
    s *= 16777619u;
  }
  return s;
}

static void Fill(StoreData_s* d) {
  for (int i = 0; i < LM_COUNT; i++) {
    MotorReadParams(i, d->params[i]);
    d->zero[i] = robot->motor->axis[i].zero;
    d->reverse[i] =
        (robot->motor->axis[i].inst->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            ? 1
            : 0;
  }
  for (int i = 0; i < 4; i++) d->pad[i] = 0;
  d->servo_std_deg = robot->servo->std_deg;
  d->servo_prep_deg = robot->servo->prep_deg;
  d->servo_offset_deg = robot->servo->offset_deg;
  d->spring_turns = CtrlGetSpringTurns();
  d->yaw_mode = CtrlGetYawMode();
  d->yaw_aim_rpm = CtrlGetAimRpm();
}

static void Apply(const StoreData_s* d) {
  for (int i = 0; i < LM_COUNT; i++) {
    for (int id = 1; id <= 11; id++) MotorLoadParamRaw(i, id, d->params[i][id - 1]);
    MotorLoadZero(i, d->zero[i], 1);
    MotorLoadDir(i, d->reverse[i]);
  }
  robot->servo->std_deg = d->servo_std_deg;
  robot->servo->prep_deg = d->servo_prep_deg;
  robot->servo->offset_deg = d->servo_offset_deg;
  CtrlSetSpringTurns(d->spring_turns);
  CtrlSetYawMode((uint8_t)d->yaw_mode);
  CtrlSetAimRpm(d->yaw_aim_rpm);
}

static int ReadBank(uint32_t addr, StoreBank_s* b) {
  flash_read(addr, (uint32_t*)b, sizeof(StoreBank_s) / 4);
  if (b->magic != CFG_MAGIC) return 0;
  if (b->crc != Crc(&b->data)) return 0;
  return 1;
}

void StoreInit(void) {
  StoreBank_s a, b;
  int oka = ReadBank(BANK_A_ADDR, &a);
  int okb = ReadBank(BANK_B_ADDR, &b);
  StoreBank_s* best = NULL;
  if (oka && okb) best = (a.seq >= b.seq) ? &a : &b;
  else if (oka) best = &a;
  else if (okb) best = &b;
  if (best == NULL) return; /* 无有效配置 -> 用默认 */
  Apply(&best->data);
  s_active_addr = (best == &a) ? BANK_A_ADDR : BANK_B_ADDR;
  s_seq = best->seq;
}

static int MotorsStopped(void) {
  for (int i = 0; i < LM_COUNT; i++)
    if (robot->motor->axis[i].mode != LMODE_STOP) return 0;
  uint8_t step = CtrlGetTaskStep();
  if (step != LAUNCH_TASK_IDLE && step != LAUNCH_TASK_DONE) return 0;
  return 1;
}

void StoreTask(void) {
  if (!s_dirty) return;
  uint32_t dt = HAL_GetTick() - s_dirty_tick;
  if (!s_force && dt < SAVE_DEBOUNCE_MS) return;
  if (!s_force && !MotorsStopped() && dt < SAVE_FORCE_MS) return;

  StoreBank_s bank;
  bank.magic = CFG_MAGIC;
  bank.seq = s_seq + 1;
  Fill(&bank.data);
  bank.crc = Crc(&bank.data);

  uint32_t target = (s_active_addr == BANK_A_ADDR) ? BANK_B_ADDR : BANK_A_ADDR;

  __disable_irq();
  flash_erase_address(target, 1);
  flash_write_single_address(target, (uint32_t*)&bank, sizeof(bank) / 4);
  __enable_irq();

  s_active_addr = target;
  s_seq = bank.seq;
  s_dirty = 0;
  s_force = 0;
  s_saved_flag = 1;
}
