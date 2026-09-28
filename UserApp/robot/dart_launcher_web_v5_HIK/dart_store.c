/*
 * dart_store.c — 掉电保存 v5: 双 Bank 交替 + 校验, 防掉电损坏
 *
 * 两个 Flash 扇区各存一份完整配置, 每次保存写到"非当前"的那一份并递增序号。
 * 上电时读两份, 校验通过且序号更大的胜出。即使保存过程中掉电, 另一份仍完好。
 *
 * 保存时机: 设定变化后去抖 300ms; 若电机在动, 最多再等 2s 强制保存。
 */
#include "dart_store.h"

#include <stdint.h>

#include "bsp_flash.h"
#include "dart_fsm.h"
#include "dart_motor.h"
#include "dart_servo.h"
#include "main.h"

#define BANK_A_ADDR ADDR_FLASH_SECTOR_10  // 0x080C0000
#define BANK_B_ADDR ADDR_FLASH_SECTOR_11  // 0x080E0000
#define CFG_MAGIC   0xD1A7B101u           // v5_HIK
#define SAVE_DEBOUNCE_MS 300u
#define SAVE_FORCE_MS    2000u

typedef struct {
  float params[MOTOR_COUNT][11];
  float zero[MOTOR_COUNT];
  uint8_t reverse[MOTOR_COUNT];
  uint8_t pad[4];
  float servo_std_deg;
  float servo_prep_deg;
  float servo_offset_deg;
  float spring_turns;
  int32_t yaw_mode;
  float yaw_aim_rpm;
} DartData_s;

typedef struct {
  uint32_t magic;
  uint32_t seq;
  uint32_t crc;
  DartData_s data;
} DartBank_s;

static volatile uint8_t s_dirty = 0;
static volatile uint8_t s_force = 0;
static volatile uint32_t s_dirty_tick = 0;
static volatile uint8_t s_saved_flag = 0;
static uint32_t s_active_addr = 0;  // 当前有效 bank
static uint32_t s_seq = 0;

void DartStoreMarkDirty(void) {
  s_dirty = 1;
  s_dirty_tick = HAL_GetTick();
}
void DartStoreMarkDirtyNow(void) {
  s_dirty = 1;
  s_force = 1;
  s_dirty_tick = HAL_GetTick();
}
int DartStoreTakeSaved(void) {
  int v = s_saved_flag;
  s_saved_flag = 0;
  return v;
}

/* FNV-1a 校验 */
static uint32_t Crc(const DartData_s* d) {
  const uint8_t* p = (const uint8_t*)d;
  uint32_t s = 0x811C9DC5u;
  for (uint32_t i = 0; i < sizeof(DartData_s); i++) {
    s ^= p[i];
    s *= 16777619u;
  }
  return s;
}

static void Fill(DartData_s* d) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    PIDInstance* sp = &Axis[i].inst->motor_controller.speed_PID;
    PIDInstance* ap = &Axis[i].inst->motor_controller.angle_PID;
    d->params[i][0] = sp->Kp;
    d->params[i][1] = sp->Ki;
    d->params[i][2] = sp->Kd;
    d->params[i][3] = sp->IntegralLimit;
    d->params[i][4] = sp->MaxOut;
    d->params[i][5] = ap->Kp;
    d->params[i][6] = ap->Ki;
    d->params[i][7] = ap->Kd;
    d->params[i][8] = ap->DeadBand;
    d->params[i][9] = ap->MaxOut;
    d->params[i][10] = Axis[i].ratio;
    d->zero[i] = Axis[i].zero;
    d->reverse[i] =
        (Axis[i].inst->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? 1 : 0;
  }
  for (int i = 0; i < 4; i++) d->pad[i] = 0;
  d->servo_std_deg = g_servo_std_deg;
  d->servo_prep_deg = g_servo_prep_deg;
  d->servo_offset_deg = g_servo_offset_deg;
  d->spring_turns = g_spring_turns;
  d->yaw_mode = g_yaw_mode;
  d->yaw_aim_rpm = g_yaw_aim_rpm;
}

static void Apply(const DartData_s* d) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    for (int id = 1; id <= 11; id++) MotorLoadParamRaw(i, id, d->params[i][id - 1]);
    MotorLoadZero(i, d->zero[i], 1);
    MotorLoadDir(i, d->reverse[i]);
  }
  g_servo_std_deg = d->servo_std_deg;
  g_servo_prep_deg = d->servo_prep_deg;
  g_servo_offset_deg = d->servo_offset_deg;
  g_spring_turns = d->spring_turns;
  g_yaw_mode = d->yaw_mode;
  g_yaw_aim_rpm = d->yaw_aim_rpm;
}

static int ReadBank(uint32_t addr, DartBank_s* b) {
  flash_read(addr, (uint32_t*)b, sizeof(DartBank_s) / 4);
  if (b->magic != CFG_MAGIC) return 0;
  if (b->crc != Crc(&b->data)) return 0;
  return 1;
}

void DartStoreInit(void) {
  DartBank_s a, b;
  int oka = ReadBank(BANK_A_ADDR, &a);
  int okb = ReadBank(BANK_B_ADDR, &b);
  DartBank_s* best = NULL;
  if (oka && okb) best = (a.seq >= b.seq) ? &a : &b;
  else if (oka) best = &a;
  else if (okb) best = &b;
  if (best == NULL) return;  // 无有效配置, 用默认
  Apply(&best->data);
  s_active_addr = (best == &a) ? BANK_A_ADDR : BANK_B_ADDR;
  s_seq = best->seq;
}

static int MotorsStopped(void) {
  for (int i = 0; i < MOTOR_COUNT; i++)
    if (Axis[i].mode != MODE_STOP) return 0;
  if (g_task_step != DART_TASK_IDLE && g_task_step != DART_TASK_DONE) return 0;
  return 1;
}

void DartStoreTask(void) {
  if (!s_dirty) return;
  uint32_t dt = HAL_GetTick() - s_dirty_tick;
  if (!s_force && dt < SAVE_DEBOUNCE_MS) return;
  if (!s_force && !MotorsStopped() && dt < SAVE_FORCE_MS) return;

  DartBank_s bank;
  bank.magic = CFG_MAGIC;
  bank.seq = s_seq + 1;
  Fill(&bank.data);
  bank.crc = Crc(&bank.data);

  uint32_t target = (s_active_addr == BANK_A_ADDR) ? BANK_B_ADDR : BANK_A_ADDR;

  __disable_irq();  // 擦写期间关中断, 防止被打断导致擦除失败
  flash_erase_address(target, 1);
  flash_write_single_address(target, (uint32_t*)&bank, sizeof(bank) / 4);
  __enable_irq();

  s_active_addr = target;
  s_seq = bank.seq;
  s_dirty = 0;
  s_force = 0;
  s_saved_flag = 1;
}
