/*
 * dart_cfg.c — 舵面标定掉电保存 (双 Bank 交替 + 序号 + CRC)
 *
 * 两个 Flash 扇区各存一份完整配置:
 *   保存时写到"非当前"的那一份并 seq+1, 写后回读校验, 通过才切换 active;
 *   上电时读两份, 校验通过且 seq 更大者胜出。
 * 这样即使保存过程中掉电, 另一份仍完好; 单扇区写失败也不会丢配置。
 */
#include "dart_cfg.h"

#include "bsp_flash.h"
#include "dart_axis.h"
#include "main.h"
#include "robot_config.h"

typedef struct {
  uint8_t reversed[DART_AXIS_N];
  uint8_t pad[4];
  float trim[DART_AXIS_N];
  float scale[DART_AXIS_N];
} DartCfgData_t;

typedef struct {
  uint32_t magic;
  uint32_t seq;
  uint32_t crc;
  DartCfgData_t data;
} DartCfgBlob_t;

static volatile uint8_t s_req;        /* 显式立即保存请求 */
static volatile uint8_t s_dirty;      /* 标定已改动, 待自动保存 */
static volatile uint32_t s_dirty_tick;
static volatile uint8_t s_result;
static uint32_t s_active_addr;        /* 当前有效 bank 地址, 0=无 */
static uint32_t s_seq;

static uint32_t Crc(const DartCfgData_t *d) {
  const uint8_t *p = (const uint8_t *)d;
  uint32_t s = 0x811C9DC5u, i;
  for (i = 0; i < sizeof(DartCfgData_t); i++) {
    s ^= p[i];
    s *= 16777619u;
  }
  return s;
}

static void Fill(DartCfgData_t *d) {
  uint8_t i;
  for (i = 0; i < 4; i++) d->pad[i] = 0;
  for (i = 0; i < DART_AXIS_N; i++) {
    d->reversed[i] = DartAxisGetDir(i);
    d->trim[i] = DartAxisGetTrim(i);
    d->scale[i] = DartAxisGetScale(i);
  }
}

static void Apply(const DartCfgData_t *d) {
  uint8_t i;
  for (i = 0; i < DART_AXIS_N; i++) {
    DartAxisSetDir(i, d->reversed[i]);
    DartAxisSetTrim(i, d->trim[i]);
    DartAxisSetScale(i, d->scale[i]);
  }
}

static int ReadBank(uint32_t addr, DartCfgBlob_t *b) {
  flash_read(addr, (uint32_t *)b, sizeof(DartCfgBlob_t) / 4);
  if (b->magic != DART_CFG_MAGIC) return 0;
  if (b->crc != Crc(&b->data)) return 0;
  return 1;
}

void DartCfgInit(void) {
  DartCfgBlob_t a, b;
  int oka = ReadBank(DART_CFG_ADDR_A, &a);
  int okb = ReadBank(DART_CFG_ADDR_B, &b);
  DartCfgBlob_t *best = NULL;

  if (oka && okb) best = (a.seq >= b.seq) ? &a : &b;
  else if (oka) best = &a;
  else if (okb) best = &b;
  if (best == NULL) return; /* 无有效配置, 用默认 */

  Apply(&best->data);
  s_seq = best->seq;
  s_active_addr = (best == &a) ? DART_CFG_ADDR_A : DART_CFG_ADDR_B;
}

void DartCfgRequestSave(void) {
  s_req = 1u;
  s_dirty = 1u;
}

void DartCfgMarkDirty(void) {
  s_dirty = 1u;
  s_dirty_tick = HAL_GetTick();
}

uint8_t DartCfgTakeResult(void) {
  uint8_t r = s_result;
  s_result = 0u;
  return r;
}

/* 擦写一个 bank 并回读校验 (擦写期间关中断, 128KB sector 约 1s) */
static int WriteBank(uint32_t addr, const DartCfgBlob_t *b) {
  DartCfgBlob_t chk;
  __disable_irq();
  flash_erase_address(addr, 1);
  flash_write_single_address(addr, (uint32_t *)b, sizeof(DartCfgBlob_t) / 4);
  __enable_irq();
  flash_read(addr, (uint32_t *)&chk, sizeof(chk) / 4);
  return (chk.magic == DART_CFG_MAGIC && chk.crc == Crc(&chk.data));
}

void DartCfgTask(void) {
  DartCfgBlob_t b;
  uint32_t target, other;
  uint8_t do_save = 0u;

  if (s_req) {
    s_req = 0u;
    s_dirty = 0u;
    do_save = 1u;
  } else if (s_dirty &&
             (HAL_GetTick() - s_dirty_tick) >= DART_CFG_SAVE_DEBOUNCE_MS) {
    s_dirty = 0u;
    do_save = 1u;
  }
  if (!do_save) return;

  b.magic = DART_CFG_MAGIC;
  b.seq = s_seq + 1u;
  Fill(&b.data);
  b.crc = Crc(&b.data);

  /* 优先写"非当前"的 bank; 首次(无 active)写 A */
  target = (s_active_addr == DART_CFG_ADDR_A) ? DART_CFG_ADDR_B : DART_CFG_ADDR_A;
  other = (target == DART_CFG_ADDR_A) ? DART_CFG_ADDR_B : DART_CFG_ADDR_A;

  if (WriteBank(target, &b)) {
    s_active_addr = target;
    s_seq = b.seq;
    s_result = 1u;
    return;
  }
  /* 目标 bank 失败则回退另一个 (单扇区坏也不丢配置) */
  if (WriteBank(other, &b)) {
    s_active_addr = other;
    s_seq = b.seq;
    s_result = 1u;
    return;
  }
  s_result = 2u;
}
