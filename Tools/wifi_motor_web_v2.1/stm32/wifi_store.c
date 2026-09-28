/*
 * wifi_store.c — 掉电保存 (STM32F407 内部 Flash, sector 11 @0x080E0000)
 *
 * 保存: 电机列表(类型/ID/零点/各自参数) + 当前选中电机。
 */
#include "wifi_store.h"

#include <stdint.h>

#include "bsp_flash.h"
#include "wifi_motor.h"

#define CFG_ADDR  ADDR_FLASH_SECTOR_11
#define CFG_MAGIC 0xA5A55A60u  // 结构变更后递增

typedef struct {
  int32_t type;
  int32_t id;
  float zero;
  MotorParamSet_s param;
} MotorEntry_s;

typedef struct {
  uint32_t magic;
  int32_t sel;
  int32_t count;
  MotorEntry_s motors[SIMPLE_MOTOR_MAX];
} WifiCfg_s;

static volatile uint8_t s_dirty = 0;

void WifiStoreInit(void) {
  WifiCfg_s cfg;
  flash_read(CFG_ADDR, (uint32_t*)&cfg, sizeof(cfg) / 4);
  if (cfg.magic != CFG_MAGIC) return;  // 无效: 用默认(空列表)

  for (int i = 0; i < cfg.count && i < SIMPLE_MOTOR_MAX; i++) {
    int idx = SimpleMotorAdd((SimpleMotorType_e)cfg.motors[i].type, (uint8_t)cfg.motors[i].id);
    if (idx >= 0) {
      g_motors[idx].zero = cfg.motors[i].zero;
      g_motor_params[idx] = cfg.motors[i].param;
    }
  }
  if (cfg.sel >= 0 && cfg.sel < SIMPLE_MOTOR_MAX && g_motors[cfg.sel].used) g_sel = cfg.sel;
  else g_sel = (g_motor_num > 0) ? 0 : -1;
}

void WifiStoreMarkDirty(void) { s_dirty = 1; }

void WifiStoreTask(void) {
  if (!s_dirty) return;
  if (wifi_motor.mode != WIFI_MODE_STOP) return;  // 只在停止时写

  WifiMotorStoreActive();

  WifiCfg_s cfg;
  cfg.magic = CFG_MAGIC;
  cfg.sel = g_sel;
  cfg.count = 0;
  for (int i = 0; i < SIMPLE_MOTOR_MAX; i++) {
    if (!g_motors[i].used) continue;
    cfg.motors[cfg.count].type = (int32_t)g_motors[i].type;
    cfg.motors[cfg.count].id = (int32_t)g_motors[i].id;
    cfg.motors[cfg.count].zero = g_motors[i].zero;
    cfg.motors[cfg.count].param = g_motor_params[i];
    cfg.count++;
  }

  flash_erase_address(CFG_ADDR, 1);
  flash_write_single_address(CFG_ADDR, (uint32_t*)&cfg, sizeof(cfg) / 4);
  s_dirty = 0;
}
