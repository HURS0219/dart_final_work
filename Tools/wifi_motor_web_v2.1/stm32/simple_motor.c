/*
 * simple_motor.c — 最简通用 DJI 电机多机 CAN 总线实现 (GM6020 / M3508 / M2006)
 */
#include "simple_motor.h"

#include "can.h"  // hcan1
#include "main.h"

SimpleMotor_t g_motors[SIMPLE_MOTOR_MAX];
uint8_t g_motor_num = 0;

static uint16_t s_scan_ids[8];
static uint8_t s_scan_cnt = 0;

static uint16_t FeedbackId(SimpleMotorType_e type, uint8_t id) {
  return (type == SIMPLE_MOTOR_GM6020) ? (uint16_t)(0x204 + id) : (uint16_t)(0x200 + id);
}

static int16_t ClampValue(SimpleMotorType_e type, int32_t v) {
  int32_t lim;
  switch (type) {
    case SIMPLE_MOTOR_GM6020: lim = 30000; break;
    case SIMPLE_MOTOR_M3508: lim = 16384; break;
    case SIMPLE_MOTOR_M2006: lim = 10000; break;
    default: lim = 30000; break;
  }
  if (v > lim) v = lim;
  else if (v < -lim) v = -lim;
  return (int16_t)v;
}

/* 关闭所有滤波 bank, 再按当前电机列表配置各自的反馈 ID */
static void ConfigureFilters(void) {
  CAN_FilterTypeDef f = {0};
  f.FilterMode = CAN_FILTERMODE_IDLIST;
  f.FilterScale = CAN_FILTERSCALE_16BIT;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = DISABLE;
  f.SlaveStartFilterBank = 14;
  for (uint8_t b = 0; b < 14; b++) {
    f.FilterBank = b;
    HAL_CAN_ConfigFilter(&hcan1, &f);
  }

  uint8_t bank = 0;
  for (int i = 0; i < SIMPLE_MOTOR_MAX; i++) {
    if (!g_motors[i].used) continue;
    CAN_FilterTypeDef mf = {0};
    mf.FilterBank = bank++;
    mf.FilterMode = CAN_FILTERMODE_IDLIST;
    mf.FilterScale = CAN_FILTERSCALE_16BIT;
    mf.FilterIdHigh = (uint16_t)(FeedbackId(g_motors[i].type, g_motors[i].id) << 5);
    mf.FilterFIFOAssignment = CAN_RX_FIFO0;
    mf.FilterActivation = ENABLE;
    mf.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &mf);
  }
}

void SimpleMotorBusInit(void) {
  for (int i = 0; i < SIMPLE_MOTOR_MAX; i++) g_motors[i].used = 0;
  g_motor_num = 0;
  HAL_CAN_Start(&hcan1);
  ConfigureFilters();
}

void SimpleMotorClear(void) {
  for (int i = 0; i < SIMPLE_MOTOR_MAX; i++) g_motors[i].used = 0;
  g_motor_num = 0;
  ConfigureFilters();
}

int SimpleMotorAdd(SimpleMotorType_e type, uint8_t id) {
  if (id < 1 || id > 8) return -1;

  int idx = -1;
  for (int i = 0; i < SIMPLE_MOTOR_MAX; i++)
    if (!g_motors[i].used) { idx = i; break; }
  if (idx < 0) return -1;

  SimpleMotor_t* m = &g_motors[idx];
  m->type = type;
  m->id = id;
  m->used = 1;
  m->ecd = 0;
  m->rpm = 0;
  m->current = 0;
  m->temp = 0;
  m->fb_count = 0;
  m->total_angle = 0.0f;
  m->round = 0;
  m->inited = 0;
  m->out = 0;
  g_motor_num++;
  ConfigureFilters();
  return idx;
}

void SimpleMotorSet(int idx, int16_t value) {
  if (idx < 0 || idx >= SIMPLE_MOTOR_MAX || !g_motors[idx].used) return;
  g_motors[idx].out = ClampValue(g_motors[idx].type, value);
}

void SimpleMotorReadAll(void) {
  CAN_RxHeaderTypeDef rx;
  uint8_t buf[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, buf) != HAL_OK) break;
    for (int i = 0; i < SIMPLE_MOTOR_MAX; i++) {
      SimpleMotor_t* m = &g_motors[i];
      if (!m->used) continue;
      if (rx.StdId != FeedbackId(m->type, m->id)) continue;

      uint16_t ecd = (uint16_t)((buf[0] << 8) | buf[1]);
      if (!m->inited) {
        m->inited = 1;
      } else {
        int32_t diff = (int32_t)ecd - (int32_t)m->ecd;
        if (diff > 4096) m->round--;
        else if (diff < -4096) m->round++;
      }
      m->ecd = ecd;
      m->total_angle = (float)m->round * 360.0f + (float)ecd * (360.0f / 8192.0f);
      m->rpm = (int16_t)((buf[2] << 8) | buf[3]);
      m->current = (int16_t)((buf[4] << 8) | buf[5]);
      m->temp = buf[6];
      m->fb_count++;
      break;
    }
  }
}

void SimpleMotorSendAll(void) {
  uint8_t data[3][8] = {{0}};
  uint8_t active[3] = {0};

  for (int i = 0; i < SIMPLE_MOTOR_MAX; i++) {
    SimpleMotor_t* m = &g_motors[i];
    if (!m->used) continue;
    int frame;
    uint8_t slot;
    if (m->type == SIMPLE_MOTOR_GM6020) {
      if (m->id <= 4) { frame = 1; slot = (uint8_t)(m->id - 1); }
      else { frame = 2; slot = (uint8_t)(m->id - 5); }
    } else {
      if (m->id <= 4) { frame = 0; slot = (uint8_t)(m->id - 1); }
      else { frame = 1; slot = (uint8_t)(m->id - 5); }
    }
    int16_t v = m->out;
    data[frame][slot * 2] = (uint8_t)(v >> 8);
    data[frame][slot * 2 + 1] = (uint8_t)(v & 0xFF);
    active[frame] = 1;
  }

  static const uint16_t ids[3] = {0x200, 0x1FF, 0x2FF};
  for (int f = 0; f < 3; f++) {
    if (!active[f]) continue;
    CAN_TxHeaderTypeDef tx = {0};
    tx.StdId = ids[f];
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    tx.TransmitGlobalTime = DISABLE;
    uint32_t t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
      if (HAL_GetTick() - t0 > 5) break;
    }
    uint32_t mailbox = 0;
    HAL_CAN_AddTxMessage(&hcan1, &tx, data[f], &mailbox);
  }
}

/* ---------------- 扫描 ---------------- */
void SimpleMotorScanBegin(void) {
  s_scan_cnt = 0;
  CAN_FilterTypeDef f = {0};
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh = 0;
  f.FilterIdLow = 0;
  f.FilterMaskIdHigh = 0;
  f.FilterMaskIdLow = 0;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  f.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &f);
}

void SimpleMotorScanTask(void) {
  CAN_RxHeaderTypeDef rx;
  uint8_t buf[8];
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, buf) != HAL_OK) break;
    if (rx.StdId >= 0x201 && rx.StdId <= 0x20B) {
      uint8_t found = 0;
      for (uint8_t i = 0; i < s_scan_cnt; i++)
        if (s_scan_ids[i] == rx.StdId) { found = 1; break; }
      if (!found && s_scan_cnt < 8) s_scan_ids[s_scan_cnt++] = rx.StdId;
    }
  }
}

uint8_t SimpleMotorScanResult(uint16_t* out, uint8_t max) {
  uint8_t n = (s_scan_cnt < max) ? s_scan_cnt : max;
  for (uint8_t i = 0; i < n; i++) out[i] = s_scan_ids[i];
  return n;
}

void SimpleMotorReconfigFilters(void) { ConfigureFilters(); }
