/**
 * @file bsp_flash.c
 * @author neozng / ai
 * @brief 内部 Flash 读写封装 + 通用掉电保存(双 Bank)
 * @note  详细说明见 bsp_flash.md
 */
#include "bsp_flash.h"

#include "main.h"
#include "string.h"

#if defined(STM32H723xx) || defined(STM32H743xx)
#define FLASH_TYPEPROGRAM_WORD FLASH_TYPEPROGRAM_FLASHWORD
#endif

static uint32_t ger_sector(uint32_t address);

/* 清除悬挂的 Flash 错误标志: 若有残留的 WRPERR/PGSERR 等, 擦除会直接静默失败 */
static void FlashClearErrors(void) {
#if defined(STM32F407xx)
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32H723xx) || defined(STM32H743xx)
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
#endif
}

/**
 * @brief          擦除 Flash(按扇区)
 * @param[in]      address: Flash 地址
 * @param[in]      len: 擦除的扇区数
 * @retval         none
 * @note           擦除前先清错误标志, 否则可能静默失败; H7 需按地址选择 Bank。
 */
void flash_erase_address(uint32_t address, uint16_t len) {
  FLASH_EraseInitTypeDef flash_erase;
  uint32_t error = 0xFFFFFFFFu;

  flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  flash_erase.NbSectors = len;
  flash_erase.Sector = ger_sector(address);
  flash_erase.Banks = 0u;
#if defined(STM32H723xx) || defined(STM32H743xx)
  flash_erase.Banks = (address >= ADDR_FLASH_SECTOR_12) ? FLASH_BANK_2 : FLASH_BANK_1;
#endif

  HAL_FLASH_Unlock();
  FlashClearErrors();  // 必须先清错误标志, 否则擦除静默失败
  HAL_FLASHEx_Erase(&flash_erase, &error);
  HAL_FLASH_Lock();
}

/**
 * @brief          向一页 Flash 写入数据
 * @param[in]      start_address: 起始地址
 * @param[in]      buf: 数据指针
 * @param[in]      len: 数据长度(以 32-bit word 计)
 * @retval         success 0, fail -1
 */
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len) {
  static uint32_t uw_address;
  static uint32_t end_address;
  static uint32_t *data_buf;
  static uint32_t data_len;

  HAL_FLASH_Unlock();

  uw_address = start_address;
  end_address = get_next_flash_address(start_address);
  data_buf = buf;
  data_len = 0;

  while (uw_address <= end_address) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, uw_address, *data_buf) == HAL_OK) {
      uw_address += 4;
      data_buf++;
      data_len++;
      if (data_len == len) {
        break;
      }
    } else {
      HAL_FLASH_Lock();
      return -1;
    }
  }

  HAL_FLASH_Lock();
  return 0;
}

/**
 * @brief          向多页 Flash 写入数据
 * @param[in]      start_address: 起始地址
 * @param[in]      end_address: 结束地址
 * @param[in]      buf: 数据指针
 * @param[in]      len: 数据长度(以 32-bit word 计)
 * @retval         success 0, fail -1
 */
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len) {
  uint32_t uw_address = 0;
  uint32_t *data_buf;
  uint32_t data_len;

  HAL_FLASH_Unlock();

  uw_address = start_address;
  data_buf = buf;
  data_len = 0;
  while (uw_address <= end_address) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, uw_address, *data_buf) == HAL_OK) {
      uw_address += 4;
      data_buf++;
      data_len++;
      if (data_len == len) {
        break;
      }
    } else {
      HAL_FLASH_Lock();
      return -1;
    }
  }

  HAL_FLASH_Lock();
  return 0;
}

/**
 * @brief          从 Flash 读取数据
 * @param[in]      address: Flash 地址
 * @param[out]     buf: 数据指针
 * @param[in]      len: 数据长度(以 32-bit word 计)
 * @retval         none
 */
void flash_read(uint32_t address, uint32_t *buf, uint32_t len) { memcpy(buf, (void *)address, len * 4); }

/**
 * @brief          获取 Flash 地址所在的扇区号
 * @param[in]      address: Flash 地址
 * @retval         sector 号
 * @note           F407 扇区大小不等(0~3 为 16KB, 4 为 64KB, 5~11 为 128KB);
 *                 H7 每扇区 128KB, 按 bank 内偏移返回扇区号(配合 flash_erase_address 的 Banks)。
 */
static uint32_t ger_sector(uint32_t address) {
  uint32_t sector = 0;
#if defined(STM32H723xx) || defined(STM32H743xx)
  /* H7: 每扇区 128KB; bank1 从 0x08000000, bank2 从 0x08100000; 返回 bank 内扇区号 */
  {
    uint32_t base = (address >= ADDR_FLASH_SECTOR_12) ? ADDR_FLASH_SECTOR_12 : ADDR_FLASH_SECTOR_0;
    return (address - base) / 0x20000u;
  }
#endif
  if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0)) {
    sector = FLASH_SECTOR_0;
  } else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1)) {
    sector = FLASH_SECTOR_1;
  } else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2)) {
    sector = FLASH_SECTOR_2;
  } else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3)) {
    sector = FLASH_SECTOR_3;
  } else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4)) {
    sector = FLASH_SECTOR_4;
  } else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5)) {
    sector = FLASH_SECTOR_5;
  } else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6)) {
    sector = FLASH_SECTOR_6;
  } else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7)) {
    sector = FLASH_SECTOR_7;
  }
#ifdef STM32F407xx
  else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8)) {
    sector = FLASH_SECTOR_8;
  } else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9)) {
    sector = FLASH_SECTOR_9;
  } else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10)) {
    sector = FLASH_SECTOR_10;
  } else if ((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11)) {
    sector = FLASH_SECTOR_11;
  } else {
    sector = FLASH_SECTOR_11;
  }
#endif

  return sector;
}

/**
 * @brief          获取下一个扇区的起始地址
 * @param[in]      address: Flash 地址
 * @retval         下一扇区起始地址
 */
uint32_t get_next_flash_address(uint32_t address) {
  uint32_t sector = 0;

  if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0)) {
    sector = ADDR_FLASH_SECTOR_1;
  } else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1)) {
    sector = ADDR_FLASH_SECTOR_2;
  } else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2)) {
    sector = ADDR_FLASH_SECTOR_3;
  } else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3)) {
    sector = ADDR_FLASH_SECTOR_4;
  } else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4)) {
    sector = ADDR_FLASH_SECTOR_5;
  } else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5)) {
    sector = ADDR_FLASH_SECTOR_6;
  } else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6)) {
    sector = ADDR_FLASH_SECTOR_7;
  } else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7)) {
    sector = ADDR_FLASH_SECTOR_8;
  } else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8)) {
    sector = ADDR_FLASH_SECTOR_9;
  } else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9)) {
    sector = ADDR_FLASH_SECTOR_10;
  } else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10)) {
    sector = ADDR_FLASH_SECTOR_11;
  } else /*(address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_23))*/
  {
    sector = FLASH_END_ADDR;
  }
  return sector;
}

/* ============================== 通用掉电保存 (双 Bank) ============================== */
/*
 * 目的: 给任意模块/应用提供"写参数 -> 掉电不丢"的能力, 并避免"单扇区写坏 / 写一半掉电"
 *       导致配置全丢。
 *
 * 记录布局(每个 bank 都从扇区起始地址开始):
 *   +0x00  magic (0xB1A5D00D)  -- 记录标记, 不匹配则视为该 bank 无效
 *   +0x04  seq   (递增序号)     -- 读取时 seq 更大的 bank 胜出
 *   +0x08  crc   (FNV-1a)       -- 对 data 区校验
 *   +0x0C  data[]               -- 调用方数据, 长度 size(必须 4 字节对齐)
 *
 * 保存策略(双 Bank 交替): 总是写"非当前"那个 bank, seq = 当前seq+1; 写完回读校验,
 *       通过才算成功; 若目标 bank 写失败则回退写另一个 bank。
 * 读取策略: 两份都校验, 取 seq 更大的那一份。
 */

#define FLASH_STORE_MAGIC 0xB1A5D00Du /* 记录记号; 记录布局变化时修改, 使旧数据失效 */
#define FLASH_STORE_HDR_WORDS 3u      /* 头长度: magic / seq / crc 共 3 个 word(12B) */

/* 记录头(4 字节对齐、无隐式 padding) */
typedef struct {
  uint32_t magic; /* 记录标记 */
  uint32_t seq;   /* 递增序号 */
  uint32_t crc;   /* data 区 FNV-1a 校验 */
} FlashStoreHeader_s;

/**
 * @brief 计算 RAM 数据的 FNV-1a 校验
 * @param p    数据首地址
 * @param size 字节数
 */
static uint32_t FlashStoreCrcRam(const uint8_t *p, uint32_t size) {
  uint32_t s = 0x811C9DC5u, i;
  for (i = 0; i < size; i++) {
    s ^= p[i];
    s *= 16777619u;
  }
  return s;
}

/**
 * @brief 计算 Flash 中数据的 FNV-1a 校验
 * @note  分块读取(64B)累加, 避免申请与 size 等大的临时缓冲
 */
static uint32_t FlashStoreCrcAt(uint32_t addr, uint32_t size) {
  uint8_t buf[64];
  uint32_t s = 0x811C9DC5u, i;
  while (size > 0u) {
    uint32_t n = (size > sizeof(buf)) ? (uint32_t)sizeof(buf) : size;
    flash_read(addr, (uint32_t *)buf, n / 4u);
    for (i = 0; i < n; i++) {
      s ^= buf[i];
      s *= 16777619u;
    }
    addr += n;
    size -= n;
  }
  return s;
}

/**
 * @brief 校验一个 bank 是否有效
 * @param  addr 扇区起始地址; size 数据字节数; seq 可选回传序号
 * @return 1 有效(magic 匹配且 data CRC 一致), 0 无效
 */
static int8_t FlashStoreValid(uint32_t addr, uint32_t size, uint32_t *seq) {
  FlashStoreHeader_s h;
  flash_read(addr, (uint32_t *)&h, FLASH_STORE_HDR_WORDS);
  if (h.magic != FLASH_STORE_MAGIC) return 0;
  if (h.crc != FlashStoreCrcAt(addr + FLASH_STORE_HDR_WORDS * 4u, size)) return 0;
  if (seq != NULL) *seq = h.seq;
  return 1;
}

/**
 * @brief 擦除并写入一个 bank 的完整记录, 然后回读校验
 * @param  addr 扇区起始地址; seq 序号; data/size 待写数据
 * @return 0 成功 / -1 失败
 */
static int8_t FlashStoreWriteBank(uint32_t addr, uint32_t seq, const void *data, uint32_t size) {
  FlashStoreHeader_s h;
  h.magic = FLASH_STORE_MAGIC;
  h.seq = seq;
  h.crc = FlashStoreCrcRam((const uint8_t *)data, size);
  flash_erase_address(addr, 1);                                                    // 擦整扇区
  flash_write_single_address(addr, (uint32_t *)&h, FLASH_STORE_HDR_WORDS);          // 写头
  flash_write_single_address(addr + FLASH_STORE_HDR_WORDS * 4u, (uint32_t *)data,   // 写数据
                             size / 4u);
  return FlashStoreValid(addr, size, NULL) ? 0 : -1;  // 回读校验
}

/**
 * @brief 保存数据(双 Bank 交替 + 回读校验 + 失败回退)
 * @param  bank_a/bank_b 两个备份扇区起始地址; data/size 待保存数据(4 字节对齐)
 * @return 0 成功 / -1 失败
 * @note   优先写"非当前"bank(首次无有效数据则写 A); 写失败自动回退写另一个 bank。
 */
int8_t flash_store_save(uint32_t bank_a, uint32_t bank_b, const void *data, uint32_t size) {
  uint32_t sa = 0u, sb = 0u, seq, active, target, other;
  int oka, okb;

  if (data == NULL || size == 0u || (size & 3u) != 0u) return -1;  // 参数非法

  /* 判定当前有效 bank 与序号 */
  oka = FlashStoreValid(bank_a, size, &sa);
  okb = FlashStoreValid(bank_b, size, &sb);
  if (oka && okb) {
    active = (sa >= sb) ? bank_a : bank_b;
    seq = (sa >= sb) ? sa : sb;
  } else if (oka) {
    active = bank_a;
    seq = sa;
  } else if (okb) {
    active = bank_b;
    seq = sb;
  } else {
    active = 0u; /* 首次: active 置非 A, 使 target 落到 A */
    seq = 0u;
  }

  target = (active == bank_a) ? bank_b : bank_a; /* 优先写非当前 bank */
  other = (target == bank_a) ? bank_b : bank_a;

  if (FlashStoreWriteBank(target, seq + 1u, data, size) == 0) return 0;
  if (FlashStoreWriteBank(other, seq + 1u, data, size) == 0) return 0; /* 回退另一个 bank */
  return -1;
}

/**
 * @brief 读取数据(取两份中 seq 更大的有效 bank)
 * @param  bank_a/bank_b 两个备份扇区起始地址; data 输出缓冲; size 数据字节数(4 字节对齐)
 * @return 1 读到有效数据 / 0 无有效数据(调用方保持默认值即可)
 */
int8_t flash_store_load(uint32_t bank_a, uint32_t bank_b, void *data, uint32_t size) {
  uint32_t sa = 0u, sb = 0u, src;
  int oka, okb;

  if (data == NULL || size == 0u || (size & 3u) != 0u) return 0;

  oka = FlashStoreValid(bank_a, size, &sa);
  okb = FlashStoreValid(bank_b, size, &sb);
  if (!oka && !okb) return 0; /* 两份都无效 */

  if (oka && okb) src = (sa >= sb) ? bank_a : bank_b; /* 取 seq 更大者 */
  else src = oka ? bank_a : bank_b;

  flash_read(src + FLASH_STORE_HDR_WORDS * 4u, (uint32_t *)data, size / 4u);  // 跳过记录头
  return 1;
}
