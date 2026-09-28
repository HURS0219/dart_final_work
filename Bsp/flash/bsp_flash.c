#include "bsp_flash.h"

#include "main.h"
#include "string.h"

#ifdef STM32H723xx
#define FLASH_TYPEPROGRAM_WORD FLASH_TYPEPROGRAM_FLASHWORD
#endif

static uint32_t ger_sector(uint32_t address);

/**
 * @brief          erase flash
 * @param[in]      address: flash address
 * @param[in]      len: page num
 * @retval         none
 */
/**
 * @brief          ����flash
 * @param[in]      address: flash ��ַ
 * @param[in]      len: ҳ����
 * @retval         none
 */
/* 清除悬挂的 Flash 错误标志: 若残留 WRPERR/PGSERR 等, 擦除会直接静默失败 */
static void FlashClearErrors(void) {
#if defined(STM32F407xx)
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32H723xx)
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
#endif
}

void flash_erase_address(uint32_t address, uint16_t len) {
  FLASH_EraseInitTypeDef flash_erase;
  uint32_t error = 0xFFFFFFFFu;

  flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  flash_erase.NbSectors = len;
  flash_erase.Sector = ger_sector(address);
  flash_erase.Banks = 0u;
#ifdef STM32H723xx
  flash_erase.Banks = (address >= ADDR_FLASH_SECTOR_12) ? FLASH_BANK_2 : FLASH_BANK_1;
#endif

  HAL_FLASH_Unlock();
  FlashClearErrors();  // 必须先清错误标志, 否则擦除静默失败
  HAL_FLASHEx_Erase(&flash_erase, &error);
  HAL_FLASH_Lock();
}

/**
 * @brief          write data to one page of flash
 * @param[in]      start_address: flash address
 * @param[in]      buf: data point
 * @param[in]      len: data num
 * @retval         success 0, fail -1
 */
/**
 * @brief          ��һҳflashд����
 * @param[in]      start_address: flash ��ַ
 * @param[in]      buf: ����ָ��
 * @param[in]      len: ���ݳ���
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
 * @brief          write data to some pages of flash
 * @param[in]      start_address: flash start address
 * @param[in]      end_address: flash end address
 * @param[in]      buf: data point
 * @param[in]      len: data num
 * @retval         success 0, fail -1
 */
/**
 * @brief          ����ҳflashд����
 * @param[in]      start_address: flash ��ʼ��ַ
 * @param[in]      end_address: flash ������ַ
 * @param[in]      buf: ����ָ��
 * @param[in]      len: ���ݳ���
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
 * @brief          read data for flash
 * @param[in]      address: flash address
 * @param[out]     buf: data point
 * @param[in]      len: data num
 * @retval         none
 */
/**
 * @brief          ��flash������
 * @param[in]      start_address: flash ��ַ
 * @param[out]     buf: ����ָ��
 * @param[in]      len: ���ݳ���
 * @retval         none
 */
void flash_read(uint32_t address, uint32_t *buf, uint32_t len) { memcpy(buf, (void *)address, len * 4); }

/**
 * @brief          get the sector number of flash
 * @param[in]      address: flash address
 * @retval         sector number
 */
/**
 * @brief          ��ȡflash��sector��
 * @param[in]      address: flash ��ַ
 * @retval         sector��
 */
static uint32_t ger_sector(uint32_t address) {
  uint32_t sector = 0;
#ifdef STM32H723xx
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
 * @brief          get the next page flash address
 * @param[in]      address: flash address
 * @retval         next page flash address
 */
/**
 * @brief          ��ȡ��һҳflash��ַ
 * @param[in]      address: flash ��ַ
 * @retval         ��һҳflash��ַ
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
/* 单份记录 = {magic, seq, crc} 头 + data; 头固定 3 word(12B), data 紧随其后。 */

#define FLASH_STORE_MAGIC 0xB1A5D00Du /* 记录记号; 记录布局变化时修改 */
#define FLASH_STORE_HDR_WORDS 3u      /* magic / seq / crc */

typedef struct {
  uint32_t magic;
  uint32_t seq;
  uint32_t crc;
} FlashStoreHeader_s;

/* FNV-1a (RAM 数据) */
static uint32_t FlashStoreCrcRam(const uint8_t *p, uint32_t size) {
  uint32_t s = 0x811C9DC5u, i;
  for (i = 0; i < size; i++) {
    s ^= p[i];
    s *= 16777619u;
  }
  return s;
}

/* FNV-1a (Flash 数据, 分块读取避免申请与 size 等大的缓冲) */
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

/* 校验一个 bank; 有效返回 1, 并可回传 seq */
static int8_t FlashStoreValid(uint32_t addr, uint32_t size, uint32_t *seq) {
  FlashStoreHeader_s h;
  flash_read(addr, (uint32_t *)&h, FLASH_STORE_HDR_WORDS);
  if (h.magic != FLASH_STORE_MAGIC) return 0;
  if (h.crc != FlashStoreCrcAt(addr + FLASH_STORE_HDR_WORDS * 4u, size)) return 0;
  if (seq != NULL) *seq = h.seq;
  return 1;
}

/* 擦+写一个 bank 的完整记录, 回读校验; 0 成功 -1 失败 */
static int8_t FlashStoreWriteBank(uint32_t addr, uint32_t seq, const void *data, uint32_t size) {
  FlashStoreHeader_s h;
  h.magic = FLASH_STORE_MAGIC;
  h.seq = seq;
  h.crc = FlashStoreCrcRam((const uint8_t *)data, size);
  flash_erase_address(addr, 1);
  flash_write_single_address(addr, (uint32_t *)&h, FLASH_STORE_HDR_WORDS);
  flash_write_single_address(addr + FLASH_STORE_HDR_WORDS * 4u, (uint32_t *)data, size / 4u);
  return FlashStoreValid(addr, size, NULL) ? 0 : -1;
}

int8_t flash_store_save(uint32_t bank_a, uint32_t bank_b, const void *data, uint32_t size) {
  uint32_t sa = 0u, sb = 0u, seq, active, target, other;
  int oka, okb;

  if (data == NULL || size == 0u || (size & 3u) != 0u) return -1;

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

int8_t flash_store_load(uint32_t bank_a, uint32_t bank_b, void *data, uint32_t size) {
  uint32_t sa = 0u, sb = 0u, src;
  int oka, okb;

  if (data == NULL || size == 0u || (size & 3u) != 0u) return 0;

  oka = FlashStoreValid(bank_a, size, &sa);
  okb = FlashStoreValid(bank_b, size, &sb);
  if (!oka && !okb) return 0;

  if (oka && okb) src = (sa >= sb) ? bank_a : bank_b;
  else src = oka ? bank_a : bank_b;

  flash_read(src + FLASH_STORE_HDR_WORDS * 4u, (uint32_t *)data, size / 4u);
  return 1;
}
