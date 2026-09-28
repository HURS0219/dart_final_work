# bsp_flash

<p align='right'>ai @ dart_final_work</p>

内部 Flash 读写封装。提供**底层原语**（擦/写/读/扇区换算）和**通用掉电保存**（双 Bank + magic + seq + CRC）。

## 1. 分层与用途

| 层 | 接口 | 说明 |
|---|---|---|
| 底层原语 | `flash_erase_address / flash_write_single_address / flash_write_muli_address / flash_read / get_next_flash_address` | 裸扇区操作，写入单位为 32-bit word |
| 通用掉电保存 | `flash_store_save / flash_store_load` | 双 Bank 交替 + `magic/seq/CRC(FNV-1a)` + 回读校验 + 失败回退，供任意模块/应用复用 |

## 2. 扇区地址

`ADDR_FLASH_SECTOR_0 ~ 11`（F407，第 0~3 扇区 16KB、第 4 扇区 64KB、第 5~11 扇区 128KB），`FLASH_END_ADDR = 0x08100000`。
`ADDR_FLASH_SECTOR_12 ~ 23` 为 `0x08100000` 起的地址（供其它用途）。

> 用于掉电保存时，务必选**未被固件占用**的整扇区起始地址。例如舵机/参数可用 `ADDR_FLASH_SECTOR_10` 与 `ADDR_FLASH_SECTOR_11` 作为 A/B 两个 Bank。

## 3. 底层原语

```c
void   flash_erase_address(uint32_t address, uint16_t len);                 // 擦除, len=扇区数
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len); // len=word 数
void   flash_read(uint32_t address, uint32_t *buf, uint32_t len);           // len=word 数
uint32_t get_next_flash_address(uint32_t address);                          // 下一扇区起始地址
```

- 擦除前会**清除 Flash 错误标志**（残留 `WRPERR/PGSERR` 会导致擦除静默失败）。
- `flash_write_single_address` 参数为 **word 个数**（字节数 / 4）。

## 4. 通用掉电保存（推荐用法）

```c
/* 单份记录布局: { magic, seq, crc } (12B) + data */
int8_t flash_store_save(uint32_t bank_a, uint32_t bank_b, const void *data, uint32_t size);
int8_t flash_store_load(uint32_t bank_a, uint32_t bank_b, void       *data, uint32_t size);
```

- `bank_a` / `bank_b`：两个备份扇区的起始地址（可来自不同扇区）。
- `size`：数据字节数，**必须 4 字节对齐**，且单份需**小于一个扇区**。
- **save**：写“非当前”那个 Bank 并 `seq+1`，回读校验通过才切换；目标 Bank 写失败则回退写另一个。返回 `0` 成功 / `-1` 失败。
- **load**：读两份，校验通过且 `seq` 更大者胜出。返回 `1` 有效 / `0` 无有效数据（调用方保持默认值即可）。

### 使用示例

```c
#include "bsp_flash.h"

#define MY_CFG_A ADDR_FLASH_SECTOR_10
#define MY_CFG_B ADDR_FLASH_SECTOR_11

MyCfg_t cfg;

void Load(void) {
    if (flash_store_load(MY_CFG_A, MY_CFG_B, &cfg, sizeof(cfg)) == 0) {
        MyCfg_Defaults(&cfg);      /* 无有效数据 -> 用默认值 */
    }
}

void Save(void) {
    flash_store_save(MY_CFG_A, MY_CFG_B, &cfg, sizeof(cfg));
}
```

> **注意**：擦写期间会打断执行（F407 单扇区 128KB 约 1s），请在任务上下文里调用，不要在中断里调用。

## 5. 注意事项

1. 传入的 `data` 结构体应**无隐式 padding**（用固定宽度整型成员），否则跨编译器的 CRC 可能不一致。
2. `FLASH_STORE_MAGIC`（`bsp_flash.c`）改动会使旧数据失效，需重新保存一次。
3. H7（`STM32H723xx`）按 128KB/扇区、双 Bank 映射；H7 分支未经真机验证，使用前请确认。
