# bsp_flash · 内部 Flash 读写 + 通用掉电保存

<p align='right'>ai @ dart_final_work</p>

面向**后续调用者**的使用说明。详细设计见同目录 [`bsp_flash.md`](bsp_flash.md)。

本模块提供两层能力：

| 层 | 接口 | 用途 |
|---|---|---|
| 底层原语 | `flash_erase_address` / `flash_write_single_address` / `flash_write_muli_address` / `flash_read` / `get_next_flash_address` | 裸扇区读写 |
| 通用掉电保存 | `flash_store_save` / `flash_store_load` | **双 Bank + magic + seq + CRC**，写一半掉电也不丢 |

> 「把一份配置写进 Flash 且掉电不丢」直接用 **`flash_store_save/load`** 即可，不必自己处理校验与备份。

---

## 1. 底层原语

```c
void   flash_erase_address(uint32_t address, uint16_t len);                              // len = 擦除的扇区数
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);  // len = 32bit 字数
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);
void   flash_read(uint32_t address, uint32_t *buf, uint32_t len);                        // len = 32bit 字数
uint32_t get_next_flash_address(uint32_t address);                                       // 下一扇区起始地址
```

> ⚠️ `flash_write_single_address` / `flash_read` 的 `len` 单位是 **word(4 字节)**，不是字节数。
> ⚠️ `flash_erase_address` 擦除前会清 Flash 错误标志（否则会静默失败），擦除过程会阻塞（128KB 扇区约 1s）。

扇区地址宏：`ADDR_FLASH_SECTOR_0 ~ 11`（F407），`FLASH_END_ADDR = 0x08100000`。
用作掉电保存时请选**未被固件占用**的整扇区起始地址，例如 `ADDR_FLASH_SECTOR_10` / `ADDR_FLASH_SECTOR_11`。

---

## 2. 通用掉电保存（推荐）

```c
int8_t flash_store_save(uint32_t bank_a, uint32_t bank_b, const void *data, uint32_t size); // 0 成功 / -1 失败
int8_t flash_store_load(uint32_t bank_a, uint32_t bank_b, void       *data, uint32_t size); // 1 有效 / 0 无数据
```

**记录布局**（每个 bank 从扇区起始地址开始，头 12 字节）：

```
+0x00  magic (0xB1A5D00D)   记录标记
+0x04  seq                   递增序号
+0x08  crc                   data 区 FNV-1a 校验
+0x0C  data[]                调用方数据，长度 size
```

**读写策略**

- **保存**：总是写「非当前」那个 bank，`seq = 当前seq + 1`；写完**回读校验**，通过才切换为当前；若目标 bank 写失败，**回退**写另一个 bank。首次无有效数据时写 A。
- **读取**：两份都校验，取 `seq` 更大的那份；两份都无效返回 0。

**约束**

| 项 | 要求 |
|---|---|
| `size` | 必须 **4 字节对齐** |
| 单份大小 | 需 **小于一个扇区**（F407 最大 128KB） |
| 结构体 | 应**无隐式 padding**（成员用 `float`/`int32_t`，勿用 `int8_t` 混排） |
| 调用位置 | **任务上下文**，不要在中断里调用 |
| magic | `FLASH_STORE_MAGIC` 改动会使旧数据失效，改布局时记得一并改 |

---

## 3. 快速上手

```c
#include "bsp_flash.h"

#define MY_BANK_A ADDR_FLASH_SECTOR_10   // 0x080C0000
#define MY_BANK_B ADDR_FLASH_SECTOR_11   // 0x080E0000

typedef struct {          // 全 4 字节字段 -> 无 padding
    float kp, ki, kd;
    int32_t enable;
} MyCfg_t;

MyCfg_t cfg;

void Load(void) {
    if (flash_store_load(MY_BANK_A, MY_BANK_B, &cfg, sizeof(cfg)) == 0) {
        MyCfg_Defaults(&cfg);             // 无有效数据 -> 用默认值
    }
}

void Save(void) {
    flash_store_save(MY_BANK_A, MY_BANK_B, &cfg, sizeof(cfg));
}
```

---

## 4. 注意事项

1. 本仓库源码统一 **UTF-8**，请勿混入 GBK。
2. 擦写会打断执行，请在任务上下文调用；`flash_store_save` 阻塞约 1s。
3. H7（`STM32H723xx`）按 128KB/扇区、双 Bank 映射；该分支**未经真机验证**，使用前请确认。
4. 给舵机存标定见 `Modules/motor/servo_motor/README.md`。
