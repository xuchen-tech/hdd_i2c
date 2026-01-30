# MSPM0G3507：TI-Drivers 的 I2C Controller / I2CTarget 配置注释版（可直接写博客）

> 目标：把 SysConfig/ti_drivers_config 生成的 I2C 相关宏和 `*_HWAttrs` / `*_config` 结构，解释成“看得懂、能改得对”的配置指南。

## 1. 先搞清楚：Controller vs Target

- **I2C Controller（主机/主控）**：主动发起传输（Start、地址、读写、Stop），典型用法是 MCU 去访问外设（EEPROM/传感器等）。
- **I2C Target（从机/被控）**：被动响应主机寻址；主机对它读/写时，它按协议给数据或收数据。

这份配置同时启用了：
- 一个 **I2CTarget**（挂在 `I2C1_INST`）
- 一个 **I2C Controller**（挂在 `I2C0_INST`）

这在“同一颗 MCU 一边当从机对外响应、另一边当主机去控制其他器件”的场景很常见。

---

## 2. 顶层配置常量（句柄索引）

下面这些通常来自 `ti_drivers_config.h` / SysConfig，用于把“逻辑名”映射成数组下标。

```c
/*
 *  ======== I2C ========
 */
extern const uint_least8_t CONFIG_I2C_CONTROLLER_CONST;
#define CONFIG_I2C_0 0

/*
 *  ======== I2C ========
 */
extern const uint_least8_t CONFIG_I2C_TARGET_CONST;
#define CONFIG_I2C_TARGET_0 0
#define CONFIG_TI_DRIVERS_I2C_COUNT 1
```

- `CONFIG_I2C_0` / `CONFIG_I2C_TARGET_0`：**数组索引**（0 表示第 1 个实例）。
- `CONFIG_TI_DRIVERS_I2C_COUNT`：I2C 实例数量（这里是 1，指 controller 侧的数量；target 侧有自己单独的 `CONFIG_I2CTARGET_COUNT`）。

> 博客提示：TI-Drivers 的 `I2C_open(CONFIG_I2C_0, ...)` 这类 API，本质是在用索引从 `I2C_config[]` 里取出对应的 `.object/.hwAttrs`。

---

## 3. I2C 速率/比特率宏

```c
#include <ti/drivers/I2C.h>

/* CONFIG_I2C_TARGET max speed (supported by all components) */
#define CONFIG_I2C_TARGET_MAXSPEED (400U) /* kbps */
#define CONFIG_I2C_TARGET_MAXBITRATE ((I2C_BitRate) I2C_400kHz)
```

- `CONFIG_I2C_TARGET_MAXSPEED`：更像是“文档级别的能力描述”（单位 kbps）。
- `CONFIG_I2C_TARGET_MAXBITRATE`：实际给 TI-Drivers 用的枚举值（这里是 400kHz）。

> 常见坑：I2C 频率由多处共同决定（时钟源/分频/bitrate 枚举/外设能力/上拉电阻/总线电容）。把宏改成 1MHz 并不保证总线真能跑到 1MHz。

---

## 4. 外设实例、中断号、PinMux（最关键）

这一段把硬件资源“起别名”，供后面的 `HWAttrs` 引用。

### 4.1 I2C0（Controller）

```c
#define I2C0_INST I2C0
#define I2C0_INST_IRQHandler I2C0_IRQHandler
#define I2C0_INST_INT_IRQN I2C0_INT_IRQn

#define GPIO_I2C0_SDA_PIN (28)
#define GPIO_I2C0_IOMUX_SDA (IOMUX_PINCM3)
#define GPIO_I2C0_IOMUX_SDA_FUNC IOMUX_PINCM3_PF_I2C0_SDA

#define GPIO_I2C0_SCL_PIN (31)
#define GPIO_I2C0_IOMUX_SCL (IOMUX_PINCM6)
#define GPIO_I2C0_IOMUX_SCL_FUNC IOMUX_PINCM6_PF_I2C0_SCL
```

- `I2C0_INST`：指向硬件寄存器基址（MSPM0 的 DriverLib 风格）。
- `I2C0_INST_INT_IRQN`：NVIC 中断号。
- `GPIO_I2C0_IOMUX_*`：IOMUX 的 PinCM 寄存器（决定某个管脚复用为 I2C SDA/SCL）。
- `GPIO_I2C0_*_PIN`：TI GPIO 驱动里的“pin index”（用于 GPIO driver 层）。

> 常见坑：PinCM 配错时，软件层面看起来“都初始化成功”，但总线就是不动；示波器/逻辑分析仪一看 SDA/SCL 根本没被复用到 I2C。

### 4.2 I2C1（Target）

```c
#define I2C1_INST I2C1
#define I2C1_INST_IRQHandler I2C1_IRQHandler
#define I2C1_INST_INT_IRQN I2C1_INT_IRQn

#define GPIO_I2C1_SDA_PIN (35)
#define GPIO_I2C1_IOMUX_SDA (IOMUX_PINCM16)
#define GPIO_I2C1_IOMUX_SDA_FUNC IOMUX_PINCM16_PF_I2C1_SDA

#define GPIO_I2C1_SCL_PIN (34)
#define GPIO_I2C1_IOMUX_SCL (IOMUX_PINCM15)
#define GPIO_I2C1_IOMUX_SCL_FUNC IOMUX_PINCM15_PF_I2C1_SCL
```

同理：这里把 **Target 侧**挂在 `I2C1`，引脚也与 `I2C0` 不同。

---

## 5. I2CTarget（从机）配置逐字段注释

```c
#include <ti/drivers/I2CTarget.h>
#include <ti/drivers/i2ctarget/I2CTargetMSPM0.h>

#define CONFIG_I2CTARGET_COUNT 1

I2CTargetMSPM0_Object I2CTargetMSPM0Objects[CONFIG_I2CTARGET_COUNT];

const I2CTargetMSPM0_HWAttrs I2CTargetMSPM0HWAttrs[CONFIG_I2CTARGET_COUNT] = {
    {
        .i2c         = I2C1_INST,             /* 选择 I2C 硬件实例：I2C1 */
        .intNum      = I2C1_INST_INT_IRQN,    /* 对应 NVIC IRQn */
        .intPriority = (~0),                 /* 中断优先级：(~0) 通常表示“使用默认/最低/不配置”，取决于驱动实现 */

        .sdaPincm    = GPIO_I2C1_IOMUX_SDA,   /* SDA 的 PinCM 寄存器 */
        .sdaPinIndex = GPIO_I2C1_SDA_PIN,    /* SDA 的 GPIO pin index */
        .sdaPinMux   = GPIO_I2C1_IOMUX_SDA_FUNC, /* SDA 复用功能选择 */

        .sclPincm    = GPIO_I2C1_IOMUX_SCL,
        .sclPinIndex = GPIO_I2C1_SCL_PIN,
        .sclPinMux   = GPIO_I2C1_IOMUX_SCL_FUNC,

        .clockSource                 = DL_I2C_CLOCK_BUSCLK,  /* I2C 外设时钟源：BUSCLK */
        .clockDivider                = DL_I2C_CLOCK_DIVIDE_1,/* 时钟分频 */
        .txIntFifoThr                = DL_I2C_TX_FIFO_LEVEL_BYTES_1, /* TX FIFO 中断阈值 */
        .rxIntFifoThr                = DL_I2C_RX_FIFO_LEVEL_BYTES_1, /* RX FIFO 中断阈值 */
        .isClockStretchingEnabled    = true, /* Target 侧常用：允许时钟拉伸，给软件处理时间 */
        .isAnalogGlitchFilterEnabled = false /* 模拟毛刺滤波：根据板级/线长/噪声情况决定 */
    },
};

const I2CTarget_Config I2CTarget_config[CONFIG_I2CTARGET_COUNT] = {
    {
        .object  = &I2CTargetMSPM0Objects[CONFIG_I2C_TARGET_0],
        .hwAttrs = &I2CTargetMSPM0HWAttrs[CONFIG_I2C_TARGET_0]
    },
};

const uint_least8_t I2CTarget_count = CONFIG_I2CTARGET_COUNT;
```

### 5.1 为什么 `.object` 和 `.hwAttrs` 要分开？

- `.hwAttrs`：**硬件属性**（寄存器基址、引脚、中断等），基本是“只读常量”。
- `.object`：**运行态对象**（驱动内部状态、锁、缓冲等），由驱动在 `*_open()` 时初始化。

这种设计能让一个驱动支持多个实例：每个实例一份 object + 一份 hwAttrs。

### 5.2 时钟拉伸（Clock Stretching）要不要开？

- **Target 侧建议默认开**：主机可能连续读/写；如果你的回调/处理不够快，拉伸 SCL 能避免丢字节。
- 但如果主机不允许拉伸或对拉伸敏感，就要谨慎（看主机 I2C 控制器/驱动是否容忍）。

---

## 6. I2C Controller（主机）配置逐字段注释

```c
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CMSPM0.h>

#define CONFIG_I2C_COUNT 1

I2CMSPM0_Object I2CMSPM0Objects[CONFIG_I2C_COUNT];

const I2CMSPM0_HWAttrs I2CMSPM0HWAttrs[CONFIG_I2C_COUNT] = {
    {
        .i2c         = I2C0_INST,            /* 选择 I2C0 作为 controller */
        .intNum      = I2C0_INST_INT_IRQN,
        .intPriority = (~0),

        .sdaPincm    = GPIO_I2C0_IOMUX_SDA,
        .sdaPinIndex = GPIO_I2C0_SDA_PIN,
        .sdaPinMux   = GPIO_I2C0_IOMUX_SDA_FUNC,

        .sclPincm    = GPIO_I2C0_IOMUX_SCL,
        .sclPinIndex = GPIO_I2C0_SCL_PIN,
        .sclPinMux   = GPIO_I2C0_IOMUX_SCL_FUNC,

        .clockSource              = DL_I2C_CLOCK_BUSCLK,
        .clockDivider             = DL_I2C_CLOCK_DIVIDE_1,
        .txIntFifoThr             = DL_I2C_TX_FIFO_LEVEL_BYTES_1,
        .rxIntFifoThr             = DL_I2C_RX_FIFO_LEVEL_BYTES_1,
        .isClockStretchingEnabled = true,    /* Controller 也可支持拉伸（对端拉伸时能正确等待） */
        .i2cClk                   = I2C_CLOCK_MHZ /* 这里通常是“输入时钟(MHz)”或派生时钟配置常量，供驱动计算时序 */
    },
};

const I2C_Config I2C_config[CONFIG_I2C_COUNT] = {
    {
        .object  = &I2CMSPM0Objects[CONFIG_I2C_0],
        .hwAttrs = &I2CMSPM0HWAttrs[CONFIG_I2C_0]
    },
};

const uint_least8_t I2C_count = CONFIG_I2C_COUNT;
```

### 6.1 `.i2cClk = I2C_CLOCK_MHZ` 是什么？

不同 TI 平台/驱动实现里命名略有差异，但总体含义是：
- 告诉驱动当前 I2C 外设时钟的“标称频率”或“换算基准”，用于生成 SCL 时序。

> 博客建议：把这里和时钟树（BUSCLK 的频率、divider）一起解释，否则读者会疑惑“为什么已经设置了 clockSource/divider 还要 i2cClk”。

---

## 7. 两个非常常见的问题（可当 FAQ）

### Q1：为什么我的 I2C Target 收不到主机的数据？

优先检查：
- I2C1 的 SDA/SCL PinCM 是否正确（PinMux 90% 的坑都在这里）
- 地址是否设置（Target 侧通常还会有 `I2CTarget_Params` / address 配置，不在这一段 `HWAttrs` 里）
- 总线上拉是否存在、阻值是否合适（尤其是 400kHz）

### Q2：Controller 侧通信偶发 NACK/超时？

排查顺序：
- 线长/上拉电阻/电容导致上升沿太慢（SDA/SCL 形状）
- `isClockStretchingEnabled` 是否需要开启（对端会拉伸时必须能等待）
- FIFO 阈值是否太低导致中断太频繁（实时性不足时反而丢数据）

---

## 8. 你可以怎么把这篇“落地”为你的工程说明

- 贴一张硬件连接图：I2C0（主机）连哪些外设、I2C1（从机）对接哪个主机。
- 给出一段最小示例：
  - Controller：`I2C_open(CONFIG_I2C_0, ...)` + `I2C_transfer()`
  - Target：`I2CTarget_open(CONFIG_I2C_TARGET_0, ...)` + 回调/读写缓冲逻辑

如果你希望我继续把“最小可运行示例代码”也补全（含 Controller 写/读、Target 收/发的 buffer 组织方式），告诉我你希望的协议格式（寄存器式/帧式/固定长度/带 CRC）。
