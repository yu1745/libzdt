# libzdt

ZDT X42S/Y42 系列闭环步进电机 CAN 通讯协议库（ESP-IDF 组件）

## 简介

`libzdt` 是一个轻量级的 C 语言通讯协议库，用于通过 CAN 总线控制 ZDT X42S/Y42 系列闭环步进电机。本项目基于 [ZDT X42S 通讯协议](./zdt-protocol.md) 第5章实现，支持 **X 固件**和 **EMM 固件**。

### 主要特性

- 协议层与驱动层完全解耦，可移植到任意平台
- 支持 X 固件和 EMM 固件两种协议
- 内置 ESP-IDF TWAI 驱动实现
- 提供完整的命令构建和响应解析函数
- 纯 C 实现，无外部依赖

## 快速开始

### 1. 集成到 ESP-IDF 项目

将本组件复制到 ESP-IDF 项目的 `components` 目录：

```bash
my_project/
├── CMakeLists.txt
├── main/
└── components/
    └── libzdt/          # 复制组件到这里
```

### 2. 使用 TWAI 驱动示例

```c
#include "zdt_twai.h"
#include "esp_log.h"

static const char *TAG = "example";

void app_main(void)
{
    // 1. 初始化 TWAI 驱动
    zdt_twai_config_t config = ZDT_TWAI_DEFAULT_CONFIG();
    config.tx_gpio = 5;
    config.rx_gpio = 4;
    config.bitrate = 1000000;  // 1Mbps

    if (zdt_twai_init(&config) != 0) {
        ESP_LOGE(TAG, "TWAI init failed");
        return;
    }

    zdt_twai_start();

    // 2. 使能电机
    zdt_can_msg_t msg;
    zdt_cmd_motor_enable(1, true, ZDT_SYNC_IMMEDIATE, &msg);
    zdt_twai_send(&msg, 100);

    // 3. 速度模式运动
    zdt_velocity_cmd_x_t vel_params = {
        .addr = 1,
        .direction = ZDT_DIRECTION_CW,
        .acceleration = 1000,   // 1000 RPM/S
        .speed = 2000,          // 200.0 RPM
        .sync_flag = ZDT_SYNC_IMMEDIATE,
        .max_current = 2000     // 2000 mA
    };
    zdt_can_msg_t msgs[2];
    uint8_t count = zdt_cmd_velocity_mode_x(&vel_params, msgs);
    zdt_twai_send_multi(msgs, count, 1000, 100);

    // 4. 位置模式运动
    zdt_position_trapezoid_cmd_x_t pos_params = {
        .addr = 1,
        .direction = ZDT_DIRECTION_CCW,
        .accel_acc = 500,
        .decel_acc = 500,
        .max_speed = 1000,     // 100.0 RPM
        .position = 36000,      // 3600.0°
        .motion_mode = ZDT_MOTION_MODE_ABSOLUTE,
        .sync_flag = ZDT_SYNC_IMMEDIATE,
        .max_current = 2000
    };
    count = zdt_cmd_position_trapezoid_x(&pos_params, msgs);
    zdt_twai_send_multi(msgs, count, 1000, 100);

    // 5. 立即停止
    zdt_cmd_immediate_stop(1, ZDT_SYNC_IMMEDIATE, &msg);
    zdt_twai_send(&msg, 100);
}
```

## 目录结构

```
libzdt/
├── include/
│   ├── libzdt.h        # 协议层头文件（命令构建/解析）
│   └── zdt_twai.h      # TWAI 驱动头文件
├── src/
│   ├── libzdt.c        # 协议层实现
│   └── zdt_twai.c      # TWAI 驱动实现
├── examples/
│   └── twai_example.c  # TWAI 使用示例
├── zdt-protocol.md     # ZDT 通讯协议文档
└── README.md
```

## 协议层 API

### 命令构建函数

所有命令构建函数位于 [libzdt.h](include/libzdt.h)，按功能分类：

#### 电机控制

| 函数 | 说明 |
|------|------|
| `zdt_cmd_motor_enable()` | 电机使能/禁用 |
| `zdt_cmd_velocity_mode_x()` | 速度模式（X 固件） |
| `zdt_cmd_velocity_mode_emm()` | 速度模式（EMM 固件） |
| `zdt_cmd_position_trapezoid_x()` | 位置模式梯形曲线（X 固件） |
| `zdt_cmd_position_mode_emm()` | 位置模式（EMM 固件） |
| `zdt_cmd_immediate_stop()` | 立即停止 |

#### 回零控制

| 函数 | 说明 |
|------|------|
| `zdt_cmd_trigger_homing()` | 触发回零 |
| `zdt_cmd_stop_homing()` | 停止回零 |
| `zdt_cmd_set_zero_point()` | 设置零点位置 |

#### 参数读取

| 函数 | 说明 |
|------|------|
| `zdt_cmd_read_motor_state()` | 读取电机状态 |
| `zdt_cmd_read_real_position()` | 读取实时位置 |
| `zdt_cmd_read_real_speed()` | 读取实时转速 |
| `zdt_cmd_read_phase_current()` | 读取相电流 |
| `zdt_cmd_read_driver_temp()` | 读取驱动温度 |
| `zdt_cmd_read_version()` | 读取固件版本 |

#### 参数写入

| 函数 | 说明 |
|------|------|
| `zdt_cmd_write_motor_id()` | 修改电机地址 |
| `zdt_cmd_write_subdivs()` | 修改细分值 |
| `zdt_cmd_write_pid_param_x()` | 修改 PID 参数（X 固件） |
| `zdt_cmd_write_closed_loop_current()` | 修改闭环最大电流 |

### 响应解析函数

| 函数 | 说明 |
|------|------|
| `zdt_verify_response()` | 验证响应是否正确 |
| `zdt_parse_motor_state()` | 解析电机状态 |
| `zdt_parse_real_position()` | 解析实时位置 |
| `zdt_parse_real_speed()` | 解析实时转速 |
| `zdt_parse_phase_current()` | 解析相电流 |
| `zdt_parse_driver_temp()` | 解析驱动温度 |
| `zdt_parse_version_info()` | 解析版本信息 |

## TWAI 驱动 API

TWAI 驱动位于 [zdt_twai.h](include/zdt_twai.h)：

```c
// 初始化/启动/停止
int zdt_twai_init(const zdt_twai_config_t *config);
int zdt_twai_start(void);
void zdt_twai_deinit(void);

// 发送消息
int zdt_twai_send(const zdt_can_msg_t *msg, uint32_t timeout_ms);
int zdt_twai_send_multi(const zdt_can_msg_t *msgs, uint8_t count,
                       uint32_t interval_us, uint32_t timeout_ms);

// 接收回调
typedef void (*zdt_twai_rx_callback_t)(const zdt_can_msg_t *msg, void *arg);
int zdt_twai_register_callback(zdt_twai_rx_callback_t callback, void *arg);

// 统计信息
uint32_t zdt_twai_get_rx_count(void);
uint32_t zdt_twai_get_tx_count(void);
uint32_t zdt_twai_get_error_count(void);
```

## 固件差异说明

X 固件与 EMM 固件在以下命令存在差异：

| 功能 | X 固件 | EMM 固件 |
|------|--------|----------|
| 力矩模式 | 支持 | 不支持 |
| 速度模式 | `zdt_cmd_velocity_mode_x()` | `zdt_cmd_velocity_mode_emm()` |
| 位置模式 | `zdt_cmd_position_trapezoid_x()` | `zdt_cmd_position_mode_emm()` |
| 位置单位 | 0.1° | 需转换 (值×360/65536) |
| 速度单位 | 0.1 RPM | RPM |

详细说明请参考 [zdt-protocol.md](zdt-protocol.md) 第5.1.2节。

## 移植到其他平台

协议层（`libzdt.h` + `libzdt.c`）不依赖任何特定硬件，可轻松移植到其他平台：

1. 实现自己的 CAN 驱动接口
2. 将 CAN 消息转换为 `zdt_can_msg_t` 结构
3. 调用协议层命令构建函数生成命令
4. 通过自己的 CAN 驱动发送消息
5. 接收响应后调用协议层解析函数

## 构建与运行

```bash
cd examples/
idf.py build flash monitor
```

## 参考文档

- [ZDT X42S 用户手册](./ZDT_X42S第二代闭环步进电机用户手册V1.0.3_251224.pdf)
- [ZDT 通讯协议](./zdt-protocol.md)
- [ESP-IDF TWAI 驱动文档](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/twai.html)

## 许可证

MIT License
