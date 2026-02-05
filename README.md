# libzdt - ZDT X42S 闭环步进电机驱动库

ESP-IDF 组件库，用于驱动 ZDT X42S/Y42 系列闭环步进电机（EMM 固件）。

## 特性

- ✅ **协议层与驱动层完全解耦**：支持任意 CAN 驱动实现
- ✅ **跨平台设计**：可在 ESP-IDF、Linux、STM32 等平台使用
- ✅ **完整的运动控制**：速度模式、位置模式、回零等
- ✅ **参数读取**：位置、脉冲数、IO 状态、配置参数等
- ✅ **定时返回**：支持电机定时上报状态，无需频繁轮询
- ✅ **多电机同步**：支持同步缓存模式，实现多电机协调运动
- ✅ **纯 C 实现**：无外部依赖，易于集成

## 架构设计

### 三层架构

```
┌─────────────────────────────────────────────┐
│         应用层 (User Application)           │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│      高级 API (zdt_can_init, zdt_set_*)     │  ← libzdt.h
│   协议层 (命令构建、响应解析、拆包组包)       │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│   驱动抽象层 (zdt_can_driver_t 接口)        │  ← zdt_can_driver.h
│   - init() / deinit()                        │
│   - send() / receive()                       │
└─────────────────────────────────────────────┘
                    ↓
┌──────────────────┬──────────────────────────┐
│  ESP-IDF TWAI    │   自定义驱动实现          │
│  (zdt_can_twai)  │   - SocketCAN            │
│                  │   - USB-CAN              │
│                  │   - MCU 自带 CAN         │
└──────────────────┴──────────────────────────┘
```

### 核心设计原则

1. **不透明指针 (Opaque Pointer)**：驱动结构体定义在源文件中，头文件只做前向声明
2. **依赖注入**：`zdt_can_init()` 通过参数注入驱动实例
3. **编译时控制**：通过宏 `ZDT_CAN_USE_TWAI` 控制是否编译 TWAI 内置驱动
4. **零拷贝**：协议层与驱动层共享消息缓冲区

## 快速开始

### ESP-IDF 项目使用内置 TWAI 驱动

#### 1. 添加组件

将 `libzdt` 复制到 ESP-IDF 项目的 `components` 目录：

```
my_project/
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   └── main.c
└── components/
    └── libzdt/          ← 复制到这里
        ├── include/
        ├── src/
        └── CMakeLists.txt
```

#### 2. 启用 TWAI 驱动

在项目的 `CMakeLists.txt` 或 `main/CMakeLists.txt` 中添加：

```cmake
# 定义使用 TWAI 驱动
target_compile_definitions(${COMPONENT_LIB} PRIVATE ZDT_CAN_USE_TWAI)
```

或在代码中定义：

```c
#define ZDT_CAN_USE_TWAI
#include "libzdt.h"
```

#### 3. 示例代码

```c
#include "libzdt.h"
#include "esp_log.h"

static const char *TAG = "example";

void app_main(void)
{
    // 1. 配置 CAN
    zdt_can_config_t can_config = {
        .tx_gpio = 5,              // CAN TX GPIO
        .rx_gpio = 4,              // CAN RX GPIO
        .bitrate = 500000,         // 500kbps
        .tx_timeout_ms = 100,
        .rx_timeout_ms = 1000,
    };

    // 2. 初始化（使用内置 TWAI 驱动）
    zdt_handle_t motor;
    zdt_error_t err = zdt_can_init(&can_config, zdt_get_twai_driver(), &motor);
    if (err != ZDT_OK) {
        ESP_LOGE(TAG, "Init failed: %d", err);
        return;
    }

    uint8_t motor_addr = 1;  // 电机地址

    // 3. 使能电机
    err = zdt_set_enable(motor, motor_addr, true);
    if (err != ZDT_OK) {
        ESP_LOGE(TAG, "Enable failed");
    }

    // 4. 位置清零
    err = zdt_clear_position(motor, motor_addr);

    // 5. 速度模式运动（300 RPM，顺时针）
    err = zdt_set_speed(motor, motor_addr, ZDT_DIR_CW, 300, 0);

    // 6. 读取位置
    zdt_position_t pos;
    err = zdt_read_position(motor, motor_addr, &pos);
    if (err == ZDT_OK) {
        ESP_LOGI(TAG, "Position: %.2f°", pos.angle_deg);
    }

    // 7. 位置模式运动（相对运动 90°，速度 100 RPM）
    err = zdt_set_position(motor, motor_addr, ZDT_DIR_CW,
                           100.0f, 90.0f, ZDT_MOTION_RELATIVE, false);

    // 8. 急停
    // zdt_emergency_stop(motor, motor_addr);

    // 清理
    zdt_can_deinit(motor);
}
```

### 非 ESP-IDF 平台使用自定义驱动

#### 1. 实现 CAN 驱动接口

创建自定义驱动实现（示例：Linux SocketCAN）：

```c
// my_socketcan_driver.c
#include "zdt_can_driver.h"
#include <linux/can.h>
#include <sys/socket.h>

// 驱动上下文
typedef struct {
    int socket_fd;
    char ifname[16];
    uint32_t tx_timeout_ms;
    uint32_t rx_timeout_ms;
} my_can_context_t;

// 实现驱动接口
static int my_can_init(const zdt_can_driver_config_t *config, void **context)
{
    my_can_context_t *ctx = calloc(1, sizeof(my_can_context_t));

    // 创建 SocketCAN 套接字
    ctx->socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    // ... 绑定到 CAN 接口

    *context = ctx;
    return 0;
}

static void my_can_deinit(void *context)
{
    my_can_context_t *ctx = context;
    close(ctx->socket_fd);
    free(ctx);
}

static int my_can_send(void *context, const zdt_can_driver_msg_t *msg)
{
    my_can_context_t *ctx = context;

    struct can_frame frame = {
        .can_id = msg->id,
        .can_dlc = msg->data_len,
    };
    memcpy(frame.data, msg->data, msg->data_len);

    write(ctx->socket_fd, &frame, sizeof(frame));
    return 0;
}

static int my_can_receive(void *context, zdt_can_driver_msg_t *msg)
{
    my_can_context_t *ctx = context;

    struct can_frame frame;
    read(ctx->socket_fd, &frame, sizeof(frame));

    msg->id = frame.can_id;
    msg->data_len = frame.can_dlc;
    memcpy(msg->data, frame.data, frame.can_dlc);

    return 0;
}

// 驱动虚函数表（定义在源文件中，不对外暴露）
struct zdt_can_driver_s {
    int (*init)(const zdt_can_driver_config_t *, void **);
    void (*deinit)(void *);
    int (*send)(void *, const zdt_can_driver_msg_t *);
    int (*receive)(void *, zdt_can_driver_msg_t *);
};

// 导出驱动实例
static const zdt_can_driver_t my_socketcan_driver = {
    .init = my_can_init,
    .deinit = my_can_deinit,
    .send = my_can_send,
    .receive = my_can_receive,
};

const zdt_can_driver_t *get_my_socketcan_driver(void)
{
    return &my_socketcan_driver;
}
```

#### 2. 使用自定义驱动

```c
#include "libzdt.h"

// 获取自定义驱动实例
extern const zdt_can_driver_t *get_my_socketcan_driver(void);

int main(void)
{
    zdt_can_config_t config = {
        .tx_gpio = -1,          // SocketCAN 不需要 GPIO
        .rx_gpio = -1,
        .bitrate = 500000,
        .tx_timeout_ms = 100,
        .rx_timeout_ms = 1000,
    };

    zdt_handle_t motor;
    zdt_can_init(&config, get_my_socketcan_driver(), &motor);

    // 使用电机...

    zdt_can_deinit(motor);
    return 0;
}
```

## API 参考

### 基础 API（协议层）

协议层函数不依赖硬件驱动，可在任何平台使用：

#### 命令构建

```c
// 构建编码器校准命令
int zdt_build_encoder_calibration(uint8_t addr, zdt_cmd_buffer_t *buf);

// 构建重启命令
int zdt_build_restart(uint8_t addr, zdt_cmd_buffer_t *buf);

// 构建位置清零命令
int zdt_build_clear_position(uint8_t addr, zdt_cmd_buffer_t *buf);

// 构建使能控制命令
int zdt_build_enable(uint8_t addr, zdt_enable_state_t enable,
                     zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf);

// 构建速度模式命令
int zdt_build_speed_mode(uint8_t addr, zdt_direction_t dir,
                         uint16_t speed_rpm, uint8_t acc,
                         zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf);

// 构建位置模式命令
int zdt_build_position_mode(uint8_t addr, zdt_direction_t dir,
                            uint16_t speed_rpm, uint8_t acc,
                            int32_t position_01deg,
                            zdt_motion_mode_t motion_mode,
                            zdt_sync_flag_t sync,
                            zdt_cmd_buffer_t *buf);
```

#### CAN 协议转换

```c
// 将命令缓冲区拆分为 CAN 帧序列
int zdt_cmd_to_can_msgs(uint8_t addr, const zdt_cmd_buffer_t *cmd,
                        zdt_can_msg_t *msgs, size_t max_msgs);

// 将 CAN 帧序列重组为响应
int zdt_can_msgs_to_response(const zdt_can_msg_t *msgs, size_t msg_count,
                             zdt_response_t *response);
```

#### 响应解析

```c
// 解析位置响应
int zdt_parse_position_response(const zdt_response_t *response,
                                zdt_position_t *position);

// 解析脉冲数响应
int zdt_parse_pulses_response(const zdt_response_t *response, int32_t *pulses);

// 检查响应是否成功
bool zdt_response_is_ok(const zdt_response_t *response);
```

### 高级 API（需要 CAN 驱动）

高级 API 提供完整的电机控制功能，需要传入 CAN 驱动实例：

#### 初始化

```c
// 初始化 CAN 通信
zdt_error_t zdt_can_init(const zdt_can_config_t *config,
                         const zdt_can_driver_t *driver,
                         zdt_handle_t *handle);

// 反初始化
void zdt_can_deinit(zdt_handle_t handle);
```

#### 电机控制

```c
// 使能/禁用电机
zdt_error_t zdt_set_enable(zdt_handle_t handle, uint8_t addr, bool enable);

// 速度模式（EMM 固件）
zdt_error_t zdt_set_speed(zdt_handle_t handle, uint8_t addr,
                          zdt_direction_t dir, uint16_t speed_rpm, uint8_t acc);

// 位置模式（EMM 固件）
zdt_error_t zdt_set_position(zdt_handle_t handle, uint8_t addr,
                             zdt_direction_t dir, float speed_rpm,
                             float angle_deg, zdt_motion_mode_t mode,
                             bool sync_cache);

// 回零运动
zdt_error_t zdt_homing(zdt_handle_t handle, uint8_t addr, uint8_t mode);

// 急停
zdt_error_t zdt_emergency_stop(zdt_handle_t handle, uint8_t addr);
```

#### 参数读取

```c
// 读取位置
zdt_error_t zdt_read_position(zdt_handle_t handle, uint8_t addr,
                              zdt_position_t *position);

// 读取脉冲数
zdt_error_t zdt_read_pulses(zdt_handle_t handle, uint8_t addr,
                            int32_t *pulses);

// 设置定时返回
zdt_error_t zdt_set_timed_return(zdt_handle_t handle, uint8_t addr,
                                 uint8_t info_func_code, uint16_t interval_ms);
```

## 高级功能

### 多电机同步运动

```c
// 1. 设置同步缓存模式
zdt_set_position(motor1, addr1, ZDT_DIR_CW, 100, 90.0,
                 ZDT_MOTION_RELATIVE, true);   // sync_cache = true
zdt_set_position(motor2, addr2, ZDT_DIR_CW, 100, 180.0,
                 ZDT_MOTION_RELATIVE, true);   // 先缓存

// 2. 触发同步执行（发送缓存立即执行命令）
zdt_cmd_buffer_t cmd;
zdt_build_enable(addr, ZDT_ENABLE_ON, ZDT_SYNC_IMMEDIATE, &cmd);
zdt_can_send_cmd(motor, addr, &cmd, NULL);

// 两台电机将同时开始运动
```

### 定时返回状态

```c
// 让电机每 100ms 返回一次位置数据
zdt_set_timed_return(motor, addr, ZDT_CMD_READ_POSITION, 100);

// 在接收线程中持续读取数据
while (1) {
    zdt_can_msg_t msg;
    // 接收 CAN 帧...
    zdt_response_t response;
    zdt_can_msgs_to_response(&msg, 1, &response);

    if (response.func_code == ZDT_CMD_READ_POSITION) {
        zdt_position_t pos;
        zdt_parse_position_response(&response, &pos);
        printf("Position: %.2f°\n", pos.angle_deg);
    }
}

// 停止定时返回
zdt_set_timed_return(motor, addr, ZDT_CMD_READ_POSITION, 0);
```

## 编译说明

### ESP-IDF 组件

```bash
cd <your_esp_idf_project>
idf.py build
idf.py flash monitor
```

### 作为静态库（非 ESP-IDF）

```bash
gcc -c src/zdt_can.c -I include -o zdt_can.o
ar rcs libzdt.a zdt_can.o

# 链接到你的项目
gcc main.c -L. -lzdt -o my_app
```

## 错误码

| 错误码 | 值 | 说明 |
|--------|-----|------|
| `ZDT_OK` | 0 | 成功 |
| `ZDT_ERR_INVALID_PARAM` | -1 | 无效参数 |
| `ZDT_ERR_BUFFER_TOO_SMALL` | -2 | 缓冲区太小 |
| `ZDT_ERR_CAN_TX_FAILED` | -3 | CAN 发送失败 |
| `ZDT_ERR_CAN_RX_TIMEOUT` | -4 | CAN 接收超时 |
| `ZDT_ERR_INVALID_RESPONSE` | -5 | 无效响应 |
| `ZDT_ERR_MOTOR_ERROR` | -6 | 电机返回错误 |
| `ZDT_ERR_NOT_INITIALIZED` | -7 | 未初始化 |

## 移植到新平台

### 步骤 1：实现驱动接口

根据你的 CAN 硬件实现四个函数：

```c
int my_driver_init(const zdt_can_driver_config_t *config, void **context);
void my_driver_deinit(void *context);
int my_driver_send(void *context, const zdt_can_driver_msg_t *msg);
int my_driver_receive(void *context, zdt_can_driver_msg_t *msg);
```

### 步骤 2：创建驱动实例

```c
static const zdt_can_driver_t my_driver = {
    .init = my_driver_init,
    .deinit = my_driver_deinit,
    .send = my_driver_send,
    .receive = my_driver_receive,
};
```

### 步骤 3：初始化并使用

```c
zdt_can_config_t config = { /* ... */ };
zdt_handle_t handle;
zdt_can_init(&config, &my_driver, &handle);
```

## 常见问题

### Q: 为什么要使用依赖注入而不是直接调用 TWAI？

**A:** 这样设计带来三个好处：
1. **可测试性**：可以注入 Mock 驱动进行单元测试
2. **可移植性**：同一套代码可在不同平台使用
3. **可维护性**：驱动和协议完全解耦，修改互不影响

### Q: 如何在同一个项目中使用多个 CAN 驱动？

**A:** 创建多个句柄，每个传入不同的驱动实例：

```c
zdt_handle_t motor_twai, motor_socketcan;
zdt_can_init(&config1, zdt_get_twai_driver(), &motor_twai);
zdt_can_init(&config2, &my_socketcan_driver, &motor_socketcan);
```

### Q: 性能开销如何？

**A:** 几乎为零。驱动接口使用函数指针，编译器优化后与直接调用性能相同。协议层与驱动层共享消息缓冲区，无拷贝开销。

## 许可证

MIT License

## 参考文档

- [ZDT X42S 电机用户手册](./ZDT_X42S第二代闭环步进电机用户手册V1.0.3_251224.pdf)
- [ESP-IDF TWAI 驱动文档](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/twai.html)

## 贡献

欢迎提交 Issue 和 Pull Request！
