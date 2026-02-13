# libzdt 测试项目

这是一个用于测试 `libzdt` 组件库的 ESP-IDF 测试项目。

## 项目结构

```
test/
├── CMakeLists.txt          # 项目 CMake 配置
├── sdkconfig.defaults      # 默认配置文件
├── main/
│   ├── CMakeLists.txt      # 主组件 CMake 配置
│   └── main.c              # 主测试程序
└── README.md               # 本文件
```

## 功能说明

本项目测试 `libzdt` 库的完整功能，包括：

### 基础功能测试
- ✅ TWAI 驱动初始化和配置
- ✅ 接收回调注册
- ✅ 统计信息获取

### 命令发送测试
- ✅ 电机使能/禁用控制
- ✅ 读取电机状态
- ✅ 读取实时位置
- ✅ 读取实时转速
- ✅ 立即停止

### 分包消息测试
- ✅ 速度模式控制（2分包）
- ✅ 位置模式控制（3分包）

### 接收模式测试
- ✅ 回调接收模式
- ✅ 轮询接收模式

## 硬件连接

### CAN 总线连接

```
ESP32          CAN收发器        ZDT电机
------         ----------       -------
GPIO5  ------>  TX
GPIO4  ------>  RX
GND    ------>  GND      -------> CAN_GND
                CAN_H    -------> CAN_H
                CAN_L    -------> CAN_L
                VCC      -------> 5V
```

### 注意事项

1. **终端电阻**：CAN总线两端需要各接一个120Ω终端电阻
2. **共地**：确保ESP32和ZDT电机共地
3. **收发器**：使用CAN收发器（如TJA1050、SN65HVD230等）
4. **波特率**：默认1Mbps，可在代码中修改

## 编译和烧录

### 1. 配置项目（可选）

```bash
cd test
idf.py menuconfig
```

在 `Component Config` -> `libzdt Configuration` 中可以配置TWAI驱动参数。

### 2. 编译项目

```bash
idf.py build
```

### 3. 烧录到设备

```bash
idf.py -p COM_PORT flash monitor
```

（将 `COM_PORT` 替换为您的实际串口，Windows如`COM3`，Linux如`/dev/ttyUSB0`）

### 4. 清理构建

```bash
idf.py fullclean
```

## 测试流程

程序会自动执行以下测试序列：

```
========================================
  ZDT TWAI Driver Test Suite
========================================
Motor Address: 1
TX GPIO: 5
RX GPIO: 4
========================================

=== Test 1: TWAI Driver Init ===
✓ TWAI Init PASSED

=== Test 2: Register RX Callback ===
✓ Register Callback PASSED

=== Test 3: Motor Enable ===
✓ Motor Enable PASSED

=== Test 4: Read Motor State ===
✓ Read Motor State PASSED

=== Test 5: Velocity Mode (Multi-packet) ===
✓ Velocity Mode PASSED

=== Test 6: Position Mode (Multi-packet) ===
✓ Position Mode PASSED

=== Test 7: Immediate Stop ===
✓ Immediate Stop PASSED

=== Test 8: Read Real Position ===
✓ Read Real Position PASSED

=== Test 9: Read Real Speed ===
✓ Read Real Speed PASSED

=== Test 10: Poll Receive Mode ===
✓ Poll Receive PASSED

=== Test 11: Statistics ===
✓ Statistics PASSED

========================================
  Test Summary
========================================
Total TX: 15
Total RX: 8
Total ACK: 8
Total Errors: 0

✓ ALL TESTS PASSED ✓
========================================
```

## 测试说明

### 测试1：TWAI驱动初始化
- 初始化TWAI驱动
- 配置GPIO引脚
- 启动驱动

### 测试2：注册接收回调
- 注册CAN消息接收回调函数
- 创建后台接收任务

### 测试3：电机使能
- 发送电机使能命令
- 等待电机响应

### 测试4：读取电机状态
- 读取电机状态标志
- 解析返回数据

### 测试5：速度模式（分包）
- 构造速度模式命令（2个分包）
- 发送所有分包
- 等待电机响应

### 测试6：位置模式（分包）
- 构造位置模式命令（3个分包）
- 发送所有分包
- 等待电机运动完成

### 测试7：立即停止
- 发送立即停止命令
- 确认电机停止

### 测试8：读取实时位置
- 发送读取位置命令
- 解析位置数据

### 测试9：读取实时转速
- 发送读取转速命令
- 解析转速数据

### 测试10：轮询接收模式
- 发送命令
- 使用轮询方式接收响应

### 测试11：统计信息
- 打印发送/接收/错误计数

## 自定义测试

### 修改电机地址

```c
#define TEST_MOTOR_ADDR        1      // 修改为您的电机地址
```

### 修改GPIO引脚

```c
#define TEST_TX_GPIO           5      // 修改TX引脚
#define TEST_RX_GPIO           4      // 修改RX引脚
```

### 修改波特率

```c
config.bitrate = 1000000;  // 1Mbps
// 或
config.bitrate = 500000;   // 500kbps
```

### 修改运动参数

```c
zdt_velocity_cmd_x_t params = {
    .addr = 1,
    .direction = ZDT_DIRECTION_CW,
    .acceleration = 1000,   // 加速度 RPM/S
    .speed = 2000,          // 速度 0.1RPM (200.0 RPM)
    .sync_flag = ZDT_SYNC_IMMEDIATE,
    .max_current = 2000     // 最大电流 mA
};
```

## 预期输出

### 有电机连接时

```
I (123) ZDT_TEST: [TX] Addr=1, Packet=0x00, Cmd=0xF3, DLC=6
I (124) ZDT_TEST:     Data: F3 AB 01 00 6B
I (135) ZDT_TEST: [RX] Addr=1, Packet=0x00, Cmd=0xF3, DLC=4
I (136) ZDT_TEST:     Data: F3 02 6B
I (137) ZDT_TEST:     → ACK received (0x02)
```

### 无电机连接时

```
I (123) ZDT_TEST: [TX] Addr=1, Packet=0x00, Cmd=0xF3, DLC=6
I (124) ZDT_TEST:     Data: F3 AB 01 00 6B
W (500) ZDT_TEST: No ACK received (motor may not be connected)
```

## 依赖项

本项目依赖以下组件：

- `libzdt` - ZDT电机驱动库（上级目录）
- `esp_driver_twai` - ESP-IDF TWAI驱动
- `freertos` - FreeRTOS

## 故障排除

### 1. TWAI驱动初始化失败

**症状**：`Failed to initialize TWAI driver`

**解决方案**：
- 检查GPIO引脚配置是否正确
- 确保TWAI驱动已启用：`idf.py menuconfig` -> `libzdt Configuration`
- 检查硬件连接

### 2. 未收到响应

**症状**：`No ACK received`

**解决方案**：
- 检查电机地址是否正确
- 检查CAN总线终端电阻（120Ω）
- 检查波特率设置
- 确认电机已上电

### 3. 分包发送失败

**症状**：`Failed to send all packets`

**解决方案**：
- 增加分包间隔时间
- 检查CAN总线负载
- 增加超时时间

### 4. 编译错误

**症状**：`undefined reference to 'zdt_twai_*'`

**解决方案**：
- 确保在上级目录运行`idf.py build`
- 检查CMakeLists.txt中的REQUIRES依赖

## 监控模式

测试完成后，程序进入监控模式，每10秒打印一次统计信息：

```
I (10000) ZDT_TEST: [Monitor] TX=15, RX=8, Errors=0
```

## 参考文档

- [ZDT协议文档](../zdt-protocol.md)
- [TWAI驱动使用指南](../TWAI_USAGE.md)
- [libzdt API文档](../README.md)

## 许可证

与 libzdt 主项目相同
