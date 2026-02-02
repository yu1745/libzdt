# ZDT CAN协议 ESP-IDF 集成测试

本目录包含在 ESP32 上运行的集成测试。

## 测试类型

### 1. 协议层测试 (默认启用)
- 不需要任何硬件
- 验证命令构建、CAN帧拆包、响应解析等
- 使用 Unity 测试框架

### 2. 硬件测试 (可选)
- 需要 CAN 收发器 (如 SN65HVD230)
- 需要 ZDT X42S 电机
- 在 menuconfig 中启用

## 编译和运行

### 1. 设置 ESP-IDF 环境

```bash
# Linux/macOS
. $HOME/esp/esp-idf/export.sh

# Windows
%IDF_PATH%\export.bat
```

### 2. 配置项目

```bash
cd test_espidf
idf.py set-target esp32  # 或 esp32s3 等
idf.py menuconfig
```

在 menuconfig 中可以配置：
- `ZDT Test Configuration` -> `Enable hardware tests` - 启用硬件测试
- `ZDT Test Configuration` -> `CAN TX/RX GPIO` - 配置CAN引脚
- `ZDT Test Configuration` -> `Motor address` - 电机地址

### 3. 编译

```bash
idf.py build
```

### 4. 烧录并运行

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

## 测试菜单

烧录后，串口会显示 Unity 测试菜单：

```
Press ENTER to see the list of tests.
Enter test for running.
Enter 'l' to list all tests.
Enter '*' to run all tests.
Enter '[tag]' to run tests with specific tag.
```

### 常用命令：
- `*` - 运行所有测试
- `[protocol]` - 只运行协议测试
- `[can]` - 只运行CAN相关测试
- `[hardware]` - 只运行硬件测试 (需要启用)

## 硬件连接

如果启用硬件测试，需要如下连接：

```
ESP32                CAN Transceiver           ZDT Motor
------               ---------------           ---------
GPIO21 (TX) -------> TXD
GPIO22 (RX) <------- RXD
3.3V ----------------> VCC
GND -----------------> GND
                       CANH <----------------> CANH
                       CANL <----------------> CANL
```

## 测试覆盖

| 测试类别 | 测试项 | 标签 |
|---------|--------|------|
| 命令构建 | 编码器校准、重启、清零等 | `[protocol][cmd]` |
| X固件命令 | 力矩、速度、位置模式 | `[protocol][x_cmd]` |
| CAN协议 | ID生成、拆包 | `[protocol][can]` |
| 响应解析 | 状态判断、字符串 | `[protocol][response]` |
| 工具函数 | 角度/速度转换 | `[protocol][util]` |
| 边界条件 | 空指针、缓冲区 | `[protocol][edge]` |
| 硬件测试 | CAN初始化、电机控制 | `[hardware]` |
