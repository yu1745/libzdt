# libzdt

ZDT_X42S 第二代闭环步进电机（含 Y42 / Emm 固件变体）的 **ESP-IDF 组件**：构建手册第 5 章 5.2–5.8 的 82 条主机命令，串口 TTL 与 CAN 两种传输都支持——UART 驱动负责发送和应答分帧，TWAI 驱动负责发送、多包拆装和应答接收。

- 要求 ESP-IDF ≥ 6.1（使用新的基于节点的 TWAI 驱动 `esp_driver_twai`）
- 协议依据：[《ZDT_X42S 第二代闭环步进电机用户手册 V1.0.3》](./ZDT_X42S第二代闭环步进电机用户手册V1.0.3_251224.pdf)，随仓库附带
- 串口校验支持 `0x6B` / XOR / CRC8；不含 Modbus。CAN 校验码固定 `0x6B`

## 用法

串口 TTL：

```c
#include "libzdt.h"

zdt_uart_handle_t bus;
zdt_uart_config_t cfg = {
    .port = UART_NUM_1, .tx_io = GPIO_NUM_1, .rx_io = GPIO_NUM_0,
    .baud_rate = 115200, .checksum = ZDT_CHECKSUM_FIXED_6B, .firmware = ZDT_FW_EMM,
};
ESP_ERROR_CHECK(zdt_uart_new(&cfg, &bus));

uint8_t frame[16];
zdt_reply_t reply;
int n = zdtBuildReadRealtimePosCmd(1, frame, sizeof frame);   /* 构建：01 36 6B */
zdt_uart_transact(bus, frame, n, &reply, 200);                /* 发送并等应答 */
/* reply.data = 符号 + u32 位置（Emm：65536 = 360°） */
```

CAN：

```c
zdt_twai_handle_t bus;
zdt_twai_config_t cfg = { .tx_io = GPIO_NUM_4, .rx_io = GPIO_NUM_5, .bitrate = 500000 };
ESP_ERROR_CHECK(zdt_twai_new(&cfg, &bus));

zdt_can_msg_t msg;
zdt_reply_t reply;

zdtCanBuildMotorEnableCmd(1, 1, ZDT_SYNC_NOW, &msg);        /* 构建 */
zdt_twai_transact(bus, &msg, 2, &reply, 500);               /* 发送并等应答 */
if (zdt_reply_is_ok(&reply)) { /* 02 */ }
```

完整例子见 [`examples/basic`](./examples/basic)。

| 层 | 头文件 | 作用 |
|---|---|---|
| 构建 | `zdt_common_*.h` `zdt_x.h` `zdt_emm.h` | 每条命令两个函数：`zdtBuild*Cmd(..., buf, len)` 输出串口原始帧（返回帧长）；`zdtCanBuild*Cmd(..., &msg)` 输出 `zdt_can_msg_t`（1–8 个 CAN 帧）。纯计算，不访问硬件 |
| 串口分帧 | `zdt_frame.h` | 校验字节计算 / 改写、按功能码查应答长度、从字节流切出一帧。纯计算 |
| 串口收发 | `zdt_uart.h` | `zdt_uart_new/del`、`zdt_uart_send`、`zdt_uart_recv_reply`、`zdt_uart_transact`、`zdt_uart_poll`、事件回调 |
| CAN 收发 | `zdt_twai.h` | `zdt_twai_new/del`、`zdt_twai_send`、`zdt_twai_recv_reply`、`zdt_twai_transact` |

应答（`zdt_reply_t`）：`code` 为功能码，`data` 为功能码之后、校验字节之前的内容，状态类命令的 `data[0]` 是 `ZDT_REPLY_OK(02)` / `ZDT_REPLY_PARAM_ERR(E2)` / `ZDT_REPLY_FORMAT_ERR(EE)`。命令格式错误时电机回 `Addr 00 EE`（`code` 为 0）。位置到位、回零完成、夹爪夹紧时电机会主动再发一帧 `ZDT_REPLY_DONE(9F)`（需电机应答方式为 Reached / Both）：CAN 用 `zdt_twai_recv_reply` 单独接收；串口上与当前请求无关的帧（9F、定时返回、其它地址）交给 `zdt_uart_set_event_cb` 设置的回调，空闲时调用 `zdt_uart_poll` 分派。

## 串口帧规则（手册 4.1，p.39；真机核对）

- 请求就是手册里的"原始命令"：`Addr + Code + 数据 + 校验`，校验出厂为固定 `0x6B`
- 应答没有长度字段：`zdt_ttl_parse` 按功能码查长度；`42` / `43` 的第 3 字节是整帧长度；`Addr 00 EE 校验` 为格式错误
- 广播（地址 0）命令只有地址 1 的电机应答；一帧里让多台电机都回数据（`AA` 打包读取、广播读 ID）会在总线上撞包
- 修改参数类命令应答后约 1–4 ms 内电机不接收新命令，`zdt_uart` 自动等待 `write_gap_us`（默认 5 ms）；带存储标志（第 4 字节 `01`）时多为 4–7 ms，擦写 Flash 时约 70 ms，改等 `store_gap_us`（默认 100 ms）
- 固件类型：读驱动配置（`42 6C`）的第 3 字节（整帧长度）Emm 为 `0x21`、X 为 `0x25`。不要用 5.6.4 选项状态的 bit1：Emm 狂暴模式下它也是 0
- 5.6.6 修改固件类型（`D5 69`）可以不存储、运行中切换，立即生效，断电恢复；真机切换时未触发校准旋转，位置连续

## CAN 帧规则（手册 4.2，p.41–43）

- 扩展帧，ID = `(Addr << 8) | Packet`，`Packet` 从 0 起
- 数据场首字节是功能码，**地址只在 ID 里**
- 超过 8 字节的命令拆包：每个包 = 功能码 + 至多 7 个命令字节，**续包同样以功能码开头**；校验码只在最后一包

例（手册 p.43）：`01 FD 01 0F A0 00 00 01 FA 00 00 00 6B` →
`ID 0x0100: FD 01 0F A0 00 00 01 FA` ＋ `ID 0x0101: FD 00 00 00 6B`

## 目录

```
include/            公共头文件
src/                实现（构建函数 + 串口分帧 / 收发 + TWAI 收发）
examples/basic/     最小用法
tests/host/         主机单元测试（构建函数、校验、分帧；金样本来自真机）
test_apps/uart_hw/  串口真机测试固件（ESP32-C3 直连电机）
test_apps/e2e/      CAN 端到端测试固件（跑在 QEMU 里）
tools/fake_motor.py 宿主机上的假电机（SocketCAN）
tools/run_e2e.sh    一键运行端到端测试
```

## 主机单元测试

```bash
make -C tests/host test
```

覆盖：33 条构建函数输出与真机接受的帧逐字节比对、98 组真机应答的分帧（含 X 固件）、乱码后重新同步、`00 EE`、`43` 自带长度、XOR / CRC8（CRC8 表与逐位实现全量比对）、CAN 拆包仍与原始帧一致。

## 串口真机测试

`test_apps/uart_hw` 在 ESP32-C3 上经 UART1（TX=GPIO1、RX=GPIO0）直连地址 1、2 的两台电机（X42S、Emm 固件、115200、6B 校验），打印 `HW PASS/FAIL`，最后 `HW SUMMARY`。只发无副作用的命令：28 条读取、store=00 写回同值、`00 EE`、定时返回 + 事件回调、±5° 相对当前位置的小幅运动（10 RPM，测完回到起点）、广播同步、多电机命令、600 次交替读取。不发回零、校准、恢复出厂、改 ID / 波特率 / 驱动配置。

```bash
cd test_apps/uart_hw && idf.py -p /dev/ttyACM0 flash monitor
```

## CAN 端到端测试（不需要真电机）

固件在 QEMU（`esp32c3`，模拟了 TWAI）里运行，经宿主机的虚拟 CAN 网卡与 `fake_motor.py` 通信。假电机按手册规则重组多包、校验长度和参数范围，回 `02` / `E2` / `EE`，读命令返回数据，并在应答方式为 Both 时主动发 `9F`。

```bash
# 一次性准备
python $IDF_PATH/tools/idf_tools.py install qemu-riscv32      # 需要带 TWAI 的版本（esp_develop_9.2.2 起）
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0   # 重启后失效

# 运行
source $IDF_PATH/export.sh
tools/run_e2e.sh
```

当前 19 个用例，覆盖：单帧命令、位置命令的 2 包拆分、回零参数 3 包、驱动配置 5 包、读命令数据、`E2`/`EE`、无应答超时、`02` 后接 `9F`。

## 已知限制

- **真机主要验证了串口 + Emm 固件**（X42S，固件 00CD）。82 条命令的请求格式都与手册逐节抽取的结果核对过；其中读取类与上面列出的写入 / 运动命令在真机上收发过。X 固件只验证了 5.6.6 切换、5.3.8 直通限速位置、5.3.10 梯形位置、5.6.21 写窗口、5.8.3 读驱动配置；其余 X 专用命令（力矩、X 速度、限电流、X 的 PID / 驱动配置写入）、回零、校准、恢复出厂、改 ID / 驱动配置没有真机验证；X 固件 `21`（读 PID）的应答长度 19 字节是按手册字段推算的。
- 手册与真机不符之处：5.6.21 修改到位窗口按手册是 7 字节 BE16，**X 固件接受，Emm 固件回 `00 EE`**；Emm 接受的 6 字节帧会把校验字节当成数值低字节（写 `03` 得到 `0x036B`），不能用。窗口参数两种固件共用，Emm 下要改可临时不存储切到 X 写入再切回。5.6.4 读选项状态应答是 2 字节；5.5.1 停止定时返回的确认是 4 字节 `Addr 11 02 6B`。此固件不接受 5.6.7 修改开环 / 闭环（回 `00 EE`）。
- XOR / CRC8 校验只按手册附录实现并做了单元测试，没有在真机上切换校验方式验证（切换需要改驱动配置）。
- 超过 8 字节的**应答**如何拆包，手册没有示例；`zdt_twai_recv_reply` 按与命令相同的规则（续包重复功能码）重组，未经真机验证。
- 同功能码在 X 固件与 Emm 固件下含义不同（如 `F6`、`FD`、`4F 71`）。本库不自动判断固件类型，调用方必须选对 `…X…` / `…Emm…` 函数。
- 假电机的命令长度表取自手册，但它不是电机固件的行为记录；它能证明收发链路和帧格式自洽，不能代替真机验证。

## 许可证

代码以 [GPL-3.0-only](./LICENSE) 授权。仓库中附带的 ZDT 用户手册 PDF 是深圳张大头电子的文档，**不在上述许可范围内**，也不会被打进组件包。
