# libzdt

ZDT_X42S 第二代闭环步进电机（含 Y42 / Emm 固件变体）的 **ESP-IDF 组件**：构建手册第 5 章 5.2–5.8 的 82 条主机命令的 CAN 扩展帧，并用 TWAI 驱动完成发送、多包拆装和应答接收。

- 要求 ESP-IDF ≥ 6.1（使用新的基于节点的 TWAI 驱动 `esp_driver_twai`）
- 协议依据：[《ZDT_X42S 第二代闭环步进电机用户手册 V1.0.3》](./ZDT_X42S第二代闭环步进电机用户手册V1.0.3_251224.pdf)，随仓库附带
- 只支持 CAN（TWAI），不含 UART / Modbus；校验码固定 `0x6B`

## 用法

```c
#include "libzdt.h"

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

两层 API：

| 层 | 头文件 | 作用 |
|---|---|---|
| 构建 | `zdt_common_*.h` `zdt_x.h` `zdt_emm.h` | `zdtCanBuild*Cmd(...)` 把参数编码成 `zdt_can_msg_t`（1–8 个 CAN 帧）。纯计算，不访问硬件 |
| 收发 | `zdt_twai.h` | `zdt_twai_new/del`、`zdt_twai_send`、`zdt_twai_recv_reply`、`zdt_twai_transact` |

应答（`zdt_reply_t`）：`code` 为功能码，`data` 为功能码之后、校验字节之前的内容，状态类命令的 `data[0]` 是 `ZDT_REPLY_OK(02)` / `ZDT_REPLY_PARAM_ERR(E2)` / `ZDT_REPLY_FORMAT_ERR(EE)`；位置到位、回零完成、夹爪夹紧时电机会主动再发一帧 `ZDT_REPLY_DONE(9F)`，用 `zdt_twai_recv_reply` 单独接收。

## CAN 帧规则（手册 4.2，p.41–43）

- 扩展帧，ID = `(Addr << 8) | Packet`，`Packet` 从 0 起
- 数据场首字节是功能码，**地址只在 ID 里**
- 超过 8 字节的命令拆包：每个包 = 功能码 + 至多 7 个命令字节，**续包同样以功能码开头**；校验码只在最后一包

例（手册 p.43）：`01 FD 01 0F A0 00 00 01 FA 00 00 00 6B` →
`ID 0x0100: FD 01 0F A0 00 00 01 FA` ＋ `ID 0x0101: FD 00 00 00 6B`

## 目录

```
include/            公共头文件
src/                实现（构建函数 + TWAI 收发）
examples/basic/     最小用法
test_apps/e2e/      端到端测试固件（跑在 QEMU 里）
tools/fake_motor.py 宿主机上的假电机（SocketCAN）
tools/run_e2e.sh    一键运行端到端测试
```

## 端到端测试（不需要真电机）

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

- **82 条命令里只有 9 条经过端到端验证**（使能、位置清零、速度(Emm)、位置(Emm)、立即停止、读总线电压、读实时位置、修改回零参数、修改驱动配置(Emm)）。其余命令的字节输出尚未逐条对照手册示例核验，不要假定它们已被锁定。
- 超过 8 字节的**应答**如何拆包，手册没有示例；`zdt_twai_recv_reply` 按与命令相同的规则（续包重复功能码）重组，未经真机验证。
- 同功能码在 X 固件与 Emm 固件下含义不同（如 `F6`、`FD`、`4F 71`）。本库不自动判断固件类型，调用方必须选对 `…X…` / `…Emm…` 函数。
- 假电机的命令长度表取自手册，但它不是电机固件的行为记录；它能证明收发链路和帧格式自洽，不能代替真机验证。

## 许可证

代码以 [GPL-3.0-only](./LICENSE) 授权。仓库中附带的 ZDT 用户手册 PDF 是深圳张大头电子的文档，**不在上述许可范围内**，也不会被打进组件包。
