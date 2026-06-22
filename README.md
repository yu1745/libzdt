# libzdt

> C99 库，用于构建 **ZDT_X42S 第二代闭环步进电机**（及同协议的 Y42 / Emm 固件变体）的串口/CAN 主机命令帧。

`libzdt` 只做一件事：把你要发起的动作翻译成电机协议要求的**精确字节序列**。不依赖任何硬件抽象、不发起 IO、不算校验码（按手册约定末尾固定 `0x6B`）。你可以拿到的字节直接喂给 UART / CAN / socket / 单元测试，随便你。

- 协议来源：《ZDT_X42S 第二代闭环步进电机用户手册 V1.0.3》（251224）——原版 PDF 已随仓库置于根目录：[`ZDT_X42S第二代闭环步进电机用户手册V1.0.3_251224.pdf`](./ZDT_X42S第二代闭环步进电机用户手册V1.0.3_251224.pdf)，作为本库所有命令字节的**唯一权威来源**
- 覆盖范围：手册第 5 章 **5.2–5.8**，共 **82 条主机命令**全部实现
- 所有命令的字节输出都**逐字节对齐手册示例**，并由 82 个单元测试锁定

## 为什么用它

写步进电机上位机最烦的一件事，是反复翻手册、数偏移、数大小端、数校验码。`libzdt` 把这件事固化成类型安全的 C 函数：

```c
uint8_t buf[16];
int n = zdtBuildTrapezoidPosModeCmd(
    0x01,               /* 地址 */
    ZDT_DIR_CW,         /* 方向 */
    500, 500,           /* 加/减速度 RPM/s */
    3000,               /* 最大速度 ×0.1 RPM */
    3600,               /* 目标角度 ×0.1° */
    ZDT_MOVE_REL_LAST,  /* 相对上一目标 */
    ZDT_SYNC_NOW,       /* 立即执行 */
    buf, sizeof buf);
/* n == 16, buf 即可直接发送 */
```

不用再手写 `buf[3] = (speed >> 8) & 0xFF;` 这种代码，也不用记 `0x6B` 写在帧尾还是帧头。

## 特性

- **纯 C99**，无外部依赖，无 malloc，无全局可变状态，中断/裸机友好
- **82 个 `zdtBuildXxxCmd` 函数**，覆盖 5.2 触发动作、5.3 运动控制、5.4 原点回零、5.5 读取系统参数、5.6 读写驱动参数、5.7 上电自动运行、5.8 读取所有驱动参数
- **大端序（Big-Endian）** 编码所有多字节数值，与手册示例一致
- **防御式 API**：每个函数都做缓冲区长度与参数检查，返回写入字节数（`>=0`）或负错误码
- **C++ 兼容**：头文件有 `extern "C"` 守卫
- **82/82 单元测试通过**，每个用例的期望字节都来自手册示例

## 目录结构

```
libzdt/
├── src/
│   ├── libzdt.h        # 公共 API，每个函数有详细注释（字段顺序/单位/范围）
│   └── libzdt.c        # 实现
├── examples/
│   └── demo.c          # 最小用法示例
├── tests/
│   └── test_zdt.c      # 82 个逐字节比对的单元测试
├── ZDT_X42S第二代闭环步进电机用户手册V1.0.3_251224.pdf   # 权威协议来源
└── CMakeLists.txt
```

## 编译

### CMake（推荐）

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build            # 跑 82 个测试
./build/zdt_demo                  # 跑示例
```

### 直接 gcc

```bash
# 编译并运行测试
gcc -std=c99 -Wall -Wextra -Isrc tests/test_zdt.c src/libzdt.c -o test_zdt
./test_zdt

# 编译示例
gcc -std=c99 -Wall -Wextra -Isrc examples/demo.c src/libzdt.c -o demo
./demo
```

预期输出末尾：

```
TOTAL: 82 PASS / 0 FAIL
```

## 快速上手

```c
#include "libzdt.h"
#include <stdio.h>

static void dump(const char *tag, const uint8_t *buf, int len) {
    printf("%s [%dB]:", tag, len);
    for (int i = 0; i < len; ++i) printf(" %02X", buf[i]);
    printf("\n");
}

int main(void) {
    uint8_t buf[64];
    int n;

    /* 5.2.1 触发编码器校准 → 01 06 45 6B */
    n = zdtBuildEncoderCalibrationCmd(0x01, buf, sizeof buf);
    dump("encoder cal", buf, n);

    /* 5.3.13 立即停止 → 01 FE 98 00 6B */
    n = zdtBuildImmediateStopCmd(0x01, ZDT_SYNC_NOW, buf, sizeof buf);
    dump("stop", buf, n);

    /* 5.6.1 修改电机地址（保存到 Flash）→ 01 AE 4B 01 02 6B */
    n = zdtBuildChangeAddrCmd(0x01, ZDT_STORE_YES, 0x02, buf, sizeof buf);
    dump("change addr", buf, n);

    return 0;
}
```

`examples/demo.c` 里有更多命令的示范。

## API 概览

所有函数返回 `int`：`>=0` 为写入 `buf` 的字节数，`<0` 为错误（`ZDT_ERR_BUF_TOO_SMALL = -1`，`ZDT_ERR_PARAM = -2`）。

通用帧格式：

```
Addr(1B) | FuncCode(1B) | [AuxCode/Params...] | CheckSum(1B = 0x6B)
```

按手册章节分组（数字为命令数）：

| 章节 | 主题 | 函数前缀 | 数量 |
|------|------|----------|------|
| 5.2 | 触发动作（校准/重启/清零/解锁/出厂复位） | `zdtBuild*Cmd` | 5 |
| 5.3 | 运动控制（使能/力矩/速度/位置/梯形/多电机/停止/同步） | `zdtBuild*Cmd` | 14 |
| 5.4 | 原点回零（设零点/触发/中断/读状态/读写参数） | `zdtBuild*Cmd` | 6 |
| 5.5 | 读取系统参数（版本/电压/电流/位置/转速/温度/IO…） | `zdtBuildRead*Cmd` | 18 |
| 5.6 | 读写驱动参数（ID/细分/PID/DMX512/保护阈值…） | `zdtBuild*Cmd` | 31 |
| 5.7 | 上电自动运行（X / Emm） | `zdtBuildStoreAutoRun*Cmd` | 2 |
| 5.8 | 读取/修改所有驱动参数（X / Emm） | `zdtBuildReadAll*` / `zdtBuildWriteAllConfig*Cmd` | 6 |

常用常量（定义在 `libzdt.h`）：

```c
ZDT_ADDR_BROADCAST     0x00   /* 广播地址 */
ZDT_CHECKSUM_DEFAULT   0x6B   /* 手册约定的固定校验码 */
ZDT_SYNC_NOW           0x00   /* 立即执行 */
ZDT_SYNC_CACHE         0x01   /* 缓存命令，等同步触发 */
ZDT_STORE_NO           0x00   /* 掉电丢失 */
ZDT_STORE_YES          0x01   /* 存入 Flash */
ZDT_DIR_CW             0x00   /* 顺时针 */
ZDT_DIR_CCW            0x01   /* 逆时针 */
ZDT_MOVE_REL_LAST      0x00   /* 相对上一目标位置 */
ZDT_MOVE_ABS_ZERO      0x01   /* 相对坐标零点（绝对） */
ZDT_MOVE_REL_NOW       0x02   /* 相对当前实时位置 */
```

每个函数的**字段顺序、单位、取值范围**都在 `src/libzdt.h` 的注释里写明，可直接当文档读。

## 约束与设计取舍

- **只构建，不收发。** 本库不包含 UART/CAN 驱动、不含应答解析、不重试。拿到字节后怎么发、发完怎么收，由你的应用层决定。
- **校验码固定 `0x6B`。** 手册第 4 章规定主机命令以固定字节 `0x6B` 结尾（不是真校验和）。如果你的固件版本要求真实校验和，自行替换最后一字节即可。
- **大端序。** 所有 BE16/BE32 数值按手册示例采用大端序。极少数手册注释写成"小端"但示例实为大端的命令，以**手册示例字节为准**（已在测试中锁定）。
- **线程安全。** 无全局可变状态，所有函数只读写调用者提供的 `buf`，可安全用于多线程。

## 测试

`tests/test_zdt.c` 为 82 条命令各写了一个用例，每个用例：

1. 调用对应 `zdtBuildXxxCmd`；
2. 与从手册示例抄录的期望字节序列做 **`memcmp` 逐字节比对**（不是只查长度或前缀）；
3. 不一致即打印实际/期望十六进制并计为失败。

这保证"测试通过"等价于"输出字节与手册示例完全一致"。

```bash
gcc -std=c99 -Wall -Wextra -Isrc tests/test_zdt.c src/libzdt.c -o test_zdt && ./test_zdt | tail -1
# TOTAL: 82 PASS / 0 FAIL
```

## 许可证

代码（`src/`、`tests/`、`examples/`）以 **MIT** 许可发布。

根目录的 `ZDT_X42S第二代闭环步进电机用户手册V1.0.3_251224.pdf` 为 ZDT 原厂用户手册，**版权归原厂所有**；本仓库仅出于"协议字节权威来源"的可追溯目的完整收录，不对其做任何修改。如有版权方反对收录，请开 issue，收到后会立即移除。
