/*
 * zdt_common_sys.c - 通用读取系统参数模块 (CAN 扩展帧构建)
 *
 * 纯 C99，无任何硬件平台或操作系统依赖，零动态内存分配。
 */

#include "zdt_common_sys.h"
#include "zdt_internal.h"

/* ====================================================================
 *  5.5 读取系统参数命令 (共 18 条)
 * ==================================================================== */

/* 5.5.1 定时返回信息命令 (X42S/Y42)
 * Addr + 11 + 18 + 信息功能码 + 定时时间(BE16, ms) + 6B — 7B */
static int _raw_SetPeriodicReportCmd(uint8_t addr, uint8_t info_func_code,
                                    uint16_t interval_ms,
                                    uint8_t *buf, size_t buf_size)
{
    const size_t need = 7;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x11);
    zdt_append_u8(buf, &i, buf_size, 0x18);
    zdt_append_u8(buf, &i, buf_size, info_func_code);
    zdt_append_u16_be(buf, &i, buf_size, interval_ms);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildSetPeriodicReportCmd(uint8_t addr, uint8_t info_func_code,
                                   uint16_t interval_ms,
                                   zdt_can_msg_t *msg)
{
    uint8_t raw[7];
    int n = _raw_SetPeriodicReportCmd(addr, info_func_code, interval_ms, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.2 读取固件版本和硬件版本
 * Addr + 1F + 6B — 3B (1F 对应功能码 0x1F) */
static int _raw_ReadVersionCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x1F);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadVersionCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadVersionCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.3 读取相电阻和相电感
 * Addr + 20 + 6B — 3B (20 对应功能码 0x20) */
static int _raw_ReadPhaseRLCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x20);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadPhaseRLCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadPhaseRLCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.4 读取总线电压
 * Addr + 24 + 6B — 3B (24 对应功能码 0x24) */
static int _raw_ReadBusVoltageCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x24);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadBusVoltageCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadBusVoltageCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.5 读取总线电流 (X42S/Y42)
 * Addr + 26 + 6B — 3B (26 对应功能码 0x26) */
static int _raw_ReadBusCurrentCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x26);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadBusCurrentCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadBusCurrentCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.6 读取相电流
 * Addr + 27 + 6B — 3B (27 对应功能码 0x27) */
static int _raw_ReadPhaseCurrentCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x27);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadPhaseCurrentCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadPhaseCurrentCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.7 读取经过线性化校准后的编码器值
 * Addr + 31 + 6B — 3B (31 对应功能码 0x31) */
static int _raw_ReadEncoderCalibratedCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x31);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadEncoderCalibratedCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadEncoderCalibratedCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.8 读取输入脉冲数
 * Addr + 32 + 6B — 3B (32 对应功能码 0x32) */
static int _raw_ReadInputPulsesCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x32);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadInputPulsesCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadInputPulsesCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.9 读取电机目标位置
 * Addr + 33 + 6B — 3B (33 对应功能码 0x33) */
static int _raw_ReadTargetPosCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x33);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadTargetPosCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadTargetPosCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.10 读取电机实时设定的目标位置
 * Addr + 34 + 6B — 3B (34 对应功能码 0x34) */
static int _raw_ReadRealtimeTargetPosCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x34);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadRealtimeTargetPosCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadRealtimeTargetPosCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.11 读取电机实时转速
 * Addr + 35 + 6B — 3B (35 对应功能码 0x35) */
static int _raw_ReadRealtimeSpeedCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x35);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadRealtimeSpeedCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadRealtimeSpeedCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.12 读取驱动温度 (X42S/Y42)
 * Addr + 39 + 6B — 3B (39 对应功能码 0x39) */
static int _raw_ReadDriverTempCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x39);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadDriverTempCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadDriverTempCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.13 读取电机实时位置
 * Addr + 36 + 6B — 3B (36 对应功能码 0x36) */
static int _raw_ReadRealtimePosCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x36);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadRealtimePosCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadRealtimePosCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.14 读取电机位置误差
 * Addr + 37 + 6B — 3B (37 对应功能码 0x37) */
static int _raw_ReadPosErrorCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x37);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadPosErrorCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadPosErrorCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.15 读取电机状态标志
 * Addr + 3A + 6B — 3B (3A 对应功能码 0x3A) */
static int _raw_ReadMotorStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x3A);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadMotorStatusCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadMotorStatusCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.16 读取回零状态标志 + 电机状态标志 (X42S/Y42)
 * Addr + 3C + 6B — 3B (3C 对应功能码 0x3C) */
static int _raw_ReadHomingAndStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x3C);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadHomingAndStatusCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadHomingAndStatusCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.17 读取引脚 IO 电平状态 (X42S/Y42)
 * Addr + 3D + 6B — 3B (3D 对应功能码 0x3D) */
static int _raw_ReadIoLevelCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x3D);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadIoLevelCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadIoLevelCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.18 读取电池电压 (Y42)
 * Addr + 38 + 6B — 3B (38 对应功能码 0x38) */
static int _raw_ReadBatteryVoltageCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x38);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadBatteryVoltageCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadBatteryVoltageCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}
