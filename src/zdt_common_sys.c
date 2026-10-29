/* SPDX-License-Identifier: GPL-3.0-only */
/*
 * zdt_common_sys.c - 通用读取系统参数模块 (串口 TTL 原始帧 / CAN 扩展帧构建)
 *
 * 构建函数只做字节编码、不访问硬件，零动态内存分配。
 */

#include "zdt_common_sys.h"
#include "zdt_internal.h"

/* ====================================================================
 *  5.5 读取系统参数命令 (共 18 条)
 * ==================================================================== */

/* 5.5.1 定时返回信息命令 (X42S/Y42)
 * Addr + 11 + 18 + 信息功能码 + 定时时间(BE16, ms) + 6B — 7B */
int zdtBuildSetPeriodicReportCmd(uint8_t addr, uint8_t info_func_code,
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
    int n = zdtBuildSetPeriodicReportCmd(addr, info_func_code, interval_ms, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.2 读取固件版本和硬件版本
 * Addr + 1F + 6B — 3B (1F 对应功能码 0x1F) */
int zdtBuildReadVersionCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadVersionCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.3 读取相电阻和相电感
 * Addr + 20 + 6B — 3B (20 对应功能码 0x20) */
int zdtBuildReadPhaseRLCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadPhaseRLCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.4 读取总线电压
 * Addr + 24 + 6B — 3B (24 对应功能码 0x24) */
int zdtBuildReadBusVoltageCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadBusVoltageCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.5 读取总线电流 (X42S/Y42)
 * Addr + 26 + 6B — 3B (26 对应功能码 0x26) */
int zdtBuildReadBusCurrentCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadBusCurrentCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.6 读取相电流
 * Addr + 27 + 6B — 3B (27 对应功能码 0x27) */
int zdtBuildReadPhaseCurrentCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadPhaseCurrentCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.7 读取经过线性化校准后的编码器值
 * Addr + 31 + 6B — 3B (31 对应功能码 0x31) */
int zdtBuildReadEncoderCalibratedCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadEncoderCalibratedCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.8 读取输入脉冲数
 * Addr + 32 + 6B — 3B (32 对应功能码 0x32) */
int zdtBuildReadInputPulsesCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadInputPulsesCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.9 读取电机目标位置
 * Addr + 33 + 6B — 3B (33 对应功能码 0x33) */
int zdtBuildReadTargetPosCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadTargetPosCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.10 读取电机实时设定的目标位置
 * Addr + 34 + 6B — 3B (34 对应功能码 0x34) */
int zdtBuildReadRealtimeTargetPosCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadRealtimeTargetPosCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.11 读取电机实时转速
 * Addr + 35 + 6B — 3B (35 对应功能码 0x35) */
int zdtBuildReadRealtimeSpeedCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadRealtimeSpeedCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.12 读取驱动温度 (X42S/Y42)
 * Addr + 39 + 6B — 3B (39 对应功能码 0x39) */
int zdtBuildReadDriverTempCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadDriverTempCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.13 读取电机实时位置
 * Addr + 36 + 6B — 3B (36 对应功能码 0x36) */
int zdtBuildReadRealtimePosCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadRealtimePosCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.14 读取电机位置误差
 * Addr + 37 + 6B — 3B (37 对应功能码 0x37) */
int zdtBuildReadPosErrorCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadPosErrorCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.15 读取电机状态标志
 * Addr + 3A + 6B — 3B (3A 对应功能码 0x3A) */
int zdtBuildReadMotorStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadMotorStatusCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.16 读取回零状态标志 + 电机状态标志 (X42S/Y42)
 * Addr + 3C + 6B — 3B (3C 对应功能码 0x3C) */
int zdtBuildReadHomingAndStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadHomingAndStatusCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.17 读取引脚 IO 电平状态 (X42S/Y42)
 * Addr + 3D + 6B — 3B (3D 对应功能码 0x3D) */
int zdtBuildReadIoLevelCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadIoLevelCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.5.18 读取电池电压 (Y42)
 * Addr + 38 + 6B — 3B (38 对应功能码 0x38) */
int zdtBuildReadBatteryVoltageCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
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
    int n = zdtBuildReadBatteryVoltageCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}
