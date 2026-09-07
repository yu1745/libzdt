/*
 * zdt_emm.c - ZDT Emm 固件专属原生 CAN 命令构建实现
 *
 * 纯 C99，无任何硬件平台或操作系统依赖，零动态内存分配。
 */

#include "zdt_emm.h"
#include "zdt_internal.h"

/* ====================================================================
 *  5.3 运动控制命令 (Emm 固件特有)
 * ==================================================================== */

/* 5.3.7 速度模式控制 (Emm)
 * Addr + F6 + 方向 + 速度(BE16) + 加速度(1B) + 同步 + 6B — 8B */
static int _raw_SpeedModeEmmCmd(uint8_t addr, uint8_t dir, uint16_t rpm,
                               uint8_t acc, uint8_t sync,
                               uint8_t *buf, size_t buf_size)
{
    const size_t need = 8;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0xF6);
    zdt_append_u8(buf, &i, buf_size, dir);
    zdt_append_u16_be(buf, &i, buf_size, rpm);
    zdt_append_u8(buf, &i, buf_size, acc);
    zdt_append_u8(buf, &i, buf_size, sync);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildSpeedModeEmmCmd(uint8_t addr, uint8_t dir, uint16_t rpm,
                              uint8_t acc, uint8_t sync,
                              zdt_can_msg_t *msg)
{
    uint8_t raw[8];
    int n = _raw_SpeedModeEmmCmd(addr, dir, rpm, acc, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.3.12 位置模式控制 (Emm)
 * Addr + FD + 方向 + 速度(BE16) + 加速度(1B；0=不使用曲线加减速) + 脉冲数(BE32) + 运动模式 + 同步 + 6B — 13B */
static int _raw_PosModeEmmCmd(uint8_t addr, uint8_t dir, uint16_t rpm,
                             uint8_t acc, uint32_t pulses,
                             uint8_t move_mode, uint8_t sync,
                             uint8_t *buf, size_t buf_size)
{
    const size_t need = 13;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0xFD);
    zdt_append_u8(buf, &i, buf_size, dir);
    zdt_append_u16_be(buf, &i, buf_size, rpm);
    zdt_append_u8(buf, &i, buf_size, acc);
    zdt_append_u32_be(buf, &i, buf_size, pulses);
    zdt_append_u8(buf, &i, buf_size, move_mode);
    zdt_append_u8(buf, &i, buf_size, sync);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildPosModeEmmCmd(uint8_t addr, uint8_t dir, uint16_t rpm,
                             uint8_t acc, uint32_t pulses,
                             uint8_t move_mode, uint8_t sync,
                             zdt_can_msg_t *msg)
{
    uint8_t raw[13];
    int n = _raw_PosModeEmmCmd(addr, dir, rpm, acc, pulses, move_mode, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* ====================================================================
 *  5.6 修改驱动参数命令 (Emm 固件特有)
 * ==================================================================== */

/* 5.6.11 修改命令速度值是否缩小 10 倍输入 (Emm)
 * Addr + 4F + 71 + store + scale + 6B — 6B */
static int _raw_ChangeSpeedScaleCmd(uint8_t addr, uint8_t store,
                                    uint8_t scale_10x,
                                    uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x4F);
    zdt_append_u8(buf, &i, buf_size, 0x71);
    zdt_append_u8(buf, &i, buf_size, store);
    zdt_append_u8(buf, &i, buf_size, scale_10x);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeSpeedScaleCmd(uint8_t addr, uint8_t store,
                                   uint8_t scale_10x,
                                   zdt_can_msg_t *msg)
{
    uint8_t raw[6];
    int n = _raw_ChangeSpeedScaleCmd(addr, store, scale_10x, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.6.16 读取 PID 参数 (Emm)
 * Addr + 21 + 6B — 3B */
static int _raw_ReadPidEmmCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x21);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadPidEmmCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadPidEmmCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.6.17 修改 PID 参数 (Emm)
 * Addr + 4A + C3 + store + Kp(BE32) + Ki(BE32) + Kd(BE32) + 6B — 17B */
static int _raw_WritePidEmmCmd(uint8_t addr, uint8_t store,
                              uint32_t kp, uint32_t ki, uint32_t kd,
                              uint8_t *buf, size_t buf_size)
{
    const size_t need = 17;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x4A);
    zdt_append_u8(buf, &i, buf_size, 0xC3);
    zdt_append_u8(buf, &i, buf_size, store);
    zdt_append_u32_be(buf, &i, buf_size, kp);
    zdt_append_u32_be(buf, &i, buf_size, ki);
    zdt_append_u32_be(buf, &i, buf_size, kd);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWritePidEmmCmd(uint8_t addr, uint8_t store,
                              uint32_t kp, uint32_t ki, uint32_t kd,
                              zdt_can_msg_t *msg)
{
    uint8_t raw[17];
    int n = _raw_WritePidEmmCmd(addr, store, kp, ki, kd, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* ====================================================================
 *  5.7 上电自动运行命令 (Emm 固件特有)
 * ==================================================================== */

/* 5.7.2 存储一组速度参数，上电自动运行 (Emm)
 * Addr + F7 + 1C + store + dir + speed(BE16) + acc(1B) + en + 6B — 10B */
static int _raw_StoreAutoRunEmmCmd(uint8_t addr, uint8_t store,
                                   uint8_t dir, uint16_t speed_rpm,
                                   uint8_t acc_level, uint8_t en_pin_enable,
                                   uint8_t *buf, size_t buf_size)
{
    const size_t need = 10;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0xF7);
    zdt_append_u8(buf, &i, buf_size, 0x1C);
    zdt_append_u8(buf, &i, buf_size, store);
    zdt_append_u8(buf, &i, buf_size, dir);
    zdt_append_u16_be(buf, &i, buf_size, speed_rpm);
    zdt_append_u8(buf, &i, buf_size, acc_level);
    zdt_append_u8(buf, &i, buf_size, en_pin_enable);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildStoreAutoRunEmmCmd(uint8_t addr, uint8_t store,
                                  uint8_t dir, uint16_t speed_rpm,
                                  uint8_t acc_level, uint8_t en_pin_enable,
                                  zdt_can_msg_t *msg)
{
    uint8_t raw[10];
    int n = _raw_StoreAutoRunEmmCmd(addr, store, dir, speed_rpm, acc_level, en_pin_enable, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* ====================================================================
 *  5.8 读取与修改所有驱动参数命令 (Emm 固件特有)
 * ==================================================================== */

/* 5.8.2 读取系统状态参数 (Emm)
 * Addr + 43 + 7A + 6B — 4B */
static int _raw_ReadAllStatusEmmCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x43);
    zdt_append_u8(buf, &i, buf_size, 0x7A);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadAllStatusEmmCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_ReadAllStatusEmmCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.8.5 读取驱动配置参数 (Emm)
 * Addr + 42 + 6C + 6B — 4B */
static int _raw_ReadAllConfigEmmCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x42);
    zdt_append_u8(buf, &i, buf_size, 0x6C);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadAllConfigEmmCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_ReadAllConfigEmmCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.8.6 修改驱动配置参数 (Emm)
 * Addr + 48 + D1 + 参数列表... + 6B — 33B */
static int _raw_WriteAllConfigEmmCmd(uint8_t addr, uint8_t store,
                                     uint8_t motor_type,
                                     uint8_t pul_mode, uint8_t com_mode,
                                     uint8_t en_level, uint8_t dir_level,
                                     uint8_t microstep, uint8_t interp,
                                     uint16_t open_current_ma,
                                     uint16_t stall_current_ma,
                                     uint16_t max_voltage_code,
                                     uint8_t uart_baud, uint8_t can_speed,
                                     uint8_t check_mode, uint8_t reply_mode,
                                     uint8_t stall_protect,
                                     uint16_t stall_speed_rpm,
                                     uint16_t stall_current_ma2,
                                     uint16_t stall_time_ms,
                                     uint16_t pos_window,
                                     uint8_t *buf, size_t buf_size)
{
    const size_t need = 33;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x48);
    zdt_append_u8(buf, &i, buf_size, 0xD1);
    zdt_append_u8(buf, &i, buf_size, store);
    zdt_append_u8(buf, &i, buf_size, motor_type);
    zdt_append_u8(buf, &i, buf_size, pul_mode);
    zdt_append_u8(buf, &i, buf_size, com_mode);
    zdt_append_u8(buf, &i, buf_size, en_level);
    zdt_append_u8(buf, &i, buf_size, dir_level);
    zdt_append_u8(buf, &i, buf_size, microstep);
    zdt_append_u8(buf, &i, buf_size, interp);
    zdt_append_u8(buf, &i, buf_size, 0x00);
    zdt_append_u16_be(buf, &i, buf_size, open_current_ma);
    zdt_append_u16_be(buf, &i, buf_size, stall_current_ma);
    zdt_append_u16_be(buf, &i, buf_size, max_voltage_code);
    zdt_append_u8(buf, &i, buf_size, uart_baud);
    zdt_append_u8(buf, &i, buf_size, can_speed);
    zdt_append_u8(buf, &i, buf_size, 0x00);
    zdt_append_u8(buf, &i, buf_size, check_mode);
    zdt_append_u8(buf, &i, buf_size, reply_mode);
    zdt_append_u8(buf, &i, buf_size, stall_protect);
    zdt_append_u16_be(buf, &i, buf_size, stall_speed_rpm);
    zdt_append_u16_be(buf, &i, buf_size, stall_current_ma2);
    zdt_append_u16_be(buf, &i, buf_size, stall_time_ms);
    zdt_append_u16_be(buf, &i, buf_size, pos_window);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWriteAllConfigEmmCmd(uint8_t addr, uint8_t store,
                                    uint8_t motor_type,
                                    uint8_t pul_mode, uint8_t com_mode,
                                    uint8_t en_level, uint8_t dir_level,
                                    uint8_t microstep, uint8_t interp,
                                    uint16_t open_current_ma,
                                    uint16_t stall_current_ma,
                                    uint16_t max_voltage_code,
                                    uint8_t uart_baud, uint8_t can_speed,
                                    uint8_t check_mode, uint8_t reply_mode,
                                    uint8_t stall_protect,
                                    uint16_t stall_speed_rpm,
                                    uint16_t stall_current_ma2,
                                    uint16_t stall_time_ms,
                                    uint16_t pos_window,
                                    zdt_can_msg_t *msg)
{
    uint8_t raw[33];
    int n = _raw_WriteAllConfigEmmCmd(addr, store, motor_type, pul_mode, com_mode,
                                      en_level, dir_level, microstep, interp,
                                      open_current_ma, stall_current_ma, max_voltage_code,
                                      uart_baud, can_speed, check_mode, reply_mode,
                                      stall_protect, stall_speed_rpm, stall_current_ma2,
                                      stall_time_ms, pos_window, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}
