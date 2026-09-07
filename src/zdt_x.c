/*
 * zdt_x.c - ZDT_X42S X 固件专属原生 CAN 命令构建实现
 *
 * 纯 C99，无任何硬件平台或操作系统依赖，零动态内存分配。
 */

#include "zdt_x.h"
#include "zdt_internal.h"

/* ====================================================================
 *  5.3 运动控制命令 (X 固件特有)
 * ==================================================================== */

/* 5.3.3 力矩模式控制 (X)
 * Addr + F5 + 符号 + 斜率(BE16) + 电流(BE16) + 同步 + 6B — 9B */
static int _raw_TorqueModeCmd(uint8_t addr, uint8_t dir,
                              uint16_t slope_ma_s, uint16_t current_ma,
                              uint8_t sync, uint8_t *buf, size_t max_len)
{
    const size_t need = 9;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xF5);
    zdt_append_u8(buf, &i, max_len, dir);
    zdt_append_u16_be(buf, &i, max_len, slope_ma_s);
    zdt_append_u16_be(buf, &i, max_len, current_ma);
    zdt_append_u8(buf, &i, max_len, sync);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildTorqueModeCmd(uint8_t addr, uint8_t dir,
                             uint16_t slope_ma_s, uint16_t current_ma,
                             uint8_t sync, zdt_can_msg_t *msg)
{
    uint8_t raw[9];
    int n = _raw_TorqueModeCmd(addr, dir, slope_ma_s, current_ma, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.3.4 力矩模式限速控制 (X)
 * Addr + C5 + 符号 + 斜率(BE16) + 电流(BE16) + 同步 + 最大速度(BE16) + 6B — 11B */
static int _raw_TorqueModeSpeedLimitCmd(uint8_t addr, uint8_t dir,
                                         uint16_t slope_ma_s, uint16_t current_ma,
                                         uint16_t max_rpm_x10,
                                         uint8_t sync, uint8_t *buf, size_t max_len)
{
    const size_t need = 11;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xC5);
    zdt_append_u8(buf, &i, max_len, dir);
    zdt_append_u16_be(buf, &i, max_len, slope_ma_s);
    zdt_append_u16_be(buf, &i, max_len, current_ma);
    zdt_append_u8(buf, &i, max_len, sync);
    zdt_append_u16_be(buf, &i, max_len, max_rpm_x10);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildTorqueModeSpeedLimitCmd(uint8_t addr, uint8_t dir,
                                       uint16_t slope_ma_s, uint16_t current_ma,
                                       uint16_t max_rpm_x10,
                                       uint8_t sync, zdt_can_msg_t *msg)
{
    uint8_t raw[11];
    int n = _raw_TorqueModeSpeedLimitCmd(addr, dir, slope_ma_s, current_ma, max_rpm_x10, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.3.5 速度模式控制 (X)
 * Addr + F6 + 方向 + 加速度(BE16) + 速度(BE16) + 同步 + 6B — 9B */
static int _raw_SpeedModeXCmd(uint8_t addr, uint8_t dir,
                             uint16_t acc_rpm_s, uint16_t rpm_x10,
                             uint8_t sync, uint8_t *buf, size_t max_len)
{
    const size_t need = 9;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xF6);
    zdt_append_u8(buf, &i, max_len, dir);
    zdt_append_u16_be(buf, &i, max_len, acc_rpm_s);
    zdt_append_u16_be(buf, &i, max_len, rpm_x10);
    zdt_append_u8(buf, &i, max_len, sync);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildSpeedModeXCmd(uint8_t addr, uint8_t dir,
                             uint16_t acc_rpm_s, uint16_t rpm_x10,
                             uint8_t sync, zdt_can_msg_t *msg)
{
    uint8_t raw[9];
    int n = _raw_SpeedModeXCmd(addr, dir, acc_rpm_s, rpm_x10, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.3.6 速度模式限电流控制 (X)
 * Addr + C6 + 符号 + 加速度(BE16) + 速度(BE16) + 同步 + 最大电流(BE16) + 6B — 11B */
static int _raw_SpeedModeXCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                         uint16_t acc_rpm_s, uint16_t rpm_x10,
                                         uint16_t max_current_ma,
                                         uint8_t sync, uint8_t *buf, size_t max_len)
{
    const size_t need = 11;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xC6);
    zdt_append_u8(buf, &i, max_len, dir);
    zdt_append_u16_be(buf, &i, max_len, acc_rpm_s);
    zdt_append_u16_be(buf, &i, max_len, rpm_x10);
    zdt_append_u8(buf, &i, max_len, sync);
    zdt_append_u16_be(buf, &i, max_len, max_current_ma);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildSpeedModeXCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                         uint16_t acc_rpm_s, uint16_t rpm_x10,
                                         uint16_t max_current_ma,
                                         uint8_t sync, zdt_can_msg_t *msg)
{
    uint8_t raw[11];
    int n = _raw_SpeedModeXCurrentLimitCmd(addr, dir, acc_rpm_s, rpm_x10, max_current_ma, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.3.8 直通限速位置模式控制 (X)
 * Addr + FB + 方向 + 速度(BE16) + 位置角度(BE32) + 运动模式 + 同步 + 6B — 12B */
static int _raw_PosModePassThroughCmd(uint8_t addr, uint8_t dir,
                                     uint16_t rpm_x10, uint32_t pos_angle,
                                     uint8_t move_mode, uint8_t sync,
                                     uint8_t *buf, size_t max_len)
{
    const size_t need = 12;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xFB);
    zdt_append_u8(buf, &i, max_len, dir);
    zdt_append_u16_be(buf, &i, max_len, rpm_x10);
    zdt_append_u32_be(buf, &i, max_len, pos_angle);
    zdt_append_u8(buf, &i, max_len, move_mode);
    zdt_append_u8(buf, &i, max_len, sync);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildPosModePassThroughCmd(uint8_t addr, uint8_t dir,
                                     uint16_t rpm_x10, uint32_t pos_angle,
                                     uint8_t move_mode, uint8_t sync,
                                     zdt_can_msg_t *msg)
{
    uint8_t raw[12];
    int n = _raw_PosModePassThroughCmd(addr, dir, rpm_x10, pos_angle, move_mode, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.3.9 直通限速位置模式限电流控制 (X)
 * Addr + CB + 方向 + 速度(BE16) + 位置角度(BE32) + 运动模式 + 同步 + 最大电流(BE16) + 6B — 14B */
static int _raw_PosModePassThroughCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                                 uint16_t rpm_x10, uint32_t pos_angle,
                                                 uint8_t move_mode, uint8_t sync,
                                                 uint16_t max_current_ma,
                                                 uint8_t *buf, size_t max_len)
{
    const size_t need = 14;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xCB);
    zdt_append_u8(buf, &i, max_len, dir);
    zdt_append_u16_be(buf, &i, max_len, rpm_x10);
    zdt_append_u32_be(buf, &i, max_len, pos_angle);
    zdt_append_u8(buf, &i, max_len, move_mode);
    zdt_append_u8(buf, &i, max_len, sync);
    zdt_append_u16_be(buf, &i, max_len, max_current_ma);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildPosModePassThroughCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                                 uint16_t rpm_x10, uint32_t pos_angle,
                                                 uint8_t move_mode, uint8_t sync,
                                                 uint16_t max_current_ma,
                                                 zdt_can_msg_t *msg)
{
    uint8_t raw[14];
    int n = _raw_PosModePassThroughCurrentLimitCmd(addr, dir, rpm_x10, pos_angle, move_mode, sync, max_current_ma, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.3.10 梯形曲线加减速位置模式控制 (X)
 * Addr + FD + 方向 + 加速加速度(BE16) + 减速加速度(BE16) + 最大速度(BE16) + 位置角度(BE32) + 运动模式 + 同步 + 6B — 16B */
static int _raw_TrapezoidPosModeCmd(uint8_t addr, uint8_t dir,
                                   uint16_t accel_rpm_s, uint16_t decel_rpm_s,
                                   uint16_t max_speed_x10, uint32_t pos_angle,
                                   uint8_t move_mode, uint8_t sync,
                                   uint8_t *buf, size_t max_len)
{
    const size_t need = 16;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xFD);
    zdt_append_u8(buf, &i, max_len, dir);
    zdt_append_u16_be(buf, &i, max_len, accel_rpm_s);
    zdt_append_u16_be(buf, &i, max_len, decel_rpm_s);
    zdt_append_u16_be(buf, &i, max_len, max_speed_x10);
    zdt_append_u32_be(buf, &i, max_len, pos_angle);
    zdt_append_u8(buf, &i, max_len, move_mode);
    zdt_append_u8(buf, &i, max_len, sync);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildTrapezoidPosModeCmd(uint8_t addr, uint8_t dir,
                                   uint16_t accel_rpm_s, uint16_t decel_rpm_s,
                                   uint16_t max_speed_x10, uint32_t pos_angle,
                                   uint8_t move_mode, uint8_t sync,
                                   zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int n = _raw_TrapezoidPosModeCmd(addr, dir, accel_rpm_s, decel_rpm_s, max_speed_x10, pos_angle, move_mode, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.3.11 梯形曲线加减速位置模式限电流控制 (X)
 * Addr + CD + 方向 + 加速加速度(BE16) + 减速加速度(BE16) + 最大速度(BE16) + 位置角度(BE32) + 运动模式 + 同步 + 最大电流(BE16) + 6B — 18B */
static int _raw_TrapezoidPosModeCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                                uint16_t accel_rpm_s,
                                                uint16_t decel_rpm_s,
                                                uint16_t max_speed,
                                                uint32_t pos_angle,
                                                uint8_t move_mode,
                                                uint8_t sync,
                                                uint16_t max_current_ma,
                                                uint8_t *buf, size_t max_len)
{
    const size_t need = 18;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xCD);
    zdt_append_u8(buf, &i, max_len, dir);
    zdt_append_u16_be(buf, &i, max_len, accel_rpm_s);
    zdt_append_u16_be(buf, &i, max_len, decel_rpm_s);
    zdt_append_u16_be(buf, &i, max_len, max_speed);
    zdt_append_u32_be(buf, &i, max_len, pos_angle);
    zdt_append_u8(buf, &i, max_len, move_mode);
    zdt_append_u8(buf, &i, max_len, sync);
    zdt_append_u16_be(buf, &i, max_len, max_current_ma);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildTrapezoidPosModeCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                              uint16_t accel_rpm_s,
                                              uint16_t decel_rpm_s,
                                              uint16_t max_speed,
                                              uint32_t pos_angle,
                                              uint8_t move_mode,
                                              uint8_t sync,
                                              uint16_t max_current_ma,
                                              zdt_can_msg_t *msg)
{
    uint8_t raw[18];
    int n = _raw_TrapezoidPosModeCurrentLimitCmd(addr, dir, accel_rpm_s, decel_rpm_s, max_speed, pos_angle, move_mode, sync, max_current_ma, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* ====================================================================
 *  5.6 修改驱动参数命令 (X 固件特有)
 * ==================================================================== */

/* 5.6.10 修改命令位置角度是否继续缩小 10 倍输入 (X)
 * Addr + 4F + 71 + store + scale(00/01) + 6B — 6B */
static int _raw_ChangePosScaleCmd(uint8_t addr, uint8_t store,
                                  uint8_t scale_10x,
                                  uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x4F);   /* FuncCode */
    zdt_append_u8(buf, &i, max_len, 0x71);   /* AuxCode */
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, scale_10x);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangePosScaleCmd(uint8_t addr, uint8_t store,
                                 uint8_t scale_10x,
                                 zdt_can_msg_t *msg)
{
    uint8_t raw[6];
    int n = _raw_ChangePosScaleCmd(addr, store, scale_10x, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.6.14 读取 PID 参数 (X)
 * Addr + 21 + 6B — 3B */
static int _raw_ReadPidXCmd(uint8_t addr, uint8_t *buf, size_t max_len)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x21);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadPidXCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadPidXCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.6.15 修改 PID 参数 (X)
 * Addr + 4A + C3 + store + pTkp(BE32) + pBkp(BE32) + vkp(BE32) + vki(BE32) + 6B — 21B */
static int _raw_WritePidXCmd(uint8_t addr, uint8_t store,
                            uint32_t pTkp, uint32_t pBkp,
                            uint32_t vkp, uint32_t vki,
                            uint8_t *buf, size_t max_len)
{
    const size_t need = 21;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x4A);   /* FuncCode */
    zdt_append_u8(buf, &i, max_len, 0xC3);   /* AuxCode */
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u32_be(buf, &i, max_len, pTkp);
    zdt_append_u32_be(buf, &i, max_len, pBkp);
    zdt_append_u32_be(buf, &i, max_len, vkp);
    zdt_append_u32_be(buf, &i, max_len, vki);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWritePidXCmd(uint8_t addr, uint8_t store,
                            uint32_t pTkp, uint32_t pBkp,
                            uint32_t vkp, uint32_t vki,
                            zdt_can_msg_t *msg)
{
    uint8_t raw[21];
    int n = _raw_WritePidXCmd(addr, store, pTkp, pBkp, vkp, vki, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* ====================================================================
 *  5.7 上电自动运行命令 (X 固件特有)
 * ==================================================================== */

/* 5.7.1 存储一组速度参数，上电自动运行 (X)
 * Addr + F7 + 1C + store + dir + acc(BE16) + speed(BE16) + en + 6B — 10B */
static int _raw_StoreAutoRunXCmd(uint8_t addr, uint8_t store,
                                uint8_t dir, uint16_t acc_rpm_per_s,
                                uint16_t speed_rpm_x10, uint8_t en_pin_enable,
                                uint8_t *buf, size_t max_len)
{
    const size_t need = 11;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xF7);   /* 功能码 */
    zdt_append_u8(buf, &i, max_len, 0x1C);   /* 辅助码 */
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, dir);
    zdt_append_u16_be(buf, &i, max_len, acc_rpm_per_s);   /* 加速度 0..65535 RPM/s */
    zdt_append_u16_be(buf, &i, max_len, speed_rpm_x10);   /* 速度 0..0x7530 (×0.1RPM) */
    zdt_append_u8(buf, &i, max_len, en_pin_enable);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildStoreAutoRunXCmd(uint8_t addr, uint8_t store,
                                uint8_t dir, uint16_t acc_rpm_per_s,
                                uint16_t speed_rpm_x10, uint8_t en_pin_enable,
                                zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int n = _raw_StoreAutoRunXCmd(addr, store, dir, acc_rpm_per_s, speed_rpm_x10, en_pin_enable, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* ====================================================================
 *  5.8 读取与修改所有驱动参数命令 (X 固件特有)
 * ==================================================================== */

/* 5.8.1 读取系统状态参数 (X)
 * Addr + 43 + 7A + 6B — 4B */
static int _raw_ReadAllStatusXCmd(uint8_t addr, uint8_t *buf, size_t max_len)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x43);   /* 功能码 */
    zdt_append_u8(buf, &i, max_len, 0x7A);   /* 辅助码 */
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadAllStatusXCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_ReadAllStatusXCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.8.3 读取驱动配置参数 (X)
 * Addr + 42 + 6C + 6B — 4B */
static int _raw_ReadAllConfigXCmd(uint8_t addr, uint8_t *buf, size_t max_len)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x42);   /* 功能码 */
    zdt_append_u8(buf, &i, max_len, 0x6C);   /* 辅助码 */
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadAllConfigXCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_ReadAllConfigXCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.8.4 修改驱动配置参数 (X)
 * Addr + 48 + D1 + store + ... + 6B — 37B */
static int _raw_WriteAllConfigXCmd(uint8_t addr, uint8_t store,
                                  uint8_t key_lock, uint8_t ctrl_mode,
                                  uint8_t pul_mode, uint8_t com_mode,
                                  uint8_t en_level, uint8_t dir_level,
                                  uint8_t microstep, uint8_t interp,
                                  uint16_t open_current_ma,
                                  uint16_t close_current_ma,
                                  uint16_t close_max_rpm,
                                  uint16_t current_bw_hz,
                                  uint8_t uart_baud, uint8_t can_speed,
                                  uint8_t check_mode, uint8_t reply_mode,
                                  uint8_t angle_scale_10,
                                  uint8_t stall_protect,
                                  uint16_t stall_speed_rpm,
                                  uint16_t stall_current_ma,
                                  uint16_t stall_time_ms,
                                  uint16_t pos_window_x10,
                                  uint8_t *buf, size_t max_len)
{
    const size_t need = 37;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x48);   /* 功能码 */
    zdt_append_u8(buf, &i, max_len, 0xD1);   /* 辅助码 */
    zdt_append_u8(buf, &i, max_len, store);                          /* 是否存储 00/01 */
    zdt_append_u8(buf, &i, max_len, key_lock);                       /* 锁定按键 00/01 */
    zdt_append_u8(buf, &i, max_len, ctrl_mode);                      /* 控制模式 00=开环 01=FOC */
    zdt_append_u8(buf, &i, max_len, pul_mode);                       /* 脉冲端口复用 00..04 */
    zdt_append_u8(buf, &i, max_len, com_mode);                       /* 通讯端口复用 00..04 */
    zdt_append_u8(buf, &i, max_len, en_level);                       /* En有效电平 00=L 01=H 02=Hold */
    zdt_append_u8(buf, &i, max_len, dir_level);                      /* Dir有效电平 00=CW 01=CCW */
    zdt_append_u8(buf, &i, max_len, microstep);                      /* 细分 0..255 (0=256细分) */
    zdt_append_u8(buf, &i, max_len, interp);                         /* 细分插补 00/01 */
    zdt_append_u8(buf, &i, max_len, 0x00);                           /* 保留 */
    zdt_append_u8(buf, &i, max_len, 0x00);                           /* 保留 */
    zdt_append_u16_be(buf, &i, max_len, open_current_ma);            /* 开环工作电流 0..0x1388 mA */
    zdt_append_u16_be(buf, &i, max_len, close_current_ma);           /* 闭环最大电流 0..0x1388 mA */
    zdt_append_u16_be(buf, &i, max_len, close_max_rpm);              /* 闭环最大速度 0..0x0BB8 RPM */
    zdt_append_u16_be(buf, &i, max_len, current_bw_hz);              /* 电流环带宽 0..0xFFFF Hz */
    zdt_append_u8(buf, &i, max_len, uart_baud);                      /* 串口波特率 00..08 */
    zdt_append_u8(buf, &i, max_len, can_speed);                      /* CAN速率 00..09 */
    zdt_append_u8(buf, &i, max_len, check_mode);                     /* 通讯校验 00..04 */
    zdt_append_u8(buf, &i, max_len, reply_mode);                     /* 控制命令应答 00..04 */
    zdt_append_u8(buf, &i, max_len, angle_scale_10);                 /* 角度缩小10倍输入 00/01 */
    zdt_append_u8(buf, &i, max_len, stall_protect);                  /* 堵转保护 00=关 01=使能 02=复位不松轴 */
    zdt_append_u16_be(buf, &i, max_len, stall_speed_rpm);            /* 堵转检测转速 0..0x0BB8 RPM */
    zdt_append_u16_be(buf, &i, max_len, stall_current_ma);           /* 堵转检测电流 0..0x1388 mA */
    zdt_append_u16_be(buf, &i, max_len, stall_time_ms);              /* 堵转检测时间 0..0xFFFF ms */
    zdt_append_u16_be(buf, &i, max_len, pos_window_x10);             /* 位置到达窗口(×10, 默认8=0.8°) */
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWriteAllConfigXCmd(uint8_t addr, uint8_t store,
                                  uint8_t key_lock, uint8_t ctrl_mode,
                                  uint8_t pul_mode, uint8_t com_mode,
                                  uint8_t en_level, uint8_t dir_level,
                                  uint8_t microstep, uint8_t interp,
                                  uint16_t open_current_ma,
                                  uint16_t close_current_ma,
                                  uint16_t close_max_rpm,
                                  uint16_t current_bw_hz,
                                  uint8_t uart_baud, uint8_t can_speed,
                                  uint8_t check_mode, uint8_t reply_mode,
                                  uint8_t angle_scale_10,
                                  uint8_t stall_protect,
                                  uint16_t stall_speed_rpm,
                                  uint16_t stall_current_ma,
                                  uint16_t stall_time_ms,
                                  uint16_t pos_window_x10,
                                  zdt_can_msg_t *msg)
{
    uint8_t raw[37];
    int n = _raw_WriteAllConfigXCmd(addr, store, key_lock, ctrl_mode, pul_mode, com_mode,
                                    en_level, dir_level, microstep, interp,
                                    open_current_ma, close_current_ma, close_max_rpm,
                                    current_bw_hz, uart_baud, can_speed, check_mode,
                                    reply_mode, angle_scale_10, stall_protect,
                                    stall_speed_rpm, stall_current_ma, stall_time_ms,
                                    pos_window_x10, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}
