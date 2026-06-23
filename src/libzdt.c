/*
 * libzdt.c - ZDT_X42S 闭环步进电机命令构建库
 * 模板文件: 实现了所有命令框架 (内部 helper + 5.2 节作为模板示例)
 *          其它命令通过 buildXxxCmd 函数对外暴露，详细见 libzdt.h
 */

#include "libzdt.h"
#include <string.h>

/* ====================================================================
 *  内部 helpers — 所有命令共用
 * ==================================================================== */

static inline void zdt_append_u8(uint8_t *buf, size_t *idx, uint8_t v)
{
    buf[(*idx)++] = v;
}

static inline void zdt_append_u16_be(uint8_t *buf, size_t *idx, uint16_t v)
{
    /* 大端序 (Big-Endian): 与手册所有示例一致
     * 例如: 1500 → 05 DC, 10000 → 27 10, 32000 → 7D 00 */
    buf[(*idx)++] = (uint8_t)((v >> 8) & 0xFF);
    buf[(*idx)++] = (uint8_t)(v & 0xFF);
}

static inline void zdt_append_u32_be(uint8_t *buf, size_t *idx, uint32_t v)
{
    buf[(*idx)++] = (uint8_t)((v >> 24) & 0xFF);
    buf[(*idx)++] = (uint8_t)((v >> 16) & 0xFF);
    buf[(*idx)++] = (uint8_t)((v >> 8) & 0xFF);
    buf[(*idx)++] = (uint8_t)(v & 0xFF);
}

static inline int zdt_check_size(size_t need, size_t buf_size)
{
    return (buf_size >= need) ? 0 : ZDT_ERR_BUF_TOO_SMALL;
}

/* ====================================================================
 *  5.2 触发动作命令  (模板示例)
 * ==================================================================== */

/* 5.2.1 触发编码器校准
 * 命令: Addr + 06 + 45 + 6B  — 4 字节
 * 例: 发送 01 06 45 6B → 返回 01 06 02 6B */
int zdtBuildEncoderCalibrationCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x06);   /* 功能码 */
    zdt_append_u8(buf, &i, 0x45);   /* 辅助码 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.2.2 重启电机 (X42S/Y42)  Addr + 08 + 97 + 6B */
int zdtBuildRestartMotorCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x08);
    zdt_append_u8(buf, &i, 0x97);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.2.3 将当前位置角度清零  Addr + 0A + 6D + 6B */
int zdtBuildClearCurrentAngleCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x0A);
    zdt_append_u8(buf, &i, 0x6D);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.2.4 解除堵转/过热/过流保护  Addr + 0E + 52 + 6B */
int zdtBuildClearProtectionCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x0E);
    zdt_append_u8(buf, &i, 0x52);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.2.5 恢复出厂设置  Addr + 0F + 5F + 6B */
int zdtBuildFactoryResetCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x0F);
    zdt_append_u8(buf, &i, 0x5F);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* ====================================================================
 *  5.3 运动控制命令
 * ==================================================================== */

/* 5.3.1 多电机命令
 * 命令: 00 + AA + 总字节数(BE16) + 子命令序列 + 6B
 * 例: 00 AA 00 22 02 FD ... 04 36 6B 6B
 *     (0022 = 总字节数 34 = 5 + 子命令长度 29, 见手册 5.3.1) */
int zdtBuildMultiMotorCmd(const uint8_t *sub_cmds, size_t sub_cmds_len,
                          uint8_t *buf, size_t buf_size)
{
    const size_t need = 5 + sub_cmds_len;
    size_t i = 0;
    if (!sub_cmds || sub_cmds_len == 0) return ZDT_ERR_PARAM;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;

    zdt_append_u8(buf, &i, ZDT_ADDR_BROADCAST);    /* 00 = 广播地址 */
    zdt_append_u8(buf, &i, 0xAA);
    /* 长度字段为整帧总字节数 (5 + 子命令长度)，见手册示例 0022=34 */
    zdt_append_u16_be(buf, &i, (uint16_t)need);
    memcpy(&buf[i], sub_cmds, sub_cmds_len);
    i += sub_cmds_len;
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.2 电机使能控制  Addr + F3 + AB + 使能 + 同步 + 6B  — 6B
 * enable: 00=释放, 01=使能 */
int zdtBuildMotorEnableCmd(uint8_t addr, uint8_t enable, uint8_t sync,
                           uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xF3);
    zdt_append_u8(buf, &i, 0xAB);
    zdt_append_u8(buf, &i, enable);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.3 力矩模式控制 (X)  Addr + F5 + 符号 + 斜率(BE16) + 电流(BE16) + 同步 + 6B  — 9B */
int zdtBuildTorqueModeCmd(uint8_t addr, uint8_t dir,
                          uint16_t slope_ma_s, uint16_t current_ma,
                          uint8_t sync, uint8_t *buf, size_t buf_size)
{
    const size_t need = 9;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xF5);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, slope_ma_s);
    zdt_append_u16_be(buf, &i, current_ma);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.4 力矩模式限速控制 (X)  Addr + C5 + 符号 + 斜率(BE16) + 电流(BE16) + 同步 + 最大速度(BE16) + 6B  — 11B
 * 例: 01 C5 01 00 C8 02 58 00 0F A0 6B
 *     (CCW, 斜率200mA/S, 电流600mA, 同步00, 最大速度400.0RPM) */
int zdtBuildTorqueModeSpeedLimitCmd(uint8_t addr, uint8_t dir,
                                    uint16_t slope_ma_s, uint16_t current_ma,
                                    uint16_t max_rpm_x10,
                                    uint8_t sync, uint8_t *buf, size_t buf_size)
{
    const size_t need = 11;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xC5);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, slope_ma_s);
    zdt_append_u16_be(buf, &i, current_ma);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u16_be(buf, &i, max_rpm_x10);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.5 速度模式控制 (X)  Addr + F6 + 方向 + 加速度(BE16) + 速度(BE16) + 同步 + 6B  — 9B
 * (注: 与 5.3.7 Emm 不同, Emm 是 速度(BE16)+加速度(1B)) */
int zdtBuildSpeedModeXCmd(uint8_t addr, uint8_t dir,
                          uint16_t acc_rpm_s, uint16_t rpm_x10,
                          uint8_t sync, uint8_t *buf, size_t buf_size)
{
    const size_t need = 9;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xF6);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, acc_rpm_s);
    zdt_append_u16_be(buf, &i, rpm_x10);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.6 速度模式限电流控制 (X)  Addr + C6 + 符号 + 加速度(BE16) + 速度(BE16) + 同步 + 最大电流(BE16) + 6B  — 11B
 * 例: 01 C6 01 03 E8 4E 20 00 07 D0 6B
 *     (CCW, 加速度1000RPM/S, 速度2000.0RPM, 同步00, 最大电流2000mA) */
int zdtBuildSpeedModeXCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                      uint16_t acc_rpm_s, uint16_t rpm_x10,
                                      uint16_t max_current_ma,
                                      uint8_t sync, uint8_t *buf, size_t buf_size)
{
    const size_t need = 11;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xC6);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, acc_rpm_s);
    zdt_append_u16_be(buf, &i, rpm_x10);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u16_be(buf, &i, max_current_ma);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.7 速度模式控制 (Emm) — 与 5.3.5 X 固件结构不同
 * Addr + F6 + 方向 + 速度(BE16, RPM) + 加速度(1B, 档位) + 同步 + 6B  — 8B
 * 注意: 速度单位是 RPM(整数), 加速度是 0..255 档位
 * 例: 01 F6 01 05 DC 0A 00 6B  (CCW, 1500RPM, 10档, 同步00) */
int zdtBuildSpeedModeEmmCmd(uint8_t addr, uint8_t dir, uint16_t rpm,
                             uint8_t acc, uint8_t sync,
                             uint8_t *buf, size_t buf_size)
{
    const size_t need = 8;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xF6);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, rpm);
    zdt_append_u8(buf, &i, acc);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.8 直通限速位置模式控制 (X)  Addr + FB + 方向 + 速度(BE16) + 位置角度(BE32) + 运动模式 + 同步 + 6B  — 12B */
int zdtBuildPosModePassThroughCmd(uint8_t addr, uint8_t dir,
                                  uint16_t rpm_x10, uint32_t pos_angle,
                                  uint8_t move_mode, uint8_t sync,
                                  uint8_t *buf, size_t buf_size)
{
    const size_t need = 12;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xFB);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, rpm_x10);
    zdt_append_u32_be(buf, &i, pos_angle);
    zdt_append_u8(buf, &i, move_mode);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.9 直通限速位置模式限电流控制 (X)  Addr + CB + 方向 + 速度(BE16) + 位置角度(BE32) + 运动模式 + 同步 + 最大电流(BE16) + 6B  — 14B
 * 例: 01 CB 01 4E 20 00 00 8C A0 00 00 07 D0 6B
 *     (CCW, 速度2000.0RPM, 相对位置+3600.0°, 同步00, 最大电流2000mA) */
int zdtBuildPosModePassThroughCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                              uint16_t rpm_x10, uint32_t pos_angle,
                                              uint8_t move_mode, uint8_t sync,
                                              uint16_t max_current_ma,
                                              uint8_t *buf, size_t buf_size)
{
    const size_t need = 14;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xCB);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, rpm_x10);
    zdt_append_u32_be(buf, &i, pos_angle);
    zdt_append_u8(buf, &i, move_mode);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u16_be(buf, &i, max_current_ma);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.10 梯形曲线加减速位置模式控制 (X)  Addr + FD + 方向 + 加速加速度(BE16) + 减速加速度(BE16) + 最大速度(BE16) + 位置角度(BE32) + 运动模式 + 同步 + 6B  — 16B */
int zdtBuildTrapezoidPosModeCmd(uint8_t addr, uint8_t dir,
                                uint16_t accel_rpm_s, uint16_t decel_rpm_s,
                                uint16_t max_speed_x10, uint32_t pos_angle,
                                uint8_t move_mode, uint8_t sync,
                                uint8_t *buf, size_t buf_size)
{
    const size_t need = 16;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xFD);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, accel_rpm_s);
    zdt_append_u16_be(buf, &i, decel_rpm_s);
    zdt_append_u16_be(buf, &i, max_speed_x10);
    zdt_append_u32_be(buf, &i, pos_angle);
    zdt_append_u8(buf, &i, move_mode);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.11 梯形曲线加减速位置模式限电流控制 (X) — 功能码 CD
 * 命令: Addr + CD + 方向 + 加速加速度(LE16) + 减速加速度(LE16)
 *       + 最大速度(LE16) + 位置角度(LE32) + 运动模式 + 同步 + 最大电流(LE16) + 6B  — 18B */
int zdtBuildTrapezoidPosModeCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                           uint16_t accel_rpm_s,
                                           uint16_t decel_rpm_s,
                                           uint16_t max_speed,
                                           uint32_t pos_angle,
                                           uint8_t move_mode,
                                           uint8_t sync,
                                           uint16_t max_current_ma,
                                           uint8_t *buf, size_t buf_size)
{
    const size_t need = 18;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xCD);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, accel_rpm_s);
    zdt_append_u16_be(buf, &i, decel_rpm_s);
    zdt_append_u16_be(buf, &i, max_speed);
    zdt_append_u32_be(buf, &i, pos_angle);
    zdt_append_u8(buf, &i, move_mode);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u16_be(buf, &i, max_current_ma);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.12 位置模式控制 (Emm) — 功能码 FD
 * 命令: Addr + FD + 方向 + 速度(BE16) + 加速度 + 编码器值(BE32) + 运动模式 + 同步 + 6B  — 13B
 *
 * 【单位说明 — EMM 固件 vs X 固件】
 *   X 固件下本字段叫"脉冲数",单位是细分后的步进脉冲(默认细分 16,1 圈 = 51200 脉冲)。
 *   EMM 固件下本字段是"编码器值",单位 0..65535 = 0..360°,1 圈 = 65536 编码器值。
 *   命名"pulses"是历史遗留,X 固件用,EMM 固件下应该理解成 encoder_count。
 *
 * 【倍率:编码器单位位移 = pulses / microstep】
 *   EMM 固件内部把 pulses 字段除以当前细分(microstep,5.6.2)后换算成编码器单位位移。
 *   - microstep=1:1 脉冲 = 1 编码器单位 (1:1)
 *   - microstep=16:1 脉冲 = 1/16 编码器单位 (16 脉冲 = 1 编码器单位)
 *   - microstep=0 表示 256 细分:1 脉冲 = 1/256 编码器单位
 *   推论:microstep=1 下 1 圈 = 65536 脉冲;microstep=16 下 1 圈 = 1048576 脉冲。
 *
 * 实测验证(microstep=1):pulses=100 → 实际位移 ≈ 128 编码器单位;
 *                       pulses=65536 → 实际位移 ≈ 65536 编码器单位 = 1 圈。
 *
 * 方向:     ZDT_DIR_CW(0x00) 顺时针 / ZDT_DIR_CCW(0x01) 逆时针
 * 速度:     RPM(BE16, 闭环建议 0..500)
 * 加速度:   0..255(0=最高加速,255=最柔和;常用 0~10)
 * pulses:   编码器值(LE32,正整数)
 * 运动模式: ZDT_MOVE_REL_LAST(0x00) 相对上次目标 / ZDT_MOVE_ABS_ZERO(0x01) 相对绝对零位
 *           / ZDT_MOVE_REL_NOW(0x02) 相对当前位置
 * 同步:     ZDT_SYNC_NOW(0x00) 立即执行 / ZDT_SYNC_CACHE(0x01) 缓存待 SyncMotion 触发
 *
 * 例: 发送 01 FD 01 05 DC 0A 00 00 C8 00 02 00 6B → CCW 1500RPM 加速度 10
 *     编码器值 200, 相对当前位置, 立即执行
 *     (注:旧例"32000脉冲"是 X 固件举例,EMM 固件下此值是编码器单位,1 圈 = 65536) */
int zdtBuildPosModeEmmCmd(uint8_t addr, uint8_t dir, uint16_t rpm,
                          uint8_t acc, uint32_t pulses,
                          uint8_t move_mode, uint8_t sync,
                          uint8_t *buf, size_t buf_size)
{
    const size_t need = 13;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xFD);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, rpm);
    zdt_append_u8(buf, &i, acc);
    zdt_append_u32_be(buf, &i, pulses);
    zdt_append_u8(buf, &i, move_mode);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.13 立即停止  Addr + FE + 98 + 同步 + 6B  — 5B
 * 例: 发送 01 FE 98 00 6B → 立即停止电机 */
int zdtBuildImmediateStopCmd(uint8_t addr, uint8_t sync,
                             uint8_t *buf, size_t buf_size)
{
    const size_t need = 5;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xFE);
    zdt_append_u8(buf, &i, 0x98);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.3.14 触发多机同步运动  Addr + FF + 66 + 6B  — 4B
 * addr 通常传 0x00 (广播) */
int zdtBuildSyncMotionCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xFF);
    zdt_append_u8(buf, &i, 0x66);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* ====================================================================
 *  5.4 原点回零命令
 * ==================================================================== */

/* 5.4.1 设置单圈回零的零点位置  Addr + 93 + 88 + 是否存储 + 6B  — 5B
 * 例: 01 93 88 01 6B (存储) */
int zdtBuildSetSingleTurnZeroCmd(uint8_t addr, uint8_t store,
                                 uint8_t *buf, size_t buf_size)
{
    const size_t need = 5;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x93);
    zdt_append_u8(buf, &i, 0x88);
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.4.2 触发回零  Addr + 9A + 模式 + 同步 + 6B  — 5B
 * homing_mode: 00..05 */
int zdtBuildTriggerHomingCmd(uint8_t addr, uint8_t homing_mode,
                             uint8_t sync, uint8_t *buf, size_t buf_size)
{
    const size_t need = 5;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x9A);
    zdt_append_u8(buf, &i, homing_mode);
    zdt_append_u8(buf, &i, sync);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.4.3 强制中断并退出回零  Addr + 9C + 48 + 6B  — 4B */
int zdtBuildAbortHomingCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x9C);
    zdt_append_u8(buf, &i, 0x48);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.4.4 读取回零状态标志 — 功能码 3B (与 5.4.5 的 22 不同, 见手册) */
int zdtBuildReadHomingStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x3B);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.4.5 读取回零参数  Addr + 22 + 6B  — 3B (返回 18B) */
int zdtBuildReadHomingParamsCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x22);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.4.6 修改回零参数  Addr + 4C + AE + 是否存储 + 模式 + 方向 + 速度(LE16)
 *       + 超时(LE32) + 碰撞转速(LE16) + 碰撞电流(LE16) + 碰撞时间(LE16)
 *       + O_POT_En + 6B  — 20B
 * 例: 01 4C AE 01 00 00 00 1E 00 00 27 10 01 2C 03 20 00 3C 00 6B */
int zdtBuildWriteHomingParamsCmd(uint8_t addr, uint8_t store,
                                 uint8_t homing_mode, uint8_t dir,
                                 uint16_t rpm, uint32_t timeout_ms,
                                 uint16_t bump_speed_rpm,
                                 uint16_t bump_current_ma,
                                 uint16_t bump_time_ms,
                                 uint8_t auto_trigger,
                                 uint8_t *buf, size_t buf_size)
{
    const size_t need = 20;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x4C);
    zdt_append_u8(buf, &i, 0xAE);
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, homing_mode);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, rpm);
    zdt_append_u32_be(buf, &i, timeout_ms);
    zdt_append_u16_be(buf, &i, bump_speed_rpm);
    zdt_append_u16_be(buf, &i, bump_current_ma);
    zdt_append_u16_be(buf, &i, bump_time_ms);
    zdt_append_u8(buf, &i, auto_trigger);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* ====================================================================
 *  5.5 读取系统参数命令
 *  全部为 Addr + FuncCode + 6B 的 4B 短帧
 * ==================================================================== */

/* 5.5.1 定时返回信息命令 (X42S/Y42)
 * Addr + 11 + 18 + 信息功能码 + 定时时间(BE16, ms) + 6B — 7B
 * info_func_code: 36/3A/... (5.5 节中任意读取命令的功能码)
 * interval_ms:    0..0xFFFF ms */
int zdtBuildSetPeriodicReportCmd(uint8_t addr, uint8_t info_func_code,
                                 uint16_t interval_ms,
                                 uint8_t *buf, size_t buf_size)
{
    const size_t need = 7;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x11);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x18);   /* AuxCode */
    zdt_append_u8(buf, &i, info_func_code);
    zdt_append_u16_be(buf, &i, interval_ms);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.2 读取固件版本和硬件版本  Addr + 1F + 6B  — 4B */
int zdtBuildReadVersionCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x1F);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.3 读取相电阻和相电感  Addr + 20 + 6B  — 4B */
int zdtBuildReadPhaseRLCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x20);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.4 读取总线电压  Addr + 24 + 6B  — 4B */
int zdtBuildReadBusVoltageCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x24);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.5 读取总线电流 (X42S/Y42)  Addr + 26 + 6B  — 4B */
int zdtBuildReadBusCurrentCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x26);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.6 读取相电流  Addr + 27 + 6B  — 4B */
int zdtBuildReadPhaseCurrentCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x27);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.7 读取经过线性化校准后的编码器值 — 功能码 0x31
 * 命令: Addr + 31 + 6B  — 4B
 *
 * 返回格式(实测 6B):
 *   [sign] [encoder_value(BE16)] [6B]   —— 共 4B 有效负载
 *   - sign:       0x00 正转 / 0x01 反转
 *   - encoder_value: 16-bit 编码器位置(0..65535 = 0..360°,经过线性化校准)
 *                   即"单圈内绝对编码器值",电机转 N 圈后值仍落在 0..65535 范围内
 *
 * 用途:
 *   - 编码器零点标定后的位置读取,精度比 5.5.13(0x36 实时位置)更高
 *   - 单圈绝对位置读取,适合周期性回读用于位置闭环控制
 *
 * 区别于 5.5.13(0x36 实时位置):5.5.7 是"线性化校准后"的单圈绝对值;
 *                          5.5.13 是"实时反馈位置",可能是累计或 mod 65535 的位置。
 *
 * 注意:旧注释"Addr + 29 + 6B"是错的(0x29 是别的命令),功能码应为 0x31。 */
int zdtBuildReadEncoderCalibratedCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x31);   /* 功能码 0x31 = 5.5.7 线性化编码器值 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.8 读取输入脉冲数  Addr + 32 + 6B  — 4B */
int zdtBuildReadInputPulsesCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x32);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.9 读取电机目标位置  Addr + 33 + 6B  — 4B */
int zdtBuildReadTargetPosCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x33);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.10 读取电机实时设定的目标位置  Addr + 35 + 6B  — 4B */
int zdtBuildReadRealtimeTargetPosCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x34);   /* 功能码 5.5.10 实时设定目标位置 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.11 读取电机实时转速  Addr + 36 + 6B  — 4B */
int zdtBuildReadRealtimeSpeedCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x35);   /* 功能码 5.5.11 实时转速 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.12 读取驱动温度 (X42S/Y42)  Addr + 39 + 6B  — 4B */
int zdtBuildReadDriverTempCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x39);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.13 读取电机实时位置 — 功能码 0x36
 * 命令: Addr + 36 + 6B  — 4B
 *
 * 返回格式(实测):
 *   [sign] [realtime_pos(BE32)] [6B]   —— 共 6B 有效负载
 *   - sign:        0x00 正向位置 / 0x01 反向位置
 *   - realtime_pos: 32-bit 实时位置(单位由固件决定)
 *                   - EMM 固件下可能是"累计编码器原始值"或"单圈 mod 65535 的位置",
 *                     取决于具体固件版本
 *                   - 实测(microstep=1):发 pulses=51200 后读到的 pos_final - pos0 ≈ 1.6e7,
 *                     说明该字段是"累计编码器原始值"而非单圈 mod 值
 *                   - 周期回读用 5.5.7(0x31 单圈绝对值)更直观
 *
 * 用途:
 *   - 位置闭环控制时的反馈
 *   - 估算实际电机位移(注意:发 pulses=51200 期望 0.78 圈,实测位移可能差 12.5 倍,
 *     因为 EMM 固件内部 pulses÷microstep 后才是编码器位移;详细见 5.3.12 注释)
 *
 * 区别于 5.5.7(0x31 编码器值):5.5.7 是单圈绝对值(0..65535),5.5.13 是 32-bit 累计/反馈位置。
 *
 * 注意:旧注释"Addr + 40 + 6B"是错的(0x28 = 40 是别的命令),功能码应为 0x36。 */
int zdtBuildReadRealtimePosCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x36);   /* 功能码 0x36 = 5.5.13 实时位置 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.14 读取电机位置误差  Addr + 41 + 6B  — 4B */
int zdtBuildReadPosErrorCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x37);   /* 功能码 5.5.14 位置误差 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.15 读取电机状态标志  Addr + 42 + 6B  — 4B */
int zdtBuildReadMotorStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x3A);   /* 功能码 5.5.15 电机状态标志 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.16 读取回零状态标志 + 电机状态标志 (X42S/Y42)  Addr + 43 + 6B  — 4B */
int zdtBuildReadHomingAndStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x3C);   /* 功能码 5.5.16 回零+电机状态 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.17 读取引脚 IO 电平状态 (X42S/Y42)  Addr + 46 + 6B  — 4B */
int zdtBuildReadIoLevelCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x3D);   /* 功能码 5.5.17 IO 电平 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.5.18 读取电池电压 (Y42)  Addr + 47 + 6B  — 4B */
int zdtBuildReadBatteryVoltageCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x38);   /* 功能码 5.5.18 电池电压 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* ====================================================================
 *  5.6 读写驱动参数命令 (31 个)
 * ==================================================================== */

/* 5.6.1 修改电机 ID/地址  Addr + AE + 4B + store + new_addr + 6B  — 6B */
int zdtBuildChangeAddrCmd(uint8_t addr, uint8_t store,
                          uint8_t new_addr,
                          uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xAE);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x4B);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, new_addr);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.2 修改细分值  Addr + 84 + 8A + store + microstep(1B) + 6B  — 6B */
int zdtBuildChangeMicrostepCmd(uint8_t addr, uint8_t store,
                               uint8_t microstep,
                               uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x84);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x8A);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, microstep);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.3 修改掉电标志  Addr + 50 + flag + 6B  — 4B */
int zdtBuildChangePowerDownFlagCmd(uint8_t addr, uint8_t flag,
                                   uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x50);   /* FuncCode */
    zdt_append_u8(buf, &i, flag);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.4 读取选项参数状态 (X42S/Y42)  Addr + 1A + 6B  — 3B */
int zdtBuildReadOptionsCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x1A);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.5 修改电机类型  Addr + D7 + 35 + store + type(0x19/0x32) + 6B  — 6B */
int zdtBuildChangeMotorTypeCmd(uint8_t addr, uint8_t store,
                               uint8_t motor_type,
                               uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xD7);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x35);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, motor_type);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.6 修改固件类型  Addr + D5 + 69 + store + type(00/01/02) + 6B  — 6B */
int zdtBuildChangeFirmwareTypeCmd(uint8_t addr, uint8_t store,
                                  uint8_t fw_type,
                                  uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xD5);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x69);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, fw_type);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.7 修改开环/闭环控制模式  Addr + 46 + A6 + store + mode(00/01) + 6B  — 6B */
int zdtBuildChangeCtrlModeCmd(uint8_t addr, uint8_t store,
                              uint8_t mode,
                              uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x46);   /* FuncCode */
    zdt_append_u8(buf, &i, 0xA6);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, mode);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.8 修改电机运动正方向  Addr + D4 + 60 + store + dir(00/01) + 6B  — 6B */
int zdtBuildChangeMotorDirCmd(uint8_t addr, uint8_t store,
                              uint8_t dir,
                              uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xD4);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x60);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.9 修改锁定按键功能  Addr + D0 + B3 + store + lock(00/01) + 6B  — 6B */
int zdtBuildChangeKeyLockCmd(uint8_t addr, uint8_t store,
                             uint8_t lock,
                             uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xD0);   /* FuncCode */
    zdt_append_u8(buf, &i, 0xB3);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, lock);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.10 修改命令位置角度是否继续缩小 10 倍输入 (X)
 * Addr + 4F + 71 + store + scale(00/01) + 6B  — 6B */
int zdtBuildChangePosScaleCmd(uint8_t addr, uint8_t store,
                              uint8_t scale_10x,
                              uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x4F);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x71);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, scale_10x);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.11 修改命令速度值是否缩小 10 倍输入 (Emm) — 与 5.6.10 帧结构相同 */
int zdtBuildChangeSpeedScaleCmd(uint8_t addr, uint8_t store,
                                uint8_t scale_10x,
                                uint8_t *buf, size_t buf_size)
{
    /* 帧结构与 5.6.10 相同: Addr + 4F + 71 + store + scale + 6B */
    return zdtBuildChangePosScaleCmd(addr, store, scale_10x, buf, buf_size);
}

/* 5.6.12 修改开环模式工作电流  Addr + 44 + 33 + store + current(BE16) + 6B  — 7B */
int zdtBuildChangeOpenLoopCurrentCmd(uint8_t addr, uint8_t store,
                                     uint16_t current_ma,
                                     uint8_t *buf, size_t buf_size)
{
    const size_t need = 7;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x44);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x33);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u16_be(buf, &i, current_ma);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.13 修改闭环模式最大电流  Addr + 45 + 66 + store + current(BE16) + 6B  — 7B */
int zdtBuildChangeClosedLoopCurrentCmd(uint8_t addr, uint8_t store,
                                       uint16_t current_ma,
                                       uint8_t *buf, size_t buf_size)
{
    const size_t need = 7;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x45);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x66);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u16_be(buf, &i, current_ma);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.14 读取 PID 参数 (X)  Addr + 21 + 6B  — 3B */
int zdtBuildReadPidXCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x21);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.15 修改 PID 参数 (X)
 * Addr + 4A + C3 + store + pTkp(BE32) + pBkp(BE32) + vkp(BE32) + vki(BE32) + 6B  — 21B
 * 例: 01 4A C3 01 00 00 01 EE B0 00 01 EE B0 00 00 3C F0 00 00 00 1A 6B (21B) */
int zdtBuildWritePidXCmd(uint8_t addr, uint8_t store,
                         uint32_t pTkp, uint32_t pBkp,
                         uint32_t vkp, uint32_t vki,
                         uint8_t *buf, size_t buf_size)
{
    const size_t need = 21;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x4A);   /* FuncCode */
    zdt_append_u8(buf, &i, 0xC3);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u32_be(buf, &i, pTkp);
    zdt_append_u32_be(buf, &i, pBkp);
    zdt_append_u32_be(buf, &i, vkp);
    zdt_append_u32_be(buf, &i, vki);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.16 读取 PID 参数 (Emm)  Addr + 21 + 6B  — 3B */
int zdtBuildReadPidEmmCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x21);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.17 修改 PID 参数 (Emm)
 * Addr + 4A + C3 + store + Kp(BE32) + Ki(BE32) + Kd(BE32) + 6B  — 17B
 * 例: 01 4A C3 01 00 00 46 50 00 00 00 0A 00 00 46 50 6B (17B) */
int zdtBuildWritePidEmmCmd(uint8_t addr, uint8_t store,
                           uint32_t kp, uint32_t ki, uint32_t kd,
                           uint8_t *buf, size_t buf_size)
{
    const size_t need = 17;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x4A);   /* FuncCode */
    zdt_append_u8(buf, &i, 0xC3);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u32_be(buf, &i, kp);
    zdt_append_u32_be(buf, &i, ki);
    zdt_append_u32_be(buf, &i, kd);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.18 读取 DMX512 协议参数 (X42S/Y42)  Addr + 49 + 78 + 6B  — 4B */
int zdtBuildReadDmx512Cmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x49);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x78);   /* AuxCode */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.19 修改 DMX512 协议参数 (X42S/Y42)
 * Addr + D9 + 90 + store + total_ch(BE16) + ch_per_motor(1B) + move_mode(1B)
 *      + single_speed(BE16) + accel(BE16) + speed_step(BE16) + motion_step(BE32) + 6B  — 19B
 * 例: 01 D9 90 01 00 C0 01 01 03 E8 03 E8 00 0A 00 00 00 64 6B (19B) */
int zdtBuildWriteDmx512Cmd(uint8_t addr, uint8_t store,
                           uint16_t total_channels,
                           uint8_t channels_per_motor,
                           uint8_t move_mode,
                           uint16_t single_speed_rpm,
                           uint16_t accel,
                           uint16_t speed_step,
                           uint32_t motion_step,
                           uint8_t *buf, size_t buf_size)
{
    const size_t need = 19;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xD9);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x90);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u16_be(buf, &i, total_channels);     /* 0001-0040 */
    zdt_append_u8(buf, &i, channels_per_motor);     /* 01 或 02 */
    zdt_append_u8(buf, &i, move_mode);              /* 00=相对, 01=绝对 */
    zdt_append_u16_be(buf, &i, single_speed_rpm);   /* 0001-0BB8 */
    zdt_append_u16_be(buf, &i, accel);              /* 0001-FFFF */
    zdt_append_u16_be(buf, &i, speed_step);         /* 0001-FFFF */
    zdt_append_u32_be(buf, &i, motion_step);        /* 00000001-FFFFFFFF */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.20 读取位置到达窗口 (X42S/Y42)  Addr + 41 + 6B  — 3B */
int zdtBuildReadPosWindowCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x41);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.21 修改位置到达窗口 (X42S/Y42)  Addr + D1 + 07 + store + window(BE16, ×0.1°) + 6B  — 6B
 * 例: 01 D1 07 01 00 08 6B (修改为 0.8°) */
int zdtBuildWritePosWindowCmd(uint8_t addr, uint8_t store,
                              uint16_t window_x10,
                              uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xD1);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x07);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u16_be(buf, &i, window_x10);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.22 读取过热过流保护检测阈值 (X42S/Y42)  Addr + 13 + 6B  — 3B */
int zdtBuildReadProtectThresholdCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x13);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.23 修改过热过流保护检测阈值 (X42S/Y42)
 * Addr + D3 + 56 + store + overheat(BE16, ℃) + overcurrent(BE16, mA) + detect_ms(BE16) + 6B  — 11B
 * 例: 01 D3 56 01 00 64 19 C8 03 E8 6B (修改为 100℃/6600mA/1000ms) */
int zdtBuildWriteProtectThresholdCmd(uint8_t addr, uint8_t store,
                                     uint16_t overheat_c,
                                     uint16_t overcurrent_ma,
                                     uint16_t detect_ms,
                                     uint8_t *buf, size_t buf_size)
{
    const size_t need = 11;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xD3);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x56);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u16_be(buf, &i, overheat_c);
    zdt_append_u16_be(buf, &i, overcurrent_ma);
    zdt_append_u16_be(buf, &i, detect_ms);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.24 读取心跳保护功能时间 (X42S/Y42)  Addr + 16 + 6B  — 3B */
int zdtBuildReadHeartbeatCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x16);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.25 修改心跳保护功能时间 (X42S/Y42)
 * Addr + 68 + 38 + store + heartbeat_ms(BE32) + 6B  — 9B */
int zdtBuildWriteHeartbeatCmd(uint8_t addr, uint8_t store,
                              uint32_t heartbeat_ms,
                              uint8_t *buf, size_t buf_size)
{
    const size_t need = 9;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x68);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x38);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u32_be(buf, &i, heartbeat_ms);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.26 读取积分限幅/刚性系数 (X42S/Y42)  Addr + 23 + 6B  — 3B */
int zdtBuildReadIntegralLimitCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x23);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.27 修改积分限幅/刚性系数 (X42S/Y42)
 * Addr + 4B + 57 + store + value(BE32) + 6B  — 9B */
int zdtBuildWriteIntegralLimitCmd(uint8_t addr, uint8_t store,
                                  uint32_t value,
                                  uint8_t *buf, size_t buf_size)
{
    const size_t need = 9;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x4B);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x57);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u32_be(buf, &i, value);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.28 读取碰撞回零返回角度 (X42S/Y42)  Addr + 3F + 6B  — 3B */
int zdtBuildReadBumpReturnAngleCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x3F);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.29 修改碰撞回零返回角度 (X42S/Y42)  Addr + 5C + AC + store + angle(BE16, ×0.1°) + 6B  — 6B */
int zdtBuildWriteBumpReturnAngleCmd(uint8_t addr, uint8_t store,
                                    uint16_t angle_x10,
                                    uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x5C);   /* FuncCode */
    zdt_append_u8(buf, &i, 0xAC);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u16_be(buf, &i, angle_x10);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.30 广播读取 ID 地址 (X42S/Y42)  00 + 15 + 6B  — 3B */
int zdtBuildBroadcastReadAddrCmd(uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, ZDT_ADDR_BROADCAST);
    zdt_append_u8(buf, &i, 0x15);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.6.31 修改锁定修改参数功能 (X42S/Y42)  Addr + D6 + 4B + store + lock_level(0-3) + 6B  — 6B */
int zdtBuildChangeParamLockCmd(uint8_t addr, uint8_t store,
                               uint8_t lock_level,
                               uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xD6);   /* FuncCode */
    zdt_append_u8(buf, &i, 0x4B);   /* AuxCode */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, lock_level);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* ====================================================================
 *  5.7 上电自动运行命令
 * ==================================================================== */

/* 5.7.1 存储一组速度参数，上电自动运行 (X)
 * 命令: Addr + F7 + 1C + 清除/存储 + 方向 + 加速度(BE16) + 速度(BE16) +
 *       是否使能En引脚控制启停 + 6B — 10 字节
 *
 * 例: 01 F7 1C 01 00 01 FF 17 70 01 6B
 *     01=保存; 00=CW; 01FF=511RPM/s; 1770=600.0RPM; 01=使能En引脚
 *
 * store:        00=清除已存储  01=存储当前速度和加速度
 * dir:          00=CW  01=CCW
 * acc_rpm_per_s:0..65535 RPM/s (BE16)
 * speed_rpm_x10:0..30000 内部×0.1 即 0..3000.0RPM (BE16, 上限 0x7530)
 * en_pin_enable:00=不使能En  01=使能En引脚控制启停 */
int zdtBuildStoreAutoRunXCmd(uint8_t addr, uint8_t store,
                             uint8_t dir, uint16_t acc_rpm_per_s,
                             uint16_t speed_rpm_x10, uint8_t en_pin_enable,
                             uint8_t *buf, size_t buf_size)
{
    const size_t need = 10;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xF7);   /* 功能码 */
    zdt_append_u8(buf, &i, 0x1C);   /* 辅助码 */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, acc_rpm_per_s);   /* 加速度 0..65535 RPM/s */
    zdt_append_u16_be(buf, &i, speed_rpm_x10);   /* 速度 0..0x7530 (×0.1RPM) */
    zdt_append_u8(buf, &i, en_pin_enable);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.7.2 存储一组速度参数，上电自动运行 (Emm)
 * 命令: Addr + F7 + 1C + 清除/存储 + 方向 + 速度(BE16) + 加速度(单字节) +
 *       是否使能En引脚控制启停 + 6B — 10 字节
 *
 * 例: 01 F7 1C 01 00 02 58 64 01 6B
 *     01=保存; 00=CW; 0258=600RPM; 64=100档; 01=使能En引脚
 *
 * store:        00=清除已存储  01=存储当前速度和加速度
 * dir:          00=CW  01=CCW
 * speed_rpm:    0..3000 (BE16, 上限 0x0BB8)
 * acc_level:    0..255 档位, 公式见手册 5.3.7 速度模式控制 (Emm)
 * en_pin_enable:00=不使能En  01=使能En引脚控制启停 */
int zdtBuildStoreAutoRunEmmCmd(uint8_t addr, uint8_t store,
                               uint8_t dir, uint16_t speed_rpm,
                               uint8_t acc_level, uint8_t en_pin_enable,
                               uint8_t *buf, size_t buf_size)
{
    const size_t need = 10;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0xF7);   /* 功能码 */
    zdt_append_u8(buf, &i, 0x1C);   /* 辅助码 */
    zdt_append_u8(buf, &i, store);
    zdt_append_u8(buf, &i, dir);
    zdt_append_u16_be(buf, &i, speed_rpm);   /* 速度 0..0x0BB8 RPM */
    zdt_append_u8(buf, &i, acc_level);       /* 加速度 0..255 档位 */
    zdt_append_u8(buf, &i, en_pin_enable);
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* ====================================================================
 *  5.8 读取所有驱动参数命令
 * ==================================================================== */

/* 5.8.1 读取系统状态参数 (X)  Addr + 43 + 7A + 6B — 4 字节
 * 返回 37B 帧 (字节数=0x25, 参数个数=0x0C):
 *   总线电压(BE16 mV) | 总线电流(BE16 mA) | 电机相电流(BE16 mA) |
 *   编码器原始值(BE16) | 线性化编码器值(BE16) | 符号1 |
 *   电机目标位置(BE32, 单位0.1°) | 符号2 | 电机实时转速(BE16, 单位0.1RPM) | 符号3 |
 *   电机实时位置(BE32, 单位0.1°) | 符号4 | 电机位置误差(BE32, 单位0.01°) | 符号5 |
 *   电机实时温度 | 回零状态标志 | 电机状态标志 */
int zdtBuildReadAllStatusXCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x43);   /* 功能码 */
    zdt_append_u8(buf, &i, 0x7A);   /* 辅助码 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.8.2 读取系统状态参数 (Emm) Addr + 43 + 7A + 6B — 4 字节
 * 返回 31B 帧 (字节数=0x1F, 参数个数=0x09):
 *   总线电压(BE16 mV) | 电机相电流(BE16 mA) |
 *   线性化编码器值(BE16) | 符号1 | 电机目标位置(BE32) | 符号2 |
 *   电机实时转速(BE16, 单位RPM) | 符号3 | 电机实时位置(BE32) | 符号4 |
 *   电机位置误差(BE32) | 回零状态标志 | 电机状态标志
 * 与 X 帧区别: Emm 没有"总线电流"和"编码器原始值",
 *              电机目标位置/实时位置 单位为圈内编码器值(0..65535=0..360°),
 *              电机实时转速单位为 RPM 而非 0.1RPM */
int zdtBuildReadAllStatusEmmCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x43);   /* 功能码 */
    zdt_append_u8(buf, &i, 0x7A);   /* 辅助码 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.8.3 读取驱动配置参数 (X)  Addr + 42 + 6C + 6B — 4 字节
 * 返回 37B 帧 (字节数=0x25, 参数个数=0x18=24):
 *   锁定按键 | 控制模式 | 脉冲端口复用 | 通讯端口复用 | En有效电平 | Dir有效电平 |
 *   细分 | 细分插补 | 自动息屏 | 保留 | 开环工作电流(BE16) |
 *   闭环最大电流(BE16) | 闭环最大速度(BE16) | 电流环带宽(BE16) |
 *   串口波特率 | CAN速率 | 通讯校验 | 控制命令应答 | 角度缩小10倍输入 |
 *   堵转保护 | 堵转检测转速(BE16) | 堵转检测电流(BE16) |
 *   堵转检测时间(BE16) | 位置到达窗口(BE16) */
int zdtBuildReadAllConfigXCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x42);   /* 功能码 */
    zdt_append_u8(buf, &i, 0x6C);   /* 辅助码 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.8.4 修改驱动配置参数 (X) — 37 字节
 *
 * 帧结构 (主机→电机):
 *   Addr | 48 | D1 | 是否存储 | 锁定按键 | 控制模式 | 脉冲端口复用 |
 *   通讯端口复用 | En引脚有效电平 | Dir引脚有效电平 | 细分 | 细分插补 |
 *   保留 | 保留 | 开环工作电流(BE16) | 闭环最大电流(BE16) |
 *   闭环最大速度(BE16) | 电流环带宽(BE16) | 串口波特率 | CAN速率 |
 *   通讯校验 | 控制命令应答 | 角度缩小10倍输入 | 堵转保护 |
 *   堵转检测转速(BE16) | 堵转检测电流(BE16) | 堵转检测时间(BE16) |
 *   位置到达窗口(BE16) | 6B
 *
 * 例: 01 48 D1 01 00 01 01 02 02 00 10 01 00 00 04 B0 0B 80
 *     0B B8 03 E8 05 07 00 01 00 01 00 08 08 98 07 D0 00 08 6B
 */
int zdtBuildWriteAllConfigXCmd(uint8_t addr, uint8_t store,
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
                               uint8_t *buf, size_t buf_size)
{
    const size_t need = 37;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x48);   /* 功能码 */
    zdt_append_u8(buf, &i, 0xD1);   /* 辅助码 */
    zdt_append_u8(buf, &i, store);                          /* 是否存储 00/01 */
    zdt_append_u8(buf, &i, key_lock);                       /* 锁定按键 00/01 */
    zdt_append_u8(buf, &i, ctrl_mode);                      /* 控制模式 00=开环 01=FOC */
    zdt_append_u8(buf, &i, pul_mode);                       /* 脉冲端口复用 00..04 */
    zdt_append_u8(buf, &i, com_mode);                       /* 通讯端口复用 00..04 */
    zdt_append_u8(buf, &i, en_level);                       /* En有效电平 00=L 01=H 02=Hold */
    zdt_append_u8(buf, &i, dir_level);                      /* Dir有效电平 00=CW 01=CCW */
    zdt_append_u8(buf, &i, microstep);                      /* 细分 0..255 (0=256细分) */
    zdt_append_u8(buf, &i, interp);                         /* 细分插补 00/01 */
    zdt_append_u8(buf, &i, 0x00);                           /* 保留 */
    zdt_append_u8(buf, &i, 0x00);                           /* 保留 */
    zdt_append_u16_be(buf, &i, open_current_ma);            /* 开环工作电流 0..0x1388 mA */
    zdt_append_u16_be(buf, &i, close_current_ma);           /* 闭环最大电流 0..0x1388 mA */
    zdt_append_u16_be(buf, &i, close_max_rpm);              /* 闭环最大速度 0..0x0BB8 RPM */
    zdt_append_u16_be(buf, &i, current_bw_hz);              /* 电流环带宽 0..0xFFFF Hz */
    zdt_append_u8(buf, &i, uart_baud);                      /* 串口波特率 00..08 */
    zdt_append_u8(buf, &i, can_speed);                      /* CAN速率 00..09 */
    zdt_append_u8(buf, &i, check_mode);                     /* 通讯校验 00..04 */
    zdt_append_u8(buf, &i, reply_mode);                     /* 控制命令应答 00..04 */
    zdt_append_u8(buf, &i, angle_scale_10);                 /* 角度缩小10倍输入 00/01 */
    zdt_append_u8(buf, &i, stall_protect);                  /* 堵转保护 00=关 01=使能 02=复位不松轴 */
    zdt_append_u16_be(buf, &i, stall_speed_rpm);            /* 堵转检测转速 0..0x0BB8 RPM */
    zdt_append_u16_be(buf, &i, stall_current_ma);           /* 堵转检测电流 0..0x1388 mA */
    zdt_append_u16_be(buf, &i, stall_time_ms);              /* 堵转检测时间 0..0xFFFF ms */
    zdt_append_u16_be(buf, &i, pos_window_x10);             /* 位置到达窗口(×10, 默认8=0.8°) */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.8.5 读取驱动配置参数 (Emm) Addr + 42 + 6C + 6B — 4 字节
 * 返回 33B 帧 (字节数=0x21, 参数个数=0x15=21):
 *   电机类型 | 脉冲端口复用 | 通讯端口复用 | En有效电平 | Dir有效电平 |
 *   细分 | 细分插补 | 自动息屏 | 开环工作电流(BE16) | 闭环堵转最大电流(BE16) |
 *   闭环最大输出电压(BE16) | 串口波特率 | CAN速率 | ID/地址 | 通讯校验 |
 *   控制命令应答 | 堵转保护 | 堵转检测转速(BE16) | 堵转检测电流(BE16) |
 *   堵转检测时间(BE16) | 位置到达窗口(BE16)
 * 与 X 帧区别: Emm 用"电机类型"代替"控制模式", 且没有"角度缩小10倍输入"参数 */
int zdtBuildReadAllConfigEmmCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x42);   /* 功能码 */
    zdt_append_u8(buf, &i, 0x6C);   /* 辅助码 */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

/* 5.8.6 修改驱动配置参数 (Emm) — 33 字节
 *
 * 帧结构 (主机→电机):
 *   Addr | 48 | D1 | 是否存储 | 电机类型 | 脉冲端口复用 | 通讯端口复用 |
 *   En引脚有效电平 | Dir引脚有效电平 | 细分 | 细分插补 | 保留 |
 *   开环工作电流(BE16) | 闭环堵转最大电流(BE16) | 闭环最大输出电压(BE16) |
 *   串口波特率 | CAN速率 | 保留 | 通讯校验 | 控制命令应答 | 堵转保护 |
 *   堵转检测转速(BE16) | 堵转检测电流(BE16) | 堵转检测时间(BE16) |
 *   位置到达窗口(BE16) | 6B
 */
int zdtBuildWriteAllConfigEmmCmd(uint8_t addr, uint8_t store,
                                 uint8_t motor_type,
                                 uint8_t pul_mode, uint8_t com_mode,
                                 uint8_t en_level, uint8_t dir_level,
                                 uint8_t microstep, uint8_t interp,
                                 uint16_t open_current_ma,
                                 uint16_t stall_current_ma,
                                 uint16_t max_voltage_mv,
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
    if (zdt_check_size(need, buf_size) < 0) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, addr);
    zdt_append_u8(buf, &i, 0x48);   /* 功能码 */
    zdt_append_u8(buf, &i, 0xD1);   /* 辅助码 */
    zdt_append_u8(buf, &i, store);                          /* 是否存储 00/01 */
    zdt_append_u8(buf, &i, motor_type);                    /* 电机类型 19=1.8° 32=0.9° */
    zdt_append_u8(buf, &i, pul_mode);                       /* 脉冲端口复用 00..04 */
    zdt_append_u8(buf, &i, com_mode);                       /* 通讯端口复用 00..04 */
    zdt_append_u8(buf, &i, en_level);                       /* En有效电平 00=L 01=H 02=Hold */
    zdt_append_u8(buf, &i, dir_level);                      /* Dir有效电平 00=CW 01=CCW */
    zdt_append_u8(buf, &i, microstep);                      /* 细分 0..255 (0=256细分) */
    zdt_append_u8(buf, &i, interp);                         /* 细分插补 00/01 */
    zdt_append_u8(buf, &i, 0x00);                           /* 保留 */
    zdt_append_u16_be(buf, &i, open_current_ma);            /* 开环工作电流 0..0x1388 mA */
    zdt_append_u16_be(buf, &i, stall_current_ma);           /* 闭环堵转最大电流 0..0x1388 mA */
    zdt_append_u16_be(buf, &i, max_voltage_mv);             /* 闭环最大输出电压 0..0x1388 mV */
    zdt_append_u8(buf, &i, uart_baud);                      /* 串口波特率 00..08 */
    zdt_append_u8(buf, &i, can_speed);                      /* CAN速率 00..09 */
    zdt_append_u8(buf, &i, 0x00);                           /* 保留 */
    zdt_append_u8(buf, &i, check_mode);                     /* 通讯校验 00..04 */
    zdt_append_u8(buf, &i, reply_mode);                     /* 控制命令应答 00..04 */
    zdt_append_u8(buf, &i, stall_protect);                  /* 堵转保护 00/01 */
    zdt_append_u16_be(buf, &i, stall_speed_rpm);            /* 堵转检测转速 0..0x0BB8 RPM */
    zdt_append_u16_be(buf, &i, stall_current_ma2);          /* 堵转检测电流 0..0x1388 mA */
    zdt_append_u16_be(buf, &i, stall_time_ms);              /* 堵转检测时间 0..0xFFFF ms */
    zdt_append_u16_be(buf, &i, pos_window);                 /* 位置到达窗口 0..0xFFFF */
    zdt_append_u8(buf, &i, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}