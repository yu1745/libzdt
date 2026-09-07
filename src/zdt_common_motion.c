/*
 * zdt_common_motion.c - 通用动作与运动控制模块 (CAN 扩展帧构建实现)
 *
 * 纯 C99，无任何硬件平台或操作系统依赖，零动态内存分配。
 */

#include "zdt_common_motion.h"
#include "zdt_internal.h"
#include <string.h>

/* ====================================================================
 *  5.2 触发动作命令 (5条)
 * ==================================================================== */

/* 5.2.1 触发编码器校准
 * Addr + 06 + 45 + 6B — 4 字节 */
static int _raw_EncoderCalibrationCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x06);
    zdt_append_u8(buf, &i, buf_size, 0x45);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildEncoderCalibrationCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_EncoderCalibrationCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.2.2 重启电机 (X42S/Y42)
 * Addr + 08 + 97 + 6B — 4 字节 */
static int _raw_RestartMotorCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x08);
    zdt_append_u8(buf, &i, buf_size, 0x97);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildRestartMotorCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_RestartMotorCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.2.3 将当前位置角度清零
 * Addr + 0A + 6D + 6B — 4 字节 */
static int _raw_ClearCurrentAngleCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x0A);
    zdt_append_u8(buf, &i, buf_size, 0x6D);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildClearCurrentAngleCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_ClearCurrentAngleCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.2.4 解除堵转/过热/过流保护
 * Addr + 0E + 52 + 6B — 4 字节 */
static int _raw_ClearProtectionCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x0E);
    zdt_append_u8(buf, &i, buf_size, 0x52);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildClearProtectionCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_ClearProtectionCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.2.5 恢复出厂设置
 * Addr + 0F + 5F + 6B — 4 字节 */
static int _raw_FactoryResetCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x0F);
    zdt_append_u8(buf, &i, buf_size, 0x5F);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildFactoryResetCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_FactoryResetCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* ====================================================================
 *  5.3 运动控制命令 (通用 4 条)
 * ==================================================================== */

/* 5.3.1 多电机命令 (广播)
 * 00 + AA + 总字节数(BE16) + 子命令序列 + 6B — (5 + sub_cmds_len) 字节
 * 注：广播命令，打包时 addr 固定使用 ZDT_ADDR_BROADCAST */
static int _raw_MultiMotorCmd(const uint8_t *sub_cmds, size_t sub_cmds_len,
                              uint8_t *buf, size_t buf_size)
{
    if (!sub_cmds || sub_cmds_len == 0) return ZDT_ERR_PARAM;
    const size_t need = 5 + sub_cmds_len;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;

    zdt_append_u8(buf, &i, buf_size, ZDT_ADDR_BROADCAST);
    zdt_append_u8(buf, &i, buf_size, 0xAA);
    zdt_append_u16_be(buf, &i, buf_size, (uint16_t)need);
    memcpy(&buf[i], sub_cmds, sub_cmds_len);
    i += sub_cmds_len;
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildMultiMotorCmd(const uint8_t *sub_cmds, size_t sub_cmds_len,
                             zdt_can_msg_t *msg)
{
    if (!sub_cmds || sub_cmds_len == 0 || !msg) return ZDT_ERR_PARAM;
    const size_t need = 5 + sub_cmds_len;
    /* ZDT_CAN_MAX_FRAMES 为 16 包，每包最多 8 字节 payload，故最大支持 raw 长度为 1 + 16*8 = 129 字节 */
    if (need > (1 + ZDT_CAN_MAX_FRAMES * 8)) {
        return ZDT_ERR_BUF_TOO_SMALL;
    }
    uint8_t raw[1 + ZDT_CAN_MAX_FRAMES * 8];
    int n = _raw_MultiMotorCmd(sub_cmds, sub_cmds_len, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(ZDT_ADDR_BROADCAST, raw, (size_t)n, msg);
}

/* 5.3.2 电机使能控制
 * Addr + F3 + AB + enable + sync + 6B — 6 字节 */
static int _raw_MotorEnableCmd(uint8_t addr, uint8_t enable, uint8_t sync,
                               uint8_t *buf, size_t buf_size)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0xF3);
    zdt_append_u8(buf, &i, buf_size, 0xAB);
    zdt_append_u8(buf, &i, buf_size, enable);
    zdt_append_u8(buf, &i, buf_size, sync);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildMotorEnableCmd(uint8_t addr, uint8_t enable, uint8_t sync,
                             zdt_can_msg_t *msg)
{
    uint8_t raw[6];
    int n = _raw_MotorEnableCmd(addr, enable, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.3.13 立即停止
 * Addr + FE + 98 + sync + 6B — 5 字节 */
static int _raw_ImmediateStopCmd(uint8_t addr, uint8_t sync,
                                 uint8_t *buf, size_t buf_size)
{
    const size_t need = 5;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0xFE);
    zdt_append_u8(buf, &i, buf_size, 0x98);
    zdt_append_u8(buf, &i, buf_size, sync);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildImmediateStopCmd(uint8_t addr, uint8_t sync,
                               zdt_can_msg_t *msg)
{
    uint8_t raw[5];
    int n = _raw_ImmediateStopCmd(addr, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.3.14 触发同步运动
 * Addr + FF + 66 + 6B — 4 字节 */
static int _raw_SyncMotionCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0xFF);
    zdt_append_u8(buf, &i, buf_size, 0x66);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildSyncMotionCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_SyncMotionCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* ====================================================================
 *  5.4 原点回零 (6条)
 * ==================================================================== */

/* 5.4.1 设置单圈回零的零点位置
 * Addr + 93 + 88 + store + 6B — 5 字节 */
static int _raw_SetSingleTurnZeroCmd(uint8_t addr, uint8_t store,
                                     uint8_t *buf, size_t buf_size)
{
    const size_t need = 5;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x93);
    zdt_append_u8(buf, &i, buf_size, 0x88);
    zdt_append_u8(buf, &i, buf_size, store);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildSetSingleTurnZeroCmd(uint8_t addr, uint8_t store,
                                   zdt_can_msg_t *msg)
{
    uint8_t raw[5];
    int n = _raw_SetSingleTurnZeroCmd(addr, store, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.4.2 触发回零
 * Addr + 9A + homing_mode + sync + 6B — 5 字节 */
static int _raw_TriggerHomingCmd(uint8_t addr, uint8_t homing_mode,
                                 uint8_t sync, uint8_t *buf, size_t buf_size)
{
    const size_t need = 5;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x9A);
    zdt_append_u8(buf, &i, buf_size, homing_mode);
    zdt_append_u8(buf, &i, buf_size, sync);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildTriggerHomingCmd(uint8_t addr, uint8_t homing_mode,
                               uint8_t sync, zdt_can_msg_t *msg)
{
    uint8_t raw[5];
    int n = _raw_TriggerHomingCmd(addr, homing_mode, sync, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.4.3 强制中断并退出回零
 * Addr + 9C + 48 + 6B — 4 字节 */
static int _raw_AbortHomingCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x9C);
    zdt_append_u8(buf, &i, buf_size, 0x48);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildAbortHomingCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[4];
    int n = _raw_AbortHomingCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.4.4 读取回零状态标志
 * Addr + 3B + 6B — 3 字节 */
static int _raw_ReadHomingStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x3B);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadHomingStatusCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadHomingStatusCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.4.5 读取回零参数
 * Addr + 22 + 6B — 3 字节 */
static int _raw_ReadHomingParamsCmd(uint8_t addr, uint8_t *buf, size_t buf_size)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x22);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadHomingParamsCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[3];
    int n = _raw_ReadHomingParamsCmd(addr, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}

/* 5.4.6 修改回零参数
 * Addr + 4C + AE + store + mode + dir + 速度(BE16) + 超时(BE32)
 * + 碰撞转速(BE16) + 碰撞电流(BE16) + 碰撞时间(BE16) + auto_trigger + 6B — 20 字节 */
static int _raw_WriteHomingParamsCmd(uint8_t addr, uint8_t store,
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
    if (zdt_check_size(0, need, buf_size) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, buf_size, addr);
    zdt_append_u8(buf, &i, buf_size, 0x4C);
    zdt_append_u8(buf, &i, buf_size, 0xAE);
    zdt_append_u8(buf, &i, buf_size, store);
    zdt_append_u8(buf, &i, buf_size, homing_mode);
    zdt_append_u8(buf, &i, buf_size, dir);
    zdt_append_u16_be(buf, &i, buf_size, rpm);
    zdt_append_u32_be(buf, &i, buf_size, timeout_ms);
    zdt_append_u16_be(buf, &i, buf_size, bump_speed_rpm);
    zdt_append_u16_be(buf, &i, buf_size, bump_current_ma);
    zdt_append_u16_be(buf, &i, buf_size, bump_time_ms);
    zdt_append_u8(buf, &i, buf_size, auto_trigger);
    zdt_append_u8(buf, &i, buf_size, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWriteHomingParamsCmd(uint8_t addr, uint8_t store,
                                   uint8_t homing_mode, uint8_t dir,
                                   uint16_t rpm, uint32_t timeout_ms,
                                   uint16_t bump_speed_rpm,
                                   uint16_t bump_current_ma,
                                   uint16_t bump_time_ms,
                                   uint8_t auto_trigger,
                                   zdt_can_msg_t *msg)
{
    uint8_t raw[20];
    int n = _raw_WriteHomingParamsCmd(addr, store, homing_mode, dir, rpm, timeout_ms,
                                      bump_speed_rpm, bump_current_ma, bump_time_ms,
                                      auto_trigger, raw, sizeof(raw));
    if (n < 0) return n;
    return zdt_pack_can_msg(addr, raw, (size_t)n, msg);
}
