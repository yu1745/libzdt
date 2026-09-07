/*
 * zdt_common_motion.h - 通用动作与运动控制模块 (CAN 扩展帧构建)
 *
 * CAN 分帧说明：下列“原始命令”包含 Addr，供与手册逐字节对照；CAN 将
 * Addr 编入 EID=(Addr<<8)|Packet（Packet 从 0 开始）。CAN data 从 Code
 * 开始、不含 Addr，且每个 data payload 最多 8 字节。
 *
 * 包含双固件通用命令 (共 15 条):
 *   5.2 触发动作 (5条):
 *     5.2.1 zdtCanBuildEncoderCalibrationCmd
 *     5.2.2 zdtCanBuildRestartMotorCmd
 *     5.2.3 zdtCanBuildClearCurrentAngleCmd
 *     5.2.4 zdtCanBuildClearProtectionCmd
 *     5.2.5 zdtCanBuildFactoryResetCmd
 *   5.3 运动控制 (通用4条):
 *     5.3.1 zdtCanBuildMultiMotorCmd (广播命令，addr 用 ZDT_ADDR_BROADCAST)
 *     5.3.2 zdtCanBuildMotorEnableCmd
 *     5.3.13 zdtCanBuildImmediateStopCmd
 *     5.3.14 zdtCanBuildSyncMotionCmd
 *   5.4 原点回零 (6条):
 *     5.4.1 zdtCanBuildSetSingleTurnZeroCmd
 *     5.4.2 zdtCanBuildTriggerHomingCmd
 *     5.4.3 zdtCanBuildAbortHomingCmd
 *     5.4.4 zdtCanBuildReadHomingStatusCmd
 *     5.4.5 zdtCanBuildReadHomingParamsCmd
 *     5.4.6 zdtCanBuildWriteHomingParamsCmd
 *
 * 纯 C99，无任何硬件平台或操作系统依赖，零动态内存分配。
 */

#ifndef ZDT_COMMON_MOTION_H
#define ZDT_COMMON_MOTION_H

#include <stdint.h>
#include <stddef.h>
#include "zdt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  5.2 触发动作命令 (5条)
 * ==================================================================== */

/**
 * @brief 5.2.1 触发编码器校准
 * 原始命令: Addr + 06 + 45 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildEncoderCalibrationCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.2.2 重启电机 (X42S/Y42)
 * 原始命令: Addr + 08 + 97 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildRestartMotorCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.2.3 将当前位置角度清零
 * 原始命令: Addr + 0A + 6D + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildClearCurrentAngleCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.2.4 解除堵转/过热/过流保护
 * 原始命令: Addr + 0E + 52 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildClearProtectionCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.2.5 恢复出厂设置
 * 原始命令: Addr + 0F + 5F + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildFactoryResetCmd(uint8_t addr, zdt_can_msg_t *msg);

/* ====================================================================
 *  5.3 运动控制命令 (通用 4 条)
 * ==================================================================== */

/**
 * @brief 5.3.1 多电机命令 (广播)
 * 原始命令: 00 + AA + 总字节数(BE16) + 子命令序列 + 6B (总长 5 + sub_cmds_len)
 *
 * @param sub_cmds     子命令字节流缓冲区指针
 * @param sub_cmds_len 子命令字节长度
 * @param msg          输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildMultiMotorCmd(const uint8_t *sub_cmds, size_t sub_cmds_len,
                             zdt_can_msg_t *msg);

/**
 * @brief 5.3.2 电机使能控制
 * 原始命令: Addr + F3 + AB + enable + sync + 6B (6 字节 -> 1 帧 CAN)
 *
 * @param addr   电机地址 (0x01..0xFF, 0x00 为广播)
 * @param enable 0x00=释放(掉电), 0x01=使能(上电锁轴)
 * @param sync   同步标志: ZDT_SYNC_NOW(0x00) / ZDT_SYNC_CACHE(0x01)
 * @param msg    输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildMotorEnableCmd(uint8_t addr, uint8_t enable, uint8_t sync,
                             zdt_can_msg_t *msg);

/**
 * @brief 5.3.13 立即停止
 * 原始命令: Addr + FE + 98 + sync + 6B (5 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param sync 同步标志: ZDT_SYNC_NOW(0x00) / ZDT_SYNC_CACHE(0x01)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildImmediateStopCmd(uint8_t addr, uint8_t sync,
                               zdt_can_msg_t *msg);

/**
 * @brief 5.3.14 触发多机同步运动 (通常广播)
 * 原始命令: Addr + FF + 66 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (通常使用 ZDT_ADDR_BROADCAST 即 0x00)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildSyncMotionCmd(uint8_t addr, zdt_can_msg_t *msg);

/* ====================================================================
 *  5.4 原点回零命令 (6条)
 * ==================================================================== */

/**
 * @brief 5.4.1 设置单圈回零的零点位置
 * 原始命令: Addr + 93 + 88 + store + 6B (5 字节 -> 1 帧 CAN)
 *
 * @param addr  电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store 是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param msg   输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildSetSingleTurnZeroCmd(uint8_t addr, uint8_t store,
                                   zdt_can_msg_t *msg);

/**
 * @brief 5.4.2 触发回零
 * 原始命令: Addr + 9A + homing_mode + sync + 6B (5 字节 -> 1 帧 CAN)
 *
 * @param addr        电机地址 (0x01..0xFF, 0x00 为广播)
 * @param homing_mode 回零模式 (0x00:单圈就近, 0x01:单圈方向, 0x02:无限位碰撞, 0x03:限位回零, 0x04:回到绝对位置坐标零点, 0x05:回到上次掉电位置角度)
 * @param sync        同步标志: ZDT_SYNC_NOW(0x00) / ZDT_SYNC_CACHE(0x01)
 * @param msg         输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildTriggerHomingCmd(uint8_t addr, uint8_t homing_mode,
                               uint8_t sync, zdt_can_msg_t *msg);

/**
 * @brief 5.4.3 强制中断并退出回零
 * 原始命令: Addr + 9C + 48 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildAbortHomingCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.4.4 读取回零状态标志
 * 原始命令: Addr + 3B + 6B (3 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadHomingStatusCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.4.5 读取回零参数
 * 原始命令: Addr + 22 + 6B (3 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadHomingParamsCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.4.6 修改回零参数
 * 原始命令: Addr + 4C + AE + store + mode + dir + 速度(BE16) + 超时(BE32)
 *           + 碰撞转速(BE16) + 碰撞电流(BE16) + 碰撞时间(BE16) + auto_trigger + 6B (20 字节 -> 3 帧 CAN)
 *
 * @param addr             电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store            是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param homing_mode      回零模式 (0x00..0x05)
 * @param dir              回零方向: ZDT_DIR_CW(0x00) / ZDT_DIR_CCW(0x01)
 * @param rpm              回零速度 (单位 RPM)
 * @param timeout_ms       回零超时时间 (单位 ms)
 * @param bump_speed_rpm   碰撞回零检测转速 (单位 RPM)
 * @param bump_current_ma  碰撞回零检测电流 (单位 mA)
 * @param bump_time_ms     碰撞回零检测时间 (单位 ms)
 * @param auto_trigger     上电自动触发回零使能 (0x00:禁用, 0x01:使能)
 * @param msg              输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildWriteHomingParamsCmd(uint8_t addr, uint8_t store,
                                   uint8_t homing_mode, uint8_t dir,
                                   uint16_t rpm, uint32_t timeout_ms,
                                   uint16_t bump_speed_rpm,
                                   uint16_t bump_current_ma,
                                   uint16_t bump_time_ms,
                                   uint8_t auto_trigger,
                                   zdt_can_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_COMMON_MOTION_H */
