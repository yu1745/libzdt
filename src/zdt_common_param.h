/*
 * zdt_common_param.h - 通用读写驱动参数模块 (CAN 扩展帧构建)
 *
 * CAN 分帧说明：下列“原始命令”包含 Addr，供与手册逐字节对照；CAN 将
 * Addr 编入 EID=(Addr<<8)|Packet（Packet 从 0 开始）。CAN data 从 Code
 * 开始、不含 Addr，且每个 data payload 最多 8 字节。
 *
 * 对应手册 5.6 读写驱动参数通用部分 (共 25 条命令):
 *   5.6.1  zdtCanBuildChangeAddrCmd
 *   5.6.2  zdtCanBuildChangeMicrostepCmd
 *   5.6.3  zdtCanBuildChangePowerDownFlagCmd
 *   5.6.4  zdtCanBuildReadOptionsCmd
 *   5.6.5  zdtCanBuildChangeMotorTypeCmd
 *   5.6.6  zdtCanBuildChangeFirmwareTypeCmd
 *   5.6.7  zdtCanBuildChangeCtrlModeCmd
 *   5.6.8  zdtCanBuildChangeMotorDirCmd
 *   5.6.9  zdtCanBuildChangeKeyLockCmd
 *   5.6.12 zdtCanBuildChangeOpenLoopCurrentCmd
 *   5.6.13 zdtCanBuildChangeClosedLoopCurrentCmd
 *   5.6.18 zdtCanBuildReadDmx512Cmd
 *   5.6.19 zdtCanBuildWriteDmx512Cmd
 *   5.6.20 zdtCanBuildReadPosWindowCmd
 *   5.6.21 zdtCanBuildWritePosWindowCmd
 *   5.6.22 zdtCanBuildReadProtectThresholdCmd
 *   5.6.23 zdtCanBuildWriteProtectThresholdCmd
 *   5.6.24 zdtCanBuildReadHeartbeatCmd
 *   5.6.25 zdtCanBuildWriteHeartbeatCmd
 *   5.6.26 zdtCanBuildReadIntegralLimitCmd
 *   5.6.27 zdtCanBuildWriteIntegralLimitCmd
 *   5.6.28 zdtCanBuildReadBumpReturnAngleCmd
 *   5.6.29 zdtCanBuildWriteBumpReturnAngleCmd
 *   5.6.30 zdtCanBuildBroadcastReadAddrCmd
 *   5.6.31 zdtCanBuildChangeParamLockCmd
 *
 * 纯 C99，无任何硬件平台或操作系统依赖，零动态内存分配。
 */

#ifndef ZDT_COMMON_PARAM_H
#define ZDT_COMMON_PARAM_H

#include <stdint.h>
#include <stddef.h>
#include "zdt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  5.6 读写驱动参数命令 (通用 25 条)
 * ==================================================================== */

/**
 * @brief 5.6.1 修改电机 ID/地址
 * 原始命令: Addr + AE + 4B + store + new_addr + 6B (6 字节 -> 1 帧 CAN)
 *
 * @param addr     当前电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store    是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param new_addr 新地址 (0x01..0xFF)
 * @param msg      输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangeAddrCmd(uint8_t addr, uint8_t store,
                            uint8_t new_addr,
                            zdt_can_msg_t *msg);

/**
 * @brief 5.6.2 修改细分值
 * 原始命令: Addr + 84 + 8A + store + microstep + 6B (6 字节 -> 1 帧 CAN)
 *
 * @param addr      电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store     是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param microstep 细分值的线码：0x00=256 细分；0x01..0xFF=1..255 细分
 * @param msg       输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangeMicrostepCmd(uint8_t addr, uint8_t store,
                                 uint8_t microstep,
                                 zdt_can_msg_t *msg);

/**
 * @brief 5.6.3 修改掉电标志
 * 原始命令: Addr + 50 + flag + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param flag 掉电指示标志: 可写入 0x00；掉电后重新上电恢复为 0x01，用于检测发生过掉电
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangePowerDownFlagCmd(uint8_t addr, uint8_t flag,
                                     zdt_can_msg_t *msg);

/**
 * @brief 5.6.4 读取选项参数状态 (X42S/Y42)
 * 原始命令: Addr + 1A + 6B (3 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadOptionsCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.6.5 修改电机类型
 * 原始命令: Addr + D7 + 35 + store + motor_type + 6B (6 字节 -> 1 帧 CAN)
 *
 * @param addr       电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store      是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param motor_type 电机类型 (0x19: 0.9°电机 / 0x32: 1.8°电机)
 * @param msg        输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangeMotorTypeCmd(uint8_t addr, uint8_t store,
                                 uint8_t motor_type,
                                 zdt_can_msg_t *msg);

/**
 * @brief 5.6.6 修改固件类型
 * 原始命令: Addr + D5 + 69 + store + fw_type + 6B (6 字节 -> 1 帧 CAN)
 *
 * @param addr    电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store   是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param fw_type 固件类型 (0x00: X 固件, 0x01: Emm 固件, 0x02: Emm 狂暴模式)
 * @param msg     输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangeFirmwareTypeCmd(uint8_t addr, uint8_t store,
                                    uint8_t fw_type,
                                    zdt_can_msg_t *msg);

/**
 * @brief 5.6.7 修改开环/闭环控制模式
 * 原始命令: Addr + 46 + A6 + store + mode + 6B (6 字节 -> 1 帧 CAN)
 *
 * @param addr  电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store 是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param mode  控制模式 (0x00: 开环控制 / 0x01: 闭环控制)
 * @param msg   输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangeCtrlModeCmd(uint8_t addr, uint8_t store,
                                uint8_t mode,
                                zdt_can_msg_t *msg);

/**
 * @brief 5.6.8 修改电机运动正方向
 * 原始命令: Addr + D4 + 60 + store + dir + 6B (6 字节 -> 1 帧 CAN)
 *
 * @param addr  电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store 是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param dir   默认方向: ZDT_DIR_CW(0x00) / ZDT_DIR_CCW(0x01)
 * @param msg   输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangeMotorDirCmd(uint8_t addr, uint8_t store,
                                uint8_t dir,
                                zdt_can_msg_t *msg);

/**
 * @brief 5.6.9 修改锁定按键功能
 * 原始命令: Addr + D0 + B3 + store + lock + 6B (6 字节 -> 1 帧 CAN)
 *
 * @param addr  电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store 是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param lock  按键锁: 0x00 不锁定 / 0x01 锁定
 * @param msg   输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangeKeyLockCmd(uint8_t addr, uint8_t store,
                               uint8_t lock,
                               zdt_can_msg_t *msg);

/**
 * @brief 5.6.12 修改开环模式工作电流
 * 原始命令: Addr + 44 + 33 + store + current(BE16) + 6B (7 字节 -> 1 帧 CAN)
 *
 * @param addr       电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store      是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param current_ma 开环工作电流 (单位 mA)
 * @param msg        输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangeOpenLoopCurrentCmd(uint8_t addr, uint8_t store,
                                       uint16_t current_ma,
                                       zdt_can_msg_t *msg);

/**
 * @brief 5.6.13 修改闭环模式最大电流
 * 原始命令: Addr + 45 + 66 + store + current(BE16) + 6B (7 字节 -> 1 帧 CAN)
 *
 * @param addr       电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store      是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param current_ma 闭环最大工作电流 (单位 mA)
 * @param msg        输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangeClosedLoopCurrentCmd(uint8_t addr, uint8_t store,
                                         uint16_t current_ma,
                                         zdt_can_msg_t *msg);

/**
 * @brief 5.6.18 读取 DMX512 协议参数 (X42S/Y42)
 * 原始命令: Addr + 49 + 78 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadDmx512Cmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.6.19 修改 DMX512 协议参数 (X42S/Y42)
 * 原始命令: Addr + D9 + 90 + store + total_ch(BE16) + ch_per_motor(1B) + move_mode(1B)
 *           + single_speed(BE16) + accel(BE16) + speed_step(BE16) + motion_step(BE32) + 6B (19 字节 -> 3 帧 CAN)
 *
 * @param addr               电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store              是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param total_channels     总通道数 (1..64)
 * @param channels_per_motor 每台电机占用通道数 (1 或 2)
 * @param move_mode          运动模式 (0x00: 相对, 0x01: 绝对)
 * @param single_speed_rpm   单通道最高转速 (1..3000 RPM)
 * @param accel              加减速加速度
 * @param speed_step         速度步进调节量
 * @param motion_step        位置步进调节量
 * @param msg                输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildWriteDmx512Cmd(uint8_t addr, uint8_t store,
                             uint16_t total_channels,
                             uint8_t channels_per_motor,
                             uint8_t move_mode,
                             uint16_t single_speed_rpm,
                             uint16_t accel,
                             uint16_t speed_step,
                             uint32_t motion_step,
                             zdt_can_msg_t *msg);

/**
 * @brief 5.6.20 读取位置到达窗口 (X42S/Y42)
 * 原始命令: Addr + 41 + 6B (3 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadPosWindowCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.6.21 修改位置到达窗口 (X42S/Y42)
 * 原始命令: Addr + D1 + 07 + store + window(BE16, ×0.1°) + 6B (7 字节 -> 1 帧 CAN)
 *
 * @param addr       电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store      是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param window_x10 到达窗口阈值 (BE16，单位 0.1°；例如 8 代表 0.8°)
 * @param msg        输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildWritePosWindowCmd(uint8_t addr, uint8_t store,
                                uint16_t window_x10,
                                zdt_can_msg_t *msg);

/**
 * @brief 5.6.22 读取过热过流保护检测阈值 (X42S/Y42)
 * 原始命令: Addr + 13 + 6B (3 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadProtectThresholdCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.6.23 修改过热过流保护检测阈值 (X42S/Y42)
 * 原始命令: Addr + D3 + 56 + store + overheat(BE16) + overcurrent(BE16) + detect_ms(BE16) + 6B (11 字节 -> 2 帧 CAN)
 *
 * @param addr           电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store          是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param overheat_c     过热保护阈值 (单位 ℃)
 * @param overcurrent_ma 过流保护阈值 (单位 mA)
 * @param detect_ms      保护持续判定时间 (单位 ms)
 * @param msg            输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildWriteProtectThresholdCmd(uint8_t addr, uint8_t store,
                                       uint16_t overheat_c,
                                       uint16_t overcurrent_ma,
                                       uint16_t detect_ms,
                                       zdt_can_msg_t *msg);

/**
 * @brief 5.6.24 读取心跳保护功能时间 (X42S/Y42)
 * 原始命令: Addr + 16 + 6B (3 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadHeartbeatCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.6.25 修改心跳保护功能时间 (X42S/Y42)
 * 原始命令: Addr + 68 + 38 + store + heartbeat_ms(BE32) + 6B (9 字节 -> 1 帧 CAN)
 *
 * @param addr         电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store        是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param heartbeat_ms 心跳超时时间 (单位 ms，0 为关闭)
 * @param msg          输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildWriteHeartbeatCmd(uint8_t addr, uint8_t store,
                                uint32_t heartbeat_ms,
                                zdt_can_msg_t *msg);

/**
 * @brief 5.6.26 读取积分限幅/刚性系数 (X42S/Y42)
 * 原始命令: Addr + 23 + 6B (3 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadIntegralLimitCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.6.27 修改积分限幅/刚性系数 (X42S/Y42)
 * 原始命令: Addr + 4B + 57 + store + value(BE32) + 6B (9 字节 -> 1 帧 CAN)
 *
 * @param addr  电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store 是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param value 积分限幅/刚性系数值 (BE32)
 * @param msg   输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildWriteIntegralLimitCmd(uint8_t addr, uint8_t store,
                                    uint32_t value,
                                    zdt_can_msg_t *msg);

/**
 * @brief 5.6.28 读取碰撞回零返回角度 (X42S/Y42)
 * 原始命令: Addr + 3F + 6B (3 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadBumpReturnAngleCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.6.29 修改碰撞回零返回角度 (X42S/Y42)
 * 原始命令: Addr + 5C + AC + store + angle(BE16) + 6B (7 字节 -> 1 帧 CAN)
 *
 * @param addr      电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store     是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param angle_x10 返回角度值 (单位 0.1°，例如 200 = 20.0°)
 * @param msg       输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildWriteBumpReturnAngleCmd(uint8_t addr, uint8_t store,
                                      uint16_t angle_x10,
                                      zdt_can_msg_t *msg);

/**
 * @brief 5.6.30 广播读取 ID 地址 (广播命令)
 * 原始命令: 00 + 15 + 6B (3 字节 -> 1 帧 CAN)
 *
 * @param msg 输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildBroadcastReadAddrCmd(zdt_can_msg_t *msg);

/**
 * @brief 5.6.31 修改锁定修改参数功能 (X42S/Y42)
 * 原始命令: Addr + D6 + 4B + store + lock_level + 6B (6 字节 -> 1 帧 CAN)
 *
 * @param addr       电机地址 (0x01..0xFF, 0x00 为广播)
 * @param store      是否存储: ZDT_STORE_NO(0x00) / ZDT_STORE_YES(0x01)
 * @param lock_level 锁定级别 (0..3)
 * @param msg        输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildChangeParamLockCmd(uint8_t addr, uint8_t store,
                                 uint8_t lock_level,
                                 zdt_can_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_COMMON_PARAM_H */
