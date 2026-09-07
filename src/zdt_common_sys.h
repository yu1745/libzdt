/*
 * zdt_common_sys.h - 通用读取系统参数模块 (CAN 扩展帧构建)
 *
 * 对应手册 5.5 读取系统参数 (共 18 条命令):
 *   5.5.1  zdtCanBuildSetPeriodicReportCmd
 *   5.5.2  zdtCanBuildReadVersionCmd
 *   5.5.3  zdtCanBuildReadPhaseRLCmd
 *   5.5.4  zdtCanBuildReadBusVoltageCmd
 *   5.5.5  zdtCanBuildReadBusCurrentCmd
 *   5.5.6  zdtCanBuildReadPhaseCurrentCmd
 *   5.5.7  zdtCanBuildReadEncoderCalibratedCmd
 *   5.5.8  zdtCanBuildReadInputPulsesCmd
 *   5.5.9  zdtCanBuildReadTargetPosCmd
 *   5.5.10 zdtCanBuildReadRealtimeTargetPosCmd
 *   5.5.11 zdtCanBuildReadRealtimeSpeedCmd
 *   5.5.12 zdtCanBuildReadDriverTempCmd
 *   5.5.13 zdtCanBuildReadRealtimePosCmd
 *   5.5.14 zdtCanBuildReadPosErrorCmd
 *   5.5.15 zdtCanBuildReadMotorStatusCmd
 *   5.5.16 zdtCanBuildReadHomingAndStatusCmd
 *   5.5.17 zdtCanBuildReadIoLevelCmd
 *   5.5.18 zdtCanBuildReadBatteryVoltageCmd
 *
 * 纯 C99，无任何硬件平台或操作系统依赖，零动态内存分配。
 */

#ifndef ZDT_COMMON_SYS_H
#define ZDT_COMMON_SYS_H

#include <stdint.h>
#include <stddef.h>
#include "zdt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  5.5 读取系统参数命令 (共 18 条)
 * ==================================================================== */

/**
 * @brief 5.5.1 定时返回信息命令 (X42S/Y42)
 * 原始命令: Addr + 11 + 18 + info_func_code + interval_ms(BE16) + 6B (7 字节 -> 1 帧 CAN)
 *
 * @param addr            电机地址 (0x01..0xFF, 0x00 为广播)
 * @param info_func_code  读取命令功能码 (如 0x31, 0x36 等)
 * @param interval_ms     定时时间 (单位 ms，0 为关闭)
 * @param msg             输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildSetPeriodicReportCmd(uint8_t addr, uint8_t info_func_code,
                                   uint16_t interval_ms,
                                   zdt_can_msg_t *msg);

/**
 * @brief 5.5.2 读取固件版本和硬件版本
 * 原始命令: Addr + 1F + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadVersionCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.3 读取相电阻和相电感
 * 原始命令: Addr + 20 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadPhaseRLCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.4 读取总线电压
 * 原始命令: Addr + 24 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadBusVoltageCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.5 读取总线电流 (X42S/Y42)
 * 原始命令: Addr + 26 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadBusCurrentCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.6 读取相电流
 * 原始命令: Addr + 27 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadPhaseCurrentCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.7 读取经过线性化校准后的编码器值
 * 原始命令: Addr + 31 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadEncoderCalibratedCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.8 读取输入脉冲数
 * 原始命令: Addr + 32 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadInputPulsesCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.9 读取电机目标位置
 * 原始命令: Addr + 33 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadTargetPosCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.10 读取电机实时设定的目标位置
 * 原始命令: Addr + 34 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadRealtimeTargetPosCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.11 读取电机实时转速
 * 原始命令: Addr + 35 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadRealtimeSpeedCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.12 读取驱动温度 (X42S/Y42)
 * 原始命令: Addr + 39 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadDriverTempCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.13 读取电机实时位置
 * 原始命令: Addr + 36 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadRealtimePosCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.14 读取电机位置误差
 * 原始命令: Addr + 37 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadPosErrorCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.15 读取电机状态标志
 * 原始命令: Addr + 3A + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadMotorStatusCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.16 读取回零状态标志 + 电机状态标志 (X42S/Y42)
 * 原始命令: Addr + 3C + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadHomingAndStatusCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.17 读取引脚 IO 电平状态 (X42S/Y42)
 * 原始命令: Addr + 3D + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadIoLevelCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.5.18 读取电池电压 (Y42)
 * 原始命令: Addr + 38 + 6B (4 字节 -> 1 帧 CAN)
 *
 * @param addr 电机地址 (0x01..0xFF, 0x00 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回生成的 CAN 帧数 (>=1)，失败返回负数错误码
 */
int zdtCanBuildReadBatteryVoltageCmd(uint8_t addr, zdt_can_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_COMMON_SYS_H */
