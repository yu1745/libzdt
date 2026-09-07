/*
 * zdt_emm.h - ZDT Emm 固件专属原生 CAN 命令构建接口
 *
 * 纯 C99，无任何硬件平台或操作系统依赖，零动态内存分配。
 * 本模块包含 9 条 Emm 固件特有命令的 CAN 报文构建函数：
 *   - 5.3.7  zdtCanBuildSpeedModeEmmCmd
 *   - 5.3.12 zdtCanBuildPosModeEmmCmd
 *   - 5.6.11 zdtCanBuildChangeSpeedScaleCmd
 *   - 5.6.16 zdtCanBuildReadPidEmmCmd
 *   - 5.6.17 zdtCanBuildWritePidEmmCmd
 *   - 5.7.2  zdtCanBuildStoreAutoRunEmmCmd
 *   - 5.8.2  zdtCanBuildReadAllStatusEmmCmd
 *   - 5.8.5  zdtCanBuildReadAllConfigEmmCmd
 *   - 5.8.6  zdtCanBuildWriteAllConfigEmmCmd
 */

#ifndef ZDT_EMM_H
#define ZDT_EMM_H

#include "zdt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  5.3 运动控制命令 (Emm 固件特有)
 * ==================================================================== */

/**
 * @brief 5.3.7 速度模式控制 (Emm)
 *
 * 原始命令格式: Addr + F6 + 方向 + 速度(BE16, RPM) + 加速度(1B, 档位) + 同步 + 6B (8 字节)
 *
 * @param addr  电机地址 (1..255, 0 为广播)
 * @param dir   方向: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param rpm   速度: 0..3000 (单位 RPM，整数)
 * @param acc   加速度: 0..255 档位 (0 为最高加速度)
 * @param sync  多机同步标志: ZDT_SYNC_NOW(0x00) 或 ZDT_SYNC_CACHE(0x01)
 * @param msg   输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildSpeedModeEmmCmd(uint8_t addr, uint8_t dir, uint16_t rpm,
                              uint8_t acc, uint8_t sync,
                              zdt_can_msg_t *msg);

/**
 * @brief 5.3.12 位置模式控制 (Emm)
 *
 * 原始命令格式: Addr + FD + 方向 + 速度(BE16, RPM) + 加速度(1B, 档位) + 脉冲数/编码器值(BE32) + 运动模式 + 同步 + 6B (13 字节)
 *
 * @param addr       电机地址 (1..255, 0 为广播)
 * @param dir        方向: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param rpm        速度: 0..3000 (单位 RPM，整数)
 * @param acc        加速度: 0..255 档位
 * @param pulses     脉冲数/编码器计数值 (BE32)
 * @param move_mode  运动模式: ZDT_MOVE_REL_LAST / ABS_ZERO / REL_NOW
 * @param sync       多机同步标志: ZDT_SYNC_NOW(0x00) 或 ZDT_SYNC_CACHE(0x01)
 * @param msg        输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildPosModeEmmCmd(uint8_t addr, uint8_t dir, uint16_t rpm,
                             uint8_t acc, uint32_t pulses,
                             uint8_t move_mode, uint8_t sync,
                             zdt_can_msg_t *msg);

/* ====================================================================
 *  5.6 修改驱动参数命令 (Emm 固件特有)
 * ==================================================================== */

/**
 * @brief 5.6.11 修改命令速度值是否缩小 10 倍输入 (Emm)
 *
 * 原始命令格式: Addr + 4F + 71 + store + scale(00/01) + 6B (6 字节)
 *
 * @param addr       电机地址 (1..255, 0 为广播)
 * @param store      是否存储: ZDT_STORE_NO(0x00) 或 ZDT_STORE_YES(0x01)
 * @param scale_10x  00=不缩小(单位RPM), 01=缩小10倍(单位0.1RPM)
 * @param msg        输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildChangeSpeedScaleCmd(uint8_t addr, uint8_t store,
                                   uint8_t scale_10x,
                                   zdt_can_msg_t *msg);

/**
 * @brief 5.6.16 读取 PID 参数 (Emm)
 *
 * 原始命令格式: Addr + 21 + 6B (3 字节)
 *
 * @param addr 电机地址 (1..255, 0 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildReadPidEmmCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.6.17 修改 PID 参数 (Emm)
 *
 * 原始命令格式: Addr + 4A + C3 + store + Kp(BE32) + Ki(BE32) + Kd(BE32) + 6B (17 字节)
 *
 * @param addr   电机地址 (1..255, 0 为广播)
 * @param store  是否存储: ZDT_STORE_NO(0x00) 或 ZDT_STORE_YES(0x01)
 * @param kp     比例系数 Kp
 * @param ki     积分系数 Ki
 * @param kd     微分系数 Kd
 * @param msg    输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildWritePidEmmCmd(uint8_t addr, uint8_t store,
                              uint32_t kp, uint32_t ki, uint32_t kd,
                              zdt_can_msg_t *msg);

/* ====================================================================
 *  5.7 上电自动运行命令 (Emm 固件特有)
 * ==================================================================== */

/**
 * @brief 5.7.2 存储一组速度参数，上电自动运行 (Emm)
 *
 * 原始命令格式: Addr + F7 + 1C + store + dir + speed(BE16) + acc(1B) + en + 6B (10 字节)
 *
 * @param addr           电机地址 (1..255, 0 为广播)
 * @param store          00=清除已存储参数, 01=存储当前速度参数
 * @param dir            旋转方向: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param speed_rpm      速度: 0..3000 (单位 RPM，BE16)
 * @param acc_level      加速度: 0..255 档位 (单字节)
 * @param en_pin_enable  00=不使能En, 01=使能En引脚控制启停
 * @param msg            输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildStoreAutoRunEmmCmd(uint8_t addr, uint8_t store,
                                  uint8_t dir, uint16_t speed_rpm,
                                  uint8_t acc_level, uint8_t en_pin_enable,
                                  zdt_can_msg_t *msg);

/* ====================================================================
 *  5.8 读取与修改所有驱动参数命令 (Emm 固件特有)
 * ==================================================================== */

/**
 * @brief 5.8.2 读取系统状态参数 (Emm)
 *
 * 原始命令格式: Addr + 43 + 7A + 6B (4 字节)
 *
 * @param addr 电机地址 (1..255, 0 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildReadAllStatusEmmCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.8.5 读取驱动配置参数 (Emm)
 *
 * 原始命令格式: Addr + 42 + 6C + 6B (4 字节)
 *
 * @param addr 电机地址 (1..255, 0 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildReadAllConfigEmmCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.8.6 修改驱动配置参数 (Emm)
 *
 * 原始命令格式: Addr + 48 + D1 + 参数列表... + 6B (33 字节)
 *
 * @param addr              电机地址 (1..255, 0 为广播)
 * @param store             是否存储: 00=不存储, 01=存储到 Flash
 * @param motor_type        电机类型: 0x19=1.8°, 0x32=0.9°
 * @param pul_mode          脉冲端口复用: 00..04
 * @param com_mode          通讯端口复用: 00..04
 * @param en_level          En引脚有效电平: 00=L, 01=H, 02=Hold
 * @param dir_level         Dir引脚有效电平: 00=CW, 01=CCW
 * @param microstep         细分设置: 0..255 (0 表示 256 细分)
 * @param interp            细分插补: 00=关闭, 01=开启
 * @param open_current_ma   开环工作电流: 0..5000 mA
 * @param stall_current_ma  闭环堵转最大电流: 0..5000 mA
 * @param max_voltage_mv    闭环最大输出电压: 0..5000 mV
 * @param uart_baud         串口波特率编号: 00..08
 * @param can_speed         CAN速率编号: 00..09
 * @param check_mode        通讯校验模式: 00..04
 * @param reply_mode        控制命令应答模式: 00..04
 * @param stall_protect     堵转保护: 00=关, 01=使能
 * @param stall_speed_rpm   堵转检测转速: 0..3000 RPM
 * @param stall_current_ma2 堵转检测电流: 0..5000 mA
 * @param stall_time_ms     堵转检测时间: 0..65535 ms
 * @param pos_window        位置到达窗口: 0..65535
 * @param msg               输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildWriteAllConfigEmmCmd(uint8_t addr, uint8_t store,
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
                                    zdt_can_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_EMM_H */
