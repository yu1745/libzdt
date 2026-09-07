/*
 * zdt_x.h - ZDT_X42S X 固件专属原生 CAN 命令构建接口
 *
 * 纯 C99，无任何硬件平台或操作系统依赖，零动态内存分配。
 * 本模块包含 15 条 X 固件特有命令的 CAN 报文构建函数。
 */

#ifndef ZDT_X_H
#define ZDT_X_H

#include "zdt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  5.3 运动控制命令 (X 固件特有)
 * ==================================================================== */

/**
 * @brief 5.3.3 力矩模式控制 (X)
 *
 * 原始命令格式: Addr + F5 + 符号 + 斜率(BE16, mA/S) + 电流(BE16, mA) + 同步 + 6B (9 字节)
 *
 * @param addr        电机地址 (1..255, 0 为广播)
 * @param dir         方向/符号: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param slope_ma_s  电流上升斜率: 0..65535 mA/s
 * @param current_ma  目标力矩电流: 0..5000 mA
 * @param sync        多机同步标志: ZDT_SYNC_NOW(0x00) 或 ZDT_SYNC_CACHE(0x01)
 * @param msg         输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildTorqueModeCmd(uint8_t addr, uint8_t dir,
                             uint16_t slope_ma_s, uint16_t current_ma,
                             uint8_t sync, zdt_can_msg_t *msg);

/**
 * @brief 5.3.4 力矩模式限速控制 (X)
 *
 * 原始命令格式: Addr + C5 + 符号 + 斜率(BE16, mA/S) + 电流(BE16, mA) + 同步 + 最大速度(BE16, ×0.1RPM) + 6B (11 字节)
 *
 * @param addr        电机地址 (1..255, 0 为广播)
 * @param dir         方向/符号: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param slope_ma_s  电流上升斜率: 0..65535 mA/s
 * @param current_ma  目标力矩电流: 0..5000 mA
 * @param max_rpm_x10 限速最大转速: 0..30000 (单位 0.1RPM，即 0..3000.0 RPM)
 * @param sync        多机同步标志: ZDT_SYNC_NOW(0x00) 或 ZDT_SYNC_CACHE(0x01)
 * @param msg         输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildTorqueModeSpeedLimitCmd(uint8_t addr, uint8_t dir,
                                       uint16_t slope_ma_s, uint16_t current_ma,
                                       uint16_t max_rpm_x10,
                                       uint8_t sync, zdt_can_msg_t *msg);

/**
 * @brief 5.3.5 速度模式控制 (X)
 *
 * 原始命令格式: Addr + F6 + 方向 + 加速度(BE16, RPM/s) + 速度(BE16, ×0.1RPM) + 同步 + 6B (9 字节)
 *
 * @param addr        电机地址 (1..255, 0 为广播)
 * @param dir         方向: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param acc_rpm_s   加速度: 0..65535 RPM/s
 * @param rpm_x10     速度: 0..30000 (单位 0.1RPM，即 0..3000.0 RPM)
 * @param sync        多机同步标志: ZDT_SYNC_NOW(0x00) 或 ZDT_SYNC_CACHE(0x01)
 * @param msg         输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildSpeedModeXCmd(uint8_t addr, uint8_t dir,
                             uint16_t acc_rpm_s, uint16_t rpm_x10,
                             uint8_t sync, zdt_can_msg_t *msg);

/**
 * @brief 5.3.6 速度模式限电流控制 (X)
 *
 * 原始命令格式: Addr + C6 + 符号 + 加速度(BE16, RPM/s) + 速度(BE16, ×0.1RPM) + 同步 + 最大电流(BE16, mA) + 6B (11 字节)
 *
 * @param addr           电机地址 (1..255, 0 为广播)
 * @param dir            方向/符号: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param acc_rpm_s      加速度: 0..65535 RPM/s
 * @param rpm_x10        速度: 0..30000 (单位 0.1RPM，即 0..3000.0 RPM)
 * @param max_current_ma 最大电流: 0..5000 mA
 * @param sync           多机同步标志: ZDT_SYNC_NOW(0x00) 或 ZDT_SYNC_CACHE(0x01)
 * @param msg            输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildSpeedModeXCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                         uint16_t acc_rpm_s, uint16_t rpm_x10,
                                         uint16_t max_current_ma,
                                         uint8_t sync, zdt_can_msg_t *msg);

/**
 * @brief 5.3.8 直通限速位置模式控制 (X)
 *
 * 原始命令格式: Addr + FB + 方向 + 速度(BE16, ×0.1RPM) + 位置角度(BE32, ×0.1°) + 运动模式 + 同步 + 6B (12 字节)
 *
 * @param addr       电机地址 (1..255, 0 为广播)
 * @param dir        方向: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param rpm_x10    速度: 0..30000 (单位 0.1RPM)
 * @param pos_angle  位置角度 (单位 0.1°，或缩小10倍后的 1°)
 * @param move_mode  相对/绝对运动模式: ZDT_MOVE_REL_LAST / ABS_ZERO / REL_NOW
 * @param sync       多机同步标志: ZDT_SYNC_NOW(0x00) 或 ZDT_SYNC_CACHE(0x01)
 * @param msg        输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildPosModePassThroughCmd(uint8_t addr, uint8_t dir,
                                     uint16_t rpm_x10, uint32_t pos_angle,
                                     uint8_t move_mode, uint8_t sync,
                                     zdt_can_msg_t *msg);

/**
 * @brief 5.3.9 直通限速位置模式限电流控制 (X)
 *
 * 原始命令格式: Addr + CB + 方向 + 速度(BE16, ×0.1RPM) + 位置角度(BE32, ×0.1°) + 运动模式 + 同步 + 最大电流(BE16, mA) + 6B (14 字节)
 *
 * @param addr           电机地址 (1..255, 0 为广播)
 * @param dir            方向: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param rpm_x10        速度: 0..30000 (单位 0.1RPM)
 * @param pos_angle      位置角度 (单位 0.1°，或缩小10倍后的 1°)
 * @param move_mode      相对/绝对运动模式: ZDT_MOVE_REL_LAST / ABS_ZERO / REL_NOW
 * @param sync           多机同步标志: ZDT_SYNC_NOW(0x00) 或 ZDT_SYNC_CACHE(0x01)
 * @param max_current_ma 最大电流: 0..5000 mA
 * @param msg            输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildPosModePassThroughCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                                 uint16_t rpm_x10, uint32_t pos_angle,
                                                 uint8_t move_mode, uint8_t sync,
                                                 uint16_t max_current_ma,
                                                 zdt_can_msg_t *msg);

/**
 * @brief 5.3.10 梯形曲线加减速位置模式控制 (X)
 *
 * 原始命令格式: Addr + FD + 方向 + 加速加速度(BE16) + 减速加速度(BE16) + 最大速度(BE16, ×0.1RPM) + 位置角度(BE32, ×0.1°) + 运动模式 + 同步 + 6B (16 字节)
 *
 * @param addr          电机地址 (1..255, 0 为广播)
 * @param dir           方向: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param accel_rpm_s   加速加速度: 0..65535 RPM/s
 * @param decel_rpm_s   减速加速度: 0..65535 RPM/s
 * @param max_speed_x10 最大速度: 0..30000 (单位 0.1RPM)
 * @param pos_angle     位置角度 (单位 0.1°，或缩小10倍后的 1°)
 * @param move_mode     相对/绝对运动模式: ZDT_MOVE_REL_LAST / ABS_ZERO / REL_NOW
 * @param sync          多机同步标志: ZDT_SYNC_NOW(0x00) 或 ZDT_SYNC_CACHE(0x01)
 * @param msg           输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildTrapezoidPosModeCmd(uint8_t addr, uint8_t dir,
                                   uint16_t accel_rpm_s, uint16_t decel_rpm_s,
                                   uint16_t max_speed_x10, uint32_t pos_angle,
                                   uint8_t move_mode, uint8_t sync,
                                   zdt_can_msg_t *msg);

/**
 * @brief 5.3.11 梯形曲线加减速位置模式限电流控制 (X)
 *
 * 原始命令格式: Addr + CD + 方向 + 加速加速度(BE16) + 减速加速度(BE16) + 最大速度(BE16, ×0.1RPM) + 位置角度(BE32, ×0.1°) + 运动模式 + 同步 + 最大电流(BE16, mA) + 6B (18 字节)
 *
 * @param addr           电机地址 (1..255, 0 为广播)
 * @param dir            方向: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param accel_rpm_s    加速加速度: 0..65535 RPM/s
 * @param decel_rpm_s    减速加速度: 0..65535 RPM/s
 * @param max_speed      最大速度: 0..30000 (单位 0.1RPM)
 * @param pos_angle      位置角度 (单位 0.1°，或缩小10倍后的 1°)
 * @param move_mode      相对/绝对运动模式: ZDT_MOVE_REL_LAST / ABS_ZERO / REL_NOW
 * @param sync           多机同步标志: ZDT_SYNC_NOW(0x00) 或 ZDT_SYNC_CACHE(0x01)
 * @param max_current_ma 最大电流: 0..5000 mA
 * @param msg            输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildTrapezoidPosModeCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                              uint16_t accel_rpm_s,
                                              uint16_t decel_rpm_s,
                                              uint16_t max_speed,
                                              uint32_t pos_angle,
                                              uint8_t move_mode,
                                              uint8_t sync,
                                              uint16_t max_current_ma,
                                              zdt_can_msg_t *msg);

/* ====================================================================
 *  5.6 修改驱动参数命令 (X 固件特有)
 * ==================================================================== */

/**
 * @brief 5.6.10 修改命令位置角度是否继续缩小 10 倍输入 (X)
 *
 * 原始命令格式: Addr + 4F + 71 + store + scale(00/01) + 6B (6 字节)
 *
 * @param addr       电机地址 (1..255, 0 为广播)
 * @param store      是否存储: ZDT_STORE_NO(0x00) 或 ZDT_STORE_YES(0x01)
 * @param scale_10x  00=不缩小(单位0.1°), 01=缩小10倍(单位1°)
 * @param msg        输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildChangePosScaleCmd(uint8_t addr, uint8_t store,
                                 uint8_t scale_10x,
                                 zdt_can_msg_t *msg);

/**
 * @brief 5.6.14 读取 PID 参数 (X)
 *
 * 原始命令格式: Addr + 21 + 6B (3 字节)
 *
 * @param addr 电机地址 (1..255, 0 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildReadPidXCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.6.15 修改 PID 参数 (X)
 *
 * 原始命令格式: Addr + 4A + C3 + store + pTkp(BE32) + pBkp(BE32) + vkp(BE32) + vki(BE32) + 6B (21 字节)
 *
 * @param addr   电机地址 (1..255, 0 为广播)
 * @param store  是否存储: ZDT_STORE_NO(0x00) 或 ZDT_STORE_YES(0x01)
 * @param pTkp   位置环转矩刚度系数
 * @param pBkp   位置环反电动势刚度系数
 * @param vkp    速度环比例系数
 * @param vki    速度环积分系数
 * @param msg    输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildWritePidXCmd(uint8_t addr, uint8_t store,
                            uint32_t pTkp, uint32_t pBkp,
                            uint32_t vkp, uint32_t vki,
                            zdt_can_msg_t *msg);

/* ====================================================================
 *  5.7 上电自动运行命令 (X 固件特有)
 * ==================================================================== */

/**
 * @brief 5.7.1 存储一组速度参数，上电自动运行 (X)
 *
 * 原始命令格式: Addr + F7 + 1C + store + dir + acc(BE16) + speed(BE16) + en + 6B (10 字节)
 *
 * @param addr           电机地址 (1..255, 0 为广播)
 * @param store          00=清除已存储参数, 01=存储当前速度参数
 * @param dir            旋转方向: ZDT_DIR_CW(0x00) 或 ZDT_DIR_CCW(0x01)
 * @param acc_rpm_per_s  加速度: 0..65535 RPM/s
 * @param speed_rpm_x10  速度: 0..30000 (单位 0.1RPM，即 0..3000.0 RPM)
 * @param en_pin_enable  00=不使能En, 01=使能En引脚控制启停
 * @param msg            输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildStoreAutoRunXCmd(uint8_t addr, uint8_t store,
                                uint8_t dir, uint16_t acc_rpm_per_s,
                                uint16_t speed_rpm_x10, uint8_t en_pin_enable,
                                zdt_can_msg_t *msg);

/* ====================================================================
 *  5.8 读取与修改所有驱动参数命令 (X 固件特有)
 * ==================================================================== */

/**
 * @brief 5.8.1 读取系统状态参数 (X)
 *
 * 原始命令格式: Addr + 43 + 7A + 6B (4 字节)
 *
 * @param addr 电机地址 (1..255, 0 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildReadAllStatusXCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.8.3 读取驱动配置参数 (X)
 *
 * 原始命令格式: Addr + 42 + 6C + 6B (4 字节)
 *
 * @param addr 电机地址 (1..255, 0 为广播)
 * @param msg  输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
int zdtCanBuildReadAllConfigXCmd(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 5.8.4 修改驱动配置参数 (X)
 *
 * 原始命令格式: Addr + 48 + D1 + 参数列表... + 6B (37 字节)
 *
 * @param addr             电机地址 (1..255, 0 为广播)
 * @param store            是否存储: 00=不存储, 01=存储到 Flash
 * @param key_lock         按键锁定: 00=不锁定, 01=锁定
 * @param ctrl_mode        控制模式: 00=开环, 01=FOC闭环
 * @param pul_mode         脉冲端口复用: 00..04
 * @param com_mode         通讯端口复用: 00..04
 * @param en_level         En引脚有效电平: 00=L, 01=H, 02=Hold
 * @param dir_level        Dir引脚有效电平: 00=CW, 01=CCW
 * @param microstep        细分设置: 0..255 (0 表示 256 细分)
 * @param interp           细分插补: 00=关闭, 01=开启
 * @param open_current_ma  开环工作电流: 0..5000 mA
 * @param close_current_ma 闭环最大电流: 0..5000 mA
 * @param close_max_rpm    闭环最大转速: 0..3000 RPM
 * @param current_bw_hz    电流环带宽: 0..65535 Hz
 * @param uart_baud        串口波特率编号: 00..08
 * @param can_speed        CAN速率编号: 00..09
 * @param check_mode       通讯校验模式: 00..04
 * @param reply_mode       控制命令应答模式: 00..04
 * @param angle_scale_10   位置角度是否缩小10倍输入: 00=否, 01=是
 * @param stall_protect    堵转保护: 00=关, 01=使能, 02=复位不松轴
 * @param stall_speed_rpm  堵转检测转速: 0..3000 RPM
 * @param stall_current_ma 堵转检测电流: 0..5000 mA
 * @param stall_time_ms    堵转检测时间: 0..65535 ms
 * @param pos_window_x10   位置到达窗口: 单位 0.1° (默认 8 对应 0.8°)
 * @param msg              输出 CAN 报文集合指针
 * @return 成功返回写入的 CAN 帧数 (>=1)，失败返回负数错误码 (ZDT_ERR_PARAM)
 */
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
                                  zdt_can_msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_X_H */
