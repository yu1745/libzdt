/*
 * libzdt.h - ZDT_X42S 第二代闭环步进电机通信命令构建库
 *
 * 只负责构建"主机发送（主机→电机）"命令字节帧。
 * 不负责发送、接收、解析、校验码计算（默认校验码 0x6B 写死在命令末尾）。
 *
 * 命令帧通用格式：
 *   Addr(1B) | FuncCode(1B) | [AuxCode/Params...] | CheckSum(1B, 0x6B)
 *
 * API 风格：每个命令对应一个 buildXxxCmd 函数。
 *   返回值: >=0 表示写入 buf 的字节数；<0 表示参数错误
 *   buf 由调用者提供，长度必须 >= 该命令的最大长度（详见各函数注释）。
 *   所有多字节数值采用小端序（Little-Endian，与手册示例一致）。
 */

#ifndef LIBZDT_H
#define LIBZDT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 默认校验码（按手册第4章说明） */
#define ZDT_CHECKSUM_DEFAULT   0x6B
/* 广播地址 */
#define ZDT_ADDR_BROADCAST     0x00
/* 同步标志: 00=立即执行, 01=缓存当前命令 */
#define ZDT_SYNC_NOW           0x00
#define ZDT_SYNC_CACHE         0x01
/* 是否存储: 00=掉电丢失, 01=保存到 Flash */
#define ZDT_STORE_NO           0x00
#define ZDT_STORE_YES          0x01
/* 方向: 00=CW(顺时针), 01=CCW(逆时针) */
#define ZDT_DIR_CW             0x00
#define ZDT_DIR_CCW            0x01
/* 运动模式: 00=相对上一目标位置, 01=相对坐标零点, 02=相对当前实时位置 */
#define ZDT_MOVE_REL_LAST      0x00
#define ZDT_MOVE_ABS_ZERO      0x01
#define ZDT_MOVE_REL_NOW       0x02

/* 通用错误码 */
#define ZDT_ERR_BUF_TOO_SMALL  (-1)
#define ZDT_ERR_PARAM          (-2)

/* ====================================================================
 *  5.2 触发动作命令  (共 5 个)
 * ==================================================================== */

/* 5.2.1 触发编码器校准  (Addr + 06 + 45 + 6B) — 4B */
int zdtBuildEncoderCalibrationCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.2.2 重启电机 (X42S/Y42)  (Addr + 08 + 97 + 6B) — 4B */
int zdtBuildRestartMotorCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.2.3 将当前位置角度清零  (Addr + 0A + 6D + 6B) — 4B */
int zdtBuildClearCurrentAngleCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.2.4 解除堵转/过热/过流保护  (Addr + 0E + 52 + 6B) — 4B */
int zdtBuildClearProtectionCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.2.5 恢复出厂设置  (Addr + 0F + 5F + 6B) — 4B */
int zdtBuildFactoryResetCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* ====================================================================
 *  5.3 运动控制命令  (共 14 个)
 * ==================================================================== */

/* 5.3.1 多电机命令 (X42S/Y42)
 * 命令格式: 00 + AA + 总字节数(BE16) + [子命令1] [子命令2] ... + 6B
 * 长度字段为整帧总字节数 (5 + 子命令总长)，见手册示例 0022=34
 * sub_cmds: 多个子命令的拼接，每个子命令必须自带 6B 校验码结尾
 * 返回写入字节数; buf_size 必须 >= 5 + 子命令总长 */
int zdtBuildMultiMotorCmd(const uint8_t *sub_cmds, size_t sub_cmds_len,
                          uint8_t *buf, size_t buf_size);

/* 5.3.2 电机使能控制  (Addr + F3 + AB + 使能标志 + 同步标志 + 6B) — 6B
 * enable: 00=掉电关闭(释放), 01=使能 */
int zdtBuildMotorEnableCmd(uint8_t addr, uint8_t enable, uint8_t sync,
                           uint8_t *buf, size_t buf_size);

/* 5.3.3 力矩模式控制 (X)  (Addr + F5 + 符号 + 斜率(BE16, mA/S) + 电流(BE16, mA) + 同步 + 6B) — 9B
 * slope_ma_s:    0..65535 mA/S
 * current_ma:    0..5000 mA */
int zdtBuildTorqueModeCmd(uint8_t addr, uint8_t dir,
                          uint16_t slope_ma_s, uint16_t current_ma,
                          uint8_t sync, uint8_t *buf, size_t buf_size);

/* 5.3.4 力矩模式限速控制 (X)  (Addr + C5 + 符号 + 斜率(BE16, mA/S) + 电流(BE16, mA) + 同步 + 最大速度(BE16, ×0.1RPM) + 6B) — 11B
 * slope_ma_s: 0..65535
 * current_ma: 0..5000
 * max_rpm_x10: 0..30000 */
int zdtBuildTorqueModeSpeedLimitCmd(uint8_t addr, uint8_t dir,
                                    uint16_t slope_ma_s, uint16_t current_ma,
                                    uint16_t max_rpm_x10,
                                    uint8_t sync, uint8_t *buf, size_t buf_size);

/* 5.3.5 速度模式控制 (X)  (Addr + F6 + 方向 + 加速度(BE16, RPM/S) + 速度(BE16, ×0.1RPM) + 同步 + 6B) — 9B
 * acc_rpm_s: 0..65535 RPM/S
 * rpm_x10:   0..30000 (×0.1RPM) */
int zdtBuildSpeedModeXCmd(uint8_t addr, uint8_t dir,
                          uint16_t acc_rpm_s, uint16_t rpm_x10,
                          uint8_t sync, uint8_t *buf, size_t buf_size);

/* 5.3.6 速度模式限电流控制 (X)  (Addr + C6 + 符号 + 加速度(BE16, RPM/S) + 速度(BE16, ×0.1RPM) + 同步 + 最大电流(BE16, mA) + 6B) — 11B
 * acc_rpm_s:    0..65535
 * rpm_x10:      0..30000
 * max_current_ma: 0..5000 */
int zdtBuildSpeedModeXCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                      uint16_t acc_rpm_s, uint16_t rpm_x10,
                                      uint16_t max_current_ma,
                                      uint8_t sync, uint8_t *buf, size_t buf_size);

/* 5.3.7 速度模式控制 (Emm)  (Addr + F6 + 方向 + 速度(LE16) + 加速度 + 同步标志 + 6B) — 7B (与 5.3.5 一致) */
int zdtBuildSpeedModeEmmCmd(uint8_t addr, uint8_t dir, uint16_t rpm,
                             uint8_t acc, uint8_t sync,
                             uint8_t *buf, size_t buf_size);

/* 5.3.8 直通限速位置模式控制 (X)  (Addr + FB + 方向 + 速度(BE16, ×0.1RPM) + 位置角度(BE32, ×0.1°) + 运动模式 + 同步 + 6B) — 12B
 * move_mode: ZDT_MOVE_REL_LAST/ABS_ZERO/REL_NOW */
int zdtBuildPosModePassThroughCmd(uint8_t addr, uint8_t dir,
                                  uint16_t rpm_x10, uint32_t pos_angle,
                                  uint8_t move_mode, uint8_t sync,
                                  uint8_t *buf, size_t buf_size);

/* 5.3.9 直通限速位置模式限电流控制 (X)  (Addr + CB + 方向 + 速度(BE16, ×0.1RPM) + 位置角度(BE32, ×0.1°) + 运动模式 + 同步 + 最大电流(BE16, mA) + 6B) — 14B
 * move_mode: ZDT_MOVE_REL_LAST/ABS_ZERO/REL_NOW */
int zdtBuildPosModePassThroughCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                              uint16_t rpm_x10, uint32_t pos_angle,
                                              uint8_t move_mode, uint8_t sync,
                                              uint16_t max_current_ma,
                                              uint8_t *buf, size_t buf_size);

/* 5.3.10 梯形曲线加减速位置模式控制 (X)  (Addr + FD + 方向 + 加速加速度(BE16, RPM/S) + 减速加速度(BE16, RPM/S) + 最大速度(BE16, ×0.1RPM) + 位置角度(BE32, ×0.1°) + 运动模式 + 同步 + 6B) — 16B
 * accel_rpm_s:   0..65535 RPM/S
 * decel_rpm_s:   0..65535 RPM/S
 * max_speed_x10: 0..30000
 * pos_angle:     0..0xFFFFFFFF (×0.1°)
 * move_mode:    ZDT_MOVE_REL_LAST/ABS_ZERO/REL_NOW */
int zdtBuildTrapezoidPosModeCmd(uint8_t addr, uint8_t dir,
                                uint16_t accel_rpm_s, uint16_t decel_rpm_s,
                                uint16_t max_speed_x10, uint32_t pos_angle,
                                uint8_t move_mode, uint8_t sync,
                                uint8_t *buf, size_t buf_size);

/* 5.3.11 梯形曲线加减速位置模式限电流控制 (X)  (Addr + CD + 方向 + 加速加速度(LE16) + 减速加速度(LE16) + 最大速度(LE16) + 位置角度(LE32) + 运动模式 + 同步标志 + 最大电流(LE16) + 6B) — 18B
 * 注意：5.3.11 在 X 固件下功能码为 CD；
 *        accel/decel 0..65535 RPM/s；
 *        max_speed 0..30000（即 0..3000.0 RPM）；
 *        pos_angle 0..0xFFFFFFFF（×0.1°）；
 *        max_current_ma 0..5000 mA。 */
int zdtBuildTrapezoidPosModeCurrentLimitCmd(uint8_t addr, uint8_t dir,
                                           uint16_t accel_rpm_s,
                                           uint16_t decel_rpm_s,
                                           uint16_t max_speed,
                                           uint32_t pos_angle,
                                           uint8_t move_mode,
                                           uint8_t sync,
                                           uint16_t max_current_ma,
                                           uint8_t *buf, size_t buf_size);

/* 5.3.12 位置模式控制 (Emm)  (Addr + FD + 方向 + 速度(LE16) + 加速度 + 脉冲数(LE32) + 运动模式 + 同步标志 + 6B) — 13B */
int zdtBuildPosModeEmmCmd(uint8_t addr, uint8_t dir, uint16_t rpm,
                          uint8_t acc, uint32_t pulses,
                          uint8_t move_mode, uint8_t sync,
                          uint8_t *buf, size_t buf_size);

/* 5.3.13 立即停止  (Addr + FE + 98 + 同步标志 + 6B) — 5B */
int zdtBuildImmediateStopCmd(uint8_t addr, uint8_t sync,
                             uint8_t *buf, size_t buf_size);

/* 5.3.14 触发多机同步运动  (Addr + FF + 66 + 6B) — 4B
 * addr 通常传 ZDT_ADDR_BROADCAST */
int zdtBuildSyncMotionCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* ====================================================================
 *  5.4 原点回零命令  (共 6 个)
 * ==================================================================== */

/* 5.4.1 设置单圈回零的零点位置  (Addr + 93 + 88 + 是否存储 + 6B) — 5B
 * store: 00=不存储 01=存储(掉电不丢失) */
int zdtBuildSetSingleTurnZeroCmd(uint8_t addr, uint8_t store,
                                 uint8_t *buf, size_t buf_size);

/* 5.4.2 触发回零  (Addr + 9A + 模式 + 同步标志 + 6B) — 5B
 * homing_mode: 00..05 */
int zdtBuildTriggerHomingCmd(uint8_t addr, uint8_t homing_mode,
                             uint8_t sync, uint8_t *buf, size_t buf_size);

/* 5.4.3 强制中断并退出回零  (Addr + 9C + 48 + 6B) — 4B */
int zdtBuildAbortHomingCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.4.4 读取回零状态标志  (Addr + 3B + 6B) — 3B (功能码 3B) */
int zdtBuildReadHomingStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.4.5 读取回零参数  (Addr + 22 + 6B) — 4B */
int zdtBuildReadHomingParamsCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.4.6 修改回零参数  (Addr + 4C + AE + 是否存储 + 模式 + 方向 + 速度(LE16) + 超时(LE32) + 碰撞转速(LE16) + 碰撞电流(LE16) + 碰撞时间(LE16) + O_POT_En + 6B) — 20B
 * store: ZDT_STORE_NO/YES
 * auto_trigger: 00/01 */
int zdtBuildWriteHomingParamsCmd(uint8_t addr, uint8_t store,
                                 uint8_t homing_mode, uint8_t dir,
                                 uint16_t rpm, uint32_t timeout_ms,
                                 uint16_t bump_speed_rpm,
                                 uint16_t bump_current_ma,
                                 uint16_t bump_time_ms,
                                 uint8_t auto_trigger,
                                 uint8_t *buf, size_t buf_size);

/* ====================================================================
 *  5.5 读取系统参数命令  (共 18 个)
 * ==================================================================== */

/* 5.5.1 定时返回信息命令 (X42S/Y42)
 * Addr + 11 + 18 + 信息功能码 + 定时时间(BE16, ms) + 6B — 7B
 * info_func_code: 36/3A/... (5.5 节中任意读取命令的功能码)
 * interval_ms: 0..0xFFFF ms */
int zdtBuildSetPeriodicReportCmd(uint8_t addr, uint8_t info_func_code,
                                 uint16_t interval_ms,
                                 uint8_t *buf, size_t buf_size);

/* 5.5.2 读取固件版本和硬件版本  (Addr + 1F + 6B) — 4B */
int zdtBuildReadVersionCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.3 读取相电阻和相电感  (Addr + 20 + 6B) — 4B */
int zdtBuildReadPhaseRLCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.4 读取总线电压  (Addr + 24 + 6B) — 4B */
int zdtBuildReadBusVoltageCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.5 读取总线电流 (X42S/Y42)  (Addr + 26 + 6B) — 4B */
int zdtBuildReadBusCurrentCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.6 读取相电流  (Addr + 27 + 6B) — 4B */
int zdtBuildReadPhaseCurrentCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.7 读取经过线性化校准后的编码器值  (Addr + 29 + 6B) — 4B */
int zdtBuildReadEncoderCalibratedCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.8 读取输入脉冲数  (Addr + 32 + 6B) — 4B */
int zdtBuildReadInputPulsesCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.9 读取电机目标位置  (Addr + 33 + 6B) — 4B */
int zdtBuildReadTargetPosCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.10 读取电机实时设定的目标位置  (Addr + 35 + 6B) — 4B */
int zdtBuildReadRealtimeTargetPosCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.11 读取电机实时转速  (Addr + 36 + 6B) — 4B */
int zdtBuildReadRealtimeSpeedCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.12 读取驱动温度 (X42S/Y42)  (Addr + 39 + 6B) — 4B */
int zdtBuildReadDriverTempCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.13 读取电机实时位置  (Addr + 40 + 6B) — 4B */
int zdtBuildReadRealtimePosCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.14 读取电机位置误差  (Addr + 41 + 6B) — 4B */
int zdtBuildReadPosErrorCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.15 读取电机状态标志  (Addr + 42 + 6B) — 4B */
int zdtBuildReadMotorStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.16 读取回零状态标志 + 电机状态标志 (X42S/Y42)  (Addr + 43 + 6B) — 4B */
int zdtBuildReadHomingAndStatusCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.17 读取引脚 IO 电平状态 (X42S/Y42)  (Addr + 46 + 6B) — 4B */
int zdtBuildReadIoLevelCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.5.18 读取电池电压 (Y42)  (Addr + 47 + 6B) — 4B */
int zdtBuildReadBatteryVoltageCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* ====================================================================
 *  5.6 读写驱动参数命令  (共 31 个)
 * ==================================================================== */

/* 5.6.1 修改电机 ID/地址  Addr + AE + 4B + 是否存储 + ID(01-FF) + 6B — 6B */
int zdtBuildChangeAddrCmd(uint8_t addr, uint8_t store,
                          uint8_t new_addr,
                          uint8_t *buf, size_t buf_size);

/* 5.6.2 修改细分值  Addr + 84 + 8A + 是否存储 + 细分值 + 6B — 6B
 * microstep: 0x01-0xFF 表示 1-255 细分, 0x00 表示 256 细分 */
int zdtBuildChangeMicrostepCmd(uint8_t addr, uint8_t store,
                               uint8_t microstep,
                               uint8_t *buf, size_t buf_size);

/* 5.6.3 修改掉电标志  Addr + 50 + 标志(00/01) + 6B — 4B */
int zdtBuildChangePowerDownFlagCmd(uint8_t addr, uint8_t flag,
                                   uint8_t *buf, size_t buf_size);

/* 5.6.4 读取选项参数状态 (X42S/Y42)  Addr + 1A + 6B — 3B */
int zdtBuildReadOptionsCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.6.5 修改电机类型  Addr + D7 + 35 + 是否存储 + 类型(19/32) + 6B — 6B
 * motor_type: 0x19=0.9°, 0x32=1.8° */
int zdtBuildChangeMotorTypeCmd(uint8_t addr, uint8_t store,
                               uint8_t motor_type,
                               uint8_t *buf, size_t buf_size);

/* 5.6.6 修改固件类型  Addr + D5 + 69 + 是否存储 + 类型(00/01/02) + 6B — 6B
 * fw_type: 0=X, 1=Emm, 2=Emm 狂暴 */
int zdtBuildChangeFirmwareTypeCmd(uint8_t addr, uint8_t store,
                                  uint8_t fw_type,
                                  uint8_t *buf, size_t buf_size);

/* 5.6.7 修改开环/闭环控制模式  Addr + 46 + A6 + 是否存储 + 模式(00/01) + 6B — 6B */
int zdtBuildChangeCtrlModeCmd(uint8_t addr, uint8_t store,
                              uint8_t mode,
                              uint8_t *buf, size_t buf_size);

/* 5.6.8 修改电机运动正方向  Addr + D4 + 60 + 是否存储 + 方向(00/01) + 6B — 6B */
int zdtBuildChangeMotorDirCmd(uint8_t addr, uint8_t store,
                              uint8_t dir,
                              uint8_t *buf, size_t buf_size);

/* 5.6.9 修改锁定按键功能  Addr + D0 + B3 + 是否存储 + 锁定(00/01) + 6B — 6B */
int zdtBuildChangeKeyLockCmd(uint8_t addr, uint8_t store,
                             uint8_t lock,
                             uint8_t *buf, size_t buf_size);

/* 5.6.10 修改命令位置角度是否继续缩小 10 倍输入 (X)
 * Addr + 4F + 71 + 是否存储 + 是否缩小(00/01) + 6B — 6B */
int zdtBuildChangePosScaleCmd(uint8_t addr, uint8_t store,
                              uint8_t scale_10x,
                              uint8_t *buf, size_t buf_size);

/* 5.6.11 修改命令速度值是否缩小 10 倍输入 (Emm)
 * Addr + 4F + 71 + 是否存储 + 是否缩小(00/01) + 6B — 6B (与 5.6.10 帧结构相同) */
int zdtBuildChangeSpeedScaleCmd(uint8_t addr, uint8_t store,
                                uint8_t scale_10x,
                                uint8_t *buf, size_t buf_size);

/* 5.6.12 修改开环模式工作电流  Addr + 44 + 33 + 是否存储 + 电流(BE16, 0-5000mA) + 6B — 7B */
int zdtBuildChangeOpenLoopCurrentCmd(uint8_t addr, uint8_t store,
                                     uint16_t current_ma,
                                     uint8_t *buf, size_t buf_size);

/* 5.6.13 修改闭环模式最大电流  Addr + 45 + 66 + 是否存储 + 电流(BE16, 0-5000mA) + 6B — 7B */
int zdtBuildChangeClosedLoopCurrentCmd(uint8_t addr, uint8_t store,
                                       uint16_t current_ma,
                                       uint8_t *buf, size_t buf_size);

/* 5.6.14 读取 PID 参数 (X)  Addr + 21 + 6B — 3B */
int zdtBuildReadPidXCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.6.15 修改 PID 参数 (X)
 * Addr + 4A + C3 + 是否存储 + pTkp(BE32) + pBkp(BE32) + vkp(BE32) + vki(BE32) + 6B — 21B */
int zdtBuildWritePidXCmd(uint8_t addr, uint8_t store,
                         uint32_t pTkp, uint32_t pBkp,
                         uint32_t vkp, uint32_t vki,
                         uint8_t *buf, size_t buf_size);

/* 5.6.16 读取 PID 参数 (Emm)  Addr + 21 + 6B — 3B */
int zdtBuildReadPidEmmCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.6.17 修改 PID 参数 (Emm)
 * Addr + 4A + C3 + 是否存储 + Kp(BE32) + Ki(BE32) + Kd(BE32) + 6B — 17B */
int zdtBuildWritePidEmmCmd(uint8_t addr, uint8_t store,
                           uint32_t kp, uint32_t ki, uint32_t kd,
                           uint8_t *buf, size_t buf_size);

/* 5.6.18 读取 DMX512 协议参数 (X42S/Y42)  Addr + 49 + 78 + 6B — 4B */
int zdtBuildReadDmx512Cmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.6.19 修改 DMX512 协议参数 (X42S/Y42)
 * Addr + D9 + 90 + 是否存储 + 总通道数(BE16) + 每电机通道数 + 运动模式 +
 *   单通道速度(BE16) + 加速度(BE16) + 速度步长(BE16) + 运动步长(BE32) + 6B — 19B */
int zdtBuildWriteDmx512Cmd(uint8_t addr, uint8_t store,
                           uint16_t total_channels,
                           uint8_t channels_per_motor,
                           uint8_t move_mode,
                           uint16_t single_speed_rpm,
                           uint16_t accel,
                           uint16_t speed_step,
                           uint32_t motion_step,
                           uint8_t *buf, size_t buf_size);

/* 5.6.20 读取位置到达窗口 (X42S/Y42)  Addr + 41 + 6B — 3B */
int zdtBuildReadPosWindowCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.6.21 修改位置到达窗口 (X42S/Y42)
 * Addr + D1 + 07 + 是否存储 + 窗口值(单字节, ×0.1°, 范围 02~30) + 6B — 6B */
int zdtBuildWritePosWindowCmd(uint8_t addr, uint8_t store,
                              uint16_t window_x10,
                              uint8_t *buf, size_t buf_size);

/* 5.6.22 读取过热过流保护检测阈值 (X42S/Y42)  Addr + 13 + 6B — 3B */
int zdtBuildReadProtectThresholdCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.6.23 修改过热过流保护检测阈值 (X42S/Y42)
 * Addr + D3 + 56 + 是否存储 + 过热阈值(BE16, ℃) + 过流阈值(BE16, mA) + 检测时间(BE16, ms) + 6B — 11B */
int zdtBuildWriteProtectThresholdCmd(uint8_t addr, uint8_t store,
                                     uint16_t overheat_c,
                                     uint16_t overcurrent_ma,
                                     uint16_t detect_ms,
                                     uint8_t *buf, size_t buf_size);

/* 5.6.24 读取心跳保护功能时间 (X42S/Y42)  Addr + 16 + 6B — 3B */
int zdtBuildReadHeartbeatCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.6.25 修改心跳保护功能时间 (X42S/Y42)
 * Addr + 68 + 38 + 是否存储 + 心跳时间(BE32, ms) + 6B — 9B */
int zdtBuildWriteHeartbeatCmd(uint8_t addr, uint8_t store,
                              uint32_t heartbeat_ms,
                              uint8_t *buf, size_t buf_size);

/* 5.6.26 读取积分限幅/刚性系数 (X42S/Y42)  Addr + 23 + 6B — 3B */
int zdtBuildReadIntegralLimitCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.6.27 修改积分限幅/刚性系数 (X42S/Y42)
 * Addr + 4B + 57 + 是否存储 + 系数(BE32) + 6B — 9B */
int zdtBuildWriteIntegralLimitCmd(uint8_t addr, uint8_t store,
                                  uint32_t value,
                                  uint8_t *buf, size_t buf_size);

/* 5.6.28 读取碰撞回零返回角度 (X42S/Y42)  Addr + 3F + 6B — 3B */
int zdtBuildReadBumpReturnAngleCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.6.29 修改碰撞回零返回角度 (X42S/Y42)
 * Addr + 5C + AC + 是否存储 + 角度值(BE16, ×0.1°) + 6B — 6B */
int zdtBuildWriteBumpReturnAngleCmd(uint8_t addr, uint8_t store,
                                    uint16_t angle_x10,
                                    uint8_t *buf, size_t buf_size);

/* 5.6.30 广播读取 ID 地址 (X42S/Y42)  00 + 15 + 6B — 3B */
int zdtBuildBroadcastReadAddrCmd(uint8_t *buf, size_t buf_size);

/* 5.6.31 修改锁定修改参数功能 (X42S/Y42)
 * Addr + D6 + 4B + 是否存储 + 锁定等级(00-03) + 6B — 6B */
int zdtBuildChangeParamLockCmd(uint8_t addr, uint8_t store,
                               uint8_t lock_level,
                               uint8_t *buf, size_t buf_size);

/* ====================================================================
 *  5.7 上电自动运行命令  (共 2 个)
 * ==================================================================== */

/* 5.7.1 存储一组速度参数，上电自动运行 (X)
 * Addr + F7 + 1C + store + dir + acc(BE16) + speed(BE16) + en + 6B — 10B
 * store:  00=清除已存储  01=存储当前
 * dir:    00=CW 01=CCW
 * acc_rpm_per_s: 0..65535 RPM/s (BE16)
 * speed_rpm_x10: 0..30000 内部×0.1, 即 0..3000.0RPM (BE16, 上限 0x7530)
 * en_pin_enable: 00=不使能En 01=使能En引脚控制启停 */
int zdtBuildStoreAutoRunXCmd(uint8_t addr, uint8_t store,
                             uint8_t dir, uint16_t acc_rpm_per_s,
                             uint16_t speed_rpm_x10, uint8_t en_pin_enable,
                             uint8_t *buf, size_t buf_size);

/* 5.7.2 存储一组速度参数，上电自动运行 (Emm)
 * Addr + F7 + 1C + store + dir + speed(BE16) + acc(档位0..255) + en + 6B — 10B
 * speed_rpm: 0..3000 (BE16, 上限 0x0BB8)
 * acc_level: 0..255 档位, 加速度公式见手册 5.3.7 */
int zdtBuildStoreAutoRunEmmCmd(uint8_t addr, uint8_t store,
                               uint8_t dir, uint16_t speed_rpm,
                               uint8_t acc_level, uint8_t en_pin_enable,
                               uint8_t *buf, size_t buf_size);

/* ====================================================================
 *  5.8 读取所有驱动参数命令  (共 6 个)
 * ==================================================================== */

/* 5.8.1 读取系统状态参数 (X)  Addr + 43 + 7A + 6B — 4B (短帧) */
int zdtBuildReadAllStatusXCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.8.2 读取系统状态参数 (Emm) Addr + 43 + 7A + 6B — 4B (短帧) */
int zdtBuildReadAllStatusEmmCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.8.3 读取驱动配置参数 (X)  Addr + 42 + 6C + 6B — 4B (短帧) */
int zdtBuildReadAllConfigXCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.8.4 修改驱动配置参数 (X) — 37B
 * store:            是否保存 00=不保存 01=保存
 * key_lock:         锁定按键 00/01
 * ctrl_mode:        控制模式 00=开环 01=FOC闭环
 * pul_mode:         脉冲端口复用 00..04 (OFF/OPEN/FOC/ESI_RCO/pLR_ESI)
 * com_mode:         通讯端口复用 00..04 (OFF/ESI_ALO/UART/CAN/uLR_ESI)
 * en_level:         En引脚有效电平 00=L 01=H 02=Hold
 * dir_level:        Dir引脚有效电平 00=CW 01=CCW
 * microstep:        细分 0..255 (0=256细分)
 * interp:           细分插补 00/01
 * open_current_ma:  开环模式工作电流 0..5000mA (BE16)
 * close_current_ma: 闭环模式最大电流 0..5000mA (BE16)
 * close_max_rpm:    闭环模式最大速度 0..3000RPM (BE16)
 * current_bw_hz:    电流环带宽 0..65535Hz (BE16, 建议保持默认)
 * uart_baud:        串口波特率 00..08
 * can_speed:        CAN通讯速率 00..09
 * check_mode:       通讯校验 00..04
 * reply_mode:       控制命令应答 00..04
 * angle_scale_10:   角度缩小10倍输入 00/01
 * stall_protect:    堵转保护 00=关 01=使能 02=堵转复位不松轴
 * stall_speed_rpm:  堵转保护检测转速 0..3000RPM (BE16)
 * stall_current_ma: 堵转保护检测电流 0..5000mA (BE16)
 * stall_time_ms:    堵转保护检测时间 0..65535ms (BE16)
 * pos_window_x10:   位置到达窗口(内部×10, 0..65535, 默认8=0.8°) (BE16) */
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
                               uint8_t *buf, size_t buf_size);

/* 5.8.5 读取驱动配置参数 (Emm) Addr + 42 + 6C + 6B — 4B (短帧) */
int zdtBuildReadAllConfigEmmCmd(uint8_t addr, uint8_t *buf, size_t buf_size);

/* 5.8.6 修改驱动配置参数 (Emm) — 33B
 * store:            是否保存 00/01
 * motor_type:       电机类型 19=1.8° 32=0.9°
 * pul_mode:         脉冲端口复用 00..04
 * com_mode:         通讯端口复用 00..04
 * en_level:         En引脚有效电平 00=L 01=H 02=Hold
 * dir_level:        Dir引脚有效电平 00=CW 01=CCW
 * microstep:        细分 0..255 (0=256细分)
 * interp:           细分插补 00/01
 * open_current_ma:  开环模式工作电流 0..5000mA (BE16)
 * stall_current_ma: 闭环模式堵转最大电流 0..5000mA (BE16)
 * max_voltage_mv:   闭环模式最大输出电压 0..5000mV (BE16)
 * uart_baud:        串口波特率 00..08
 * can_speed:        CAN通讯速率 00..09
 * check_mode:       通讯校验 00..04
 * reply_mode:       控制命令应答 00..04
 * stall_protect:    堵转保护 00/01
 * stall_speed_rpm:  堵转保护检测转速 0..3000RPM (BE16)
 * stall_current_ma2:堵转保护检测电流 0..5000mA (BE16)
 * stall_time_ms:    堵转保护检测时间 0..65535ms (BE16)
 * pos_window:       位置到达窗口 0..65535 (BE16) */
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
                                 uint8_t *buf, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* LIBZDT_H */