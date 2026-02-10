/**
 * @file zdt_can.c
 * @brief ZDT X42S 电机驱动库 - CAN 协议实现 (EMM固件)
 */

#include "esp_err.h"
#include "libzdt.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * 内部辅助函数
 * ========================================================================== */

/**
 * @brief 将16位整数写入大端序缓冲区
 */
static inline void write_u16_be(uint8_t *buf, uint16_t val) {
  buf[0] = (val >> 8) & 0xFF;
  buf[1] = val & 0xFF;
}

/**
 * @brief 将32位整数写入大端序缓冲区
 */
static inline void write_u32_be(uint8_t *buf, uint32_t val) {
  buf[0] = (val >> 24) & 0xFF;
  buf[1] = (val >> 16) & 0xFF;
  buf[2] = (val >> 8) & 0xFF;
  buf[3] = val & 0xFF;
}

/**
 * @brief 将有符号32位整数写入大端序缓冲区
 */
static inline void write_i32_be(uint8_t *buf, int32_t val) {
  write_u32_be(buf, (uint32_t)val);
}

/**
 * @brief 从大端序缓冲区读取16位整数
 */
static inline uint16_t read_u16_be(const uint8_t *buf) {
  return ((uint16_t)buf[0] << 8) | buf[1];
}

/**
 * @brief 从大端序缓冲区读取32位整数
 */
static inline uint32_t read_u32_be(const uint8_t *buf) {
  return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
         ((uint32_t)buf[2] << 8) | buf[3];
}

/**
 * @brief 从大端序缓冲区读取有符号32位整数
 */
static inline int32_t read_i32_be(const uint8_t *buf) {
  return (int32_t)read_u32_be(buf);
}

/* ============================================================================
 * 通用命令构建
 * ========================================================================== */

int zdt_build_encoder_calibration(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_ENCODER_CALIBRATION;
  buf->data[2] = ZDT_AUX_ENCODER_CAL;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_restart(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_RESTART;
  buf->data[2] = ZDT_AUX_RESTART;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_clear_position(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_CLEAR_POSITION;
  buf->data[2] = ZDT_AUX_CLEAR_POS;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_clear_protection(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_CLEAR_PROTECTION;
  buf->data[2] = ZDT_AUX_CLEAR_PROT;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_factory_reset(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_FACTORY_RESET;
  buf->data[2] = ZDT_AUX_FACTORY_RESET;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_enable(uint8_t addr, zdt_enable_state_t enable,
                     zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_ENABLE;
  buf->data[2] = ZDT_AUX_ENABLE;
  buf->data[3] = (uint8_t)enable;
  buf->data[4] = (uint8_t)sync;
  buf->data[5] = ZDT_CHECKSUM;
  buf->len = 6;

  return 6;
}

int zdt_build_homing(uint8_t addr, uint8_t mode, zdt_sync_flag_t sync,
                     zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_HOMING;
  buf->data[2] = mode;
  buf->data[3] = (uint8_t)sync;
  buf->data[4] = ZDT_CHECKSUM;
  buf->len = 5;

  return 5;
}

int zdt_build_emergency_stop(uint8_t addr, zdt_sync_flag_t sync,
                             zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_EMERGENCY_STOP;
  buf->data[2] = ZDT_AUX_EMERGENCY_STOP;
  buf->data[3] = (uint8_t)sync;
  buf->data[4] = ZDT_CHECKSUM;
  buf->len = 5;

  return 5;
}

int zdt_build_read_position(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_POSITION;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_pulses(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_PULSES;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_io_status(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_IO_STATUS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_status_flags(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_STATUS_FLAGS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_config(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_CONFIG;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_timed_return(uint8_t addr, uint8_t info_func_code,
                           uint16_t interval_ms, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_TIMED_RETURN;
  buf->data[2] = ZDT_AUX_TIMED_RETURN;
  buf->data[3] = info_func_code;
  write_u16_be(&buf->data[4], interval_ms);
  buf->data[6] = ZDT_CHECKSUM;
  buf->len = 7;

  return 7;
}

/* ============================================================================
 * 其他读取命令构建函数
 * ========================================================================== */

int zdt_build_read_version(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_VERSION;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_phase_rl(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_PHASE_RL;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_bus_voltage(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_BUS_VOLTAGE;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_bus_current(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_BUS_CURRENT;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_phase_current(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_PHASE_CURRENT;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_battery_voltage(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_BATTERY_VOLTAGE;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_encoder_linear(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_ENCODER_LINEAR;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_input_pulses(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_INPUT_PULSES;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_target_pos(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_TARGET_POS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_set_target_pos(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_SET_TARGET_POS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_speed(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_SPEED;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_position_error(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_POSITION_ERROR;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_homing_status(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_HOMING_STATUS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_both_status(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_BOTH_STATUS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_io_levels(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_IO_LEVELS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_system_status(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_SYSTEM_STATUS;
  buf->data[2] = 0x7A;  // 辅助码
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

/* ============================================================================
 * 其他控制命令构建函数
 * ========================================================================== */

int zdt_build_set_homing_zero(uint8_t addr, bool store, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_SET_HOMING_ZERO;
  buf->data[2] = ZDT_AUX_SET_HOMING_ZERO;
  buf->data[3] = store ? 0x01 : 0x00;
  buf->data[4] = ZDT_CHECKSUM;
  buf->len = 5;

  return 5;
}

int zdt_build_exit_homing(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_EXIT_HOMING;
  buf->data[2] = ZDT_AUX_EXIT_HOMING;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_sync_trigger(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_SYNC_TRIGGER;
  buf->data[2] = ZDT_AUX_SYNC_TRIGGER;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

/* ============================================================================
 * EMM固件运动控制命令构建
 * ========================================================================== */

/**
 * @brief 构建力矩模式控制命令 (X固件)
 */
int zdt_build_torque_mode(uint8_t addr, zdt_direction_t dir,
                          uint32_t slope, uint16_t current_ma,
                          zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // X固件力矩模式命令格式 (11字节):
  // Addr | F5 | Dir | Slope[2] | Current[2] | Sync | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_TORQUE_MODE;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], (uint16_t)slope);  // 斜率2字节
  write_u16_be(&buf->data[5], current_ma);        // 电流2字节
  buf->data[7] = (uint8_t)sync;
  buf->data[8] = ZDT_CHECKSUM;
  buf->len = 9;

  return 9;
}

/**
 * @brief 构建力矩模式限速控制命令 (X固件)
 */
int zdt_build_torque_mode_limit(uint8_t addr, zdt_direction_t dir,
                                uint32_t slope, uint16_t current_ma,
                                zdt_sync_flag_t sync, uint16_t max_speed,
                                zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // X固件力矩模式限速控制命令格式 (11字节):
  // Addr | C5 | Dir | Slope[2] | Current[2] | Sync | MaxSpeed[2] | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_TORQUE_MODE_LIMIT;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], (uint16_t)slope);   // 斜率2字节
  write_u16_be(&buf->data[5], current_ma);         // 电流2字节
  buf->data[7] = (uint8_t)sync;
  write_u16_be(&buf->data[8], max_speed);          // 最大速度2字节
  buf->data[10] = ZDT_CHECKSUM;
  buf->len = 11;

  return 11;
}

/**
 * @brief 构建速度模式限电流控制命令 (X固件)
 */
int zdt_build_speed_mode_limit(uint8_t addr, zdt_direction_t dir,
                               uint32_t acceleration, uint16_t speed_rpm,
                               zdt_sync_flag_t sync, uint16_t max_current_ma,
                               zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // X固件速度模式限电流控制命令格式 (11字节):
  // Addr | C6 | Dir | Acc[2] | Speed[2] | Sync | MaxCurrent[2] | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_SPEED_MODE_LIMIT;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], (uint16_t)acceleration);  // 加速度2字节
  write_u16_be(&buf->data[5], speed_rpm);                // 速度2字节
  buf->data[7] = (uint8_t)sync;
  write_u16_be(&buf->data[8], max_current_ma);           // 最大电流2字节
  buf->data[10] = ZDT_CHECKSUM;
  buf->len = 11;

  return 11;
}

/**
 * @brief 构建速度模式控制命令 (X固件，见5.3.5)
 */
int zdt_build_speed_mode_x(uint8_t addr, zdt_direction_t dir,
                           uint32_t acceleration, uint16_t speed_rpm,
                           zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // X固件速度模式控制命令格式 (9字节):
  // Addr | F6 | Dir | Acc[2] | Speed[2] | Sync | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_SPEED_MODE;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], (uint16_t)acceleration);  // 加速度2字节
  write_u16_be(&buf->data[5], speed_rpm);                // 速度2字节
  buf->data[7] = (uint8_t)sync;
  buf->data[8] = ZDT_CHECKSUM;
  buf->len = 9;

  return 9;
}

int zdt_build_speed_mode(uint8_t addr, zdt_direction_t dir, uint16_t speed_rpm,
                         uint8_t acc, zdt_sync_flag_t sync,
                         zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // EMM固件速度模式命令格式 (8字节):
  // Addr | F6 | Dir | Speed[2] | Acc | Sync | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_SPEED_MODE;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], speed_rpm);
  buf->data[5] = acc;
  buf->data[6] = (uint8_t)sync;
  buf->data[7] = ZDT_CHECKSUM;
  buf->len = 8;

  return 8;
}

int zdt_build_position_mode_emm(uint8_t addr, zdt_direction_t dir,
                                uint16_t speed_rpm, uint8_t acc,
                                int32_t position_01deg,
                                zdt_motion_mode_t motion_mode, zdt_sync_flag_t sync,
                                zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // EMM固件位置模式命令格式 (13字节):
  // Addr | FD | Dir | Speed[2] | Acc | Pos[4] | Mode | Sync | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_POSITION_MODE_EMM;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], speed_rpm);
  buf->data[5] = acc;
  write_i32_be(&buf->data[6], position_01deg);
  buf->data[10] = (uint8_t)motion_mode;
  buf->data[11] = (uint8_t)sync;
  buf->data[12] = ZDT_CHECKSUM;
  buf->len = 13;

  return 13;
}

/**
 * @brief 构建直通限速位置模式控制命令 (X固件)
 */
int zdt_build_position_through(uint8_t addr, zdt_direction_t dir,
                               uint16_t speed_rpm, int32_t position_01deg,
                               zdt_motion_mode_t motion_mode, zdt_sync_flag_t sync,
                               zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // X固件直通限速位置模式命令格式 (12字节):
  // Addr | FB | Dir | Speed[2] | Pos[4] | Mode | Sync | 6B
  // 注意：速度需要乘以10，位置已经是0.1°单位
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_POSITION_THROUGH;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], speed_rpm * 10);  // 速度乘以10
  write_i32_be(&buf->data[5], position_01deg);
  buf->data[9] = (uint8_t)motion_mode;
  buf->data[10] = (uint8_t)sync;
  buf->data[11] = ZDT_CHECKSUM;
  buf->len = 12;

  return 12;
}

/**
 * @brief 构建直通限速位置模式限电流控制命令 (X固件)
 */
int zdt_build_position_through_limit(uint8_t addr, zdt_direction_t dir,
                                     uint16_t speed_rpm, int32_t position_01deg,
                                     zdt_motion_mode_t motion_mode, zdt_sync_flag_t sync,
                                     uint16_t max_current_ma, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // X固件直通限速位置模式限电流控制命令格式 (14字节):
  // Addr | CB | Dir | Speed[2] | Pos[4] | Mode | Sync | MaxCurrent[2] | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_POSITION_THROUGH_LIMIT;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], speed_rpm * 10);  // 速度乘以10
  write_i32_be(&buf->data[5], position_01deg);
  buf->data[9] = (uint8_t)motion_mode;
  buf->data[10] = (uint8_t)sync;
  write_u16_be(&buf->data[11], max_current_ma);
  buf->data[13] = ZDT_CHECKSUM;
  buf->len = 14;

  return 14;
}

/**
 * @brief 构建梯形曲线加减速位置模式控制命令 (X固件)
 */
int zdt_build_position_trapezoid(uint8_t addr, zdt_direction_t dir,
                                 uint16_t acc_acceleration, uint16_t dec_acceleration,
                                 uint16_t max_speed_rpm, int32_t position_01deg,
                                 zdt_motion_mode_t motion_mode, zdt_sync_flag_t sync,
                                 zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // X固件梯形曲线加减速位置模式命令格式 (16字节):
  // Addr | FD | Dir | AccAcc[2] | DecAcc[2] | MaxSpeed[2] | Pos[4] | Mode | Sync | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_POSITION_TRAPEZOID;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], acc_acceleration);
  write_u16_be(&buf->data[5], dec_acceleration);
  write_u16_be(&buf->data[7], max_speed_rpm * 10);  // 最大速度乘以10
  write_i32_be(&buf->data[9], position_01deg);
  buf->data[13] = (uint8_t)motion_mode;
  buf->data[14] = (uint8_t)sync;
  buf->data[15] = ZDT_CHECKSUM;
  buf->len = 16;

  return 16;
}

/**
 * @brief 构建梯形曲线加减速位置模式限电流控制命令 (X固件)
 */
int zdt_build_position_trapezoid_limit(uint8_t addr, zdt_direction_t dir,
                                       uint16_t acc_acceleration, uint16_t dec_acceleration,
                                       uint16_t max_speed_rpm, int32_t position_01deg,
                                       zdt_motion_mode_t motion_mode, zdt_sync_flag_t sync,
                                       uint16_t max_current_ma, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // X固件梯形曲线加减速位置模式限电流控制命令格式 (18字节):
  // Addr | CD | Dir | AccAcc[2] | DecAcc[2] | MaxSpeed[2] | Pos[4] | Mode | Sync | MaxCurrent[2] | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_POSITION_TRAPEZOID_LIMIT;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], acc_acceleration);
  write_u16_be(&buf->data[5], dec_acceleration);
  write_u16_be(&buf->data[7], max_speed_rpm * 10);  // 最大速度乘以10
  write_i32_be(&buf->data[9], position_01deg);
  buf->data[13] = (uint8_t)motion_mode;
  buf->data[14] = (uint8_t)sync;
  write_u16_be(&buf->data[15], max_current_ma);
  buf->data[17] = ZDT_CHECKSUM;
  buf->len = 18;

  return 18;
}

/* ============================================================================
 * CAN协议层API
 * ========================================================================== */

int zdt_cmd_to_can_msgs(uint8_t addr, const zdt_cmd_buffer_t *cmd,
                        zdt_can_msg_t *msgs, size_t max_msgs) {
  if (!cmd || !msgs || max_msgs == 0) {
    return ZDT_ERR_INVALID_PARAM;
  }

  // 跳过地址字节，CAN帧中地址在ID中
  // cmd->data 格式: [Addr][CmdCode][Data...][Checksum]
  uint8_t cmd_code = cmd->data[1];    // 保存命令码
  const uint8_t *data = cmd->data + 1; // 跳过地址字节，从命令码开始
  size_t data_len = cmd->len - 1;      // 减去地址字节

  // 计算需要的包数
  // 第一包可放8字节数据，后续包每包需要预留1字节给命令码，实际数据7字节
  size_t num_packets;
  if (data_len <= ZDT_CAN_MAX_DATA_LEN) {
    num_packets = 1;
  } else {
    // 第一包8字节，后续每包7字节数据+1字节命令码
    size_t remaining_after_first = data_len - ZDT_CAN_MAX_DATA_LEN;
    num_packets = 1 + (remaining_after_first + (ZDT_CAN_MAX_DATA_LEN - 1) - 1) /
                          (ZDT_CAN_MAX_DATA_LEN - 1);
  }

  if (num_packets > max_msgs) {
    return ZDT_ERR_BUFFER_TOO_SMALL;
  }

  // 拆分数据到CAN帧
  size_t offset = 0;
  for (size_t i = 0; i < num_packets; i++) {
    msgs[i].id = zdt_make_can_id(addr, (uint8_t)i);
    msgs[i].is_extended = true;

    if (i == 0) {
      // 第一包：直接复制数据（包含命令码）
      size_t remaining = data_len - offset;
      msgs[i].data_len = (remaining > ZDT_CAN_MAX_DATA_LEN)
                             ? ZDT_CAN_MAX_DATA_LEN
                             : (uint8_t)remaining;
      memcpy(msgs[i].data, data + offset, msgs[i].data_len);
      offset += msgs[i].data_len;
    } else {
      // 后续包：先放命令码，再放数据
      msgs[i].data[0] = cmd_code;
      size_t remaining = data_len - offset;
      size_t payload_len = (remaining > (ZDT_CAN_MAX_DATA_LEN - 1))
                               ? (ZDT_CAN_MAX_DATA_LEN - 1)
                               : remaining;
      memcpy(msgs[i].data + 1, data + offset, payload_len);
      msgs[i].data_len = (uint8_t)(1 + payload_len);
      offset += payload_len;
    }
  }

  return (int)num_packets;
}

int zdt_can_msgs_to_response(const zdt_can_msg_t *msgs, size_t msg_count,
                             zdt_response_t *response) {
  if (!msgs || !response || msg_count == 0) {
    return ZDT_ERR_INVALID_PARAM;
  }

  // 清空响应结构
  memset(response, 0, sizeof(*response));

  // 解析地址
  uint8_t addr, packet;
  zdt_parse_can_id(msgs[0].id, &addr, &packet);
  response->addr = addr;

  // 组合所有包的数据
  size_t total_len = 0;
  for (size_t i = 0; i < msg_count; i++) {
    if (total_len + msgs[i].data_len > ZDT_MAX_CMD_SIZE) {
      return ZDT_ERR_BUFFER_TOO_SMALL;
    }

    memcpy(response->data + total_len, msgs[i].data, msgs[i].data_len);
    total_len += msgs[i].data_len;
  }

  // 解析功能码和状态
  if (total_len >= 2) {
    response->func_code = response->data[0];
    response->status = response->data[1];

    // 移动额外数据
    if (total_len > 2) {
      response->data_len = total_len - 2;
      memmove(response->data, response->data + 2, response->data_len);
    }
  }

  return ZDT_OK;
}

/* ============================================================================
 * 响应解析API
 * ========================================================================== */

int zdt_parse_position_response(const zdt_response_t *response,
                                zdt_position_t *position) {
  if (!response || !position) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_POSITION) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 36 P3 P2 P1 P0 6B
  // 在response中，data包含P3 P2 P1 P0（4字节）
  if (response->data_len < 4) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  position->angle_raw = read_i32_be(response->data);
  
  // 根据协议文档5.5.13，两种固件的位置角度含义不同：
  // Emm固件：0-65535表示一圈0-360°，角度值=(实时位置 * 360)/65536
  // X固件：角度值 = 实时位置/10，比如返回的实时位置为16，则为1.6°
  // 这里默认为X固件，如果需要支持Emm固件，需要根据固件类型判断
  position->angle_deg = position->angle_raw / 10.0f;

  return ZDT_OK;
}

int zdt_parse_pulses_response(const zdt_response_t *response, int32_t *pulses) {
  if (!response || !pulses) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_PULSES) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  if (response->data_len < 4) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *pulses = read_i32_be(response->data);

  return ZDT_OK;
}

bool zdt_response_is_ok(const zdt_response_t *response) {
  if (!response)
    return false;

  return response->status == ZDT_RESP_OK ||
         response->status == ZDT_RESP_AT_ZERO ||
         response->status == ZDT_RESP_ACTION_DONE;
}

const char *zdt_response_status_str(uint8_t status) {
  switch (status) {
  case ZDT_RESP_OK:
    return "OK";
  case ZDT_RESP_AT_ZERO:
    return "Already at zero";
  case ZDT_RESP_PARAM_ERROR:
    return "Parameter error";
  case ZDT_RESP_FORMAT_ERROR:
    return "Format error";
  case ZDT_RESP_ACTION_DONE:
    return "Action completed";
  default:
    return "Unknown";
  }
}

/* ============================================================================
 * 其他响应解析函数
 * ========================================================================== */

int zdt_parse_version_response(const zdt_response_t *response,
                               zdt_version_info_t *version) {
  if (!response || !version) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_VERSION) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 1F FW_Ver HW_Series HW_Type HW_Ver 6B
  if (response->data_len < 4) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  version->firmware_version = ((uint16_t)response->data[0] << 8) | response->data[1];
  version->hardware_series = response->data[2];
  version->hardware_type = response->data[3];
  if (response->data_len >= 5) {
    version->hardware_version = response->data[4];
  }

  return ZDT_OK;
}

int zdt_parse_phase_rl_response(const zdt_response_t *response,
                               zdt_phase_rl_t *phase_rl) {
  if (!response || !phase_rl) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_PHASE_RL) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 20 Pha_R[2] Pha_L[2] 6B
  if (response->data_len < 4) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  phase_rl->phase_resistance = read_u16_be(&response->data[0]);
  phase_rl->phase_inductance = read_u16_be(&response->data[2]);

  return ZDT_OK;
}

int zdt_parse_homing_status_response(const zdt_response_t *response,
                                    zdt_homing_status_t *status) {
  if (!response || !status) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_HOMING_STATUS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 3B Status 6B
  if (response->data_len < 1) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  uint8_t status_byte = response->data[0];
  status->enc_ready = (status_byte & 0x01) != 0;
  status->cal_ready = (status_byte & 0x02) != 0;
  status->homing_running = (status_byte & 0x04) != 0;
  status->homing_failed = (status_byte & 0x08) != 0;
  status->otp_triggered = (status_byte & 0x10) != 0;
  status->ocp_triggered = (status_byte & 0x20) != 0;

  return ZDT_OK;
}

int zdt_parse_motor_status_response(const zdt_response_t *response,
                                   zdt_motor_status_t *status) {
  if (!response || !status) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_STATUS_FLAGS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 3A Status 6B
  if (response->data_len < 1) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  uint8_t status_byte = response->data[0];
  status->motor_enabled = (status_byte & 0x01) != 0;
  status->position_reached = (status_byte & 0x02) != 0;
  status->stall_flag = (status_byte & 0x04) != 0;
  status->stall_protection = (status_byte & 0x08) != 0;
  status->left_limit_switch = (status_byte & 0x10) != 0;
  status->right_limit_switch = (status_byte & 0x20) != 0;
  status->power_loss = (status_byte & 0x80) != 0;

  return ZDT_OK;
}

int zdt_parse_both_status_response(const zdt_response_t *response,
                                  zdt_homing_status_t *homing_status,
                                  zdt_motor_status_t *motor_status) {
  if (!response || !homing_status || !motor_status) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_BOTH_STATUS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 3C Homing_Status Motor_Status 6B
  if (response->data_len < 2) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  uint8_t homing_byte = response->data[0];
  homing_status->enc_ready = (homing_byte & 0x01) != 0;
  homing_status->cal_ready = (homing_byte & 0x02) != 0;
  homing_status->homing_running = (homing_byte & 0x04) != 0;
  homing_status->homing_failed = (homing_byte & 0x08) != 0;
  homing_status->otp_triggered = (homing_byte & 0x10) != 0;
  homing_status->ocp_triggered = (homing_byte & 0x20) != 0;

  uint8_t motor_byte = response->data[1];
  motor_status->motor_enabled = (motor_byte & 0x01) != 0;
  motor_status->position_reached = (motor_byte & 0x02) != 0;
  motor_status->stall_flag = (motor_byte & 0x04) != 0;
  motor_status->stall_protection = (motor_byte & 0x08) != 0;
  motor_status->left_limit_switch = (motor_byte & 0x10) != 0;
  motor_status->right_limit_switch = (motor_byte & 0x20) != 0;
  motor_status->power_loss = (motor_byte & 0x80) != 0;

  return ZDT_OK;
}

int zdt_parse_speed_response(const zdt_response_t *response, float *speed_rpm) {
  if (!response || !speed_rpm) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_SPEED) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 35 Sign Speed[2] 6B
  if (response->data_len < 3) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  uint8_t sign = response->data[0];
  uint16_t speed = read_u16_be(&response->data[1]);

  // Emm固件: 单位为RPM
  // X固件: 单位为0.1RPM，需要除以10
  // 这里假设为EMM固件，如果需要支持X固件，需要根据固件类型判断
  *speed_rpm = (sign == 0x01) ? -(float)speed : (float)speed;

  return ZDT_OK;
}

int zdt_parse_target_pos_response(const zdt_response_t *response,
                                 zdt_position_t *position) {
  if (!response || !position) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_TARGET_POS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 33 Sign Pos[4] 6B
  if (response->data_len < 5) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  uint8_t sign = response->data[0];
  int32_t pos = read_i32_be(&response->data[1]);

  position->angle_raw = (sign == 0x01) ? -pos : pos;

  // 根据协议文档5.5.9，两种固件返回的含义不同：
  // Emm固件: 0-65535表示一圈0-360°，角度值=(目标位置 * 360)/65536
  // X固件: 角度值 = 目标位置/10，比如返回的目标位置为16，则为1.6°
  // 这里默认为X固件，如果需要支持Emm固件，需要根据固件类型判断
  position->angle_deg = position->angle_raw / 10.0f;

  return ZDT_OK;
}

int zdt_parse_encoder_linear_response(const zdt_response_t *response,
                                     uint16_t *encoder_val) {
  if (!response || !encoder_val) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_ENCODER_LINEAR) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 31 Encoder[2] 6B
  if (response->data_len < 2) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *encoder_val = read_u16_be(response->data);

  return ZDT_OK;
}

/* ============================================================================
 * 调试工具函数
 * ========================================================================== */

void zdt_print_cmd_buffer(const zdt_cmd_buffer_t *buf) {
  if (!buf)
    return;

  printf("CMD [%zu]: ", buf->len);
  for (size_t i = 0; i < buf->len; i++) {
    printf("%02X ", buf->data[i]);
  }
  printf("\n");
}

void zdt_print_can_msg(const zdt_can_msg_t *msg) {
  if (!msg)
    return;

  printf("CAN ID: 0x%08lX (%s), Data [%u]: ", (unsigned long)msg->id,
         msg->is_extended ? "EXT" : "STD", msg->data_len);

  for (int i = 0; i < msg->data_len; i++) {
    printf("%02X ", msg->data[i]);
  }
  printf("\n");
}

/* ============================================================================
 * ESP-IDF CAN驱动集成 (需要CONFIG_IDF_TARGET)
 * ========================================================================== */

#ifdef CONFIG_IDF_TARGET

#include "zdt_can_driver.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "zdt_can";

/** 内部句柄结构 */
struct zdt_handle_s {
  const zdt_can_driver_t *driver;
  void *driver_context;
};

zdt_error_t zdt_can_init(const zdt_can_config_t *config,
                         const zdt_can_driver_t *driver,
                         zdt_handle_t *handle) {
  if (!config || !handle) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (!driver) {
    ESP_LOGE(TAG, "Driver not provided");
    return ZDT_ERR_INVALID_PARAM;
  }

  struct zdt_handle_s *h = calloc(1, sizeof(struct zdt_handle_s));
  if (!h) {
    ESP_LOGE(TAG, "Failed to allocate handle");
    return ZDT_ERR_INVALID_PARAM;
  }

  h->driver = driver;

  // 映射配置到驱动配置
  zdt_can_driver_config_t driver_config = {
      .tx_gpio = config->tx_gpio,
      .rx_gpio = config->rx_gpio,
      .bitrate = config->bitrate,
      .tx_timeout_ms = config->tx_timeout_ms,
      .rx_timeout_ms = config->rx_timeout_ms,
  };

  // 初始化驱动
  int ret = zdt_driver_init(h->driver, &driver_config, &h->driver_context);
  if (ret != 0) {
    free(h);
    return (zdt_error_t)ret;
  }

  *handle = h;

  ESP_LOGI(TAG, "ZDT CAN initialized (TX:%d, RX:%d, %lu bps)", config->tx_gpio,
           config->rx_gpio, (unsigned long)config->bitrate);

  return ZDT_OK;
}

void zdt_can_deinit(zdt_handle_t handle) {
  if (!handle)
    return;

  if (handle->driver) {
    zdt_driver_deinit(handle->driver, handle->driver_context);
  }

  free(handle);
}

zdt_error_t zdt_can_send_cmd(zdt_handle_t handle, uint8_t addr,
                             const zdt_cmd_buffer_t *cmd,
                             zdt_response_t *response) {
  if (!handle || !cmd) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (!handle->driver) {
    return ZDT_ERR_NOT_INITIALIZED;
  }

  // 转换为CAN消息
  zdt_can_msg_t can_msgs[8];
  int num_msgs = zdt_cmd_to_can_msgs(addr, cmd, can_msgs, 8);
  if (num_msgs < 0) {
    return (zdt_error_t)num_msgs;
  }

  // 发送所有CAN帧
  for (int i = 0; i < num_msgs; i++) {
    zdt_can_driver_msg_t driver_msg = {
        .id = can_msgs[i].id,
        .is_extended = can_msgs[i].is_extended,
        .data_len = can_msgs[i].data_len,
    };
    memcpy(driver_msg.data, can_msgs[i].data, can_msgs[i].data_len);

    int ret = zdt_driver_send(handle->driver, handle->driver_context, &driver_msg);
    if (ret != 0) {
      ESP_LOGE(TAG, "CAN TX failed");
      return (zdt_error_t)ret;
    }
  }

  // 如果需要响应，接收
  if (response) {
    zdt_can_driver_msg_t driver_msg;
    int ret = zdt_driver_receive(handle->driver, handle->driver_context, &driver_msg);
    if (ret != 0) {
      ESP_LOGE(TAG, "CAN RX failed");
      return (zdt_error_t)ret;
    }

    // 转换为响应结构
    zdt_can_msg_t rx_can = {
        .id = driver_msg.id,
        .is_extended = driver_msg.is_extended,
        .data_len = driver_msg.data_len,
    };
    memcpy(rx_can.data, driver_msg.data, driver_msg.data_len);

    zdt_can_msgs_to_response(&rx_can, 1, response);
  }

  return ZDT_OK;
}

/* 便捷函数实现 */

zdt_error_t zdt_encoder_calibration(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_build_encoder_calibration(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, NULL);
}

zdt_error_t zdt_restart(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_build_restart(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, NULL);
}

zdt_error_t zdt_clear_position(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_clear_position(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_clear_protection(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_clear_protection(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_factory_reset(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_factory_reset(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_enable(zdt_handle_t handle, uint8_t addr, bool enable) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_enable(addr, enable ? ZDT_ENABLE_ON : ZDT_ENABLE_OFF,
                   ZDT_SYNC_IMMEDIATE, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_homing(zdt_handle_t handle, uint8_t addr, uint8_t mode) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_homing(addr, mode, ZDT_SYNC_IMMEDIATE, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_emergency_stop(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_emergency_stop(addr, ZDT_SYNC_IMMEDIATE, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_read_position(zdt_handle_t handle, uint8_t addr,
                              zdt_position_t *position) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;

  zdt_build_read_position(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;

  return zdt_parse_position_response(&response, position);
}

zdt_error_t zdt_read_pulses(zdt_handle_t handle, uint8_t addr,
                            int32_t *pulses) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;

  zdt_build_read_pulses(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;

  return zdt_parse_pulses_response(&response, pulses);
}

zdt_error_t zdt_set_timed_return(zdt_handle_t handle, uint8_t addr,
                                 uint8_t info_func_code,
                                 uint16_t interval_ms) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;

  int len = zdt_build_timed_return(addr, info_func_code, interval_ms, &cmd);
  if (len < 0)
    return (zdt_error_t)len;

  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_speed(zdt_handle_t handle, uint8_t addr,
                          zdt_direction_t dir, uint16_t speed_rpm,
                          uint8_t acc) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_speed_mode(addr, dir, speed_rpm, acc, ZDT_SYNC_IMMEDIATE, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_torque_mode(zdt_handle_t handle, uint8_t addr,
                                zdt_direction_t dir, uint32_t slope,
                                uint16_t current_ma) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_torque_mode(addr, dir, slope, current_ma, ZDT_SYNC_IMMEDIATE, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_torque_mode_limit(zdt_handle_t handle, uint8_t addr,
                                      zdt_direction_t dir, uint32_t slope,
                                      uint16_t current_ma, uint16_t max_speed) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_torque_mode_limit(addr, dir, slope, current_ma, ZDT_SYNC_IMMEDIATE, max_speed, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_speed_mode_limit(zdt_handle_t handle, uint8_t addr,
                                     zdt_direction_t dir, uint32_t acceleration,
                                     uint16_t speed_rpm, uint16_t max_current_ma) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_speed_mode_limit(addr, dir, acceleration, speed_rpm, ZDT_SYNC_IMMEDIATE, max_current_ma, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_position(zdt_handle_t handle, uint8_t addr,
                             zdt_direction_t dir, float speed_rpm,
                             float angle_deg, zdt_motion_mode_t mode,
                             bool sync_cache) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;

  // 使用默认加速度档位 0 (不限制加速度)
  const uint8_t default_acc = 0;

  // 同步标志: sync_cache=true 时使用缓存等待同步命令
  zdt_sync_flag_t sync = sync_cache ? ZDT_SYNC_CACHE : ZDT_SYNC_IMMEDIATE;

  // 根据协议文档5.3.12，Emm固件位置模式使用脉冲数作为位置单位
  // 默认1.8°步进电机，16细分下，3200个脉冲 = 360°
  // 脉冲数 = (角度 / 360.0) * 3200
  int32_t position_pulses = (int32_t)(angle_deg * 3200.0f / 360.0f);

  zdt_build_position_mode_emm(addr, dir, (uint16_t)speed_rpm, default_acc,
                               position_pulses, mode, sync, &cmd);

  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

/* ============================================================================
 * 其他高级封装函数
 * ========================================================================== */

zdt_error_t zdt_sync_trigger(zdt_handle_t handle) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 使用广播地址触发多机同步运动
  zdt_build_sync_trigger(ZDT_BROADCAST_ADDR, &cmd);
  return zdt_can_send_cmd(handle, ZDT_BROADCAST_ADDR, &cmd, &response);
}

zdt_error_t zdt_set_homing_zero(zdt_handle_t handle, uint8_t addr, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_set_homing_zero(addr, store, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_exit_homing(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_exit_homing(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_read_version(zdt_handle_t handle, uint8_t addr,
                            zdt_version_info_t *version) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_version(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  return zdt_parse_version_response(&response, version);
}

zdt_error_t zdt_read_phase_rl(zdt_handle_t handle, uint8_t addr,
                             zdt_phase_rl_t *phase_rl) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_phase_rl(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  return zdt_parse_phase_rl_response(&response, phase_rl);
}

zdt_error_t zdt_read_bus_voltage(zdt_handle_t handle, uint8_t addr,
                                uint16_t *voltage_mv) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_bus_voltage(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 24 VBus[2] 6B
  if (response.data_len >= 2) {
    *voltage_mv = read_u16_be(response.data);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

zdt_error_t zdt_read_bus_current(zdt_handle_t handle, uint8_t addr,
                                uint16_t *current_ma) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_bus_current(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 26 CBus[2] 6B
  if (response.data_len >= 2) {
    *current_ma = read_u16_be(response.data);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

zdt_error_t zdt_read_phase_current(zdt_handle_t handle, uint8_t addr,
                                 uint16_t *current_ma) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_phase_current(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 27 CPha[2] 6B
  if (response.data_len >= 2) {
    *current_ma = read_u16_be(response.data);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

zdt_error_t zdt_read_battery_voltage(zdt_handle_t handle, uint8_t addr,
                                    uint16_t *voltage_mv) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_battery_voltage(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 38 VBat[2] 6B
  if (response.data_len >= 2) {
    *voltage_mv = read_u16_be(response.data);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

zdt_error_t zdt_read_encoder_linear(zdt_handle_t handle, uint8_t addr,
                                   uint16_t *encoder_val) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_encoder_linear(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  return zdt_parse_encoder_linear_response(&response, encoder_val);
}

zdt_error_t zdt_read_speed(zdt_handle_t handle, uint8_t addr, float *speed_rpm) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_speed(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  return zdt_parse_speed_response(&response, speed_rpm);
}

zdt_error_t zdt_read_position_error(zdt_handle_t handle, uint8_t addr,
                                   zdt_position_t *error) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_position_error(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  return zdt_parse_position_error_response(&response, error);
}

zdt_error_t zdt_read_homing_status(zdt_handle_t handle, uint8_t addr,
                                  zdt_homing_status_t *status) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_homing_status(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  return zdt_parse_homing_status_response(&response, status);
}

zdt_error_t zdt_read_motor_status(zdt_handle_t handle, uint8_t addr,
                                 zdt_motor_status_t *status) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_status_flags(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  return zdt_parse_motor_status_response(&response, status);
}

zdt_error_t zdt_read_both_status(zdt_handle_t handle, uint8_t addr,
                               zdt_homing_status_t *homing_status,
                               zdt_motor_status_t *motor_status) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_both_status(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  return zdt_parse_both_status_response(&response, homing_status, motor_status);
}

zdt_error_t zdt_read_system_status(zdt_handle_t handle, uint8_t addr,
                                   zdt_system_status_t *status) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_system_status(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  return zdt_parse_system_status_response(&response, status);
}

zdt_error_t zdt_read_homing_params(zdt_handle_t handle, uint8_t addr,
                                   zdt_homing_params_t *params) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建读取回零参数命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_READ_HOMING_PARAMS;
  cmd.data[2] = ZDT_CHECKSUM;
  cmd.len = 3;
  
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 解析回零参数响应 (根据协议文档格式解析)
  if (response.data_len >= 14) {
    params->mode = response.data[0];
    params->direction = response.data[1];
    params->speed = read_u16_be(&response.data[2]);
    params->timeout_ms = read_u32_be(&response.data[4]);
    params->collision_speed = read_u16_be(&response.data[8]);
    params->collision_current = read_u16_be(&response.data[10]);
    params->collision_time = read_u16_be(&response.data[12]);
    if (response.data_len >= 15) {
      params->auto_power_on_homing = response.data[14] != 0;
    }
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

zdt_error_t zdt_set_homing_params(zdt_handle_t handle, uint8_t addr,
                                const zdt_homing_params_t *params,
                                bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改回零参数命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_HOMING_PARAMS;
  cmd.data[2] = ZDT_AUX_SET_HOMING_PARAMS;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = params->mode;
  cmd.data[5] = params->direction;
  write_u16_be(&cmd.data[6], params->speed);
  write_u32_be(&cmd.data[8], params->timeout_ms);
  write_u16_be(&cmd.data[12], params->collision_speed);
  write_u16_be(&cmd.data[14], params->collision_current);
  write_u16_be(&cmd.data[16], params->collision_time);
  cmd.data[18] = params->auto_power_on_homing ? 0x01 : 0x00;
  cmd.data[19] = ZDT_CHECKSUM;
  cmd.len = 20;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_motor_id(zdt_handle_t handle, uint8_t addr,
                            uint8_t new_id, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改电机ID命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_MOTOR_ID;
  cmd.data[2] = ZDT_AUX_SET_MOTOR_ID;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = new_id;
  cmd.data[5] = ZDT_CHECKSUM;
  cmd.len = 6;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_subdivision(zdt_handle_t handle, uint8_t addr,
                               uint8_t subdivision, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改细分值命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_SUBDIVISION;
  cmd.data[2] = ZDT_AUX_SET_SUBDIVISION;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = subdivision;
  cmd.data[5] = ZDT_CHECKSUM;
  cmd.len = 6;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_power_loss_flag(zdt_handle_t handle, uint8_t addr,
                                   bool flag) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改掉电标志命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_POWER_LOSS_FLAG;
  cmd.data[2] = flag ? 0x01 : 0x00;
  cmd.data[3] = ZDT_CHECKSUM;
  cmd.len = 4;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_motor_type(zdt_handle_t handle, uint8_t addr,
                              zdt_motor_type_t type, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改电机类型命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_MOTOR_TYPE;
  cmd.data[2] = ZDT_AUX_SET_MOTOR_TYPE;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = (uint8_t)type;
  cmd.data[5] = ZDT_CHECKSUM;
  cmd.len = 6;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_firmware_type(zdt_handle_t handle, uint8_t addr,
                                 zdt_firmware_type_t type, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改固件类型命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_FIRMWARE_TYPE;
  cmd.data[2] = ZDT_AUX_SET_FIRMWARE_TYPE;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = (uint8_t)type;
  cmd.data[5] = ZDT_CHECKSUM;
  cmd.len = 6;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_control_mode(zdt_handle_t handle, uint8_t addr,
                                zdt_control_mode_t mode, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改控制模式命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_CONTROL_MODE;
  cmd.data[2] = ZDT_AUX_SET_CONTROL_MODE;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = (uint8_t)mode;
  cmd.data[5] = ZDT_CHECKSUM;
  cmd.len = 6;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_motor_dir(zdt_handle_t handle, uint8_t addr,
                              zdt_direction_t dir, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改电机方向命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_MOTOR_DIR;
  cmd.data[2] = ZDT_AUX_SET_MOTOR_DIR;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = (uint8_t)dir;
  cmd.data[5] = ZDT_CHECKSUM;
  cmd.len = 6;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_key_lock(zdt_handle_t handle, uint8_t addr,
                            bool lock, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改锁定按键功能命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_KEY_LOCK;
  cmd.data[2] = ZDT_AUX_SET_KEY_LOCK;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = lock ? 0x01 : 0x00;
  cmd.data[5] = ZDT_CHECKSUM;
  cmd.len = 6;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_open_loop_current(zdt_handle_t handle, uint8_t addr,
                                     uint16_t current_ma, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改开环模式工作电流命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_OPEN_LOOP_CURRENT;
  cmd.data[2] = ZDT_AUX_SET_OPEN_LOOP_CURRENT;
  cmd.data[3] = store ? 0x01 : 0x00;
  write_u16_be(&cmd.data[4], current_ma);
  cmd.data[6] = ZDT_CHECKSUM;
  cmd.len = 7;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_closed_loop_current(zdt_handle_t handle, uint8_t addr,
                                       uint16_t current_ma, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改闭环模式最大电流命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_CLOSED_LOOP_CURRENT;
  cmd.data[2] = ZDT_AUX_SET_CLOSED_LOOP_CURRENT;
  cmd.data[3] = store ? 0x01 : 0x00;
  write_u16_be(&cmd.data[4], current_ma);
  cmd.data[6] = ZDT_CHECKSUM;
  cmd.len = 7;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_pid_params_x(zdt_handle_t handle, uint8_t addr,
                               const zdt_pid_params_x_t *params,
                               bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改X固件PID参数命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_PID_PARAMS;
  cmd.data[2] = ZDT_AUX_SET_PID_PARAMS;
  cmd.data[3] = store ? 0x01 : 0x00;
  write_u32_be(&cmd.data[4], params->trapezoid_pos_kp);
  write_u32_be(&cmd.data[8], params->through_pos_kp);
  write_u32_be(&cmd.data[12], params->speed_kp);
  write_u32_be(&cmd.data[16], params->speed_ki);
  cmd.data[20] = ZDT_CHECKSUM;
  cmd.len = 21;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_pid_params_emm(zdt_handle_t handle, uint8_t addr,
                                  const zdt_pid_params_emm_t *params,
                                  bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改Emm固件PID参数命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_PID_PARAMS;
  cmd.data[2] = ZDT_AUX_SET_PID_PARAMS;
  cmd.data[3] = store ? 0x01 : 0x00;
  write_u32_be(&cmd.data[4], params->kp);
  write_u32_be(&cmd.data[8], params->ki);
  write_u32_be(&cmd.data[12], params->kd);
  cmd.data[16] = ZDT_CHECKSUM;
  cmd.len = 17;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_broadcast_read_id(zdt_handle_t handle, uint8_t *id) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建广播读取ID命令
  cmd.data[0] = ZDT_BROADCAST_ADDR;
  cmd.data[1] = ZDT_CMD_READ_ID;
  cmd.data[2] = ZDT_CHECKSUM;
  cmd.len = 3;
  
  zdt_error_t err = zdt_can_send_cmd(handle, ZDT_BROADCAST_ADDR, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 15 Addr 6B
  if (response.data_len >= 1) {
    *id = response.data[0];
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

/* ============================================================================
 * 补充缺失的高级封装函数
 * ========================================================================== */

/**
 * @brief 解析回零参数响应
 */
int zdt_parse_homing_params_response(const zdt_response_t *response,
                                     zdt_homing_params_t *params) {
  if (!response || !params) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_HOMING_PARAMS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 22 Mode Dir Speed[2] Timeout[4] 
  //             CollisionSpeed[2] CollisionCurrent[2] CollisionTime[2] AutoEn 6B
  if (response->data_len < 15) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  params->mode = response->data[0];
  params->direction = response->data[1];
  params->speed = read_u16_be(&response->data[2]);
  params->timeout_ms = read_u32_be(&response->data[4]);
  params->collision_speed = read_u16_be(&response->data[8]);
  params->collision_current = read_u16_be(&response->data[10]);
  params->collision_time = read_u16_be(&response->data[12]);
  params->auto_power_on_homing = response->data[14] != 0;

  return ZDT_OK;
}

/**
 * @brief 解析驱动温度响应
 */
int zdt_parse_temperature_response(const zdt_response_t *response, int8_t *temp_c) {
  if (!response || !temp_c) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != 0x39) { // ZDT_CMD_READ_IO_STATUS
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 39 Sign Temp 6B
  if (response->data_len < 2) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  uint8_t sign = response->data[0];
  uint8_t temp = response->data[1];
  *temp_c = (sign == 0x01) ? -(int8_t)temp : (int8_t)temp;

  return ZDT_OK;
}

/**
 * @brief 构建读取选项参数状态命令（X42S/Y42）
 */
int zdt_build_read_options(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_OPTIONS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

/**
 * @brief 构建读取位置到达窗口命令（X42S/Y42）
 */
int zdt_build_read_pos_window(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_POS_WINDOW;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

/**
 * @brief 构建读取过热过流保护检测阈值命令（X42S/Y42）
 */
int zdt_build_read_protection_threshold(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_PROTECT_THRESH;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

/**
 * @brief 构建读取心跳保护功能时间命令（X42S/Y42）
 */
int zdt_build_read_heartbeat_time(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_HEARTBEAT_TIME;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

/**
 * @brief 构建读取积分限幅/刚性系数命令（X42S/Y42）
 */
int zdt_build_read_integral_limit(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_INTEGRAL_LIMIT;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

/**
 * @brief 构建读取碰撞回零返回角度命令（X42S/Y42）
 */
int zdt_build_read_homing_angle(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_HOMING_ANGLE;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

/**
 * @brief 构建读取PID参数命令
 */
int zdt_build_read_pid_params(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_PID_PARAMS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

/**
 * @brief 读取选项参数状态（X42S/Y42）
 */
zdt_error_t zdt_read_options(zdt_handle_t handle, uint8_t addr,
                             uint8_t *options) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_options(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 1A Options 6B
  if (response.data_len >= 1) {
    *options = response.data[0];
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

/**
 * @brief 读取位置到达窗口（X42S/Y42）
 */
zdt_error_t zdt_read_position_window(zdt_handle_t handle, uint8_t addr,
                                    uint16_t *window) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_pos_window(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 41 Window[2] 6B
  if (response.data_len >= 2) {
    *window = read_u16_be(response.data);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

/**
 * @brief 读取过热过流保护检测阈值（X42S/Y42）
 */
zdt_error_t zdt_read_protection_threshold(zdt_handle_t handle, uint8_t addr,
                                         uint16_t *temp_threshold,
                                         uint16_t *current_threshold,
                                         uint16_t *time_threshold) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_protection_threshold(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 13 Temp[2] Current[2] Time[2] 6B
  if (response.data_len >= 6) {
    *temp_threshold = read_u16_be(&response.data[0]);
    *current_threshold = read_u16_be(&response.data[2]);
    *time_threshold = read_u16_be(&response.data[4]);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

/**
 * @brief 读取心跳保护功能时间（X42S/Y42）
 */
zdt_error_t zdt_read_heartbeat_time(zdt_handle_t handle, uint8_t addr,
                                    uint32_t *time_ms) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_heartbeat_time(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 16 Time[4] 6B
  if (response.data_len >= 4) {
    *time_ms = read_u32_be(response.data);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

/**
 * @brief 读取积分限幅/刚性系数（X42S/Y42）
 */
zdt_error_t zdt_read_integral_limit(zdt_handle_t handle, uint8_t addr,
                                    uint32_t *limit) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_integral_limit(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 23 Limit[4] 6B
  if (response.data_len >= 4) {
    *limit = read_u32_be(response.data);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

/**
 * @brief 读取碰撞回零返回角度（X42S/Y42）
 */
zdt_error_t zdt_read_homing_return_angle(zdt_handle_t handle, uint8_t addr,
                                        uint16_t *angle) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_homing_angle(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 3F Angle[2] 6B
  if (response.data_len >= 2) {
    *angle = read_u16_be(response.data);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

/**
 * @brief 读取PID参数（X固件）
 */
zdt_error_t zdt_read_pid_params_x(zdt_handle_t handle, uint8_t addr,
                                  zdt_pid_params_x_t *params) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_pid_params(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 21 TrapezoidKp[4] ThroughKp[4] SpeedKp[4] SpeedKi[4] 6B
  if (response.data_len >= 16) {
    params->trapezoid_pos_kp = read_u32_be(&response.data[0]);
    params->through_pos_kp = read_u32_be(&response.data[4]);
    params->speed_kp = read_u32_be(&response.data[8]);
    params->speed_ki = read_u32_be(&response.data[12]);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

/**
 * @brief 读取PID参数（Emm固件）
 */
zdt_error_t zdt_read_pid_params_emm(zdt_handle_t handle, uint8_t addr,
                                    zdt_pid_params_emm_t *params) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  zdt_build_read_pid_params(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  // 响应格式: Addr 21 Kp[4] Ki[4] Kd[4] 6B
  if (response.data_len >= 12) {
    params->kp = read_u32_be(&response.data[0]);
    params->ki = read_u32_be(&response.data[4]);
    params->kd = read_u32_be(&response.data[8]);
    return ZDT_OK;
  }
  
  return ZDT_ERR_INVALID_RESPONSE;
}

/**
 * @brief 修改位置到达窗口（X42S/Y42）
 */
zdt_error_t zdt_set_position_window(zdt_handle_t handle, uint8_t addr,
                                   uint16_t window, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改位置到达窗口命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_POS_WINDOW;
  cmd.data[2] = ZDT_AUX_SET_POS_WINDOW;
  cmd.data[3] = store ? 0x01 : 0x00;
  write_u16_be(&cmd.data[4], window);
  cmd.data[6] = ZDT_CHECKSUM;
  cmd.len = 7;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

/**
 * @brief 修改过热过流保护检测阈值（X42S/Y42）
 */
zdt_error_t zdt_set_protection_threshold(zdt_handle_t handle, uint8_t addr,
                                        uint16_t temp_threshold,
                                        uint16_t current_threshold,
                                        uint16_t time_threshold, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改保护阈值命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_PROTECT_THRESH;
  cmd.data[2] = ZDT_AUX_SET_PROTECT_THRESH;
  cmd.data[3] = store ? 0x01 : 0x00;
  write_u16_be(&cmd.data[4], temp_threshold);
  write_u16_be(&cmd.data[6], current_threshold);
  write_u16_be(&cmd.data[8], time_threshold);
  cmd.data[10] = ZDT_CHECKSUM;
  cmd.len = 11;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

/**
 * @brief 修改心跳保护功能时间（X42S/Y42）
 */
zdt_error_t zdt_set_heartbeat_time(zdt_handle_t handle, uint8_t addr,
                                   uint32_t time_ms, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改心跳时间命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_HEARTBEAT_TIME;
  cmd.data[2] = ZDT_AUX_SET_HEARTBEAT_TIME;
  cmd.data[3] = store ? 0x01 : 0x00;
  write_u32_be(&cmd.data[4], time_ms);
  cmd.data[8] = ZDT_CHECKSUM;
  cmd.len = 9;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

/**
 * @brief 修改积分限幅/刚性系数（X42S/Y42）
 */
zdt_error_t zdt_set_integral_limit(zdt_handle_t handle, uint8_t addr,
                                   uint32_t limit, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改积分限幅命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_INTEGRAL_LIMIT;
  cmd.data[2] = ZDT_AUX_SET_INTEGRAL_LIMIT;
  cmd.data[3] = store ? 0x01 : 0x00;
  write_u32_be(&cmd.data[4], limit);
  cmd.data[8] = ZDT_CHECKSUM;
  cmd.len = 9;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

/**
 * @brief 修改碰撞回零返回角度（X42S/Y42）
 */
zdt_error_t zdt_set_homing_return_angle(zdt_handle_t handle, uint8_t addr,
                                       uint16_t angle, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改回零返回角度命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_HOMING_ANGLE;
  cmd.data[2] = ZDT_AUX_SET_HOMING_ANGLE;
  cmd.data[3] = store ? 0x01 : 0x00;
  write_u16_be(&cmd.data[4], angle);
  cmd.data[6] = ZDT_CHECKSUM;
  cmd.len = 7;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

/**
 * @brief 修改输入缩小功能（X固件）或速度缩小功能（Emm固件）
 */
zdt_error_t zdt_set_input_scale(zdt_handle_t handle, uint8_t addr,
                               bool enable, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改输入缩小命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_INPUT_SCALE;
  cmd.data[2] = ZDT_AUX_SET_INPUT_SCALE;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = enable ? 0x01 : 0x00;
  cmd.data[5] = ZDT_CHECKSUM;
  cmd.len = 6;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

/**
 * @brief 修改锁定修改参数功能（X42S/Y42）
 */
zdt_error_t zdt_set_param_lock(zdt_handle_t handle, uint8_t addr,
                              uint8_t lock_level, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建修改参数锁定命令
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_PARAM_LOCK;
  cmd.data[2] = ZDT_AUX_SET_PARAM_LOCK;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = lock_level;
  cmd.data[5] = ZDT_CHECKSUM;
  cmd.len = 6;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

/**
 * @brief 存储一组速度参数，上电自动运行（X固件）
 */
zdt_error_t zdt_set_auto_run_params_x(zdt_handle_t handle, uint8_t addr,
                                     zdt_direction_t dir,
                                     uint16_t acceleration,
                                     uint16_t speed,
                                     bool enable_en_pin, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建存储上电自动运行参数命令（X固件）
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_AUTO_RUN_PARAMS;
  cmd.data[2] = ZDT_AUX_SET_AUTO_RUN_PARAMS;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = (uint8_t)dir;
  write_u16_be(&cmd.data[5], acceleration);
  write_u16_be(&cmd.data[7], speed);
  cmd.data[9] = enable_en_pin ? 0x01 : 0x00;
  cmd.data[10] = ZDT_CHECKSUM;
  cmd.len = 11;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

/**
 * @brief 存储一组速度参数，上电自动运行（Emm固件）
 */
zdt_error_t zdt_set_auto_run_params_emm(zdt_handle_t handle, uint8_t addr,
                                       zdt_direction_t dir,
                                       uint16_t speed,
                                       uint8_t acceleration,
                                       bool enable_en_pin, bool store) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建存储上电自动运行参数命令（Emm固件）
  cmd.data[0] = addr;
  cmd.data[1] = ZDT_CMD_SET_AUTO_RUN_PARAMS;
  cmd.data[2] = ZDT_AUX_SET_AUTO_RUN_PARAMS;
  cmd.data[3] = store ? 0x01 : 0x00;
  cmd.data[4] = (uint8_t)dir;
  write_u16_be(&cmd.data[5], speed);
  cmd.data[7] = acceleration;
  cmd.data[8] = enable_en_pin ? 0x01 : 0x00;
  cmd.data[9] = ZDT_CHECKSUM;
  cmd.len = 10;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

/**
 * @brief 读取驱动温度（X42S/Y42）
 */
zdt_error_t zdt_read_temperature(zdt_handle_t handle, uint8_t addr, int8_t *temp_c) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  // 构建读取温度命令
  cmd.data[0] = addr;
  cmd.data[1] = 0x39; // 温度读取命令码
  cmd.data[2] = ZDT_CHECKSUM;
  cmd.len = 3;
  
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;
  
  return zdt_parse_temperature_response(&response, temp_c);
}

/**
 * @brief 构建多电机命令
 */
int zdt_build_multi_motor_command(uint8_t addr, const uint8_t *motor_commands,
                                   size_t total_len, zdt_cmd_buffer_t *buf) {
  if (!buf || !motor_commands || total_len == 0) {
    return ZDT_ERR_INVALID_PARAM;
  }
  
  if (total_len > ZDT_MAX_CMD_SIZE - 4) { // 需要留出地址、功能码、长度、校验码的空间
    return ZDT_ERR_BUFFER_TOO_SMALL;
  }
  
  // 多电机命令格式: Addr | AA | TotalLen[2] | Commands... | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_MULTI_MOTOR;
  write_u16_be(&buf->data[2], (uint16_t)total_len);
  memcpy(&buf->data[4], motor_commands, total_len);
  buf->data[4 + total_len] = ZDT_CHECKSUM;
  buf->len = 4 + total_len + 1;
  
  return (int)buf->len;
}

/**
 * @brief 构建广播读取ID命令
 */
int zdt_build_broadcast_read_id(zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = ZDT_BROADCAST_ADDR;
  buf->data[1] = ZDT_CMD_READ_ID;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

/**
 * @brief 解析系统状态参数响应（X固件，见5.8.1）
 */
int zdt_parse_system_status_response(const zdt_response_t *response,
                                     zdt_system_status_t *status) {
  if (!response || !status) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_SYSTEM_STATUS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 43 25 0C VBus[2] IBus[2] IPha[2] EncRaw[2] EncLin[2] 
  //             S1 PosTarget[4] S2 Speed[2] S3 PosReal[4] S4 PosErr[4] S5 Temp Homing Motor 6B
  if (response->data_len < 37) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  status->bus_voltage = read_u16_be(&response->data[0]);      // 总线电压
  status->bus_current = read_u16_be(&response->data[2]);      // 总线电流
  status->phase_current = read_u16_be(&response->data[4]);    // 相电流
  status->encoder_raw = read_u16_be(&response->data[6]);      // 编码器原始值
  status->encoder_linear = read_u16_be(&response->data[8]);   // 线性化编码器值
  
  uint8_t sign1 = response->data[10];
  status->target_position = read_i32_be(&response->data[11]);
  if (sign1) status->target_position = -status->target_position;
  
  uint8_t sign2 = response->data[15];
  status->speed = read_u16_be(&response->data[16]);
  if (sign2) status->speed = -status->speed;
  
  uint8_t sign3 = response->data[18];
  status->real_position = read_i32_be(&response->data[19]);
  if (sign3) status->real_position = -status->real_position;
  
  uint8_t sign4 = response->data[23];
  status->position_error = read_i32_be(&response->data[24]);
  if (sign4) status->position_error = -status->position_error;
  
  uint8_t sign5 = response->data[28];
  status->temperature = response->data[29];
  if (sign5) status->temperature = -status->temperature;
  
  status->homing_status = response->data[30];
  status->motor_status = response->data[31];

  return ZDT_OK;
}

/**
 * @brief 解析总线电压响应
 */
int zdt_parse_bus_voltage_response(const zdt_response_t *response, uint16_t *voltage_mv) {
  if (!response || !voltage_mv) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_BUS_VOLTAGE) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 24 VBus[2] 6B
  if (response->data_len < 2) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *voltage_mv = read_u16_be(response->data);

  return ZDT_OK;
}

/**
 * @brief 解析总线电流响应（X42S/Y42）
 */
int zdt_parse_bus_current_response(const zdt_response_t *response, uint16_t *current_ma) {
  if (!response || !current_ma) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_BUS_CURRENT) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 26 CBus[2] 6B
  if (response->data_len < 2) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *current_ma = read_u16_be(response->data);

  return ZDT_OK;
}

/**
 * @brief 解析相电流响应
 */
int zdt_parse_phase_current_response(const zdt_response_t *response, uint16_t *current_ma) {
  if (!response || !current_ma) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_PHASE_CURRENT) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 27 CPha[2] 6B
  if (response->data_len < 2) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *current_ma = read_u16_be(response->data);

  return ZDT_OK;
}

/**
 * @brief 解析电池电压响应（Y42）
 */
int zdt_parse_battery_voltage_response(const zdt_response_t *response, uint16_t *voltage_mv) {
  if (!response || !voltage_mv) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_BATTERY_VOLTAGE) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 38 VBat[2] 6B
  if (response->data_len < 2) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *voltage_mv = read_u16_be(response->data);

  return ZDT_OK;
}

/**
 * @brief 解析输入脉冲数响应
 */
int zdt_parse_input_pulses_response(const zdt_response_t *response, int32_t *pulses) {
  if (!response || !pulses) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_INPUT_PULSES) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 32 Sign Pulses[4] 6B
  if (response->data_len < 5) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  uint8_t sign = response->data[0];
  *pulses = read_i32_be(&response->data[1]);
  if (sign) *pulses = -*pulses;

  return ZDT_OK;
}

/**
 * @brief 解析设定目标位置响应
 */
int zdt_parse_set_target_pos_response(const zdt_response_t *response, zdt_position_t *position) {
  if (!response || !position) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_SET_TARGET_POS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 34 Sign Pos[4] 6B
  if (response->data_len < 5) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  uint8_t sign = response->data[0];
  int32_t pos = read_i32_be(&response->data[1]);
  position->angle_raw = (sign == 0x01) ? -pos : pos;

  // X固件: 角度值 = 目标位置/10
  position->angle_deg = position->angle_raw / 10.0f;

  return ZDT_OK;
}

/**
 * @brief 解析位置误差响应
 */
int zdt_parse_position_error_response(const zdt_response_t *response, zdt_position_t *error) {
  if (!response || !error) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_POSITION_ERROR) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 37 Sign Err[4] 6B
  if (response->data_len < 5) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  uint8_t sign = response->data[0];
  int32_t err = read_i32_be(&response->data[1]);
  error->angle_raw = (sign == 0x01) ? -err : err;

  // X固件: 角度值 = 位置误差/100
  error->angle_deg = error->angle_raw / 100.0f;

  return ZDT_OK;
}

/**
 * @brief 解析IO电平状态响应（X42S/Y42）
 */
int zdt_parse_io_levels_response(const zdt_response_t *response, uint8_t *io_levels) {
  if (!response || !io_levels) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_IO_LEVELS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 3D IO_Levels 6B
  if (response->data_len < 1) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *io_levels = response->data[0];

  return ZDT_OK;
}

/**
 * @brief 解析选项参数状态响应（X42S/Y42）
 */
int zdt_parse_options_response(const zdt_response_t *response, uint8_t *options) {
  if (!response || !options) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_OPTIONS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 1A Options 6B
  if (response->data_len < 1) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *options = response->data[0];

  return ZDT_OK;
}

/**
 * @brief 解析位置到达窗口响应（X42S/Y42）
 */
int zdt_parse_pos_window_response(const zdt_response_t *response, uint16_t *window) {
  if (!response || !window) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_POS_WINDOW) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 41 Window[2] 6B
  if (response->data_len < 2) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *window = read_u16_be(response->data);

  return ZDT_OK;
}

/**
 * @brief 解析过热过流保护检测阈值响应（X42S/Y42）
 */
int zdt_parse_protection_threshold_response(const zdt_response_t *response,
                                          uint16_t *temp_threshold,
                                          uint16_t *current_threshold,
                                          uint16_t *time_threshold) {
  if (!response || !temp_threshold || !current_threshold || !time_threshold) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_PROTECT_THRESH) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 13 Temp[2] Current[2] Time[2] 6B
  if (response->data_len < 6) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *temp_threshold = read_u16_be(&response->data[0]);
  *current_threshold = read_u16_be(&response->data[2]);
  *time_threshold = read_u16_be(&response->data[4]);

  return ZDT_OK;
}

/**
 * @brief 解析心跳保护功能时间响应（X42S/Y42）
 */
int zdt_parse_heartbeat_time_response(const zdt_response_t *response, uint32_t *time_ms) {
  if (!response || !time_ms) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_HEARTBEAT_TIME) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 16 Time[4] 6B
  if (response->data_len < 4) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *time_ms = read_u32_be(response->data);

  return ZDT_OK;
}

/**
 * @brief 解析积分限幅/刚性系数响应（X42S/Y42）
 */
int zdt_parse_integral_limit_response(const zdt_response_t *response, uint32_t *limit) {
  if (!response || !limit) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_INTEGRAL_LIMIT) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 23 Limit[4] 6B
  if (response->data_len < 4) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *limit = read_u32_be(response->data);

  return ZDT_OK;
}

/**
 * @brief 解析碰撞回零返回角度响应（X42S/Y42）
 */
int zdt_parse_homing_angle_response(const zdt_response_t *response, uint16_t *angle) {
  if (!response || !angle) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_HOMING_ANGLE) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 3F Angle[2] 6B
  if (response->data_len < 2) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *angle = read_u16_be(response->data);

  return ZDT_OK;
}

/**
 * @brief 解析PID参数响应（X固件）
 */
int zdt_parse_pid_params_x_response(const zdt_response_t *response, zdt_pid_params_x_t *params) {
  if (!response || !params) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_PID_PARAMS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 21 TrapezoidKp[4] ThroughKp[4] SpeedKp[4] SpeedKi[4] 6B
  if (response->data_len < 16) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  params->trapezoid_pos_kp = read_u32_be(&response->data[0]);
  params->through_pos_kp = read_u32_be(&response->data[4]);
  params->speed_kp = read_u32_be(&response->data[8]);
  params->speed_ki = read_u32_be(&response->data[12]);

  return ZDT_OK;
}

/**
 * @brief 解析PID参数响应（Emm固件）
 */
int zdt_parse_pid_params_emm_response(const zdt_response_t *response, zdt_pid_params_emm_t *params) {
  if (!response || !params) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_PID_PARAMS) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 21 Kp[4] Ki[4] Kd[4] 6B
  if (response->data_len < 12) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  params->kp = read_u32_be(&response->data[0]);
  params->ki = read_u32_be(&response->data[4]);
  params->kd = read_u32_be(&response->data[8]);

  return ZDT_OK;
}

/**
 * @brief 发送多电机命令
 */
zdt_error_t zdt_send_multi_motor_command(zdt_handle_t handle, uint8_t addr,
                                        const uint8_t *motor_commands,
                                        size_t total_len) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  
  int len = zdt_build_multi_motor_command(addr, motor_commands, total_len, &cmd);
  if (len < 0)
    return (zdt_error_t)len;
  
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

#endif /* CONFIG_IDF_TARGET */
