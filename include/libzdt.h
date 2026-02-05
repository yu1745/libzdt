/**
 * @file libzdt.h
 * @brief ZDT X42S 电机驱动库 - CAN协议实现
 *
 * 本库实现了 ZDT EMM 固件的 CAN 通讯协议，校验码固定为 0x6B。
 * 适用于 ZDT X42S/Y42 系列闭环步进电机。
 *
 * @note 这是一个 ESP-IDF 组件库
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef CONFIG_IDF_TARGET
/* CAN驱动抽象接口，用于高级API */
#include "zdt_can_driver.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 常量定义
 * ========================================================================== */

#define ZDT_CHECKSUM            0x6B    ///< 固定校验码
#define ZDT_BROADCAST_ADDR      0x00    ///< 广播地址
#define ZDT_MAX_CMD_SIZE        64      ///< 最大命令长度
#define ZDT_CAN_MAX_DATA_LEN    8       ///< CAN帧最大数据长度

/* ============================================================================
 * 功能码定义
 * ========================================================================== */

/** 触发动作命令功能码 */
typedef enum {
    ZDT_CMD_ENCODER_CALIBRATION = 0x06, ///< 编码器校准
    ZDT_CMD_RESTART             = 0x08, ///< 重启电机
    ZDT_CMD_CLEAR_POSITION      = 0x0A, ///< 位置清零
    ZDT_CMD_CLEAR_PROTECTION    = 0x0E, ///< 解除保护
    ZDT_CMD_FACTORY_RESET       = 0x0F, ///< 恢复出厂
} zdt_action_cmd_t;

/** 运动控制命令功能码 */
typedef enum {
    ZDT_CMD_MULTI_MOTOR         = 0xAA, ///< 多电机命令
    ZDT_CMD_ENABLE              = 0xF3, ///< 使能控制
    ZDT_CMD_SPEED_MODE          = 0xF6, ///< 速度模式
    ZDT_CMD_POSITION_MODE       = 0xFD, ///< 位置模式
    ZDT_CMD_HOMING              = 0x9A, ///< 回零运动
    ZDT_CMD_EMERGENCY_STOP      = 0xFE, ///< 电机急停
} zdt_motion_cmd_t;

/** 读取参数命令功能码 */
typedef enum {
    ZDT_CMD_TIMED_RETURN        = 0x11, ///< 定时返回信息命令（X42S/Y42，见5.5.1）
    ZDT_CMD_READ_POSITION       = 0x36, ///< 读取位置角度
    ZDT_CMD_READ_PULSES         = 0x37, ///< 读取脉冲数
    ZDT_CMD_READ_IO_STATUS      = 0x39, ///< 读取IO状态
    ZDT_CMD_READ_STATUS_FLAGS   = 0x3A, ///< 读取状态标志
    ZDT_CMD_READ_CONFIG         = 0x42, ///< 读取配置参数
} zdt_read_cmd_t;

/** 修改参数命令功能码 */
typedef enum {
    ZDT_CMD_WRITE_CONFIG        = 0x48, ///< 修改配置参数
} zdt_write_cmd_t;

/* ============================================================================
 * 辅助码定义
 * ========================================================================== */

#define ZDT_AUX_ENCODER_CAL     0x45    ///< 编码器校准辅助码
#define ZDT_AUX_RESTART         0x97    ///< 重启辅助码
#define ZDT_AUX_CLEAR_POS       0x6D    ///< 位置清零辅助码
#define ZDT_AUX_CLEAR_PROT      0x52    ///< 解除保护辅助码
#define ZDT_AUX_FACTORY_RESET   0x5F    ///< 恢复出厂辅助码
#define ZDT_AUX_ENABLE          0xAB    ///< 使能控制辅助码
#define ZDT_AUX_EMERGENCY_STOP  0x98    ///< 急停辅助码
#define ZDT_AUX_TIMED_RETURN    0x18    ///< 定时返回信息辅助码（5.5.1）

/* ============================================================================
 * 返回状态码
 * ========================================================================== */

/** 电机返回的状态码 */
typedef enum {
    ZDT_RESP_OK             = 0x02, ///< 命令正确
    ZDT_RESP_AT_ZERO        = 0x12, ///< 已在零点
    ZDT_RESP_PARAM_ERROR    = 0xE2, ///< 参数错误
    ZDT_RESP_FORMAT_ERROR   = 0xEE, ///< 格式错误
    ZDT_RESP_ACTION_DONE    = 0x9F, ///< 动作完成
} zdt_response_status_t;

/* ============================================================================
 * 枚举类型
 * ========================================================================== */

/** 旋转方向 */
typedef enum {
    ZDT_DIR_CW  = 0x00, ///< 顺时针
    ZDT_DIR_CCW = 0x01, ///< 逆时针
} zdt_direction_t;

/** 同步标志 */
typedef enum {
    ZDT_SYNC_IMMEDIATE      = 0x00, ///< 立即执行
    ZDT_SYNC_CACHE          = 0x01, ///< 先缓存
} zdt_sync_flag_t;

/** 使能状态 */
typedef enum {
    ZDT_ENABLE_OFF  = 0x00, ///< 不使能(松轴)
    ZDT_ENABLE_ON   = 0x01, ///< 使能(锁轴)
} zdt_enable_state_t;

/** 运动模式（位置模式） */
typedef enum {
    ZDT_MOTION_RELATIVE = 0x00, ///< 相对位置运动
    ZDT_MOTION_ABSOLUTE = 0x01, ///< 绝对位置运动
} zdt_motion_mode_t;

/** 库返回错误码 */
typedef enum {
    ZDT_OK                  = 0,    ///< 成功
    ZDT_ERR_INVALID_PARAM   = -1,   ///< 无效参数
    ZDT_ERR_BUFFER_TOO_SMALL= -2,   ///< 缓冲区太小
    ZDT_ERR_CAN_TX_FAILED   = -3,   ///< CAN发送失败
    ZDT_ERR_CAN_RX_TIMEOUT  = -4,   ///< CAN接收超时
    ZDT_ERR_INVALID_RESPONSE= -5,   ///< 无效响应
    ZDT_ERR_MOTOR_ERROR     = -6,   ///< 电机返回错误
    ZDT_ERR_NOT_INITIALIZED = -7,   ///< 未初始化
} zdt_error_t;

/* ============================================================================
 * 数据结构
 * ========================================================================== */

/** CAN消息结构 */
typedef struct {
    uint32_t id;                    ///< 扩展帧ID = (addr << 8) | packet
    uint8_t  data[8];               ///< 数据
    uint8_t  data_len;              ///< 数据长度 (0-8)
    bool     is_extended;           ///< 是否为扩展帧 (ZDT固定使用扩展帧)
} zdt_can_msg_t;

/** 命令缓冲区结构 */
typedef struct {
    uint8_t  data[ZDT_MAX_CMD_SIZE]; ///< 命令数据
    size_t   len;                    ///< 数据长度
} zdt_cmd_buffer_t;

/** 电机响应结构 */
typedef struct {
    uint8_t  addr;                  ///< 电机地址
    uint8_t  func_code;             ///< 功能码
    uint8_t  status;                ///< 状态码
    uint8_t  data[ZDT_MAX_CMD_SIZE];///< 额外数据
    size_t   data_len;              ///< 数据长度
} zdt_response_t;

/** 位置数据结构 */
typedef struct {
    int32_t  angle_raw;             ///< 原始角度值 (单位 0.1°)
    float    angle_deg;             ///< 角度值 (度)
} zdt_position_t;

/** CAN通讯配置 */
typedef struct {
    int      tx_gpio;               ///< CAN TX GPIO
    int      rx_gpio;               ///< CAN RX GPIO
    uint32_t bitrate;               ///< 波特率 (默认500000)
    uint32_t tx_timeout_ms;         ///< 发送超时 (ms)
    uint32_t rx_timeout_ms;         ///< 接收超时 (ms)
} zdt_can_config_t;

/** ZDT句柄 */
typedef struct zdt_handle_s* zdt_handle_t;

/* ============================================================================
 * 协议层API - 命令构建
 * ========================================================================== */

/**
 * @brief 构建编码器校准命令
 * @param addr 电机地址 (1-255)
 * @param buf 输出缓冲区
 * @return 命令长度，失败返回负数错误码
 */
int zdt_build_encoder_calibration(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建重启电机命令
 */
int zdt_build_restart(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建位置清零命令
 */
int zdt_build_clear_position(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建解除保护命令
 */
int zdt_build_clear_protection(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建恢复出厂设置命令
 */
int zdt_build_factory_reset(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建使能控制命令
 * @param addr 电机地址
 * @param enable 使能状态
 * @param sync 同步标志
 * @param buf 输出缓冲区
 */
int zdt_build_enable(uint8_t addr, zdt_enable_state_t enable, 
                     zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建回零运动命令
 * @param addr 电机地址
 * @param mode 回零模式
 * @param sync 同步标志
 * @param buf 输出缓冲区
 */
int zdt_build_homing(uint8_t addr, uint8_t mode, 
                     zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建急停命令
 * @param addr 电机地址
 * @param sync 同步标志
 * @param buf 输出缓冲区
 */
int zdt_build_emergency_stop(uint8_t addr, zdt_sync_flag_t sync, 
                             zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取位置角度命令
 */
int zdt_build_read_position(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取脉冲数命令
 */
int zdt_build_read_pulses(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取IO状态命令
 */
int zdt_build_read_io_status(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取状态标志命令
 */
int zdt_build_read_status_flags(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取配置参数命令
 */
int zdt_build_read_config(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建定时返回信息命令（5.5.1，X42S/Y42）
 *
 * 使电机按指定周期定时返回“读取系统参数”中某命令的数据，无需主机频繁轮询。
 * 定时时间为 0 时停止该信息类型的定时返回。
 *
 * @param addr 电机地址
 * @param info_func_code 要定时返回的读取命令功能码（如 0x36=位置、0x3A=状态标志等，见5.5节）
 * @param interval_ms 定时周期（毫秒），0 表示停止定时返回
 * @param buf 输出缓冲区
 * @return 命令长度(7)，失败返回负数错误码
 */
int zdt_build_timed_return(uint8_t addr, uint8_t info_func_code,
                           uint16_t interval_ms, zdt_cmd_buffer_t *buf);

/* ============================================================================
 * EMM固件运动控制命令构建
 * ========================================================================== */

/**
 * @brief 构建速度模式命令 (EMM固件)
 * 
 * 命令格式: 地址 | 0xF6 | 方向 | 速度[2] | 加速度 | 同步标志 | 校验码
 * 
 * @param addr 电机地址
 * @param dir 旋转方向 (0=CW, 1=CCW)
 * @param speed_rpm 速度 (0-3000 RPM)
 * @param acc 加速度档位 (0-255, 0=直接启动)
 * @param sync 同步标志
 * @param buf 输出缓冲区
 * @return 命令长度(8)，失败返回负数错误码
 */
int zdt_build_speed_mode(uint8_t addr, zdt_direction_t dir,
                         uint16_t speed_rpm, uint8_t acc,
                         zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建位置模式命令 (EMM固件)
 * 
 * 命令格式: 地址 | 0xFD | 方向 | 速度[2] | 加速度 | 位置[4] | 相对/绝对 | 同步标志 | 校验码
 * 
 * @param addr 电机地址
 * @param dir 旋转方向 (0=CW, 1=CCW)
 * @param speed_rpm 速度 (0-3000 RPM)
 * @param acc 加速度档位 (0-255)
 * @param position_01deg 位置角度 (0.1° 单位)
 * @param motion_mode 运动模式 (0=相对, 1=绝对)
 * @param sync 同步标志 (0=立即执行, 1=缓存等待同步命令一起执行)
 * @param buf 输出缓冲区
 * @return 命令长度(13)，失败返回负数错误码
 */
int zdt_build_position_mode(uint8_t addr, zdt_direction_t dir,
                            uint16_t speed_rpm, uint8_t acc,
                            int32_t position_01deg,
                            zdt_motion_mode_t motion_mode, 
                            zdt_sync_flag_t sync,
                            zdt_cmd_buffer_t *buf);

/* ============================================================================
 * CAN协议层API
 * ========================================================================== */

/**
 * @brief 将命令缓冲区转换为CAN消息（处理拆包）
 * @param addr 电机地址
 * @param cmd 命令缓冲区
 * @param msgs 输出CAN消息数组
 * @param max_msgs 最大消息数量
 * @return 实际消息数量，失败返回负数错误码
 */
int zdt_cmd_to_can_msgs(uint8_t addr, const zdt_cmd_buffer_t *cmd,
                        zdt_can_msg_t *msgs, size_t max_msgs);

/**
 * @brief 从CAN消息重组响应（处理组包）
 * @param msgs CAN消息数组
 * @param msg_count 消息数量
 * @param response 输出响应结构
 * @return ZDT_OK成功，失败返回负数错误码
 */
int zdt_can_msgs_to_response(const zdt_can_msg_t *msgs, size_t msg_count,
                             zdt_response_t *response);

/**
 * @brief 生成CAN扩展帧ID
 * @param addr 电机地址
 * @param packet 包序号
 * @return 扩展帧ID
 */
static inline uint32_t zdt_make_can_id(uint8_t addr, uint8_t packet) {
    return ((uint32_t)addr << 8) | packet;
}

/**
 * @brief 从CAN扩展帧ID解析地址和包序号
 * @param id 扩展帧ID
 * @param addr 输出电机地址
 * @param packet 输出包序号
 */
static inline void zdt_parse_can_id(uint32_t id, uint8_t *addr, uint8_t *packet) {
    if (addr) *addr = (id >> 8) & 0xFF;
    if (packet) *packet = id & 0xFF;
}

/* ============================================================================
 * 响应解析API
 * ========================================================================== */

/**
 * @brief 解析位置响应
 * @param response 响应结构
 * @param position 输出位置数据
 * @return ZDT_OK成功
 */
int zdt_parse_position_response(const zdt_response_t *response, zdt_position_t *position);

/**
 * @brief 解析脉冲数响应
 * @param response 响应结构
 * @param pulses 输出脉冲数
 * @return ZDT_OK成功
 */
int zdt_parse_pulses_response(const zdt_response_t *response, int32_t *pulses);

/**
 * @brief 检查响应状态是否成功
 * @param response 响应结构
 * @return true=成功, false=有错误
 */
bool zdt_response_is_ok(const zdt_response_t *response);

/**
 * @brief 获取响应状态的字符串描述
 * @param status 状态码
 * @return 状态描述字符串
 */
const char* zdt_response_status_str(uint8_t status);

/* ============================================================================
 * 高级封装API (需要CAN驱动实现)
 * ========================================================================== */

#ifdef CONFIG_IDF_TARGET

/**
 * @brief 初始化ZDT CAN通讯
 * @param config CAN配置
 * @param driver CAN驱动实现（用户需要提供）
 * @param handle 输出句柄
 * @return ZDT_OK成功
 *
 * @note 用户需要提供CAN驱动实现，可以是TWAI或其他自定义驱动
 * @see zdt_can_driver.h
 */
zdt_error_t zdt_can_init(const zdt_can_config_t *config,
                         const zdt_can_driver_t *driver,
                         zdt_handle_t *handle);

/**
 * @brief 反初始化ZDT CAN通讯
 * @param handle 句柄
 */
void zdt_can_deinit(zdt_handle_t handle);

/**
 * @brief 发送命令并等待响应
 * @param handle 句柄
 * @param addr 电机地址
 * @param cmd 命令缓冲区
 * @param response 输出响应（可为NULL表示不需要响应）
 * @return ZDT_OK成功
 */
zdt_error_t zdt_can_send_cmd(zdt_handle_t handle, uint8_t addr,
                             const zdt_cmd_buffer_t *cmd, zdt_response_t *response);

/* 便捷函数 */

/**
 * @brief 编码器校准
 */
zdt_error_t zdt_encoder_calibration(zdt_handle_t handle, uint8_t addr);

/**
 * @brief 重启电机
 */
zdt_error_t zdt_restart(zdt_handle_t handle, uint8_t addr);

/**
 * @brief 位置清零
 */
zdt_error_t zdt_clear_position(zdt_handle_t handle, uint8_t addr);

/**
 * @brief 解除保护
 */
zdt_error_t zdt_clear_protection(zdt_handle_t handle, uint8_t addr);

/**
 * @brief 恢复出厂设置
 */
zdt_error_t zdt_factory_reset(zdt_handle_t handle, uint8_t addr);

/**
 * @brief 使能/禁用电机
 */
zdt_error_t zdt_set_enable(zdt_handle_t handle, uint8_t addr, bool enable);

/**
 * @brief 回零运动
 */
zdt_error_t zdt_homing(zdt_handle_t handle, uint8_t addr, uint8_t mode);

/**
 * @brief 急停
 */
zdt_error_t zdt_emergency_stop(zdt_handle_t handle, uint8_t addr);

/**
 * @brief 读取当前位置
 */
zdt_error_t zdt_read_position(zdt_handle_t handle, uint8_t addr, zdt_position_t *position);

/**
 * @brief 读取脉冲数
 */
zdt_error_t zdt_read_pulses(zdt_handle_t handle, uint8_t addr, int32_t *pulses);

/**
 * @brief 设置/取消定时返回信息（5.5.1，X42S/Y42）
 * @param info_func_code 要定时返回的读取命令功能码（如 ZDT_CMD_READ_POSITION、ZDT_CMD_READ_STATUS_FLAGS）
 * @param interval_ms 周期毫秒，0 表示停止定时返回
 */
zdt_error_t zdt_set_timed_return(zdt_handle_t handle, uint8_t addr,
                                 uint8_t info_func_code, uint16_t interval_ms);

/**
 * @brief 设置速度模式 (EMM固件)
 * @param speed_rpm 速度 (0-3000 RPM)
 * @param acc 加速度档位 (0-255)
 */
zdt_error_t zdt_set_speed(zdt_handle_t handle, uint8_t addr,
                          zdt_direction_t dir, uint16_t speed_rpm, uint8_t acc);

/**
 * @brief 设置位置模式 (EMM固件)
 * @param speed_rpm 速度 (0-3000 RPM)
 * @param angle_deg 角度 (度)
 * @param acc 加速度档位 (0-255)
 * @param mode 运动模式
 * @param sync_cache 是否使用同步缓存模式 (true=缓存等待同步命令, false=立即执行)
 */
zdt_error_t zdt_set_position(zdt_handle_t handle, uint8_t addr,
                             zdt_direction_t dir, float speed_rpm, float angle_deg,
                             zdt_motion_mode_t mode, bool sync_cache);

#endif /* CONFIG_IDF_TARGET */

#ifdef __cplusplus
}
#endif
