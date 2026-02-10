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
    ZDT_CMD_TORQUE_MODE         = 0xF5, ///< 力矩模式控制（X）
    ZDT_CMD_SPEED_MODE          = 0xF6, ///< 速度模式
    ZDT_CMD_POSITION_MODE_EMM   = 0xFD, ///< 位置模式（Emm固件，见5.3.12）
    ZDT_CMD_TORQUE_MODE_LIMIT   = 0xC5, ///< 力矩模式限速控制（X）
    ZDT_CMD_SPEED_MODE_LIMIT    = 0xC6, ///< 速度模式限电流控制（X）
    ZDT_CMD_POSITION_THROUGH    = 0xFB, ///< 直通限速位置模式控制（X，见5.3.8）
    ZDT_CMD_POSITION_THROUGH_LIMIT = 0xCB, ///< 直通限速位置模式限电流控制（X，见5.3.9）
    ZDT_CMD_POSITION_TRAPEZOID  = 0xFD, ///< 梯形曲线加减速位置模式控制（X，见5.3.10）
    ZDT_CMD_POSITION_TRAPEZOID_LIMIT = 0xCD, ///< 梯形曲线加减速位置模式限电流控制（X，见5.3.11）
    ZDT_CMD_HOMING              = 0x9A, ///< 回零运动
    ZDT_CMD_EMERGENCY_STOP      = 0xFE, ///< 电机急停
} zdt_motion_cmd_t;

/** 读取参数命令功能码 */
typedef enum {
    ZDT_CMD_TIMED_RETURN        = 0x11, ///< 定时返回信息命令（X42S/Y42，见5.5.1）
    ZDT_CMD_READ_VERSION        = 0x1F, ///< 读取固件版本和硬件版本
    ZDT_CMD_READ_PHASE_RL       = 0x20, ///< 读取相电阻和相电感
    ZDT_CMD_READ_BUS_VOLTAGE    = 0x24, ///< 读取总线电压
    ZDT_CMD_READ_BUS_CURRENT    = 0x26, ///< 读取总线电流（X42S/Y42）
    ZDT_CMD_READ_PHASE_CURRENT  = 0x27, ///< 读取相电流
    ZDT_CMD_READ_BATTERY_VOLTAGE= 0x38, ///< 读取电池电压（Y42）
    ZDT_CMD_READ_ENCODER_LINEAR = 0x31, ///< 读取经过线性化校准后的编码器值
    ZDT_CMD_READ_INPUT_PULSES   = 0x32, ///< 读取输入脉冲数
    ZDT_CMD_READ_TARGET_POS     = 0x33, ///< 读取电机目标位置
    ZDT_CMD_READ_SET_TARGET_POS = 0x34, ///< 读取电机实时设定的目标位置
    ZDT_CMD_READ_SPEED          = 0x35, ///< 读取电机实时转速
    ZDT_CMD_READ_POSITION       = 0x36, ///< 读取位置角度
    ZDT_CMD_READ_POSITION_ERROR = 0x37, ///< 读取电机位置误差
    ZDT_CMD_READ_TEMPERATURE    = 0x39, ///< 读取驱动温度（X42S/Y42，见5.5.12）
    ZDT_CMD_READ_STATUS_FLAGS   = 0x3A, ///< 读取状态标志
    ZDT_CMD_READ_HOMING_STATUS  = 0x3B, ///< 读取回零状态标志
    ZDT_CMD_READ_BOTH_STATUS    = 0x3C, ///< 读取回零状态标志+电机状态标志（X42S/Y42）
    ZDT_CMD_READ_IO_LEVELS      = 0x3D, ///< 读取引脚IO电平状态（X42S/Y42）
    ZDT_CMD_READ_CONFIG         = 0x42, ///< 读取配置参数
    ZDT_CMD_READ_PID_PARAMS     = 0x21, ///< 读取PID参数
    ZDT_CMD_READ_HOMING_PARAMS  = 0x22, ///< 读取回零参数
    ZDT_CMD_READ_OPTIONS        = 0x1A, ///< 读取选项参数状态（X42S/Y42）
    ZDT_CMD_READ_POS_WINDOW     = 0x41, ///< 读取位置到达窗口（X42S/Y42）
    ZDT_CMD_READ_PROTECT_THRESH = 0x13, ///< 读取过热过流保护检测阈值（X42S/Y42）
    ZDT_CMD_READ_HEARTBEAT_TIME = 0x16, ///< 读取心跳保护功能时间（X42S/Y42）
    ZDT_CMD_READ_INTEGRAL_LIMIT = 0x23, ///< 读取积分限幅/刚性系数（X42S/Y42）
    ZDT_CMD_READ_HOMING_ANGLE   = 0x3F, ///< 读取碰撞回零返回角度（X42S/Y42）
    ZDT_CMD_READ_ID             = 0x15, ///< 广播读取ID地址（X42S/Y42）
    ZDT_CMD_READ_SYSTEM_STATUS  = 0x43, ///< 读取系统状态参数（X固件，见5.8.1）
} zdt_read_cmd_t;

/** 修改参数命令功能码 */
typedef enum {
    ZDT_CMD_WRITE_CONFIG        = 0x48, ///< 修改配置参数
    ZDT_CMD_SET_HOMING_ZERO     = 0x93, ///< 设置单圈回零的零点位置
    ZDT_CMD_EXIT_HOMING         = 0x9C, ///< 强制中断并退出回零操作
    ZDT_CMD_SET_MOTOR_ID        = 0xAE, ///< 修改电机ID/地址
    ZDT_CMD_SET_SUBDIVISION     = 0x84, ///< 修改细分值
    ZDT_CMD_SET_POWER_LOSS_FLAG = 0x50, ///< 修改掉电标志
    ZDT_CMD_SET_MOTOR_TYPE      = 0xD7, ///< 修改电机类型
    ZDT_CMD_SET_FIRMWARE_TYPE   = 0xD5, ///< 修改固件类型
    ZDT_CMD_SET_CONTROL_MODE    = 0x46, ///< 修改开环/闭环控制模式
    ZDT_CMD_SET_MOTOR_DIR       = 0xD4, ///< 修改电机运动正方向
    ZDT_CMD_SET_KEY_LOCK        = 0xD0, ///< 修改锁定按键功能
    ZDT_CMD_SET_INPUT_SCALE     = 0x4F, ///< 修改命令位置角度是否继续缩小10倍输入
    ZDT_CMD_SET_OPEN_LOOP_CURRENT = 0x44, ///< 修改开环模式工作电流
    ZDT_CMD_SET_CLOSED_LOOP_CURRENT = 0x45, ///< 修改闭环模式最大电流
    ZDT_CMD_SET_PID_PARAMS      = 0x4A, ///< 修改PID参数
    ZDT_CMD_SET_HOMING_PARAMS   = 0x4C, ///< 修改回零参数
    ZDT_CMD_SET_POS_WINDOW      = 0xD1, ///< 修改位置到达窗口（X42S/Y42）
    ZDT_CMD_SET_PROTECT_THRESH  = 0xD3, ///< 修改过热过流保护检测阈值（X42S/Y42）
    ZDT_CMD_SET_HEARTBEAT_TIME  = 0x68, ///< 修改心跳保护功能时间（X42S/Y42）
    ZDT_CMD_SET_INTEGRAL_LIMIT  = 0x4B, ///< 修改积分限幅/刚性系数（X42S/Y42）
    ZDT_CMD_SET_HOMING_ANGLE    = 0x5C, ///< 修改碰撞回零返回角度（X42S/Y42）
    ZDT_CMD_SET_PARAM_LOCK      = 0xD6, ///< 修改锁定修改参数功能（X42S/Y42）
    ZDT_CMD_SET_AUTO_RUN_PARAMS = 0xF7, ///< 存储一组速度参数，上电自动运行
    ZDT_CMD_SYNC_TRIGGER        = 0xFF, ///< 触发多机同步运动
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
#define ZDT_AUX_SET_HOMING_ZERO 0x88    ///< 设置单圈回零零点位置辅助码
#define ZDT_AUX_EXIT_HOMING     0x48    ///< 强制中断并退出回零操作辅助码
#define ZDT_AUX_SET_MOTOR_ID    0x02    ///< 修改电机ID辅助码（示例）
#define ZDT_AUX_SET_SUBDIVISION 0x8A    ///< 修改细分值辅助码
#define ZDT_AUX_SET_MOTOR_TYPE  0x35    ///< 修改电机类型辅助码
#define ZDT_AUX_SET_FIRMWARE_TYPE 0x69  ///< 修改固件类型辅助码
#define ZDT_AUX_SET_CONTROL_MODE 0xA6  ///< 修改控制模式辅助码
#define ZDT_AUX_SET_MOTOR_DIR   0x60    ///< 修改电机方向辅助码
#define ZDT_AUX_SET_KEY_LOCK    0xB3    ///< 修改锁定按键辅助码
#define ZDT_AUX_SET_INPUT_SCALE 0x71    ///< 修改输入缩小辅助码
#define ZDT_AUX_SET_OPEN_LOOP_CURRENT 0x33 ///< 修改开环电流辅助码
#define ZDT_AUX_SET_CLOSED_LOOP_CURRENT 0x66 ///< 修改闭环电流辅助码
#define ZDT_AUX_SET_PID_PARAMS  0xC3    ///< 修改PID参数辅助码
#define ZDT_AUX_SET_HOMING_PARAMS 0xAE  ///< 修改回零参数辅助码
#define ZDT_AUX_SET_POS_WINDOW  0x07    ///< 修改位置到达窗口辅助码
#define ZDT_AUX_SET_PROTECT_THRESH 0x56 ///< 修改保护阈值辅助码
#define ZDT_AUX_SET_HEARTBEAT_TIME 0x38 ///< 修改心跳时间辅助码
#define ZDT_AUX_SET_INTEGRAL_LIMIT 0x57 ///< 修改积分限幅辅助码
#define ZDT_AUX_SET_HOMING_ANGLE 0xAC  ///< 修改回零返回角度辅助码
#define ZDT_AUX_SET_PARAM_LOCK   0x4B   ///< 修改参数锁定辅助码
#define ZDT_AUX_SET_AUTO_RUN_PARAMS 0x1C ///< 存储上电自动运行参数辅助码
#define ZDT_AUX_SYNC_TRIGGER     0x66    ///< 触发多机同步运动辅助码

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
    ZDT_MOTION_RELATIVE_INPUT  = 0x00, ///< 相对上一输入目标位置进行相对位置运动
    ZDT_MOTION_ABSOLUTE_ZERO   = 0x01, ///< 相对坐标零点进行绝对位置运动
    ZDT_MOTION_RELATIVE_REALTIME = 0x02, ///< 相对当前实时位置进行相对位置运动
} zdt_motion_mode_t;

/** 回零模式 */
typedef enum {
    ZDT_HOMING_NEAREST         = 0x00, ///< 单圈就近回零
    ZDT_HOMING_DIRECTION       = 0x01, ///< 单圈方向回零
    ZDT_HOMING_COLLISION       = 0x02, ///< 无限位碰撞回零
    ZDT_HOMING_LIMIT_SWITCH    = 0x03, ///< 限位回零
    ZDT_HOMING_ABSOLUTE_ZERO   = 0x04, ///< 回到绝对位置坐标零点
    ZDT_HOMING_POWER_LOSS_POS  = 0x05, ///< 回到上次掉电位置角度
} zdt_homing_mode_t;

/** 电机类型 */
typedef enum {
    ZDT_MOTOR_TYPE_0_9_DEG     = 25,   ///< 0.9°步进电机 (协议值=25)
    ZDT_MOTOR_TYPE_1_8_DEG     = 50,   ///< 1.8°步进电机 (协议值=50)
} zdt_motor_type_t;

/** 固件类型 */
typedef enum {
    ZDT_FIRMWARE_X             = 0x00, ///< X固件
    ZDT_FIRMWARE_EMM           = 0x01, ///< Emm固件
    ZDT_FIRMWARE_EMM_RAGE      = 0x02, ///< Emm固件狂暴模式
} zdt_firmware_type_t;

/** 控制模式 */
typedef enum {
    ZDT_CONTROL_OPEN_LOOP      = 0x00, ///< 开环模式
    ZDT_CONTROL_CLOSED_LOOP    = 0x01, ///< 闭环模式
} zdt_control_mode_t;

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

/** 版本信息结构 */
typedef struct {
    uint16_t firmware_version;      ///< 固件版本号
    uint8_t  hardware_series;       ///< 硬件板卡系列 (0=X系列, 1=Y系列)
    uint8_t  hardware_type;         ///< 硬件板卡类型 (0=20, 1=28, 2=35, 3=42, 4=57, 5=86)
    uint8_t  hardware_version;      ///< 硬件版本号
} zdt_version_info_t;

/** 相电阻相电感结构 */
typedef struct {
    uint16_t phase_resistance;      ///< 相电阻 (mΩ)
    uint16_t phase_inductance;      ///< 相电感 (uH)
} zdt_phase_rl_t;

/** 回零状态标志结构 */
typedef struct {
    bool     ocp_triggered;         ///< 过流保护标志
    bool     otp_triggered;         ///< 过热保护标志
    bool     homing_failed;         ///< 回零失败标志
    bool     homing_running;        ///< 正在回零标志
    bool     cal_ready;             ///< 校准表就绪标志
    bool     enc_ready;             ///< 编码器就绪标志
} zdt_homing_status_t;

/** 电机状态标志结构 */
typedef struct {
    bool     power_loss;            ///< 掉电标志
    bool     right_limit_switch;    ///< 右限位开关状态
    bool     left_limit_switch;     ///< 左限位开关状态
    bool     stall_protection;      ///< 堵转保护标志
    bool     stall_flag;            ///< 堵转标志
    bool     position_reached;      ///< 位置到达标志
    bool     motor_enabled;         ///< 使能状态标志
} zdt_motor_status_t;

/** 回零参数结构 */
typedef struct {
    uint8_t  mode;                  ///< 回零模式
    uint8_t  direction;             ///< 回零方向
    uint16_t speed;                 ///< 回零速度 (RPM, 范围0-3000)
    uint32_t timeout_ms;            ///< 回零超时时间 (ms)
    uint16_t collision_speed;       ///< 碰撞回零检测转速 (RPM, 范围0-3000)
    uint16_t collision_current;     ///< 碰撞回零检测电流 (mA, 范围0-5000)
    uint16_t collision_time;        ///< 碰撞回零检测时间 (ms)
    bool     auto_power_on_homing;  ///< 上电自动触发回零功能
} zdt_homing_params_t;

/** 系统状态参数结构（X固件，见5.8.1） */
typedef struct {
    uint16_t bus_voltage;           ///< 总线电压 (mV)
    uint16_t bus_current;           ///< 总线电流 (mA)
    uint16_t phase_current;         ///< 相电流 (mA)
    uint16_t encoder_raw;           ///< 编码器原始值
    uint16_t encoder_linear;        ///< 线性化编码器值
    int32_t  target_position;       ///< 电机目标位置
    int16_t  speed;                 ///< 电机实时转速 (0.1RPM)
    int32_t  real_position;         ///< 电机实时位置 (0.1°)
    int32_t  position_error;        ///< 电机位置误差 (0.01°)
    int8_t   temperature;           ///< 驱动温度 (℃)
    uint8_t  homing_status;         ///< 回零状态标志
    uint8_t  motor_status;          ///< 电机状态标志
} zdt_system_status_t;

/** PID参数结构（X固件） */
typedef struct {
    uint32_t trapezoid_pos_kp;      ///< 梯形曲线位置环Kp
    uint32_t through_pos_kp;        ///< 直通限速位置环Kp
    uint32_t speed_kp;              ///< 速度环Kp
    uint32_t speed_ki;              ///< 速度环Ki
} zdt_pid_params_x_t;

/** PID参数结构（Emm固件） */
typedef struct {
    uint32_t kp;                    ///< 比例系数
    uint32_t ki;                    ///< 积分系数
    uint32_t kd;                    ///< 微分系数
} zdt_pid_params_emm_t;

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
 * @brief 构建编码器校准命令（见5.2.1）
 * @param addr 电机地址 (1-255)
 * @param buf 输出缓冲区
 * @return 命令长度，失败返回负数错误码
 */
int zdt_build_encoder_calibration(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建重启电机命令（见5.2.2）
 */
int zdt_build_restart(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建位置清零命令（见5.2.3）
 */
int zdt_build_clear_position(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建解除保护命令（见5.2.4）
 */
int zdt_build_clear_protection(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建恢复出厂设置命令（见5.2.5）
 */
int zdt_build_factory_reset(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建使能控制命令（见5.3.2）
 * @param addr 电机地址
 * @param enable 使能状态
 * @param sync 同步标志
 * @param buf 输出缓冲区
 */
int zdt_build_enable(uint8_t addr, zdt_enable_state_t enable, 
                     zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建回零运动命令（见5.4.2）
 * @param addr 电机地址
 * @param mode 回零模式
 * @param sync 同步标志
 * @param buf 输出缓冲区
 */
int zdt_build_homing(uint8_t addr, uint8_t mode, 
                     zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建急停命令（见5.3.13）
 * @param addr 电机地址
 * @param sync 同步标志
 * @param buf 输出缓冲区
 */
int zdt_build_emergency_stop(uint8_t addr, zdt_sync_flag_t sync, 
                             zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取位置角度命令（见5.5.13）
 */
int zdt_build_read_position(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取脉冲数命令（见5.5.8）
 */
int zdt_build_read_pulses(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取驱动温度命令（X42S/Y42，见5.5.12）
 */
int zdt_build_read_temperature(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取状态标志命令（见5.5.15）
 */
int zdt_build_read_status_flags(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取配置参数命令
 */
int zdt_build_read_config(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建定时返回信息命令（5.5.1，X42S/Y42）
 *
 * 使电机按指定周期定时返回"读取系统参数"中某命令的数据，无需主机频繁轮询。
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

/**
 * @brief 构建读取版本信息命令（见5.5.2）
 */
int zdt_build_read_version(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取相电阻相电感命令（见5.5.3）
 */
int zdt_build_read_phase_rl(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取总线电压命令（见5.5.4）
 */
int zdt_build_read_bus_voltage(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取总线电流命令（X42S/Y42，见5.5.5）
 */
int zdt_build_read_bus_current(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取相电流命令（见5.5.6）
 */
int zdt_build_read_phase_current(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取电池电压命令（Y42，见5.5.18）
 */
int zdt_build_read_battery_voltage(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取编码器线性值命令（见5.5.7）
 */
int zdt_build_read_encoder_linear(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取输入脉冲数命令（见5.5.8）
 */
int zdt_build_read_input_pulses(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取目标位置命令（见5.5.9）
 */
int zdt_build_read_target_pos(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取设定目标位置命令（见5.5.10）
 */
int zdt_build_read_set_target_pos(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取速度命令（见5.5.11）
 */
int zdt_build_read_speed(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取位置误差命令（见5.5.14）
 */
int zdt_build_read_position_error(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取回零状态标志命令（见5.4.4）
 */
int zdt_build_read_homing_status(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取系统状态参数命令（X固件，见5.8.1）
 */
int zdt_build_read_system_status(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取双重状态标志命令（X42S/Y42，见5.5.16）
 */
int zdt_build_read_both_status(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取IO电平状态命令（X42S/Y42，见5.5.17）
 */
int zdt_build_read_io_levels(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建设置回零零点位置命令（见5.4.1）
 */
int zdt_build_set_homing_zero(uint8_t addr, bool store, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建退出回零命令（见5.4.3）
 */
int zdt_build_exit_homing(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建触发多机同步运动命令（见5.3.14）
 */
int zdt_build_sync_trigger(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建广播读取ID地址命令（X42S/Y42，见5.6.30）
 *
 * 用于忘记电机ID地址时，单独接线该电机，广播读取该电机的地址。
 *
 * @param buf 输出缓冲区
 * @return 命令长度(3)，失败返回负数错误码
 */
int zdt_build_broadcast_read_id(zdt_cmd_buffer_t *buf);

/* ============================================================================
 * 参数修改命令构建API
 * ========================================================================== */

/**
 * @brief 构建修改细分值命令
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param subdivision 细分值 (1-255, 0表示256细分)
 * @param buf 输出缓冲区
 * @return 命令长度(6)，失败返回负数错误码
 */
int zdt_build_set_subdivision(uint8_t addr, bool store, uint8_t subdivision, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改掉电标志命令
 * @param addr 电机地址
 * @param flag 掉电标志 (默认为1，可修改为0)
 * @param buf 输出缓冲区
 * @return 命令长度(4)，失败返回负数错误码
 */
int zdt_build_set_power_loss_flag(uint8_t addr, bool flag, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改电机类型命令
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param type 电机类型 (0.9°/1.8°步进电机)
 * @param buf 输出缓冲区
 * @return 命令长度(6)，失败返回负数错误码
 */
int zdt_build_set_motor_type(uint8_t addr, bool store, zdt_motor_type_t type, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改固件类型命令
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param type 固件类型 (X固件/Emm固件/Emm固件狂暴模式)
 * @param buf 输出缓冲区
 * @return 命令长度(6)，失败返回负数错误码
 */
int zdt_build_set_firmware_type(uint8_t addr, bool store, zdt_firmware_type_t type, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改控制模式命令
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param mode 控制模式 (开环/闭环)
 * @param buf 输出缓冲区
 * @return 命令长度(6)，失败返回负数错误码
 */
int zdt_build_set_control_mode(uint8_t addr, bool store, zdt_control_mode_t mode, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改电机方向命令
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param dir 电机运动正方向 (CW/CCW)
 * @param buf 输出缓冲区
 * @return 命令长度(6)，失败返回负数错误码
 */
int zdt_build_set_motor_dir(uint8_t addr, bool store, zdt_direction_t dir, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改锁定按键功能命令
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param lock 锁定状态 (false=解锁, true=锁定)
 * @param buf 输出缓冲区
 * @return 命令长度(6)，失败返回负数错误码
 */
int zdt_build_set_key_lock(uint8_t addr, bool store, bool lock, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改输入缩小功能命令
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param enable 使能状态 (false=关闭, true=使能)
 * @param buf 输出缓冲区
 * @return 命令长度(6)，失败返回负数错误码
 */
int zdt_build_set_input_scale(uint8_t addr, bool store, bool enable, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改开环模式工作电流命令
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param current_ma 工作电流 (0-5000 mA)
 * @param buf 输出缓冲区
 * @return 命令长度(7)，失败返回负数错误码
 */
int zdt_build_set_open_loop_current(uint8_t addr, bool store, uint16_t current_ma, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改闭环模式最大电流命令
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param current_ma 最大电流 (0-5000 mA)
 * @param buf 输出缓冲区
 * @return 命令长度(7)，失败返回负数错误码
 */
int zdt_build_set_closed_loop_current(uint8_t addr, bool store, uint16_t current_ma, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改PID参数命令（X固件，见5.6.15）
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param params PID参数结构
 * @param buf 输出缓冲区
 * @return 命令长度(20)，失败返回负数错误码
 */
int zdt_build_set_pid_params_x(uint8_t addr, bool store, const zdt_pid_params_x_t *params, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改PID参数命令（Emm固件，见5.6.17）
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param params PID参数结构
 * @param buf 输出缓冲区
 * @return 命令长度(16)，失败返回负数错误码
 */
int zdt_build_set_pid_params_emm(uint8_t addr, bool store, const zdt_pid_params_emm_t *params, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改回零参数命令（见5.4.6）
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param params 回零参数结构
 * @param buf 输出缓冲区
 * @return 命令长度(21)，失败返回负数错误码
 */
int zdt_build_set_homing_params(uint8_t addr, bool store, const zdt_homing_params_t *params, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改位置到达窗口命令（X42S/Y42，见5.6.21）
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param window 位置到达窗口 (内部缩小10倍处理，如输入8表示0.8°)
 * @param buf 输出缓冲区
 * @return 命令长度(7)，失败返回负数错误码
 */
int zdt_build_set_pos_window(uint8_t addr, bool store, uint16_t window, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改过热过流保护检测阈值命令（X42S/Y42，见5.6.23）
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param temp_threshold 过热保护检测阈值 (℃)
 * @param current_threshold 过流保护检测阈值 (mA)
 * @param time_threshold 检测时间 (ms)
 * @param buf 输出缓冲区
 * @return 命令长度(12)，失败返回负数错误码
 */
int zdt_build_set_protection_threshold(uint8_t addr, bool store,
                                      uint16_t temp_threshold,
                                      uint16_t current_threshold,
                                      uint16_t time_threshold, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改心跳保护功能时间命令（X42S/Y42，见5.6.25）
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param time_ms 心跳保护时间 (ms)
 * @param buf 输出缓冲区
 * @return 命令长度(9)，失败返回负数错误码
 */
int zdt_build_set_heartbeat_time(uint8_t addr, bool store, uint32_t time_ms, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改积分限幅/刚性系数命令（X42S/Y42，见5.6.27）
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param limit 积分限幅/刚性系数 (Emm:积分限幅65535/X:刚性系数388-512)
 * @param buf 输出缓冲区
 * @return 命令长度(9)，失败返回负数错误码
 */
int zdt_build_set_integral_limit(uint8_t addr, bool store, uint32_t limit, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改碰撞回零返回角度命令（X42S/Y42，见5.6.29）
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param angle 返回角度 (放大10倍输入，单位0.1°，如20表示2.0°)
 * @param buf 输出缓冲区
 * @return 命令长度(7)，失败返回负数错误码
 */
int zdt_build_set_homing_angle(uint8_t addr, bool store, uint16_t angle, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改参数锁定功能命令（X42S/Y42，见5.6.31）
 * @param addr 电机地址
 * @param store 是否存储到flash
 * @param lock_level 锁定参数等级 (0=解锁, 1=禁止修改基础参数, 2/3=禁止修改所有参数)
 * @param buf 输出缓冲区
 * @return 命令长度(6)，失败返回负数错误码
 */
int zdt_build_set_param_lock(uint8_t addr, bool store, uint8_t lock_level, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建修改电机ID命令（见5.6.1）
 * @param addr 当前电机地址
 * @param new_id 新的电机ID (1-255)
 * @param store 是否存储到flash
 * @param buf 输出缓冲区
 * @return 命令长度(6)，失败返回负数错误码
 */
int zdt_build_set_motor_id(uint8_t addr, uint8_t new_id, bool store, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建存储上电自动运行参数命令（X固件，见5.7.1）
 * @param addr 电机地址
 * @param store 是否存储（true=存储速度和加速度, false=清除已存储参数）
 * @param dir 运动方向
 * @param acceleration 加速度 (0-65535 RPM/S)
 * @param speed 速度 (0-3000.0 RPM, 单位0.1RPM)
 * @param enable_en_pin 是否使能EN引脚控制启停
 * @param buf 输出缓冲区
 * @return 命令长度(11)，失败返回负数错误码
 */
int zdt_build_set_auto_run_params_x(uint8_t addr, bool store, zdt_direction_t dir,
                                   uint16_t acceleration, uint16_t speed,
                                   bool enable_en_pin, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建存储上电自动运行参数命令（Emm固件，见5.7.2）
 * @param addr 电机地址
 * @param store 是否存储（true=存储速度和加速度, false=清除已存储参数）
 * @param dir 运动方向
 * @param speed 速度 (0-3000 RPM)
 * @param acceleration 加速度档位 (0-255)
 * @param enable_en_pin 是否使能EN引脚控制启停
 * @param buf 输出缓冲区
 * @return 命令长度(10)，失败返回负数错误码
 */
int zdt_build_set_auto_run_params_emm(uint8_t addr, bool store, zdt_direction_t dir,
                                     uint16_t speed, uint8_t acceleration,
                                     bool enable_en_pin, zdt_cmd_buffer_t *buf);

/* ============================================================================
 * EMM固件运动控制命令构建
 * ========================================================================== */

/**
 * @brief 构建力矩模式控制命令 (X固件，见5.3.3)
 * 
 * 命令格式: 地址 | 0xF5 | 符号 | 斜率[2] | 电流[2] | 同步标志 | 校验码
 * 
 * @param addr 电机地址
 * @param dir 旋转方向 (0=CW, 1=CCW)
 * @param slope 加速度/斜率 (0-65535 mA/S)
 * @param current_ma 电流 (0-5000 mA)
 * @param sync 同步标志
 * @param buf 输出缓冲区
 * @return 命令长度(9)，失败返回负数错误码
 */
int zdt_build_torque_mode(uint8_t addr, zdt_direction_t dir,
                          uint32_t slope, uint16_t current_ma,
                          zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建力矩模式限速控制命令 (X固件，见5.3.4)
 * 
 * 命令格式: 地址 | 0xC5 | 符号 | 斜率[2] | 电流[2] | 同步标志 | 最大速度[2] | 校验码
 * 
 * @param addr 电机地址
 * @param dir 旋转方向 (0=CW, 1=CCW)
 * @param slope 加速度/斜率 (0-65535 mA/S)
 * @param current_ma 电流 (0-5000 mA)
 * @param sync 同步标志
 * @param max_speed 最大速度 (0-3000.0 RPM, 单位0.1RPM)
 * @param buf 输出缓冲区
 * @return 命令长度(11)，失败返回负数错误码
 */
int zdt_build_torque_mode_limit(uint8_t addr, zdt_direction_t dir,
                                uint32_t slope, uint16_t current_ma,
                                zdt_sync_flag_t sync, uint16_t max_speed,
                                zdt_cmd_buffer_t *buf);

/**
 * @brief 构建速度模式限电流控制命令 (X固件，见5.3.6)
 * 
 * 命令格式: 地址 | 0xC6 | 符号 | 加速度[2] | 速度[2] | 同步标志 | 最大电流[2] | 校验码
 * 
 * @param addr 电机地址
 * @param dir 旋转方向 (0=CW, 1=CCW)
 * @param acceleration 加速度 (0-65535 RPM/S)
 * @param speed_rpm 速度 (0-3000.0 RPM, 单位0.1RPM)
 * @param sync 同步标志
 * @param max_current_ma 最大电流 (0-5000 mA)
 * @param buf 输出缓冲区
 * @return 命令长度(11)，失败返回负数错误码
 */
int zdt_build_speed_mode_limit(uint8_t addr, zdt_direction_t dir,
                               uint32_t acceleration, uint16_t speed_rpm,
                               zdt_sync_flag_t sync, uint16_t max_current_ma,
                               zdt_cmd_buffer_t *buf);

/**
 * @brief 构建速度模式命令 (EMM固件，见5.3.7)
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
 * @brief 构建位置模式命令 (EMM固件，见5.3.12)
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
int zdt_build_position_mode_emm(uint8_t addr, zdt_direction_t dir,
                                uint16_t speed_rpm, uint8_t acc,
                                int32_t position_01deg,
                                zdt_motion_mode_t motion_mode, 
                                zdt_sync_flag_t sync,
                                zdt_cmd_buffer_t *buf);

/**
 * @brief 构建直通限速位置模式控制命令 (X固件，见5.3.8)
 * 
 * 命令格式: 地址 | 0xFB | 方向 | 速度[2] | 位置[4] | 运动模式 | 同步标志 | 校验码
 * 
 * @param addr 电机地址
 * @param dir 旋转方向 (0=CW, 1=CCW)
 * @param speed_rpm 速度 (0-3000.0 RPM, 单位0.1RPM)
 * @param position_01deg 位置角度 (0.1° 单位)
 * @param motion_mode 运动模式 (0=相对上一输入, 1=绝对, 2=相对当前位置)
 * @param sync 同步标志
 * @param buf 输出缓冲区
 * @return 命令长度(12)，失败返回负数错误码
 */
int zdt_build_position_through(uint8_t addr, zdt_direction_t dir,
                               uint16_t speed_rpm, int32_t position_01deg,
                               zdt_motion_mode_t motion_mode, zdt_sync_flag_t sync,
                               zdt_cmd_buffer_t *buf);

/**
 * @brief 构建直通限速位置模式限电流控制命令 (X固件，见5.3.9)
 * 
 * 命令格式: 地址 | 0xCB | 方向 | 速度[2] | 位置[4] | 运动模式 | 同步标志 | 最大电流[2] | 校验码
 * 
 * @param addr 电机地址
 * @param dir 旋转方向 (0=CW, 1=CCW)
 * @param speed_rpm 速度 (0-3000.0 RPM, 单位0.1RPM)
 * @param position_01deg 位置角度 (0.1° 单位)
 * @param motion_mode 运动模式
 * @param sync 同步标志
 * @param max_current_ma 最大电流 (0-5000 mA)
 * @param buf 输出缓冲区
 * @return 命令长度(14)，失败返回负数错误码
 */
int zdt_build_position_through_limit(uint8_t addr, zdt_direction_t dir,
                                     uint16_t speed_rpm, int32_t position_01deg,
                                     zdt_motion_mode_t motion_mode, zdt_sync_flag_t sync,
                                     uint16_t max_current_ma, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建梯形曲线加减速位置模式控制命令 (X固件，见5.3.10)
 * 
 * 命令格式: 地址 | 0xFD | 方向 | 加速加速度[2] | 减速加速度[2] | 最大速度[2] | 位置[4] | 运动模式 | 同步标志 | 校验码
 * 
 * @param addr 电机地址
 * @param dir 旋转方向 (0=CW, 1=CCW)
 * @param acc_acceleration 加速加速度 (0-65535 RPM/S)
 * @param dec_acceleration 减速加速度 (0-65535 RPM/S)
 * @param max_speed_rpm 最大速度 (0-3000.0 RPM, 单位0.1RPM)
 * @param position_01deg 位置角度 (0.1° 单位)
 * @param motion_mode 运动模式
 * @param sync 同步标志
 * @param buf 输出缓冲区
 * @return 命令长度(16)，失败返回负数错误码
 */
int zdt_build_position_trapezoid(uint8_t addr, zdt_direction_t dir,
                                 uint16_t acc_acceleration, uint16_t dec_acceleration,
                                 uint16_t max_speed_rpm, int32_t position_01deg,
                                 zdt_motion_mode_t motion_mode, zdt_sync_flag_t sync,
                                 zdt_cmd_buffer_t *buf);

/**
 * @brief 构建梯形曲线加减速位置模式限电流控制命令 (X固件，见5.3.11)
 * 
 * 命令格式: 地址 | 0xCD | 方向 | 加速加速度[2] | 减速加速度[2] | 最大速度[2] | 位置[4] | 运动模式 | 同步标志 | 最大电流[2] | 校验码
 * 
 * @param addr 电机地址
 * @param dir 旋转方向 (0=CW, 1=CCW)
 * @param acc_acceleration 加速加速度 (0-65535 RPM/S)
 * @param dec_acceleration 减速加速度 (0-65535 RPM/S)
 * @param max_speed_rpm 最大速度 (0-3000.0 RPM, 单位0.1RPM)
 * @param position_01deg 位置角度 (0.1° 单位)
 * @param motion_mode 运动模式
 * @param sync 同步标志
 * @param max_current_ma 最大电流 (0-5000 mA)
 * @param buf 输出缓冲区
 * @return 命令长度(18)，失败返回负数错误码
 */
int zdt_build_position_trapezoid_limit(uint8_t addr, zdt_direction_t dir,
                                       uint16_t acc_acceleration, uint16_t dec_acceleration,
                                       uint16_t max_speed_rpm, int32_t position_01deg,
                                       zdt_motion_mode_t motion_mode, zdt_sync_flag_t sync,
                                       uint16_t max_current_ma, zdt_cmd_buffer_t *buf);

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
 * @brief 解析位置响应（见5.5.13）
 * @param response 响应结构
 * @param position 输出位置数据
 * @return ZDT_OK成功
 */
int zdt_parse_position_response(const zdt_response_t *response, zdt_position_t *position);

/**
 * @brief 解析脉冲数响应（见5.5.8）
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

/**
 * @brief 解析版本信息响应（见5.5.2）
 * @param response 响应结构
 * @param version 输出版本信息
 * @return ZDT_OK成功
 */
int zdt_parse_version_response(const zdt_response_t *response, zdt_version_info_t *version);

/**
 * @brief 解析相电阻相电感响应（见5.5.3）
 * @param response 响应结构
 * @param phase_rl 输出相电阻相电感数据
 * @return ZDT_OK成功
 */
int zdt_parse_phase_rl_response(const zdt_response_t *response, zdt_phase_rl_t *phase_rl);

/**
 * @brief 解析回零状态标志响应（见5.4.4）
 * @param response 响应结构
 * @param status 输出回零状态标志
 * @return ZDT_OK成功
 */
int zdt_parse_homing_status_response(const zdt_response_t *response, zdt_homing_status_t *status);

/**
 * @brief 解析电机状态标志响应（见5.5.15）
 * @param response 响应结构
 * @param status 输出电机状态标志
 * @return ZDT_OK成功
 */
int zdt_parse_motor_status_response(const zdt_response_t *response, zdt_motor_status_t *status);

/**
 * @brief 解析双重状态标志响应（X42S/Y42，见5.5.16）
 * @param response 响应结构
 * @param homing_status 输出回零状态标志
 * @param motor_status 输出电机状态标志
 * @return ZDT_OK成功
 */
int zdt_parse_both_status_response(const zdt_response_t *response,
                                  zdt_homing_status_t *homing_status,
                                  zdt_motor_status_t *motor_status);

/**
 * @brief 解析回零参数响应
 * @param response 响应结构
 * @param params 输出回零参数
 * @return ZDT_OK成功
 */
int zdt_parse_homing_params_response(const zdt_response_t *response, zdt_homing_params_t *params);

/**
 * @brief 解析速度响应（见5.5.11）
 * @param response 响应结构
 * @param speed_rpm 输出速度 (RPM)
 * @return ZDT_OK成功
 */
int zdt_parse_speed_response(const zdt_response_t *response, float *speed_rpm);

/**
 * @brief 解析目标位置响应（见5.5.9）
 * @param response 响应结构
 * @param position 输出位置数据
 * @return ZDT_OK成功
 */
int zdt_parse_target_pos_response(const zdt_response_t *response, zdt_position_t *position);

/**
 * @brief 解析编码器线性值响应（见5.5.7）
 * @param response 响应结构
 * @param encoder_val 输出编码器值 (0-65535)
 * @return ZDT_OK成功
 */
int zdt_parse_encoder_linear_response(const zdt_response_t *response, uint16_t *encoder_val);

/**
 * @brief 解析总线电压响应（见5.5.4）
 * @param response 响应结构
 * @param voltage_mv 输出总线电压 (mV)
 * @return ZDT_OK成功
 */
int zdt_parse_bus_voltage_response(const zdt_response_t *response, uint16_t *voltage_mv);

/**
 * @brief 解析总线电流响应（X42S/Y42，见5.5.5）
 * @param response 响应结构
 * @param current_ma 输出总线电流 (mA)
 * @return ZDT_OK成功
 */
int zdt_parse_bus_current_response(const zdt_response_t *response, uint16_t *current_ma);

/**
 * @brief 解析相电流响应（见5.5.6）
 * @param response 响应结构
 * @param current_ma 输出相电流 (mA)
 * @return ZDT_OK成功
 */
int zdt_parse_phase_current_response(const zdt_response_t *response, uint16_t *current_ma);

/**
 * @brief 解析电池电压响应（Y42，见5.5.18）
 * @param response 响应结构
 * @param voltage_mv 输出电池电压 (mV)
 * @return ZDT_OK成功
 */
int zdt_parse_battery_voltage_response(const zdt_response_t *response, uint16_t *voltage_mv);

/**
 * @brief 解析输入脉冲数响应（见5.5.8）
 * @param response 响应结构
 * @param pulses 输出脉冲数
 * @return ZDT_OK成功
 */
int zdt_parse_input_pulses_response(const zdt_response_t *response, int32_t *pulses);

/**
 * @brief 解析设定目标位置响应（见5.5.10）
 * @param response 响应结构
 * @param position 输出位置数据
 * @return ZDT_OK成功
 */
int zdt_parse_set_target_pos_response(const zdt_response_t *response, zdt_position_t *position);

/**
 * @brief 解析位置误差响应（见5.5.14）
 * @param response 响应结构
 * @param error 输出位置误差数据
 * @return ZDT_OK成功
 */
int zdt_parse_position_error_response(const zdt_response_t *response, zdt_position_t *error);

/**
 * @brief 解析IO电平状态响应（X42S/Y42，见5.5.17）
 * @param response 响应结构
 * @param io_levels 输出IO电平状态字节
 * @return ZDT_OK成功
 */
int zdt_parse_io_levels_response(const zdt_response_t *response, uint8_t *io_levels);

/**
 * @brief 解析选项参数状态响应（X42S/Y42，见5.6.4）
 * @param response 响应结构
 * @param options 输出选项参数字节
 * @return ZDT_OK成功
 */
int zdt_parse_options_response(const zdt_response_t *response, uint8_t *options);

/**
 * @brief 解析位置到达窗口响应（X42S/Y42，见5.6.20）
 * @param response 响应结构
 * @param window 输出位置到达窗口值
 * @return ZDT_OK成功
 */
int zdt_parse_pos_window_response(const zdt_response_t *response, uint16_t *window);

/**
 * @brief 解析过热过流保护检测阈值响应（X42S/Y42，见5.6.22）
 * @param response 响应结构
 * @param temp_threshold 输出过热保护检测阈值 (℃)
 * @param current_threshold 输出过流保护检测阈值 (mA)
 * @param time_threshold 输出检测时间 (ms)
 * @return ZDT_OK成功
 */
int zdt_parse_protection_threshold_response(const zdt_response_t *response,
                                          uint16_t *temp_threshold,
                                          uint16_t *current_threshold,
                                          uint16_t *time_threshold);

/**
 * @brief 解析心跳保护功能时间响应（X42S/Y42，见5.6.24）
 * @param response 响应结构
 * @param time_ms 输出心跳保护时间 (ms)
 * @return ZDT_OK成功
 */
int zdt_parse_heartbeat_time_response(const zdt_response_t *response, uint32_t *time_ms);

/**
 * @brief 解析积分限幅/刚性系数响应（X42S/Y42，见5.6.26）
 * @param response 响应结构
 * @param limit 输出积分限幅/刚性系数
 * @return ZDT_OK成功
 */
int zdt_parse_integral_limit_response(const zdt_response_t *response, uint32_t *limit);

/**
 * @brief 解析碰撞回零返回角度响应（X42S/Y42，见5.6.28）
 * @param response 响应结构
 * @param angle 输出返回角度
 * @return ZDT_OK成功
 */
int zdt_parse_homing_angle_response(const zdt_response_t *response, uint16_t *angle);

/**
 * @brief 解析PID参数响应（X固件，见5.6.14）
 * @param response 响应结构
 * @param params 输出PID参数
 * @return ZDT_OK成功
 */
int zdt_parse_pid_params_x_response(const zdt_response_t *response, zdt_pid_params_x_t *params);

/**
 * @brief 解析PID参数响应（Emm固件，见5.6.16）
 * @param response 响应结构
 * @param params 输出PID参数
 * @return ZDT_OK成功
 */
int zdt_parse_pid_params_emm_response(const zdt_response_t *response, zdt_pid_params_emm_t *params);

/**
 * @brief 解析系统状态参数响应（X固件，见5.8.1）
 * @param response 响应结构
 * @param status 输出系统状态参数
 * @return ZDT_OK成功
 */
int zdt_parse_system_status_response(const zdt_response_t *response, zdt_system_status_t *status);

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
 * @brief 设置力矩模式 (X固件)
 * @param slope 加速度/斜率 (0-65535 mA/S)
 * @param current_ma 电流 (0-5000 mA)
 */
zdt_error_t zdt_set_torque_mode(zdt_handle_t handle, uint8_t addr,
                                zdt_direction_t dir, uint32_t slope, uint16_t current_ma);

/**
 * @brief 设置力矩模式限速控制 (X固件)
 * @param slope 加速度/斜率 (0-65535 mA/S)
 * @param current_ma 电流 (0-5000 mA)
 * @param max_speed 最大速度 (0-3000.0 RPM)
 */
zdt_error_t zdt_set_torque_mode_limit(zdt_handle_t handle, uint8_t addr,
                                      zdt_direction_t dir, uint32_t slope, 
                                      uint16_t current_ma, uint16_t max_speed);

/**
 * @brief 设置速度模式限电流控制 (X固件)
 * @param acceleration 加速度 (0-65535 RPM/S)
 * @param speed_rpm 速度 (0-3000.0 RPM)
 * @param max_current_ma 最大电流 (0-5000 mA)
 */
zdt_error_t zdt_set_speed_mode_limit(zdt_handle_t handle, uint8_t addr,
                                     zdt_direction_t dir, uint32_t acceleration,
                                     uint16_t speed_rpm, uint16_t max_current_ma);

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

/**
 * @brief 触发多机同步运动
 */
zdt_error_t zdt_sync_trigger(zdt_handle_t handle);

/**
 * @brief 设置换站零零点位置
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_homing_zero(zdt_handle_t handle, uint8_t addr, bool store);

/**
 * @brief 退出回零操作
 */
zdt_error_t zdt_exit_homing(zdt_handle_t handle, uint8_t addr);

/**
 * @brief 读取版本信息
 */
zdt_error_t zdt_read_version(zdt_handle_t handle, uint8_t addr, zdt_version_info_t *version);

/**
 * @brief 读取相电阻相电感
 */
zdt_error_t zdt_read_phase_rl(zdt_handle_t handle, uint8_t addr, zdt_phase_rl_t *phase_rl);

/**
 * @brief 读取总线电压
 */
zdt_error_t zdt_read_bus_voltage(zdt_handle_t handle, uint8_t addr, uint16_t *voltage_mv);

/**
 * @brief 读取总线电流（X42S/Y42）
 */
zdt_error_t zdt_read_bus_current(zdt_handle_t handle, uint8_t addr, uint16_t *current_ma);

/**
 * @brief 读取相电流
 */
zdt_error_t zdt_read_phase_current(zdt_handle_t handle, uint8_t addr, uint16_t *current_ma);

/**
 * @brief 读取电池电压（Y42）
 */
zdt_error_t zdt_read_battery_voltage(zdt_handle_t handle, uint8_t addr, uint16_t *voltage_mv);

/**
 * @brief 读取编码器线性值
 */
zdt_error_t zdt_read_encoder_linear(zdt_handle_t handle, uint8_t addr, uint16_t *encoder_val);

/**
 * @brief 读取电机速度
 */
zdt_error_t zdt_read_speed(zdt_handle_t handle, uint8_t addr, float *speed_rpm);

/**
 * @brief 读取位置误差
 */
zdt_error_t zdt_read_position_error(zdt_handle_t handle, uint8_t addr, zdt_position_t *error);

/**
 * @brief 读取回零状态标志
 */
zdt_error_t zdt_read_homing_status(zdt_handle_t handle, uint8_t addr, zdt_homing_status_t *status);

/**
 * @brief 读取电机状态标志
 */
zdt_error_t zdt_read_motor_status(zdt_handle_t handle, uint8_t addr, zdt_motor_status_t *status);

/**
 * @brief 读取双重状态标志（X42S/Y42）
 */
zdt_error_t zdt_read_both_status(zdt_handle_t handle, uint8_t addr,
                               zdt_homing_status_t *homing_status,
                               zdt_motor_status_t *motor_status);

/**
 * @brief 读取系统状态参数（X固件，见5.8.1）
 * @param handle 句柄
 * @param addr 电机地址
 * @param status 输出系统状态参数
 * @return ZDT_OK成功
 *
 * @note 该命令一次性读取12个系统参数，包括总线电压、总线电流、相电流、
 *       编码器值、目标位置、实时转速、实时位置、位置误差、温度、回零状态和电机状态
 */
zdt_error_t zdt_read_system_status(zdt_handle_t handle, uint8_t addr,
                                   zdt_system_status_t *status);

/**
 * @brief 构建读取选项参数状态命令（X42S/Y42，见5.6.4）
 */
int zdt_build_read_options(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取位置到达窗口命令（X42S/Y42，见5.6.20）
 */
int zdt_build_read_pos_window(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取过热过流保护检测阈值命令（X42S/Y42，见5.6.22）
 */
int zdt_build_read_protection_threshold(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取心跳保护功能时间命令（X42S/Y42，见5.6.24）
 */
int zdt_build_read_heartbeat_time(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取积分限幅/刚性系数命令（X42S/Y42，见5.6.26）
 */
int zdt_build_read_integral_limit(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取碰撞回零返回角度命令（X42S/Y42，见5.6.28）
 */
int zdt_build_read_homing_angle(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 构建读取PID参数命令（见5.6.14/5.6.16）
 */
int zdt_build_read_pid_params(uint8_t addr, zdt_cmd_buffer_t *buf);

/**
 * @brief 解析回零参数响应（见5.4.5）
 */
int zdt_parse_homing_params_response(const zdt_response_t *response, zdt_homing_params_t *params);

/**
 * @brief 解析驱动温度响应（见5.5.12）
 */
int zdt_parse_temperature_response(const zdt_response_t *response, int8_t *temp_c);

/**
 * @brief 构建多电机命令（见5.3.1）
 */
int zdt_build_multi_motor_command(uint8_t addr, const uint8_t *motor_commands,
                                   size_t total_len, zdt_cmd_buffer_t *buf);

/**
 * @brief 读取回零参数
 */
zdt_error_t zdt_read_homing_params(zdt_handle_t handle, uint8_t addr, zdt_homing_params_t *params);

/**
 * @brief 修改回零参数
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_homing_params(zdt_handle_t handle, uint8_t addr,
                                const zdt_homing_params_t *params, bool store);

/**
 * @brief 修改电机ID
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_motor_id(zdt_handle_t handle, uint8_t addr, uint8_t new_id, bool store);

/**
 * @brief 修改细分值
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_subdivision(zdt_handle_t handle, uint8_t addr, uint8_t subdivision, bool store);

/**
 * @brief 修改掉电标志
 */
zdt_error_t zdt_set_power_loss_flag(zdt_handle_t handle, uint8_t addr, bool flag);

/**
 * @brief 修改电机类型
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_motor_type(zdt_handle_t handle, uint8_t addr, zdt_motor_type_t type, bool store);

/**
 * @brief 修改固件类型
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_firmware_type(zdt_handle_t handle, uint8_t addr, zdt_firmware_type_t type, bool store);

/**
 * @brief 修改控制模式
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_control_mode(zdt_handle_t handle, uint8_t addr, zdt_control_mode_t mode, bool store);

/**
 * @brief 修改电机方向
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_motor_dir(zdt_handle_t handle, uint8_t addr, zdt_direction_t dir, bool store);

/**
 * @brief 修改锁定按键功能
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_key_lock(zdt_handle_t handle, uint8_t addr, bool lock, bool store);

/**
 * @brief 修改开环模式工作电流
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_open_loop_current(zdt_handle_t handle, uint8_t addr, uint16_t current_ma, bool store);

/**
 * @brief 修改闭环模式最大电流
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_closed_loop_current(zdt_handle_t handle, uint8_t addr, uint16_t current_ma, bool store);

/**
 * @brief 修改PID参数（X固件）
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_pid_params_x(zdt_handle_t handle, uint8_t addr,
                               const zdt_pid_params_x_t *params, bool store);

/**
 * @brief 修改PID参数（Emm固件）
 * @param store 是否存储到flash
 */
zdt_error_t zdt_set_pid_params_emm(zdt_handle_t handle, uint8_t addr,
                                 const zdt_pid_params_emm_t *params, bool store);

/**
 * @brief 广播读取电机ID（X42S/Y42）
 * @param id 输出读取到的电机ID
 */
zdt_error_t zdt_broadcast_read_id(zdt_handle_t handle, uint8_t *id);

/**
 * @brief 读取选项参数状态（X42S/Y42）
 */
zdt_error_t zdt_read_options(zdt_handle_t handle, uint8_t addr, uint8_t *options);

/**
 * @brief 读取位置到达窗口（X42S/Y42）
 */
zdt_error_t zdt_read_position_window(zdt_handle_t handle, uint8_t addr, uint16_t *window);

/**
 * @brief 读取过热过流保护检测阈值（X42S/Y42）
 */
zdt_error_t zdt_read_protection_threshold(zdt_handle_t handle, uint8_t addr,
                                         uint16_t *temp_threshold,
                                         uint16_t *current_threshold,
                                         uint16_t *time_threshold);

/**
 * @brief 读取心跳保护功能时间（X42S/Y42）
 */
zdt_error_t zdt_read_heartbeat_time(zdt_handle_t handle, uint8_t addr, uint32_t *time_ms);

/**
 * @brief 读取积分限幅/刚性系数（X42S/Y42）
 */
zdt_error_t zdt_read_integral_limit(zdt_handle_t handle, uint8_t addr, uint32_t *limit);

/**
 * @brief 读取碰撞回零返回角度（X42S/Y42）
 */
zdt_error_t zdt_read_homing_return_angle(zdt_handle_t handle, uint8_t addr, uint16_t *angle);

/**
 * @brief 读取PID参数（X固件）
 */
zdt_error_t zdt_read_pid_params_x(zdt_handle_t handle, uint8_t addr,
                                  zdt_pid_params_x_t *params);

/**
 * @brief 读取PID参数（Emm固件）
 */
zdt_error_t zdt_read_pid_params_emm(zdt_handle_t handle, uint8_t addr,
                                    zdt_pid_params_emm_t *params);

/**
 * @brief 读取驱动温度（X42S/Y42）
 */
zdt_error_t zdt_read_temperature(zdt_handle_t handle, uint8_t addr, int8_t *temp_c);

/**
 * @brief 修改位置到达窗口（X42S/Y42）
 */
zdt_error_t zdt_set_position_window(zdt_handle_t handle, uint8_t addr,
                                   uint16_t window, bool store);

/**
 * @brief 修改过热过流保护检测阈值（X42S/Y42）
 */
zdt_error_t zdt_set_protection_threshold(zdt_handle_t handle, uint8_t addr,
                                        uint16_t temp_threshold,
                                        uint16_t current_threshold,
                                        uint16_t time_threshold, bool store);

/**
 * @brief 修改心跳保护功能时间（X42S/Y42）
 */
zdt_error_t zdt_set_heartbeat_time(zdt_handle_t handle, uint8_t addr,
                                   uint32_t time_ms, bool store);

/**
 * @brief 修改积分限幅/刚性系数（X42S/Y42）
 */
zdt_error_t zdt_set_integral_limit(zdt_handle_t handle, uint8_t addr,
                                   uint32_t limit, bool store);

/**
 * @brief 修改碰撞回零返回角度（X42S/Y42）
 */
zdt_error_t zdt_set_homing_return_angle(zdt_handle_t handle, uint8_t addr,
                                       uint16_t angle, bool store);

/**
 * @brief 修改输入缩小功能（X固件）或速度缩小功能（Emm固件）
 */
zdt_error_t zdt_set_input_scale(zdt_handle_t handle, uint8_t addr,
                               bool enable, bool store);

/**
 * @brief 修改锁定修改参数功能（X42S/Y42）
 */
zdt_error_t zdt_set_param_lock(zdt_handle_t handle, uint8_t addr,
                              uint8_t lock_level, bool store);

/**
 * @brief 存储一组速度参数，上电自动运行（X固件）
 */
zdt_error_t zdt_set_auto_run_params_x(zdt_handle_t handle, uint8_t addr,
                                     zdt_direction_t dir,
                                     uint16_t acceleration,
                                     uint16_t speed,
                                     bool enable_en_pin, bool store);

/**
 * @brief 存储一组速度参数，上电自动运行（Emm固件）
 */
zdt_error_t zdt_set_auto_run_params_emm(zdt_handle_t handle, uint8_t addr,
                                       zdt_direction_t dir,
                                       uint16_t speed,
                                       uint8_t acceleration,
                                       bool enable_en_pin, bool store);

/**
 * @brief 发送多电机命令
 */
zdt_error_t zdt_send_multi_motor_command(zdt_handle_t handle, uint8_t addr,
                                        const uint8_t *motor_commands,
                                        size_t total_len);

#endif /* CONFIG_IDF_TARGET */

#ifdef __cplusplus
}
#endif
