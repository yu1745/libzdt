/**
 * @file libzdt.h
 * @brief ZDT闭环步进电机通讯协议库
 *
 * 基于 zdt-protocol.md 第5章通讯命令生成
 */

#ifndef LIBZDT_H
#define LIBZDT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 常量定义 - 基于 zdt-protocol.md 第5章
 *============================================================================*/

/*------------------ 5.0.1 CAN协议帧结构 ------------------*/
#define ZDT_CAN_FRAME_ID(addr, packet_code) (((addr) << 8) | (packet_code))
#define ZDT_MAX_DLC 8  /* CAN数据长度 */

/*------------------ 5.0.3 校验码类型 ------------------*/
#define ZDT_CHECKSUM_FIXED 0x6B    /* 固定校验码（默认） */
#define ZDT_CHECKSUM_XOR   0x00    /* XOR校验（需要计算） */
#define ZDT_CHECKSUM_CRC8  0x00    /* CRC8校验（需要计算） */

/*------------------ 5.0.4 分包代码定义 ------------------*/
#define ZDT_PACKET_0 0x00  /* 第1个分包 */
#define ZDT_PACKET_1 0x01  /* 第2个分包 */
#define ZDT_PACKET_2 0x02  /* 第3个分包 */

/*------------------ 返回数据码（5.0.2 电机返回格式）------------------*/
#define ZDT_RESPONSE_OK          0x02  /* 接收的命令正确 */
#define ZDT_RESPONSE_PARAM_ERR   0xE2  /* 命令参数错误 */
#define ZDT_RESPONSE_FORMAT_ERR  0xEE  /* 命令格式错误 */
#define ZDT_RESPONSE_COMPLETED   0x9F  /* 动作执行完成（电机主动返回） */

/*------------------ 5.2 触发动作命令 ------------------*/
#define ZDT_CMD_CALIBRATE_ENCODER  0x06  /* 触发编码器校准 */
#define ZDT_CMD_REBOOT_MOTOR       0x08  /* 重启电机 */
#define ZDT_CMD_ZERO_POSITION      0x0A  /* 将当前位置角度清零 */
#define ZDT_CMD_CLEAR_PROTECTION   0x0E  /* 解除堵转/过热/过流保护 */
#define ZDT_CMD_FACTORY_RESET      0x0F  /* 恢复出厂设置 */

/* 辅助码（5.2节）*/
#define ZDT_AUX_CALIBRATE  0x45  /* 编码器校准辅助码 */
#define ZDT_AUX_REBOOT     0x97  /* 重启辅助码 */
#define ZDT_AUX_ZERO       0x6D  /* 清零辅助码 */
#define ZDT_AUX_CLEAR_PROT 0x52  /* 清除保护辅助码 */
#define ZDT_AUX_FACTORY    0x5F  /* 恢复出厂辅助码 */

/*------------------ 5.3 运动控制命令 ------------------*/
#define ZDT_CMD_MOTOR_ENABLE    0xF3  /* 电机使能控制 */
#define ZDT_CMD_TORQUE_MODE     0xF5  /* 力矩模式控制（X固件） */
#define ZDT_CMD_TORQUE_LIMITED  0xC5  /* 力矩模式限速控制（X固件） */
#define ZDT_CMD_VELOCITY_MODE   0xF6  /* 速度模式控制 */
#define ZDT_CMD_VELOCITY_LIMITED 0xC6 /* 速度模式限电流控制（X固件） */
#define ZDT_CMD_POSITION_DIRECT 0xFB  /* 直通限速位置模式控制（X固件） */
#define ZDT_CMD_POSITION_DIRECT_LIMITED 0xCB /* 直通限速位置模式限电流控制（X固件） */
#define ZDT_CMD_POSITION_TRAPEZOID 0xFD /* 梯形曲线加减速位置模式控制 */
#define ZDT_CMD_POSITION_TRAPEZOID_LIMITED 0xCD /* 梯形曲线加减速位置模式限电流控制（X固件） */
#define ZDT_CMD_IMMEDIATE_STOP   0xFE  /* 立即停止 */
#define ZDT_CMD_SYNC_TRIGGER     0xFF  /* 触发多机同步运动 */

/* 运动控制辅助码（5.3节）*/
#define ZDT_AUX_MOTOR_ENABLE 0xAB  /* 电机使能辅助码 */
#define ZDT_AUX_STOP         0x98  /* 立即停止辅助码 */
#define ZDT_AUX_SYNC         0x66  /* 同步运动辅助码 */

/* 运动模式（5.3.8/5.3.9/5.3.10/5.3.11/5.3.12）*/
#define ZDT_MOTION_MODE_RELATIVE_LAST  0x00  /* 相对上一输入目标位置进行相对位置运动 */
#define ZDT_MOTION_MODE_ABSOLUTE       0x01  /* 相对坐标零点进行绝对位置运动 */
#define ZDT_MOTION_MODE_RELATIVE_REAL  0x02  /* 相对当前实时位置进行相对位置运动 */

/* 方向定义 */
#define ZDT_DIRECTION_CW   0x00  /* 顺时针（Clockwise） */
#define ZDT_DIRECTION_CCW  0x01  /* 逆时针（Counter-Clockwise） */

/* 同步标志（通用）*/
#define ZDT_SYNC_IMMEDIATE 0x00  /* 立即执行 */
#define ZDT_SYNC_CACHED    0x01  /* 先缓存当前命令 */

/* 是否存储标志（通用）*/
#define ZDT_STORAGE_NO  0x00  /* 不存储 */
#define ZDT_STORAGE_YES 0x01  /* 存储，掉电不丢失 */

/* 符号标志 */
#define ZDT_SIGN_POSITIVE 0x00  /* 正号 */
#define ZDT_SIGN_NEGATIVE 0x01  /* 负号 */

/*------------------ 5.4 原点回零命令 ------------------*/
#define ZDT_CMD_SET_ZERO_POINT    0x93  /* 设置单圈回零的零点位置 */
#define ZDT_CMD_TRIGGER_HOMING    0x9A  /* 触发回零 */
#define ZDT_CMD_STOP_HOMING       0x9C  /* 强制中断并退出回零操作 */
#define ZDT_CMD_READ_HOMING_STATE 0x3B  /* 读取回零状态标志 */
#define ZDT_CMD_READ_HOMING_PARAM 0x22  /* 读取回零参数 */
#define ZDT_CMD_WRITE_HOMING_PARAM 0x4C /* 修改回零参数 */

/* 回零相关辅助码（5.4节）*/
#define ZDT_AUX_SET_ZERO    0x88  /* 设置零点辅助码 */
#define ZDT_AUX_STOP_HOMING 0x48  /* 停止回零辅助码 */
#define ZDT_AUX_WRITE_HOMING 0xAE /* 修改回零参数辅助码 */

/* 回零模式（5.4.2/5.4.6）*/
#define ZDT_HOMING_MODE_NEAREST         0x00  /* 单圈就近回零 */
#define ZDT_HOMING_MODE_DIRECTION       0x01  /* 单圈方向回零 */
#define ZDT_HOMING_MODE_COLLISION       0x02  /* 无限位碰撞回零 */
#define ZDT_HOMING_MODE_LIMIT_SWITCH    0x03  /* 限位回零 */
#define ZDT_HOMING_MODE_ABSOLUTE_ZERO   0x04  /* 回到绝对位置坐标零点 */
#define ZDT_HOMING_MODE_POWER_LOST_POS  0x05  /* 回到上次掉电位置角度 */

/*------------------ 5.5 读取系统参数命令 ------------------*/
#define ZDT_CMD_TIMED_RETURN     0x11  /* 定时返回信息命令 */
#define ZDT_CMD_READ_VERSION     0x1F  /* 读取固件版本和硬件版本 */
#define ZDT_CMD_READ_MOTOR_PARAM 0x20  /* 读取相电阻和相电感 */
#define ZDT_CMD_READ_BUS_VOLTAGE 0x24  /* 读取总线电压 */
#define ZDT_CMD_READ_BUS_CURRENT 0x26  /* 读取总线电流（X42S/Y42）*/
#define ZDT_CMD_READ_PHASE_CURRENT 0x27 /* 读取相电流 */
#define ZDT_CMD_READ_ENCODER_LINEARIZED 0x31 /* 读取经过线性化校准后的编码器值 */
#define ZDT_CMD_READ_PULSE_COUNT  0x32  /* 读取输入脉冲数 */
#define ZDT_CMD_READ_TARGET_POS   0x33  /* 读取电机目标位置 */
#define ZDT_CMD_READ_SET_POS      0x34  /* 读取电机实时设定的目标位置 */
#define ZDT_CMD_READ_SPEED        0x35  /* 读取电机实时转速 */
#define ZDT_CMD_READ_BATTERY_VOLTAGE 0x38 /* 读取电池电压（Y42）*/
#define ZDT_CMD_READ_DRIVER_TEMP  0x39  /* 读取驱动温度（X42S/Y42）*/
#define ZDT_CMD_READ_REAL_POS     0x36  /* 读取电机实时位置 */
#define ZDT_CMD_READ_POS_ERROR    0x37  /* 读取电机位置误差 */
#define ZDT_CMD_READ_MOTOR_STATE  0x3A  /* 读取电机状态标志 */
#define ZDT_CMD_READ_HOMING_MOTOR_STATE 0x3C /* 读取回零状态标志+电机状态标志（X42S/Y42）*/
#define ZDT_CMD_READ_IO_STATE     0x3D  /* 读取引脚IO电平状态（X42S/Y42）*/
#define ZDT_CMD_READ_SYSTEM_STATE 0x43  /* 读取系统状态参数（X固件）*/
#define ZDT_CMD_READ_ALL_PARAMS   0x4A  /* 读取所有驱动参数 */

/* 定时返回辅助码（5.5.1）*/
#define ZDT_AUX_TIMED_RETURN 0x18

/*------------------ 5.6 读写驱动参数命令 ------------------*/
#define ZDT_CMD_WRITE_MOTOR_ID     0xAE  /* 修改电机ID/地址 */
#define ZDT_CMD_WRITE_SUBDIVS      0x84  /* 修改细分值 */
#define ZDT_CMD_WRITE_POWER_LOST_FLAG 0x50 /* 修改掉电标志 */
#define ZDT_CMD_READ_OPTION_PARAM  0x1A  /* 读取选项参数状态（X42S/Y42）*/
#define ZDT_CMD_WRITE_MOTOR_TYPE   0xD7  /* 修改电机类型 */
#define ZDT_CMD_WRITE_FW_TYPE      0xD5  /* 修改固件类型 */
#define ZDT_CMD_WRITE_CONTROL_MODE 0x46  /* 修改开环/闭环控制模式 */
#define ZDT_CMD_WRITE_MOTOR_DIR    0xD4  /* 修改电机运动正方向 */
#define ZDT_CMD_WRITE_KEY_LOCK     0xD0  /* 修改锁定按键功能 */
#define ZDT_CMD_WRITE_SCALE_INPUT  0x4F  /* 修改命令位置角度是否继续缩小10倍输入（X）*/
#define ZDT_CMD_WRITE_OPEN_LOOP_CURRENT 0x44 /* 修改开环模式工作电流 */
#define ZDT_CMD_WRITE_CLOSED_LOOP_CURRENT 0x45 /* 修改闭环模式最大电流 */
#define ZDT_CMD_READ_PID_PARAM     0x21  /* 读取PID参数 */
#define ZDT_CMD_WRITE_PID_PARAM    0x4A  /* 修改PID参数 */
#define ZDT_CMD_READ_POS_WINDOW    0x41  /* 读取位置到达窗口（X42S/Y42）*/
#define ZDT_CMD_WRITE_POS_WINDOW   0xD1  /* 修改位置到达窗口（X42S/Y42）*/
#define ZDT_CMD_READ_PROTECT_THRESHOLD 0x13 /* 读取过热过流保护检测阈值（X42S/Y42）*/
#define ZDT_CMD_WRITE_PROTECT_THRESHOLD 0xD3 /* 修改过热过流保护检测阈值（X42S/Y42）*/
#define ZDT_CMD_READ_HEARTBEAT     0x16  /* 读取心跳保护功能时间（X42S/Y42）*/
#define ZDT_CMD_WRITE_HEARTBEAT    0x68  /* 修改心跳保护功能时间（X42S/Y42）*/
#define ZDT_CMD_READ_INTEGRAL_LIMIT 0x23 /* 读取积分限幅/刚性系数（X42S/Y42）*/
#define ZDT_CMD_WRITE_INTEGRAL_LIMIT 0x4B /* 修改积分限幅/刚性系数（X42S/Y42）*/
#define ZDT_CMD_READ_HOMING_ANGLE  0x3F  /* 读取碰撞回零返回角度（X42S/Y42）*/
#define ZDT_CMD_WRITE_HOMING_ANGLE 0x5C  /* 修改碰撞回零返回角度（X42S/Y42）*/
#define ZDT_CMD_BROADCAST_READ_ID  0x15  /* 广播读取ID地址（X42S/Y42）*/
#define ZDT_CMD_WRITE_PARAM_LOCK   0xD6  /* 修改锁定修改参数功能（X42S/Y42）*/

/* 参数修改辅助码（5.6节）*/
#define ZDT_AUX_WRITE_ID       0x02  /* 修改ID辅助码 */
#define ZDT_AUX_WRITE_SUBDIVS  0x8A  /* 修改细分辅助码 */
#define ZDT_AUX_WRITE_MOTOR_TYPE 0x35 /* 修改电机类型辅助码 */
#define ZDT_AUX_WRITE_FW_TYPE  0x69  /* 修改固件类型辅助码 */
#define ZDT_AUX_WRITE_CTRL_MODE 0xA6 /* 修改控制模式辅助码 */
#define ZDT_AUX_WRITE_MOTOR_DIR 0x60 /* 修改电机方向辅助码 */
#define ZDT_AUX_WRITE_KEY_LOCK 0xB3 /* 修改按键锁定辅助码 */
#define ZDT_AUX_WRITE_SCALE    0x71  /* 修改缩放输入辅助码 */
#define ZDT_AUX_WRITE_OPEN_CURRENT 0x33 /* 开环电流辅助码 */
#define ZDT_AUX_WRITE_CLOSED_CURRENT 0x66 /* 闭环电流辅助码 */
#define ZDT_AUX_WRITE_PID      0xC3 /* 修改PID辅助码 */
#define ZDT_AUX_WRITE_POS_WINDOW 0x07 /* 修改位置窗口辅助码 */
#define ZDT_AUX_WRITE_PROTECT   0x56 /* 修改保护阈值辅助码 */
#define ZDT_AUX_WRITE_HEARTBEAT 0x38 /* 修改心跳辅助码 */
#define ZDT_AUX_WRITE_INTEGRAL  0x57 /* 修改积分限幅辅助码 */
#define ZDT_AUX_WRITE_HOMING_ANGLE 0xAC /* 修改回零角度辅助码 */
#define ZDT_AUX_WRITE_PARAM_LOCK 0x4B /* 修改参数锁定辅助码 */

/* 电机类型（5.6.5）*/
#define ZDT_MOTOR_TYPE_0_9_DEG  19  /* 0.9°步进电机 */
#define ZDT_MOTOR_TYPE_1_8_DEG  32  /* 1.8°步进电机 */

/* 固件类型（5.6.6）*/
#define ZDT_FW_TYPE_X      0x00  /* X固件 */
#define ZDT_FW_TYPE_EMM    0x01  /* Emm固件 */
#define ZDT_FW_TYPE_EMM_RAMP 0x02  /* Emm固件狂暴模式 */

/* 控制模式（5.6.7）*/
#define ZDT_CONTROL_MODE_OPEN_LOOP  0x00  /* 开环模式 */
#define ZDT_CONTROL_MODE_CLOSED_LOOP 0x01  /* 闭环模式 */

/*==============================================================================
 * 数据结构定义
 *============================================================================*/

/**
 * @brief CAN消息结构
 * 基于 5.0.1 CAN协议帧结构
 */
typedef struct {
    uint32_t id;      /* 扩展帧ID */
    uint8_t dlc;      /* 数据长度 */
    uint8_t data[8];  /* 数据域 */
} zdt_can_msg_t;

/**
 * @brief 电机状态标志
 * 基于 5.5.15 读取电机状态标志
 */
typedef struct {
    uint8_t ens_tf : 1;  /* bit0: 使能状态标志 */
    uint8_t prf_tf : 1;  /* bit1: 位置到达标志 */
    uint8_t cgi_tf : 1;  /* bit2: 堵转标志 */
    uint8_t cgp_tf : 1;  /* bit3: 堵转保护标志 */
    uint8_t esi_lf : 1;  /* bit4: 左限位开关状态 */
    uint8_t esi_rf : 1;  /* bit5: 右限位开关状态 */
    uint8_t reserved : 1; /* bit6: 保留 */
    uint8_t oac_tf : 1;  /* bit7: 掉电标志 */
} zdt_motor_state_t;

/**
 * @brief 回零状态标志
 * 基于 5.4.4 读取回零状态标志
 */
typedef struct {
    uint8_t enc_rdy : 1;  /* bit0: 编码器就绪标志 */
    uint8_t cal_rdy : 1;  /* bit1: 校准表就绪标志 */
    uint8_t org_sf  : 1;  /* bit2: 正在回零标志 */
    uint8_t org_cf  : 1;  /* bit3: 回零失败标志 */
    uint8_t otp_tf  : 1;  /* bit4: 过热保护标志 */
    uint8_t ocp_tf  : 1;  /* bit5: 过流保护标志 */
    uint8_t reserved1 : 1; /* bit6: 保留 */
    uint8_t reserved2 : 1; /* bit7: 保留 */
} zdt_homing_state_t;

/**
 * @brief 速度模式命令参数（X固件）
 * 基于 5.3.6 速度模式限电流控制（X）
 */
typedef struct {
    uint8_t addr;           /* 电机地址 */
    uint8_t direction;      /* 方向：CW/CCW */
    uint16_t acceleration;  /* 加速度 RPM/S */
    uint16_t speed;         /* 速度 0.1RPM */
    uint8_t sync_flag;      /* 同步标志 */
    uint16_t max_current;   /* 最大电流 mA */
} zdt_velocity_cmd_x_t;

/**
 * @brief 位置模式命令参数（X固件梯形曲线）
 * 基于 5.3.11 梯形曲线加减速位置模式限电流控制（X）
 */
typedef struct {
    uint8_t addr;           /* 电机地址 */
    uint8_t direction;      /* 方向：CW/CCW */
    uint16_t accel_acc;     /* 加速加速度 RPM/S */
    uint16_t decel_acc;     /* 减速加速度 RPM/S */
    uint16_t max_speed;     /* 最大速度 0.1RPM */
    uint32_t position;      /* 位置角度 0.1° */
    uint8_t motion_mode;    /* 运动模式 */
    uint8_t sync_flag;      /* 同步标志 */
    uint16_t max_current;   /* 最大电流 mA */
} zdt_position_trapezoid_cmd_x_t;

/**
 * @brief CAN消息数组最大分包数
 * 每个CAN分包最多7字节载荷（第1字节为功能码）
 */
#define ZDT_MAX_PACKETS 4

/**
 * @brief 系统状态参数（X固件）
 * 基于 5.8.1 读取系统状态参数（X）
 */
typedef struct {
    uint16_t bus_voltage;      /* 总线电压 mV */
    uint16_t bus_current;      /* 总线电流 mA */
    uint16_t phase_current;    /* 相电流 mA */
    uint16_t encoder_raw;      /* 编码器原始值 */
    uint16_t encoder_linear;   /* 线性化编码器值 */
    uint32_t target_pos;       /* 目标位置 */
    uint16_t speed;            /* 转速 0.1RPM */
    uint32_t real_pos;         /* 实时位置 */
    uint32_t pos_error;        /* 位置误差 */
    int8_t temperature;        /* 温度 ℃ */
    zdt_homing_state_t homing_state;  /* 回零状态 */
    zdt_motor_state_t motor_state;    /* 电机状态 */
} zdt_system_state_x_t;

/**
 * @brief 力矩模式命令参数（X固件）
 * 基于 5.3.3 力矩模式控制（X）
 */
typedef struct {
    uint8_t addr;           /* 电机地址 */
    uint8_t sign;           /* 符号：00=CW, 01=CCW */
    uint16_t slope;          /* 斜率（加速度）mA/S */
    uint16_t current;        /* 电流 mA */
    uint8_t sync_flag;      /* 同步标志 */
} zdt_torque_cmd_x_t;

/**
 * @brief 力矩模式限速命令参数（X固件）
 * 基于 5.3.4 力矩模式限速控制（X）
 */
typedef struct {
    uint8_t addr;           /* 电机地址 */
    uint8_t sign;           /* 符号：00=CW, 01=CCW */
    uint16_t slope;          /* 斜率（加速度）mA/S */
    uint16_t current;        /* 电流 mA */
    uint8_t sync_flag;      /* 同步标志 */
    uint16_t max_speed;     /* 最大速度 0.1RPM */
} zdt_torque_limited_cmd_x_t;

/**
 * @brief 直通限速位置模式命令参数（X固件）
 * 基于 5.3.8 直通限速位置模式控制（X）
 */
typedef struct {
    uint8_t addr;           /* 电机地址 */
    uint8_t direction;      /* 方向：CW/CCW */
    uint16_t speed;         /* 速度 0.1RPM */
    uint32_t position;      /* 位置角度 0.1° */
    uint8_t motion_mode;    /* 运动模式 */
    uint8_t sync_flag;      /* 同步标志 */
} zdt_position_direct_cmd_x_t;

/**
 * @brief 直通限速位置模式限电流命令参数（X固件）
 * 基于 5.3.9 直通限速位置模式限电流控制（X）
 */
typedef struct {
    uint8_t addr;           /* 电机地址 */
    uint8_t direction;      /* 方向：CW/CCW */
    uint16_t speed;         /* 速度 0.1RPM */
    uint32_t position;      /* 位置角度 0.1° */
    uint8_t motion_mode;    /* 运动模式 */
    uint8_t sync_flag;      /* 同步标志 */
    uint16_t max_current;   /* 最大电流 mA */
} zdt_position_direct_limited_cmd_x_t;

/**
 * @brief 定时返回信息命令参数
 * 基于 5.5.1 定时返回信息命令（X42S/Y42）
 */
typedef struct {
    uint8_t addr;           /* 电机地址 */
    uint8_t info_code;      /* 信息功能码（要定时返回的命令码） */
    uint16_t interval_ms;   /* 定时时间（毫秒）0=停止 */
} zdt_timed_return_cmd_t;

/**
 * @brief 版本信息返回数据
 * 基于 5.5.2 读取固件版本和硬件版本
 */
typedef struct {
    uint16_t fw_version;     /* 固件版本 */
    uint8_t hw_series;     /* 硬件系列：0=X, 1=Y */
    uint8_t hw_type;        /* 硬件类型：20/28/35/42/57 */
    uint8_t hw_ver;         /* 硬件版本 */
} zdt_version_info_t;

/**
 * @brief 速度模式命令参数（Emm固件）
 * 基于 5.3.7 速度模式控制
 */
typedef struct {
    uint8_t addr;           /* 电机地址 */
    uint8_t direction;      /* 方向：CW/CCW */
    uint16_t speed;         /* 速度 RPM */
    uint8_t acceleration;   /* 加速度档位 0-255 */
    uint8_t sync_flag;      /* 同步标志 */
} zdt_velocity_cmd_emm_t;

/**
 * @brief 位置模式命令参数（Emm固件）
 * 基于 5.3.12 位置模式控制
 */
typedef struct {
    uint8_t addr;           /* 电机地址 */
    uint8_t direction;      /* 方向：CW/CCW */
    uint16_t speed;         /* 速度 RPM */
    uint8_t acceleration;   /* 加速度档位 0-255 */
    uint32_t pulse_count;   /* 脉冲数 */
    uint8_t motion_mode;    /* 运动模式 */
    uint8_t sync_flag;      /* 同步标志 */
} zdt_position_cmd_emm_t;

/**
 * @brief 回零参数
 * 基于 5.4.5/5.4.6 读取/修改回零参数
 */
typedef struct {
    uint8_t homing_mode;     /* 回零模式 */
    uint8_t direction;       /* 回零方向 */
    uint16_t speed;          /* 回零速度 RPM */
    uint32_t timeout_ms;     /* 回零超时时间 ms */
    uint16_t collision_speed; /* 碰撞回零检测转速 RPM */
    uint16_t collision_current; /* 碰撞回零检测电流 mA */
    uint16_t collision_time;  /* 碰撞回零检测时间 ms */
    uint8_t auto_power_on_homing; /* 上电自动触发回零使能 */
} zdt_homing_param_t;

/*==============================================================================
 * 函数声明 - 构造命令（基于 zdt-protocol.md 第5章）
 *============================================================================*/

/*------------------ 5.2 触发动作命令 ------------------*/

/**
 * @brief 构造编码器校准命令
 * 基于 5.2.1 触发编码器校准
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_calibrate_encoder(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造重启电机命令
 * 基于 5.2.2 重启电机
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_reboot_motor(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造位置清零命令
 * 基于 5.2.3 将当前位置角度清零
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_zero_position(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造清除保护命令
 * 基于 5.2.4 解除堵转/过热/过流保护
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_clear_protection(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造恢复出厂设置命令
 * 基于 5.2.5 恢复出厂设置
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_factory_reset(uint8_t addr, zdt_can_msg_t *msg);

/*------------------ 5.3 运动控制命令 ------------------*/

/**
 * @brief 构造电机使能命令
 * 基于 5.3.2 电机使能控制
 * @param addr 电机地址
 * @param enable true=使能, false=不使能
 * @param sync_flag 同步标志
 * @param msg 输出的CAN消息
 */
void zdt_cmd_motor_enable(uint8_t addr, bool enable, uint8_t sync_flag, zdt_can_msg_t *msg);

/**
 * @brief 构造速度模式命令（X固件）
 * 基于 5.3.6 速度模式限电流控制（X）
 * @param params 速度命令参数
 * @param msgs 输出的CAN消息数组（至少2个元素）
 * @return 返回使用的CAN消息数量（1或2）
 */
uint8_t zdt_cmd_velocity_mode_x(const zdt_velocity_cmd_x_t *params, zdt_can_msg_t *msgs);

/**
 * @brief 构造位置模式命令（X固件梯形曲线）
 * 基于 5.3.11 梯形曲线加减速位置模式限电流控制（X）
 * @param params 位置命令参数
 * @param msgs 输出的CAN消息数组（至少3个元素）
 * @return 返回使用的CAN消息数量（2或3）
 */
uint8_t zdt_cmd_position_trapezoid_x(const zdt_position_trapezoid_cmd_x_t *params, zdt_can_msg_t *msgs);

/**
 * @brief 构造力矩模式命令（X固件）
 * 基于 5.3.3 力矩模式控制（X）
 * @param params 力矩命令参数
 * @param msgs 输出的CAN消息数组（至少2个元素）
 * @return 返回使用的CAN消息数量（1或2）
 */
uint8_t zdt_cmd_torque_mode_x(const zdt_torque_cmd_x_t *params, zdt_can_msg_t *msgs);

/**
 * @brief 构造力矩模式限速命令（X固件）
 * 基于 5.3.4 力矩模式限速控制（X）
 * @param params 力矩限速命令参数
 * @param msgs 输出的CAN消息数组（至少2个元素）
 * @return 返回使用的CAN消息数量（1或2）
 */
uint8_t zdt_cmd_torque_limited_x(const zdt_torque_limited_cmd_x_t *params, zdt_can_msg_t *msgs);

/**
 * @brief 构造直通限速位置模式命令（X固件）
 * 基于 5.3.8 直通限速位置模式控制（X）
 * @param params 位置命令参数
 * @param msgs 输出的CAN消息数组（至少2个元素）
 * @return 返回使用的CAN消息数量（1或2）
 */
uint8_t zdt_cmd_position_direct_x(const zdt_position_direct_cmd_x_t *params, zdt_can_msg_t *msgs);

/**
 * @brief 构造直通限速位置模式限电流命令（X固件）
 * 基于 5.3.9 直通限速位置模式限电流控制（X）
 * @param params 位置限电流命令参数
 * @param msgs 输出的CAN消息数组（至少2个元素）
 * @return 返回使用的CAN消息数量（1或2）
 */
uint8_t zdt_cmd_position_direct_limited_x(const zdt_position_direct_limited_cmd_x_t *params, zdt_can_msg_t *msgs);

/**
 * @brief 构造立即停止命令
 * 基于 5.3.13 立即停止
 * @param addr 电机地址
 * @param sync_flag 同步标志
 * @param msg 输出的CAN消息
 */
void zdt_cmd_immediate_stop(uint8_t addr, uint8_t sync_flag, zdt_can_msg_t *msg);

/**
 * @brief 构造速度模式命令（Emm固件）
 * 基于 5.3.7 速度模式控制
 * @param params 速度命令参数
 * @param msg 输出的CAN消息
 */
void zdt_cmd_velocity_mode_emm(const zdt_velocity_cmd_emm_t *params, zdt_can_msg_t *msg);

/**
 * @brief 构造位置模式命令（Emm固件）
 * 基于 5.3.12 位置模式控制
 * @param params 位置命令参数
 * @param msgs 输出的CAN消息数组（至少2个元素）
 * @return 返回使用的CAN消息数量（1或2）
 */
uint8_t zdt_cmd_position_mode_emm(const zdt_position_cmd_emm_t *params, zdt_can_msg_t *msgs);

/**
 * @brief 构造多机同步运动触发命令
 * 基于 5.3.14 触发多机同步运动
 * @param msg 输出的CAN消息（使用广播地址0）
 */
void zdt_cmd_sync_trigger(zdt_can_msg_t *msg);

/*------------------ 5.4 原点回零命令 ------------------*/

/**
 * @brief 构造设置零点位置命令
 * 基于 5.4.1 设置单圈回零的零点位置
 * @param addr 电机地址
 * @param save 是否存储
 * @param msg 输出的CAN消息
 */
void zdt_cmd_set_zero_point(uint8_t addr, uint8_t save, zdt_can_msg_t *msg);

/**
 * @brief 构造触发回零命令
 * 基于 5.4.2 触发回零
 * @param addr 电机地址
 * @param homing_mode 回零模式
 * @param sync_flag 同步标志
 * @param msg 输出的CAN消息
 */
void zdt_cmd_trigger_homing(uint8_t addr, uint8_t homing_mode, uint8_t sync_flag, zdt_can_msg_t *msg);

/**
 * @brief 构造停止回零命令
 * 基于 5.4.3 强制中断并退出回零操作
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_stop_homing(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取回零参数命令
 * 基于 5.4.5 读取回零参数
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_homing_param(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造修改回零参数命令
 * 基于 5.4.6 修改回零参数
 * @param addr 电机地址
 * @param save 是否存储
 * @param params 回零参数
 * @param msgs 输出的CAN消息数组（至少3个元素）
 * @return 返回使用的CAN消息数量（2或3）
 */
uint8_t zdt_cmd_write_homing_param(uint8_t addr, uint8_t save,
                                  const zdt_homing_param_t *params,
                                  zdt_can_msg_t *msgs);

/*------------------ 5.5 读取系统参数命令 ------------------*/

/**
 * @brief 构造读取电机状态命令
 * 基于 5.5.15 读取电机状态标志
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_motor_state(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取回零状态命令
 * 基于 5.4.4 读取回零状态标志
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_homing_state(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取实时位置命令
 * 基于 5.5.13 读取电机实时位置
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_real_position(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取实时转速命令
 * 基于 5.5.11 读取电机实时转速
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_real_speed(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取相电流命令
 * 基于 5.5.6 读取相电流
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_phase_current(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造定时返回信息命令
 * 基于 5.5.1 定时返回信息命令（X42S/Y42）
 * @param addr 电机地址
 * @param info_code 信息功能码（要定时返回的命令码）
 * @param interval_ms 定时时间（毫秒）0=停止
 * @param msg 输出的CAN消息
 */
void zdt_cmd_timed_return(uint8_t addr, uint8_t info_code, uint16_t interval_ms, zdt_can_msg_t *msg);

/**
 * @brief 构造读取固件版本和硬件版本命令
 * 基于 5.5.2 读取固件版本和硬件版本
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_version(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取相电阻和相电感命令
 * 基于 5.5.3 读取相电阻和相电感
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_motor_param(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取总线电压命令
 * 基于 5.5.4 读取总线电压
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_bus_voltage(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取总线电流命令（X42S/Y42）
 * 基于 5.5.5 读取总线电流（X42S/Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_bus_current(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取线性化编码器值命令
 * 基于 5.5.7 读取线性化编码器值
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_encoder_linearized(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取输入脉冲数命令
 * 基于 5.5.8 读取输入脉冲数
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_pulse_count(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取电机目标位置命令
 * 基于 5.5.9 读取电机目标位置
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_target_position(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取电机实时设定目标位置命令
 * 基于 5.5.10 读取电机实时设定目标位置
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_set_position(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取驱动温度命令（X42S/Y42）
 * 基于 5.5.12 读取驱动温度（X42S/Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_driver_temp(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取电机位置误差命令
 * 基于 5.5.14 读取电机位置误差
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_position_error(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取回零状态+电机状态命令（X42S/Y42）
 * 基于 5.5.16 读取回零状态标志+电机状态标志（X42S/Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_homing_motor_state(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取引脚IO电平状态命令（X42S/Y42）
 * 基于 5.5.17 读取引脚IO电平状态（X42S/Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_io_state(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取电池电压命令（Y42）
 * 基于 5.5.18 读取电池电压（Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_battery_voltage(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造读取系统状态命令（X固件）
 * 基于 5.8.1 读取系统状态参数（X）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_system_state_x(uint8_t addr, zdt_can_msg_t *msg);

/*------------------ 5.6 读写驱动参数命令 ------------------*/

/**
 * @brief 构造修改电机ID/地址命令
 * 基于 5.6.1 修改电机ID/地址
 * @param addr 电机地址
 * @param save 是否存储
 * @param new_id 新的ID地址（1-255）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_motor_id(uint8_t addr, uint8_t save, uint8_t new_id, zdt_can_msg_t *msg);

/**
 * @brief 构造修改细分值命令
 * 基于 5.6.2 修改细分值
 * @param addr 电机地址
 * @param save 是否存储
 * @param subdivs 细分值（1-255，0=256）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_subdivs(uint8_t addr, uint8_t save, uint8_t subdivs, zdt_can_msg_t *msg);

/**
 * @brief 构造修改掉电标志命令
 * 基于 5.6.3 修改掉电标志
 * @param addr 电机地址
 * @param flag 掉电标志（0=清除，1=设置）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_power_lost_flag(uint8_t addr, uint8_t flag, zdt_can_msg_t *msg);

/**
 * @brief 构造读取选项参数状态命令（X42S/Y42）
 * 基于 5.6.4 读取选项参数状态（X42S/Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_option_param(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造修改电机类型命令
 * 基于 5.6.5 修改电机类型
 * @param addr 电机地址
 * @param save 是否存储
 * @param motor_type 电机类型（19=0.9°, 32=1.8°）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_motor_type(uint8_t addr, uint8_t save, uint8_t motor_type, zdt_can_msg_t *msg);

/**
 * @brief 构造修改固件类型命令
 * 基于 5.6.6 修改固件类型
 * @param addr 电机地址
 * @param save 是否存储
 * @param fw_type 固件类型（0=X, 1=Emm, 2=Emm狂暴）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_fw_type(uint8_t addr, uint8_t save, uint8_t fw_type, zdt_can_msg_t *msg);

/**
 * @brief 构造修改控制模式命令
 * 基于 5.6.7 修改开环/闭环控制模式
 * @param addr 电机地址
 * @param save 是否存储
 * @param mode 控制模式（0=开环, 1=闭环）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_control_mode(uint8_t addr, uint8_t save, uint8_t mode, zdt_can_msg_t *msg);

/**
 * @brief 构造修改电机运动正方向命令
 * 基于 5.6.8 修改电机运动正方向
 * @param addr 电机地址
 * @param save 是否存储
 * @param dir 方向（0=CW, 1=CCW）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_motor_dir(uint8_t addr, uint8_t save, uint8_t dir, zdt_can_msg_t *msg);

/**
 * @brief 构造修改锁定按键功能命令
 * 基于 5.6.9 修改锁定按键功能
 * @param addr 电机地址
 * @param save 是否存储
 * @param lock 是否锁定（0=解锁, 1=锁定）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_key_lock(uint8_t addr, uint8_t save, uint8_t lock, zdt_can_msg_t *msg);

/**
 * @brief 构造修改缩放输入命令（X固件）
 * 基于 5.6.10 修改命令位置角度是否继续缩小10倍输入（X）
 * @param addr 电机地址
 * @param save 是否存储
 * @param enable 是否使能（0=失能, 1=使能）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_scale_input_x(uint8_t addr, uint8_t save, uint8_t enable, zdt_can_msg_t *msg);

/**
 * @brief 构造修改缩放输入命令（Emm固件）
 * 基于 5.6.11 修改命令速度值是否缩小10倍输入
 * @param addr 电机地址
 * @param save 是否存储
 * @param enable 是否使能（0=失能, 1=使能）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_scale_input_emm(uint8_t addr, uint8_t save, uint8_t enable, zdt_can_msg_t *msg);

/**
 * @brief 构造修改开环模式工作电流命令
 * 基于 5.6.12 修改开环模式工作电流
 * @param addr 电机地址
 * @param save 是否存储
 * @param current_ma 工作电流（mA）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_open_loop_current(uint8_t addr, uint8_t save, uint16_t current_ma, zdt_can_msg_t *msg);

/**
 * @brief 构造修改闭环模式最大电流命令
 * 基于 5.6.13 修改闭环模式最大电流
 * @param addr 电机地址
 * @param save 是否存储
 * @param current_ma 最大电流（mA）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_closed_loop_current(uint8_t addr, uint8_t save, uint16_t current_ma, zdt_can_msg_t *msg);

/**
 * @brief 构造读取PID参数命令（X固件）
 * 基于 5.6.14 读取PID参数（X）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_pid_param(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造修改PID参数命令（X固件）
 * 基于 5.6.15 修改PID参数（X）
 * @param addr 电机地址
 * @param save 是否存储
 * @param trapezoid_kp 梯形曲线位置环Kp
 * @param direct_kp 直通限速位置环Kp
 * @param velocity_kp 速度环Kp
 * @param velocity_ki 速度环Ki
 * @param msgs 输出的CAN消息数组（至少3个元素）
 * @return 返回使用的CAN消息数量（2或3）
 */
uint8_t zdt_cmd_write_pid_param_x(uint8_t addr, uint8_t save,
                                uint32_t trapezoid_kp, uint32_t direct_kp,
                                uint16_t velocity_kp, uint16_t velocity_ki,
                                zdt_can_msg_t *msgs);

/**
 * @brief 构造读取PID参数命令（Emm固件）
 * 基于 5.6.16 读取PID参数
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_pid_param_emm(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造修改PID参数命令（Emm固件）
 * 基于 5.6.17 修改PID参数
 * @param addr 电机地址
 * @param save 是否存储
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param msgs 输出的CAN消息数组（至少3个元素）
 * @return 返回使用的CAN消息数量（2或3）
 */
uint8_t zdt_cmd_write_pid_param_emm(uint8_t addr, uint8_t save,
                                   uint32_t kp, uint32_t ki, uint32_t kd,
                                   zdt_can_msg_t *msgs);

/**
 * @brief 解析PID参数（X固件）
 * 基于 5.6.14 读取PID参数（X）
 * @param msg 接收到的CAN消息
 * @param trapezoid_kp 输出的梯形曲线位置环Kp
 * @param direct_kp 输出的直通限速位置环Kp
 * @param velocity_kp 输出的速度环Kp
 * @param velocity_ki 输出的速度环Ki
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_pid_param_x(const zdt_can_msg_t *msg,
                       uint32_t *trapezoid_kp, uint32_t *direct_kp,
                       uint16_t *velocity_kp, uint16_t *velocity_ki);

/**
 * @brief 解析PID参数（Emm固件）
 * 基于 5.6.16 读取PID参数
 * @param msg 接收到的CAN消息
 * @param kp 输出的比例系数
 * @param ki 输出的积分系数
 * @param kd 输出的微分系数
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_pid_param_emm(const zdt_can_msg_t *msg,
                            uint32_t *kp, uint32_t *ki, uint32_t *kd);

/**
 * @brief 构造读取位置到达窗口命令（X42S/Y42）
 * 基于 5.6.20 读取位置到达窗口（X42S/Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_pos_window(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造修改位置到达窗口命令
 * 基于 5.6.21 修改位置到达窗口（X42S/Y42）
 * @param addr 电机地址
 * @param save 是否存储
 * @param window 位置到达窗口值
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_pos_window(uint8_t addr, uint8_t save, uint16_t window, zdt_can_msg_t *msg);

/**
 * @brief 解析位置到达窗口
 * 基于 5.6.20 读取位置到达窗口（X42S/Y42）
 * @param msg 接收到的CAN消息
 * @param window 输出的位置到达窗口值
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_pos_window(const zdt_can_msg_t *msg, uint16_t *window);

/**
 * @brief 构造读取过热过流保护检测阈值命令（X42S/Y42）
 * 基于 5.6.22 读取过热过流保护检测阈值（X42S/Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_protect_threshold(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造修改过热过流保护检测阈值命令
 * 基于 5.6.23 修改过热过流保护检测阈值（X42S/Y42）
 * @param addr 电机地址
 * @param save 是否存储
 * @param temp_threshold 过热保护检测阈值（℃）
 * @param current_threshold 过流保护检测阈值（mA）
 * @param time_threshold 检测时间（ms）
 * @param msgs 输出的CAN消息数组（至少2个元素）
 * @return 返回使用的CAN消息数量（1或2）
 */
uint8_t zdt_cmd_write_protect_threshold(uint8_t addr, uint8_t save,
                                      uint16_t temp_threshold,
                                      uint16_t current_threshold,
                                      uint16_t time_threshold,
                                      zdt_can_msg_t *msgs);

/**
 * @brief 解析过热过流保护检测阈值
 * 基于 5.6.22 读取过热过流保护检测阈值（X42S/Y42）
 * @param msg 接收到的CAN消息
 * @param temp_threshold 输出的过热保护检测阈值（℃）
 * @param current_threshold 输出的过流保护检测阈值（mA）
 * @param time_threshold 输出的检测时间（ms）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_protect_threshold(const zdt_can_msg_t *msg,
                             uint16_t *temp_threshold, uint16_t *current_threshold,
                             uint16_t *time_threshold);

/**
 * @brief 构造读取心跳保护功能时间命令（X42S/Y42）
 * 基于 5.6.24 读取心跳保护功能时间（X42S/Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_heartbeat(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造修改心跳保护功能时间命令
 * 基于 5.6.25 修改心跳保护功能时间（X42S/Y42）
 * @param addr 电机地址
 * @param save 是否存储
 * @param time_ms 心跳保护时间（ms）
 * @param msgs 输出的CAN消息数组（至少2个元素）
 * @return 返回使用的CAN消息数量（1或2）
 */
uint8_t zdt_cmd_write_heartbeat(uint8_t addr, uint8_t save,
                                 uint32_t time_ms,
                                 zdt_can_msg_t *msgs);

/**
 * @brief 解析心跳保护功能时间
 * 基于 5.6.24 读取心跳保护功能时间（X42S/Y42）
 * @param msg 接收到的CAN消息
 * @param time_ms 输出的心跳保护时间（ms）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_heartbeat(const zdt_can_msg_t *msg, uint32_t *time_ms);

/**
 * @brief 构造读取积分限幅/刚性系数命令（X42S/Y42）
 * 基于 5.6.26 读取积分限幅/刚性系数（X42S/Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_integral_limit(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造修改积分限幅/刚性系数命令
 * 基于 5.6.27 修改积分限幅/刚性系数（X42S/Y42）
 * @param addr 电机地址
 * @param save 是否存储
 * @param limit 积分限幅/刚性系数值
 * @param msgs 输出的CAN消息数组（至少2个元素）
 * @return 返回使用的CAN消息数量（1或2）
 */
uint8_t zdt_cmd_write_integral_limit(uint8_t addr, uint8_t save,
                                   uint32_t limit,
                                   zdt_can_msg_t *msgs);

/**
 * @brief 解析积分限幅/刚性系数
 * 基于 5.6.26 读取积分限幅/刚性系数（X42S/Y42）
 * @param msg 接收到的CAN消息
 * @param limit 输出的积分限幅/刚性系数值
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_integral_limit(const zdt_can_msg_t *msg, uint32_t *limit);

/**
 * @brief 构造读取碰撞回零返回角度命令（X42S/Y42）
 * 基于 5.6.28 读取碰撞回零返回角度（X42S/Y42）
 * @param addr 电机地址
 * @param msg 输出的CAN消息
 */
void zdt_cmd_read_homing_angle(uint8_t addr, zdt_can_msg_t *msg);

/**
 * @brief 构造修改碰撞回零返回角度命令
 * 基于 5.6.29 修改碰撞回零返回角度（X42S/Y42）
 * @param addr 电机地址
 * @param save 是否存储
 * @param angle 碰撞回零返回角度值
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_homing_angle(uint8_t addr, uint8_t save, uint16_t angle, zdt_can_msg_t *msg);

/**
 * @brief 解析碰撞回零返回角度
 * 基于 5.6.28 读取碰撞回零返回角度（X42S/Y42）
 * @param msg 接收到的CAN消息
 * @param angle 输出的碰撞回零返回角度值
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_homing_angle(const zdt_can_msg_t *msg, uint16_t *angle);

/**
 * @brief 构造广播读取ID地址命令（X42S/Y42）
 * 基于 5.6.30 广播读取ID地址（X42S/Y42）
 * @param msg 输出的CAN消息（使用广播地址0）
 */
void zdt_cmd_broadcast_read_id(zdt_can_msg_t *msg);

/**
 * @brief 解析广播读取ID地址响应
 * 基于 5.6.30 广播读取ID地址（X42S/Y42）
 * @param msg 接收到的CAN消息
 * @param addr 输出的电机地址
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_broadcast_read_id(const zdt_can_msg_t *msg, uint8_t *addr);

/**
 * @brief 构造修改锁定修改参数功能命令（X42S/Y42）
 * 基于 5.6.31 修改锁定修改参数功能（X42S/Y42）
 * @param addr 电机地址
 * @param save 是否存储
 * @param lock_level 锁定参数等级（0-3）
 * @param msg 输出的CAN消息
 */
void zdt_cmd_write_param_lock(uint8_t addr, uint8_t save, uint8_t lock_level, zdt_can_msg_t *msg);

/**
 * @brief 解析回零参数
 * 基于 5.4.5 读取回零参数
 * @param msg 接收到的CAN消息
 * @param params 输出的回零参数
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_homing_param(const zdt_can_msg_t *msg, zdt_homing_param_t *params);

/*==============================================================================
 * 函数声明 - 解析返回（基于 zdt-protocol.md 第5章）
 *============================================================================*/

/**
 * @brief 验证电机返回是否正确
 * 基于 5.0.2 电机返回格式
 * @param msg 接收到的CAN消息
 * @param expected_cmd 期望的功能码
 * @return true=返回正确, false=返回错误
 */
bool zdt_verify_response(const zdt_can_msg_t *msg, uint8_t expected_cmd);

/**
 * @brief 解析返回码
 * 基于 5.0.2 电机返回格式
 * @param msg 接收到的CAN消息
 * @return 返回码（02/E2/EE/9F）
 */
uint8_t zdt_get_response_code(const zdt_can_msg_t *msg);

/**
 * @brief 检查是否返回成功
 * @param code 返回码
 * @return true=成功, false=失败
 */
static inline bool zdt_is_response_ok(uint8_t code) {
    return code == ZDT_RESPONSE_OK || code == ZDT_RESPONSE_COMPLETED;
}

/**
 * @brief 检查是否参数错误
 * @param code 返回码
 * @return true=参数错误
 */
static inline bool zdt_is_param_error(uint8_t code) {
    return code == ZDT_RESPONSE_PARAM_ERR;
}

/**
 * @brief 检查是否格式错误
 * @param code 返回码
 * @return true=格式错误
 */
static inline bool zdt_is_format_error(uint8_t code) {
    return code == ZDT_RESPONSE_FORMAT_ERR;
}

/**
 * @brief 解析电机状态标志
 * 基于 5.5.15 读取电机状态标志
 * @param msg 接收到的CAN消息
 * @param state 输出的电机状态
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_motor_state(const zdt_can_msg_t *msg, zdt_motor_state_t *state);

/**
 * @brief 解析回零状态标志
 * 基于 5.4.4 读取回零状态标志
 * @param msg 接收到的CAN消息
 * @param state 输出的回零状态
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_homing_state(const zdt_can_msg_t *msg, zdt_homing_state_t *state);

/**
 * @brief 解析实时位置
 * 基于 5.5.13 读取电机实时位置
 * @param msg 接收到的CAN消息
 * @param position 输出的位置值（X固件：0.1°, Emm固件：需转换）
 * @param sign 输出的符号（0=正, 1=负）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_real_position(const zdt_can_msg_t *msg, uint32_t *position, uint8_t *sign);

/**
 * @brief 解析实时转速
 * 基于 5.5.11 读取电机实时转速
 * @param msg 接收到的CAN消息
 * @param speed 输出的速度值（X固件：0.1RPM, Emm固件：RPM）
 * @param sign 输出的符号（0=正, 1=负）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_real_speed(const zdt_can_msg_t *msg, uint16_t *speed, uint8_t *sign);

/**
 * @brief 解析相电流
 * 基于 5.5.6 读取相电流
 * @param msg 接收到的CAN消息
 * @param current 输出的电流值（mA）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_phase_current(const zdt_can_msg_t *msg, uint16_t *current);

/**
 * @brief 解析版本信息
 * 基于 5.5.2 读取固件版本和硬件版本
 * @param msg 接收到的CAN消息
 * @param info 输出的版本信息
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_version_info(const zdt_can_msg_t *msg, zdt_version_info_t *info);

/**
 * @brief 解析相电阻和相电感
 * 基于 5.5.3 读取相电阻和相电感
 * @param msg 接收到的CAN消息
 * @param phase_resistance 输出的相电阻（mΩ）
 * @param phase_inductance 输出的相电感（uH）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_motor_param(const zdt_can_msg_t *msg, uint16_t *phase_resistance, uint16_t *phase_inductance);

/**
 * @brief 解析总线电压
 * 基于 5.5.4 读取总线电压
 * @param msg 接收到的CAN消息
 * @param voltage 输出的总线电压
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_bus_voltage(const zdt_can_msg_t *msg, uint16_t *voltage);

/**
 * @brief 解析总线电流
 * 基于 5.5.5 读取总线电流（X42S/Y42）
 * @param msg 接收到的CAN消息
 * @param current 输出的总线电流
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_bus_current(const zdt_can_msg_t *msg, uint16_t *current);

/**
 * @brief 解析线性化编码器值
 * 基于 5.5.7 读取线性化编码器值
 * @param msg 接收到的CAN消息
 * @param value 输出的线性化编码器值（0-65535）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_encoder_linearized(const zdt_can_msg_t *msg, uint16_t *value);

/**
 * @brief 解析输入脉冲数
 * 基于 5.5.8 读取输入脉冲数
 * @param msg 接收到的CAN消息
 * @param count 输出的脉冲数
 * @param sign 输出的符号（0=正, 1=负）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_pulse_count(const zdt_can_msg_t *msg, uint32_t *count, uint8_t *sign);

/**
 * @brief 解析电机目标位置
 * 基于 5.5.9 读取电机目标位置
 * @param msg 接收到的CAN消息
 * @param position 输出的位置值（X固件：0.1°, Emm固件：需转换）
 * @param sign 输出的符号（0=正, 1=负）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_target_position(const zdt_can_msg_t *msg, uint32_t *position, uint8_t *sign);

/**
 * @brief 解析电机实时设定目标位置
 * 基于 5.5.10 读取电机实时设定目标位置
 * @param msg 接收到的CAN消息
 * @param position 输出的位置值（X固件：0.1°, Emm固件：需转换）
 * @param sign 输出的符号（0=正, 1=负）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_set_position(const zdt_can_msg_t *msg, uint32_t *position, uint8_t *sign);

/**
 * @brief 解析驱动温度
 * 基于 5.5.12 读取驱动温度（X42S/Y42）
 * @param msg 接收到的CAN消息
 * @param temperature 输出的温度（℃）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_driver_temp(const zdt_can_msg_t *msg, int8_t *temperature);

/**
 * @brief 解析电机位置误差
 * 基于 5.5.14 读取电机位置误差
 * @param msg 接收到的CAN消息
 * @param error 输出的位置误差值（X固件：0.01°, Emm固件：需转换）
 * @param sign 输出的符号（0=正, 1=负）
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_position_error(const zdt_can_msg_t *msg, uint32_t *error, uint8_t *sign);

/**
 * @brief 解析回零状态+电机状态
 * 基于 5.5.16 读取回零状态标志+电机状态标志（X42S/Y42）
 * @param msg 接收到的CAN消息
 * @param homing 输出的回零状态
 * @param motor 输出的电机状态
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_homing_motor_state(const zdt_can_msg_t *msg, zdt_homing_state_t *homing, zdt_motor_state_t *motor);

/**
 * @brief 解析IO电平状态
 * 基于 5.5.17 读取引脚IO电平状态（X42S/Y42）
 * @param msg 接收到的CAN消息
 * @param io_state 输出的IO电平状态
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_io_state(const zdt_can_msg_t *msg, uint8_t *io_state);

/**
 * @brief 解析电池电压
 * 基于 5.5.18 读取电池电压（Y42）
 * @param msg 接收到的CAN消息
 * @param voltage 输出的电池电压
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_battery_voltage(const zdt_can_msg_t *msg, uint16_t *voltage);

/**
 * @brief 解析系统状态（X固件）
 * 基于 5.8.1 读取系统状态参数（X）
 * @param data CAN数据域指针
 * @param dlc 数据长度
 * @param state 输出的系统状态
 * @return true=解析成功, false=解析失败
 */
bool zdt_parse_system_state_x(const uint8_t *data, uint8_t dlc, zdt_system_state_x_t *state);

/*==============================================================================
 * 辅助函数
 *============================================================================*/

/**
 * @brief 计算XOR校验码
 * 基于 5.0.3 校验码类型
 * @param data 数据指针
 * @param len 数据长度
 * @return XOR校验值
 */
uint8_t zdt_calc_xor_checksum(const uint8_t *data, uint8_t len);

/**
 * @brief 计算CRC8校验码
 * 基于 5.0.3 校验码类型
 * @param data 数据指针
 * @param len 数据长度
 * @return CRC8校验值
 */
uint8_t zdt_calc_crc8(const uint8_t *data, uint8_t len);

/**
 * @brief 角度转换：X固件内部值转角度
 * X固件：角度值 = 值/10
 * @param value 内部值
 * @return 角度值（度）
 */
static inline float zdt_angle_from_x(uint32_t value) {
    return value / 10.0f;
}

/**
 * @brief 角度转换：角度转X固件内部值
 * X固件：值 = 角度*10
 * @param angle 角度值（度）
 * @return 内部值
 */
static inline uint32_t zdt_angle_to_x(float angle) {
    return (uint32_t)(angle * 10.0f);
}

/**
 * @brief 角度转换：Emm固件内部值转角度
 * Emm固件：角度值 = (值 * 360) / 65536
 * @param value 内部值（0-65535）
 * @return 角度值（度）
 */
static inline float zdt_angle_from_emm(uint16_t value) {
    return (value * 360.0f) / 65536.0f;
}

/**
 * @brief 角度转换：角度转Emm固件内部值
 * Emm固件：值 = (角度 * 65536) / 360
 * @param angle 角度值（度）
 * @return 内部值（0-65535）
 */
static inline uint16_t zdt_angle_to_emm(float angle) {
    return (uint16_t)((angle * 65536.0f) / 360.0f);
}

/**
 * @brief 速度转换：X固件内部值转速度
 * X固件：速度值 = 值/10
 * @param value 内部值
 * @return 速度值（RPM）
 */
static inline float zdt_speed_from_x(uint16_t value) {
    return value / 10.0f;
}

/**
 * @brief 速度转换：速度转X固件内部值
 * X固件：值 = 速度*10
 * @param speed 速度值（RPM）
 * @return 内部值
 */
static inline uint16_t zdt_speed_to_x(float speed) {
    return (uint16_t)(speed * 10.0f);
}

#ifdef __cplusplus
}
#endif

#endif /* LIBZDT_H */
