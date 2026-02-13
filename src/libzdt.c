/**
 * @file libzdt.c
 * @brief ZDT闭环步进电机通讯协议库实现
 *
 * 基于 zdt-protocol.md 第5章通讯命令实现
 */

#include "libzdt.h"
#include <string.h>
#include "esp_log.h"

/*==============================================================================
 * 命令构造函数实现 - 基于 zdt-protocol.md 第5章
 *============================================================================*/

/*------------------ 5.2 触发动作命令 ------------------*/

/**
 * @brief 构造编码器校准命令
 * 基于 5.2.1 触发编码器校准
 *
 * 主机发送格式：Addr + 06 + 45 + 6B
 */
void zdt_cmd_calibrate_encoder(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 3;
    msg->data[0] = ZDT_CMD_CALIBRATE_ENCODER;  /* 0x06 */
    msg->data[1] = ZDT_AUX_CALIBRATE;           /* 0x45 */
    msg->data[2] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造重启电机命令
 * 基于 5.2.2 重启电机
 *
 * 主机发送格式：Addr + 08 + 97 + 6B
 */
void zdt_cmd_reboot_motor(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 3;
    msg->data[0] = ZDT_CMD_REBOOT_MOTOR;        /* 0x08 */
    msg->data[1] = ZDT_AUX_REBOOT;              /* 0x97 */
    msg->data[2] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造位置清零命令
 * 基于 5.2.3 将当前位置角度清零
 *
 * 主机发送格式：Addr + 0A + 6D + 6B
 */
void zdt_cmd_zero_position(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 3;
    msg->data[0] = ZDT_CMD_ZERO_POSITION;       /* 0x0A */
    msg->data[1] = ZDT_AUX_ZERO;                /* 0x6D */
    msg->data[2] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造清除保护命令
 * 基于 5.2.4 解除堵转/过热/过流保护
 *
 * 主机发送格式：Addr + 0E + 52 + 6B
 */
void zdt_cmd_clear_protection(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 3;
    msg->data[0] = ZDT_CMD_CLEAR_PROTECTION;    /* 0x0E */
    msg->data[1] = ZDT_AUX_CLEAR_PROT;          /* 0x52 */
    msg->data[2] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造恢复出厂设置命令
 * 基于 5.2.5 恢复出厂设置
 *
 * 主机发送格式：Addr + 0F + 5F + 6B
 */
void zdt_cmd_factory_reset(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 3;
    msg->data[0] = ZDT_CMD_FACTORY_RESET;       /* 0x0F */
    msg->data[1] = ZDT_AUX_FACTORY;             /* 0x5F */
    msg->data[2] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/*------------------ 5.3 运动控制命令 ------------------*/

/**
 * @brief 构造电机使能命令
 * 基于 5.3.2 电机使能控制
 *
 * 主机发送格式：Addr + F3 + AB + 使能状态 + 同步标志 + 6B
 */
void zdt_cmd_motor_enable(uint8_t addr, bool enable, uint8_t sync_flag, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_MOTOR_ENABLE;        /* 0xF3 */
    msg->data[1] = ZDT_AUX_MOTOR_ENABLE;        /* 0xAB */
    msg->data[2] = enable ? 0x01 : 0x00;        /* 使能状态：01=使能, 00=不使能 */
    msg->data[3] = sync_flag;                   /* 同步标志：00=立即, 01=缓存 */
    msg->data[4] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造力矩模式命令（X固件）
 * 基于 5.3.3 力矩模式控制（X）
 *
 * 主机发送格式：Addr + F5 + 符号 + 斜率(H) + 斜率(L) + 电流(H) + 电流(L) + 同步标志 + 6B
 *
 * 载荷共8字节，1个CAN分包
 */
uint8_t zdt_cmd_torque_mode_x(const zdt_torque_cmd_x_t *params, zdt_can_msg_t *msgs)
{
    msgs[0].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_TORQUE_MODE;     /* 0xF5 */
    msgs[0].data[1] = params->sign;            /* 00=CW, 01=CCW */
    msgs[0].data[2] = (params->slope >> 8) & 0xFF;  /* 斜率高字节 */
    msgs[0].data[3] = params->slope & 0xFF;         /* 斜率低字节 */
    msgs[0].data[4] = (params->current >> 8) & 0xFF; /* 电流高字节 */
    msgs[0].data[5] = params->current & 0xFF;        /* 电流低字节 */
    msgs[0].data[6] = params->sync_flag;       /* 同步标志 */
    msgs[0].data[7] = ZDT_CHECKSUM_FIXED;         /* 0x6B */

    return 1;  /* 返回使用的CAN消息数量 */
}

/**
 * @brief 构造力矩模式限速命令（X固件）
 * 基于 5.3.4 力矩模式限速控制（X）
 *
 * 主机发送格式：Addr + C5 + 符号 + 斜率(H) + 斜率(L) + 电流(H) + 电流(L) + 同步标志 + 最大速度(H) + 最大速度(L) + 6B
 *
 * 载荷共10字节，需要2个CAN分包：
 * - 分包0：C5 + 符号 + 斜率(H) + 斜率(L) + 电流(H) + 电流(L) + 同步标志
 * - 分包1：C5 + 最大速度(H) + 最大速度(L) + 6B
 */
uint8_t zdt_cmd_torque_limited_x(const zdt_torque_limited_cmd_x_t *params, zdt_can_msg_t *msgs)
{
    /* 第1个分包 */
    msgs[0].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_TORQUE_LIMITED;     /* 0xC5 */
    msgs[0].data[1] = params->sign;            /* 00=CW, 01=CCW */
    msgs[0].data[2] = (params->slope >> 8) & 0xFF;  /* 斜率高字节 */
    msgs[0].data[3] = params->slope & 0xFF;         /* 斜率低字节 */
    msgs[0].data[4] = (params->current >> 8) & 0xFF; /* 电流高字节 */
    msgs[0].data[5] = params->current & 0xFF;        /* 电流低字节 */
    msgs[0].data[6] = params->sync_flag;       /* 同步标志 */
    msgs[0].data[7] = ZDT_CHECKSUM_FIXED;         /* 0x6B */

    /* 第2个分包 */
    msgs[1].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_1);
    msgs[1].dlc = 4;
    msgs[1].data[0] = ZDT_CMD_TORQUE_LIMITED;     /* 0xC5 */
    msgs[1].data[1] = (params->max_speed >> 8) & 0xFF; /* 最大速度高字节 */
    msgs[1].data[2] = params->max_speed & 0xFF;        /* 最大速度低字节 */
    msgs[1].data[3] = ZDT_CHECKSUM_FIXED;          /* 0x6B */

    return 2;  /* 返回使用的CAN消息数量 */
}

/**
 * @brief 构造直通限速位置模式命令（X固件）
 * 基于 5.3.8 直通限速位置模式控制（X）
 *
 * 主机发送格式：Addr + FB + 方向 + 速度(H) + 速度(L) + 位置(4) + 运动模式 + 同步标志 + 6B
 *
 * 载荷共9字节，需要2个CAN分包：
 * - 分包0：FB + 方向 + 速度(H) + 速度(L) + 速度(L) + 位置(4)
 * - 分包1：FB + 运动模式 + 同步标志 + 6B
 */
uint8_t zdt_cmd_position_direct_x(const zdt_position_direct_cmd_x_t *params, zdt_can_msg_t *msgs)
{
    /* 第1个分包 */
    msgs[0].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_POSITION_DIRECT;      /* 0xFB */
    msgs[0].data[1] = params->direction;            /* 00=CW, 01=CCW */
    msgs[0].data[2] = (params->speed >> 8) & 0xFF;  /* 速度高字节 */
    msgs[0].data[3] = params->speed & 0xFF;         /* 速度低字节 */
    /* 位置角度（4字节）*/
    msgs[0].data[4] = (params->position >> 24) & 0xFF; /* 位置字节3 */
    msgs[0].data[5] = (params->position >> 16) & 0xFF; /* 位置字节2 */
    msgs[0].data[6] = (params->position >> 8) & 0xFF;  /* 位置字节1 */
    msgs[0].data[7] = params->position & 0xFF;           /* 位置字节0 */

    /* 第2个分包 */
    msgs[1].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_1);
    msgs[1].dlc = 4;
    msgs[1].data[0] = ZDT_CMD_POSITION_DIRECT;      /* 0xFB */
    msgs[1].data[1] = params->motion_mode;         /* 运动模式 */
    msgs[1].data[2] = params->sync_flag;            /* 同步标志 */
    msgs[1].data[3] = ZDT_CHECKSUM_FIXED;            /* 0x6B */

    return 2;  /* 返回使用的CAN消息数量 */
}

/**
 * @brief 构造直通限速位置模式限电流命令（X固件）
 * 基于 5.3.9 直通限速位置模式限电流控制（X）
 *
 * 主机发送格式：Addr + CB + 方向 + 速度(H) + 速度(L) + 位置(4) + 运动模式 + 同步标志 + 最大电流(H) + 最大电流(L) + 6B
 *
 * 载荷共11字节，需要2个CAN分包：
 * - 分包0：CB + 方向 + 速度(H) + 速度(L) + 位置(4)
 * - 分包1：CB + 运动模式 + 同步标志 + 最大电流(H) + 最大电流(L) + 6B
 */
uint8_t zdt_cmd_position_direct_limited_x(const zdt_position_direct_limited_cmd_x_t *params, zdt_can_msg_t *msgs)
{
    /* 第1个分包 */
    msgs[0].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_POSITION_DIRECT_LIMITED; /* 0xCB */
    msgs[0].data[1] = params->direction;            /* 00=CW, 01=CCW */
    msgs[0].data[2] = (params->speed >> 8) & 0xFF;  /* 速度高字节 */
    msgs[0].data[3] = params->speed & 0xFF;         /* 速度低字节 */
    /* 位置角度（4字节）*/
    msgs[0].data[4] = (params->position >> 24) & 0xFF; /* 位置字节3 */
    msgs[0].data[5] = (params->position >> 16) & 0xFF; /* 位置字节2 */
    msgs[0].data[6] = (params->position >> 8) & 0xFF;  /* 位置字节1 */
    msgs[0].data[7] = params->position & 0xFF;           /* 位置字节0 */

    /* 第2个分包 */
    msgs[1].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_1);
    msgs[1].dlc = 5;
    msgs[1].data[0] = ZDT_CMD_POSITION_DIRECT_LIMITED; /* 0xCB */
    msgs[1].data[1] = params->motion_mode;         /* 运动模式 */
    msgs[1].data[2] = params->sync_flag;            /* 同步标志 */
    msgs[1].data[3] = (params->max_current >> 8) & 0xFF; /* 最大电流高字节 */
    msgs[1].data[4] = params->max_current & 0xFF;        /* 最大电流低字节 */
    msgs[1].data[5] = ZDT_CHECKSUM_FIXED;             /* 0x6B */

    return 2;  /* 返回使用的CAN消息数量 */
}

/**
 * @brief 构造速度模式命令（X固件）
 * 基于 5.3.6 速度模式限电流控制（X）
 *
 * 主机发送格式：Addr + C6 + 符号 + 加速度(H) + 加速度(L) + 速度(H) + 速度(L) + 同步标志 + 最大电流(H) + 最大电流(L) + 6B
 *
 * 载荷共9字节，需要2个CAN分包：
 * - 分包0：C6 + 符号 + 加速度(H) + 加速度(L) + 速度(H) + 速度(L) + 同步标志
 * - 分包1：C6 + 最大电流(H) + 最大电流(L) + 6B
 */
uint8_t zdt_cmd_velocity_mode_x(const zdt_velocity_cmd_x_t *params, zdt_can_msg_t *msgs)
{
    /* 第1个分包 */
    msgs[0].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_VELOCITY_LIMITED;    /* 0xC6 */
    msgs[0].data[1] = params->direction;           /* 00=CW, 01=CCW */
    msgs[0].data[2] = (params->acceleration >> 8) & 0xFF;  /* 加速度高字节 */
    msgs[0].data[3] = params->acceleration & 0xFF;         /* 加速度低字节 */
    msgs[0].data[4] = (params->speed >> 8) & 0xFF;         /* 速度高字节 */
    msgs[0].data[5] = params->speed & 0xFF;                /* 速度低字节 */
    msgs[0].data[6] = params->sync_flag;           /* 同步标志 */
    msgs[0].data[7] = ZDT_CHECKSUM_FIXED;          /* 0x6B */

    /* 第2个分包 */
    msgs[1].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_1);
    msgs[1].dlc = 4;
    msgs[1].data[0] = ZDT_CMD_VELOCITY_LIMITED;    /* 0xC6 */
    msgs[1].data[1] = (params->max_current >> 8) & 0xFF;   /* 最大电流高字节 */
    msgs[1].data[2] = params->max_current & 0xFF;          /* 最大电流低字节 */
    msgs[1].data[3] = ZDT_CHECKSUM_FIXED;          /* 0x6B */

    return 2;  /* 返回使用的CAN消息数量 */
}

/**
 * @brief 构造位置模式命令（X固件梯形曲线）
 * 基于 5.3.11 梯形曲线加减速位置模式限电流控制（X）
 *
 * 主机发送格式：Addr + CD + 方向 + 加速ACC(H) + 加速ACC(L) + 减速ACC(H) + 减速ACC(L) +
 *               最大速度(H) + 最大速度(L) + 位置角度 + 运动模式 + 同步标志 + 最大电流(H) + 最大电流(L) + 6B
 *
 * 载荷共16字节，需要3个CAN分包：
 * - 分包0：CD + 方向 + 加速ACC(H) + 加速ACC(L) + 减速ACC(H) + 减速ACC(L) + 最大速度(H)
 * - 分包1：CD + 最大速度(L) + 位置角度(4) + 运动模式
 * - 分包2：CD + 同步标志 + 最大电流(H) + 最大电流(L) + 6B
 */
uint8_t zdt_cmd_position_trapezoid_x(const zdt_position_trapezoid_cmd_x_t *params, zdt_can_msg_t *msgs)
{
    /* 第1个分包 */
    msgs[0].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_POSITION_TRAPEZOID_LIMITED;  /* 0xCD */
    msgs[0].data[1] = params->direction;           /* 00=CW, 01=CCW */
    msgs[0].data[2] = (params->accel_acc >> 8) & 0xFF;     /* 加速加速度高字节 */
    msgs[0].data[3] = params->accel_acc & 0xFF;           /* 加速加速度低字节 */
    msgs[0].data[4] = (params->decel_acc >> 8) & 0xFF;     /* 减速加速度高字节 */
    msgs[0].data[5] = params->decel_acc & 0xFF;           /* 减速加速度低字节 */
    msgs[0].data[6] = (params->max_speed >> 8) & 0xFF;     /* 最大速度高字节 */
    msgs[0].data[7] = ZDT_CHECKSUM_FIXED;          /* 0x6B */

    /* 第2个分包 */
    msgs[1].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_1);
    msgs[1].dlc = 8;
    msgs[1].data[0] = ZDT_CMD_POSITION_TRAPEZOID_LIMITED;  /* 0xCD */
    msgs[1].data[1] = params->max_speed & 0xFF;           /* 最大速度低字节 */
    msgs[1].data[2] = (params->position >> 24) & 0xFF;    /* 位置角度字节3 */
    msgs[1].data[3] = (params->position >> 16) & 0xFF;    /* 位置角度字节2 */
    msgs[1].data[4] = (params->position >> 8) & 0xFF;     /* 位置角度字节1 */
    msgs[1].data[5] = params->position & 0xFF;           /* 位置角度字节0 */
    msgs[1].data[6] = params->motion_mode;        /* 运动模式 */
    msgs[1].data[7] = ZDT_CHECKSUM_FIXED;          /* 0x6B */

    /* 第3个分包 */
    msgs[2].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_2);
    msgs[2].dlc = 5;
    msgs[2].data[0] = ZDT_CMD_POSITION_TRAPEZOID_LIMITED;  /* 0xCD */
    msgs[2].data[1] = params->sync_flag;          /* 同步标志 */
    msgs[2].data[2] = (params->max_current >> 8) & 0xFF;  /* 最大电流高字节 */
    msgs[2].data[3] = params->max_current & 0xFF;         /* 最大电流低字节 */
    msgs[2].data[4] = ZDT_CHECKSUM_FIXED;          /* 0x6B */

    return 3;  /* 返回使用的CAN消息数量 */
}

/**
 * @brief 构造立即停止命令
 * 基于 5.3.13 立即停止
 *
 * 主机发送格式：Addr + FE + 98 + 同步标志 + 6B
 */
void zdt_cmd_immediate_stop(uint8_t addr, uint8_t sync_flag, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 4;
    msg->data[0] = ZDT_CMD_IMMEDIATE_STOP;      /* 0xFE */
    msg->data[1] = ZDT_AUX_STOP;                /* 0x98 */
    msg->data[2] = sync_flag;                   /* 同步标志 */
    msg->data[3] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造速度模式命令（Emm固件）
 * 基于 5.3.7 速度模式控制（Emm）
 *
 * 主机发送格式：Addr + F6 + 方向 + 速度(2) + 加速度 + 同步标志 + 6B
 */
void zdt_cmd_velocity_mode_emm(const zdt_velocity_cmd_emm_t *params, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_0);
    msg->dlc = 7;
    msg->data[0] = ZDT_CMD_VELOCITY_MODE;      /* 0xF6 */
    msg->data[1] = params->direction;           /* 00=CW, 01=CCW */
    msg->data[2] = (params->speed >> 8) & 0xFF;
    msg->data[3] = params->speed & 0xFF;
    msg->data[4] = params->acceleration;
    msg->data[5] = params->sync_flag;
    msg->data[6] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造位置模式命令（Emm固件）
 * 基于 5.3.12 位置模式控制（Emm）
 *
 * 主机发送格式：Addr + FD + 方向 + 速度(2) + 加速度 + 脉冲数(4) + 运动模式 + 同步标志 + 6B
 *
 * 载荷共13字节，需要2个CAN分包
 */
uint8_t zdt_cmd_position_mode_emm(const zdt_position_cmd_emm_t *params, zdt_can_msg_t *msgs)
{
    /* 第1个分包：FD + 方向 + 速度(2) + 加速度 + 脉冲数(4) */
    msgs[0].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_POSITION_TRAPEZOID; /* Emm用FD */
    msgs[0].data[1] = params->direction;
    msgs[0].data[2] = (params->speed >> 8) & 0xFF;
    msgs[0].data[3] = params->speed & 0xFF;
    msgs[0].data[4] = params->acceleration;
    msgs[0].data[5] = (params->pulse_count >> 24) & 0xFF;
    msgs[0].data[6] = (params->pulse_count >> 16) & 0xFF;
    msgs[0].data[7] = (params->pulse_count >> 8) & 0xFF;

    /* 第2个分包：FD + 脉冲数(1) + 运动模式 + 同步标志 + 6B */
    msgs[1].id = ZDT_CAN_FRAME_ID(params->addr, ZDT_PACKET_1);
    msgs[1].dlc = 5;
    msgs[1].data[0] = ZDT_CMD_POSITION_TRAPEZOID;
    msgs[1].data[1] = params->pulse_count & 0xFF;
    msgs[1].data[2] = params->motion_mode;
    msgs[1].data[3] = params->sync_flag;
    msgs[1].data[4] = ZDT_CHECKSUM_FIXED;

    return 2;
}

/**
 * @brief 构造多机同步运动触发命令
 * 基于 5.3.14 触发多机同步运动
 *
 * 主机发送格式：00 + FF + 66 + 6B（使用广播地址0）
 */
void zdt_cmd_sync_trigger(zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(0x00, ZDT_PACKET_0);  /* 广播地址 */
    msg->dlc = 3;
    msg->data[0] = ZDT_CMD_SYNC_TRIGGER;     /* 0xFF */
    msg->data[1] = ZDT_AUX_SYNC;             /* 0x66 */
    msg->data[2] = ZDT_CHECKSUM_FIXED;       /* 0x6B */
}

/*------------------ 5.4 原点回零命令 ------------------*/

/**
 * @brief 构造设置零点位置命令
 * 基于 5.4.1 设置单圈回零的零点位置
 *
 * 主机发送格式：Addr + 93 + 88 + 是否存储 + 6B
 */
void zdt_cmd_set_zero_point(uint8_t addr, uint8_t save, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 4;
    msg->data[0] = ZDT_CMD_SET_ZERO_POINT;    /* 0x93 */
    msg->data[1] = ZDT_AUX_SET_ZERO;          /* 0x88 */
    msg->data[2] = save;                      /* 00=不存储, 01=存储 */
    msg->data[3] = ZDT_CHECKSUM_FIXED;        /* 0x6B */
}

/**
 * @brief 构造触发回零命令
 * 基于 5.4.2 触发回零
 *
 * 主机发送格式：Addr + 9A + 回零模式 + 同步标志 + 6B
 */
void zdt_cmd_trigger_homing(uint8_t addr, uint8_t homing_mode, uint8_t sync_flag, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 4;
    msg->data[0] = ZDT_CMD_TRIGGER_HOMING;    /* 0x9A */
    msg->data[1] = homing_mode;               /* 回零模式：00-05 */
    msg->data[2] = sync_flag;                 /* 同步标志 */
    msg->data[3] = ZDT_CHECKSUM_FIXED;        /* 0x6B */
}

/**
 * @brief 构造停止回零命令
 * 基于 5.4.3 强制中断并退出回零操作
 *
 * 主机发送格式：Addr + 9C + 48 + 6B
 */
void zdt_cmd_stop_homing(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 3;
    msg->data[0] = ZDT_CMD_STOP_HOMING;       /* 0x9C */
    msg->data[1] = ZDT_AUX_STOP_HOMING;       /* 0x48 */
    msg->data[2] = ZDT_CHECKSUM_FIXED;        /* 0x6B */
}

/**
 * @brief 构造读取回零参数命令
 * 基于 5.4.5 读取回零参数
 *
 * 主机发送格式：Addr + 22 + 6B
 */
void zdt_cmd_read_homing_param(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_HOMING_PARAM; /* 0x22 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/**
 * @brief 构造修改回零参数命令
 * 基于 5.4.6 修改回零参数
 *
 * 主机发送格式：Addr + 4C + AE + 是否存储 + 回零模式 + 回零方向 + 回零速度(2) +
 *               回零超时(4) + 碰撞转速(2) + 碰撞电流(2) + 碰撞时间(2) + O_POT_En + 6B
 *
 * 载荷共19字节，需要3个CAN分包
 */
uint8_t zdt_cmd_write_homing_param(uint8_t addr, uint8_t save,
                                   const zdt_homing_param_t *params,
                                   zdt_can_msg_t *msgs)
{
    /* 第1个分包：4C + AE + save + homing_mode + direction + speed(2) + timeout_byte0 */
    msgs[0].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_WRITE_HOMING_PARAM; /* 0x4C */
    msgs[0].data[1] = ZDT_AUX_WRITE_HOMING;       /* 0xAE */
    msgs[0].data[2] = save ? 0x01 : 0x00;
    msgs[0].data[3] = params->homing_mode;
    msgs[0].data[4] = params->direction;
    msgs[0].data[5] = (params->speed >> 8) & 0xFF;
    msgs[0].data[6] = params->speed & 0xFF;
    msgs[0].data[7] = (params->timeout_ms >> 24) & 0xFF;

    /* 第2个分包：4C + timeout(3) + collision_speed(2) + collision_current(2) */
    msgs[1].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_1);
    msgs[1].dlc = 8;
    msgs[1].data[0] = ZDT_CMD_WRITE_HOMING_PARAM; /* 0x4C */
    msgs[1].data[1] = (params->timeout_ms >> 16) & 0xFF;
    msgs[1].data[2] = (params->timeout_ms >> 8) & 0xFF;
    msgs[1].data[3] = params->timeout_ms & 0xFF;
    msgs[1].data[4] = (params->collision_speed >> 8) & 0xFF;
    msgs[1].data[5] = params->collision_speed & 0xFF;
    msgs[1].data[6] = (params->collision_current >> 8) & 0xFF;
    msgs[1].data[7] = params->collision_current & 0xFF;

    /* 第3个分包：4C + collision_time(2) + O_POT_En + 6B */
    msgs[2].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_2);
    msgs[2].dlc = 5;
    msgs[2].data[0] = ZDT_CMD_WRITE_HOMING_PARAM; /* 0x4C */
    msgs[2].data[1] = (params->collision_time >> 8) & 0xFF;
    msgs[2].data[2] = params->collision_time & 0xFF;
    msgs[2].data[3] = params->auto_power_on_homing ? 0x01 : 0x00;
    msgs[2].data[4] = ZDT_CHECKSUM_FIXED;          /* 0x6B */

    return 3;
}

/*------------------ 5.5 读取系统参数命令 ------------------*/

/**
 * @brief 构造读取电机状态命令
 * 基于 5.5.15 读取电机状态标志
 *
 * 主机发送格式：Addr + 3A + 6B
 */
void zdt_cmd_read_motor_state(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_MOTOR_STATE;   /* 0x3A */
    msg->data[1] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/**
 * @brief 构造读取回零状态命令
 * 基于 5.4.4 读取回零状态标志
 *
 * 主机发送格式：Addr + 3B + 6B
 */
void zdt_cmd_read_homing_state(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_HOMING_STATE;  /* 0x3B */
    msg->data[1] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/**
 * @brief 构造读取实时位置命令
 * 基于 5.5.13 读取电机实时位置
 *
 * 主机发送格式：Addr + 36 + 6B
 */
void zdt_cmd_read_real_position(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_REAL_POS;      /* 0x36 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/**
 * @brief 构造读取实时转速命令
 * 基于 5.5.11 读取电机实时转速
 *
 * 主机发送格式：Addr + 35 + 6B
 */
void zdt_cmd_read_real_speed(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_SPEED;         /* 0x35 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/**
 * @brief 构造读取相电流命令
 * 基于 5.5.6 读取相电流
 *
 * 主机发送格式：Addr + 27 + 6B
 */
void zdt_cmd_read_phase_current(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_PHASE_CURRENT; /* 0x27 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/**
 * @brief 构造定时返回信息命令
 * 基于 5.5.1 定时返回信息命令（X42S/Y42）
 *
 * 主机发送格式：Addr + 11 + 18 + 信息功能码 + 定时时间(H) + 定时时间(L) + 6B
 */
void zdt_cmd_timed_return(uint8_t addr, uint8_t info_code, uint16_t interval_ms, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 6;
    msg->data[0] = ZDT_CMD_TIMED_RETURN;      /* 0x11 */
    msg->data[1] = ZDT_AUX_TIMED_RETURN;     /* 0x18 */
    msg->data[2] = info_code;                 /* 信息功能码 */
    msg->data[3] = (interval_ms >> 8) & 0xFF;  /* 定时时间高字节 */
    msg->data[4] = interval_ms & 0xFF;          /* 定时时间低字节 */
    msg->data[5] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造读取固件版本和硬件版本命令
 * 基于 5.5.2 读取固件版本和硬件版本
 *
 * 主机发送格式：Addr + 1F + 6B
 */
void zdt_cmd_read_version(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_VERSION;      /* 0x1F */
    msg->data[1] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/**
 * @brief 构造读取相电阻和相电感命令
 * 基于 5.5.3 读取相电阻和相电感
 *
 * 主机发送格式：Addr + 20 + 6B
 */
void zdt_cmd_read_motor_param(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_MOTOR_PARAM;  /* 0x20 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/**
 * @brief 构造读取总线电压命令
 * 基于 5.5.4 读取总线电压
 *
 * 主机发送格式：Addr + 24 + 6B
 */
void zdt_cmd_read_bus_voltage(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_BUS_VOLTAGE; /* 0x24 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/**
 * @brief 构造读取总线电流命令（X42S/Y42）
 * 基于 5.5.5 读取总线电流（X42S/Y42）
 *
 * 主机发送格式：Addr + 26 + 6B
 */
void zdt_cmd_read_bus_current(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_BUS_CURRENT; /* 0x26 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/**
 * @brief 构造读取线性化编码器值命令
 * 基于 5.5.7 读取线性化编码器值
 *
 * 主机发送格式：Addr + 31 + 6B
 */
void zdt_cmd_read_encoder_linearized(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_ENCODER_LINEARIZED; /* 0x31 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;            /* 0x6B */
}

/**
 * @brief 构造读取输入脉冲数命令
 * 基于 5.5.8 读取输入脉冲数
 *
 * 主机发送格式：Addr + 32 + 6B
 */
void zdt_cmd_read_pulse_count(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_PULSE_COUNT;  /* 0x32 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造读取电机目标位置命令
 * 基于 5.5.9 读取电机目标位置
 *
 * 主机发送格式：Addr + 33 + 6B
 */
void zdt_cmd_read_target_position(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_TARGET_POS;  /* 0x33 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;           /* 0x6B */
}

/**
 * @brief 构造读取电机实时设定目标位置命令
 * 基于 5.5.10 读取电机实时设定目标位置
 *
 * 主机发送格式：Addr + 34 + 6B
 */
void zdt_cmd_read_set_position(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_SET_POS;     /* 0x34 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造读取驱动温度命令（X42S/Y42）
 * 基于 5.5.12 读取驱动温度（X42S/Y42）
 *
 * 主机发送格式：Addr + 39 + 6B
 */
void zdt_cmd_read_driver_temp(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_DRIVER_TEMP;  /* 0x39 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造读取电机位置误差命令
 * 基于 5.5.14 读取电机位置误差
 *
 * 主机发送格式：Addr + 37 + 6B
 */
void zdt_cmd_read_position_error(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_POS_ERROR;   /* 0x37 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造读取回零状态+电机状态命令（X42S/Y42）
 * 基于 5.5.16 读取回零状态标志+电机状态标志（X42S/Y42）
 *
 * 主机发送格式：Addr + 3C + 6B
 */
void zdt_cmd_read_homing_motor_state(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_HOMING_MOTOR_STATE; /* 0x3C */
    msg->data[1] = ZDT_CHECKSUM_FIXED;                 /* 0x6B */
}

/**
 * @brief 构造读取引脚IO电平状态命令（X42S/Y42）
 * 基于 5.5.17 读取引脚IO电平状态（X42S/Y42）
 *
 * 主机发送格式：Addr + 3D + 6B
 */
void zdt_cmd_read_io_state(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_IO_STATE;    /* 0x3D */
    msg->data[1] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造读取电池电压命令（Y42）
 * 基于 5.5.18 读取电池电压（Y42）
 *
 * 主机发送格式：Addr + 38 + 6B
 */
void zdt_cmd_read_battery_voltage(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_BATTERY_VOLTAGE; /* 0x38 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;             /* 0x6B */
}

/**
 * @brief 构造读取系统状态命令（X固件）
 * 基于 5.8.1 读取系统状态参数（X）
 *
 * 主机发送格式：Addr + 43 + 7A + 6B
 */
void zdt_cmd_read_system_state_x(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 3;
    msg->data[0] = ZDT_CMD_READ_SYSTEM_STATE;  /* 0x43 */
    msg->data[1] = 0x7A;                       /* 辅助码 */
    msg->data[2] = ZDT_CHECKSUM_FIXED;         /* 0x6B */
}

/*==============================================================================
 * 返回验证函数实现 - 基于 zdt-protocol.md 第5章
 *============================================================================*/

/**
 * @brief 验证电机返回是否正确
 * 基于 5.0.2 电机返回格式
 *
 * 电机返回格式：Addr + 功能码 + 返回数据 + 6B
 * 返回数据：02=正确, E2=参数错误, EE=格式错误, 9F=动作完成
 */
bool zdt_verify_response(const zdt_can_msg_t *msg, uint8_t expected_cmd)
{
    if (msg == NULL || msg->dlc < 3) {
        return false;
    }

    /* 检查校验码 */
    if (msg->data[msg->dlc - 1] != ZDT_CHECKSUM_FIXED) {
        return false;
    }

    /* 检查功能码是否匹配 */
    if (msg->data[0] != expected_cmd) {
        return false;
    }

    /* 检查返回码 */
    uint8_t response_code = msg->data[1];
    return zdt_is_response_ok(response_code);
}

/**
 * @brief 解析返回码
 * 基于 5.0.2 电机返回格式
 */
uint8_t zdt_get_response_code(const zdt_can_msg_t *msg)
{
    if (msg == NULL || msg->dlc < 3) {
        return ZDT_RESPONSE_FORMAT_ERR;
    }

    return msg->data[1];
}

/**
 * @brief 解析电机状态标志
 * 基于 5.5.15 读取电机状态标志
 *
 * 电机返回格式：Addr + 3A + 电机状态标志 + 6B
 * 电机状态标志(bit7-bit0)：Oac_TF 保留 Esi_RF Esi_LF Cgp_TF Cgi_TF Prf_TF Ens_TF
 */
bool zdt_parse_motor_state(const zdt_can_msg_t *msg, zdt_motor_state_t *state)
{
    if (msg == NULL || state == NULL || msg->dlc < 3) {
        ESP_LOGE("LIBZDT", "parse_motor_state: NULL check or DLC failed (dlc=%d)", msg ? msg->dlc : 0);
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_MOTOR_STATE) {
        ESP_LOGE("LIBZDT", "parse_motor_state: Command mismatch (0x%02X)", msg->data[0]);
        return false;
    }

    uint8_t flags = msg->data[1];
    state->ens_tf  = (flags & 0x01) ? 1 : 0;  /* bit0 */
    state->prf_tf  = (flags & 0x02) ? 1 : 0;  /* bit1 */
    state->cgi_tf  = (flags & 0x04) ? 1 : 0;  /* bit2 */
    state->cgp_tf  = (flags & 0x08) ? 1 : 0;  /* bit3 */
    state->esi_lf  = (flags & 0x10) ? 1 : 0;  /* bit4 */
    state->esi_rf  = (flags & 0x20) ? 1 : 0;  /* bit5 */
    state->reserved = (flags & 0x40) ? 1 : 0; /* bit6 */
    state->oac_tf  = (flags & 0x80) ? 1 : 0;  /* bit7 */

    ESP_LOGI("LIBZDT", "parse_motor_state: Success! flags=0x%02X, ens=%d, prf=%d",
             flags, state->ens_tf, state->prf_tf);
    return true;
}

/**
 * @brief 解析回零状态标志
 * 基于 5.4.4 读取回零状态标志
 *
 * 电机返回格式：Addr + 3B + 回零状态标志 + 6B
 * 回零状态标志(bit7-bit0)：保留 保留 Ocp_TF Otp_TF Org_CF Org_SF Cal_Rdy Enc_Rdy
 */
bool zdt_parse_homing_state(const zdt_can_msg_t *msg, zdt_homing_state_t *state)
{
    if (msg == NULL || state == NULL || msg->dlc < 4) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_HOMING_STATE) {
        return false;
    }

    uint8_t flags = msg->data[1];
    state->enc_rdy  = (flags & 0x01) ? 1 : 0;  /* bit0 */
    state->cal_rdy  = (flags & 0x02) ? 1 : 0;  /* bit1 */
    state->org_sf   = (flags & 0x04) ? 1 : 0;  /* bit2 */
    state->org_cf   = (flags & 0x08) ? 1 : 0;  /* bit3 */
    state->otp_tf   = (flags & 0x10) ? 1 : 0;  /* bit4 */
    state->ocp_tf   = (flags & 0x20) ? 1 : 0;  /* bit5 */
    state->reserved1 = (flags & 0x40) ? 1 : 0; /* bit6 */
    state->reserved2 = (flags & 0x80) ? 1 : 0; /* bit7 */

    return true;
}

/**
 * @brief 解析实时位置
 * 基于 5.5.13 读取电机实时位置
 *
 * 电机返回格式：Addr + 36 + 符号 + 位置(4字节) + 6B
 * X固件：角度值 = 位置/10
 * Emm固件：角度值 = (位置 * 360) / 65536
 */
bool zdt_parse_real_position(const zdt_can_msg_t *msg, uint32_t *position, uint8_t *sign)
{
    if (msg == NULL || position == NULL || sign == NULL || msg->dlc < 7) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_REAL_POS) {
        return false;
    }

    *sign = msg->data[1];  /* 符号：00=正, 01=负 */

    /* 位置数据：字节2-5，大端序 */
    *position = ((uint32_t)msg->data[2] << 24) |
                ((uint32_t)msg->data[3] << 16) |
                ((uint32_t)msg->data[4] << 8) |
                (uint32_t)msg->data[5];

    return true;
}

/**
 * @brief 解析实时转速
 * 基于 5.5.11 读取电机实时转速
 *
 * 电机返回格式：Addr + 35 + 符号 + 转速(2字节) + 6B
 * X固件：速度值 = 转速/10
 * Emm固件：速度值 = 转速
 */
bool zdt_parse_real_speed(const zdt_can_msg_t *msg, uint16_t *speed, uint8_t *sign)
{
    if (msg == NULL || speed == NULL || sign == NULL || msg->dlc < 5) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_SPEED) {
        return false;
    }

    *sign = msg->data[1];  /* 符号：00=正, 01=负 */

    /* 转速数据：字节2-3，大端序 */
    *speed = ((uint16_t)msg->data[2] << 8) | (uint16_t)msg->data[3];

    return true;
}

/**
 * @brief 解析相电流
 * 基于 5.5.6 读取相电流
 *
 * 电机返回格式：Addr + 27 + 相电流(2字节) + 6B
 * 单位：mA
 */
bool zdt_parse_phase_current(const zdt_can_msg_t *msg, uint16_t *current)
{
    if (msg == NULL || current == NULL || msg->dlc < 4) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_PHASE_CURRENT) {
        return false;
    }

    /* 相电流数据：字节1-2，大端序 */
    *current = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[2];

    return true;
}

/**
 * @brief 解析系统状态（X固件）
 * 基于 5.8.1 读取系统状态参数（X）
 *
 * 电机返回格式：共37字节
 * Addr + 43 + 25 + 0C + 总线电压(2) + 总线电流(2) + 相电流(2) + 编码器原始值(2) +
 * 线性化编码器值(2) + 符号1 + 目标位置(4) + 符号2 + 转速(2) + 符号3 + 实时位置(4) +
 * 符号4 + 位置误差(4) + 符号5 + 温度 + 回零状态 + 电机状态 + 6B
 */
bool zdt_parse_system_state_x(const uint8_t *data, uint8_t dlc, zdt_system_state_x_t *state)
{
    if (data == NULL || state == NULL || dlc < 37) {
        return false;
    }

    if (data[1] != ZDT_CMD_READ_SYSTEM_STATE) {
        return false;
    }

    /* 跳过：地址(0) + 功能码(1) + 字节数(2) + 参数个数(3) */
    uint8_t offset = 4;

    /* 总线电压 mV */
    state->bus_voltage = ((uint16_t)data[offset] << 8) | (uint16_t)data[offset + 1];
    offset += 2;

    /* 总线电流 mA */
    state->bus_current = ((uint16_t)data[offset] << 8) | (uint16_t)data[offset + 1];
    offset += 2;

    /* 相电流 mA */
    state->phase_current = ((uint16_t)data[offset] << 8) | (uint16_t)data[offset + 1];
    offset += 2;

    /* 编码器原始值 */
    state->encoder_raw = ((uint16_t)data[offset] << 8) | (uint16_t)data[offset + 1];
    offset += 2;

    /* 线性化编码器值 */
    state->encoder_linear = ((uint16_t)data[offset] << 8) | (uint16_t)data[offset + 1];
    offset += 2;

    /* 符号1（跳过）*/
    offset += 1;

    /* 目标位置 */
    state->target_pos = ((uint32_t)data[offset] << 24) |
                       ((uint32_t)data[offset + 1] << 16) |
                       ((uint32_t)data[offset + 2] << 8) |
                       (uint32_t)data[offset + 3];
    offset += 4;

    /* 符号2（跳过）*/
    offset += 1;

    /* 转速 0.1RPM */
    state->speed = ((uint16_t)data[offset] << 8) | (uint16_t)data[offset + 1];
    offset += 2;

    /* 符号3（跳过）*/
    offset += 1;

    /* 实时位置 */
    state->real_pos = ((uint32_t)data[offset] << 24) |
                      ((uint32_t)data[offset + 1] << 16) |
                      ((uint32_t)data[offset + 2] << 8) |
                      (uint32_t)data[offset + 3];
    offset += 4;

    /* 符号4（跳过）*/
    offset += 1;

    /* 位置误差 */
    state->pos_error = ((uint32_t)data[offset] << 24) |
                       ((uint32_t)data[offset + 1] << 16) |
                       ((uint32_t)data[offset + 2] << 8) |
                       (uint32_t)data[offset + 3];
    offset += 4;

    /* 符号5和温度 */
    offset += 1;  /* 符号5 */
    state->temperature = (int8_t)data[offset];  /* 温度 */
    offset += 1;

    /* 回零状态标志 */
    uint8_t homing_flags = data[offset];
    state->homing_state.enc_rdy  = (homing_flags & 0x01) ? 1 : 0;
    state->homing_state.cal_rdy  = (homing_flags & 0x02) ? 1 : 0;
    state->homing_state.org_sf   = (homing_flags & 0x04) ? 1 : 0;
    state->homing_state.org_cf   = (homing_flags & 0x08) ? 1 : 0;
    state->homing_state.otp_tf   = (homing_flags & 0x10) ? 1 : 0;
    state->homing_state.ocp_tf   = (homing_flags & 0x20) ? 1 : 0;
    state->homing_state.reserved1 = (homing_flags & 0x40) ? 1 : 0;
    state->homing_state.reserved2 = (homing_flags & 0x80) ? 1 : 0;
    offset += 1;

    /* 电机状态标志 */
    uint8_t motor_flags = data[offset];
    state->motor_state.ens_tf  = (motor_flags & 0x01) ? 1 : 0;
    state->motor_state.prf_tf  = (motor_flags & 0x02) ? 1 : 0;
    state->motor_state.cgi_tf  = (motor_flags & 0x04) ? 1 : 0;
    state->motor_state.cgp_tf  = (motor_flags & 0x08) ? 1 : 0;
    state->motor_state.esi_lf  = (motor_flags & 0x10) ? 1 : 0;
    state->motor_state.esi_rf  = (motor_flags & 0x20) ? 1 : 0;
    state->motor_state.reserved = (motor_flags & 0x40) ? 1 : 0;
    state->motor_state.oac_tf  = (motor_flags & 0x80) ? 1 : 0;

    return true;
}

/**
 * @brief 解析回零参数
 * 基于 5.4.5 读取回零参数
 *
 * 电机返回格式：Addr + 22 + 回零模式 + 回零方向 + 回零速度(2) + 回零超时(4) +
 *               碰撞转速(2) + 碰撞电流(2) + 碰撞时间(2) + O_POT_En + 6B
 *
 * 注意：多分包返回需由调用方重组后传入，msg->dlc >= 17
 */
bool zdt_parse_homing_param(const zdt_can_msg_t *msg, zdt_homing_param_t *params)
{
    if (msg == NULL || params == NULL || msg->dlc < 17) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_HOMING_PARAM) {
        return false;
    }

    params->homing_mode = msg->data[1];
    params->direction = msg->data[2];
    params->speed = ((uint16_t)msg->data[3] << 8) | (uint16_t)msg->data[4];
    params->timeout_ms = ((uint32_t)msg->data[5] << 24) |
                        ((uint32_t)msg->data[6] << 16) |
                        ((uint32_t)msg->data[7] << 8) |
                        (uint32_t)msg->data[8];
    params->collision_speed = ((uint16_t)msg->data[9] << 8) | (uint16_t)msg->data[10];
    params->collision_current = ((uint16_t)msg->data[11] << 8) | (uint16_t)msg->data[12];
    params->collision_time = ((uint16_t)msg->data[13] << 8) | (uint16_t)msg->data[14];
    params->auto_power_on_homing = msg->data[15] ? 1 : 0;

    return true;
}

/**
 * @brief 解析版本信息
 * 基于 5.5.2 读取固件版本和硬件版本
 *
 * 电机返回格式：Addr + 1F + FW_Ver(H) + FW_Ver(L) + HW(H) + 6B
 */
bool zdt_parse_version_info(const zdt_can_msg_t *msg, zdt_version_info_t *info)
{
    ESP_LOGI("LIBZDT", "parse_version_info: msg=%p, info=%p, dlc=%d", msg, info, msg ? msg->dlc : 0);

    if (msg == NULL || info == NULL || msg->dlc < 6) {
        ESP_LOGE("LIBZDT", "parse_version_info: NULL check or DLC failed (dlc=%d)", msg ? msg->dlc : 0);
        return false;
    }

    ESP_LOGI("LIBZDT", "parse_version_info: data[0]=0x%02X, expected=0x1F", msg->data[0]);
    if (msg->data[0] != ZDT_CMD_READ_VERSION) {
        ESP_LOGE("LIBZDT", "parse_version_info: Command mismatch");
        return false;
    }

    info->fw_version = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[2];
    uint8_t hw = msg->data[3];
    info->hw_series = (hw >> 6) & 0x01;     /* bit7 */
    info->hw_type = (hw >> 3) & 0x07;       /* bit4-6 */
    info->hw_ver = hw & 0x07;                /* bit2-0 */

    ESP_LOGI("LIBZDT", "parse_version_info: Success! FW=%04X, HW=%02X", info->fw_version, hw);
    return true;
}

/**
 * @brief 解析相电阻和相电感
 * 基于 5.5.3 读取相电阻和相电感
 *
 * 电机返回格式：Addr + 20 + Pha_R(H) + Pha_R(L) + Pha_L(H) + Pha_L(L) + 6B
 */
bool zdt_parse_motor_param(const zdt_can_msg_t *msg, uint16_t *phase_resistance, uint16_t *phase_inductance)
{
    if (msg == NULL || phase_resistance == NULL || phase_inductance == NULL || msg->dlc < 6) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_MOTOR_PARAM) {
        return false;
    }

    *phase_resistance = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[2];  /* mΩ */
    *phase_inductance = ((uint16_t)msg->data[3] << 8) | (uint16_t)msg->data[4];  /* uH */

    return true;
}

/**
 * @brief 解析总线电压
 * 基于 5.5.4 读取总线电压
 *
 * 电机返回格式：Addr + 24 + VBus(H) + VBus(L) + 6B
 */
bool zdt_parse_bus_voltage(const zdt_can_msg_t *msg, uint16_t *voltage)
{
    if (msg == NULL || voltage == NULL || msg->dlc < 4) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_BUS_VOLTAGE) {
        return false;
    }

    *voltage = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[2];  /* mV */

    return true;
}

/**
 * @brief 解析总线电流
 * 基于 5.5.5 读取总线电流（X42S/Y42）
 *
 * 电机返回格式：Addr + 26 + CBus(H) + CBus(L) + 6B
 */
bool zdt_parse_bus_current(const zdt_can_msg_t *msg, uint16_t *current)
{
    if (msg == NULL || current == NULL || msg->dlc < 5) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_BUS_CURRENT) {
        return false;
    }

    *current = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[2];  /* mA */

    return true;
}

/**
 * @brief 解析线性化编码器值
 * 基于 5.5.7 读取线性化编码器值
 *
 * 电机返回格式：Addr + 31 + 编码器值(H) + 编码器值(L) + 6B
 */
bool zdt_parse_encoder_linearized(const zdt_can_msg_t *msg, uint16_t *value)
{
    if (msg == NULL || value == NULL || msg->dlc < 5) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_ENCODER_LINEARIZED) {
        return false;
    }

    *value = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[2];  /* 0-65535 */

    return true;
}

/**
 * @brief 解析输入脉冲数
 * 基于 5.5.8 读取输入脉冲数
 *
 * 电机返回格式：Addr + 32 + 符号 + 输入脉冲数(4) + 6B
 */
bool zdt_parse_pulse_count(const zdt_can_msg_t *msg, uint32_t *count, uint8_t *sign)
{
    if (msg == NULL || count == NULL || sign == NULL || msg->dlc < 7) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_PULSE_COUNT) {
        return false;
    }

    *sign = msg->data[1];  /* 00=正, 01=负 */
    *count = ((uint32_t)msg->data[2] << 24) |
             ((uint32_t)msg->data[3] << 16) |
             ((uint32_t)msg->data[4] << 8) |
             (uint32_t)msg->data[5];

    return true;
}

/**
 * @brief 解析电机目标位置
 * 基于 5.5.9 读取电机目标位置
 *
 * 电机返回格式：Addr + 33 + 符号 + 目标位置(4) + 6B
 */
bool zdt_parse_target_position(const zdt_can_msg_t *msg, uint32_t *position, uint8_t *sign)
{
    if (msg == NULL || position == NULL || sign == NULL || msg->dlc < 7) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_TARGET_POS) {
        return false;
    }

    *sign = msg->data[1];  /* 00=正, 01=负 */
    *position = ((uint32_t)msg->data[2] << 24) |
                ((uint32_t)msg->data[3] << 16) |
                ((uint32_t)msg->data[4] << 8) |
                (uint32_t)msg->data[5];

    return true;
}

/**
 * @brief 解析电机实时设定目标位置
 * 基于 5.5.10 读取电机实时设定目标位置
 *
 * 电机返回格式：Addr + 34 + 符号 + 设定目标位置(4) + 6B
 */
bool zdt_parse_set_position(const zdt_can_msg_t *msg, uint32_t *position, uint8_t *sign)
{
    if (msg == NULL || position == NULL || sign == NULL || msg->dlc < 7) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_SET_POS) {
        return false;
    }

    *sign = msg->data[1];  /* 00=正, 01=负 */
    *position = ((uint32_t)msg->data[2] << 24) |
                ((uint32_t)msg->data[3] << 16) |
                ((uint32_t)msg->data[4] << 8) |
                (uint32_t)msg->data[5];

    return true;
}

/**
 * @brief 解析驱动温度
 * 基于 5.5.12 读取驱动温度（X42S/Y42）
 *
 * 电机返回格式：Addr + 39 + 温度符号 + 温度 + 6B
 */
bool zdt_parse_driver_temp(const zdt_can_msg_t *msg, int8_t *temperature)
{
    if (msg == NULL || temperature == NULL || msg->dlc < 5) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_DRIVER_TEMP) {
        return false;
    }

    int8_t temp = (int8_t)msg->data[2];
    *temperature = (msg->data[1] == 0x01) ? -temp : temp;

    return true;
}

/**
 * @brief 解析电机位置误差
 * 基于 5.5.14 读取电机位置误差
 *
 * 电机返回格式：Addr + 37 + 符号 + 位置误差(4) + 6B
 */
bool zdt_parse_position_error(const zdt_can_msg_t *msg, uint32_t *error, uint8_t *sign)
{
    if (msg == NULL || error == NULL || sign == NULL || msg->dlc < 7) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_POS_ERROR) {
        return false;
    }

    *sign = msg->data[1];  /* 00=正, 01=负 */
    *error = ((uint32_t)msg->data[2] << 24) |
             ((uint32_t)msg->data[3] << 16) |
             ((uint32_t)msg->data[4] << 8) |
             (uint32_t)msg->data[5];

    return true;
}

/**
 * @brief 解析回零状态+电机状态
 * 基于 5.5.16 读取回零状态标志+电机状态标志（X42S/Y42）
 *
 * 电机返回格式：Addr + 3C + 回零状态标志 + 电机状态标志 + 6B
 */
bool zdt_parse_homing_motor_state(const zdt_can_msg_t *msg, zdt_homing_state_t *homing, zdt_motor_state_t *motor)
{
    if (msg == NULL || homing == NULL || motor == NULL || msg->dlc < 5) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_HOMING_MOTOR_STATE) {
        return false;
    }

    uint8_t homing_flags = msg->data[1];
    homing->enc_rdy = (homing_flags & 0x01) ? 1 : 0;
    homing->cal_rdy = (homing_flags & 0x02) ? 1 : 0;
    homing->org_sf  = (homing_flags & 0x04) ? 1 : 0;
    homing->org_cf  = (homing_flags & 0x08) ? 1 : 0;
    homing->otp_tf  = (homing_flags & 0x10) ? 1 : 0;
    homing->ocp_tf  = (homing_flags & 0x20) ? 1 : 0;
    homing->reserved1 = (homing_flags & 0x40) ? 1 : 0;
    homing->reserved2 = (homing_flags & 0x80) ? 1 : 0;

    uint8_t motor_flags = msg->data[2];
    motor->ens_tf  = (motor_flags & 0x01) ? 1 : 0;
    motor->prf_tf  = (motor_flags & 0x02) ? 1 : 0;
    motor->cgi_tf  = (motor_flags & 0x04) ? 1 : 0;
    motor->cgp_tf  = (motor_flags & 0x08) ? 1 : 0;
    motor->esi_lf  = (motor_flags & 0x10) ? 1 : 0;
    motor->esi_rf  = (motor_flags & 0x20) ? 1 : 0;
    motor->reserved = (motor_flags & 0x40) ? 1 : 0;
    motor->oac_tf  = (motor_flags & 0x80) ? 1 : 0;

    return true;
}

/**
 * @brief 解析电池电压
 * 基于 5.5.18 读取电池电压（Y42）
 *
 * 电机返回格式：Addr + 38 + VBat(H) + VBat(L) + 6B
 */
bool zdt_parse_battery_voltage(const zdt_can_msg_t *msg, uint16_t *voltage)
{
    if (msg == NULL || voltage == NULL || msg->dlc < 5) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_BATTERY_VOLTAGE) {
        return false;
    }

    *voltage = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[2];  /* mV */

    return true;
}

/**
 * @brief 构造读取选项参数状态命令（X42S/Y42）
 * 基于 5.6.4 读取选项参数状态（X42S/Y42）
 *
 * 主机发送格式：Addr + 1A + 6B
 */
void zdt_cmd_read_option_param(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_OPTION_PARAM; /* 0x1A */
    msg->data[1] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造修改电机ID/地址命令
 * 基于 5.6.1 修改电机ID/地址
 */
void zdt_cmd_write_motor_id(uint8_t addr, uint8_t save, uint8_t new_id, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_WRITE_MOTOR_ID;
    msg->data[1] = ZDT_AUX_WRITE_ID;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = new_id;
    msg->data[4] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改细分值命令
 * 基于 5.6.2 修改细分值
 */
void zdt_cmd_write_subdivs(uint8_t addr, uint8_t save, uint8_t subdivs, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_WRITE_SUBDIVS;
    msg->data[1] = ZDT_AUX_WRITE_SUBDIVS;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = subdivs;
    msg->data[4] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改掉电标志命令
 * 基于 5.6.3 修改掉电标志（无是否存储参数）
 */
void zdt_cmd_write_power_lost_flag(uint8_t addr, uint8_t flag, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 3;
    msg->data[0] = ZDT_CMD_WRITE_POWER_LOST_FLAG;
    msg->data[1] = flag ? 0x01 : 0x00;
    msg->data[2] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改电机类型命令
 * 基于 5.6.5 修改电机类型
 */
void zdt_cmd_write_motor_type(uint8_t addr, uint8_t save, uint8_t motor_type, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_WRITE_MOTOR_TYPE;
    msg->data[1] = ZDT_AUX_WRITE_MOTOR_TYPE;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = motor_type;
    msg->data[4] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改固件类型命令
 * 基于 5.6.6 修改固件类型
 */
void zdt_cmd_write_fw_type(uint8_t addr, uint8_t save, uint8_t fw_type, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_WRITE_FW_TYPE;
    msg->data[1] = ZDT_AUX_WRITE_FW_TYPE;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = fw_type;
    msg->data[4] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改控制模式命令
 * 基于 5.6.7 修改开环/闭环控制模式
 */
void zdt_cmd_write_control_mode(uint8_t addr, uint8_t save, uint8_t mode, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_WRITE_CONTROL_MODE;
    msg->data[1] = ZDT_AUX_WRITE_CTRL_MODE;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = mode;
    msg->data[4] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改电机运动正方向命令
 * 基于 5.6.8 修改电机运动正方向
 */
void zdt_cmd_write_motor_dir(uint8_t addr, uint8_t save, uint8_t dir, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_WRITE_MOTOR_DIR;
    msg->data[1] = ZDT_AUX_WRITE_MOTOR_DIR;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = dir;
    msg->data[4] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改锁定按键功能命令
 * 基于 5.6.9 修改锁定按键功能
 */
void zdt_cmd_write_key_lock(uint8_t addr, uint8_t save, uint8_t lock, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_WRITE_KEY_LOCK;
    msg->data[1] = ZDT_AUX_WRITE_KEY_LOCK;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = lock ? 0x01 : 0x00;
    msg->data[4] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改缩放输入命令（X固件）
 * 基于 5.6.10 修改命令位置角度是否继续缩小10倍输入（X）
 */
void zdt_cmd_write_scale_input_x(uint8_t addr, uint8_t save, uint8_t enable, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_WRITE_SCALE_INPUT;
    msg->data[1] = ZDT_AUX_WRITE_SCALE;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = enable ? 0x01 : 0x00;
    msg->data[4] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改缩放输入命令（Emm固件）
 * 基于 5.6.11 修改命令速度值是否缩小10倍输入（Emm）
 */
void zdt_cmd_write_scale_input_emm(uint8_t addr, uint8_t save, uint8_t enable, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_WRITE_SCALE_INPUT;
    msg->data[1] = ZDT_AUX_WRITE_SCALE;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = enable ? 0x01 : 0x00;
    msg->data[4] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改开环模式工作电流命令
 * 基于 5.6.12 修改开环模式工作电流
 */
void zdt_cmd_write_open_loop_current(uint8_t addr, uint8_t save, uint16_t current_ma, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 6;
    msg->data[0] = ZDT_CMD_WRITE_OPEN_LOOP_CURRENT;
    msg->data[1] = ZDT_AUX_WRITE_OPEN_CURRENT;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = (current_ma >> 8) & 0xFF;
    msg->data[4] = current_ma & 0xFF;
    msg->data[5] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改闭环模式最大电流命令
 * 基于 5.6.13 修改闭环模式最大电流
 */
void zdt_cmd_write_closed_loop_current(uint8_t addr, uint8_t save, uint16_t current_ma, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 6;
    msg->data[0] = ZDT_CMD_WRITE_CLOSED_LOOP_CURRENT;
    msg->data[1] = ZDT_AUX_WRITE_CLOSED_CURRENT;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = (current_ma >> 8) & 0xFF;
    msg->data[4] = current_ma & 0xFF;
    msg->data[5] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造读取PID参数命令（X固件）
 * 基于 5.6.14 读取PID参数（X）
 *
 * 主机发送格式：Addr + 21 + 6B
 */
void zdt_cmd_read_pid_param(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_PID_PARAM;  /* 0x21 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 构造读取PID参数命令（Emm固件）
 * 基于 5.6.16 读取PID参数（Emm）- 与X固件发送相同命令0x21，返回格式不同
 */
void zdt_cmd_read_pid_param_emm(uint8_t addr, zdt_can_msg_t *msg)
{
    zdt_cmd_read_pid_param(addr, msg);
}

/**
 * @brief 构造修改PID参数命令（X固件）
 * 基于 5.6.15 修改PID参数（X）
 *
 * 主机发送：4A + C3 + save + 梯形Kp(4) + 直通Kp(4) + 速度Kp(2) + 速度Ki(2) + 6B
 */
uint8_t zdt_cmd_write_pid_param_x(uint8_t addr, uint8_t save,
                                uint32_t trapezoid_kp, uint32_t direct_kp,
                                uint16_t velocity_kp, uint16_t velocity_ki,
                                zdt_can_msg_t *msgs)
{
    /* 第1个分包：4A + C3 + save + 梯形Kp(4) + 6B */
    msgs[0].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_WRITE_PID_PARAM;
    msgs[0].data[1] = ZDT_AUX_WRITE_PID;
    msgs[0].data[2] = save ? 0x01 : 0x00;
    msgs[0].data[3] = (trapezoid_kp >> 24) & 0xFF;
    msgs[0].data[4] = (trapezoid_kp >> 16) & 0xFF;
    msgs[0].data[5] = (trapezoid_kp >> 8) & 0xFF;
    msgs[0].data[6] = trapezoid_kp & 0xFF;
    msgs[0].data[7] = ZDT_CHECKSUM_FIXED;

    /* 第2个分包：4A + 直通Kp(4) + 速度Kp(2) + 6B */
    msgs[1].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_1);
    msgs[1].dlc = 8;
    msgs[1].data[0] = ZDT_CMD_WRITE_PID_PARAM;
    msgs[1].data[1] = (direct_kp >> 24) & 0xFF;
    msgs[1].data[2] = (direct_kp >> 16) & 0xFF;
    msgs[1].data[3] = (direct_kp >> 8) & 0xFF;
    msgs[1].data[4] = direct_kp & 0xFF;
    msgs[1].data[5] = (velocity_kp >> 8) & 0xFF;
    msgs[1].data[6] = velocity_kp & 0xFF;
    msgs[1].data[7] = ZDT_CHECKSUM_FIXED;

    /* 第3个分包：4A + 速度Ki(2) + 6B */
    msgs[2].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_2);
    msgs[2].dlc = 5;
    msgs[2].data[0] = ZDT_CMD_WRITE_PID_PARAM;
    msgs[2].data[1] = (velocity_ki >> 8) & 0xFF;
    msgs[2].data[2] = velocity_ki & 0xFF;
    msgs[2].data[3] = ZDT_CHECKSUM_FIXED;

    return 3;
}

/**
 * @brief 构造修改PID参数命令（Emm固件）
 * 基于 5.6.17 修改PID参数（Emm）
 *
 * 主机发送：4A + C3 + save + Kp(4) + Ki(4) + Kd(4) + 6B
 */
uint8_t zdt_cmd_write_pid_param_emm(uint8_t addr, uint8_t save,
                                   uint32_t kp, uint32_t ki, uint32_t kd,
                                   zdt_can_msg_t *msgs)
{
    /* 第1个分包：4A + C3 + save + Kp(4) + 6B */
    msgs[0].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_WRITE_PID_PARAM;
    msgs[0].data[1] = ZDT_AUX_WRITE_PID;
    msgs[0].data[2] = save ? 0x01 : 0x00;
    msgs[0].data[3] = (kp >> 24) & 0xFF;
    msgs[0].data[4] = (kp >> 16) & 0xFF;
    msgs[0].data[5] = (kp >> 8) & 0xFF;
    msgs[0].data[6] = kp & 0xFF;
    msgs[0].data[7] = ZDT_CHECKSUM_FIXED;

    /* 第2个分包：4A + Ki(4) + Kd(2) + 6B */
    msgs[1].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_1);
    msgs[1].dlc = 8;
    msgs[1].data[0] = ZDT_CMD_WRITE_PID_PARAM;
    msgs[1].data[1] = (ki >> 24) & 0xFF;
    msgs[1].data[2] = (ki >> 16) & 0xFF;
    msgs[1].data[3] = (ki >> 8) & 0xFF;
    msgs[1].data[4] = ki & 0xFF;
    msgs[1].data[5] = (kd >> 24) & 0xFF;
    msgs[1].data[6] = (kd >> 16) & 0xFF;
    msgs[1].data[7] = ZDT_CHECKSUM_FIXED;

    /* 第3个分包：4A + Kd(2) + 6B */
    msgs[2].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_2);
    msgs[2].dlc = 5;
    msgs[2].data[0] = ZDT_CMD_WRITE_PID_PARAM;
    msgs[2].data[1] = (kd >> 8) & 0xFF;
    msgs[2].data[2] = kd & 0xFF;
    msgs[2].data[3] = ZDT_CHECKSUM_FIXED;

    return 3;
}

/**
 * @brief 构造修改位置到达窗口命令
 * 基于 5.6.21 修改位置到达窗口（X42S/Y42）
 */
void zdt_cmd_write_pos_window(uint8_t addr, uint8_t save, uint16_t window, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 6;
    msg->data[0] = ZDT_CMD_WRITE_POS_WINDOW;
    msg->data[1] = ZDT_AUX_WRITE_POS_WINDOW;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = (window >> 8) & 0xFF;
    msg->data[4] = window & 0xFF;
    msg->data[5] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 构造修改过热过流保护检测阈值命令
 * 基于 5.6.23 修改过热过流保护检测阈值（X42S/Y42）
 */
uint8_t zdt_cmd_write_protect_threshold(uint8_t addr, uint8_t save,
                                      uint16_t temp_threshold,
                                      uint16_t current_threshold,
                                      uint16_t time_threshold,
                                      zdt_can_msg_t *msgs)
{
    /* 第1个分包：D3 + 56 + save + temp(2) + current(2) + 6B */
    msgs[0].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_WRITE_PROTECT_THRESHOLD;
    msgs[0].data[1] = ZDT_AUX_WRITE_PROTECT;
    msgs[0].data[2] = save ? 0x01 : 0x00;
    msgs[0].data[3] = (temp_threshold >> 8) & 0xFF;
    msgs[0].data[4] = temp_threshold & 0xFF;
    msgs[0].data[5] = (current_threshold >> 8) & 0xFF;
    msgs[0].data[6] = current_threshold & 0xFF;
    msgs[0].data[7] = ZDT_CHECKSUM_FIXED;

    /* 第2个分包：D3 + time(2) + 6B */
    msgs[1].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_1);
    msgs[1].dlc = 5;
    msgs[1].data[0] = ZDT_CMD_WRITE_PROTECT_THRESHOLD;
    msgs[1].data[1] = (time_threshold >> 8) & 0xFF;
    msgs[1].data[2] = time_threshold & 0xFF;
    msgs[1].data[3] = ZDT_CHECKSUM_FIXED;

    return 2;
}

/**
 * @brief 构造修改心跳保护功能时间命令
 * 基于 5.6.25 修改心跳保护功能时间（X42S/Y42）
 */
uint8_t zdt_cmd_write_heartbeat(uint8_t addr, uint8_t save,
                                 uint32_t time_ms,
                                 zdt_can_msg_t *msgs)
{
    msgs[0].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_WRITE_HEARTBEAT;
    msgs[0].data[1] = ZDT_AUX_WRITE_HEARTBEAT;
    msgs[0].data[2] = save ? 0x01 : 0x00;
    msgs[0].data[3] = (time_ms >> 24) & 0xFF;
    msgs[0].data[4] = (time_ms >> 16) & 0xFF;
    msgs[0].data[5] = (time_ms >> 8) & 0xFF;
    msgs[0].data[6] = time_ms & 0xFF;
    msgs[0].data[7] = ZDT_CHECKSUM_FIXED;

    return 1;
}

/**
 * @brief 构造修改积分限幅/刚性系数命令
 * 基于 5.6.27 修改积分限幅/刚性系数（X42S/Y42）
 */
uint8_t zdt_cmd_write_integral_limit(uint8_t addr, uint8_t save,
                                   uint32_t limit,
                                   zdt_can_msg_t *msgs)
{
    msgs[0].id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msgs[0].dlc = 8;
    msgs[0].data[0] = ZDT_CMD_WRITE_INTEGRAL_LIMIT;
    msgs[0].data[1] = ZDT_AUX_WRITE_INTEGRAL;
    msgs[0].data[2] = save ? 0x01 : 0x00;
    msgs[0].data[3] = (limit >> 24) & 0xFF;
    msgs[0].data[4] = (limit >> 16) & 0xFF;
    msgs[0].data[5] = (limit >> 8) & 0xFF;
    msgs[0].data[6] = limit & 0xFF;
    msgs[0].data[7] = ZDT_CHECKSUM_FIXED;

    return 1;
}

/**
 * @brief 构造修改碰撞回零返回角度命令
 * 基于 5.6.29 修改碰撞回零返回角度（X42S/Y42）
 */
void zdt_cmd_write_homing_angle(uint8_t addr, uint8_t save, uint16_t angle, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 6;
    msg->data[0] = ZDT_CMD_WRITE_HOMING_ANGLE;
    msg->data[1] = ZDT_AUX_WRITE_HOMING_ANGLE;
    msg->data[2] = save ? 0x01 : 0x00;
    msg->data[3] = (angle >> 8) & 0xFF;
    msg->data[4] = angle & 0xFF;
    msg->data[5] = ZDT_CHECKSUM_FIXED;
}

/**
 * @brief 解析PID参数（X固件）
 * 基于 5.6.14 读取PID参数（X）
 *
 * 电机返回格式：Addr + 21 + 梯形位置环Kp(4) + 直通位置环Kp(4) + 速度环Kp(2) + 速度环Ki(2) + 6B
 */
bool zdt_parse_pid_param_x(const zdt_can_msg_t *msg,
                       uint32_t *trapezoid_kp, uint32_t *direct_kp,
                       uint16_t *velocity_kp, uint16_t *velocity_ki)
{
    if (msg == NULL || trapezoid_kp == NULL || direct_kp == NULL ||
        velocity_kp == NULL || velocity_ki == NULL || msg->dlc < 14) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_PID_PARAM) {
        return false;
    }

    *trapezoid_kp = ((uint32_t)msg->data[1] << 24) |
                     ((uint32_t)msg->data[2] << 16) |
                     ((uint32_t)msg->data[3] << 8) |
                     (uint32_t)msg->data[4];

    *direct_kp = ((uint32_t)msg->data[5] << 24) |
                   ((uint32_t)msg->data[6] << 16) |
                   ((uint32_t)msg->data[7] << 8) |
                   (uint32_t)msg->data[8];

    *velocity_kp = ((uint16_t)msg->data[9] << 8) | (uint16_t)msg->data[10];

    *velocity_ki = ((uint16_t)msg->data[11] << 8) | (uint16_t)msg->data[12];

    return true;
}

/**
 * @brief 解析PID参数（Emm固件）
 * 基于 5.6.16 读取PID参数（Emm）
 *
 * 电机返回格式：Addr + 21 + Kp(4) + Ki(4) + Kd(4) + 6B
 */
bool zdt_parse_pid_param_emm(const zdt_can_msg_t *msg,
                            uint32_t *kp, uint32_t *ki, uint32_t *kd)
{
    if (msg == NULL || kp == NULL || ki == NULL || kd == NULL || msg->dlc < 14) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_PID_PARAM) {
        return false;
    }

    *kp = ((uint32_t)msg->data[1] << 24) |
          ((uint32_t)msg->data[2] << 16) |
          ((uint32_t)msg->data[3] << 8) |
          (uint32_t)msg->data[4];

    *ki = ((uint32_t)msg->data[5] << 24) |
          ((uint32_t)msg->data[6] << 16) |
          ((uint32_t)msg->data[7] << 8) |
          (uint32_t)msg->data[8];

    *kd = ((uint32_t)msg->data[9] << 24) |
          ((uint32_t)msg->data[10] << 16) |
          ((uint32_t)msg->data[11] << 8) |
          (uint32_t)msg->data[12];

    return true;
}

/**
 * @brief 构造读取位置到达窗口命令（X42S/Y42）
 * 基于 5.6.20 读取位置到达窗口（X42S/Y42）
 *
 * 主机发送格式：Addr + 41 + 6B
 */
void zdt_cmd_read_pos_window(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_POS_WINDOW;  /* 0x41 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 解析位置到达窗口
 * 基于 5.6.20 读取位置到达窗口（X42S/Y42）
 *
 * 电机返回格式：Addr + 41 + Pos_Window(H) + Pos_Window(L) + 6B
 */
bool zdt_parse_pos_window(const zdt_can_msg_t *msg, uint16_t *window)
{
    if (msg == NULL || window == NULL || msg->dlc < 5) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_POS_WINDOW) {
        return false;
    }

    *window = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[2];

    return true;
}

/**
 * @brief 构造读取过热过流保护检测阈值命令（X42S/Y42）
 * 基于 5.6.22 读取过热过流保护检测阈值（X42S/Y42）
 *
 * 主机发送格式：Addr + 13 + 6B
 */
void zdt_cmd_read_protect_threshold(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_PROTECT_THRESHOLD; /* 0x13 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;             /* 0x6B */
}

/**
 * @brief 解析过热过流保护检测阈值
 * 基于 5.6.22 读取过热过流保护检测阈值（X42S/Y42）
 *
 * 电机返回格式：Addr + 13 + 过热保护检测阈值(H) + 过热保护检测阈值(L) + 过流保护检测阈值(H) + 过流保护检测阈值(L) + 过热过流检测时间(H) + 过热过流检测时间(L) + 6B
 */
bool zdt_parse_protect_threshold(const zdt_can_msg_t *msg,
                             uint16_t *temp_threshold, uint16_t *current_threshold,
                             uint16_t *time_threshold)
{
    if (msg == NULL || temp_threshold == NULL || current_threshold == NULL ||
        time_threshold == NULL || msg->dlc < 8) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_PROTECT_THRESHOLD) {
        return false;
    }

    *temp_threshold = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[2];    /* ℃ */
    *current_threshold = ((uint16_t)msg->data[3] << 8) | (uint16_t)msg->data[4];  /* mA */
    *time_threshold = ((uint16_t)msg->data[5] << 8) | (uint16_t)msg->data[6];    /* ms */

    return true;
}

/**
 * @brief 构造读取心跳保护功能时间命令（X42S/Y42）
 * 基于 5.6.24 读取心跳保护功能时间（X42S/Y42）
 *
 * 主机发送格式：Addr + 16 + 6B
 */
void zdt_cmd_read_heartbeat(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_HEARTBEAT;     /* 0x16 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;          /* 0x6B */
}

/**
 * @brief 解析心跳保护功能时间
 * 基于 5.6.24 读取心跳保护功能时间（X42S/Y42）
 *
 * 电机返回格式：Addr + 16 + 心跳保护时间(4) + 6B
 */
bool zdt_parse_heartbeat(const zdt_can_msg_t *msg, uint32_t *time_ms)
{
    if (msg == NULL || time_ms == NULL || msg->dlc < 7) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_HEARTBEAT) {
        return false;
    }

    *time_ms = ((uint32_t)msg->data[1] << 24) |
               ((uint32_t)msg->data[2] << 16) |
               ((uint32_t)msg->data[3] << 8) |
               (uint32_t)msg->data[4];

    return true;
}

/**
 * @brief 构造读取积分限幅/刚性系数命令（X42S/Y42）
 * 基于 5.6.26 读取积分限幅/刚性系数（X42S/Y42）
 *
 * 主机发送格式：Addr + 23 + 6B
 */
void zdt_cmd_read_integral_limit(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_INTEGRAL_LIMIT; /* 0x23 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;           /* 0x6B */
}

/**
 * @brief 解析积分限幅/刚性系数
 * 基于 5.6.26 读取积分限幅/刚性系数（X42S/Y42）
 *
 * 电机返回格式：Addr + 23 + 积分限幅/刚性系数(4) + 6B
 */
bool zdt_parse_integral_limit(const zdt_can_msg_t *msg, uint32_t *limit)
{
    if (msg == NULL || limit == NULL || msg->dlc < 7) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_INTEGRAL_LIMIT) {
        return false;
    }

    *limit = ((uint32_t)msg->data[1] << 24) |
             ((uint32_t)msg->data[2] << 16) |
             ((uint32_t)msg->data[3] << 8) |
             (uint32_t)msg->data[4];

    return true;
}

/**
 * @brief 构造读取碰撞回零返回角度命令（X42S/Y42）
 * 基于 5.6.28 读取碰撞回零返回角度（X42S/Y42）
 *
 * 主机发送格式：Addr + 3F + 6B
 */
void zdt_cmd_read_homing_angle(uint8_t addr, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_READ_HOMING_ANGLE; /* 0x3F */
    msg->data[1] = ZDT_CHECKSUM_FIXED;           /* 0x6B */
}

/**
 * @brief 解析碰撞回零返回角度
 * 基于 5.6.28 读取碰撞回零返回角度（X42S/Y42）
 *
 * 电机返回格式：Addr + 3F + O_SL_RP(H) + O_SL_RP(L) + 6B
 */
bool zdt_parse_homing_angle(const zdt_can_msg_t *msg, uint16_t *angle)
{
    if (msg == NULL || angle == NULL || msg->dlc < 5) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_READ_HOMING_ANGLE) {
        return false;
    }

    *angle = ((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[2];

    return true;
}

/**
 * @brief 构造广播读取ID地址命令（X42S/Y42）
 * 基于 5.6.30 广播读取ID地址（X42S/Y42）
 *
 * 主机发送格式：00 + 15 + 6B
 */
void zdt_cmd_broadcast_read_id(zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(0, ZDT_PACKET_0);  /* 广播地址0 */
    msg->dlc = 2;
    msg->data[0] = ZDT_CMD_BROADCAST_READ_ID;   /* 0x15 */
    msg->data[1] = ZDT_CHECKSUM_FIXED;            /* 0x6B */
}

/**
 * @brief 解析广播读取ID地址响应
 * 基于 5.6.30 广播读取ID地址（X42S/Y42）
 *
 * 电机返回格式：Addr + 15 + Addr + 6B
 */
bool zdt_parse_broadcast_read_id(const zdt_can_msg_t *msg, uint8_t *addr)
{
    if (msg == NULL || addr == NULL || msg->dlc < 4) {
        return false;
    }

    if (msg->data[0] != ZDT_CMD_BROADCAST_READ_ID) {
        return false;
    }

    *addr = msg->data[1];  /* 返回的电机地址 */

    return true;
}

/**
 * @brief 构造修改锁定修改参数功能命令（X42S/Y42）
 * 基于 5.6.31 修改锁定修改参数功能（X42S/Y42）
 *
 * 主机发送格式：Addr + D6 + 4B + 锁定参数等级 + 6B
 */
void zdt_cmd_write_param_lock(uint8_t addr, uint8_t save, uint8_t lock_level, zdt_can_msg_t *msg)
{
    msg->id = ZDT_CAN_FRAME_ID(addr, ZDT_PACKET_0);
    msg->dlc = 5;
    msg->data[0] = ZDT_CMD_WRITE_PARAM_LOCK;  /* 0xD6 */
    msg->data[1] = ZDT_AUX_WRITE_PARAM_LOCK; /* 0x4B */
    msg->data[2] = save ? 0x01 : 0x00;        /* 是否存储 */
    msg->data[3] = lock_level;                 /* 锁定参数等级 0-3 */
    msg->data[4] = ZDT_CHECKSUM_FIXED;        /* 0x6B */
}

/*==============================================================================
 * 辅助函数实现
 *============================================================================*/

/**
 * @brief 计算XOR校验码
 * 基于 5.0.3 校验码类型
 */
uint8_t zdt_calc_xor_checksum(const uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief 计算CRC8校验码
 * 基于 5.0.3 校验码类型
 * 使用多项式：x^8 + x^2 + x + 1 (0x07)
 */
uint8_t zdt_calc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    const uint8_t polynomial = 0x07;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}
