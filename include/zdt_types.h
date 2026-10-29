/* SPDX-License-Identifier: GPL-3.0-only */
#ifndef ZDT_TYPES_H
#define ZDT_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* CAN 分帧说明：各模块文档中的“原始命令”包含 Addr；CAN 将 Addr 编入
 * EID=(Addr<<8)|Packet（Packet 从 0 开始）。CAN data 从 Code 开始、不含
 * Addr。每个 data payload 从 Code 开始；长命令每包均重复 Code，后接至多
 * 7 个连续命令字节。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 常量宏 */
#define ZDT_CHECKSUM_DEFAULT    0x6B
#define ZDT_ADDR_BROADCAST      0x00

#define ZDT_SYNC_NOW            0
#define ZDT_SYNC_CACHE          1

#define ZDT_STORE_NO            0
#define ZDT_STORE_YES           1

#define ZDT_DIR_CW              0
#define ZDT_DIR_CCW             1

#define ZDT_MOVE_REL_LAST       0
#define ZDT_MOVE_ABS_ZERO       1
#define ZDT_MOVE_REL_NOW        2

/* 容量宏 */
#define ZDT_CAN_MAX_FRAMES      8

/* 错误码 */
#define ZDT_OK                   0
#define ZDT_ERR_BUF_TOO_SMALL   -1
#define ZDT_ERR_PARAM           -2

/* CAN 帧结构体 */
typedef struct {
    uint32_t id;         /* 扩展帧 ID: (addr << 8) | packet */
    uint8_t  dlc;        /* 数据长度 (2..8) */
    uint8_t  data[8];    /* 有效负载 */
} zdt_can_frame_t;

/* CAN 复合消息结构体 */
typedef struct {
    uint8_t         frame_count;                    /* 帧数量 (1..ZDT_CAN_MAX_FRAMES) */
    zdt_can_frame_t frames[ZDT_CAN_MAX_FRAMES];     /* CAN 帧列表 */
} zdt_can_msg_t;

/* 应答数据字节（手册 4.1.2 / 4.2.2） */
#define ZDT_REPLY_OK            0x02    /* 命令正确 */
#define ZDT_REPLY_DONE          0x9F    /* 动作完成（电机主动返回：到位 / 回零完成 / 夹紧） */
#define ZDT_REPLY_PARAM_ERR     0xE2    /* 参数错误或条件不满足 */
#define ZDT_REPLY_FORMAT_ERR    0xEE    /* 命令格式错误 */

#define ZDT_REPLY_MAX_LEN       64      /* zdt_reply_t.data 容量 */

/* 电机应答。data 为 Code 之后、校验字节之前的内容。 */
typedef struct {
    uint8_t addr;
    uint8_t code;                       /* 命令的功能码 */
    uint8_t len;                        /* data 有效字节数 */
    uint8_t data[ZDT_REPLY_MAX_LEN];
    uint8_t checksum;                   /* 应答末尾的校验字节 */
} zdt_reply_t;

/** 状态类应答（data 仅 1 字节）是否为 02 */
static inline bool zdt_reply_is_ok(const zdt_reply_t *r)
{
    return r->len == 1 && r->data[0] == ZDT_REPLY_OK;
}

/* 校验方式（手册 8.1；电机侧在"修改驱动配置参数"里设置，出厂为 6B） */
typedef enum {
    ZDT_CHECKSUM_FIXED_6B = 0,  /* 末字节固定 0x6B */
    ZDT_CHECKSUM_XOR      = 1,  /* 前面全部字节（含地址）异或 */
    ZDT_CHECKSUM_CRC8     = 2,  /* 手册 cmd_calCRC8：Maxim 表，初值为首字节 */
} zdt_checksum_t;

/* 电机固件。只影响少数读取命令的应答长度（手册 5.1.2） */
typedef enum {
    ZDT_FW_EMM = 0,
    ZDT_FW_X   = 1,
} zdt_firmware_t;

#ifdef __cplusplus
}
#endif

#endif /* ZDT_TYPES_H */
