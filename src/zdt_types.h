#ifndef ZDT_TYPES_H
#define ZDT_TYPES_H

#include <stdint.h>
#include <stddef.h>

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
    uint8_t  dlc;        /* 数据长度 (1..8) */
    uint8_t  data[8];    /* 有效负载 */
} zdt_can_frame_t;

/* CAN 复合消息结构体 */
typedef struct {
    uint8_t         frame_count;                    /* 帧数量 (1..ZDT_CAN_MAX_FRAMES) */
    zdt_can_frame_t frames[ZDT_CAN_MAX_FRAMES];     /* CAN 帧列表 */
} zdt_can_msg_t;

#ifdef __cplusplus
}
#endif

#endif /* ZDT_TYPES_H */
