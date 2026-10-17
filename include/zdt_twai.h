/*
 * zdt_twai.h - ZDT_X42S 的 ESP-IDF TWAI 收发层
 *
 * 构建帧由 zdtCanBuild*Cmd 完成（得到 zdt_can_msg_t），本模块负责：
 *   - 创建/删除 TWAI 节点（扩展帧，默认接收全部）
 *   - 发送一条 zdt_can_msg_t（多包按顺序发出）
 *   - 接收并重组电机应答（ID=(Addr<<8)|Packet，每个包首字节为 Code）
 */

#ifndef ZDT_TWAI_H
#define ZDT_TWAI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "hal/gpio_types.h"
#include "zdt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 应答数据字节（手册 4.2.2） */
#define ZDT_REPLY_OK            0x02    /* 命令正确 */
#define ZDT_REPLY_DONE          0x9F    /* 动作完成（电机主动返回：F5 / FD / 9A） */
#define ZDT_REPLY_PARAM_ERR     0xE2    /* 参数错误或条件不满足 */
#define ZDT_REPLY_FORMAT_ERR    0xEE    /* 命令格式错误 */

#define ZDT_REPLY_MAX_LEN       64      /* zdt_reply_t.data 容量 */
#define ZDT_REPLY_GAP_MS        20      /* 长度未知时，收到首包后等待后续包的最长间隔 */

typedef struct zdt_twai *zdt_twai_handle_t;

typedef struct {
    gpio_num_t tx_io;
    gpio_num_t rx_io;
    uint32_t   bitrate;         /* bit/s，0 表示 500000（电机出厂默认） */
    uint32_t   tx_queue_depth;  /* 0 表示 8 */
    uint32_t   rx_queue_depth;  /* 软件接收队列帧数，0 表示 32 */
    bool       self_test;       /* 发送不需要 ACK（无电机联调用） */
    bool       loopback;        /* 控制器收回自己发出的帧 */
} zdt_twai_config_t;

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

/**
 * @brief 创建并启用 TWAI 节点
 * @return ESP_OK；参数错误 ESP_ERR_INVALID_ARG；内存不足 ESP_ERR_NO_MEM；其余为 TWAI 驱动错误码
 */
esp_err_t zdt_twai_new(const zdt_twai_config_t *config, zdt_twai_handle_t *handle_ret);

/** 停用并删除节点，释放资源 */
esp_err_t zdt_twai_del(zdt_twai_handle_t handle);

/**
 * @brief 发送一条命令的全部 CAN 帧
 * @param timeout_ms 每帧入队和整条消息发完的等待上限
 */
esp_err_t zdt_twai_send(zdt_twai_handle_t handle, const zdt_can_msg_t *msg, int timeout_ms);

/** 丢弃软件接收队列中尚未取走的帧 */
void zdt_twai_flush(zdt_twai_handle_t handle);

/** 取一个原始 CAN 帧；超时返回 ESP_ERR_TIMEOUT */
esp_err_t zdt_twai_recv_frame(zdt_twai_handle_t handle, zdt_can_frame_t *frame, int timeout_ms);

/**
 * @brief 接收并重组一条应答
 *
 * 只接收 ID 中地址为 addr 的帧（addr=0 不过滤）。首包 Packet 必须为 0；续包的序号必须连续且
 * 首字节必须等于首包的 Code，否则返回 ESP_ERR_INVALID_RESPONSE。
 * 结束条件：收到不满 8 字节的包；或已收到 expect_len 字节；或 expect_len 为 0 且 ZDT_REPLY_GAP_MS 内无后续包。
 *
 * @param expect_len 预期的 Code 之后总字节数（含末尾校验字节），0 表示未知
 * @param timeout_ms 等待首包的上限
 * @return ESP_OK；ESP_ERR_TIMEOUT 未收到首包；ESP_ERR_INVALID_RESPONSE 包序/Code 不符；
 *         ESP_ERR_NO_MEM 应答超过 ZDT_REPLY_MAX_LEN；ESP_ERR_INVALID_CRC 校验字节不是 0x6B（reply 仍已填充）
 */
esp_err_t zdt_twai_recv_reply(zdt_twai_handle_t handle, uint8_t addr, size_t expect_len,
                              zdt_reply_t *reply, int timeout_ms);

/**
 * @brief 清空接收队列 → 发送 → 接收应答（互斥，多任务可并发调用）
 *
 * 地址取自 msg 第一帧的 ID。广播（地址 0）命令的应答来自地址 1（手册 p.36），此时不按地址过滤。
 */
esp_err_t zdt_twai_transact(zdt_twai_handle_t handle, const zdt_can_msg_t *msg, size_t expect_len,
                            zdt_reply_t *reply, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_TWAI_H */
