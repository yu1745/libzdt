/* SPDX-License-Identifier: GPL-3.0-only */
/*
 * zdt_uart.h - ZDT_X42S 的 ESP-IDF 串口 TTL 收发层
 *
 * 构建帧由 zdtBuild*Cmd 完成（原始命令字节），本模块负责：
 *   - 安装 / 删除 UART 驱动（8N1，无流控）
 *   - 按配置的校验方式改写末字节后发送
 *   - 从字节流切分应答（zdt_ttl_parse），把与当前请求无关的帧
 *     （到位 9F、定时返回、其它地址）交给事件回调
 *
 * 不创建后台任务：只在调用 recv / transact / poll 时读串口，回调也在调用方任务里执行。
 * 电机开了定时返回或到位返回时，空闲期间要定期调用 zdt_uart_poll，否则 UART 接收缓冲会溢出。
 */

#ifndef ZDT_UART_H
#define ZDT_UART_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/uart.h"
#include "hal/gpio_types.h"
#include "zdt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct zdt_uart *zdt_uart_handle_t;

/** 未被 recv / transact 消费的应答帧（到位 9F、定时返回、其它地址的应答） */
typedef void (*zdt_uart_event_cb_t)(const zdt_reply_t *frame, void *user_ctx);

typedef struct {
    uart_port_t    port;
    gpio_num_t     tx_io;
    gpio_num_t     rx_io;
    uint32_t       baud_rate;      /* 0 表示 115200（电机出厂默认） */
    zdt_checksum_t checksum;       /* 与电机设置一致，出厂为 ZDT_CHECKSUM_FIXED_6B */
    zdt_firmware_t firmware;       /* 决定少数读取命令的应答长度 */
    size_t         rx_buffer_size; /* UART 驱动接收缓冲，0 表示 512 */
    uint32_t       write_gap_us;   /* 修改参数命令（不存储）应答后到下一次发送的最小间隔，0 表示 5000
                                      （真机在应答后约 1-4 ms 内不接收新命令） */
    uint32_t       store_gap_us;   /* 同上，但命令带存储标志（第 4 字节 01）时用，0 表示 100000
                                      （真机多为 4-7 ms，擦写 Flash 时约 70 ms） */
    uint32_t       frame_gap_us;   /* 总线上每帧（发出或收到）后到下一次发送的最小空闲，0 表示 100；
                                      广播帧（地址 0）后至少 300。921600 下真机：应答后紧接着发下一帧
                                      约 1/4 超时，隔 100 us 即无失败；广播同步（FF 66）后需约 300 us */
} zdt_uart_config_t;

/**
 * @brief 安装 UART 驱动并创建句柄
 * @return ESP_OK；参数错误 ESP_ERR_INVALID_ARG；内存不足 ESP_ERR_NO_MEM；其余为 UART 驱动错误码
 */
esp_err_t zdt_uart_new(const zdt_uart_config_t *config, zdt_uart_handle_t *handle_ret);

/**
 * @brief 切换本端波特率（电机侧用 5.8.6 驱动配置的波特率字段修改）
 *
 * 等待发送完成后切换，清空接收缓冲。与 transact 等互斥，可在任意任务调用。
 * @return ESP_OK；参数错误 ESP_ERR_INVALID_ARG；其余为 UART 驱动错误码
 */
esp_err_t zdt_uart_set_baud_rate(zdt_uart_handle_t handle, uint32_t baud_rate);

/** 删除 UART 驱动，释放句柄 */
esp_err_t zdt_uart_del(zdt_uart_handle_t handle);

/** 设置事件回调（NULL 表示丢弃无关帧） */
void zdt_uart_set_event_cb(zdt_uart_handle_t handle, zdt_uart_event_cb_t cb, void *user_ctx);

/**
 * @brief 发送一帧
 * frame 末字节按配置的校验方式改写后发出（frame 本身不修改），等待发完。
 * @return ESP_OK；ESP_ERR_INVALID_ARG（len < 3 或超过 ZDT_TTL_MAX_FRAME 的 4 倍）；ESP_ERR_TIMEOUT
 */
esp_err_t zdt_uart_send(zdt_uart_handle_t handle, const uint8_t *frame, size_t len, int timeout_ms);

/** 丢弃已收到但未解析的字节 */
void zdt_uart_flush(zdt_uart_handle_t handle);

/**
 * @brief 接收地址为 addr、功能码为 code 的下一帧应答
 *
 * 功能码 00（命令格式错误）也视为匹配。addr 为 0 时不过滤地址（广播命令的应答来自地址 1）。
 * 其余帧交给事件回调后继续等待。
 *
 * @param timeout_ms 总等待上限
 * @return ESP_OK；ESP_ERR_TIMEOUT
 */
esp_err_t zdt_uart_recv_reply(zdt_uart_handle_t handle, uint8_t addr, uint8_t code,
                              zdt_reply_t *reply, int timeout_ms);

/**
 * @brief 发送 → 接收应答（互斥，多任务可并发调用）
 *
 * 地址与功能码取自 frame[0] / frame[1]。发送前不清空已收字节：先到的帧照常分派给事件回调。
 */
esp_err_t zdt_uart_transact(zdt_uart_handle_t handle, const uint8_t *frame, size_t len,
                            zdt_reply_t *reply, int timeout_ms);

/**
 * @brief 在 timeout_ms 内接收并分派帧给事件回调（用于定时返回 / 到位返回）
 * @return 分派的帧数
 */
int zdt_uart_poll(zdt_uart_handle_t handle, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_UART_H */
