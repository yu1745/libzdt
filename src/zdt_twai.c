/* SPDX-License-Identifier: GPL-3.0-only */
#include "zdt_twai.h"

#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define ZDT_DEFAULT_BITRATE     500000
#define ZDT_DEFAULT_TX_DEPTH    8
#define ZDT_DEFAULT_RX_DEPTH    32

static const char *TAG = "zdt_twai";

struct zdt_twai {
    twai_node_handle_t node;
    QueueHandle_t      rx_queue;
    SemaphoreHandle_t  lock;
};

static IRAM_ATTR bool zdt_twai_rx_done_cb(twai_node_handle_t node, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    struct zdt_twai *ctx = (struct zdt_twai *)user_ctx;
    zdt_can_frame_t out;
    uint8_t buf[8];
    twai_frame_t frame = {
        .buffer = buf,
        .buffer_len = sizeof(buf),
    };
    BaseType_t woken = pdFALSE;

    if (twai_node_receive_from_isr(node, &frame) != ESP_OK) {
        return false;
    }
    /* 电机只使用扩展数据帧 */
    if (!frame.header.ide || frame.header.rtr || frame.header.dlc > 8) {
        return false;
    }

    out.id = frame.header.id;
    out.dlc = (uint8_t)frame.header.dlc;
    memcpy(out.data, buf, out.dlc);
    xQueueSendFromISR(ctx->rx_queue, &out, &woken);
    return woken == pdTRUE;
}

esp_err_t zdt_twai_new(const zdt_twai_config_t *config, zdt_twai_handle_t *handle_ret)
{
    ESP_RETURN_ON_FALSE(config && handle_ret, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    struct zdt_twai *ctx = calloc(1, sizeof(*ctx));
    ESP_RETURN_ON_FALSE(ctx, ESP_ERR_NO_MEM, TAG, "no mem");

    esp_err_t ret = ESP_ERR_NO_MEM;
    uint32_t rx_depth = config->rx_queue_depth ? config->rx_queue_depth : ZDT_DEFAULT_RX_DEPTH;
    ctx->rx_queue = xQueueCreate(rx_depth, sizeof(zdt_can_frame_t));
    ctx->lock = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(ctx->rx_queue && ctx->lock, ESP_ERR_NO_MEM, err, TAG, "no mem");

    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx = config->tx_io,
            .rx = config->rx_io,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing.bitrate = config->bitrate ? config->bitrate : ZDT_DEFAULT_BITRATE,
        .fail_retry_cnt = 3,
        .tx_queue_depth = config->tx_queue_depth ? config->tx_queue_depth : ZDT_DEFAULT_TX_DEPTH,
        .flags = {
            .enable_self_test = config->self_test,
            .enable_loopback = config->loopback,
        },
    };
    ESP_GOTO_ON_ERROR(twai_new_node_onchip(&node_cfg, &ctx->node), err, TAG, "new node failed");

    twai_event_callbacks_t cbs = {
        .on_rx_done = zdt_twai_rx_done_cb,
    };
    ESP_GOTO_ON_ERROR(twai_node_register_event_callbacks(ctx->node, &cbs, ctx), err_node, TAG, "register cb failed");
    ESP_GOTO_ON_ERROR(twai_node_enable(ctx->node), err_node, TAG, "enable failed");

    *handle_ret = ctx;
    return ESP_OK;

err_node:
    twai_node_delete(ctx->node);
err:
    if (ctx->rx_queue) {
        vQueueDelete(ctx->rx_queue);
    }
    if (ctx->lock) {
        vSemaphoreDelete(ctx->lock);
    }
    free(ctx);
    return ret;
}

esp_err_t zdt_twai_del(zdt_twai_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    twai_node_disable(handle->node);
    twai_node_delete(handle->node);
    vQueueDelete(handle->rx_queue);
    vSemaphoreDelete(handle->lock);
    free(handle);
    return ESP_OK;
}

esp_err_t zdt_twai_send(zdt_twai_handle_t handle, const zdt_can_msg_t *msg, int timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && msg && msg->frame_count >= 1 && msg->frame_count <= ZDT_CAN_MAX_FRAMES,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    for (uint8_t i = 0; i < msg->frame_count; ++i) {
        const zdt_can_frame_t *f = &msg->frames[i];
        twai_frame_t frame = {
            .header = {
                .id = f->id,
                .dlc = f->dlc,
                .ide = true,
            },
            .buffer = (uint8_t *)f->data,
            .buffer_len = f->dlc,
        };
        ESP_RETURN_ON_ERROR(twai_node_transmit(handle->node, &frame, timeout_ms), TAG, "transmit failed");
    }
    return twai_node_transmit_wait_all_done(handle->node, timeout_ms);
}

void zdt_twai_flush(zdt_twai_handle_t handle)
{
    if (handle) {
        xQueueReset(handle->rx_queue);
    }
}

esp_err_t zdt_twai_recv_frame(zdt_twai_handle_t handle, zdt_can_frame_t *frame, int timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && frame, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    if (xQueueReceive(handle->rx_queue, frame, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t zdt_twai_recv_reply(zdt_twai_handle_t handle, uint8_t addr, size_t expect_len,
                              zdt_reply_t *reply, int timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && reply, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    uint8_t raw[ZDT_REPLY_MAX_LEN + 1];     /* Code 之后的内容，含末尾校验字节 */
    size_t raw_len = 0;
    uint8_t code = 0;
    uint8_t next_packet = 0;
    bool have_first = false;
    int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    for (;;) {
        int wait_ms;
        if (have_first && expect_len == 0) {
            wait_ms = ZDT_REPLY_GAP_MS;
        } else {
            int64_t left_us = deadline_us - esp_timer_get_time();
            wait_ms = left_us > 0 ? (int)((left_us + 999) / 1000) : 0;
        }

        zdt_can_frame_t f;
        if (zdt_twai_recv_frame(handle, &f, wait_ms) != ESP_OK) {
            if (have_first && expect_len == 0) {
                break;
            }
            return ESP_ERR_TIMEOUT;
        }
        if ((f.id >> 16) != 0 || f.dlc < 1) {
            continue;
        }
        if (addr != 0 && ((f.id >> 8) & 0xFF) != addr) {
            continue;
        }

        uint8_t packet = (uint8_t)(f.id & 0xFF);
        if (!have_first) {
            if (packet != 0) {
                continue;
            }
            have_first = true;
            reply->addr = (uint8_t)((f.id >> 8) & 0xFF);
            code = f.data[0];
            addr = reply->addr;     /* 续包只认首包的地址 */
        } else {
            if (packet != next_packet || f.data[0] != code) {
                return ESP_ERR_INVALID_RESPONSE;
            }
        }
        next_packet = (uint8_t)(packet + 1);

        size_t chunk = (size_t)f.dlc - 1;
        if (raw_len + chunk > sizeof(raw)) {
            return ESP_ERR_NO_MEM;
        }
        memcpy(raw + raw_len, f.data + 1, chunk);
        raw_len += chunk;

        if (f.dlc < 8 || (expect_len && raw_len >= expect_len)) {
            break;
        }
    }

    if (raw_len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    reply->code = code;
    reply->len = (uint8_t)(raw_len - 1);
    memcpy(reply->data, raw, reply->len);
    reply->checksum = raw[raw_len - 1];
    return reply->checksum == ZDT_CHECKSUM_DEFAULT ? ESP_OK : ESP_ERR_INVALID_CRC;
}

esp_err_t zdt_twai_transact(zdt_twai_handle_t handle, const zdt_can_msg_t *msg, size_t expect_len,
                            zdt_reply_t *reply, int timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && msg && msg->frame_count >= 1 && reply, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    uint8_t addr = (uint8_t)((msg->frames[0].id >> 8) & 0xFF);
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    zdt_twai_flush(handle);
    esp_err_t ret = zdt_twai_send(handle, msg, timeout_ms);
    if (ret == ESP_OK) {
        ret = zdt_twai_recv_reply(handle, addr, expect_len, reply, timeout_ms);
    }
    xSemaphoreGive(handle->lock);
    return ret;
}
