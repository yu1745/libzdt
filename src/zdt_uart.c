/* SPDX-License-Identifier: GPL-3.0-only */
#include "zdt_uart.h"

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "zdt_frame.h"

#define ZDT_UART_DEFAULT_BAUD   115200
#define ZDT_UART_DEFAULT_RXBUF  512
#define ZDT_UART_TX_MAX         (ZDT_TTL_MAX_FRAME * 4)   /* 多电机命令可串多条子命令 */
#define ZDT_UART_DEFAULT_WRITE_GAP_US   5000

static const char *TAG = "zdt_uart";

struct zdt_uart {
    uart_port_t         port;
    zdt_checksum_t      checksum;
    zdt_firmware_t      firmware;
    SemaphoreHandle_t   lock;
    zdt_uart_event_cb_t cb;
    void               *cb_ctx;
    uint32_t            write_gap_us;
    int64_t             hold_until_us;   /* 修改参数命令应答后，此前不发送 */
    size_t              rx_len;
    uint8_t             rx[ZDT_TTL_MAX_FRAME * 2];   /* 已收未解析的字节 */
    uint8_t             tx[ZDT_UART_TX_MAX];
};

static TickType_t ms_to_ticks(int64_t ms)
{
    if (ms <= 0) return 0;
    TickType_t t = pdMS_TO_TICKS(ms);
    return t ? t : 1;
}

esp_err_t zdt_uart_new(const zdt_uart_config_t *config, zdt_uart_handle_t *handle_ret)
{
    ESP_RETURN_ON_FALSE(config && handle_ret, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(config->port < UART_NUM_MAX, ESP_ERR_INVALID_ARG, TAG, "bad port");

    struct zdt_uart *h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "no mem");
    h->port = config->port;
    h->checksum = config->checksum;
    h->firmware = config->firmware;
    h->write_gap_us = config->write_gap_us ? config->write_gap_us : ZDT_UART_DEFAULT_WRITE_GAP_US;
    h->lock = xSemaphoreCreateMutex();
    if (!h->lock) {
        free(h);
        return ESP_ERR_NO_MEM;
    }

    const uart_config_t ucfg = {
        .baud_rate = config->baud_rate ? (int)config->baud_rate : ZDT_UART_DEFAULT_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    size_t rxbuf = config->rx_buffer_size ? config->rx_buffer_size : ZDT_UART_DEFAULT_RXBUF;
    esp_err_t err = uart_driver_install(h->port, (int)rxbuf, 0, 0, NULL, 0);
    if (err == ESP_OK) err = uart_param_config(h->port, &ucfg);
    if (err == ESP_OK) err = uart_set_pin(h->port, config->tx_io, config->rx_io,
                                          UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        if (uart_is_driver_installed(h->port)) uart_driver_delete(h->port);
        vSemaphoreDelete(h->lock);
        free(h);
        return err;
    }
    *handle_ret = h;
    return ESP_OK;
}

esp_err_t zdt_uart_del(zdt_uart_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");
    esp_err_t err = uart_driver_delete(handle->port);
    vSemaphoreDelete(handle->lock);
    free(handle);
    return err;
}

void zdt_uart_set_event_cb(zdt_uart_handle_t handle, zdt_uart_event_cb_t cb, void *user_ctx)
{
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    handle->cb = cb;
    handle->cb_ctx = user_ctx;
    xSemaphoreGive(handle->lock);
}

/* 修改参数类命令（5.4.1 / 5.4.6 / 5.6 / 5.7 / 5.8 的写入）。
 * 真机（X42S Emm）在这类命令应答后约 1-2 ms 内不接收下一条命令；读取、运动、停止命令无此现象。 */
static bool is_param_write(uint8_t code)
{
    switch (code) {
    case 0x93: case 0x4C: case 0xAE: case 0x84: case 0x50: case 0xD7: case 0xD5: case 0x46:
    case 0xD4: case 0xD0: case 0x4F: case 0x44: case 0x45: case 0x4A: case 0xD9: case 0xD1:
    case 0xD3: case 0x68: case 0x4B: case 0x5C: case 0xD6: case 0xF7: case 0x48:
        return true;
    default:
        return false;
    }
}

static void wait_hold(struct zdt_uart *h)
{
    for (;;) {
        int64_t left = h->hold_until_us - esp_timer_get_time();
        if (left <= 0) return;
        if (left >= 2000) vTaskDelay(ms_to_ticks(left / 1000));
        else esp_rom_delay_us((uint32_t)left);
    }
}

static esp_err_t send_locked(struct zdt_uart *h, const uint8_t *frame, size_t len, int timeout_ms)
{
    if (!frame || len < 3 || len > sizeof(h->tx)) return ESP_ERR_INVALID_ARG;
    wait_hold(h);
    memcpy(h->tx, frame, len);
    zdt_frame_set_checksum(h->tx, len, h->checksum);
    if (uart_write_bytes(h->port, h->tx, len) != (int)len) return ESP_FAIL;
    return uart_wait_tx_done(h->port, ms_to_ticks(timeout_ms));
}

esp_err_t zdt_uart_send(zdt_uart_handle_t handle, const uint8_t *frame, size_t len, int timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    esp_err_t err = send_locked(handle, frame, len, timeout_ms);
    xSemaphoreGive(handle->lock);
    return err;
}

void zdt_uart_flush(zdt_uart_handle_t handle)
{
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    uart_flush_input(handle->port);
    handle->rx_len = 0;
    xSemaphoreGive(handle->lock);
}

/* 取下一帧：先解析已收字节，不够再从 UART 读，直到 deadline_us */
static esp_err_t next_frame(struct zdt_uart *h, int64_t deadline_us, zdt_reply_t *out)
{
    for (;;) {
        while (h->rx_len) {
            int r = zdt_ttl_parse(h->rx, h->rx_len, h->checksum, h->firmware, out);
            if (r > 0) {
                h->rx_len -= (size_t)r;
                memmove(h->rx, h->rx + r, h->rx_len);
                return ESP_OK;
            }
            if (r == 0 && h->rx_len < sizeof(h->rx)) break;
            /* 不是有效帧（或缓冲已满仍凑不出一帧）：丢 1 字节重新同步 */
            h->rx_len--;
            memmove(h->rx, h->rx + 1, h->rx_len);
        }
        int64_t left_ms = (deadline_us - esp_timer_get_time() + 999) / 1000;
        if (left_ms <= 0) return ESP_ERR_TIMEOUT;
        /* 先阻塞等 1 字节，再把驱动缓冲里已有的一次取完 */
        int n = uart_read_bytes(h->port, h->rx + h->rx_len, 1, ms_to_ticks(left_ms));
        if (n <= 0) continue;
        h->rx_len += (size_t)n;
        size_t avail = 0;
        uart_get_buffered_data_len(h->port, &avail);
        size_t space = sizeof(h->rx) - h->rx_len;
        if (avail > space) avail = space;
        if (avail) {
            n = uart_read_bytes(h->port, h->rx + h->rx_len, avail, 0);
            if (n > 0) h->rx_len += (size_t)n;
        }
    }
}

static esp_err_t recv_locked(struct zdt_uart *h, uint8_t addr, uint8_t code,
                             zdt_reply_t *reply, int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    for (;;) {
        esp_err_t err = next_frame(h, deadline, reply);
        if (err != ESP_OK) return err;
        bool addr_ok = addr == ZDT_ADDR_BROADCAST || reply->addr == addr;
        if (addr_ok && (reply->code == code || reply->code == 0x00)) {
            if (is_param_write(reply->code)) h->hold_until_us = esp_timer_get_time() + h->write_gap_us;
            return ESP_OK;
        }
        if (h->cb) h->cb(reply, h->cb_ctx);
    }
}

esp_err_t zdt_uart_recv_reply(zdt_uart_handle_t handle, uint8_t addr, uint8_t code,
                              zdt_reply_t *reply, int timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && reply, ESP_ERR_INVALID_ARG, TAG, "null arg");
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    esp_err_t err = recv_locked(handle, addr, code, reply, timeout_ms);
    xSemaphoreGive(handle->lock);
    return err;
}

esp_err_t zdt_uart_transact(zdt_uart_handle_t handle, const uint8_t *frame, size_t len,
                            zdt_reply_t *reply, int timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && reply && frame && len >= 3, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    esp_err_t err = send_locked(handle, frame, len, timeout_ms);
    if (err == ESP_OK) err = recv_locked(handle, frame[0], frame[1], reply, timeout_ms);
    xSemaphoreGive(handle->lock);
    return err;
}

int zdt_uart_poll(zdt_uart_handle_t handle, int timeout_ms)
{
    if (!handle) return 0;
    int n = 0;
    zdt_reply_t f;
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (next_frame(handle, deadline, &f) == ESP_OK) {
        if (handle->cb) handle->cb(&f, handle->cb_ctx);
        n++;
    }
    xSemaphoreGive(handle->lock);
    return n;
}
