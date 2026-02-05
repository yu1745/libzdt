/**
 * @file zdt_can_twai.c
 * @brief ESP-IDF TWAI驱动实现
 *
 * 仅在定义了 ZDT_CAN_USE_TWAI 时编译
 */

#ifdef ZDT_CAN_USE_TWAI

#include "zdt_can_driver.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "zdt_twai";

/** TWAI驱动上下文 */
typedef struct {
    zdt_can_driver_config_t config;
    SemaphoreHandle_t mutex;
    bool initialized;
} zdt_twai_context_t;

/* ============================================================================
 * TWAI驱动实现
 * ========================================================================== */

static int twai_init(const zdt_can_driver_config_t *config, void **context) {
    if (!config || !context) {
        return -1;
    }

    zdt_twai_context_t *ctx = calloc(1, sizeof(zdt_twai_context_t));
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate context");
        return -1;
    }

    ctx->config = *config;
    ctx->mutex = xSemaphoreCreateMutex();
    if (!ctx->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(ctx);
        return -1;
    }

    // 配置TWAI
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        config->tx_gpio, config->rx_gpio, TWAI_MODE_NORMAL);

    twai_timing_config_t t_config;
    if (config->bitrate == 250000) {
        t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
    } else if (config->bitrate == 500000) {
        t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
    } else if (config->bitrate == 1000000) {
        t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
    } else {
        t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        vSemaphoreDelete(ctx->mutex);
        free(ctx);
        return -3;
    }

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        vSemaphoreDelete(ctx->mutex);
        free(ctx);
        return -3;
    }

    ctx->initialized = true;
    *context = ctx;

    ESP_LOGI(TAG, "TWAI driver initialized (TX:%d, RX:%d, %lu bps)",
             config->tx_gpio, config->rx_gpio, (unsigned long)config->bitrate);

    return 0;
}

static void twai_deinit(void *context) {
    if (!context) {
        return;
    }

    zdt_twai_context_t *ctx = (zdt_twai_context_t *)context;

    if (ctx->initialized) {
        twai_stop();
        twai_driver_uninstall();
    }

    if (ctx->mutex) {
        vSemaphoreDelete(ctx->mutex);
    }

    free(ctx);
}

static int twai_send(void *context, const zdt_can_driver_msg_t *msg) {
    if (!context || !msg) {
        return -1;
    }

    zdt_twai_context_t *ctx = (zdt_twai_context_t *)context;

    if (!ctx->initialized) {
        return -7;
    }

    xSemaphoreTake(ctx->mutex, portMAX_DELAY);

    twai_message_t tx_msg = {
        .extd = msg->is_extended ? 1 : 0,
        .identifier = msg->id,
        .data_length_code = msg->data_len,
    };
    memcpy(tx_msg.data, msg->data, msg->data_len);

    // 打印CAN发送内容
    char hexstr[3 * 8 + 1] = {0};
    int offset = 0;
    for (int i = 0; i < tx_msg.data_length_code; ++i) {
        offset += snprintf(hexstr + offset, sizeof(hexstr) - offset, "%02X ",
                          tx_msg.data[i]);
    }
    ESP_LOGI(TAG, "CAN Tx: id=0x%08X, data(%d): %s",
             tx_msg.identifier, tx_msg.data_length_code, hexstr);

    esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(ctx->config.tx_timeout_ms));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN TX failed: %s", esp_err_to_name(err));
        xSemaphoreGive(ctx->mutex);
        return -3;
    }

    xSemaphoreGive(ctx->mutex);
    return 0;
}

static int twai_driver_receive(void *context, zdt_can_driver_msg_t *msg) {
    if (!context || !msg) {
        return -1;
    }

    zdt_twai_context_t *ctx = (zdt_twai_context_t *)context;

    if (!ctx->initialized) {
        return -7;
    }

    xSemaphoreTake(ctx->mutex, portMAX_DELAY);

    twai_message_t rx_msg;
    esp_err_t err = twai_receive(&rx_msg, pdMS_TO_TICKS(ctx->config.rx_timeout_ms));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN RX timeout: %s", esp_err_to_name(err));
        xSemaphoreGive(ctx->mutex);
        return -4;
    }

    msg->id = rx_msg.identifier;
    msg->is_extended = rx_msg.extd ? true : false;
    msg->data_len = rx_msg.data_length_code;
    memcpy(msg->data, rx_msg.data, rx_msg.data_length_code);

    // 打印CAN接收内容
    char hexstr[3 * 8 + 1] = {0};
    int offset = 0;
    for (int i = 0; i < msg->data_len; ++i) {
        offset += snprintf(hexstr + offset, sizeof(hexstr) - offset, "%02X ",
                          msg->data[i]);
    }
    ESP_LOGI(TAG, "CAN Rx: id=0x%08X, data(%d): %s",
             msg->id, msg->data_len, hexstr);

    xSemaphoreGive(ctx->mutex);
    return 0;
}

/* ============================================================================
 * 驱动接口导出
 * ========================================================================== */

static const zdt_can_driver_t twai_driver = {
    .init = twai_init,
    .deinit = twai_deinit,
    .send = twai_send,
    .receive = twai_driver_receive,
};

const zdt_can_driver_t* zdt_get_twai_driver(void) {
    return &twai_driver;
}

#endif /* ZDT_CAN_USE_TWAI */
