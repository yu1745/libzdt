/**
 * @file zdt_twai.c
 * @brief ZDT电机TWAI驱动实现
 *
 * 基于ESP-IDF TWAI驱动实现CAN总线通讯
 */

#include "zdt_twai.h"
#if CONFIG_ZDT_ENABLE_TWAI_DRIVER

#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>


/*==============================================================================
 * 宏定义
 *============================================================================*/

#define TAG "ZDT_TWAI"

#define TWAI_RX_TASK_STACK_SIZE (4096)
#define TWAI_RX_TASK_PRIORITY (12)
#define TWAI_RX_TASK_DELAY_MS (10)

/*==============================================================================
 * 全局变量
 *============================================================================*/

#define TWAI_RECOVERY_TASK_STACK_SIZE (4096)
#define TWAI_RECOVERY_TASK_PRIORITY (10)

static struct {
  twai_mode_t mode;
  uint32_t tx_gpio;
  uint32_t rx_gpio;
  uint32_t bitrate;
  bool is_running;
  QueueHandle_t tx_queue;
  zdt_twai_rx_callback_t rx_callback;
  void *rx_callback_arg;
  TaskHandle_t rx_task_handle;
  uint32_t rx_count;
  uint32_t tx_count;
  uint32_t error_count;
  int last_error;
  zdt_twai_config_t saved_config;
  bool config_saved;
  SemaphoreHandle_t recovery_mutex;
} twai_ctx = {.mode = TWAI_MODE_NORMAL,
              .tx_gpio = GPIO_NUM_NC,
              .rx_gpio = GPIO_NUM_NC,
              .bitrate = 1000000,
              .is_running = false,
              .tx_queue = NULL,
              .rx_callback = NULL,
              .rx_callback_arg = NULL,
              .rx_task_handle = NULL,
              .rx_count = 0,
              .tx_count = 0,
              .error_count = 0,
              .last_error = 0,
              .saved_config = {0},
              .config_saved = false,
              .recovery_mutex = NULL};

/*==============================================================================
 * 内部函数声明
 *============================================================================*/

static void twai_rx_task(void *arg);
static void twai_recovery_task(void *arg);
static esp_err_t twai_convert_to_esp_msg(const zdt_can_msg_t *zdt_msg,
                                         twai_message_t *esp_msg);
static esp_err_t twai_convert_from_esp_msg(const twai_message_t *esp_msg,
                                           zdt_can_msg_t *zdt_msg);

/*==============================================================================
 * TWAI驱动API实现
 *============================================================================*/

int zdt_twai_init(const zdt_twai_config_t *config) {
  if (twai_ctx.is_running) {
    ESP_LOGW(TAG, "TWAI driver already initialized");
    return 0;
  }

  if (config == NULL) {
    ESP_LOGE(TAG, "Invalid config parameter");
    return -1;
  }

  /* 保存配置（用于恢复） */
  twai_ctx.tx_gpio = config->tx_gpio;
  twai_ctx.rx_gpio = config->rx_gpio;
  twai_ctx.bitrate = config->bitrate;
  twai_ctx.saved_config = *config;
  twai_ctx.config_saved = true;

  /* 配置TWAI时序 */
  twai_timing_config_t t_config;
  if (config->bitrate == 1000000) {
    t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
  } else if (config->bitrate == 500000) {
    t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
  } else if (config->bitrate == 250000) {
    t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
  } else if (config->bitrate == 125000) {
    t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
  } else {
    ESP_LOGE(TAG, "Unsupported bitrate: %lu", config->bitrate);
    return -1;
  }

  /* 配置TWAI过滤器（接收所有消息） */
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  /* 配置TWAI通用设置 */
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      config->tx_gpio, config->rx_gpio, twai_ctx.mode);
  g_config.tx_queue_len = config->enable_tx_queue ? config->tx_queue_len : 0;

  /* 安装TWAI驱动 */
  esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
    return -1;
  }

  /* 创建发送队列（如果启用） */
  if (config->enable_tx_queue && config->tx_queue_len > 0) {
    twai_ctx.tx_queue =
        xQueueCreate(config->tx_queue_len, sizeof(zdt_can_msg_t));
    if (twai_ctx.tx_queue == NULL) {
      ESP_LOGE(TAG, "Failed to create TX queue");
      twai_driver_uninstall();
      return -1;
    }
  }

  ESP_LOGI(TAG, "TWAI driver initialized (TX=%d, RX=%d, Bitrate=%lu)",
           config->tx_gpio, config->rx_gpio, config->bitrate);

  return 0;
}

int zdt_twai_deinit(void) {
  if (twai_ctx.is_running) {
    ESP_LOGW(TAG, "TWAI driver is running, stop it first");
    return -1;
  }

  /* 停止接收任务 */
  if (twai_ctx.rx_task_handle != NULL) {
    zdt_twai_unregister_callback();
  }

  /* 删除发送队列 */
  if (twai_ctx.tx_queue != NULL) {
    vQueueDelete(twai_ctx.tx_queue);
    twai_ctx.tx_queue = NULL;
  }

  /* 卸载TWAI驱动 */
  esp_err_t ret = twai_driver_uninstall();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to uninstall TWAI driver: %s", esp_err_to_name(ret));
    return -1;
  }

  ESP_LOGI(TAG, "TWAI driver deinitialized");
  return 0;
}

int zdt_twai_start(void) {
  if (twai_ctx.is_running) {
    ESP_LOGW(TAG, "TWAI driver already started");
    return 0;
  }

  esp_err_t ret = twai_start();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
    return -1;
  }

  twai_ctx.is_running = true;
  ESP_LOGI(TAG, "TWAI driver started");
  return 0;
}

int zdt_twai_stop(void) {
  if (!twai_ctx.is_running) {
    ESP_LOGW(TAG, "TWAI driver already stopped");
    return 0;
  }

  esp_err_t ret = twai_stop();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to stop TWAI driver: %s", esp_err_to_name(ret));
    return -1;
  }

  twai_ctx.is_running = false;
  ESP_LOGI(TAG, "TWAI driver stopped");
  return 0;
}

int zdt_twai_send(const zdt_can_msg_t *msg, uint32_t timeout_ms) {
  if (!twai_ctx.is_running) {
    ESP_LOGE(TAG, "TWAI driver not running");
    return -1;
  }

  if (msg == NULL || msg->dlc > 8) {
    ESP_LOGE(TAG, "Invalid message parameter");
    return -1;
  }

  twai_message_t esp_msg;
  esp_err_t ret = twai_convert_to_esp_msg(msg, &esp_msg);
  if (ret != ESP_OK) {
    return -1;
  }

  /* 发送消息 */
  ret = twai_transmit(&esp_msg, pdMS_TO_TICKS(timeout_ms));
  if (ret != ESP_OK) {
    twai_ctx.last_error = (int)ret;
    ESP_LOGE(TAG, "Failed to transmit message: %s", esp_err_to_name(ret));
    twai_ctx.error_count++;
    return -1;
  }

  twai_ctx.tx_count++;
  return 0;
}

int zdt_twai_send_multi(const zdt_can_msg_t *msgs, uint8_t num,uint32_t timeout_ms) {
  if (!twai_ctx.is_running) {
    ESP_LOGE(TAG, "TWAI driver not running");
    return -1;
  }

  if (msgs == NULL || num == 0) {
    ESP_LOGE(TAG, "Invalid message array");
    return -1;
  }

  int sent_count = 0;

  for (uint8_t i = 0; i < num; i++) {
    if (zdt_twai_send(&msgs[i], timeout_ms) == 0) {
      sent_count++;
    } else {
      ESP_LOGE(TAG, "Failed to send message %d/%d", i + 1, num);
      break;
    }
  }

  return sent_count;
}

int zdt_twai_receive(zdt_can_msg_t *msg, uint32_t timeout_ms) {
  if (!twai_ctx.is_running) {
    ESP_LOGE(TAG, "TWAI driver not running");
    return -1;
  }

  if (msg == NULL) {
    ESP_LOGE(TAG, "Invalid message buffer");
    return -1;
  }

  twai_message_t esp_msg;
  esp_err_t ret = twai_receive(&esp_msg, pdMS_TO_TICKS(timeout_ms));
  if (ret != ESP_OK) {
    if (ret != ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "Failed to receive message: %s", esp_err_to_name(ret));
      twai_ctx.error_count++;
    }
    return -1;
  }

  ret = twai_convert_from_esp_msg(&esp_msg, msg);
  if (ret != ESP_OK) {
    return -1;
  }

  twai_ctx.rx_count++;
  return 0;
}

int zdt_twai_register_callback(zdt_twai_rx_callback_t callback,
                               void *user_arg) {
  if (twai_ctx.rx_task_handle != NULL) {
    ESP_LOGW(TAG, "RX task already running");
    return -1;
  }

  twai_ctx.rx_callback = callback;
  twai_ctx.rx_callback_arg = user_arg;

  /* 创建接收任务 */
  BaseType_t ret =
      xTaskCreate(twai_rx_task, "twai_rx", TWAI_RX_TASK_STACK_SIZE, NULL,
                  TWAI_RX_TASK_PRIORITY, &twai_ctx.rx_task_handle);

  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create RX task");
    twai_ctx.rx_callback = NULL;
    twai_ctx.rx_callback_arg = NULL;
    return -1;
  }

  ESP_LOGI(TAG, "RX callback registered and task started");
  return 0;
}

int zdt_twai_unregister_callback(void) {
  if (twai_ctx.rx_task_handle == NULL) {
    ESP_LOGW(TAG, "No RX task running");
    return 0;
  }

  /* 删除接收任务 */
  vTaskDelete(twai_ctx.rx_task_handle);
  twai_ctx.rx_task_handle = NULL;
  twai_ctx.rx_callback = NULL;
  twai_ctx.rx_callback_arg = NULL;

  ESP_LOGI(TAG, "RX callback unregistered");
  return 0;
}

bool zdt_twai_is_running(void) { return twai_ctx.is_running; }

uint32_t zdt_twai_get_rx_count(void) { return twai_ctx.rx_count; }

uint32_t zdt_twai_get_tx_count(void) { return twai_ctx.tx_count; }

uint32_t zdt_twai_get_error_count(void) { return twai_ctx.error_count; }

void zdt_twai_clear_stats(void) {
  twai_ctx.rx_count = 0;
  twai_ctx.tx_count = 0;
  twai_ctx.error_count = 0;
}

int zdt_twai_get_last_error(void) { return twai_ctx.last_error; }

int zdt_twai_recover_from_invalid_state(void) {
  if (!twai_ctx.config_saved) {
    ESP_LOGE(TAG, "Recovery failed: no saved config");
    return -1;
  }
  /* 防止并发恢复 */
  if (twai_ctx.recovery_mutex == NULL) {
    twai_ctx.recovery_mutex = xSemaphoreCreateMutex();
  }
  if (twai_ctx.recovery_mutex != NULL &&
      xSemaphoreTake(twai_ctx.recovery_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    ESP_LOGW(TAG, "Recovery already in progress, skip");
    return -1;
  }

  zdt_twai_rx_callback_t saved_cb = twai_ctx.rx_callback;
  void *saved_arg = twai_ctx.rx_callback_arg;

  /* 1. 杀死接收任务 */
  if (twai_ctx.rx_task_handle != NULL) {
    vTaskDelete(twai_ctx.rx_task_handle);
    twai_ctx.rx_task_handle = NULL;
  }
  twai_ctx.rx_callback = NULL;
  twai_ctx.rx_callback_arg = NULL;

  vTaskDelay(pdMS_TO_TICKS(50));

  /* 2. 停止并反初始化 TWAI
   * 当驱动处于 BUS_OFF 等异常状态时，twai_stop() 会返回 ESP_ERR_INVALID_STATE，
   * 但 twai_driver_uninstall() 在 BUS_OFF 状态下可成功，故 stop 失败时直接尝试 uninstall */
  if (twai_ctx.is_running) {
    if (zdt_twai_stop() != 0) {
      ESP_LOGW(TAG, "Recovery: twai_stop failed (e.g. BUS_OFF), trying direct uninstall");
      twai_ctx.is_running = false;
    }
  }
  /* 删除我们的 tx_queue，然后卸载 ESP-IDF 驱动 */
  if (twai_ctx.tx_queue != NULL) {
    vQueueDelete(twai_ctx.tx_queue);
    twai_ctx.tx_queue = NULL;
  }
  esp_err_t uninstall_ret = twai_driver_uninstall();
  if (uninstall_ret != ESP_OK) {
    ESP_LOGE(TAG, "Recovery: twai_driver_uninstall failed: %s",
             esp_err_to_name(uninstall_ret));
    goto recovery_fail;
  }

  /* 3. 重新初始化并启动 */
  if (zdt_twai_init(&twai_ctx.saved_config) != 0) {
    ESP_LOGE(TAG, "Recovery: zdt_twai_init failed");
    goto recovery_fail;
  }
  if (zdt_twai_start() != 0) {
    ESP_LOGE(TAG, "Recovery: zdt_twai_start failed");
    zdt_twai_deinit();
    goto recovery_fail;
  }

  /* 4. 重新注册回调（重建接收任务） */
  if (saved_cb != NULL && zdt_twai_register_callback(saved_cb, saved_arg) != 0) {
    ESP_LOGE(TAG, "Recovery: zdt_twai_register_callback failed");
    goto recovery_fail;
  }

  twai_ctx.last_error = 0;
  ESP_LOGI(TAG, "TWAI recovery completed successfully");
  if (twai_ctx.recovery_mutex != NULL) {
    xSemaphoreGive(twai_ctx.recovery_mutex);
  }
  return 0;

recovery_fail:
  if (twai_ctx.recovery_mutex != NULL) {
    xSemaphoreGive(twai_ctx.recovery_mutex);
  }
  return -1;
}

/*==============================================================================
 * 内部函数实现
 *============================================================================*/

/**
 * @brief TWAI 恢复任务（从 RX 任务内触发）
 */
static void twai_recovery_task(void *arg) {
  struct {
    zdt_twai_rx_callback_t cb;
    void *arg;
  } *params = (typeof(params))arg;
  zdt_twai_rx_callback_t saved_cb = params ? params->cb : NULL;
  void *saved_arg = params ? params->arg : NULL;
  if (params != NULL) {
    vPortFree(params);
  }

  vTaskDelay(pdMS_TO_TICKS(100));

  /* 防止与 executor 触发的恢复并发 */
  if (twai_ctx.recovery_mutex == NULL) {
    twai_ctx.recovery_mutex = xSemaphoreCreateMutex();
  }
  if (twai_ctx.recovery_mutex != NULL &&
      xSemaphoreTake(twai_ctx.recovery_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    ESP_LOGW(TAG, "Recovery task: another recovery in progress, exit");
    vTaskDelete(NULL);
    return;
  }

  /* 此时 rx_task 已退出，rx_task_handle 已为 NULL */
  twai_ctx.rx_callback = NULL;
  twai_ctx.rx_callback_arg = NULL;

  if (!twai_ctx.config_saved) {
    ESP_LOGE(TAG, "Recovery task: no saved config");
    if (twai_ctx.recovery_mutex != NULL) {
      xSemaphoreGive(twai_ctx.recovery_mutex);
    }
    vTaskDelete(NULL);
    return;
  }

  /* 停止 TWAI；若 stop 失败（如 BUS_OFF），直接尝试 uninstall */
  if (twai_ctx.is_running) {
    if (zdt_twai_stop() != 0) {
      ESP_LOGW(TAG, "Recovery task: twai_stop failed, trying direct uninstall");
      twai_ctx.is_running = false;
    }
  }
  if (twai_ctx.tx_queue != NULL) {
    vQueueDelete(twai_ctx.tx_queue);
    twai_ctx.tx_queue = NULL;
  }
  esp_err_t uninstall_ret = twai_driver_uninstall();
  if (uninstall_ret != ESP_OK) {
    ESP_LOGE(TAG, "Recovery task: twai_driver_uninstall failed: %s",
             esp_err_to_name(uninstall_ret));
    goto rec_task_fail;
  }

  /* 重新初始化并启动 */
  if (zdt_twai_init(&twai_ctx.saved_config) != 0) {
    ESP_LOGE(TAG, "Recovery task: zdt_twai_init failed");
    goto rec_task_fail;
  }
  if (zdt_twai_start() != 0) {
    ESP_LOGE(TAG, "Recovery task: zdt_twai_start failed");
    zdt_twai_deinit();
    goto rec_task_fail;
  }

  /* 重新注册回调 */
  if (saved_cb != NULL) {
    zdt_twai_register_callback(saved_cb, saved_arg);
  }

  twai_ctx.last_error = 0;
  ESP_LOGI(TAG, "TWAI recovery task completed successfully");
  if (twai_ctx.recovery_mutex != NULL) {
    xSemaphoreGive(twai_ctx.recovery_mutex);
  }
  vTaskDelete(NULL);
  return;

rec_task_fail:
  if (twai_ctx.recovery_mutex != NULL) {
    xSemaphoreGive(twai_ctx.recovery_mutex);
  }
  vTaskDelete(NULL);
}

/**
 * @brief TWAI接收任务
 */
static void twai_rx_task(void *arg) {
  ESP_LOGI(TAG, "RX task started");

  twai_message_t esp_msg;
  zdt_can_msg_t zdt_msg;

  while (twai_ctx.is_running) {
    /* 接收消息 */
    esp_err_t ret = twai_receive(&esp_msg, pdMS_TO_TICKS(100));

    if (ret == ESP_OK) {
      /* 转换为ZDT格式 */
      if (twai_convert_from_esp_msg(&esp_msg, &zdt_msg) == ESP_OK) {
        /* 调用回调函数 */
        if (twai_ctx.rx_callback != NULL) {
          twai_ctx.rx_callback(&zdt_msg, twai_ctx.rx_callback_arg);
        }
        twai_ctx.rx_count++;
      }
    } else if (ret == ESP_ERR_TIMEOUT) {
      /* 超时是正常的，继续循环 */
    } else if (ret == ESP_ERR_INVALID_STATE) {
      twai_ctx.last_error = (int)ret;
      twai_ctx.error_count++;
      ESP_LOGE(TAG, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      ESP_LOGE(TAG, "!!! TWAI ESP_ERR_INVALID_STATE - KILLING RX TASK  !!!");
      ESP_LOGE(TAG, "!!! AND RESTARTING TWAI DRIVER - RECOVERY...     !!!");
      ESP_LOGE(TAG, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      /* 保存回调信息，创建恢复任务后退出 */
      zdt_twai_rx_callback_t saved_cb = twai_ctx.rx_callback;
      void *saved_arg = twai_ctx.rx_callback_arg;
      twai_ctx.rx_callback = NULL;
      twai_ctx.rx_callback_arg = NULL;
      twai_ctx.rx_task_handle = NULL;
      /* 创建恢复任务（恢复任务会 stop/deinit/init/start/register） */
      struct {
        zdt_twai_rx_callback_t cb;
        void *arg;
      } *params = pvPortMalloc(sizeof(*params));
      if (params != NULL) {
        params->cb = saved_cb;
        params->arg = saved_arg;
        xTaskCreate(twai_recovery_task, "twai_rec", TWAI_RECOVERY_TASK_STACK_SIZE,
                    params, TWAI_RECOVERY_TASK_PRIORITY, NULL);
      }
      vTaskDelete(NULL);
    } else {
      twai_ctx.last_error = (int)ret;
      ESP_LOGE(TAG, "RX task error: %s", esp_err_to_name(ret));
      twai_ctx.error_count++;
      vTaskDelay(pdMS_TO_TICKS(TWAI_RX_TASK_DELAY_MS));
    }
  }

  ESP_LOGI(TAG, "RX task stopped");
  vTaskDelete(NULL);
}

/**
 * @brief 将ZDT消息转换为ESP格式
 */
static esp_err_t twai_convert_to_esp_msg(const zdt_can_msg_t *zdt_msg,
                                         twai_message_t *esp_msg) {
  if (zdt_msg == NULL || esp_msg == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(esp_msg, 0, sizeof(twai_message_t));

  esp_msg->identifier = zdt_msg->id;
  esp_msg->extd = 1; /* 扩展帧 */
  esp_msg->data_length_code = zdt_msg->dlc;
  memcpy(esp_msg->data, zdt_msg->data, zdt_msg->dlc);

  return ESP_OK;
}

/**
 * @brief 将ESP格式转换为ZDT消息
 */
static esp_err_t twai_convert_from_esp_msg(const twai_message_t *esp_msg,
                                           zdt_can_msg_t *zdt_msg) {
  if (esp_msg == NULL || zdt_msg == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(zdt_msg, 0, sizeof(zdt_can_msg_t));

  zdt_msg->id = esp_msg->identifier;
  zdt_msg->dlc = esp_msg->data_length_code;
  memcpy(zdt_msg->data, esp_msg->data, esp_msg->data_length_code);

  return ESP_OK;
}

#endif /* CONFIG_ZDT_ENABLE_TWAI_DRIVER */
