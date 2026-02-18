/**
 * @file zdt_twai.h
 * @brief ZDT电机TWAI驱动接口
 *
 * 基于ESP-IDF TWAI驱动实现CAN总线通讯
 */

#ifndef ZDT_TWAI_H
#define ZDT_TWAI_H

#include "libzdt.h"
#include "sdkconfig.h"
#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_ZDT_ENABLE_TWAI_DRIVER

/*==============================================================================
 * 配置结构
 *============================================================================*/

/**
 * @brief TWAI配置参数
 */
typedef struct {
  uint32_t tx_gpio;     /* TX引脚号 */
  uint32_t rx_gpio;     /* RX引脚号 */
  uint32_t bitrate;     /* 波特率（默认1000000 = 1Mbps） */
  uint8_t rx_queue_len; /* 接收队列长度 */
  bool enable_tx_queue; /* 是否启用发送队列 */
  uint8_t tx_queue_len; /* 发送队列长度 */
} zdt_twai_config_t;

/**
 * @brief 默认TWAI配置
 *
 * 注意：使用时需要根据实际硬件修改GPIO引脚号
 */
#define ZDT_TWAI_DEFAULT_CONFIG()                                              \
  {.tx_gpio = 7,                                                               \
   .rx_gpio = 6,                                                               \
   .bitrate = 1000 * 1000,                                                      \
   .rx_queue_len = 20,                                                         \
   .enable_tx_queue = true,                                                    \
   .tx_queue_len = 10}

/*==============================================================================
 * 接收回调函数类型
 *============================================================================*/

/**
 * @brief CAN消息接收回调函数类型
 * @param msg 接收到的CAN消息
 * @param user_arg 用户自定义参数
 */
typedef void (*zdt_twai_rx_callback_t)(const zdt_can_msg_t *msg,
                                       void *user_arg);

/*==============================================================================
 * TWAI驱动API
 *============================================================================*/

/**
 * @brief 初始化TWAI驱动
 * @param config TWAI配置参数
 * @return 0=成功, 负数=错误码
 */
int zdt_twai_init(const zdt_twai_config_t *config);

/**
 * @brief 反初始化TWAI驱动
 * @return 0=成功, 负数=错误码
 */
int zdt_twai_deinit(void);

/**
 * @brief 启动TWAI驱动
 * @return 0=成功, 负数=错误码
 */
int zdt_twai_start(void);

/**
 * @brief 停止TWAI驱动
 * @return 0=成功, 负数=错误码
 */
int zdt_twai_stop(void);

/**
 * @brief 发送单个CAN消息
 * @param msg 要发送的CAN消息
 * @param timeout_ms 超时时间（毫秒）
 * @return 0=成功, 负数=错误码
 */
int zdt_twai_send(const zdt_can_msg_t *msg, uint32_t timeout_ms);

/**
 * @brief 发送多个CAN消息（分包消息）
 * @param msgs CAN消息数组
 * @param num 消息数量
 * @param interval_us 消息间隔（微秒）
 * @param timeout_ms 单个消息超时时间（毫秒）
 * @return 实际发送的消息数量，负数=错误码
 */
int zdt_twai_send_multi(const zdt_can_msg_t *msgs, uint8_t num,
                        uint32_t timeout_ms);

/**
 * @brief 接收CAN消息（阻塞）
 * @param msg 接收消息的缓冲区
 * @param timeout_ms 超时时间（毫秒）
 * @return 0=成功, 负数=错误码
 */
int zdt_twai_receive(zdt_can_msg_t *msg, uint32_t timeout_ms);

/**
 * @brief 注册接收回调函数
 * @param callback 回调函数指针
 * @param user_arg 用户自定义参数
 * @return 0=成功, 负数=错误码
 *
 * @note 注册回调后会创建接收任务
 */
int zdt_twai_register_callback(zdt_twai_rx_callback_t callback, void *user_arg);

/**
 * @brief 注销接收回调函数
 * @return 0=成功, 负数=错误码
 *
 * @note 会停止接收任务
 */
int zdt_twai_unregister_callback(void);

/**
 * @brief 获取TWAI驱动状态
 * @return true=已启动, false=已停止
 */
bool zdt_twai_is_running(void);

/**
 * @brief 获取接收消息计数
 * @return 接收消息数量
 */
uint32_t zdt_twai_get_rx_count(void);

/**
 * @brief 获取发送消息计数
 * @return 发送消息数量
 */
uint32_t zdt_twai_get_tx_count(void);

/**
 * @brief 获取错误计数
 * @return 错误数量
 */
uint32_t zdt_twai_get_error_count(void);

/**
 * @brief 清除统计计数
 */
void zdt_twai_clear_stats(void);

/**
 * @brief 获取最后一次 TWAI 操作的错误码
 * @return esp_err_t 错误码，ESP_OK 表示无错误
 */
int zdt_twai_get_last_error(void);

/**
 * @brief 从 ESP_ERR_INVALID_STATE 恢复：杀死接收任务，重启 TWAI 驱动
 * 调用前需确保已通过 zdt_twai_register_callback 注册过回调
 * @return 0=成功, 负数=错误码
 */
int zdt_twai_recover_from_invalid_state(void);

#endif /* CONFIG_ZDT_ENABLE_TWAI_DRIVER */

#ifdef __cplusplus
}
#endif

#endif /* ZDT_TWAI_H */
