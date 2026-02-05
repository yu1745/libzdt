/**
 * @file zdt_can_driver.h
 * @brief CAN驱动抽象接口层
 *
 * 提供CAN驱动抽象接口，支持不同的底层CAN实现（如ESP-IDF TWAI、Linux SocketCAN等）
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * CAN驱动抽象接口
 * ========================================================================== */

/**
 * @brief CAN消息结构（与zdt_can_msg_t保持一致）
 */
typedef struct {
    uint32_t id;                    ///< 扩展帧ID
    uint8_t  data[8];               ///< 数据
    uint8_t  data_len;              ///< 数据长度 (0-8)
    bool     is_extended;           ///< 是否为扩展帧
} zdt_can_driver_msg_t;

/**
 * @brief CAN驱动配置
 */
typedef struct {
    int      tx_gpio;               ///< CAN TX GPIO (平台相关)
    int      rx_gpio;               ///< CAN RX GPIO (平台相关)
    uint32_t bitrate;               ///< 波特率
    uint32_t tx_timeout_ms;         ///< 发送超时 (ms)
    uint32_t rx_timeout_ms;         ///< 接收超时 (ms)
} zdt_can_driver_config_t;

/**
 * @brief CAN驱动接口（不透明指针）
 *
 * 驱动结构体定义在源文件中，头文件只做前向声明
 * 通过工厂函数获取驱动实例
 */
typedef struct zdt_can_driver_s zdt_can_driver_t;

/* ============================================================================
 * 驱动访问函数
 * ========================================================================== */

/**
 * @brief 驱动初始化
 * @param driver 驱动实例
 * @param config 驱动配置
 * @param context 输出驱动上下文
 * @return 0=成功, 负数=错误码
 */
static inline int zdt_driver_init(const zdt_can_driver_t *driver,
                                  const zdt_can_driver_config_t *config,
                                  void **context) {
    return driver->init(config, context);
}

/**
 * @brief 驱动反初始化
 * @param driver 驱动实例
 * @param context 驱动上下文
 */
static inline void zdt_driver_deinit(const zdt_can_driver_t *driver,
                                     void *context) {
    driver->deinit(context);
}

/**
 * @brief 发送CAN消息
 * @param driver 驱动实例
 * @param context 驱动上下文
 * @param msg CAN消息
 * @return 0=成功, 负数=错误码
 */
static inline int zdt_driver_send(const zdt_can_driver_t *driver,
                                  void *context,
                                  const zdt_can_driver_msg_t *msg) {
    return driver->send(context, msg);
}

/**
 * @brief 接收CAN消息
 * @param driver 驱动实例
 * @param context 驱动上下文
 * @param msg 输出CAN消息
 * @return 0=成功, 负数=错误码
 */
static inline int zdt_driver_receive(const zdt_can_driver_t *driver,
                                     void *context,
                                     zdt_can_driver_msg_t *msg) {
    return driver->receive(context, msg);
}

/* ============================================================================
 * 驱动实现选择
 * ========================================================================== */

/**
 * @def ZDT_CAN_DRIVER_IMPL
 * @brief 定义要使用的CAN驱动实现
 *
 * 可选值:
 * - ZDT_CAN_DRIVER_TWAI: ESP-IDF TWAI驱动 (需要ESP-IDF环境)
 * - 自定义: 用户在应用层实现
 *
 * @note 如果不定义任何驱动，需要在应用层提供驱动实现
 */
// #define ZDT_CAN_DRIVER_IMPL  ZDT_CAN_DRIVER_TWAI

/* ============================================================================
 * ESP-IDF TWAI驱动实现
 * ========================================================================== */

#if defined(ZDT_CAN_DRIVER_IMPL) && ZDT_CAN_DRIVER_IMPL == ZDT_CAN_DRIVER_TWAI

/**
 * @brief 获取ESP-IDF TWAI驱动实现
 * @return CAN驱动接口指针
 */
const zdt_can_driver_t* zdt_get_twai_driver(void);

#endif /* ZDT_CAN_DRIVER_IMPL == ZDT_CAN_DRIVER_TWAI */

#ifdef __cplusplus
}
#endif
