/**
 * @file zdt_can.c
 * @brief ZDT X42S 电机驱动库 - CAN 协议实现 (EMM固件)
 */

#include "esp_err.h"
#include "libzdt.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * 内部辅助函数
 * ========================================================================== */

/**
 * @brief 将16位整数写入大端序缓冲区
 */
static inline void write_u16_be(uint8_t *buf, uint16_t val) {
  buf[0] = (val >> 8) & 0xFF;
  buf[1] = val & 0xFF;
}

/**
 * @brief 将32位整数写入大端序缓冲区
 */
static inline void write_u32_be(uint8_t *buf, uint32_t val) {
  buf[0] = (val >> 24) & 0xFF;
  buf[1] = (val >> 16) & 0xFF;
  buf[2] = (val >> 8) & 0xFF;
  buf[3] = val & 0xFF;
}

/**
 * @brief 将有符号32位整数写入大端序缓冲区
 */
static inline void write_i32_be(uint8_t *buf, int32_t val) {
  write_u32_be(buf, (uint32_t)val);
}

/**
 * @brief 从大端序缓冲区读取16位整数
 */
static inline uint16_t read_u16_be(const uint8_t *buf) {
  return ((uint16_t)buf[0] << 8) | buf[1];
}

/**
 * @brief 从大端序缓冲区读取32位整数
 */
static inline uint32_t read_u32_be(const uint8_t *buf) {
  return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
         ((uint32_t)buf[2] << 8) | buf[3];
}

/**
 * @brief 从大端序缓冲区读取有符号32位整数
 */
static inline int32_t read_i32_be(const uint8_t *buf) {
  return (int32_t)read_u32_be(buf);
}

/* ============================================================================
 * 通用命令构建
 * ========================================================================== */

int zdt_build_encoder_calibration(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_ENCODER_CALIBRATION;
  buf->data[2] = ZDT_AUX_ENCODER_CAL;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_restart(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_RESTART;
  buf->data[2] = ZDT_AUX_RESTART;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_clear_position(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_CLEAR_POSITION;
  buf->data[2] = ZDT_AUX_CLEAR_POS;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_clear_protection(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_CLEAR_PROTECTION;
  buf->data[2] = ZDT_AUX_CLEAR_PROT;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_factory_reset(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_FACTORY_RESET;
  buf->data[2] = ZDT_AUX_FACTORY_RESET;
  buf->data[3] = ZDT_CHECKSUM;
  buf->len = 4;

  return 4;
}

int zdt_build_enable(uint8_t addr, zdt_enable_state_t enable,
                     zdt_sync_flag_t sync, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_ENABLE;
  buf->data[2] = ZDT_AUX_ENABLE;
  buf->data[3] = (uint8_t)enable;
  buf->data[4] = (uint8_t)sync;
  buf->data[5] = ZDT_CHECKSUM;
  buf->len = 6;

  return 6;
}

int zdt_build_homing(uint8_t addr, uint8_t mode, zdt_sync_flag_t sync,
                     zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_HOMING;
  buf->data[2] = mode;
  buf->data[3] = (uint8_t)sync;
  buf->data[4] = ZDT_CHECKSUM;
  buf->len = 5;

  return 5;
}

int zdt_build_emergency_stop(uint8_t addr, zdt_sync_flag_t sync,
                             zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_EMERGENCY_STOP;
  buf->data[2] = ZDT_AUX_EMERGENCY_STOP;
  buf->data[3] = (uint8_t)sync;
  buf->data[4] = ZDT_CHECKSUM;
  buf->len = 5;

  return 5;
}

int zdt_build_read_position(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_POSITION;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_pulses(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_PULSES;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_io_status(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_IO_STATUS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_status_flags(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_STATUS_FLAGS;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_read_config(uint8_t addr, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_READ_CONFIG;
  buf->data[2] = ZDT_CHECKSUM;
  buf->len = 3;

  return 3;
}

int zdt_build_timed_return(uint8_t addr, uint8_t info_func_code,
                           uint16_t interval_ms, zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_TIMED_RETURN;
  buf->data[2] = ZDT_AUX_TIMED_RETURN;
  buf->data[3] = info_func_code;
  write_u16_be(&buf->data[4], interval_ms);
  buf->data[6] = ZDT_CHECKSUM;
  buf->len = 7;

  return 7;
}

/* ============================================================================
 * EMM固件运动控制命令构建
 * ========================================================================== */

int zdt_build_speed_mode(uint8_t addr, zdt_direction_t dir, uint16_t speed_rpm,
                         uint8_t acc, zdt_sync_flag_t sync,
                         zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // EMM固件速度模式命令格式 (8字节):
  // Addr | F6 | Dir | Speed[2] | Acc | Sync | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_SPEED_MODE;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], speed_rpm);
  buf->data[5] = acc;
  buf->data[6] = (uint8_t)sync;
  buf->data[7] = ZDT_CHECKSUM;
  buf->len = 8;

  return 8;
}

int zdt_build_position_mode(uint8_t addr, zdt_direction_t dir,
                            uint16_t speed_rpm, uint8_t acc,
                            int32_t position_01deg,
                            zdt_motion_mode_t motion_mode, zdt_sync_flag_t sync,
                            zdt_cmd_buffer_t *buf) {
  if (!buf)
    return ZDT_ERR_INVALID_PARAM;

  // EMM固件位置模式命令格式 (13字节):
  // Addr | FD | Dir | Speed[2] | Acc | Pos[4] | Mode | Sync | 6B
  buf->data[0] = addr;
  buf->data[1] = ZDT_CMD_POSITION_MODE;
  buf->data[2] = (uint8_t)dir;
  write_u16_be(&buf->data[3], speed_rpm);
  buf->data[5] = acc;
  write_i32_be(&buf->data[6], position_01deg);
  buf->data[10] = (uint8_t)motion_mode;
  buf->data[11] = (uint8_t)sync;
  buf->data[12] = ZDT_CHECKSUM;
  buf->len = 13;

  return 13;
}

/* ============================================================================
 * CAN协议层API
 * ========================================================================== */

int zdt_cmd_to_can_msgs(uint8_t addr, const zdt_cmd_buffer_t *cmd,
                        zdt_can_msg_t *msgs, size_t max_msgs) {
  if (!cmd || !msgs || max_msgs == 0) {
    return ZDT_ERR_INVALID_PARAM;
  }

  // 跳过地址字节，CAN帧中地址在ID中
  // cmd->data 格式: [Addr][CmdCode][Data...][Checksum]
  uint8_t cmd_code = cmd->data[1];    // 保存命令码
  const uint8_t *data = cmd->data + 1; // 跳过地址字节，从命令码开始
  size_t data_len = cmd->len - 1;      // 减去地址字节

  // 计算需要的包数
  // 第一包可放8字节数据，后续包每包需要预留1字节给命令码，实际数据7字节
  size_t num_packets;
  if (data_len <= ZDT_CAN_MAX_DATA_LEN) {
    num_packets = 1;
  } else {
    // 第一包8字节，后续每包7字节数据+1字节命令码
    size_t remaining_after_first = data_len - ZDT_CAN_MAX_DATA_LEN;
    num_packets = 1 + (remaining_after_first + (ZDT_CAN_MAX_DATA_LEN - 1) - 1) /
                          (ZDT_CAN_MAX_DATA_LEN - 1);
  }

  if (num_packets > max_msgs) {
    return ZDT_ERR_BUFFER_TOO_SMALL;
  }

  // 拆分数据到CAN帧
  size_t offset = 0;
  for (size_t i = 0; i < num_packets; i++) {
    msgs[i].id = zdt_make_can_id(addr, (uint8_t)i);
    msgs[i].is_extended = true;

    if (i == 0) {
      // 第一包：直接复制数据（包含命令码）
      size_t remaining = data_len - offset;
      msgs[i].data_len = (remaining > ZDT_CAN_MAX_DATA_LEN)
                             ? ZDT_CAN_MAX_DATA_LEN
                             : (uint8_t)remaining;
      memcpy(msgs[i].data, data + offset, msgs[i].data_len);
      offset += msgs[i].data_len;
    } else {
      // 后续包：先放命令码，再放数据
      msgs[i].data[0] = cmd_code;
      size_t remaining = data_len - offset;
      size_t payload_len = (remaining > (ZDT_CAN_MAX_DATA_LEN - 1))
                               ? (ZDT_CAN_MAX_DATA_LEN - 1)
                               : remaining;
      memcpy(msgs[i].data + 1, data + offset, payload_len);
      msgs[i].data_len = (uint8_t)(1 + payload_len);
      offset += payload_len;
    }
  }

  return (int)num_packets;
}

int zdt_can_msgs_to_response(const zdt_can_msg_t *msgs, size_t msg_count,
                             zdt_response_t *response) {
  if (!msgs || !response || msg_count == 0) {
    return ZDT_ERR_INVALID_PARAM;
  }

  // 清空响应结构
  memset(response, 0, sizeof(*response));

  // 解析地址
  uint8_t addr, packet;
  zdt_parse_can_id(msgs[0].id, &addr, &packet);
  response->addr = addr;

  // 组合所有包的数据
  size_t total_len = 0;
  for (size_t i = 0; i < msg_count; i++) {
    if (total_len + msgs[i].data_len > ZDT_MAX_CMD_SIZE) {
      return ZDT_ERR_BUFFER_TOO_SMALL;
    }

    memcpy(response->data + total_len, msgs[i].data, msgs[i].data_len);
    total_len += msgs[i].data_len;
  }

  // 解析功能码和状态
  if (total_len >= 2) {
    response->func_code = response->data[0];
    response->status = response->data[1];

    // 移动额外数据
    if (total_len > 2) {
      response->data_len = total_len - 2;
      memmove(response->data, response->data + 2, response->data_len);
    }
  }

  return ZDT_OK;
}

/* ============================================================================
 * 响应解析API
 * ========================================================================== */

int zdt_parse_position_response(const zdt_response_t *response,
                                zdt_position_t *position) {
  if (!response || !position) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_POSITION) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  // 响应格式: Addr 36 P3 P2 P1 P0 6B
  // 在response中，data包含P3 P2 P1 P0（4字节）
  if (response->data_len < 4) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  position->angle_raw = read_i32_be(response->data);
  position->angle_deg = zdt_01deg_to_deg(position->angle_raw);

  return ZDT_OK;
}

int zdt_parse_pulses_response(const zdt_response_t *response, int32_t *pulses) {
  if (!response || !pulses) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (response->func_code != ZDT_CMD_READ_PULSES) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  if (response->data_len < 4) {
    return ZDT_ERR_INVALID_RESPONSE;
  }

  *pulses = read_i32_be(response->data);

  return ZDT_OK;
}

bool zdt_response_is_ok(const zdt_response_t *response) {
  if (!response)
    return false;

  return response->status == ZDT_RESP_OK ||
         response->status == ZDT_RESP_AT_ZERO ||
         response->status == ZDT_RESP_ACTION_DONE;
}

const char *zdt_response_status_str(uint8_t status) {
  switch (status) {
  case ZDT_RESP_OK:
    return "OK";
  case ZDT_RESP_AT_ZERO:
    return "Already at zero";
  case ZDT_RESP_PARAM_ERROR:
    return "Parameter error";
  case ZDT_RESP_FORMAT_ERROR:
    return "Format error";
  case ZDT_RESP_ACTION_DONE:
    return "Action completed";
  default:
    return "Unknown";
  }
}

/* ============================================================================
 * 调试工具函数
 * ========================================================================== */

void zdt_print_cmd_buffer(const zdt_cmd_buffer_t *buf) {
  if (!buf)
    return;

  printf("CMD [%zu]: ", buf->len);
  for (size_t i = 0; i < buf->len; i++) {
    printf("%02X ", buf->data[i]);
  }
  printf("\n");
}

void zdt_print_can_msg(const zdt_can_msg_t *msg) {
  if (!msg)
    return;

  printf("CAN ID: 0x%08lX (%s), Data [%u]: ", (unsigned long)msg->id,
         msg->is_extended ? "EXT" : "STD", msg->data_len);

  for (int i = 0; i < msg->data_len; i++) {
    printf("%02X ", msg->data[i]);
  }
  printf("\n");
}

/* ============================================================================
 * ESP-IDF CAN驱动集成 (需要CONFIG_IDF_TARGET)
 * ========================================================================== */

#ifdef CONFIG_IDF_TARGET

#include "zdt_can_driver.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "zdt_can";

/** 内部句柄结构 */
struct zdt_handle_s {
  const zdt_can_driver_t *driver;
  void *driver_context;
};

zdt_error_t zdt_can_init(const zdt_can_config_t *config,
                         const zdt_can_driver_t *driver,
                         zdt_handle_t *handle) {
  if (!config || !handle) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (!driver) {
    ESP_LOGE(TAG, "Driver not provided");
    return ZDT_ERR_INVALID_PARAM;
  }

  struct zdt_handle_s *h = calloc(1, sizeof(struct zdt_handle_s));
  if (!h) {
    ESP_LOGE(TAG, "Failed to allocate handle");
    return ZDT_ERR_INVALID_PARAM;
  }

  h->driver = driver;

  // 映射配置到驱动配置
  zdt_can_driver_config_t driver_config = {
      .tx_gpio = config->tx_gpio,
      .rx_gpio = config->rx_gpio,
      .bitrate = config->bitrate,
      .tx_timeout_ms = config->tx_timeout_ms,
      .rx_timeout_ms = config->rx_timeout_ms,
  };

  // 初始化驱动
  int ret = zdt_driver_init(h->driver, &driver_config, &h->driver_context);
  if (ret != 0) {
    free(h);
    return (zdt_error_t)ret;
  }

  *handle = h;

  ESP_LOGI(TAG, "ZDT CAN initialized (TX:%d, RX:%d, %lu bps)", config->tx_gpio,
           config->rx_gpio, (unsigned long)config->bitrate);

  return ZDT_OK;
}

void zdt_can_deinit(zdt_handle_t handle) {
  if (!handle)
    return;

  if (handle->driver) {
    zdt_driver_deinit(handle->driver, handle->driver_context);
  }

  free(handle);
}

zdt_error_t zdt_can_send_cmd(zdt_handle_t handle, uint8_t addr,
                             const zdt_cmd_buffer_t *cmd,
                             zdt_response_t *response) {
  if (!handle || !cmd) {
    return ZDT_ERR_INVALID_PARAM;
  }

  if (!handle->driver) {
    return ZDT_ERR_NOT_INITIALIZED;
  }

  // 转换为CAN消息
  zdt_can_msg_t can_msgs[8];
  int num_msgs = zdt_cmd_to_can_msgs(addr, cmd, can_msgs, 8);
  if (num_msgs < 0) {
    return (zdt_error_t)num_msgs;
  }

  // 发送所有CAN帧
  for (int i = 0; i < num_msgs; i++) {
    zdt_can_driver_msg_t driver_msg = {
        .id = can_msgs[i].id,
        .is_extended = can_msgs[i].is_extended,
        .data_len = can_msgs[i].data_len,
    };
    memcpy(driver_msg.data, can_msgs[i].data, can_msgs[i].data_len);

    int ret = zdt_driver_send(handle->driver, handle->driver_context, &driver_msg);
    if (ret != 0) {
      ESP_LOGE(TAG, "CAN TX failed");
      return (zdt_error_t)ret;
    }
  }

  // 如果需要响应，接收
  if (response) {
    zdt_can_driver_msg_t driver_msg;
    int ret = zdt_driver_receive(handle->driver, handle->driver_context, &driver_msg);
    if (ret != 0) {
      ESP_LOGE(TAG, "CAN RX failed");
      return (zdt_error_t)ret;
    }

    // 转换为响应结构
    zdt_can_msg_t rx_can = {
        .id = driver_msg.id,
        .is_extended = driver_msg.is_extended,
        .data_len = driver_msg.data_len,
    };
    memcpy(rx_can.data, driver_msg.data, driver_msg.data_len);

    zdt_can_msgs_to_response(&rx_can, 1, response);
  }

  return ZDT_OK;
}

/* 便捷函数实现 */

zdt_error_t zdt_encoder_calibration(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_build_encoder_calibration(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, NULL);
}

zdt_error_t zdt_restart(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_build_restart(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, NULL);
}

zdt_error_t zdt_clear_position(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_clear_position(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_clear_protection(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_clear_protection(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_factory_reset(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_factory_reset(addr, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_enable(zdt_handle_t handle, uint8_t addr, bool enable) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_enable(addr, enable ? ZDT_ENABLE_ON : ZDT_ENABLE_OFF,
                   ZDT_SYNC_IMMEDIATE, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_homing(zdt_handle_t handle, uint8_t addr, uint8_t mode) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_homing(addr, mode, ZDT_SYNC_IMMEDIATE, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_emergency_stop(zdt_handle_t handle, uint8_t addr) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_emergency_stop(addr, ZDT_SYNC_IMMEDIATE, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_read_position(zdt_handle_t handle, uint8_t addr,
                              zdt_position_t *position) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;

  zdt_build_read_position(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;

  return zdt_parse_position_response(&response, position);
}

zdt_error_t zdt_read_pulses(zdt_handle_t handle, uint8_t addr,
                            int32_t *pulses) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;

  zdt_build_read_pulses(addr, &cmd);
  zdt_error_t err = zdt_can_send_cmd(handle, addr, &cmd, &response);
  if (err != ZDT_OK)
    return err;

  return zdt_parse_pulses_response(&response, pulses);
}

zdt_error_t zdt_set_timed_return(zdt_handle_t handle, uint8_t addr,
                                 uint8_t info_func_code,
                                 uint16_t interval_ms) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;

  int len = zdt_build_timed_return(addr, info_func_code, interval_ms, &cmd);
  if (len < 0)
    return (zdt_error_t)len;

  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_speed(zdt_handle_t handle, uint8_t addr,
                          zdt_direction_t dir, uint16_t speed_rpm,
                          uint8_t acc) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;
  zdt_build_speed_mode(addr, dir, speed_rpm, acc, ZDT_SYNC_IMMEDIATE, &cmd);
  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

zdt_error_t zdt_set_position(zdt_handle_t handle, uint8_t addr,
                             zdt_direction_t dir, float speed_rpm,
                             float angle_deg, zdt_motion_mode_t mode,
                             bool sync_cache) {
  zdt_cmd_buffer_t cmd;
  zdt_response_t response;

  // 使用默认加速度档位 0 (不限制加速度)
  const uint8_t default_acc = 0;

  // 同步标志: sync_cache=true 时使用缓存等待同步命令
  zdt_sync_flag_t sync = sync_cache ? ZDT_SYNC_CACHE : ZDT_SYNC_IMMEDIATE;

  zdt_build_position_mode(addr, dir, (uint16_t)speed_rpm, default_acc,
                          zdt_deg_to_01deg(angle_deg), mode, sync, &cmd);

  return zdt_can_send_cmd(handle, addr, &cmd, &response);
}

#endif /* CONFIG_IDF_TARGET */
