#ifndef ZDT_INTERNAL_H
#define ZDT_INTERNAL_H

#include <stdint.h>
#include <stddef.h>
#include "zdt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline int zdt_check_size(size_t current_len, size_t needed, size_t max_len) {
    if (current_len + needed > max_len) {
        return ZDT_ERR_BUF_TOO_SMALL;
    }
    return ZDT_OK;
}

static inline int zdt_append_u8(uint8_t *buf, size_t *offset, size_t max_len, uint8_t val) {
    if (zdt_check_size(*offset, 1, max_len) != ZDT_OK) {
        return ZDT_ERR_BUF_TOO_SMALL;
    }
    buf[(*offset)++] = val;
    return ZDT_OK;
}

static inline int zdt_append_u16_be(uint8_t *buf, size_t *offset, size_t max_len, uint16_t val) {
    if (zdt_check_size(*offset, 2, max_len) != ZDT_OK) {
        return ZDT_ERR_BUF_TOO_SMALL;
    }
    buf[(*offset)++] = (uint8_t)((val >> 8) & 0xFF);
    buf[(*offset)++] = (uint8_t)(val & 0xFF);
    return ZDT_OK;
}

static inline int zdt_append_u32_be(uint8_t *buf, size_t *offset, size_t max_len, uint32_t val) {
    if (zdt_check_size(*offset, 4, max_len) != ZDT_OK) {
        return ZDT_ERR_BUF_TOO_SMALL;
    }
    buf[(*offset)++] = (uint8_t)((val >> 24) & 0xFF);
    buf[(*offset)++] = (uint8_t)((val >> 16) & 0xFF);
    buf[(*offset)++] = (uint8_t)((val >> 8) & 0xFF);
    buf[(*offset)++] = (uint8_t)(val & 0xFF);
    return ZDT_OK;
}

/**
 * 将原始串口帧打包为 CAN 多包扩展帧。
 * 规则（按手册 4.2.1 及 split_host_test）：
 * - raw_frame[0] 为地址，不进入 payload，编码进 CAN ID = (addr << 8) | packet (Packet 从 0 开始计数)
 * - payload 从 raw_frame[1] 开始切片，每包最多 8 字节
 * - 成功返回分包总数 (即 msg->frame_count)，失败返回负数错误码
 */
static inline int zdt_pack_can_msg(uint8_t addr, const uint8_t *raw_frame, size_t raw_len, zdt_can_msg_t *msg) {
    if (!raw_frame || !msg || raw_len <= 1) {
        return ZDT_ERR_PARAM;
    }

    size_t payload_len = raw_len - 1;
    const uint8_t *payload = raw_frame + 1;
    uint8_t pkt = 0;
    size_t offset = 0;

    while (offset < payload_len) {
        if (pkt >= ZDT_CAN_MAX_FRAMES) {
            return ZDT_ERR_BUF_TOO_SMALL;
        }

        size_t chunk = payload_len - offset;
        if (chunk > 8) {
            chunk = 8;
        }

        msg->frames[pkt].id = ((uint32_t)addr << 8) | (uint32_t)pkt;
        msg->frames[pkt].dlc = (uint8_t)chunk;
        for (size_t i = 0; i < chunk; ++i) {
            msg->frames[pkt].data[i] = payload[offset + i];
        }

        pkt++;
        offset += chunk;
    }

    msg->frame_count = pkt;
    return (int)pkt;
}

#ifdef __cplusplus
}
#endif

#endif /* ZDT_INTERNAL_H */
