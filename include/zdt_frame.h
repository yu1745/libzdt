/* SPDX-License-Identifier: GPL-3.0-only */
/*
 * zdt_frame.h - 串口 TTL 帧的校验与应答分帧（纯计算，不访问硬件）
 *
 * 串口帧就是手册里的"原始命令"：Addr + Code + 数据 + 校验字节，由 zdtBuild*Cmd 构建
 * （校验字节固定写 0x6B）。电机设为 XOR / CRC8 校验时，用 zdt_frame_set_checksum 改写末字节。
 *
 * 电机应答没有长度字段，接收方按功能码查长度（zdt_ttl_reply_len）切分字节流：
 *   - 功能码 00：命令格式错误，Addr + 00 + EE + 校验，4 字节
 *   - 42 / 43（读驱动配置 / 系统状态）：第 3 字节为整帧长度
 *   - 读取类命令：按表（真机 X42S Emm 实测，X 固件按手册）
 *   - 其余命令：Addr + Code + 状态(02/12/9F/E2/EE) + 校验，4 字节
 */

#ifndef ZDT_FRAME_H
#define ZDT_FRAME_H

#include <stddef.h>
#include <stdint.h>
#include "zdt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ZDT_ERR_FRAME           -3      /* 字节流开头不是有效应答帧 */

#define ZDT_TTL_MAX_FRAME       (ZDT_REPLY_MAX_LEN + 3)   /* 应答整帧上限：Addr + Code + data + 校验 */

/** 计算 data[0..len) 的校验字节（6B 模式恒为 0x6B） */
uint8_t zdt_checksum_calc(zdt_checksum_t mode, const uint8_t *data, size_t len);

/**
 * @brief 按校验方式改写帧的末字节
 * @param frame 完整帧（含末尾校验字节），len >= 2
 * @return ZDT_OK；参数错误 ZDT_ERR_PARAM
 */
int zdt_frame_set_checksum(uint8_t *frame, size_t len, zdt_checksum_t mode);

/**
 * @brief 某功能码的应答整帧长度（含地址与校验字节）
 * @return 字节数；0 表示长度写在帧内（42 / 43 的第 3 字节）
 */
size_t zdt_ttl_reply_len(uint8_t code, zdt_firmware_t fw);

/**
 * @brief 从字节流开头解析一帧应答
 *
 * @param out 解析出的应答（data 为 Code 之后、校验字节之前的内容）
 * @return >0：成功，值为本帧消耗的字节数；
 *         0：数据还不够一帧，继续接收；
 *         ZDT_ERR_FRAME：开头不是有效帧（地址为 0、长度越界或校验不符），调用方丢弃 1 字节后重试
 */
int zdt_ttl_parse(const uint8_t *buf, size_t len, zdt_checksum_t mode, zdt_firmware_t fw,
                  zdt_reply_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_FRAME_H */
