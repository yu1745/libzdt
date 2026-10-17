/* SPDX-License-Identifier: GPL-3.0-only */
/*
 * libzdt.h - ZDT_X42S 第二代闭环步进电机 ESP-IDF 组件 (CAN 扩展帧)
 *
 * zdtCanBuild*Cmd 构建函数只做字节编码、不访问硬件，零动态内存分配，
 * 得到符合 ZDT CAN 扩展帧协议的报文 (zdt_can_msg_t)；
 * 发送与应答接收见 zdt_twai.h。
 * CAN 分帧说明：各模块文档中的“原始命令”包含 Addr；CAN 将 Addr 编入
 * EID=(Addr<<8)|Packet（Packet 从 0 开始）。CAN data 从 Code 开始、不含
 * Addr，且每个 data payload 最多 8 字节。
 */

#ifndef LIBZDT_H
#define LIBZDT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "zdt_types.h"
#include "zdt_common_motion.h"
#include "zdt_common_sys.h"
#include "zdt_common_param.h"
#include "zdt_x.h"
#include "zdt_emm.h"
#include "zdt_twai.h"

#ifdef __cplusplus
}
#endif

#endif /* LIBZDT_H */
