/* SPDX-License-Identifier: GPL-3.0-only */
/*
 * libzdt.h - ZDT_X42S 第二代闭环步进电机 ESP-IDF 组件 (串口 TTL / CAN 扩展帧)
 *
 * 每条命令有两个构建函数，都只做字节编码、不访问硬件，零动态内存分配：
 *   - zdtBuild*Cmd    得到串口 TTL 原始帧（Addr + Code + 数据 + 0x6B），
 *                     校验与应答分帧见 zdt_frame.h，收发见 zdt_uart.h；
 *   - zdtCanBuild*Cmd 得到 CAN 扩展帧报文 (zdt_can_msg_t)，收发见 zdt_twai.h。
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
#include "zdt_frame.h"
#include "zdt_uart.h"
#include "zdt_twai.h"

#ifdef __cplusplus
}
#endif

#endif /* LIBZDT_H */
