/*
 * libzdt.h - ZDT_X42S 第二代闭环步进电机通信命令构建库 (CAN 扩展帧)
 *
 * 纯 C99，无任何硬件平台或操作系统依赖，零动态内存分配。
 * 本库负责构建符合 ZDT CAN 扩展帧协议的发送报文 (zdt_can_msg_t)。
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

#ifdef __cplusplus
}
#endif

#endif /* LIBZDT_H */
