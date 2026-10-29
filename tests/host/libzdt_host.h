/* SPDX-License-Identifier: GPL-3.0-only */
/* 主机测试只用纯计算部分（构建函数、分帧），不含依赖 ESP-IDF 的 zdt_uart / zdt_twai */
#pragma once
#include <stdbool.h>
#include "zdt_types.h"
#include "zdt_common_motion.h"
#include "zdt_common_sys.h"
#include "zdt_common_param.h"
#include "zdt_x.h"
#include "zdt_emm.h"
#include "zdt_frame.h"
