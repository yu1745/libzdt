/* demo.c - libzdt 原生 CAN 接口使用示例 */
#include "libzdt.h"
#include <stdio.h>

static void dump_can_msg(const char *tag, const zdt_can_msg_t *msg)
{
    printf("%s: %d CAN frame(s)\n", tag, msg->frame_count);
    for (int i = 0; i < msg->frame_count; ++i) {
        const zdt_can_frame_t *f = &msg->frames[i];
        printf("  [frame %d] ID=0x%04X, DLC=%d, Data:", i, (unsigned int)f->id, f->dlc);
        for (int j = 0; j < f->dlc; ++j) {
            printf(" %02X", f->data[j]);
        }
        printf("\n");
    }
}

int main(void)
{
    zdt_can_msg_t msg;

    /* 5.2.1 触发编码器校准 */
    zdtCanBuildEncoderCalibrationCmd(0x01, &msg);
    dump_can_msg("5.2.1 编码器校准", &msg);

    /* 5.3.3 力矩模式 (X) — CCW, 斜率 1000mA/S, 电流 1500mA */
    zdtCanBuildTorqueModeCmd(0x01, ZDT_DIR_CCW, 1000, 1500, ZDT_SYNC_NOW, &msg);
    dump_can_msg("5.3.3 力矩模式(X)", &msg);

    /* 5.3.13 立即停止 */
    zdtCanBuildImmediateStopCmd(0x01, ZDT_SYNC_NOW, &msg);
    dump_can_msg("5.3.13 立即停止", &msg);

    /* 5.4.6 修改回零参数 (多包长命令) */
    zdtCanBuildWriteHomingParamsCmd(0x01, ZDT_STORE_YES, 0x00,
                                    ZDT_DIR_CW, 30, 10000,
                                    300, 800, 60, 0x00, &msg);
    dump_can_msg("5.4.6 修改回零参数", &msg);

    /* 5.6.1 修改电机 ID — 存储并把地址改为 0x02 */
    zdtCanBuildChangeAddrCmd(0x01, ZDT_STORE_YES, 0x02, &msg);
    dump_can_msg("5.6.1 修改电机 ID", &msg);

    return 0;
}
