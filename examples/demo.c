/* demo.c - libzdt 使用示例 */
#include "../src/libzdt.h"
#include <stdio.h>

static void dump(const char *tag, const uint8_t *buf, int len)
{
    printf("%s [%dB]:", tag, len);
    for (int i = 0; i < len; ++i) printf(" %02X", buf[i]);
    printf("\n");
}

int main(void)
{
    uint8_t buf[64];

    /* 5.2.1 触发编码器校准 */
    int n = zdtBuildEncoderCalibrationCmd(0x01, buf, sizeof buf);
    dump("5.2.1", buf, n);  /* 期望: 01 06 45 6B */

    /* 5.3.3 力矩模式 (X) — CCW 1500mA */
    n = zdtBuildTorqueModeCmd(0x01, ZDT_DIR_CCW, 1500, ZDT_SYNC_NOW,
                              buf, sizeof buf);
    dump("5.3.3", buf, n);

    /* 5.3.13 立即停止 */
    n = zdtBuildImmediateStopCmd(0x01, ZDT_SYNC_NOW, buf, sizeof buf);
    dump("5.3.13", buf, n);

    /* 5.4.6 修改回零参数 */
    n = zdtBuildWriteHomingParamsCmd(0x01, ZDT_STORE_YES, 0x00,
                                     ZDT_DIR_CW, 30, 10000,
                                     300, 800, 60, 0x00,
                                     buf, sizeof buf);
    dump("5.4.6", buf, n);

    /* 5.6.1 修改电机 ID */
    n = zdtBuildChangeAddrCmd(0x01, 0x02, ZDT_SYNC_NOW, buf, sizeof buf);
    dump("5.6.1", buf, n);

    return 0;
}