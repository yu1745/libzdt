/* test_zdt.c - libzdt 完整单元测试 (82 个命令全覆盖)
 *
 * 每个测试用例基于《ZDT_X42S 第二代闭环步进电机用户手册 V1.0.3》中
 * "主机发送（主机→电机）"命令表的字段顺序与字节定义编写。
 *
 * 通过条件: 实际生成的字节序列与手册示例/规则完全一致。
 */
#include "../src/libzdt.h"
#include <stdio.h>
#include <string.h>

static int g_pass = 0, g_fail = 0;

static int check(const char *name, const uint8_t *got, int got_len,
                 const uint8_t *expected, int exp_len)
{
    int ok = (got_len == exp_len) && (memcmp(got, expected, exp_len) == 0);
    if (ok) {
        g_pass++;
        printf("  PASS  %-32s [%dB]\n", name, got_len);
    } else {
        g_fail++;
        printf("  FAIL  %-32s\n", name);
        printf("        got (%dB):", got_len);
        for (int i = 0; i < got_len; ++i) printf(" %02X", got[i]);
        printf("\n        exp (%dB):", exp_len);
        for (int i = 0; i < exp_len; ++i) printf(" %02X", expected[i]);
        printf("\n");
    }
    return ok;
}

int main(void)
{
    uint8_t buf[64];
    int n;

    /* =================================================================
     *  5.2 触发动作命令 (5 个)
     * ================================================================= */

    /* 5.2.1 触发编码器校准 — 手册: 发送 01 06 45 6B */
    n = zdtBuildEncoderCalibrationCmd(0x01, buf, sizeof buf);
    check("5.2.1 编码器校准", buf, n,
          (const uint8_t[]){0x01, 0x06, 0x45, 0x6B}, 4);

    /* 5.2.2 重启电机 — 手册: 发送 01 08 97 6B */
    n = zdtBuildRestartMotorCmd(0x01, buf, sizeof buf);
    check("5.2.2 重启电机", buf, n,
          (const uint8_t[]){0x01, 0x08, 0x97, 0x6B}, 4);

    /* 5.2.3 当前位置角度清零 — 手册: 发送 01 0A 6D 6B */
    n = zdtBuildClearCurrentAngleCmd(0x01, buf, sizeof buf);
    check("5.2.3 角度清零", buf, n,
          (const uint8_t[]){0x01, 0x0A, 0x6D, 0x6B}, 4);

    /* 5.2.4 解除堵转/过热/过流保护 — 手册: 发送 01 0E 52 6B */
    n = zdtBuildClearProtectionCmd(0x01, buf, sizeof buf);
    check("5.2.4 解除保护", buf, n,
          (const uint8_t[]){0x01, 0x0E, 0x52, 0x6B}, 4);

    /* 5.2.5 恢复出厂设置 — 手册: 发送 01 0F 5F 6B */
    n = zdtBuildFactoryResetCmd(0x01, buf, sizeof buf);
    check("5.2.5 恢复出厂", buf, n,
          (const uint8_t[]){0x01, 0x0F, 0x5F, 0x6B}, 4);

    /* =================================================================
     *  5.3 运动控制命令 (14 个)
     * ================================================================= */

    /* 5.3.1 多电机命令 — 手册(PDF L2711): 00 AA 00 22 02 FD ... 6B
     * 0022 = 总字节数 34 (= 5 + 子命令长度 29) */
    {
        const uint8_t sub[] = {0x02, 0xFD, 0x01, 0x05, 0xDC, 0x08, 0x00, 0x00,
                               0x7D, 0x00, 0x00, 0x00, 0x6B,
                               0x03, 0xFD, 0x00, 0x03, 0xE8, 0x0A, 0x00, 0x00,
                               0xFA, 0x00, 0x01, 0x01, 0x6B,
                               0x04, 0x36, 0x6B};
        const uint8_t exp[] = {0x00, 0xAA, 0x00, 0x22,
                               0x02, 0xFD, 0x01, 0x05, 0xDC, 0x08, 0x00, 0x00,
                               0x7D, 0x00, 0x00, 0x00, 0x6B,
                               0x03, 0xFD, 0x00, 0x03, 0xE8, 0x0A, 0x00, 0x00,
                               0xFA, 0x00, 0x01, 0x01, 0x6B,
                               0x04, 0x36, 0x6B, 0x6B};
        n = zdtBuildMultiMotorCmd(sub, sizeof sub, buf, sizeof buf);
        check("5.3.1 多电机命令", buf, n, exp, sizeof exp);
    }

    /* 5.3.2 电机使能控制 — 手册: 01 F3 AB 01 00 6B (使能) */
    n = zdtBuildMotorEnableCmd(0x01, 0x01, ZDT_SYNC_NOW, buf, sizeof buf);
    check("5.3.2 电机使能", buf, n,
          (const uint8_t[]){0x01, 0xF3, 0xAB, 0x01, 0x00, 0x6B}, 6);

    /* 5.3.3 力矩模式控制 (X) — 手册表格: Addr+F5+符号+斜率(BE16)+电流(BE16)+同步+6B
     * 这里斜率=200(0x00C8), 电流=1500(0x05DC), 同步=00 */
    n = zdtBuildTorqueModeCmd(0x01, ZDT_DIR_CCW, 200, 1500, ZDT_SYNC_NOW,
                              buf, sizeof buf);
    check("5.3.3 力矩模式(X) 200mA/S 1500mA", buf, n,
          (const uint8_t[]){0x01, 0xF5, 0x01, 0x00, 0xC8, 0x05, 0xDC, 0x00, 0x6B}, 9);

    /* 5.3.4 力矩模式限速控制 (X) — 手册示例 01 C5 01 00 C8 02 58 00 0F A0 6B
     * 斜率200mA/S, 电流600mA, 同步00, 最大速度400.0RPM (0x0FA0) */
    n = zdtBuildTorqueModeSpeedLimitCmd(0x01, ZDT_DIR_CCW, 200, 600, 4000,
                                        ZDT_SYNC_NOW, buf, sizeof buf);
    check("5.3.4 力矩模式限速(X) 手册示例", buf, n,
          (const uint8_t[]){0x01, 0xC5, 0x01, 0x00, 0xC8, 0x02, 0x58, 0x00, 0x0F, 0xA0, 0x6B}, 11);

    /* 5.3.5 速度模式控制 (X) — 表格: Addr+F6+方向+加速度(BE16)+速度(BE16)+同步+6B (9B)
     * 加速度=1000RPM/S(0x03E8), 速度=1000.0RPM(0x2710), 同步=00 */
    n = zdtBuildSpeedModeXCmd(0x01, ZDT_DIR_CW, 1000, 10000, ZDT_SYNC_NOW,
                              buf, sizeof buf);
    check("5.3.5 速度模式(X) 1000RPM/S 1000RPM", buf, n,
          (const uint8_t[]){0x01, 0xF6, 0x00, 0x03, 0xE8, 0x27, 0x10, 0x00, 0x6B}, 9);

    /* 5.3.6 速度模式限电流 (X) — 手册示例 01 C6 01 03 E8 4E 20 00 07 D0 6B
     * 方向CCW, 加速度1000RPM/S(0x03E8), 速度2000.0RPM(0x4E20), 同步00, 最大电流2000mA(0x07D0) */
    n = zdtBuildSpeedModeXCurrentLimitCmd(0x01, ZDT_DIR_CCW, 1000, 20000,
                                          2000, ZDT_SYNC_NOW, buf, sizeof buf);
    check("5.3.6 速度模式限流(X) 手册示例", buf, n,
          (const uint8_t[]){0x01, 0xC6, 0x01, 0x03, 0xE8, 0x4E, 0x20, 0x00, 0x07, 0xD0, 0x6B}, 11);

    /* 5.3.7 速度模式控制 (Emm) — 手册示例: 01 F6 01 05 DC 0A 00 6B (1500RPM 10档) */
    n = zdtBuildSpeedModeEmmCmd(0x01, ZDT_DIR_CCW, 1500, 10, ZDT_SYNC_NOW,
                                buf, sizeof buf);
    check("5.3.7 速度模式(Emm) 1500RPM", buf, n,
          (const uint8_t[]){0x01, 0xF6, 0x01, 0x05, 0xDC, 0x0A, 0x00, 0x6B}, 8);

    /* 5.3.8 直通限速位置模式 (X) — 表格: Addr+FB+方向+速度(BE16)+位置角度(BE32)+运动模式+同步+6B (12B)
     * 这里: 速度1000.0RPM(0x2710), 位置+320.0°(0x00000C80), 运动模式=相对上一目标(00) */
    n = zdtBuildPosModePassThroughCmd(0x01, ZDT_DIR_CW, 10000, 3200,
                                      ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW,
                                      buf, sizeof buf);
    check("5.3.8 直通位置(X)", buf, n,
          (const uint8_t[]){0x01, 0xFB, 0x00, 0x27, 0x10, 0x00, 0x00, 0x0C, 0x80, 0x00, 0x00, 0x6B}, 12);

    /* 5.3.9 直通限速位置模式限电流 (X) — 手册示例 01 CB 01 4E 20 00 00 8C A0 00 00 07 D0 6B
     * 方向CCW, 速度2000.0RPM(0x4E20), 位置+3600.0°(0x00008CA0),
     * 运动模式=相对上一目标(00), 同步00, 最大电流2000mA(0x07D0) */
    n = zdtBuildPosModePassThroughCurrentLimitCmd(0x01, ZDT_DIR_CCW, 20000, 36000,
                                                  ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW,
                                                  2000, buf, sizeof buf);
    check("5.3.9 直通位置限流(X) 手册示例", buf, n,
          (const uint8_t[]){0x01, 0xCB, 0x01, 0x4E, 0x20, 0x00, 0x00, 0x8C, 0xA0, 0x00, 0x00, 0x07, 0xD0, 0x6B}, 14);

    /* 5.3.10 梯形曲线位置 (X) — 表格: Addr+FD+方向+加速加速度(BE16)+减速加速度(BE16)
     * +最大速度(BE16)+位置角度(BE32)+运动模式+同步+6B (16B)
     * 这里: 加速500RPM/S(0x01F4), 减速500RPM/S, 最大500.0RPM(0x1388),
     *       位置+640.0°(0x0000_1900), 运动模式=相对上一目标(00) */
    n = zdtBuildTrapezoidPosModeCmd(0x01, ZDT_DIR_CW,
                                    500, 500, 5000, 6400,
                                    ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW,
                                    buf, sizeof buf);
    check("5.3.10 梯形位置(X)", buf, n,
          (const uint8_t[]){0x01, 0xFD, 0x00, 0x01, 0xF4, 0x01, 0xF4, 0x13, 0x88,
                             0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0x6B}, 16);

    /* 5.3.11 梯形曲线位置限电流 (X) — 手册示例: 01 CD 01 01 FF 01 FA 27 10 00 00 8C A0 00 00 07 D0 6B
     * 表格: Addr+CD+方向+加速加速度+减速加速度+最大速度+位置角度+运动模式+同步+最大电流+6B (18B) */
    n = zdtBuildTrapezoidPosModeCurrentLimitCmd(0x01, ZDT_DIR_CCW,
                                                511, 506, 10000, 36000,
                                                0x00, ZDT_SYNC_NOW, 2000,
                                                buf, sizeof buf);
    check("5.3.11 梯形位置限流(X)", buf, n,
          (const uint8_t[]){0x01, 0xCD, 0x01, 0x01, 0xFF, 0x01, 0xFA, 0x27, 0x10,
                             0x00, 0x00, 0x8C, 0xA0, 0x00, 0x00, 0x07, 0xD0, 0x6B}, 18);

    /* 5.3.12 位置模式控制 (Emm) — 手册示例: 01 FD 01 05 DC 00 00 00 7D 00 00 00 6B
     * CCW 1500RPM 32000脉冲(10圈) */
    n = zdtBuildPosModeEmmCmd(0x01, ZDT_DIR_CCW, 1500, 0, 32000,
                              ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW,
                              buf, sizeof buf);
    check("5.3.12 位置模式(Emm)", buf, n,
          (const uint8_t[]){0x01, 0xFD, 0x01, 0x05, 0xDC, 0x00, 0x00,
                             0x00, 0x7D, 0x00, 0x00, 0x00, 0x6B}, 13);

    /* 5.3.13 立即停止 — 手册: 发送 01 FE 98 00 6B */
    n = zdtBuildImmediateStopCmd(0x01, ZDT_SYNC_NOW, buf, sizeof buf);
    check("5.3.13 立即停止", buf, n,
          (const uint8_t[]){0x01, 0xFE, 0x98, 0x00, 0x6B}, 5);

    /* 5.3.14 触发多机同步运动 — 手册: 发送 00 FF 66 6B */
    n = zdtBuildSyncMotionCmd(0x00, buf, sizeof buf);
    check("5.3.14 多机同步", buf, n,
          (const uint8_t[]){0x00, 0xFF, 0x66, 0x6B}, 4);

    /* =================================================================
     *  5.4 原点回零命令 (6 个)
     * ================================================================= */

    /* 5.4.1 设置单圈回零的零点位置 — 表格: Addr+93+88+是否存储+6B
     * 手册示例: 01 93 88 01 6B (存储) */
    n = zdtBuildSetSingleTurnZeroCmd(0x01, ZDT_STORE_YES, buf, sizeof buf);
    check("5.4.1 单圈回零点", buf, n,
          (const uint8_t[]){0x01, 0x93, 0x88, 0x01, 0x6B}, 5);

    /* 5.4.2 触发回零 — 手册示例: 01 9A 02 00 6B (模式 02=无限位碰撞) */
    n = zdtBuildTriggerHomingCmd(0x01, 0x02, ZDT_SYNC_NOW, buf, sizeof buf);
    check("5.4.2 触发回零 mode=02", buf, n,
          (const uint8_t[]){0x01, 0x9A, 0x02, 0x00, 0x6B}, 5);

    /* 5.4.3 强制中断并退出回零 — 表格: Addr+9C+48+6B */
    n = zdtBuildAbortHomingCmd(0x01, buf, sizeof buf);
    check("5.4.3 中断回零", buf, n,
          (const uint8_t[]){0x01, 0x9C, 0x48, 0x6B}, 4);

    /* 5.4.4 读取回零状态标志 — 功能码 3B (手册 5.4.4 表格) */
    n = zdtBuildReadHomingStatusCmd(0x01, buf, sizeof buf);
    check("5.4.4 读取回零状态", buf, n,
          (const uint8_t[]){0x01, 0x3B, 0x6B}, 3);

    /* 5.4.5 读取回零参数 — 表格: Addr+22+6B */
    n = zdtBuildReadHomingParamsCmd(0x01, buf, sizeof buf);
    check("5.4.5 读取回零参数", buf, n,
          (const uint8_t[]){0x01, 0x22, 0x6B}, 3);

    /* 5.4.6 修改回零参数 — 手册示例: 01 4C AE 01 00 00 00 1E 00 00 27 10 01 2C 03 20 00 3C 00 6B */
    n = zdtBuildWriteHomingParamsCmd(0x01, ZDT_STORE_YES, 0x00, ZDT_DIR_CW,
                                     30, 10000, 300, 800, 60, 0x00,
                                     buf, sizeof buf);
    check("5.4.6 修改回零参数(手册)", buf, n,
          (const uint8_t[]){0x01, 0x4C, 0xAE, 0x01, 0x00, 0x00, 0x00, 0x1E,
                             0x00, 0x00, 0x27, 0x10, 0x01, 0x2C, 0x03, 0x20,
                             0x00, 0x3C, 0x00, 0x6B}, 20);

    /* =================================================================
     *  5.5 读取系统参数命令 (18 个) — 全为短帧
     * ================================================================= */

    /* 5.5.1 定时返回信息 — 表格: Addr+11+18+信息功能码+时间(BE16)+6B (7B)
     * 例: 让电机 1 每 100ms 返回一次实时转速(0x36) */
    n = zdtBuildSetPeriodicReportCmd(0x01, 0x36, 100, buf, sizeof buf);
    check("5.5.1 定时返回(实时转速)", buf, n,
          (const uint8_t[]){0x01, 0x11, 0x18, 0x36, 0x00, 0x64, 0x6B}, 7);

    n = zdtBuildReadVersionCmd(0x01, buf, sizeof buf);
    check("5.5.2 读取版本", buf, n,
          (const uint8_t[]){0x01, 0x1F, 0x6B}, 3);

    n = zdtBuildReadPhaseRLCmd(0x01, buf, sizeof buf);
    check("5.5.3 读取相电阻电感", buf, n,
          (const uint8_t[]){0x01, 0x20, 0x6B}, 3);

    n = zdtBuildReadBusVoltageCmd(0x01, buf, sizeof buf);
    check("5.5.4 读取总线电压", buf, n,
          (const uint8_t[]){0x01, 0x24, 0x6B}, 3);

    n = zdtBuildReadBusCurrentCmd(0x01, buf, sizeof buf);
    check("5.5.5 读取总线电流", buf, n,
          (const uint8_t[]){0x01, 0x26, 0x6B}, 3);

    n = zdtBuildReadPhaseCurrentCmd(0x01, buf, sizeof buf);
    check("5.5.6 读取相电流", buf, n,
          (const uint8_t[]){0x01, 0x27, 0x6B}, 3);

    n = zdtBuildReadEncoderCalibratedCmd(0x01, buf, sizeof buf);
    check("5.5.7 读取编码器校准值", buf, n,
          (const uint8_t[]){0x01, 0x31, 0x6B}, 3);

    n = zdtBuildReadInputPulsesCmd(0x01, buf, sizeof buf);
    check("5.5.8 读取输入脉冲数", buf, n,
          (const uint8_t[]){0x01, 0x32, 0x6B}, 3);

    n = zdtBuildReadTargetPosCmd(0x01, buf, sizeof buf);
    check("5.5.9 读取电机目标位置", buf, n,
          (const uint8_t[]){0x01, 0x33, 0x6B}, 3);

    n = zdtBuildReadRealtimeTargetPosCmd(0x01, buf, sizeof buf);
    check("5.5.10 读取实时目标位置", buf, n,
          (const uint8_t[]){0x01, 0x34, 0x6B}, 3);

    n = zdtBuildReadRealtimeSpeedCmd(0x01, buf, sizeof buf);
    check("5.5.11 读取实时转速", buf, n,
          (const uint8_t[]){0x01, 0x35, 0x6B}, 3);

    n = zdtBuildReadDriverTempCmd(0x01, buf, sizeof buf);
    check("5.5.12 读取驱动温度", buf, n,
          (const uint8_t[]){0x01, 0x39, 0x6B}, 3);

    n = zdtBuildReadRealtimePosCmd(0x01, buf, sizeof buf);
    check("5.5.13 读取实时位置", buf, n,
          (const uint8_t[]){0x01, 0x36, 0x6B}, 3);

    n = zdtBuildReadPosErrorCmd(0x01, buf, sizeof buf);
    check("5.5.14 读取位置误差", buf, n,
          (const uint8_t[]){0x01, 0x37, 0x6B}, 3);

    n = zdtBuildReadMotorStatusCmd(0x01, buf, sizeof buf);
    check("5.5.15 读取电机状态", buf, n,
          (const uint8_t[]){0x01, 0x3A, 0x6B}, 3);

    n = zdtBuildReadHomingAndStatusCmd(0x01, buf, sizeof buf);
    check("5.5.16 读取回零+电机状态", buf, n,
          (const uint8_t[]){0x01, 0x3C, 0x6B}, 3);

    n = zdtBuildReadIoLevelCmd(0x01, buf, sizeof buf);
    check("5.5.17 读取IO电平", buf, n,
          (const uint8_t[]){0x01, 0x3D, 0x6B}, 3);

    n = zdtBuildReadBatteryVoltageCmd(0x01, buf, sizeof buf);
    check("5.5.18 读取电池电压(Y42)", buf, n,
          (const uint8_t[]){0x01, 0x38, 0x6B}, 3);

    /* =================================================================
     *  5.6 读写驱动参数命令 (31 个)
     * ================================================================= */

    /* 5.6.1 修改电机 ID/地址 — 表格: Addr+AE+4B+store+id+6B */
    n = zdtBuildChangeAddrCmd(0x01, ZDT_STORE_YES, 0x02, buf, sizeof buf);
    check("5.6.1 修改电机地址→2", buf, n,
          (const uint8_t[]){0x01, 0xAE, 0x4B, 0x01, 0x02, 0x6B}, 6);

    /* 5.6.2 修改细分值 — 表格: Addr+84+8A+store+microstep+6B */
    n = zdtBuildChangeMicrostepCmd(0x01, ZDT_STORE_YES, 16, buf, sizeof buf);
    check("5.6.2 修改细分值→16", buf, n,
          (const uint8_t[]){0x01, 0x84, 0x8A, 0x01, 0x10, 0x6B}, 6);

    /* 5.6.3 修改掉电标志 — 表格: Addr+50+flag+6B */
    n = zdtBuildChangePowerDownFlagCmd(0x01, 0x00, buf, sizeof buf);
    check("5.6.3 修改掉电标志=0", buf, n,
          (const uint8_t[]){0x01, 0x50, 0x00, 0x6B}, 4);

    /* 5.6.4 读取选项参数状态 — 表格: Addr+1A+6B */
    n = zdtBuildReadOptionsCmd(0x01, buf, sizeof buf);
    check("5.6.4 读取选项参数", buf, n,
          (const uint8_t[]){0x01, 0x1A, 0x6B}, 3);

    /* 5.6.5 修改电机类型 — 表格: Addr+D7+35+store+type+6B */
    n = zdtBuildChangeMotorTypeCmd(0x01, ZDT_STORE_YES, 0x32, buf, sizeof buf);
    check("5.6.5 修改电机类型=1.8°", buf, n,
          (const uint8_t[]){0x01, 0xD7, 0x35, 0x01, 0x32, 0x6B}, 6);

    /* 5.6.6 修改固件类型 — 表格: Addr+D5+69+store+type+6B */
    n = zdtBuildChangeFirmwareTypeCmd(0x01, ZDT_STORE_YES, 0x01, buf, sizeof buf);
    check("5.6.6 修改固件=Emm", buf, n,
          (const uint8_t[]){0x01, 0xD5, 0x69, 0x01, 0x01, 0x6B}, 6);

    /* 5.6.7 修改开环/闭环模式 — 表格: Addr+46+A6+store+mode+6B */
    n = zdtBuildChangeCtrlModeCmd(0x01, ZDT_STORE_YES, 0x01, buf, sizeof buf);
    check("5.6.7 修改为闭环", buf, n,
          (const uint8_t[]){0x01, 0x46, 0xA6, 0x01, 0x01, 0x6B}, 6);

    /* 5.6.8 修改电机运动正方向 — 表格: Addr+D4+60+store+dir+6B */
    n = zdtBuildChangeMotorDirCmd(0x01, ZDT_STORE_YES, 0x00, buf, sizeof buf);
    check("5.6.8 方向=CW", buf, n,
          (const uint8_t[]){0x01, 0xD4, 0x60, 0x01, 0x00, 0x6B}, 6);

    /* 5.6.9 修改锁定按键功能 — 表格: Addr+D0+B3+store+lock+6B */
    n = zdtBuildChangeKeyLockCmd(0x01, ZDT_STORE_YES, 0x01, buf, sizeof buf);
    check("5.6.9 锁定按键", buf, n,
          (const uint8_t[]){0x01, 0xD0, 0xB3, 0x01, 0x01, 0x6B}, 6);

    /* 5.6.10 修改位置角度缩小10倍 (X) — 表格: Addr+4F+71+store+scale+6B */
    n = zdtBuildChangePosScaleCmd(0x01, ZDT_STORE_YES, 0x01, buf, sizeof buf);
    check("5.6.10 角度缩小10倍", buf, n,
          (const uint8_t[]){0x01, 0x4F, 0x71, 0x01, 0x01, 0x6B}, 6);

    /* 5.6.11 修改速度缩小10倍 (Emm) — 与 5.6.10 同帧结构 */
    n = zdtBuildChangeSpeedScaleCmd(0x01, ZDT_STORE_YES, 0x01, buf, sizeof buf);
    check("5.6.11 速度缩小10倍", buf, n,
          (const uint8_t[]){0x01, 0x4F, 0x71, 0x01, 0x01, 0x6B}, 6);

    /* 5.6.12 修改开环工作电流 — 表格: Addr+44+33+store+current(BE16)+6B */
    n = zdtBuildChangeOpenLoopCurrentCmd(0x01, ZDT_STORE_YES, 1500, buf, sizeof buf);
    check("5.6.12 开环电流1500mA", buf, n,
          (const uint8_t[]){0x01, 0x44, 0x33, 0x01, 0x05, 0xDC, 0x6B}, 7);

    /* 5.6.13 修改闭环最大电流 — 表格: Addr+45+66+store+current(BE16)+6B */
    n = zdtBuildChangeClosedLoopCurrentCmd(0x01, ZDT_STORE_YES, 3000, buf, sizeof buf);
    check("5.6.13 闭环最大3000mA", buf, n,
          (const uint8_t[]){0x01, 0x45, 0x66, 0x01, 0x0B, 0xB8, 0x6B}, 7);

    /* 5.6.14 读取 PID 参数 (X) — 表格: Addr+21+6B */
    n = zdtBuildReadPidXCmd(0x01, buf, sizeof buf);
    check("5.6.14 读PID(X)", buf, n,
          (const uint8_t[]){0x01, 0x21, 0x6B}, 3);

    /* 5.6.15 修改 PID 参数 (X) — 手册示例: 01 4A C3 01 00 00 01 EE B0 00 01 EE B0 00 00 3C F0 00 00 00 1A 6B */
    n = zdtBuildWritePidXCmd(0x01, ZDT_STORE_YES,
                             0x0001EEB0, 0x0001EEB0,
                             0x00003CF0, 0x0000001A,
                             buf, sizeof buf);
    check("5.6.15 改PID(X) 手册示例", buf, n,
          (const uint8_t[]){0x01, 0x4A, 0xC3, 0x01,
                             0x00, 0x01, 0xEE, 0xB0,
                             0x00, 0x01, 0xEE, 0xB0,
                             0x00, 0x00, 0x3C, 0xF0,
                             0x00, 0x00, 0x00, 0x1A, 0x6B}, 21);

    /* 5.6.16 读取 PID 参数 (Emm) — 表格: Addr+21+6B */
    n = zdtBuildReadPidEmmCmd(0x01, buf, sizeof buf);
    check("5.6.16 读PID(Emm)", buf, n,
          (const uint8_t[]){0x01, 0x21, 0x6B}, 3);

    /* 5.6.17 修改 PID 参数 (Emm) — 手册示例: 01 4A C3 01 00 00 46 50 00 00 00 0A 00 00 46 50 6B */
    n = zdtBuildWritePidEmmCmd(0x01, ZDT_STORE_YES,
                               18000, 10, 18000,
                               buf, sizeof buf);
    check("5.6.17 改PID(Emm) 手册示例", buf, n,
          (const uint8_t[]){0x01, 0x4A, 0xC3, 0x01,
                             0x00, 0x00, 0x46, 0x50,
                             0x00, 0x00, 0x00, 0x0A,
                             0x00, 0x00, 0x46, 0x50, 0x6B}, 17);

    /* 5.6.18 读取 DMX512 参数 — 表格: Addr+49+78+6B */
    n = zdtBuildReadDmx512Cmd(0x01, buf, sizeof buf);
    check("5.6.18 读DMX512", buf, n,
          (const uint8_t[]){0x01, 0x49, 0x78, 0x6B}, 4);

    /* 5.6.19 修改 DMX512 参数 — 手册示例: 01 D9 90 01 00 C0 01 01 03 E8 03 E8 00 0A 00 00 00 64 6B */
    n = zdtBuildWriteDmx512Cmd(0x01, ZDT_STORE_YES,
                               192, 1, 1, 1000, 1000, 10, 100,
                               buf, sizeof buf);
    check("5.6.19 改DMX512 手册示例", buf, n,
          (const uint8_t[]){0x01, 0xD9, 0x90, 0x01,
                             0x00, 0xC0, 0x01, 0x01, 0x03, 0xE8, 0x03, 0xE8,
                             0x00, 0x0A, 0x00, 0x00, 0x00, 0x64, 0x6B}, 19);

    /* 5.6.20 读取位置到达窗口 — 表格: Addr+41+6B */
    n = zdtBuildReadPosWindowCmd(0x01, buf, sizeof buf);
    check("5.6.20 读位置窗口", buf, n,
          (const uint8_t[]){0x01, 0x41, 0x6B}, 3);

    /* 5.6.21 修改位置到达窗口 — 真机验证: 01 D1 07 01 08 6B (0.8°)
     * Value 是单字节(02~30)，不是 BE16。手册原例 00 08 有误，真机返回 EE。 */
    n = zdtBuildWritePosWindowCmd(0x01, ZDT_STORE_YES, 8, buf, sizeof buf);
    check("5.6.21 改位置窗口=0.8°", buf, n,
          (const uint8_t[]){0x01, 0xD1, 0x07, 0x01, 0x08, 0x6B}, 6);

    /* 5.6.22 读取过热过流阈值 — 表格: Addr+13+6B */
    n = zdtBuildReadProtectThresholdCmd(0x01, buf, sizeof buf);
    check("5.6.22 读保护阈值", buf, n,
          (const uint8_t[]){0x01, 0x13, 0x6B}, 3);

    /* 5.6.23 修改过热过流阈值 — 手册示例: 01 D3 56 01 00 64 19 C8 03 E8 6B
     * (100℃/6600mA/1000ms) */
    n = zdtBuildWriteProtectThresholdCmd(0x01, ZDT_STORE_YES,
                                          100, 6600, 1000,
                                          buf, sizeof buf);
    check("5.6.23 改保护阈值 手册示例", buf, n,
          (const uint8_t[]){0x01, 0xD3, 0x56, 0x01,
                             0x00, 0x64, 0x19, 0xC8, 0x03, 0xE8, 0x6B}, 11);

    /* 5.6.24 读取心跳时间 — 表格: Addr+16+6B */
    n = zdtBuildReadHeartbeatCmd(0x01, buf, sizeof buf);
    check("5.6.24 读心跳", buf, n,
          (const uint8_t[]){0x01, 0x16, 0x6B}, 3);

    /* 5.6.25 修改心跳时间 — 表格: Addr+68+38+store+heartbeat(BE32)+6B (9B) */
    n = zdtBuildWriteHeartbeatCmd(0x01, ZDT_STORE_YES, 5000, buf, sizeof buf);
    check("5.6.25 改心跳=5000ms", buf, n,
          (const uint8_t[]){0x01, 0x68, 0x38, 0x01, 0x00, 0x00, 0x13, 0x88, 0x6B}, 9);

    /* 5.6.26 读取积分限幅/刚性系数 — 表格: Addr+23+6B */
    n = zdtBuildReadIntegralLimitCmd(0x01, buf, sizeof buf);
    check("5.6.26 读积分限幅", buf, n,
          (const uint8_t[]){0x01, 0x23, 0x6B}, 3);

    /* 5.6.27 修改积分限幅/刚性系数 — 表格: Addr+4B+57+store+value(BE32)+6B (9B) */
    n = zdtBuildWriteIntegralLimitCmd(0x01, ZDT_STORE_YES, 388, buf, sizeof buf);
    check("5.6.27 改积分限幅=388", buf, n,
          (const uint8_t[]){0x01, 0x4B, 0x57, 0x01, 0x00, 0x00, 0x01, 0x84, 0x6B}, 9);

    /* 5.6.28 读取碰撞回零返回角度 — 表格: Addr+3F+6B */
    n = zdtBuildReadBumpReturnAngleCmd(0x01, buf, sizeof buf);
    check("5.6.28 读碰撞角度", buf, n,
          (const uint8_t[]){0x01, 0x3F, 0x6B}, 3);

    /* 5.6.29 修改碰撞回零返回角度 — 表格: Addr+5C+AC+store+angle(BE16)+6B (6B) */
    n = zdtBuildWriteBumpReturnAngleCmd(0x01, ZDT_STORE_YES, 20, buf, sizeof buf);
    check("5.6.29 改碰撞角度=2.0°", buf, n,
          (const uint8_t[]){0x01, 0x5C, 0xAC, 0x01, 0x00, 0x14, 0x6B}, 7);

    /* 5.6.30 广播读取 ID 地址 — 表格: 00+15+6B */
    n = zdtBuildBroadcastReadAddrCmd(buf, sizeof buf);
    check("5.6.30 广播读ID", buf, n,
          (const uint8_t[]){0x00, 0x15, 0x6B}, 3);

    /* 5.6.31 修改锁定修改参数功能 — 表格: Addr+D6+4B+store+lock(0-3)+6B */
    n = zdtBuildChangeParamLockCmd(0x01, ZDT_STORE_YES, 0x02, buf, sizeof buf);
    check("5.6.31 锁定参数等级=2", buf, n,
          (const uint8_t[]){0x01, 0xD6, 0x4B, 0x01, 0x02, 0x6B}, 6);

    /* =================================================================
     *  5.7 上电自动运行命令 (2 个)
     * ================================================================= */

    /* 5.7.1 (X) — 手册示例: 01 F7 1C 01 00 01 FF 17 70 01 6B
     * store=01 dir=00 acc=0x01FF=511RPM/s speed=0x1770=600.0RPM en=01 */
    n = zdtBuildStoreAutoRunXCmd(0x01, 0x01, 0x00, 511, 6000, 0x01,
                                 buf, sizeof buf);
    check("5.7.1 (X) 手册示例", buf, n,
          (const uint8_t[]){0x01, 0xF7, 0x1C, 0x01, 0x00,
                             0x01, 0xFF, 0x17, 0x70, 0x01, 0x6B}, 11);

    /* 5.7.2 (Emm) — 手册示例: 01 F7 1C 01 00 02 58 64 01 6B
     * store=01 dir=00 speed=0x0258=600RPM acc=0x64=100档 en=01 */
    n = zdtBuildStoreAutoRunEmmCmd(0x01, 0x01, 0x00, 600, 100, 0x01,
                                   buf, sizeof buf);
    check("5.7.2 (Emm) 手册示例", buf, n,
          (const uint8_t[]){0x01, 0xF7, 0x1C, 0x01, 0x00,
                             0x02, 0x58, 0x64, 0x01, 0x6B}, 10);

    /* =================================================================
     *  5.8 读取所有驱动参数命令 (6 个)
     * ================================================================= */

    /* 5.8.1 读取系统状态 (X) — 表格: Addr+43+7A+6B */
    n = zdtBuildReadAllStatusXCmd(0x01, buf, sizeof buf);
    check("5.8.1 读系统状态(X)", buf, n,
          (const uint8_t[]){0x01, 0x43, 0x7A, 0x6B}, 4);

    /* 5.8.2 读取系统状态 (Emm) — 同帧 */
    n = zdtBuildReadAllStatusEmmCmd(0x01, buf, sizeof buf);
    check("5.8.2 读系统状态(Emm)", buf, n,
          (const uint8_t[]){0x01, 0x43, 0x7A, 0x6B}, 4);

    /* 5.8.3 读取驱动配置 (X) — 表格: Addr+42+6C+6B */
    n = zdtBuildReadAllConfigXCmd(0x01, buf, sizeof buf);
    check("5.8.3 读驱动配置(X)", buf, n,
          (const uint8_t[]){0x01, 0x42, 0x6C, 0x6B}, 4);

    /* 5.8.4 修改驱动配置 (X) — 手册示例(PDF L4784): 37B
     * 01 48 D1 01 00 01 01 02 02 00 10 01 00 00 04 B0 0B 80 0B B8 03 E8 05 07
     * 00 01 00 01 00 08 08 98 07 D0 00 08 6B */
    {
        const uint8_t exp[] = {0x01, 0x48, 0xD1, 0x01, 0x00, 0x01, 0x01, 0x02,
                               0x02, 0x00, 0x10, 0x01, 0x00, 0x00,
                               0x04, 0xB0, 0x0B, 0x80, 0x0B, 0xB8, 0x03, 0xE8,
                               0x05, 0x07,
                               0x00, 0x01, 0x00, 0x01,
                               0x00, 0x08, 0x08, 0x98, 0x07, 0xD0, 0x00, 0x08,
                               0x6B};
        n = zdtBuildWriteAllConfigXCmd(0x01, ZDT_STORE_YES,
                                       0x00, 0x01, 0x01, 0x02, 0x02, 0x00,
                                       0x10, 0x01,
                                       0x04B0, 0x0B80, 0x0BB8, 0x03E8,
                                       0x05, 0x07, 0x00, 0x01, 0x00,
                                       0x01,
                                       0x0008, 0x0898, 0x07D0, 0x0008,
                                       buf, sizeof buf);
        check("5.8.4 改驱动配置(X)", buf, n, exp, sizeof exp);
    }

    /* 5.8.5 读取驱动配置 (Emm) — 表格: Addr+42+6C+6B */
    n = zdtBuildReadAllConfigEmmCmd(0x01, buf, sizeof buf);
    check("5.8.5 读驱动配置(Emm)", buf, n,
          (const uint8_t[]){0x01, 0x42, 0x6C, 0x6B}, 4);

    /* 5.8.6 修改驱动配置 (Emm) — 手册示例(PDF L5008): 33B
     * 01 48 D1 01 19 02 02 02 00 10 01 00 04 B0 0C 80 0F A0 05 07 00 00 01 01
     * 00 08 0B B8 07 D0 00 01 6B */
    {
        const uint8_t exp[] = {0x01, 0x48, 0xD1, 0x01, 0x19, 0x02, 0x02, 0x02,
                               0x00, 0x10, 0x01, 0x00,
                               0x04, 0xB0, 0x0C, 0x80, 0x0F, 0xA0,
                               0x05, 0x07, 0x00, 0x00, 0x01,
                               0x01,
                               0x00, 0x08, 0x0B, 0xB8, 0x07, 0xD0, 0x00, 0x01,
                               0x6B};
        n = zdtBuildWriteAllConfigEmmCmd(0x01, ZDT_STORE_YES,
                                         0x19, 0x02, 0x02, 0x02, 0x00,
                                         0x10, 0x01,
                                         0x04B0, 0x0C80, 0x0FA0,
                                         0x05, 0x07, 0x00, 0x01,
                                         0x01,
                                         0x0008, 0x0BB8, 0x07D0, 0x0001,
                                         buf, sizeof buf);
        check("5.8.6 改驱动配置(Emm)", buf, n, exp, sizeof exp);
    }

    printf("\n=========================================\n");
    printf("TOTAL: %d PASS / %d FAIL\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}