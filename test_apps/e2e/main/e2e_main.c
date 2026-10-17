/* libzdt 端到端测试：固件在 QEMU 里通过 TWAI 与宿主机上的假电机（tools/fake_motor.py）通信。
 *
 * 每个用例打印 "E2E PASS <name>" 或 "E2E FAIL <name>: <原因>"，最后打印
 * "E2E SUMMARY pass=N fail=M"，由 tools/run_e2e.sh 解析。 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "libzdt.h"
#include "zdt_twai.h"

#define MOTOR_ADDR  0x01
#define TIMEOUT_MS  500

static zdt_twai_handle_t s_bus;
static int s_pass;
static int s_fail;

static void report(const char *name, bool ok, const char *fmt, ...)
{
    if (ok) {
        printf("E2E PASS %s\n", name);
        s_pass++;
    } else {
        va_list ap;
        va_start(ap, fmt);
        printf("E2E FAIL %s: ", name);
        vprintf(fmt, ap);
        printf("\n");
        va_end(ap);
        s_fail++;
    }
}

/* 发送构建出的命令并检查是 02 状态应答 */
static void expect_ok(const char *name, int built, const zdt_can_msg_t *msg)
{
    zdt_reply_t r;
    if (built < 1) {
        report(name, false, "build failed (%d)", built);
        return;
    }
    esp_err_t err = zdt_twai_transact(s_bus, msg, 2, &r, TIMEOUT_MS);
    if (err != ESP_OK) {
        report(name, false, "transact %s", esp_err_to_name(err));
    } else if (!zdt_reply_is_ok(&r)) {
        report(name, false, "len=%u data0=%02X", r.len, r.len ? r.data[0] : 0);
    } else {
        report(name, true, "");
    }
}

/* 手工构造一帧（绕过构建函数的参数检查），期望电机回指定的状态字节 */
static void expect_status(const char *name, const zdt_can_msg_t *msg, uint8_t status)
{
    zdt_reply_t r;
    esp_err_t err = zdt_twai_transact(s_bus, msg, 2, &r, TIMEOUT_MS);
    if (err != ESP_OK) {
        report(name, false, "transact %s", esp_err_to_name(err));
    } else if (r.len != 1 || r.data[0] != status) {
        report(name, false, "len=%u data0=%02X want %02X", r.len, r.len ? r.data[0] : 0, status);
    } else {
        report(name, true, "");
    }
}

static void expect_position(const char *name, uint8_t sign, uint32_t counts)
{
    zdt_can_msg_t msg;
    zdt_reply_t r;
    int n = zdtCanBuildReadRealtimePosCmd(MOTOR_ADDR, &msg);
    esp_err_t err = n < 1 ? ESP_FAIL : zdt_twai_transact(s_bus, &msg, 6, &r, TIMEOUT_MS);
    if (err != ESP_OK || r.code != 0x36 || r.len != 5) {
        report(name, false, "err=%s len=%u", esp_err_to_name(err), r.len);
        return;
    }
    uint32_t got = ((uint32_t)r.data[1] << 24) | ((uint32_t)r.data[2] << 16) | ((uint32_t)r.data[3] << 8) | r.data[4];
    report(name, r.data[0] == sign && got == counts, "sign=%u counts=%u want sign=%u counts=%u",
           r.data[0], (unsigned)got, sign, (unsigned)counts);
}

static void run_tests(void)
{
    zdt_can_msg_t msg;
    zdt_reply_t r;
    int n;

    /* 单帧状态命令 */
    n = zdtCanBuildMotorEnableCmd(MOTOR_ADDR, 1, ZDT_SYNC_NOW, &msg);
    expect_ok("enable", n, &msg);
    n = zdtCanBuildClearCurrentAngleCmd(MOTOR_ADDR, &msg);
    expect_ok("clear_angle", n, &msg);
    n = zdtCanBuildSpeedModeEmmCmd(MOTOR_ADDR, ZDT_DIR_CCW, 1500, 10, ZDT_SYNC_NOW, &msg);
    expect_ok("speed_emm", n, &msg);
    n = zdtCanBuildImmediateStopCmd(MOTOR_ADDR, ZDT_SYNC_NOW, &msg);
    expect_ok("immediate_stop", n, &msg);

    /* 读命令：单帧应答 */
    n = zdtCanBuildReadBusVoltageCmd(MOTOR_ADDR, &msg);
    esp_err_t err = n < 1 ? ESP_FAIL : zdt_twai_transact(s_bus, &msg, 3, &r, TIMEOUT_MS);
    report("read_bus_voltage", err == ESP_OK && r.len == 2 && r.data[0] == 0x5D && r.data[1] == 0xC0,
           "err=%s len=%u", esp_err_to_name(err), r.len);

    /* 位置命令：命令本身是 2 个包（手册 p.43 示例同型） */
    n = zdtCanBuildPosModeEmmCmd(MOTOR_ADDR, ZDT_DIR_CW, 1000, 0, 3200, ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW, &msg);
    report("pos_emm_two_packets", n == 2 && msg.frames[1].data[0] == 0xFD && msg.frames[1].id == 0x0101,
           "n=%d", n);
    expect_ok("pos_emm_cw_3200", n, &msg);
    expect_position("pos_after_cw_3200", 0, 65536);

    n = zdtCanBuildPosModeEmmCmd(MOTOR_ADDR, ZDT_DIR_CCW, 1000, 0, 1600, ZDT_MOVE_ABS_ZERO, ZDT_SYNC_NOW, &msg);
    expect_ok("pos_emm_abs_ccw_1600", n, &msg);
    expect_position("pos_after_abs_ccw", 1, 32768);

    /* 长命令：3 个包 / 5 个包 */
    n = zdtCanBuildWriteHomingParamsCmd(MOTOR_ADDR, ZDT_STORE_YES, 0x00, ZDT_DIR_CW, 30, 10000,
                                        300, 800, 60, 0x00, &msg);
    report("homing_params_three_packets", n == 3, "n=%d", n);
    expect_ok("homing_params", n, &msg);

    /* 错误应答：参数越界回 E2，长度不足回 EE */
    zdt_can_msg_t bad = { .frame_count = 1 };
    bad.frames[0].id = (MOTOR_ADDR << 8);
    bad.frames[0].dlc = 7;
    memcpy(bad.frames[0].data, (uint8_t[]){ 0xF6, 0x00, 0x0B, 0xB9, 0x0A, 0x00, 0x6B }, 7);  /* 3001 RPM */
    expect_status("speed_out_of_range_E2", &bad, ZDT_REPLY_PARAM_ERR);
    bad.frames[0].dlc = 5;                      /* F6 00 0B B8 6B：缺加速度和同步标志 */
    memcpy(bad.frames[0].data, (uint8_t[]){ 0xF6, 0x00, 0x0B, 0xB8, 0x6B }, 5);
    expect_status("speed_truncated_EE", &bad, ZDT_REPLY_FORMAT_ERR);

    /* 无人应答的地址：超时 */
    n = zdtCanBuildMotorEnableCmd(0x05, 1, ZDT_SYNC_NOW, &msg);
    err = zdt_twai_transact(s_bus, &msg, 2, &r, 200);
    report("no_motor_timeout", err == ESP_ERR_TIMEOUT, "err=%s", esp_err_to_name(err));

    /* 切换应答方式为 Both（5 个包），随后位置命令应先 02 再 9F */
    n = zdtCanBuildWriteAllConfigEmmCmd(MOTOR_ADDR, ZDT_STORE_NO, 0x19, 0x02, 0x03, 0x02, 0x00, 16, 1,
                                        1200, 3000, 4000, 5, 7, 0, 3, 1, 28, 1000, 4000, 8, &msg);
    report("config_five_packets", n == 5, "n=%d", n);
    expect_ok("config_response_both", n, &msg);

    n = zdtCanBuildPosModeEmmCmd(MOTOR_ADDR, ZDT_DIR_CW, 1000, 0, 3200, ZDT_MOVE_REL_NOW, ZDT_SYNC_NOW, &msg);
    expect_ok("pos_emm_ack_both", n, &msg);
    err = zdt_twai_recv_reply(s_bus, MOTOR_ADDR, 2, &r, 1000);
    report("pos_emm_reached_9F", err == ESP_OK && r.code == 0xFD && r.len == 1 && r.data[0] == ZDT_REPLY_DONE,
           "err=%s code=%02X", esp_err_to_name(err), r.code);
}

void app_main(void)
{
    zdt_twai_config_t cfg = {
        .tx_io = GPIO_NUM_4,
        .rx_io = GPIO_NUM_5,
        .bitrate = 500000,
    };
    esp_err_t err = zdt_twai_new(&cfg, &s_bus);
    if (err != ESP_OK) {
        printf("E2E FATAL zdt_twai_new: %s\n", esp_err_to_name(err));
        printf("E2E SUMMARY pass=0 fail=1\n");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
    run_tests();
    printf("E2E SUMMARY pass=%d fail=%d\n", s_pass, s_fail);

    zdt_twai_del(s_bus);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
