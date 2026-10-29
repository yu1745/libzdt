/* SPDX-License-Identifier: GPL-3.0-only */
/* libzdt 串口 TTL 真机测试：ESP32-C3 经 UART1 直连两台 ZDT 电机（地址 1、2，Emm 固件，6B 校验）。
 *
 * 只发真机上验证过无副作用的命令：读取、store=00 写回同值、±5° 相对当前位置的小幅运动
 * （10 RPM，测完回到起点）。不发回零、校准、恢复出厂、改 ID / 波特率 / 驱动配置。
 *
 * 每个用例打印 "HW PASS <name>" 或 "HW FAIL <name>: <原因>"，最后打印 "HW SUMMARY pass=N fail=M"。 */
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "libzdt.h"

#define TX_IO       GPIO_NUM_1
#define RX_IO       GPIO_NUM_0
#define TIMEOUT_MS  200
#define PULSES_5DEG 711         /* 细分 256：51200 脉冲 / 圈 */

static zdt_uart_handle_t s_bus;
static int s_pass, s_fail;

static void report(const char *name, bool ok, const char *fmt, ...)
{
    if (ok) {
        printf("HW PASS %s\n", name);
        s_pass++;
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    printf("HW FAIL %s: ", name);
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
    s_fail++;
}

static esp_err_t xfer(const uint8_t *f, int n, zdt_reply_t *r)
{
    if (n < 0) return ESP_ERR_INVALID_ARG;
    return zdt_uart_transact(s_bus, f, (size_t)n, r, TIMEOUT_MS);
}

static void expect_ok(const char *name, const uint8_t *f, int n)
{
    zdt_reply_t r;
    esp_err_t err = xfer(f, n, &r);
    report(name, err == ESP_OK && r.code == f[1] && zdt_reply_is_ok(&r),
           "%s code=%02X len=%u d0=%02X", esp_err_to_name(err), r.code, r.len, r.data[0]);
}

/* 读实时位置，单位 0.01°；失败返回 INT32_MIN */
static int32_t pos_cdeg(uint8_t addr)
{
    uint8_t f[8];
    zdt_reply_t r;
    int n = zdtBuildReadRealtimePosCmd(addr, f, sizeof f);
    if (xfer(f, n, &r) != ESP_OK || r.len != 5) return INT32_MIN;
    uint32_t raw = ((uint32_t)r.data[1] << 24) | ((uint32_t)r.data[2] << 16) | ((uint32_t)r.data[3] << 8) | r.data[4];
    int64_t cdeg = (int64_t)raw * 36000 / 65536;
    return (int32_t)(r.data[0] ? -cdeg : cdeg);
}

/* ---------------------------------------------------------------- 读取 */

static void test_reads(uint8_t a)
{
    struct { const char *name; int (*build)(uint8_t, uint8_t *, size_t); uint8_t len; } reads[] = {
        {"version",       zdtBuildReadVersionCmd,           4},
        {"phase_rl",      zdtBuildReadPhaseRLCmd,           4},
        {"bus_voltage",   zdtBuildReadBusVoltageCmd,        2},
        {"bus_current",   zdtBuildReadBusCurrentCmd,        2},
        {"phase_current", zdtBuildReadPhaseCurrentCmd,      2},
        {"encoder",       zdtBuildReadEncoderCalibratedCmd, 2},
        {"input_pulses",  zdtBuildReadInputPulsesCmd,       5},
        {"target_pos",    zdtBuildReadTargetPosCmd,         5},
        {"rt_target_pos", zdtBuildReadRealtimeTargetPosCmd, 5},
        {"rt_speed",      zdtBuildReadRealtimeSpeedCmd,     3},
        {"temp",          zdtBuildReadDriverTempCmd,        2},
        {"rt_pos",        zdtBuildReadRealtimePosCmd,       5},
        {"pos_error",     zdtBuildReadPosErrorCmd,          5},
        {"status",        zdtBuildReadMotorStatusCmd,       1},
        {"homing_status", zdtBuildReadHomingStatusCmd,      1},
        {"home+status",   zdtBuildReadHomingAndStatusCmd,   2},
        {"io_level",      zdtBuildReadIoLevelCmd,           1},
        {"options",       zdtBuildReadOptionsCmd,           2},
        {"homing_params", zdtBuildReadHomingParamsCmd,      15},
        {"pid_emm",       zdtBuildReadPidEmmCmd,            12},
        {"dmx512",        zdtBuildReadDmx512Cmd,            14},
        {"pos_window",    zdtBuildReadPosWindowCmd,         2},
        {"protect",       zdtBuildReadProtectThresholdCmd,  6},
        {"heartbeat",     zdtBuildReadHeartbeatCmd,         4},
        {"integral",      zdtBuildReadIntegralLimitCmd,     4},
        {"bump_angle",    zdtBuildReadBumpReturnAngleCmd,   2},
        {"all_status",    zdtBuildReadAllStatusEmmCmd,      28},
        {"all_config",    zdtBuildReadAllConfigEmmCmd,      30},
    };
    for (size_t i = 0; i < sizeof reads / sizeof reads[0]; ++i) {
        uint8_t f[8];
        zdt_reply_t r;
        char name[48];
        snprintf(name, sizeof name, "read %s id%u", reads[i].name, a);
        int n = reads[i].build(a, f, sizeof f);
        esp_err_t err = xfer(f, n, &r);
        report(name, err == ESP_OK && r.addr == a && r.code == f[1] && r.len == reads[i].len,
               "%s addr=%u code=%02X len=%u", esp_err_to_name(err), r.addr, r.code, r.len);
    }
}

/* ------------------------------------------------- 写入（store=00，写回同值） */

static void test_writes(uint8_t a)
{
    uint8_t f[24];
    zdt_reply_t r;
    char name[48];

    /* 5.6.21：读出当前窗口，单字节写回同值 */
    int n = zdtBuildReadPosWindowCmd(a, f, sizeof f);
    esp_err_t err = xfer(f, n, &r);
    uint8_t win = r.len >= 1 ? r.data[r.len - 1] : 0;      /* 应答 00 03，写过后变成 03 6B（固件怪现象） */
    if (r.len == 2 && r.data[1] == 0x6B) win = r.data[0];
    snprintf(name, sizeof name, "pos_window same id%u", a);
    if (err != ESP_OK || win < 2) {
        report(name, false, "read %s win=%u", esp_err_to_name(err), win);
    } else {
        n = zdtBuildWritePosWindowCmd(a, ZDT_STORE_NO, win, f, sizeof f);
        expect_ok(name, f, n);
    }

    /* 回零参数：读出后原样写回（不触发回零） */
    n = zdtBuildReadHomingParamsCmd(a, f, sizeof f);
    err = xfer(f, n, &r);
    snprintf(name, sizeof name, "homing_params same id%u", a);
    if (err != ESP_OK || r.len != 15) {
        report(name, false, "read %s len=%u", esp_err_to_name(err), r.len);
    } else {
        const uint8_t *d = r.data;
        n = zdtBuildWriteHomingParamsCmd(a, ZDT_STORE_NO, d[0], d[1], (uint16_t)(d[2] << 8 | d[3]),
                                         (uint32_t)d[4] << 24 | (uint32_t)d[5] << 16 | (uint32_t)d[6] << 8 | d[7],
                                         (uint16_t)(d[8] << 8 | d[9]), (uint16_t)(d[10] << 8 | d[11]),
                                         (uint16_t)(d[12] << 8 | d[13]), d[14], f, sizeof f);
        expect_ok(name, f, n);
    }

    /* PID：读出后原样写回 */
    n = zdtBuildReadPidEmmCmd(a, f, sizeof f);
    err = xfer(f, n, &r);
    snprintf(name, sizeof name, "pid same id%u", a);
    if (err != ESP_OK || r.len != 12) {
        report(name, false, "read %s len=%u", esp_err_to_name(err), r.len);
    } else {
        uint32_t k[3];
        for (int i = 0; i < 3; ++i)
            k[i] = (uint32_t)r.data[i * 4] << 24 | (uint32_t)r.data[i * 4 + 1] << 16 |
                   (uint32_t)r.data[i * 4 + 2] << 8 | r.data[i * 4 + 3];
        n = zdtBuildWritePidEmmCmd(a, ZDT_STORE_NO, k[0], k[1], k[2], f, sizeof f);
        expect_ok(name, f, n);
    }

    /* 命令格式错误应答（00 EE）：此固件不接受 5.6.7，状态不变 */
    n = zdtBuildChangeCtrlModeCmd(a, ZDT_STORE_NO, 1, f, sizeof f);
    err = xfer(f, n, &r);
    snprintf(name, sizeof name, "format_err 00EE id%u", a);
    report(name, err == ESP_OK && r.code == 0x00 && r.len == 1 && r.data[0] == ZDT_REPLY_FORMAT_ERR,
           "%s code=%02X d0=%02X", esp_err_to_name(err), r.code, r.data[0]);
}

/* ---------------------------------------------------------- 定时返回 + 事件回调 */

static int s_events;
static uint8_t s_event_addr, s_event_code;

static void on_event(const zdt_reply_t *f, void *ctx)
{
    (void)ctx;
    s_events++;
    s_event_addr = f->addr;
    s_event_code = f->code;
}

static void test_periodic(uint8_t a)
{
    uint8_t f[8];
    zdt_reply_t r;
    char name[48];
    zdt_uart_set_event_cb(s_bus, on_event, NULL);
    s_events = 0;

    int n = zdtBuildSetPeriodicReportCmd(a, 0x36, 50, f, sizeof f);
    snprintf(name, sizeof name, "periodic start id%u", a);
    expect_ok(name, f, n);
    int polled = zdt_uart_poll(s_bus, 400);
    snprintf(name, sizeof name, "periodic frames via cb id%u", a);
    report(name, polled >= 5 && s_events == polled && s_event_addr == a && s_event_code == 0x36,
           "polled=%d events=%d addr=%u code=%02X", polled, s_events, s_event_addr, s_event_code);

    /* 定时返回进行中照常收发：读状态的应答不会被位置帧顶替 */
    n = zdtBuildReadMotorStatusCmd(a, f, sizeof f);
    esp_err_t err = xfer(f, n, &r);
    snprintf(name, sizeof name, "transact during periodic id%u", a);
    report(name, err == ESP_OK && r.code == 0x3A && r.len == 1, "%s code=%02X", esp_err_to_name(err), r.code);

    n = zdtBuildSetPeriodicReportCmd(a, 0x36, 0, f, sizeof f);
    snprintf(name, sizeof name, "periodic stop id%u", a);
    expect_ok(name, f, n);
    vTaskDelay(pdMS_TO_TICKS(120));
    s_events = 0;
    zdt_uart_poll(s_bus, 150);
    snprintf(name, sizeof name, "periodic stopped id%u", a);
    report(name, s_events == 0, "events=%d", s_events);
    zdt_uart_set_event_cb(s_bus, NULL, NULL);
}

/* ---------------------------------------------------------- 小幅运动 */

static bool near(int32_t got, int32_t want) { return got - want < 30 && want - got < 30; }   /* ±0.3° */

static void move_rel(uint8_t a, uint8_t dir, uint32_t pulses, uint8_t sync, const char *name)
{
    uint8_t f[16];
    int n = zdtBuildPosModeEmmCmd(a, dir, 10, 10, pulses, ZDT_MOVE_REL_NOW, sync, f, sizeof f);
    expect_ok(name, f, n);
}

static void test_motion(uint8_t a)
{
    char name[48];
    int32_t p0 = pos_cdeg(a);
    snprintf(name, sizeof name, "pos read id%u", a);
    report(name, p0 != INT32_MIN, "no reply");
    if (p0 == INT32_MIN) return;

    snprintf(name, sizeof name, "pos +5 id%u", a);
    move_rel(a, ZDT_DIR_CW, PULSES_5DEG, 0, name);
    vTaskDelay(pdMS_TO_TICKS(1200));
    int32_t p1 = pos_cdeg(a);
    snprintf(name, sizeof name, "pos +5 reached id%u", a);
    report(name, near(p1 - p0, 500), "delta=%" PRId32, p1 - p0);

    snprintf(name, sizeof name, "pos -5 id%u", a);
    move_rel(a, ZDT_DIR_CCW, PULSES_5DEG, 0, name);
    vTaskDelay(pdMS_TO_TICKS(1200));
    int32_t p2 = pos_cdeg(a);
    snprintf(name, sizeof name, "pos back id%u", a);
    report(name, near(p2, p0), "delta=%" PRId32, p2 - p0);
}

static void test_broadcast_sync(void)
{
    uint8_t f[48], sub[32];
    zdt_reply_t r;
    int32_t p0[3] = {0, pos_cdeg(1), pos_cdeg(2)};
    if (p0[1] == INT32_MIN || p0[2] == INT32_MIN) {
        report("bcast sync", false, "pos read failed");
        return;
    }
    move_rel(1, ZDT_DIR_CW, PULSES_5DEG, 1, "cache +5 id1");
    move_rel(2, ZDT_DIR_CW, PULSES_5DEG, 1, "cache +5 id2");
    vTaskDelay(pdMS_TO_TICKS(300));
    report("cached not moving", near(pos_cdeg(1), p0[1]) && near(pos_cdeg(2), p0[2]), "moved early");

    /* 广播触发：只有地址 1 应答 */
    int n = zdtBuildSyncMotionCmd(ZDT_ADDR_BROADCAST, f, sizeof f);
    esp_err_t err = xfer(f, n, &r);
    report("bcast sync reply from id1", err == ESP_OK && r.addr == 1 && r.code == 0xFF && zdt_reply_is_ok(&r),
           "%s addr=%u", esp_err_to_name(err), r.addr);
    vTaskDelay(pdMS_TO_TICKS(1300));
    int32_t d1 = pos_cdeg(1) - p0[1], d2 = pos_cdeg(2) - p0[2];
    report("bcast sync both moved", near(d1, 500) && near(d2, 500), "d1=%" PRId32 " d2=%" PRId32, d1, d2);

    /* 多电机命令同时回位（无应答，只发送） */
    int a = zdtBuildPosModeEmmCmd(1, ZDT_DIR_CCW, 10, 10, PULSES_5DEG, ZDT_MOVE_REL_NOW, 0, sub, sizeof sub);
    int b = zdtBuildPosModeEmmCmd(2, ZDT_DIR_CCW, 10, 10, PULSES_5DEG, ZDT_MOVE_REL_NOW, 0, sub + a, sizeof sub - (size_t)a);
    n = zdtBuildMultiMotorCmd(sub, (size_t)(a + b), f, sizeof f);
    err = n > 0 ? zdt_uart_send(s_bus, f, (size_t)n, TIMEOUT_MS) : ESP_ERR_INVALID_ARG;
    report("multi-motor send", err == ESP_OK, "%s", esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(1300));
    zdt_uart_flush(s_bus);
    d1 = pos_cdeg(1) - p0[1];
    d2 = pos_cdeg(2) - p0[2];
    report("multi-motor both back", near(d1, 0) && near(d2, 0), "d1=%" PRId32 " d2=%" PRId32, d1, d2);
}

/* ---------------------------------------------------------- 交替读压测 */

static void test_stress(void)
{
    int fail = 0;
    for (int i = 0; i < 300; ++i) {
        for (uint8_t a = 1; a <= 2; ++a) {
            uint8_t f[4];
            zdt_reply_t r;
            int n = zdtBuildReadRealtimePosCmd(a, f, sizeof f);
            if (xfer(f, n, &r) != ESP_OK || r.addr != a || r.code != 0x36 || r.len != 5) fail++;
        }
    }
    report("stress 600 reads", fail == 0, "fail=%d", fail);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1500));       /* 给主机打开串口留时间 */
    const zdt_uart_config_t cfg = {
        .port = UART_NUM_1,
        .tx_io = TX_IO,
        .rx_io = RX_IO,
        .baud_rate = 115200,
        .checksum = ZDT_CHECKSUM_FIXED_6B,
        .firmware = ZDT_FW_EMM,
    };
    ESP_ERROR_CHECK(zdt_uart_new(&cfg, &s_bus));
    printf("HW BEGIN\n");

    for (uint8_t a = 1; a <= 2; ++a) {
        test_reads(a);
        test_writes(a);
        test_periodic(a);
        test_motion(a);
    }
    test_broadcast_sync();
    test_stress();

    printf("HW SUMMARY pass=%d fail=%d\n", s_pass, s_fail);
    zdt_uart_del(s_bus);
}
