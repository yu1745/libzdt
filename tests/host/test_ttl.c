/* SPDX-License-Identifier: GPL-3.0-only */
/* 串口 TTL 部分的主机单元测试：构建函数、校验、应答分帧。金样本来自真机（hw_cases.inc）。 */
#include <stdio.h>
#include <string.h>

#include "libzdt_host.h"

#include "hw_cases.inc"

static int s_fail, s_pass;

#define CHECK(cond, ...) do { \
    if (cond) { s_pass++; } else { s_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

static void hex(const char *tag, const uint8_t *p, size_t n)
{
    printf("  %s:", tag);
    for (size_t i = 0; i < n; ++i) printf(" %02X", p[i]);
    printf("\n");
}

/* 构建结果必须与真机接受的帧逐字节一致 */
static void expect_frame(const char *name, int n, const uint8_t *buf, const char *want_hex)
{
    uint8_t want[64];
    size_t wn = 0;
    for (const char *p = want_hex; *p;) {
        unsigned v;
        int k;
        if (sscanf(p, "%2x%n", &v, &k) != 1) break;
        want[wn++] = (uint8_t)v;
        p += k;
        while (*p == ' ') p++;
    }
    bool ok = n == (int)wn && memcmp(buf, want, wn) == 0;
    CHECK(ok, "%s", name);
    if (!ok) {
        hex("got ", buf, n > 0 ? (size_t)n : 0);
        hex("want", want, wn);
    }
}

static void test_builders_vs_hw(void)
{
    uint8_t b[64];
    int n;
#define T(name, call, want) do { n = call; expect_frame(name, n, b, want); } while (0)
    T("5.2.4",  zdtBuildClearProtectionCmd(1, b, sizeof b), "01 0E 52 6B");
    T("5.3.13", zdtBuildImmediateStopCmd(1, 0, b, sizeof b), "01 FE 98 00 6B");
    T("5.3.13 bcast", zdtBuildImmediateStopCmd(0, 0, b, sizeof b), "00 FE 98 00 6B");
    T("5.4.3",  zdtBuildAbortHomingCmd(1, b, sizeof b), "01 9C 48 6B");
    T("5.3.2",  zdtBuildMotorEnableCmd(1, 1, 0, b, sizeof b), "01 F3 AB 01 00 6B");
    T("5.6.2",  zdtBuildChangeMicrostepCmd(1, 0, 0, b, sizeof b), "01 84 8A 00 00 6B");
    T("5.6.8",  zdtBuildChangeMotorDirCmd(1, 0, 0, b, sizeof b), "01 D4 60 00 00 6B");
    T("5.6.9",  zdtBuildChangeKeyLockCmd(1, 0, 0, b, sizeof b), "01 D0 B3 00 00 6B");
    T("5.6.11", zdtBuildChangeSpeedScaleCmd(1, 0, 0, b, sizeof b), "01 4F 71 00 00 6B");
    T("5.6.12", zdtBuildChangeOpenLoopCurrentCmd(1, 0, 1000, b, sizeof b), "01 44 33 00 03 E8 6B");
    T("5.6.13", zdtBuildChangeClosedLoopCurrentCmd(1, 0, 2400, b, sizeof b), "01 45 66 00 09 60 6B");
    T("5.6.17", zdtBuildWritePidEmmCmd(1, 0, 0x6590, 0x14, 0x0FA000, b, sizeof b),
      "01 4A C3 00 00 00 65 90 00 00 00 14 00 0F A0 00 6B");
    T("5.6.19", zdtBuildWriteDmx512Cmd(1, 0, 192, 1, 1, 1000, 1000, 10, 100, b, sizeof b),
      "01 D9 90 00 00 C0 01 01 03 E8 03 E8 00 0A 00 00 00 64 6B");
    T("5.6.21", zdtBuildWritePosWindowCmd(1, 0, 3, b, sizeof b), "01 D1 07 00 00 03 6B");
    T("5.6.23", zdtBuildWriteProtectThresholdCmd(1, 0, 100, 6600, 1000, b, sizeof b),
      "01 D3 56 00 00 64 19 C8 03 E8 6B");
    T("5.6.25", zdtBuildWriteHeartbeatCmd(1, 0, 0, b, sizeof b), "01 68 38 00 00 00 00 00 6B");
    T("5.6.27", zdtBuildWriteIntegralLimitCmd(1, 0, 0x00020000, b, sizeof b), "01 4B 57 00 00 02 00 00 6B");
    T("5.6.29", zdtBuildWriteBumpReturnAngleCmd(1, 0, 5, b, sizeof b), "01 5C AC 00 00 05 6B");
    T("5.6.31", zdtBuildChangeParamLockCmd(1, 0, 0, b, sizeof b), "01 D6 4B 00 00 6B");
    T("5.4.6",  zdtBuildWriteHomingParamsCmd(1, 0, 2, 1, 100, 5000, 50, 300, 10, 0, b, sizeof b),
      "01 4C AE 00 02 01 00 64 00 00 13 88 00 32 01 2C 00 0A 00 6B");
    T("5.5.1 start", zdtBuildSetPeriodicReportCmd(1, 0x36, 100, b, sizeof b), "01 11 18 36 00 64 6B");
    T("5.5.1 stop",  zdtBuildSetPeriodicReportCmd(1, 0x36, 0, b, sizeof b), "01 11 18 36 00 00 6B");
    T("5.3.12", zdtBuildPosModeEmmCmd(1, 0, 10, 10, 711, ZDT_MOVE_REL_NOW, 0, b, sizeof b),
      "01 FD 00 00 0A 0A 00 00 02 C7 02 00 6B");
    T("5.3.12 sync", zdtBuildPosModeEmmCmd(1, 0, 10, 10, 711, ZDT_MOVE_REL_NOW, 1, b, sizeof b),
      "01 FD 00 00 0A 0A 00 00 02 C7 02 01 6B");
    T("5.3.7",  zdtBuildSpeedModeEmmCmd(1, 0, 5, 10, 0, b, sizeof b), "01 F6 00 00 05 0A 00 6B");
    T("5.3.14", zdtBuildSyncMotionCmd(0, b, sizeof b), "00 FF 66 6B");
    T("5.5.2",  zdtBuildReadVersionCmd(2, b, sizeof b), "02 1F 6B");
    T("5.6.4",  zdtBuildReadOptionsCmd(1, b, sizeof b), "01 1A 6B");
    T("5.6.18", zdtBuildReadDmx512Cmd(1, b, sizeof b), "01 49 78 6B");
    T("5.8.5",  zdtBuildReadAllConfigEmmCmd(1, b, sizeof b), "01 42 6C 6B");

    uint8_t sub[26];
    int a = zdtBuildPosModeEmmCmd(1, 1, 10, 10, 711, ZDT_MOVE_REL_NOW, 0, sub, sizeof sub);
    int c = zdtBuildPosModeEmmCmd(2, 1, 10, 10, 711, ZDT_MOVE_REL_NOW, 0, sub + a, sizeof sub - (size_t)a);
    T("5.3.1",  zdtBuildMultiMotorCmd(sub, (size_t)(a + c), b, sizeof b),
      "00 AA 00 1F 01 FD 01 00 0A 0A 00 00 02 C7 02 00 6B 02 FD 01 00 0A 0A 00 00 02 C7 02 00 6B 6B");
#undef T
}

/* 每条真机应答都能被完整切分：地址、功能码与请求一致，消耗全部字节 */
static void test_parse_hw_replies(void)
{
    for (size_t i = 0; i < sizeof kHwCases / sizeof kHwCases[0]; ++i) {
        const struct hw_case *c = &kHwCases[i];
        if (c->rx_len == 0) continue;               /* 多电机命令无应答 */
        size_t off = 0;
        int frames = 0;
        bool ok = true;
        while (off < c->rx_len) {
            zdt_reply_t r;
            int n = zdt_ttl_parse(c->rx + off, c->rx_len - off, ZDT_CHECKSUM_FIXED_6B, ZDT_FW_EMM, &r);
            if (n <= 0) { ok = false; break; }
            uint8_t want_addr = c->tx[0] ? c->tx[0] : 1;     /* 广播命令由地址 1 应答 */
            if (r.addr != want_addr) ok = false;
            /* 首帧是请求的应答（或 00 EE）；定时返回之后的帧为被查询命令（36）的格式 */
            if (frames == 0 && r.code != c->tx[1] && r.code != 0x00) ok = false;
            off += (size_t)n;
            frames++;
        }
        CHECK(ok && off == c->rx_len, "parse %s (frames=%d off=%zu/%u)", c->sec, frames, off, c->rx_len);
    }
}

static void test_parse_edge(void)
{
    zdt_reply_t r;
    /* 定时返回：确认帧后连续 36 帧 */
    const uint8_t stream[] = {0x01, 0x11, 0x02, 0x6B, 0x01, 0x36, 0x01, 0x00, 0x01, 0x7B, 0x54, 0x6B,
                              0x01, 0x36, 0x01, 0x00, 0x01, 0x7B, 0x54, 0x6B};
    int n = zdt_ttl_parse(stream, sizeof stream, ZDT_CHECKSUM_FIXED_6B, ZDT_FW_EMM, &r);
    CHECK(n == 4 && r.code == 0x11 && zdt_reply_is_ok(&r), "periodic ack");
    n = zdt_ttl_parse(stream + 4, sizeof stream - 4, ZDT_CHECKSUM_FIXED_6B, ZDT_FW_EMM, &r);
    CHECK(n == 8 && r.code == 0x36 && r.len == 5 && r.data[0] == 0x01, "periodic frame");

    /* 半帧：等待更多数据 */
    n = zdt_ttl_parse(stream + 4, 5, ZDT_CHECKSUM_FIXED_6B, ZDT_FW_EMM, &r);
    CHECK(n == 0, "partial frame waits");

    /* 命令格式错误 */
    const uint8_t ee[] = {0x02, 0x00, 0xEE, 0x6B};
    n = zdt_ttl_parse(ee, sizeof ee, ZDT_CHECKSUM_FIXED_6B, ZDT_FW_EMM, &r);
    CHECK(n == 4 && r.code == 0x00 && r.data[0] == ZDT_REPLY_FORMAT_ERR, "00 EE");

    /* 42 / 43 按帧内长度切分 */
    const uint8_t st[] = {0x01, 0x43, 0x1F, 0x09, 0x4E, 0x74, 0x00, 0x31, 0x5B, 0x95, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7B, 0x61, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x01, 0x6B};
    n = zdt_ttl_parse(st, sizeof st, ZDT_CHECKSUM_FIXED_6B, ZDT_FW_EMM, &r);
    CHECK(n == 31 && r.len == 28, "43 self-length");

    /* 两台同时应答撞包后的乱码：不得解析出帧 */
    const uint8_t junk[] = {0x02, 0x12, 0x08, 0x01, 0x21, 0xFF};
    size_t off = 0;
    int got = 0;
    while (off < sizeof junk) {
        n = zdt_ttl_parse(junk + off, sizeof junk - off, ZDT_CHECKSUM_FIXED_6B, ZDT_FW_EMM, &r);
        if (n > 0) { got++; off += (size_t)n; }
        else if (n == 0) break;
        else off++;
    }
    CHECK(got == 0, "collision garbage yields no frame");

    /* 乱码之后跟一帧有效应答：丢弃前缀后同步上 */
    const uint8_t resync[] = {0x00, 0x00, 0x21, 0x01, 0x3A, 0x03, 0x6B};
    off = 0; got = 0;
    while (off < sizeof resync) {
        n = zdt_ttl_parse(resync + off, sizeof resync - off, ZDT_CHECKSUM_FIXED_6B, ZDT_FW_EMM, &r);
        if (n > 0) { got++; off += (size_t)n; }
        else if (n == 0) break;
        else off++;
    }
    CHECK(got == 1 && r.addr == 1 && r.code == 0x3A && r.data[0] == 0x03, "resync after garbage");

    /* PID 应答长度随固件不同 */
    CHECK(zdt_ttl_reply_len(0x21, ZDT_FW_EMM) == 15 && zdt_ttl_reply_len(0x21, ZDT_FW_X) == 19, "pid len by fw");
}

static uint8_t crc8_bitwise(const uint8_t *p, size_t n)
{
    /* 与手册 cmd_calCRC8 等价的逐位实现：Maxim 反射多项式 0x8C，初值为首字节 */
    uint8_t crc = p[0];
    for (size_t i = 1; i < n; ++i) {
        crc ^= p[i];
        for (int k = 0; k < 8; ++k) crc = (crc & 1) ? (uint8_t)((crc >> 1) ^ 0x8C) : (uint8_t)(crc >> 1);
    }
    return crc;
}

static void test_checksum(void)
{
    const uint8_t cal[] = {0x01, 0x06, 0x45};
    CHECK(zdt_checksum_calc(ZDT_CHECKSUM_FIXED_6B, cal, 3) == 0x6B, "6B");
    CHECK(zdt_checksum_calc(ZDT_CHECKSUM_XOR, cal, 3) == (0x01 ^ 0x06 ^ 0x45), "xor (manual 8.1 example)");
    CHECK(zdt_checksum_calc(ZDT_CHECKSUM_CRC8, cal, 3) == crc8_bitwise(cal, 3), "crc8 example");

    /* 查表实现与逐位实现在所有单字节续接上一致（覆盖整张表） */
    bool ok = true;
    for (int a = 0; a < 256 && ok; ++a)
        for (int b = 0; b < 256; ++b) {
            uint8_t p[2] = {(uint8_t)a, (uint8_t)b};
            if (zdt_checksum_calc(ZDT_CHECKSUM_CRC8, p, 2) != crc8_bitwise(p, 2)) { ok = false; break; }
        }
    CHECK(ok, "crc8 table == bitwise Maxim");

    uint8_t f[4];
    zdtBuildReadMotorStatusCmd(1, f, sizeof f);
    zdt_frame_set_checksum(f, 3, ZDT_CHECKSUM_XOR);
    CHECK(f[2] == (0x01 ^ 0x3A), "set_checksum xor");

    /* XOR 模式下的应答也能分帧 */
    const uint8_t rx[] = {0x01, 0x3A, 0x03, 0x01 ^ 0x3A ^ 0x03};
    zdt_reply_t r;
    CHECK(zdt_ttl_parse(rx, 4, ZDT_CHECKSUM_XOR, ZDT_FW_EMM, &r) == 4, "parse xor");
    CHECK(zdt_ttl_parse(rx, 4, ZDT_CHECKSUM_FIXED_6B, ZDT_FW_EMM, &r) == ZDT_ERR_FRAME, "6B rejects xor frame");
}

/* CAN 构建函数仍等于原始帧拆包（5.6.21 为手册的 BE16 格式） */
static void test_can_still_packs_raw(void)
{
    zdt_can_msg_t m;
    int n = zdtCanBuildWritePosWindowCmd(1, 1, 8, &m);
    CHECK(n == 1 && m.frames[0].id == 0x100 && m.frames[0].dlc == 6 &&
          memcmp(m.frames[0].data, (const uint8_t[]){0xD1, 0x07, 0x01, 0x00, 0x08, 0x6B}, 6) == 0, "can 5.6.21");
    n = zdtCanBuildPosModeEmmCmd(1, 0, 10, 10, 711, ZDT_MOVE_REL_NOW, 0, &m);
    CHECK(n == 2 && m.frames[0].dlc == 8 && m.frames[1].dlc == 5 && m.frames[1].data[0] == 0xFD, "can 5.3.12 split");
}

int main(void)
{
    test_builders_vs_hw();
    test_parse_hw_replies();
    test_parse_edge();
    test_checksum();
    test_can_still_packs_raw();
    printf("%s: pass=%d fail=%d\n", s_fail ? "FAILED" : "OK", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
