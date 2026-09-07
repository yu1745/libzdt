/*
 * zdt_common_param.c - 通用读写驱动参数模块实现 (CAN 扩展帧构建)
 */

#include "zdt_common_param.h"
#include "zdt_internal.h"

/* 5.6.1 修改电机 ID/地址  Addr + AE + 4B + store + new_addr + 6B  — 6B */
static int _raw_change_addr(uint8_t addr, uint8_t store, uint8_t new_addr,
                            uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xAE);
    zdt_append_u8(buf, &i, max_len, 0x4B);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, new_addr);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeAddrCmd(uint8_t addr, uint8_t store,
                            uint8_t new_addr,
                            zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_addr(addr, store, new_addr, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.2 修改细分值  Addr + 84 + 8A + store + microstep + 6B  — 6B */
static int _raw_change_microstep(uint8_t addr, uint8_t store, uint8_t microstep,
                                uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x84);
    zdt_append_u8(buf, &i, max_len, 0x8A);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, microstep);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeMicrostepCmd(uint8_t addr, uint8_t store,
                                 uint8_t microstep,
                                 zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_microstep(addr, store, microstep, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.3 修改掉电标志  Addr + 50 + flag + 6B  — 4B */
static int _raw_change_power_down_flag(uint8_t addr, uint8_t flag,
                                      uint8_t *buf, size_t max_len)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x50);
    zdt_append_u8(buf, &i, max_len, flag);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangePowerDownFlagCmd(uint8_t addr, uint8_t flag,
                                     zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_power_down_flag(addr, flag, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.4 读取选项参数状态 (X42S/Y42)  Addr + 1A + 6B  — 3B */
static int _raw_read_options(uint8_t addr, uint8_t *buf, size_t max_len)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x1A);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadOptionsCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_read_options(addr, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.5 修改电机类型  Addr + D7 + 35 + store + motor_type + 6B  — 6B */
static int _raw_change_motor_type(uint8_t addr, uint8_t store, uint8_t motor_type,
                                 uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xD7);
    zdt_append_u8(buf, &i, max_len, 0x35);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, motor_type);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeMotorTypeCmd(uint8_t addr, uint8_t store,
                                 uint8_t motor_type,
                                 zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_motor_type(addr, store, motor_type, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.6 修改固件类型  Addr + D5 + 69 + store + fw_type + 6B  — 6B */
static int _raw_change_firmware_type(uint8_t addr, uint8_t store, uint8_t fw_type,
                                    uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xD5);
    zdt_append_u8(buf, &i, max_len, 0x69);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, fw_type);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeFirmwareTypeCmd(uint8_t addr, uint8_t store,
                                    uint8_t fw_type,
                                    zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_firmware_type(addr, store, fw_type, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.7 修改开环/闭环控制模式  Addr + 46 + A6 + store + mode + 6B  — 6B */
static int _raw_change_ctrl_mode(uint8_t addr, uint8_t store, uint8_t mode,
                                uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x46);
    zdt_append_u8(buf, &i, max_len, 0xA6);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, mode);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeCtrlModeCmd(uint8_t addr, uint8_t store,
                                uint8_t mode,
                                zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_ctrl_mode(addr, store, mode, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.8 修改电机运动正方向  Addr + D4 + 60 + store + dir + 6B  — 6B */
static int _raw_change_motor_dir(uint8_t addr, uint8_t store, uint8_t dir,
                                uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xD4);
    zdt_append_u8(buf, &i, max_len, 0x60);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, dir);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeMotorDirCmd(uint8_t addr, uint8_t store,
                                uint8_t dir,
                                zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_motor_dir(addr, store, dir, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.9 修改锁定按键功能  Addr + D0 + B3 + store + lock + 6B  — 6B */
static int _raw_change_key_lock(uint8_t addr, uint8_t store, uint8_t lock,
                               uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xD0);
    zdt_append_u8(buf, &i, max_len, 0xB3);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, lock);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeKeyLockCmd(uint8_t addr, uint8_t store,
                               uint8_t lock,
                               zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_key_lock(addr, store, lock, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.12 修改开环模式工作电流  Addr + 44 + 33 + store + current(BE16) + 6B  — 7B */
static int _raw_change_open_loop_current(uint8_t addr, uint8_t store, uint16_t current_ma,
                                        uint8_t *buf, size_t max_len)
{
    const size_t need = 7;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x44);
    zdt_append_u8(buf, &i, max_len, 0x33);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u16_be(buf, &i, max_len, current_ma);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeOpenLoopCurrentCmd(uint8_t addr, uint8_t store,
                                       uint16_t current_ma,
                                       zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_open_loop_current(addr, store, current_ma, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.13 修改闭环模式最大电流  Addr + 45 + 66 + store + current(BE16) + 6B  — 7B */
static int _raw_change_closed_loop_current(uint8_t addr, uint8_t store, uint16_t current_ma,
                                          uint8_t *buf, size_t max_len)
{
    const size_t need = 7;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x45);
    zdt_append_u8(buf, &i, max_len, 0x66);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u16_be(buf, &i, max_len, current_ma);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeClosedLoopCurrentCmd(uint8_t addr, uint8_t store,
                                         uint16_t current_ma,
                                         zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_closed_loop_current(addr, store, current_ma, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.18 读取 DMX512 协议参数 (X42S/Y42)  Addr + 49 + 78 + 6B  — 4B */
static int _raw_read_dmx512(uint8_t addr, uint8_t *buf, size_t max_len)
{
    const size_t need = 4;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x49);
    zdt_append_u8(buf, &i, max_len, 0x78);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadDmx512Cmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_read_dmx512(addr, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.19 修改 DMX512 协议参数 (X42S/Y42)  — 19B */
static int _raw_write_dmx512(uint8_t addr, uint8_t store,
                            uint16_t total_channels,
                            uint8_t channels_per_motor,
                            uint8_t move_mode,
                            uint16_t single_speed_rpm,
                            uint16_t accel,
                            uint16_t speed_step,
                            uint32_t motion_step,
                            uint8_t *buf, size_t max_len)
{
    const size_t need = 19;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xD9);
    zdt_append_u8(buf, &i, max_len, 0x90);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u16_be(buf, &i, max_len, total_channels);
    zdt_append_u8(buf, &i, max_len, channels_per_motor);
    zdt_append_u8(buf, &i, max_len, move_mode);
    zdt_append_u16_be(buf, &i, max_len, single_speed_rpm);
    zdt_append_u16_be(buf, &i, max_len, accel);
    zdt_append_u16_be(buf, &i, max_len, speed_step);
    zdt_append_u32_be(buf, &i, max_len, motion_step);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWriteDmx512Cmd(uint8_t addr, uint8_t store,
                             uint16_t total_channels,
                             uint8_t channels_per_motor,
                             uint8_t move_mode,
                             uint16_t single_speed_rpm,
                             uint16_t accel,
                             uint16_t speed_step,
                             uint32_t motion_step,
                             zdt_can_msg_t *msg)
{
    uint8_t raw[32];
    int len = _raw_write_dmx512(addr, store, total_channels, channels_per_motor,
                               move_mode, single_speed_rpm, accel, speed_step,
                               motion_step, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.20 读取位置到达窗口 (X42S/Y42)  Addr + 41 + 6B  — 3B */
static int _raw_read_pos_window(uint8_t addr, uint8_t *buf, size_t max_len)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x41);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadPosWindowCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_read_pos_window(addr, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.21 修改位置到达窗口 (X42S/Y42)  Addr + D1 + 07 + store + window(单字节, ×0.1°) + 6B  — 6B */
static int _raw_write_pos_window(uint8_t addr, uint8_t store, uint16_t window_x10,
                                uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xD1);
    zdt_append_u8(buf, &i, max_len, 0x07);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, (uint8_t)window_x10);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWritePosWindowCmd(uint8_t addr, uint8_t store,
                                uint16_t window_x10,
                                zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_write_pos_window(addr, store, window_x10, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.22 读取过热过流保护检测阈值 (X42S/Y42)  Addr + 13 + 6B  — 3B */
static int _raw_read_protect_threshold(uint8_t addr, uint8_t *buf, size_t max_len)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x13);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadProtectThresholdCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_read_protect_threshold(addr, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.23 修改过热过流保护检测阈值 (X42S/Y42)  — 11B */
static int _raw_write_protect_threshold(uint8_t addr, uint8_t store,
                                       uint16_t overheat_c,
                                       uint16_t overcurrent_ma,
                                       uint16_t detect_ms,
                                       uint8_t *buf, size_t max_len)
{
    const size_t need = 11;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xD3);
    zdt_append_u8(buf, &i, max_len, 0x56);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u16_be(buf, &i, max_len, overheat_c);
    zdt_append_u16_be(buf, &i, max_len, overcurrent_ma);
    zdt_append_u16_be(buf, &i, max_len, detect_ms);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWriteProtectThresholdCmd(uint8_t addr, uint8_t store,
                                       uint16_t overheat_c,
                                       uint16_t overcurrent_ma,
                                       uint16_t detect_ms,
                                       zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_write_protect_threshold(addr, store, overheat_c, overcurrent_ma,
                                          detect_ms, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.24 读取心跳保护功能时间 (X42S/Y42)  Addr + 16 + 6B  — 3B */
static int _raw_read_heartbeat(uint8_t addr, uint8_t *buf, size_t max_len)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x16);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadHeartbeatCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_read_heartbeat(addr, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.25 修改心跳保护功能时间 (X42S/Y42)  Addr + 68 + 38 + store + heartbeat_ms(BE32) + 6B  — 9B */
static int _raw_write_heartbeat(uint8_t addr, uint8_t store, uint32_t heartbeat_ms,
                               uint8_t *buf, size_t max_len)
{
    const size_t need = 9;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x68);
    zdt_append_u8(buf, &i, max_len, 0x38);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u32_be(buf, &i, max_len, heartbeat_ms);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWriteHeartbeatCmd(uint8_t addr, uint8_t store,
                                uint32_t heartbeat_ms,
                                zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_write_heartbeat(addr, store, heartbeat_ms, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.26 读取积分限幅/刚性系数 (X42S/Y42)  Addr + 23 + 6B  — 3B */
static int _raw_read_integral_limit(uint8_t addr, uint8_t *buf, size_t max_len)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x23);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadIntegralLimitCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_read_integral_limit(addr, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.27 修改积分限幅/刚性系数 (X42S/Y42)  Addr + 4B + 57 + store + value(BE32) + 6B  — 9B */
static int _raw_write_integral_limit(uint8_t addr, uint8_t store, uint32_t value,
                                    uint8_t *buf, size_t max_len)
{
    const size_t need = 9;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x4B);
    zdt_append_u8(buf, &i, max_len, 0x57);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u32_be(buf, &i, max_len, value);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWriteIntegralLimitCmd(uint8_t addr, uint8_t store,
                                    uint32_t value,
                                    zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_write_integral_limit(addr, store, value, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.28 读取碰撞回零返回角度 (X42S/Y42)  Addr + 3F + 6B  — 3B */
static int _raw_read_bump_return_angle(uint8_t addr, uint8_t *buf, size_t max_len)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x3F);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildReadBumpReturnAngleCmd(uint8_t addr, zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_read_bump_return_angle(addr, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.29 修改碰撞回零返回角度 (X42S/Y42)  Addr + 5C + AC + store + angle(BE16, ×0.1°) + 6B  — 6B */
static int _raw_write_bump_return_angle(uint8_t addr, uint8_t store, uint16_t angle_x10,
                                       uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0x5C);
    zdt_append_u8(buf, &i, max_len, 0xAC);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u16_be(buf, &i, max_len, angle_x10);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildWriteBumpReturnAngleCmd(uint8_t addr, uint8_t store,
                                      uint16_t angle_x10,
                                      zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_write_bump_return_angle(addr, store, angle_x10, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}

/* 5.6.30 广播读取 ID 地址 (X42S/Y42)  00 + 15 + 6B  — 3B */
static int _raw_broadcast_read_addr(uint8_t *buf, size_t max_len)
{
    const size_t need = 3;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, ZDT_ADDR_BROADCAST);
    zdt_append_u8(buf, &i, max_len, 0x15);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildBroadcastReadAddrCmd(zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_broadcast_read_addr(raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(ZDT_ADDR_BROADCAST, raw, (size_t)len, msg);
}

/* 5.6.31 修改锁定修改参数功能 (X42S/Y42)  Addr + D6 + 4B + store + lock_level(0-3) + 6B  — 6B */
static int _raw_change_param_lock(uint8_t addr, uint8_t store, uint8_t lock_level,
                                 uint8_t *buf, size_t max_len)
{
    const size_t need = 6;
    size_t i = 0;
    if (zdt_check_size(0, need, max_len) != ZDT_OK) return ZDT_ERR_BUF_TOO_SMALL;
    zdt_append_u8(buf, &i, max_len, addr);
    zdt_append_u8(buf, &i, max_len, 0xD6);
    zdt_append_u8(buf, &i, max_len, 0x4B);
    zdt_append_u8(buf, &i, max_len, store);
    zdt_append_u8(buf, &i, max_len, lock_level);
    zdt_append_u8(buf, &i, max_len, ZDT_CHECKSUM_DEFAULT);
    return (int)i;
}

int zdtCanBuildChangeParamLockCmd(uint8_t addr, uint8_t store,
                                 uint8_t lock_level,
                                 zdt_can_msg_t *msg)
{
    uint8_t raw[16];
    int len = _raw_change_param_lock(addr, store, lock_level, raw, sizeof(raw));
    if (len < 0) return len;
    return zdt_pack_can_msg(addr, raw, (size_t)len, msg);
}
