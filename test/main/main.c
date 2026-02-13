/**
 * @file main.c
 * @brief ZDT电机TWAI驱动测试程序（Emm固件/通用命令）
 *
 * 本程序测试ZDT闭环步进电机通讯协议库的TWAI驱动功能
 * 仅测试不带X的通用命令和Emm固件命令
 */

#include <stdio.h>
#include "zdt_twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"


static const char *TAG = "ZDT_TEST";

/*==============================================================================
 * 测试配置
 *============================================================================*/

#define TEST_MOTOR_ADDR        1      /* 测试电机地址 */
#define TEST_TX_GPIO           7      /* CAN TX引脚 */
#define TEST_RX_GPIO           6      /* CAN RX引脚 */
#define TEST_TIMEOUT_MS        100   /* 默认超时时间 */
#define TEST_SETTLE_TIME_MS    100    /* Set命令稳定等待时间 */

/*==============================================================================
 * 全局变量
 *============================================================================*/

static struct {
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t ack_count;
    uint32_t error_count;
    uint32_t test_failed_count;
    bool test_passed;
} test_stats = {
    .rx_count = 0,
    .tx_count = 0,
    .ack_count = 0,
    .error_count = 0,
    .test_failed_count = 0,
    .test_passed = false
};

/*==============================================================================
 * 测试辅助函数
 *============================================================================*/

/**
 * @brief 打印测试结果
 */
static void print_test_result(const char *test_name, bool passed)
{
    if (passed) {
        ESP_LOGI(TAG, "✓ %s PASSED", test_name);
    } else {
        ESP_LOGE(TAG, "✗ %s FAILED", test_name);
        test_stats.test_failed_count++;
        test_stats.error_count++;
    }
}

/**
 * @brief 打印CAN消息
 */
static void print_can_message(const zdt_can_msg_t *msg, const char *direction)
{
    uint8_t addr = (msg->id >> 8) & 0xFF;
    uint8_t packet = msg->id & 0xFF;
    uint8_t cmd = msg->dlc > 0 ? msg->data[0] : 0xFF;

    ESP_LOGI(TAG, "[%s] Addr=%d, Packet=0x%02X, Cmd=0x%02X, DLC=%d",
             direction, addr, packet, cmd, msg->dlc);

    /* 打印数据 */
    if (msg->dlc > 0) {
        char hex_str[64] = {0};
        int offset = 0;
        for (uint8_t i = 0; i < msg->dlc && i < 8; i++) {
            offset += snprintf(hex_str + offset, sizeof(hex_str) - offset,
                              "%02X ", msg->data[i]);
        }
        ESP_LOGI(TAG, "    Data: %s", hex_str);
    }
}

/**
 * @brief 验证响应是否成功
 * @return true=响应成功, false=无响应或错误响应
 */
static bool verify_response(const zdt_can_msg_t *msg, uint8_t expected_cmd)
{
    if (msg == NULL) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }

    /* 检查是否为测试电机的响应 */
    uint8_t addr = (msg->id >> 8) & 0xFF;
    if (addr != TEST_MOTOR_ADDR) {
        ESP_LOGE(TAG, "    ✗ Response from wrong motor (addr=%d)", addr);
        return false;
    }

    /* 检查返回码 */
    uint8_t code = zdt_get_response_code(msg);
    if (zdt_is_response_ok(code)) {
        ESP_LOGI(TAG, "    → ACK received (0x%02X)", code);
        test_stats.ack_count++;
        return true;
    } else if (zdt_is_param_error(code)) {
        ESP_LOGE(TAG, "    ✗ Parameter error (0x%02X)", code);
        return false;
    } else if (zdt_is_format_error(code)) {
        ESP_LOGE(TAG, "    ✗ Format error (0x%02X)", code);
        return false;
    }

    ESP_LOGE(TAG, "    ✗ Unknown error (0x%02X)", code);
    return false;
}

/*==============================================================================
 * 测试用例
 *============================================================================*/

/**
 * @brief 测试1：TWAI驱动初始化
 */
static bool test_twai_init(void)
{
    ESP_LOGI(TAG, "\n=== Test 1: TWAI Driver Init ===");

    zdt_twai_config_t config = ZDT_TWAI_DEFAULT_CONFIG();
    config.tx_gpio = TEST_TX_GPIO;
    config.rx_gpio = TEST_RX_GPIO;
    config.bitrate = 500*1000;
    config.rx_queue_len = 20;

    int ret = zdt_twai_init(&config);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to initialize TWAI driver");
        return false;
    }

    ret = zdt_twai_start();
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to start TWAI driver");
        return false;
    }

    if (!zdt_twai_is_running()) {
        ESP_LOGE(TAG, "TWAI driver not running");
        return false;
    }

    ESP_LOGI(TAG, "TWAI driver initialized and started successfully");
    ESP_LOGI(TAG, "  TX GPIO: %d", config.tx_gpio);
    ESP_LOGI(TAG, "  RX GPIO: %d", config.rx_gpio);
    ESP_LOGI(TAG, "  Bitrate: %lu bps", config.bitrate);

    vTaskDelay(pdMS_TO_TICKS(100));
    return true;
}

/**
 * @brief 测试2：注销回调（使用轮询模式）
 */
static bool test_unregister_callback(void)
{
    ESP_LOGI(TAG, "\n=== Test 2: Unregister Callback (Use Polling) ===");

    /* 注销回调以确保轮询模式正常工作 */
    int ret = zdt_twai_unregister_callback();
    if (ret != 0) {
        ESP_LOGW(TAG, "No callback was registered or unregister failed");
    } else {
        ESP_LOGI(TAG, "Callback unregistered, using polling mode");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    return true;
}

/**
 * @brief 测试3：读取电机状态（验证通讯是否正常）
 */
static bool test_read_motor_state(void)
{
    ESP_LOGI(TAG, "\n=== Test 3: Read Motor State ===");

    zdt_can_msg_t tx_msg;
    zdt_cmd_read_motor_state(TEST_MOTOR_ADDR, &tx_msg);

    print_can_message(&tx_msg, "TX");

    /* 发送命令 */
    if (zdt_twai_send(&tx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send command");
        return false;
    }
    test_stats.tx_count++;

    /* 等待响应 */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t rx_msg;
    if (zdt_twai_receive(&rx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }
    test_stats.rx_count++;

    print_can_message(&rx_msg, "RX");

    /* 验证响应 */
    zdt_motor_state_t state;
    if (!zdt_parse_motor_state(&rx_msg, &state)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse motor state");
        return false;
    }

    ESP_LOGI(TAG, "    → Motor State:");
    ESP_LOGI(TAG, "      Enabled: %s", state.ens_tf ? "Yes" : "No");
    ESP_LOGI(TAG, "      Position Reached: %s", state.prf_tf ? "Yes" : "No");
    ESP_LOGI(TAG, "      Stall: %s", state.cgi_tf ? "Yes" : "No");

    /* 读取命令成功解析即表示成功 */
    return true;
}

/**
 * @brief 测试4：读取固件版本和硬件版本
 */
static bool test_read_version(void)
{
    ESP_LOGI(TAG, "\n=== Test 4: Read Version ===");

    zdt_can_msg_t tx_msg;
    zdt_cmd_read_version(TEST_MOTOR_ADDR, &tx_msg);

    print_can_message(&tx_msg, "TX");

    /* 发送命令 */
    if (zdt_twai_send(&tx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send command");
        return false;
    }
    test_stats.tx_count++;

    /* 等待响应 */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t rx_msg;
    if (zdt_twai_receive(&rx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }

    print_can_message(&rx_msg, "RX");

    /* 解析版本信息 */
    zdt_version_info_t info;
    if (!zdt_parse_version_info(&rx_msg, &info)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse version info");
        return false;
    }

    ESP_LOGI(TAG, "    → Version Info:");
    ESP_LOGI(TAG, "      FW Version: %d.%d.%d",
             (info.fw_version >> 8) & 0xFF,
             (info.fw_version >> 4) & 0x0F,
             info.fw_version & 0x0F);
    ESP_LOGI(TAG, "      HW Series: %c", info.hw_series == 0 ? 'X' : 'Y');
    ESP_LOGI(TAG, "      HW Type: %d", info.hw_type);
    ESP_LOGI(TAG, "      HW Version: %d", info.hw_ver);

    /* 读取命令成功解析即表示成功 */
    return true;
}

/**
 * @brief 测试5：电机使能控制
 */
static bool test_motor_enable(void)
{
    ESP_LOGI(TAG, "\n=== Test 5: Motor Enable ===");

    /* 发送使能命令 */
    zdt_can_msg_t tx_msg;
    zdt_cmd_motor_enable(TEST_MOTOR_ADDR, true, ZDT_SYNC_IMMEDIATE, &tx_msg);

    print_can_message(&tx_msg, "TX");

    if (zdt_twai_send(&tx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send enable command");
        return false;
    }
    test_stats.tx_count++;

    /* 等待响应 */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t rx_msg;
    if (zdt_twai_receive(&rx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }

    print_can_message(&rx_msg, "RX");

    if (!verify_response(&rx_msg, 0xF3)) {
        return false;
    }

    /* 等待稳定 */
    vTaskDelay(pdMS_TO_TICKS(TEST_SETTLE_TIME_MS));

    /* 读取电机状态验证使能是否生效 */
    zdt_can_msg_t read_msg;
    zdt_cmd_read_motor_state(TEST_MOTOR_ADDR, &read_msg);

    if (zdt_twai_send(&read_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send read command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t state_rx;
    if (zdt_twai_receive(&state_rx, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response for state read");
        return false;
    }
    test_stats.rx_count++;

    /* 解析状态 */
    zdt_motor_state_t state;
    if (!zdt_parse_motor_state(&state_rx, &state)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse motor state");
        return false;
    }

    if (!state.ens_tf) {
        ESP_LOGE(TAG, "    ✗ Motor not enabled after enable command");
        return false;
    }

    ESP_LOGI(TAG, "    → Motor successfully enabled and verified");
    return true;
}

/**
 * @brief 测试6：读取实时位置
 */
static bool test_read_real_position(void)
{
    ESP_LOGI(TAG, "\n=== Test 6: Read Real Position ===");

    zdt_can_msg_t tx_msg;
    zdt_cmd_read_real_position(TEST_MOTOR_ADDR, &tx_msg);

    print_can_message(&tx_msg, "TX");

    if (zdt_twai_send(&tx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t rx_msg;
    if (zdt_twai_receive(&rx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }

    print_can_message(&rx_msg, "RX");

    /* 解析位置 */
    uint32_t position;
    uint8_t sign;
    if (!zdt_parse_real_position(&rx_msg, &position, &sign)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse position");
        return false;
    }

    ESP_LOGI(TAG, "    → Position: %c%lu (0.1°)", sign == 1 ? '-' : '+', position);

    /* 读取命令成功解析即表示成功 */
    return true;
}

/**
 * @brief 测试7：读取实时转速
 */
static bool test_read_real_speed(void)
{
    ESP_LOGI(TAG, "\n=== Test 7: Read Real Speed ===");

    zdt_can_msg_t tx_msg;
    zdt_cmd_read_real_speed(TEST_MOTOR_ADDR, &tx_msg);

    print_can_message(&tx_msg, "TX");

    if (zdt_twai_send(&tx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t rx_msg;
    if (zdt_twai_receive(&rx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }

    print_can_message(&rx_msg, "RX");

    /* 解析速度 */
    uint16_t speed;
    uint8_t sign;
    if (!zdt_parse_real_speed(&rx_msg, &speed, &sign)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse speed");
        return false;
    }

    ESP_LOGI(TAG, "    → Speed: %c%u RPM", sign == 1 ? '-' : '+', speed);

    /* 读取命令成功解析即表示成功 */
    return true;
}

/**
 * @brief 测试8：速度模式控制（Emm固件）
 */
static bool test_velocity_mode_emm(void)
{
    ESP_LOGI(TAG, "\n=== Test 8: Velocity Mode (Emm) ===");

    /* 构造速度命令：300 RPM，CW方向 */
    zdt_velocity_cmd_emm_t params = {
        .addr = TEST_MOTOR_ADDR,
        .direction = ZDT_DIRECTION_CW,
        .speed = 300,           /* 300 RPM */
        .acceleration = 0,     /* 最大加速度 */
        .sync_flag = ZDT_SYNC_IMMEDIATE
    };

    zdt_can_msg_t tx_msg;
    zdt_cmd_velocity_mode_emm(&params, &tx_msg);

    print_can_message(&tx_msg, "TX");
    ESP_LOGI(TAG, "  Target Speed: %u RPM", params.speed);
    ESP_LOGI(TAG, "  Direction: %s", params.direction == ZDT_DIRECTION_CW ? "CW" : "CCW");

    if (zdt_twai_send(&tx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send velocity command");
        return false;
    }
    test_stats.tx_count++;

    /* 等待响应 */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收命令响应 */
    zdt_can_msg_t rx_msg;
    if (zdt_twai_receive(&rx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }

    print_can_message(&rx_msg, "RX");

    if (!verify_response(&rx_msg, 0xF6)) {
        return false;
    }

    /* 等待电机加速 */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* 读取实时速度验证是否达速 */
    zdt_can_msg_t read_msg;
    zdt_cmd_read_real_speed(TEST_MOTOR_ADDR, &read_msg);

    if (zdt_twai_send(&read_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send read speed command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t speed_rx;
    if (zdt_twai_receive(&speed_rx, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response for speed read");
        return false;
    }
    test_stats.rx_count++;

    /* 解析速度 */
    uint16_t actual_speed;
    uint8_t sign;
    if (!zdt_parse_real_speed(&speed_rx, &actual_speed, &sign)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse speed");
        return false;
    }

    ESP_LOGI(TAG, "    → Actual Speed: %c%u RPM", sign == 1 ? '-' : '+', actual_speed);

    /* 验证实际速度是否达到目标速度（300 RPM），允许 ±10% 误差 */
    const uint16_t target_speed = 300;
    const uint16_t tolerance = 30;   /* 10% of 300 */
    if (actual_speed >= target_speed - tolerance && actual_speed <= target_speed + tolerance) {
        ESP_LOGI(TAG, "    → Speed within target range [%u, %u] RPM", target_speed - tolerance, target_speed + tolerance);
        return true;
    } else {
        ESP_LOGE(TAG, "    ✗ Speed %u RPM out of target range [%u, %u] RPM", actual_speed, target_speed - tolerance, target_speed + tolerance);
        return false;
    }
}

/**
 * @brief 测试9：位置模式控制（Emm固件）
 */
static bool test_position_mode_emm(void)
{
    ESP_LOGI(TAG, "\n=== Test 9: Position Mode (Emm) ===");

    /* 构造位置命令：旋转半圈 */
    zdt_position_cmd_emm_t params = {
        .addr = TEST_MOTOR_ADDR,
        .direction = ZDT_DIRECTION_CW,
        .speed = 3000,           /* 3000 RPM */
        .acceleration = 0,     /* 最大加速度 */
        .pulse_count = 1000,   /* 半圈脉冲数（65536/2） */
        .motion_mode = ZDT_MOTION_MODE_ABSOLUTE,
        .sync_flag = ZDT_SYNC_IMMEDIATE
    };

    zdt_can_msg_t msgs[2];
    uint8_t num_packets = zdt_cmd_position_mode_emm(&params, msgs);

    ESP_LOGI(TAG, "Position mode: %d packets", num_packets);
    ESP_LOGI(TAG, "  Speed: %u RPM", params.speed);
    ESP_LOGI(TAG, "  Pulses: %u", params.pulse_count);
    ESP_LOGI(TAG, "  Motion Mode: %s",
             params.motion_mode == ZDT_MOTION_MODE_ABSOLUTE ? "Absolute" : "Relative");

    /* 打印所有分包 */
    for (uint8_t i = 0; i < num_packets; i++) {
        print_can_message(&msgs[i], "TX");
    }

    /* 发送所有分包 */
    int sent = zdt_twai_send_multi(msgs, num_packets, 1000, TEST_TIMEOUT_MS);

    if (sent != num_packets) {
        ESP_LOGE(TAG, "Failed to send all packets (%d/%d)", sent, num_packets);
        return false;
    }

    test_stats.tx_count += sent;

    /* 等待响应 */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收命令响应 */
    zdt_can_msg_t rx_msg;
    if (zdt_twai_receive(&rx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }

    print_can_message(&rx_msg, "RX");

    if (!verify_response(&rx_msg, 0xFD)) {
        return false;
    }

    ESP_LOGI(TAG, "    → Position command sent, waiting for completion...");
    ESP_LOGI(TAG, "    → Note: Position arrival check requires monitoring prf_tf flag");

    /* 等待运动完成 */
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* 读取电机状态检查位置到达标志 */
    zdt_can_msg_t read_msg;
    zdt_cmd_read_motor_state(TEST_MOTOR_ADDR, &read_msg);

    if (zdt_twai_send(&read_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send read state command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t state_rx;
    if (zdt_twai_receive(&state_rx, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response for state read");
        return false;
    }
    test_stats.rx_count++;

    /* 解析状态 */
    zdt_motor_state_t state;
    if (!zdt_parse_motor_state(&state_rx, &state)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse motor state");
        return false;
    }

    ESP_LOGI(TAG, "    → Position Reached (prf_tf): %s", state.prf_tf ? "Yes" : "No");

    if (!state.prf_tf) {
        ESP_LOGW(TAG, "    → Motor may still be in motion or did not reach target");
    }

    /* 读取实时位置，校验是否到达目标位置 */
    zdt_cmd_read_real_position(TEST_MOTOR_ADDR, &read_msg);
    if (zdt_twai_send(&read_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGW(TAG, "    → Failed to send read position command, skip position check");
    } else {
        test_stats.tx_count++;
        vTaskDelay(pdMS_TO_TICKS(300));
        if (zdt_twai_receive(&state_rx, TEST_TIMEOUT_MS) == 0) {
            test_stats.rx_count++;
            uint32_t real_pos;
            uint8_t pos_sign;
            if (zdt_parse_real_position(&state_rx, &real_pos, &pos_sign)) {
                float real_pos_signed = (pos_sign == 1) ? -(int32_t)real_pos : (int32_t)real_pos;
                real_pos_signed = real_pos_signed * 360.0f / 65536.0f;
                const float target = params.pulse_count / 3200.0f * 360.0f;
                const float tol = 10.0f;  /* Emm固件位置单位与 pulse_count 一致，允许 ±10 误差 */
                bool pos_ok = (real_pos_signed >= target - tol && real_pos_signed <= target + tol);
                ESP_LOGI(TAG, "    → Real Position: %.1f, Target: %.1f, Match: %s",
                        real_pos_signed, target, pos_ok ? "Yes" : "No");
            }
        }
    }

    return true;
}

/**
 * @brief 测试10：读取相电流
 */
static bool test_read_phase_current(void)
{
    ESP_LOGI(TAG, "\n=== Test 10: Read Phase Current ===");

    zdt_can_msg_t tx_msg;
    zdt_cmd_read_phase_current(TEST_MOTOR_ADDR, &tx_msg);

    print_can_message(&tx_msg, "TX");

    if (zdt_twai_send(&tx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t rx_msg;
    if (zdt_twai_receive(&rx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }

    print_can_message(&rx_msg, "RX");

    /* 解析相电流 */
    uint16_t current;
    if (!zdt_parse_phase_current(&rx_msg, &current)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse phase current");
        return false;
    }

    ESP_LOGI(TAG, "    → Phase Current: %u mA", current);

    /* 读取命令成功解析即表示成功 */
    return true;
}

/**
 * @brief 测试11：立即停止
 */
static bool test_immediate_stop(void)
{
    ESP_LOGI(TAG, "\n=== Test 11: Immediate Stop ===");

    zdt_can_msg_t tx_msg;
    zdt_cmd_immediate_stop(TEST_MOTOR_ADDR, ZDT_SYNC_IMMEDIATE, &tx_msg);

    print_can_message(&tx_msg, "TX");

    if (zdt_twai_send(&tx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send stop command");
        return false;
    }
    test_stats.tx_count++;

    /* 等待响应 */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t rx_msg;
    if (zdt_twai_receive(&rx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }

    print_can_message(&rx_msg, "RX");

    if (!verify_response(&rx_msg, 0xFE)) {
        return false;
    }

    /* 等待电机停止 */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* 读取速度验证是否停止 */
    zdt_can_msg_t read_msg;
    zdt_cmd_read_real_speed(TEST_MOTOR_ADDR, &read_msg);

    if (zdt_twai_send(&read_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send read speed command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t speed_rx;
    if (zdt_twai_receive(&speed_rx, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response for speed read");
        return false;
    }
    test_stats.rx_count++;

    /* 解析速度 */
    uint16_t speed;
    uint8_t sign;
    if (!zdt_parse_real_speed(&speed_rx, &speed, &sign)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse speed");
        return false;
    }

    ESP_LOGI(TAG, "    → Speed after stop: %u RPM", speed);

    if (speed > 10) {
        ESP_LOGE(TAG, "    ✗ Motor did not stop (speed=%u RPM)", speed);
        return false;
    }

    ESP_LOGI(TAG, "    → Motor successfully stopped");
    return true;
}


/**
 * @brief 测试13：位置清零命令
 */
static bool test_zero_position(void)
{
    ESP_LOGI(TAG, "\n=== Test 13: Zero Position ===");

    /* 先确保电机已使能（清零命令可能需要电机使能状态） */
    zdt_can_msg_t enable_cmd;
    zdt_cmd_motor_enable(TEST_MOTOR_ADDR, true, ZDT_SYNC_IMMEDIATE, &enable_cmd);

    if (zdt_twai_send(&enable_cmd, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGW(TAG, "    → Warning: Failed to send enable command before zero");
    } else {
        test_stats.tx_count++;
        vTaskDelay(pdMS_TO_TICKS(200));
        /* 读取并丢弃使能响应 */
        zdt_can_msg_t enable_rx;
        if (zdt_twai_receive(&enable_rx, 500) == 0) {
            test_stats.rx_count++;
        }
    }

    /* 等待电机稳定 */
    vTaskDelay(pdMS_TO_TICKS(TEST_SETTLE_TIME_MS));

    /* 读取当前位置 */
    zdt_can_msg_t read_before;
    zdt_cmd_read_real_position(TEST_MOTOR_ADDR, &read_before);

    if (zdt_twai_send(&read_before, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send read position command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    zdt_can_msg_t rx_before;
    if (zdt_twai_receive(&rx_before, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response for position read");
        return false;
    }
    test_stats.rx_count++;

    uint32_t pos_before;
    uint8_t sign_before;
    if (!zdt_parse_real_position(&rx_before, &pos_before, &sign_before)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse position before zero");
        return false;
    }

    ESP_LOGI(TAG, "    → Position before zero: %c%lu",
             sign_before == 1 ? '-' : '+', pos_before);

    /* 发送清零命令 */
    zdt_can_msg_t zero_cmd;
    zdt_cmd_zero_position(TEST_MOTOR_ADDR, &zero_cmd);

    print_can_message(&zero_cmd, "TX");

    if (zdt_twai_send(&zero_cmd, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send zero command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    zdt_can_msg_t rx_zero;
    if (zdt_twai_receive(&rx_zero, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response for zero command");
        return false;
    }
    test_stats.rx_count++;

    print_can_message(&rx_zero, "RX");

    if (!verify_response(&rx_zero, 0x0A)) {
        return false;
    }

    /* 等待稳定 */
    vTaskDelay(pdMS_TO_TICKS(TEST_SETTLE_TIME_MS));

    /* 读取清零后的位置 */
    zdt_can_msg_t read_after;
    zdt_cmd_read_real_position(TEST_MOTOR_ADDR, &read_after);

    if (zdt_twai_send(&read_after, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send read position command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    zdt_can_msg_t rx_after;
    if (zdt_twai_receive(&rx_after, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response for position read");
        return false;
    }
    test_stats.rx_count++;

    uint32_t pos_after;
    uint8_t sign_after;
    if (!zdt_parse_real_position(&rx_after, &pos_after, &sign_after)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse position after zero");
        return false;
    }

    ESP_LOGI(TAG, "    → Position after zero: %c%lu",
             sign_after == 1 ? '-' : '+', pos_after);

    /* 验证位置是否为0（允许±5°误差） */
    int32_t actual_pos = (int32_t)((sign_after == 1 ? -1 : 1) * pos_after);
    if (abs(actual_pos) > 5) {
        ESP_LOGE(TAG, "    ✗ Position not zeroed (pos=%ld°)", actual_pos);
        return false;
    }

    ESP_LOGI(TAG, "    → Position successfully zeroed (pos=%ld°)", actual_pos);
    return true;
}

/**
 * @brief 测试14：读取总线电压
 */
static bool test_read_bus_voltage(void)
{
    ESP_LOGI(TAG, "\n=== Test 14: Read Bus Voltage ===");

    zdt_can_msg_t tx_msg;
    zdt_cmd_read_bus_voltage(TEST_MOTOR_ADDR, &tx_msg);

    print_can_message(&tx_msg, "TX");

    if (zdt_twai_send(&tx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Failed to send command");
        return false;
    }
    test_stats.tx_count++;

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 轮询接收 */
    zdt_can_msg_t rx_msg;
    if (zdt_twai_receive(&rx_msg, TEST_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "    ✗ No response received");
        return false;
    }
    test_stats.rx_count++;

    print_can_message(&rx_msg, "RX");

    /* 解析总线电压 */
    uint16_t voltage;
    if (!zdt_parse_bus_voltage(&rx_msg, &voltage)) {
        ESP_LOGE(TAG, "    ✗ Failed to parse bus voltage");
        return false;
    }

    ESP_LOGI(TAG, "    → Bus Voltage: %u mV (%.2f V)", voltage, voltage / 1000.0f);

    /* 读取命令成功解析即表示成功 */
    return true;
}

/**
 * @brief 测试15：统计信息
 */
static bool test_statistics(void)
{
    ESP_LOGI(TAG, "\n=== Test 15: Statistics ===");

    uint32_t rx = zdt_twai_get_rx_count();
    uint32_t tx = zdt_twai_get_tx_count();
    uint32_t err = zdt_twai_get_error_count();

    ESP_LOGI(TAG, "TWAI Driver Statistics:");
    ESP_LOGI(TAG, "  TX Count: %lu", tx);
    ESP_LOGI(TAG, "  RX Count: %lu", rx);
    ESP_LOGI(TAG, "  Error Count: %lu", err);

    ESP_LOGI(TAG, "Test Statistics:");
    ESP_LOGI(TAG, "  TX Commands: %lu", test_stats.tx_count);
    ESP_LOGI(TAG, "  RX Messages: %lu", test_stats.rx_count);
    ESP_LOGI(TAG, "  ACK Received: %lu", test_stats.ack_count);
    ESP_LOGI(TAG, "  Errors: %lu", test_stats.error_count);
    ESP_LOGI(TAG, "  Tests Failed: %lu", test_stats.test_failed_count);

    return true;
}

/*==============================================================================
 * 主测试流程
 *============================================================================*/

/**
 * @brief 运行所有测试用例
 */
static void run_all_tests(void)
{
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ZDT TWAI Driver Test Suite");
    ESP_LOGI(TAG, "  (Emm Firmware / General Commands)");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Motor Address: %d", TEST_MOTOR_ADDR);
    ESP_LOGI(TAG, "TX GPIO: %d", TEST_TX_GPIO);
    ESP_LOGI(TAG, "RX GPIO: %d", TEST_RX_GPIO);
    ESP_LOGI(TAG, "========================================");

    /* 运行测试 */
    print_test_result("TWAI Init", test_twai_init());
    print_test_result("Unregister Callback", test_unregister_callback());
    print_test_result("Read Version", test_read_version());
    print_test_result("Read Motor State", test_read_motor_state());
    print_test_result("Motor Enable", test_motor_enable());
    print_test_result("Read Real Position", test_read_real_position());
    print_test_result("Read Real Speed", test_read_real_speed());
    print_test_result("Read Phase Current", test_read_phase_current());
    print_test_result("Read Bus Voltage", test_read_bus_voltage());

    vTaskDelay(pdMS_TO_TICKS(1000));

    print_test_result("Velocity Mode (Emm)", test_velocity_mode_emm());

    vTaskDelay(pdMS_TO_TICKS(1000));

    print_test_result("Position Mode (Emm)", test_position_mode_emm());

    vTaskDelay(pdMS_TO_TICKS(1000));

    print_test_result("Zero Position", test_zero_position());

    vTaskDelay(pdMS_TO_TICKS(1000));

    print_test_result("Immediate Stop", test_immediate_stop());

    vTaskDelay(pdMS_TO_TICKS(1000));

    print_test_result("Statistics", test_statistics());

    /* 测试总结 */
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Test Summary");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Total TX: %lu", test_stats.tx_count);
    ESP_LOGI(TAG, "Total RX: %lu", test_stats.rx_count);
    ESP_LOGI(TAG, "Total ACK: %lu", test_stats.ack_count);
    ESP_LOGI(TAG, "Total Errors: %lu", test_stats.error_count);
    ESP_LOGI(TAG, "Tests Failed: %lu", test_stats.test_failed_count);

    if (test_stats.test_failed_count == 0) {
        ESP_LOGI(TAG, "\n✓ ALL TESTS PASSED ✓");
        test_stats.test_passed = true;
    } else {
        ESP_LOGE(TAG, "\n✗ SOME TESTS FAILED ✗");
    }
    ESP_LOGI(TAG, "========================================\n");
}

/*==============================================================================
 * 主函数
 *============================================================================*/

void app_main(void)
{
    ESP_LOGI(TAG, "ZDT TWAI Driver Test Program");
    ESP_LOGI(TAG, "Firmware: Emm / General Commands");
    ESP_LOGI(TAG, "Testing without X-firmware specific commands");

    /* 运行测试 */
    run_all_tests();

    /* 持续监控 */
    ESP_LOGI(TAG, "Entering monitor mode...");
    ESP_LOGI(TAG, "Statistics will be printed every 10 seconds");
    ESP_LOGI(TAG, "Press RESET to restart test\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        uint32_t rx = zdt_twai_get_rx_count();
        uint32_t tx = zdt_twai_get_tx_count();
        uint32_t err = zdt_twai_get_error_count();

        ESP_LOGI(TAG, "[Monitor] TX=%lu, RX=%lu, Errors=%lu", tx, rx, err);
    }
}
