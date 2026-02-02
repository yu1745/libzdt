/**
 * @file test_main.c
 * @brief ZDT CAN Protocol ESP-IDF Integration Test (EMM Firmware)
 * 
 * 本测试文件用于 ZDT EMM 固件的物理测试，包括：
 * - CAN 通讯初始化
 * - 位置清零
 * - 使能控制
 * - 位置模式运动
 */

#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "libzdt.h"
#include "unity.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "zdt_test";

/* ============================================================================
 * CAN Hardware Tests (requires actual hardware)
 * ========================================================================== */

#define TEST_POSITION_TOLERANCE 50 // 5.0 degrees tolerance (0.1 deg unit)

#ifdef CONFIG_ZDT_TEST_HARDWARE

// Global handle for setUp/tearDown
static zdt_handle_t g_test_handle = NULL;

// Helper: Initialize CAN for hardware tests
static zdt_error_t hw_test_init(void) {
  if (g_test_handle != NULL) {
    return ZDT_OK; // Already initialized
  }

  zdt_can_config_t config = {.tx_gpio = CONFIG_ZDT_TEST_CAN_TX_GPIO,
                             .rx_gpio = CONFIG_ZDT_TEST_CAN_RX_GPIO,
                             .bitrate = 500000,
                             .tx_timeout_ms = 100,
                             .rx_timeout_ms = 500};

  return zdt_can_init(&config, &g_test_handle);
}

// Helper: Cleanup after hardware tests
static void hw_test_cleanup(void) {
  if (g_test_handle != NULL) {
    // Emergency stop and disable motor
    zdt_emergency_stop(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR);
    vTaskDelay(pdMS_TO_TICKS(50));
    zdt_set_enable(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, false);
    vTaskDelay(pdMS_TO_TICKS(50));

    zdt_can_deinit(g_test_handle);
    g_test_handle = NULL;
  }
}

/* ============================================================================
 * CAN Initialization Tests
 * ========================================================================== */

TEST_CASE("CAN init and deinit", "[hardware][can]") {
  zdt_can_config_t config = {.tx_gpio = CONFIG_ZDT_TEST_CAN_TX_GPIO,
                             .rx_gpio = CONFIG_ZDT_TEST_CAN_RX_GPIO,
                             .bitrate = 500000,
                             .tx_timeout_ms = 100,
                             .rx_timeout_ms = 500};

  zdt_handle_t handle = NULL;
  zdt_error_t err = zdt_can_init(&config, &handle);

  TEST_ASSERT_EQUAL(ZDT_OK, err);
  TEST_ASSERT_NOT_NULL(handle);

  zdt_can_deinit(handle);
}

/* ============================================================================
 * Control Tests (Non-Motion)
 * ========================================================================== */

TEST_CASE("Clear position and verify", "[hardware][control]") {
  zdt_error_t err = hw_test_init();
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  // Clear position
  err = zdt_clear_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  ESP_LOGI(TAG, "Position cleared");

  // Wait for command to take effect
  vTaskDelay(pdMS_TO_TICKS(100));

  // Read and verify position is near zero
  zdt_position_t pos;
  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  ESP_LOGI(TAG, "Position after clear: %.1f deg (raw: %ld)", pos.angle_deg,
           (long)pos.angle_raw);

  // Verify position is within tolerance of zero
  TEST_ASSERT_INT_WITHIN(TEST_POSITION_TOLERANCE, 0, pos.angle_raw);

  hw_test_cleanup();
}

TEST_CASE("Enable/disable motor", "[hardware][control]") {
  zdt_error_t err = hw_test_init();
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  // Enable motor
  ESP_LOGI(TAG, "Enabling motor...");
  err = zdt_set_enable(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, true);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(200));

  ESP_LOGI(TAG, "Motor enabled (locked)");

  // Disable motor
  ESP_LOGI(TAG, "Disabling motor...");
  err = zdt_set_enable(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, false);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(200));

  ESP_LOGI(TAG, "Motor disabled (free)");

  hw_test_cleanup();
}

TEST_CASE("Read position", "[hardware][control]") {
  zdt_error_t err = hw_test_init();
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  // Read current position
  zdt_position_t pos;
  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  ESP_LOGI(TAG, "Current position: %.1f deg (raw: %ld)", pos.angle_deg,
           (long)pos.angle_raw);

  hw_test_cleanup();
}

TEST_CASE("Emergency stop", "[hardware][control]") {
  zdt_error_t err = hw_test_init();
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  // Send emergency stop command
  ESP_LOGI(TAG, "Sending emergency stop...");
  err = zdt_emergency_stop(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR);
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  ESP_LOGI(TAG, "Emergency stop sent successfully");

  hw_test_cleanup();
}

/* ============================================================================
 * Position Mode Tests (Motion)
 * ========================================================================== */

TEST_CASE("Position mode - move and verify", "[hardware][motion]") {
  zdt_error_t err = hw_test_init();
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  // First clear position to establish zero point
  err = zdt_clear_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Enable motor
  err = zdt_set_enable(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, true);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Move to 10000 pulses
  const int32_t target_raw = 10000;

  ESP_LOGI(TAG, "Moving to %d pulses...", target_raw);
  err = zdt_set_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, ZDT_DIR_CW,
                         100.0f, target_raw, 0, false);
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  // Wait for movement to complete (with timeout)
  int timeout_count = 0;
  const int max_timeout = 50; // 5 seconds max
  zdt_position_t pos;

  while (timeout_count < max_timeout) {
    vTaskDelay(pdMS_TO_TICKS(100));
    timeout_count++;

    err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
    if (err == ZDT_OK) {
      ESP_LOGI(TAG, "Current position: %.1f deg", pos.angle_deg);

      // Check if we're close to target
      if (abs(pos.angle_raw - target_raw) < TEST_POSITION_TOLERANCE) {
        break;
      }
    }
  }

  // Final verification
  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  ESP_LOGI(TAG, "Final position: %.1f deg (target: %.1f deg)", pos.angle_deg,
           target_raw);

  TEST_ASSERT_INT_WITHIN(TEST_POSITION_TOLERANCE, target_raw, pos.angle_raw);

  hw_test_cleanup();
}

TEST_CASE("Position mode - move back to zero", "[hardware][motion]") {
  zdt_error_t err = hw_test_init();
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  // Enable motor
  err = zdt_set_enable(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, true);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Read current position
  zdt_position_t pos;
  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  ESP_LOGI(TAG, "Current position: %.1f deg", pos.angle_deg);

  // Move back to zero
  ESP_LOGI(TAG, "Moving back to 0 degrees...");
  zdt_direction_t dir = (pos.angle_raw > 0) ? ZDT_DIR_CCW : ZDT_DIR_CW;
  err = zdt_set_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, dir,
                         100.0f, 0, ZDT_MOTION_ABSOLUTE, false);
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  // Wait for movement to complete
  int timeout_count = 0;
  const int max_timeout = 50; // 5 seconds max

  while (timeout_count < max_timeout) {
    vTaskDelay(pdMS_TO_TICKS(100));
    timeout_count++;

    err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
    if (err == ZDT_OK) {
      ESP_LOGI(TAG, "Current position: %.1f deg", pos.angle_deg);

      if (abs(pos.angle_raw) < TEST_POSITION_TOLERANCE) {
        break;
      }
    }
  }

  // Final verification
  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  ESP_LOGI(TAG, "Final position: %.1f deg (target: 0)", pos.angle_deg);
  TEST_ASSERT_INT_WITHIN(TEST_POSITION_TOLERANCE, 0, pos.angle_raw);

  hw_test_cleanup();
}

TEST_CASE("Position mode - bidirectional movement", "[hardware][motion]") {
  zdt_error_t err = hw_test_init();
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  ESP_LOGI(TAG, "=== Bidirectional Movement Test ===");

  // Clear position
  err = zdt_clear_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Enable motor
  err = zdt_set_enable(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, true);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(100));

  zdt_position_t pos;

  // Test 1: Move CW to +45 degrees
  ESP_LOGI(TAG, "Test 1: Move CW to +45 degrees");
  err = zdt_set_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, ZDT_DIR_CW,
                         150.0f, 450, ZDT_MOTION_ABSOLUTE, false);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(2000));

  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  ESP_LOGI(TAG, "  Position: %.1f deg (target: 45)", pos.angle_deg);
  TEST_ASSERT_INT_WITHIN(TEST_POSITION_TOLERANCE, 450, pos.angle_raw);

  // Test 2: Move CCW to -45 degrees
  ESP_LOGI(TAG, "Test 2: Move CCW to -45 degrees");
  err = zdt_set_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, ZDT_DIR_CCW,
                         150.0f, -450, ZDT_MOTION_ABSOLUTE, false);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(2000));

  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  ESP_LOGI(TAG, "  Position: %.1f deg (target: -45)", pos.angle_deg);
  TEST_ASSERT_INT_WITHIN(TEST_POSITION_TOLERANCE, -450, pos.angle_raw);

  // Test 3: Move back to 0
  ESP_LOGI(TAG, "Test 3: Move back to 0 degrees");
  err = zdt_set_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, ZDT_DIR_CW,
                         150.0f, 0, ZDT_MOTION_ABSOLUTE, false);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(2000));

  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  ESP_LOGI(TAG, "  Position: %.1f deg (target: 0)", pos.angle_deg);
  TEST_ASSERT_INT_WITHIN(TEST_POSITION_TOLERANCE, 0, pos.angle_raw);

  ESP_LOGI(TAG, "=== Bidirectional Movement Test PASSED ===");

  hw_test_cleanup();
}

TEST_CASE("Full motion sequence test", "[hardware][motion]") {
  zdt_error_t err = hw_test_init();
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  ESP_LOGI(TAG, "=== Full Motion Sequence Test ===");

  // Step 1: Clear position
  ESP_LOGI(TAG, "Step 1: Clear position");
  err = zdt_clear_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Verify zero
  zdt_position_t pos;
  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  ESP_LOGI(TAG, "  Position: %.1f deg", pos.angle_deg);
  TEST_ASSERT_INT_WITHIN(TEST_POSITION_TOLERANCE, 0, pos.angle_raw);

  // Step 2: Enable
  ESP_LOGI(TAG, "Step 2: Enable motor");
  err = zdt_set_enable(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, true);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Step 3: Move to 180 degrees
  ESP_LOGI(TAG, "Step 3: Move to 180 deg");
  err = zdt_set_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, ZDT_DIR_CW,
                         200.0f, 1800, ZDT_MOTION_ABSOLUTE, false);
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  // Wait for movement
  vTaskDelay(pdMS_TO_TICKS(2000));

  // Verify position
  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  ESP_LOGI(TAG, "  Position: %.1f deg (target: 180)", pos.angle_deg);
  TEST_ASSERT_INT_WITHIN(TEST_POSITION_TOLERANCE, 1800, pos.angle_raw);

  // Step 4: Move back to 0
  ESP_LOGI(TAG, "Step 4: Move back to 0 deg");
  err = zdt_set_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, ZDT_DIR_CCW,
                         200.0f, 0, ZDT_MOTION_ABSOLUTE, false);
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  // Wait for movement
  vTaskDelay(pdMS_TO_TICKS(2000));

  // Verify position
  err = zdt_read_position(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, &pos);
  TEST_ASSERT_EQUAL(ZDT_OK, err);
  ESP_LOGI(TAG, "  Position: %.1f deg (target: 0)", pos.angle_deg);
  TEST_ASSERT_INT_WITHIN(TEST_POSITION_TOLERANCE, 0, pos.angle_raw);

  // Step 5: Disable
  ESP_LOGI(TAG, "Step 5: Disable motor");
  err = zdt_set_enable(g_test_handle, CONFIG_ZDT_TEST_MOTOR_ADDR, false);
  TEST_ASSERT_EQUAL(ZDT_OK, err);

  ESP_LOGI(TAG, "=== Full Motion Sequence PASSED ===");

  hw_test_cleanup();
}

#endif // CONFIG_ZDT_TEST_HARDWARE

/* ============================================================================
 * Main
 * ========================================================================== */

void app_main(void) {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "ZDT CAN Protocol ESP-IDF Integration Test");
  ESP_LOGI(TAG, "EMM Firmware - Hardware Tests Only");
  ESP_LOGI(TAG, "========================================");

#ifdef CONFIG_ZDT_TEST_HARDWARE
  ESP_LOGI(TAG, "Running hardware tests...");
  ESP_LOGI(TAG, "Motor address: %d", CONFIG_ZDT_TEST_MOTOR_ADDR);
  ESP_LOGI(TAG, "CAN TX GPIO: %d", CONFIG_ZDT_TEST_CAN_TX_GPIO);
  ESP_LOGI(TAG, "CAN RX GPIO: %d", CONFIG_ZDT_TEST_CAN_RX_GPIO);
#else
  ESP_LOGW(TAG, "Hardware tests disabled!");
  ESP_LOGW(TAG, "Enable CONFIG_ZDT_TEST_HARDWARE to run tests");
#endif

  UNITY_BEGIN();
  unity_run_all_tests();
  UNITY_END();

  ESP_LOGI(TAG, "Test completed!");
}
