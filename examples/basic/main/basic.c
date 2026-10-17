/* 最小用法：使能 1 号电机，转一圈（Emm 固件，3200 脉冲/圈），读回实时位置。 */
#include <stdio.h>

#include "esp_err.h"
#include "libzdt.h"

#define MOTOR_ADDR 0x01

void app_main(void)
{
    zdt_twai_handle_t bus;
    zdt_twai_config_t cfg = {
        .tx_io = GPIO_NUM_4,
        .rx_io = GPIO_NUM_5,
        .bitrate = 500000,
    };
    ESP_ERROR_CHECK(zdt_twai_new(&cfg, &bus));

    zdt_can_msg_t msg;
    zdt_reply_t reply;

    zdtCanBuildMotorEnableCmd(MOTOR_ADDR, 1, ZDT_SYNC_NOW, &msg);
    ESP_ERROR_CHECK(zdt_twai_transact(bus, &msg, 2, &reply, 500));
    printf("enable: %s\n", zdt_reply_is_ok(&reply) ? "ok" : "rejected");

    zdtCanBuildPosModeEmmCmd(MOTOR_ADDR, ZDT_DIR_CW, 300, 10, 3200, ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW, &msg);
    ESP_ERROR_CHECK(zdt_twai_transact(bus, &msg, 2, &reply, 500));
    printf("move: %s\n", zdt_reply_is_ok(&reply) ? "ok" : "rejected");

    /* 读实时位置：应答为 符号(1) + 位置(4, 大端, 65536 = 360°) + 校验 */
    zdtCanBuildReadRealtimePosCmd(MOTOR_ADDR, &msg);
    ESP_ERROR_CHECK(zdt_twai_transact(bus, &msg, 6, &reply, 500));
    uint32_t counts = ((uint32_t)reply.data[1] << 24) | ((uint32_t)reply.data[2] << 16) |
                      ((uint32_t)reply.data[3] << 8) | reply.data[4];
    printf("position: %s%.2f deg\n", reply.data[0] ? "-" : "", counts * 360.0 / 65536.0);

    zdt_twai_del(bus);
}
