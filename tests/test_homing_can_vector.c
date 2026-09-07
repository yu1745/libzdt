/* Host-only regression test for manual 5.4.6 / CAN 4.2.1 framing. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "libzdt.h"

int main(void)
{
    static const uint8_t expected_data[][8] = {
        {0x4C, 0xAE, 0x01, 0x00, 0x00, 0x00, 0x1E, 0x00},
        {0x00, 0x27, 0x10, 0x01, 0x2C, 0x03, 0x20, 0x00},
        {0x3C, 0x00, 0x6B},
    };
    static const uint8_t expected_dlc[] = {8, 8, 3};
    zdt_can_msg_t msg = {0};

    int count = zdtCanBuildWriteHomingParamsCmd(
        0x01, ZDT_STORE_YES, 0x00, ZDT_DIR_CW, 30, 10000,
        300, 800, 60, 0x00, &msg);
    if (count != 3 || msg.frame_count != 3) {
        fprintf(stderr, "expected three CAN frames, got result=%d count=%u\n",
                count, msg.frame_count);
        return 1;
    }

    for (uint8_t packet = 0; packet < msg.frame_count; ++packet) {
        const zdt_can_frame_t *frame = &msg.frames[packet];
        if (frame->id != (0x100U | packet) ||
            frame->dlc != expected_dlc[packet] ||
            memcmp(frame->data, expected_data[packet], frame->dlc) != 0) {
            fprintf(stderr, "5.4.6 vector mismatch in packet %u\n", packet);
            return 1;
        }
    }
    return 0;
}
