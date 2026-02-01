#include "hdd_i2c_payload_manager.h"

void *payloadManagerThread(void *arg0) {
    while (1) {
        sleep(1);
        SEGGER_RTT_printf(0, "Payload Manager Thread alive\n");
    }
}