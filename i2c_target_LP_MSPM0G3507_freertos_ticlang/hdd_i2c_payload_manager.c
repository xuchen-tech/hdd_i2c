#include "hdd_i2c_payload_manager.h"

void *payloadManagerThread(void *arg0) {
    bool ret;
    uint16_t pt100Raw;

    (void)arg0;

    ret = pt100Init();
    if (ret == false) {
        SEGGER_RTT_printf(0, "Payload Manager thread: pt100Init failed\n");
        return NULL;
    }
    ret = nsa2300Init();
    if (ret == false) {
        SEGGER_RTT_printf(0, "Payload Manager thread: nas2300Init failed\n");
        return NULL;
    }
    while (1) {
        sleep(1);
        SEGGER_RTT_printf(0, "Payload Manager thread: running...\n");
        ret = pt100ReadRaw(&pt100Raw);
        if (ret == true) {
            SEGGER_RTT_printf(0, "PT100 Raw Data: %u\n", (unsigned)pt100Raw);
        } else {
            SEGGER_RTT_printf(0, "PT100 Read Raw failed\n");
        }
        if (!nsa2300StartMeasurement()) {
            SEGGER_RTT_printf(0, "NSA2300 Start Measurement failed\n");
            continue;
        }
        if (nsa2300WaitForDataReady()) {
            SEGGER_RTT_printf(0, "NSA2300 Wait For Data Ready failed\n");
            continue;
        }
        uint32_t pressureRaw24;
        if (nsa2300ReadPressureRaw24Single(&pressureRaw24)) {
            SEGGER_RTT_printf(0, "NSA2300 Pressure Raw 24-bit: %u, 0x:%x\n", pressureRaw24, pressureRaw24);
        } else {
            SEGGER_RTT_printf(0, "NSA2300 Read Pressure Raw 24-bit failed\n");
        }
    }
}