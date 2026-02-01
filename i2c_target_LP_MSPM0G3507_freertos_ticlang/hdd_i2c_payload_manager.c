#include "hdd_i2c_payload_manager.h"

#include "hdd_i2c_utils.h"

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>

/* For sleep() */
#include <unistd.h>

/* For memset() */
#include <string.h>

/* For SEGGER_RTT_printf() */
#include <ti/segger/SEGGER_RTT.h>

static TaskHandle_t g_payloadManagerTaskHandle = NULL;

static uint8_t buildDataPayload(uint8_t *buf, size_t bufSize,
                                uint32_t pressureRaw24, uint16_t pt100Raw)
{
    if (buf == NULL || bufSize < 8u) {
        return 0;
    }

    /* Layout (8 bytes total), all little-endian:
     * 0..3: pressure (uint32), but only low 24 bits valid; high byte padded 0
     * 4..5: pt100Raw (uint16)
     * 6..7: CRC16(Modbus) over bytes 0..5
     */
    const uint32_t p24 = (pressureRaw24 & 0x00FFFFFFu);

    buf[0] = (uint8_t)(p24 & 0xFFu);
    buf[1] = (uint8_t)((p24 >> 8) & 0xFFu);
    buf[2] = (uint8_t)((p24 >> 16) & 0xFFu);
    buf[3] = 0x00u;

    buf[4] = (uint8_t)(pt100Raw & 0xFFu);
    buf[5] = (uint8_t)((pt100Raw >> 8) & 0xFFu);

    const uint16_t crc = crc16_modbus(buf, 6u);
    buf[6] = (uint8_t)(crc & 0xFFu);
    buf[7] = (uint8_t)((crc >> 8) & 0xFFu);

    return 8u;
}

void PayloadManager_requestSampleFromISR(void) {
    if (g_payloadManagerTaskHandle == NULL) {
        return;
    }

    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(g_payloadManagerTaskHandle, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void *payloadManagerThread(void *arg0) {
    bool ret;
    uint16_t pt100Raw;
    uint8_t dataBuffer[BUFFER_SIZE] = {0};

    (void)arg0;

    /* Capture task handle so ISR-side code can notify us */
    g_payloadManagerTaskHandle = xTaskGetCurrentTaskHandle();

    memset(dataBuffer, 0, sizeof(dataBuffer));
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
        /* Wait for a request from the I2C ISR / protocol handler */
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        SEGGER_RTT_printf(0, "Payload Manager: notified, sampling...\n");

        ret = pt100ReadRaw(&pt100Raw);
        if (ret == true) {
            SEGGER_RTT_printf(0, "PT100 Raw Data: %u\n", (unsigned)pt100Raw);
        } else {
            SEGGER_RTT_printf(0, "PT100 Read Raw failed\n");
            pt100Raw = 0;
        }

        if (!nsa2300StartMeasurement()) {
            SEGGER_RTT_printf(0, "NSA2300 Start Measurement failed\n");
            continue;
        }
        if (!nsa2300WaitForDataReady()) {
            SEGGER_RTT_printf(0, "NSA2300 Wait For Data Ready failed\n");
            continue;
        }

        uint32_t pressureRaw24;
        if (nsa2300ReadPressureRaw24Single(&pressureRaw24)) {
            SEGGER_RTT_printf(0, "NSA2300 Pressure Raw 24-bit: %u, 0x:%x\n", pressureRaw24, pressureRaw24);
        } else {
            SEGGER_RTT_printf(0, "NSA2300 Read Pressure Raw 24-bit failed\n");
            continue;
        }

        const uint8_t payloadLen = buildDataPayload(dataBuffer, sizeof(dataBuffer), pressureRaw24, pt100Raw);
        if (payloadLen == 0u) {
            SEGGER_RTT_printf(0, "Payload Manager: buildDataPayload failed\n");
            continue;
        }
        SEGGER_RTT_printf(0, "Payload Manager: built payload of %u bytes\n", (unsigned)payloadLen);
        updatePayloadData(dataBuffer, payloadLen);
        updateReady(payloadLen);
    }
}