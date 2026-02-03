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
    uint8_t mode = 0, ready = 0;

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

        ret = pt100ReadTemperature_x10(&pt100Raw);
        if (ret == true) {
            SEGGER_RTT_printf(0, "PT100 temp: %d\n", (uint16_t)pt100Raw);
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
        if (hddI2CReadMode(&mode)) {
            SEGGER_RTT_printf(0, "HDD I2C Mode: 0x%02x\n", (unsigned)mode);
            if (hddI2CWriteMode(HDD_I2C_MODE_D1)) {
                SEGGER_RTT_printf(0, "HDD I2C Write Mode D1 succeeded\n");
                for (uint8_t i = 0; i < 100; i ++) {
                    if (hddI2CReadReady(&ready)) {
                        SEGGER_RTT_printf(0, "HDD I2C Ready: 0x%02x\n", (unsigned)ready);
                        if (ready != 0) {
                            SEGGER_RTT_printf(0, "HDD I2C Read Ready indicates data ready\n");
                            if (hddI2CReadData(dataBuffer + payloadLen, ready)) {
                                SEGGER_RTT_printf(0, "HDD I2C Read Data of %u bytes succeeded\n", (unsigned)ready);
                                updatePayloadData(dataBuffer, payloadLen + ready);
                                updateReady(payloadLen + ready);
                                if (hddI2CWriteReady(0)) {
                                    SEGGER_RTT_printf(0, "HDD I2C Write Ready 0 succeeded\n");
                                } else {
                                    SEGGER_RTT_printf(0, "HDD I2C Write Ready 0 failed\n");
                                }
                            } else {
                                SEGGER_RTT_printf(0, "HDD I2C Read Data failed\n");
                            }
                            break;
                        } else {
                            SEGGER_RTT_printf(0, "HDD I2C Read Ready indicates data NOT ready; retrying...\n");
                            usleep(1000); /* 10ms */
                            continue;
                        }
                    } else {
                        SEGGER_RTT_printf(0, "HDD I2C Read Ready failed\n");
                    }
                }
            } else {
                SEGGER_RTT_printf(0, "HDD I2C Write Mode D1 failed\n");
            }
        } else {
            SEGGER_RTT_printf(0, "HDD I2C Read Mode failed\n"); // this indicate has no conntectted device
            updatePayloadData(dataBuffer, payloadLen);
            updateReady(payloadLen);
        }
    }
}