#include "hdd_i2c_payload_manager.h"

#include "hdd_i2c_utils.h"
#include "i2c_controller.h"

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

#define PAYLOAD_UPDATE_PERIOD_MS 100

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

    SEGGER_RTT_printf(0, "Built Payload: P24=0x%06x, PT100Raw=0x%04x, CRC16=0x%04x\n",
                      (unsigned)p24, (unsigned)pt100Raw, (unsigned)crc);

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
    uint8_t nas2300_init_attempts = 10;
    (void)arg0;

    /* Capture task handle so ISR-side code can notify us */
    g_payloadManagerTaskHandle = xTaskGetCurrentTaskHandle();

    memset(dataBuffer, 0, sizeof(dataBuffer));
    ret = pt1000Init();
    if (ret == false) {
        SEGGER_RTT_printf(0, "Payload Manager thread: pt1000Init failed\n");
        return NULL;
    }
    for (uint8_t i = 0; i < nas2300_init_attempts; i++) {
        ret = nsa2300Init();
        if (ret == true) {
            break;
        }
        SEGGER_RTT_printf(0, "Payload Manager thread: nsa2300Init attempt %u failed\n", (unsigned)(i + 1));
        usleep(200000);
    }

    while (1) {
        uint8_t readyToPublish = 0u;

        SEGGER_RTT_printf(0, "PM: before pt1000ReadTemperature_x10\n");
        ret = pt1000ReadTemperature_x10(&pt100Raw);
        if (ret == true) {
            SEGGER_RTT_printf(0, "PM: pt1000ReadTemperature_x10 OK, temp=%u\n", (uint16_t)pt100Raw);
        } else {
            SEGGER_RTT_printf(0, "PM: pt1000ReadTemperature_x10 FAILED\n");
            pt100Raw = 0;
        }

        /* I2C target service is on I2C0 while NSA2300 controller access is on
         * I2C1, so there is no need to wait for the target-side IRQ counter to
         * become idle before starting a controller-side measurement.
         * Pause target service only for the short controller transaction window.
         */
        SEGGER_RTT_printf(0, "PM: pausing I2C target service before NSA2300 start\n");

        SEGGER_RTT_printf(0, "PM: before nsa2300StartMeasurement\n");
        if (!nsa2300StartMeasurement()) {
            SEGGER_RTT_printf(0, "PM: nsa2300StartMeasurement FAILED\n");
            continue;
        }
        SEGGER_RTT_printf(0, "PM: before nsa2300WaitForDataReady\n");
        if (!nsa2300WaitForDataReady()) {
            SEGGER_RTT_printf(0, "PM: nsa2300WaitForDataReady FAILED\n");
            continue;
        }

        uint32_t pressureRaw24;
        SEGGER_RTT_printf(0, "PM: before nsa2300ReadPressureRaw24Single\n");
        if (nsa2300ReadPressureRaw24Single(&pressureRaw24)) {
            SEGGER_RTT_printf(0, "PM: nsa2300ReadPressureRaw24Single OK: %u, 0x:%x\n", pressureRaw24, pressureRaw24);
        } else {
            SEGGER_RTT_printf(0, "PM: nsa2300ReadPressureRaw24Single FAILED\n");
            continue;
        }

        const uint8_t payloadLen = buildDataPayload(dataBuffer, sizeof(dataBuffer), pressureRaw24, pt100Raw);
        if (payloadLen == 0u) {
            SEGGER_RTT_printf(0, "Payload Manager: buildDataPayload failed\n");
            continue;
        }
        readyToPublish = payloadLen;
        SEGGER_RTT_printf(0, "Payload Manager: built payload of %u bytes\n", (unsigned)payloadLen);

        if (hddI2CReadMode(&mode)) {
            SEGGER_RTT_printf(0, "HDD I2C Mode: 0x%02x\n", (unsigned)mode);
            if (hddI2CWriteMode(HDD_I2C_MODE_D1)) {
                SEGGER_RTT_printf(0, "HDD I2C Write Mode D1 succeeded\n");
                for (uint8_t i = 0; i < 100; i ++) {
                    if (hddI2CReadReady(&ready)) {
                        SEGGER_RTT_printf(0, "HDD I2C Ready: 0x%02x\n", (unsigned)ready);
                        if (ready != 0) {
                            uint8_t childLen = ready;
                            const uint8_t maxChildLen = (uint8_t)(sizeof(dataBuffer) - payloadLen);

                            if (childLen > maxChildLen) {
                                SEGGER_RTT_printf(0,
                                                  "HDD I2C Ready length %u invalid for local buffer limit %u; skipping child read this cycle\n",
                                                  (unsigned)ready,
                                                  (unsigned)maxChildLen);
                                usleep(1000);
                                continue;
                            }

                            SEGGER_RTT_printf(0, "HDD I2C Read Ready indicates data ready\n");
                            if (hddI2CReadData(dataBuffer + payloadLen, childLen)) {
                                SEGGER_RTT_printf(0, "HDD I2C Read Data of %u bytes succeeded\n", (unsigned)childLen);
                                readyToPublish = (uint8_t)(payloadLen + childLen);
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
                            usleep(1000);
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
            readyToPublish = payloadLen;
        }

        /* Publish latest completed payload and then expose true size via 0x81 */
        updatePayloadData(dataBuffer, readyToPublish);
        // print all data in payload for debug
        SEGGER_RTT_printf(0, "Published Payload Data (%u bytes):", (unsigned)readyToPublish);
        for (uint8_t i = 0; i < readyToPublish; i++) {
            SEGGER_RTT_printf(0, " %02x", dataBuffer[i]);
        }
        SEGGER_RTT_printf(0, "\n");
        usleep(100000);
    }
}