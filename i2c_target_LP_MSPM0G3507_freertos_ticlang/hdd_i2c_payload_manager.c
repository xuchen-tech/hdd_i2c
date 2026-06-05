#include "hdd_i2c_payload_manager.h"

#include "hdd_i2c_utils.h"
#include "i2c_controller.h"
#include "i2ctargetApp.h"

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
static TaskHandle_t g_i2cWorkerTaskHandle = NULL;

#define PAYLOAD_UPDATE_PERIOD_MS 100
#define HDD_CHILD_ABSENT_BACKOFF_CYCLES 5u
#define HDD_CHILD_SETTLE_CYCLES 3u

typedef struct {
    bool valid;
    uint32_t generation;
    uint32_t pressureValue;
    uint8_t childLen;
    uint8_t childData[BUFFER_SIZE];
} RemoteSampleSnapshot;

static volatile RemoteSampleSnapshot g_remoteSample = {0};

static void i2cWorkerTask(void *arg0);
static void publishRemoteSample(uint32_t pressureValue,
                                const uint8_t *childData,
                                uint8_t childLen);
static bool copyLatestRemoteSample(uint32_t *pressureValue,
                                   uint8_t *childData,
                                   uint8_t *childLen,
                                   uint32_t *generation);
static bool reinitNsa2300WithRetry(uint8_t maxAttempts,
                                   const char *successFmt,
                                   const char *failFmt);

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
    if (g_payloadManagerTaskHandle == NULL && g_i2cWorkerTaskHandle == NULL) {
        return;
    }

    BaseType_t hpw = pdFALSE;
    if (g_i2cWorkerTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(g_i2cWorkerTaskHandle, &hpw);
    }
    if (g_payloadManagerTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(g_payloadManagerTaskHandle, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

static void publishRemoteSample(uint32_t pressureValue,
                                const uint8_t *childData,
                                uint8_t childLen)
{
    if (childLen > BUFFER_SIZE) {
        childLen = BUFFER_SIZE;
    }

    taskENTER_CRITICAL();
    g_remoteSample.pressureValue = pressureValue;
    g_remoteSample.childLen = childLen;
    if (childLen > 0u && childData != NULL) {
        memcpy((void *)g_remoteSample.childData, childData, childLen);
    }
    g_remoteSample.valid = true;
    g_remoteSample.generation++;
    taskEXIT_CRITICAL();
}

static bool copyLatestRemoteSample(uint32_t *pressureValue,
                                   uint8_t *childData,
                                   uint8_t *childLen,
                                   uint32_t *generation)
{
    bool valid;

    if (pressureValue == NULL || childLen == NULL || generation == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    valid = g_remoteSample.valid;
    if (valid) {
        *pressureValue = g_remoteSample.pressureValue;
        *childLen = g_remoteSample.childLen;
        *generation = g_remoteSample.generation;
        if ((*childLen > 0u) && (childData != NULL)) {
            memcpy(childData, (const void *)g_remoteSample.childData, *childLen);
        }
    }
    taskEXIT_CRITICAL();

    if (!valid) {
        *childLen = 0u;
        *generation = 0u;
    }

    return valid;
}

static bool reinitNsa2300WithRetry(uint8_t maxAttempts,
                                   const char *successFmt,
                                   const char *failFmt)
{
    for (uint8_t attempt = 0; attempt < maxAttempts; attempt++) {
        if (nsa2300Init()) {
            if (successFmt != NULL) {
                SEGGER_RTT_printf(0, successFmt, (unsigned)(attempt + 1u));
            }
            return true;
        }
        if (failFmt != NULL) {
            SEGGER_RTT_printf(0, failFmt, (unsigned)(attempt + 1u));
        }
        usleep(200000);
    }

    return false;
}

static void i2cWorkerTask(void *arg0) {
    uint8_t mode = 0u;
    uint8_t ready = 0u;
    uint8_t childData[BUFFER_SIZE] = {0};
    uint8_t nas2300_init_attempts = 10u;
    uint8_t nsa2300ConsecFail = 0u;
    bool childTargetPresent = false;
    uint8_t childProbeBackoff = 0u;
    uint8_t childSettleCycles = 0u;
    uint32_t pressureValue = 0u;
    (void)arg0;

    g_i2cWorkerTaskHandle = xTaskGetCurrentTaskHandle();

    if (!reinitNsa2300WithRetry(nas2300_init_attempts,
                                "I2C worker: NSA2300 initialized on attempt %u\n",
                                "I2C worker: nsa2300Init attempt %u failed\n")) {
        SEGGER_RTT_printf(0, "I2C worker: NSA2300 init exhausted retries\n");
    }

    while (1) {
        uint8_t childLen = 0u;

        if (I2CTarget_isNsa2300ReinitRequired()) {
            I2CTarget_clearNsa2300ReinitRequired();
            SEGGER_RTT_printf(0, "I2C worker: NSA2300 reinit requested by REG_CTRL\n");
            nsa2300ConsecFail = 0u;
            nas2300Deinit();
            usleep(50000);
            reinitNsa2300WithRetry(nas2300_init_attempts,
                                   "I2C worker: NSA2300 reinit OK on attempt %u\n",
                                   "I2C worker: NSA2300 reinit attempt %u failed\n");
        }

        SEGGER_RTT_printf(0, "I2C worker: before nsa2300StartMeasurement\n");
        if (!nsa2300StartMeasurement()) {
            SEGGER_RTT_printf(0, "I2C worker: nsa2300StartMeasurement FAILED\n");
            nsa2300ConsecFail++;
            if (nsa2300ConsecFail >= 3u) {
                SEGGER_RTT_printf(0,
                                  "I2C worker: NSA2300 failed %u consecutive times; reinitializing I2C and sensor\n",
                                  (unsigned)nsa2300ConsecFail);
                nsa2300ConsecFail = 0u;
                nas2300Deinit();
                usleep(50000);
                reinitNsa2300WithRetry(nas2300_init_attempts,
                                       "I2C worker: NSA2300 reinitialized on attempt %u\n",
                                       "I2C worker: NSA2300 reinit attempt %u failed\n");
            }
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
            continue;
        }

        SEGGER_RTT_printf(0, "I2C worker: before nsa2300WaitForDataReady\n");
        if (!nsa2300WaitForDataReady()) {
            SEGGER_RTT_printf(0, "I2C worker: nsa2300WaitForDataReady FAILED\n");
            nsa2300ConsecFail++;
            if (nsa2300ConsecFail >= 3u) {
                SEGGER_RTT_printf(0,
                                  "I2C worker: NSA2300 failed %u consecutive times; reinitializing I2C and sensor\n",
                                  (unsigned)nsa2300ConsecFail);
                nsa2300ConsecFail = 0u;
                nas2300Deinit();
                usleep(50000);
                reinitNsa2300WithRetry(nas2300_init_attempts,
                                       "I2C worker: NSA2300 reinitialized on attempt %u\n",
                                       "I2C worker: NSA2300 reinit attempt %u failed\n");
            }
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
            continue;
        }

        if (nsa2300ReadPressureOutputSingle(&pressureValue)) {
            SEGGER_RTT_printf(0, "I2C worker: nsa2300ReadPressureOutputSingle OK: %u, 0x:%x\n",
                              pressureValue, pressureValue);
            nsa2300ConsecFail = 0u;
        } else {
            SEGGER_RTT_printf(0, "I2C worker: nsa2300ReadPressureOutputSingle FAILED\n");
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
            continue;
        }

        if (childProbeBackoff > 0u) {
            childProbeBackoff--;
        } else if (hddI2CReadMode(&mode)) {
            SEGGER_RTT_printf(0, "I2C worker: HDD I2C Mode: 0x%02x\n", (unsigned)mode);
            if (!childTargetPresent) {
                childTargetPresent = true;
                childSettleCycles = HDD_CHILD_SETTLE_CYCLES;
                SEGGER_RTT_printf(0,
                                  "I2C worker: HDD target detected, waiting %u cycles before full access\n",
                                  (unsigned)childSettleCycles);
            }

            if (childSettleCycles > 0u) {
                childSettleCycles--;
            } else if (hddI2CWriteMode(HDD_I2C_MODE_D1)) {
                SEGGER_RTT_printf(0, "I2C worker: HDD I2C Write Mode D1 succeeded\n");
                for (uint8_t index = 0; index < 100u; index++) {
                    if (!hddI2CReadReady(&ready)) {
                        SEGGER_RTT_printf(0, "I2C worker: HDD I2C Read Ready failed\n");
                        continue;
                    }
                    SEGGER_RTT_printf(0, "I2C worker: HDD I2C Ready: 0x%02x\n", (unsigned)ready);
                    if (ready == 0u) {
                        usleep(1000);
                        continue;
                    }

                    childLen = ready;
                    if (childLen > BUFFER_SIZE) {
                        SEGGER_RTT_printf(0,
                                          "I2C worker: HDD child length %u invalid for local buffer %u\n",
                                          (unsigned)childLen,
                                          (unsigned)BUFFER_SIZE);
                        childLen = 0u;
                        break;
                    }

                    if (hddI2CReadData(childData, childLen)) {
                        SEGGER_RTT_printf(0, "I2C worker: HDD I2C Read Data of %u bytes succeeded\n",
                                          (unsigned)childLen);
                        if (hddI2CWriteReady(0u)) {
                            SEGGER_RTT_printf(0, "I2C worker: HDD I2C Write Ready 0 succeeded\n");
                        } else {
                            SEGGER_RTT_printf(0, "I2C worker: HDD I2C Write Ready 0 failed\n");
                        }
                    } else {
                        SEGGER_RTT_printf(0, "I2C worker: HDD I2C Read Data failed\n");
                        childLen = 0u;
                    }
                    break;
                }
            } else {
                SEGGER_RTT_printf(0, "I2C worker: HDD I2C Write Mode D1 failed\n");
            }
        } else {
            if (childTargetPresent) {
                SEGGER_RTT_printf(0,
                                  "I2C worker: HDD target removed or not ready; falling back to local payload only\n");
            }
            childTargetPresent = false;
            childSettleCycles = 0u;
            childProbeBackoff = HDD_CHILD_ABSENT_BACKOFF_CYCLES;
            SEGGER_RTT_printf(0, "I2C worker: HDD I2C Read Mode failed\n");
        }

        publishRemoteSample(pressureValue, childData, childLen);
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
    }
}

/* ---- Rolling average state (shared pressure + temperature ring buffer) ---- */
#define AVG_BUF_MAX 32u
static uint32_t g_pressureAvgBuf[AVG_BUF_MAX];
static int32_t  g_tempAvgBuf[AVG_BUF_MAX];
static uint8_t  g_avgBufHead  = 0u; /* next write index */
static uint8_t  g_avgBufCount = 0u; /* valid entries (0..AVG_BUF_MAX) */

/* Decode REG_CTRL bits[4:3] → sample count (8 / 16 / 32) */
static uint8_t regCtrlAvgSamples(uint8_t ctrl) {
    switch ((ctrl >> 3u) & 0x03u) {
        case 0u: return 8u;
        case 2u: return 32u;
        default: return 16u; /* 0x01 default and 0x03 reserved → 16 */
    }
}

void *payloadManagerThread(void *arg0) {
    bool ret;
    int16_t pt100Raw;
    uint8_t dataBuffer[BUFFER_SIZE] = {0};
    uint8_t childData[BUFFER_SIZE] = {0};
    uint8_t childLen = 0u;
    uint32_t remoteGeneration = 0u;
    uint32_t lastRemoteGeneration = 0u;
    uint32_t pressureValue = 0u;
    (void)arg0;

    /* Capture task handle so ISR-side code can notify us */
    g_payloadManagerTaskHandle = xTaskGetCurrentTaskHandle();

    memset(dataBuffer, 0, sizeof(dataBuffer));
    ret = pt1000Init();
    if (ret == false) {
        SEGGER_RTT_printf(0, "Payload Manager thread: pt1000Init failed\n");
        return NULL;
    }

    if (xTaskCreate(i2cWorkerTask, "I2C1Wkr", 768, NULL, 2,
                    &g_i2cWorkerTaskHandle) != pdPASS) {
        SEGGER_RTT_printf(0, "Payload Manager thread: failed to create I2C worker task\n");
        return NULL;
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

        if (!copyLatestRemoteSample(&pressureValue, childData, &childLen,
                                    &remoteGeneration)) {
            SEGGER_RTT_printf(0, "PM: no remote I2C1 sample yet; keeping previous payload\n");
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
            continue;
        }

        if (remoteGeneration != lastRemoteGeneration) {
            lastRemoteGeneration = remoteGeneration;
            /* Update rolling average ring buffer only on fresh remote samples. */
            g_pressureAvgBuf[g_avgBufHead] = pressureValue;
            g_tempAvgBuf[g_avgBufHead]     = (int32_t)pt100Raw;
            g_avgBufHead = (uint8_t)((g_avgBufHead + 1u) % AVG_BUF_MAX);
            if (g_avgBufCount < AVG_BUF_MAX) { g_avgBufCount++; }
        }

        /* Apply rolling average when REG_CTRL bit 2 is set */
        {
            uint8_t regCtrl = I2CTarget_getRegCtrl();
            if (regCtrl & REG_CTRL_BIT2_AVERAGING) {
                uint8_t n = regCtrlAvgSamples(regCtrl);
                if (n > g_avgBufCount) { n = g_avgBufCount; }
                if (n == 0u) { n = 1u; }
                uint32_t pressureSum = 0u;
                int32_t  tempSum     = 0;
                for (uint8_t k = 0u; k < n; k++) {
                    uint8_t idx = (uint8_t)((g_avgBufHead + AVG_BUF_MAX - 1u - k) % AVG_BUF_MAX);
                    pressureSum += g_pressureAvgBuf[idx];
                    tempSum     += g_tempAvgBuf[idx];
                }
                pressureValue = pressureSum / (uint32_t)n;
                pt100Raw      = (int16_t)(tempSum / (int32_t)n);
                SEGGER_RTT_printf(0, "PM: rolling avg (%u samples): pressure=%lu temp_x10=%d\n",
                                  (unsigned)n, (unsigned long)pressureValue, (int)pt100Raw);
            }
        }

        const uint8_t payloadLen = buildDataPayload(dataBuffer, sizeof(dataBuffer), pressureValue, (uint16_t)pt100Raw);
        if (payloadLen == 0u) {
            SEGGER_RTT_printf(0, "Payload Manager: buildDataPayload failed\n");
            continue;
        }
        readyToPublish = payloadLen;
        SEGGER_RTT_printf(0, "Payload Manager: built payload of %u bytes\n", (unsigned)payloadLen);

        if (childLen > 0u) {
            const uint8_t maxChildLen = (uint8_t)(sizeof(dataBuffer) - payloadLen);
            if (childLen > maxChildLen) {
                SEGGER_RTT_printf(0,
                                  "Payload Manager: child data length %u exceeds local buffer limit %u\n",
                                  (unsigned)childLen,
                                  (unsigned)maxChildLen);
            } else {
                memcpy(dataBuffer + payloadLen, childData, childLen);
                readyToPublish = (uint8_t)(payloadLen + childLen);
            }
        }

        /* Publish latest completed payload and then expose true size via 0x81 */
        updatePayloadData(dataBuffer, readyToPublish);
        // print all data in payload for debug
        SEGGER_RTT_printf(0, "Published Payload Data (%u bytes):", (unsigned)readyToPublish);
        for (uint8_t i = 0; i < readyToPublish; i++) {
            SEGGER_RTT_printf(0, " %02x", dataBuffer[i]);
        }
        SEGGER_RTT_printf(0, "\n");
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
    }
}