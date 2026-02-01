/*
 *  ======== i2ccontroller.c ========
 */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
/* For sleep() */
#include <unistd.h>
/* For memcpy() */
#include <string.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>

/* Driver Header files */

#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/segger/SEGGER_RTT.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include "hdd_i2c_config.h"
#include "nas2300.h"
#define BUFFER_SIZE 32
uint8_t txBuffer[BUFFER_SIZE];
uint8_t rxBuffer[BUFFER_SIZE];

/* Number of supported sensor iterations */
#define TMP_COUNT 1

static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[TMP_COUNT] = {{NAS2300_I2C_ADDRESS, 0x01, "NSA2300"}};

static uint8_t targetAddress;

static TaskHandle_t g_nsa2300TaskHandle = NULL;

TaskHandle_t NSA2300_getTaskHandle(void)
{
    return g_nsa2300TaskHandle;
}

static void i2cErrorHandler(
    I2C_Transaction *transaction);

static bool nas2300Init(I2C_Handle i2cHandle, I2C_Transaction *transaction, NAS2300_Config *cfg);

/* Empirically helps with occasional "all-zero" boots (sensor power-up / bus settle) */
#define NSA2300_POWERUP_DELAY_US  (100000u) /* 100ms */
#define NSA2300_INIT_RETRIES      (20u)
#define NSA2300_INIT_RETRY_US     (50000u)  /* 50ms */
#define NSA2300_SAMPLE_RETRIES    (3u)

/*
 *  ======== i2cControllerThread ========
 */
void *i2cControllerThread(void *arg0) {
    int8_t i;
    I2C_Handle i2cHandle;
    I2C_Params i2cParams;
    I2C_Transaction i2cTransaction;
    NAS2300_Config nasCfg;

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cHandle         = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2cHandle == NULL) {
        SEGGER_RTT_printf(0, "Error Initializing I2C\n");
        while (1) {
        }
    } else {
        SEGGER_RTT_printf(0, "I2C Initialized!\n");
    }

    SEGGER_RTT_printf(0,  "Starting the i2ccontroller example\n");

    /* Capture task handle so other code can notify us */
    g_nsa2300TaskHandle = xTaskGetCurrentTaskHandle();


    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 0;

    /*
     * Determine which I2C sensors are present by querying known I2C
     * target addresses.
     */
    for (i = TMP_COUNT - 1; i >= 0; i--) {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0]                  = sensors[i].resultReg;

        if (I2C_transfer(i2cHandle, &i2cTransaction)) {
            targetAddress = sensors[i].address;
            SEGGER_RTT_printf(0,
                "Detected TMP%s sensor with target"
                " address 0x%x\n",
                sensors[i].id, sensors[i].address);
        } else {
            i2cErrorHandler(&i2cTransaction);
        }
    }

    /* If we never assigned a target address */
    if (targetAddress == 0) {
        SEGGER_RTT_printf(0, "Failed to detect a sensor!\n");
        I2C_close(i2cHandle);
        while (1) {
        }
    }

    SEGGER_RTT_printf(0, "\nUsing last known sensor for samples.");
    i2cTransaction.targetAddress = targetAddress;

    NAS2300_Config_init(&nasCfg);
    nasCfg.i2cAddr = targetAddress;

    /* Give the sensor rail/time-to-ready a moment after MCU boot */
    usleep(NSA2300_POWERUP_DELAY_US);

    /* Robust init: retry until cfg readback matches expected (avoids intermittent all-0 boots) */
    {
        bool ok = false;
        for (uint32_t attempt = 1; attempt <= NSA2300_INIT_RETRIES; attempt++) {
            ok = nas2300Init(i2cHandle, &i2cTransaction, &nasCfg);
            if (ok) {
                break;
            }
            SEGGER_RTT_printf(0, "NSA2300 init not ready (attempt %lu/%u); retrying...\n",
                              (unsigned long)attempt, (unsigned)NSA2300_INIT_RETRIES);
            usleep(NSA2300_INIT_RETRY_US);
        }

        if (!ok) {
            SEGGER_RTT_printf(0, "NSA2300 init failed after retries; will keep retrying in background\n");
        }
    }


    while (1) {
        /* Wait for a sampling request (task notify) */
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint8_t status = 0;
        uint8_t statusAfter = 0;
        uint32_t p24 = 0;
        uint32_t p24Single = 0;

        bool sampleOk = false;
        for (uint32_t sampleAttempt = 1; sampleAttempt <= NSA2300_SAMPLE_RETRIES; sampleAttempt++) {
            uint8_t cmdReadback = 0;

            /* Step 3: Start ADC conversion by writing CMD (0x30) */
            if (!NAS2300_startSinglePressureConversion(i2cHandle, &i2cTransaction,
                                                       txBuffer, sizeof(txBuffer),
                                                       rxBuffer, sizeof(rxBuffer),
                                                       &nasCfg, &cmdReadback)) {
                SEGGER_RTT_printf(0, "Failed to write CMD (reg 0x%02x)\n", 0x30);
                i2cErrorHandler(&i2cTransaction);
                nas2300Init(i2cHandle, &i2cTransaction, &nasCfg);
                continue;
            }

            SEGGER_RTT_printf(0, "CMD readback: 0x%02x\n", cmdReadback);
            if (cmdReadback != 0x09) {
                SEGGER_RTT_printf(0, "CMD readback mismatch (got 0x%02x); re-init\n", cmdReadback);
                nas2300Init(i2cHandle, &i2cTransaction, &nasCfg);
                usleep(10000);
                continue;
            }

            /* Step 4: Wait for conversion complete by polling DRDY (STATUS bit0) */
            {
                const uint32_t maxPolls = 3000; /* ~3s if poll delay is 1ms */
                if (!NAS2300_waitDrdy(i2cHandle, &i2cTransaction,
                                      txBuffer, sizeof(txBuffer),
                                      rxBuffer, sizeof(rxBuffer),
                                      &nasCfg,
                                      maxPolls,
                                      1000,
                                      &status)) {
                    SEGGER_RTT_printf(0, "Timeout waiting DRDY; STATUS=0x%02x (sampleAttempt %lu/%u)\n",
                                      status, (unsigned long)sampleAttempt, (unsigned)NSA2300_SAMPLE_RETRIES);
                    nas2300Init(i2cHandle, &i2cTransaction, &nasCfg);
                    continue;
                }

                SEGGER_RTT_printf(0, "DRDY observed; final STATUS=0x%02x\n", status);
            }

            /* Re-read STATUS immediately before reading data */
            if (NAS2300_readReg8(i2cHandle, &i2cTransaction,
                                 txBuffer, sizeof(txBuffer),
                                 rxBuffer, sizeof(rxBuffer),
                                 0x02,
                                 &statusAfter)) {
                SEGGER_RTT_printf(0, "STATUS before data read: 0x%02x\n", statusAfter);
            }

            /* Step 5: Read pressure result bytes 0x06/0x07/0x08 */
            if (!NAS2300_readPressureRaw24_burst(i2cHandle, &i2cTransaction,
                                                 txBuffer, sizeof(txBuffer),
                                                 rxBuffer, sizeof(rxBuffer),
                                                 &nasCfg,
                                                 &p24)) {
                SEGGER_RTT_printf(0, "Failed to read pressure data (reg 0x%02x..0x%02x)\n", 0x06, (uint8_t)(0x06 + 2));
                i2cErrorHandler(&i2cTransaction);
                nas2300Init(i2cHandle, &i2cTransaction, &nasCfg);
                continue;
            }

            /* Also try three single-byte reads (some devices don't support burst/auto-increment) */
            if (!NAS2300_readPressureRaw24_single(i2cHandle, &i2cTransaction,
                                                  txBuffer, sizeof(txBuffer),
                                                  rxBuffer, sizeof(rxBuffer),
                                                  &nasCfg,
                                                  &p24Single)) {
                SEGGER_RTT_printf(0, "Single-byte pressure reads failed\n");
                i2cErrorHandler(&i2cTransaction);
                nas2300Init(i2cHandle, &i2cTransaction, &nasCfg);
                continue;
            }

            {
                SEGGER_RTT_printf(0, "Pressure raw (burst): 0x%06lx\n", (unsigned long)p24);
                SEGGER_RTT_printf(0, "Pressure raw (single): 0x%06lx\n", (unsigned long)p24Single);

                {
                    int32_t code = NAS2300_decode_code24(p24, nasCfg.dataFormat);
                    /* Fraction of +FS (roughly) in Q0.24-ish float printed as milliFS */
                    int32_t milliFS = (int32_t)((int64_t)code * 1000LL / 8388607LL);

                    /* Always publish something so seq advances for "ready" semantics.
                     * If FS is configured, publish uV; otherwise publish raw code24.
                     */
                    int32_t publishValue = code;

                    if (nasCfg.fs_uV > 0) {
                        int32_t vin_uV = NAS2300_code24_to_uV(code, nasCfg.fs_uV);
                        SEGGER_RTT_printf(0, "Pressure code=%ld (~%ld/1000 FS), Vin=%ld uV (FS=%ld uV)\n",
                                          (long)code, (long)milliFS, (long)vin_uV, (long)nasCfg.fs_uV);
                        publishValue = vin_uV;
                    } else {
                        SEGGER_RTT_printf(0, "Pressure code=%ld (~%ld/1000 FS), Vin(uV)=<set nasCfg.fs_uV>\n",
                                          (long)code, (long)milliFS);
                    }

                    NAS2300_publish_latest_force_uV(publishValue);
                    /* Also publish raw 24-bit pressure for use in payloads */
                    NAS2300_publish_latest_p24(p24);
                }

                if (p24 == 0 && p24Single == 0) {
                    SEGGER_RTT_printf(0, "Pressure is all-zero; re-init and retry (sampleAttempt %lu/%u)\n",
                                      (unsigned long)sampleAttempt, (unsigned)NSA2300_SAMPLE_RETRIES);
                    nas2300Init(i2cHandle, &i2cTransaction, &nasCfg);
                    usleep(10000);
                    continue;
                }
            }

            sampleOk = true;
            break;
        }

        if (!sampleOk) {
            SEGGER_RTT_printf(0, "Sample failed after retries\n");
        }
    }
}

static bool nas2300Init(I2C_Handle i2cHandle, I2C_Transaction *transaction, NAS2300_Config *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    bool ok = NAS2300_init(i2cHandle, transaction,
                           txBuffer, sizeof(txBuffer),
                           rxBuffer, sizeof(rxBuffer),
                           cfg);

    if (!ok) {
        SEGGER_RTT_printf(0, "NSA2300 init failed (cfg readback mismatch or I2C error)\n");
        return false;
    }

    SEGGER_RTT_printf(0, "NSA2300 cfg OK: A5=0x%02x A6=0x%02x A7=0x%02x\n",
                      (unsigned)cfg->p_cfg0_value,
                      (unsigned)cfg->p_cfg1_value,
                      (unsigned)cfg->p_cfg2_value);

    return true;
}

/*
 *  ======== i2cErrorHandler ========
 */
static void i2cErrorHandler(
    I2C_Transaction *transaction)
{
    switch (transaction->status) {
        case I2C_STATUS_TIMEOUT:
            SEGGER_RTT_printf(0,  "I2C transaction timed out!\n");
            break;
        case I2C_STATUS_CLOCK_TIMEOUT:
            SEGGER_RTT_printf(0,  "I2C serial clock line timed out!\n");
            break;
        case I2C_STATUS_ADDR_NACK:
            SEGGER_RTT_printf(0,  "I2C target address 0x%x not acknowledged!\n", transaction->targetAddress);
            break;
        case I2C_STATUS_DATA_NACK:
            SEGGER_RTT_printf(0,  "I2C data byte not acknowledged!\n");
            break;
        case I2C_STATUS_ARB_LOST:
            SEGGER_RTT_printf(0,  "I2C arbitration to another controller!\n");
            break;
        case I2C_STATUS_INCOMPLETE:
            SEGGER_RTT_printf(0,  "I2C transaction returned before completion!\n");
            break;
        case I2C_STATUS_BUS_BUSY:
            SEGGER_RTT_printf(0,  "I2C bus is already in use!\n");
            break;
        case I2C_STATUS_CANCEL:
            SEGGER_RTT_printf(0,  "I2C transaction cancelled!\n");
            break;
        case I2C_STATUS_INVALID_TRANS:
            SEGGER_RTT_printf(0,  "I2C transaction invalid!\n");
            break;
        case I2C_STATUS_ERROR:
            SEGGER_RTT_printf(0,  "I2C generic error!\n");
            break;
        default:
            SEGGER_RTT_printf(0,  "I2C undefined error case!\n");
            break;
    }
}