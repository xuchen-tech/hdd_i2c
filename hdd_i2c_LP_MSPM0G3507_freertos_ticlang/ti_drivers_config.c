/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== ti_drivers_config.c ========
 *  Configured TI-Drivers module definitions
 */

#include <stddef.h>
#include <stdint.h>

#include "ti_drivers_config.h"


/*
 *  =============================== GPIO ===============================
 */

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSPM0.h>

/* The range of pins available on this device */
const uint_least8_t GPIO_pinLowerBound = 0;
const uint_least8_t GPIO_pinUpperBound = 60;

/*
 *  ======== gpioPinConfigs ========
 *  Array of Pin configurations
 */
GPIO_PinConfig gpioPinConfigs[60] = {
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA0 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA1 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA2 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA3 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA4 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA5 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA6 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA7 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA8 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA9 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA10 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA11 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA12 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA13 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA14 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA15 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA16 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA17 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA18 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA19 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA20 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA21 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA22 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA23 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA24 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA25 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA26 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA27 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA28 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA29 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA30 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PA31 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB0 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB1 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB2 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB3 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB4 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB5 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB6 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB7 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB8 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB9 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB10 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB11 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB12 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB13 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB14 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB15 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB16 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB17 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB18 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB19 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB20 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB21 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB22 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB23 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB24 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB25 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB26 */
    GPIO_CFG_INPUT | GPIO_DO_NOT_CONFIG, /* PB27 */
};

/*
 *  ======== gpioCallbackFunctions ========
 *  Array of callback function pointers
 *  Change at runtime with GPIO_setCallback()
 */
GPIO_CallbackFxn gpioCallbackFunctions[60];

/*
 *  ======== gpioUserArgs ========
 *  Array of user argument pointers
 *  Change at runtime with GPIO_setUserArg()
 *  Get values with GPIO_getUserArg()
 */
void *gpioUserArgs[60];

const uint_least8_t CONFIG_GPIO_LED_0_CONST = CONFIG_GPIO_LED_0;

/*
 *  ======== GPIO_config ========
 */
const GPIO_Config GPIO_config = {.configs = (GPIO_PinConfig *) gpioPinConfigs,
    .callbacks   = (GPIO_CallbackFxn *) gpioCallbackFunctions,
    .userArgs    = gpioUserArgs,
    .intPriority = (~0)};

/*
 *  =============================== I2C ===============================
 */

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CMSPM0.h>

#define CONFIG_I2C_COUNT 1

/*
 *  ======== i2cObjects ========
 */
I2CMSPM0_Object I2CMSPM0Objects[CONFIG_I2C_COUNT];

/*
 *  ======== i2cHWAttrs ========
 */
const I2CMSPM0_HWAttrs I2CMSPM0HWAttrs[CONFIG_I2C_COUNT] = {
    /* CONFIG_I2C_TMP */
    /* LaunchPad I2C */
    {
        .i2c         = I2C1_INST,
        .intNum      = I2C1_INST_INT_IRQN,
        .intPriority = (~0),

        .sdaPincm    = GPIO_I2C1_IOMUX_SDA,
        .sdaPinIndex = GPIO_I2C1_SDA_PIN,
        .sdaPinMux   = GPIO_I2C1_IOMUX_SDA_FUNC,

        .sclPincm    = GPIO_I2C1_IOMUX_SCL,
        .sclPinIndex = GPIO_I2C1_SCL_PIN,
        .sclPinMux   = GPIO_I2C1_IOMUX_SCL_FUNC,

        .clockSource              = DL_I2C_CLOCK_BUSCLK,
        .clockDivider             = DL_I2C_CLOCK_DIVIDE_1,
        .txIntFifoThr             = DL_I2C_TX_FIFO_LEVEL_BYTES_1,
        .rxIntFifoThr             = DL_I2C_RX_FIFO_LEVEL_BYTES_1,
        .isClockStretchingEnabled = true,
        .i2cClk                   = I2C_CLOCK_MHZ,
    },
};

/*
 *  ======== I2C_config ========
 */
const I2C_Config I2C_config[CONFIG_I2C_COUNT] = {
    /* CONFIG_I2C_TMP */
    /* LaunchPad I2C */
    {.object     = &I2CMSPM0Objects[CONFIG_I2C_TMP],
        .hwAttrs = &I2CMSPM0HWAttrs[CONFIG_I2C_TMP]},
};

const uint_least8_t CONFIG_I2C_TMP_CONST = CONFIG_I2C_TMP;
const uint_least8_t I2C_count            = CONFIG_I2C_COUNT;
