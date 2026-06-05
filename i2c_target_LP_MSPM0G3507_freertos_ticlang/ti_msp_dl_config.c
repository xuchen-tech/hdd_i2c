/*
 * Copyright (c) 2023-2024, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 */

#include "ti_msp_dl_config.h"

static const DL_I2C_ClockConfig gI2CClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_SYSCTL_CLK_init();
    SYSCFG_DL_I2C_init();
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_I2C_reset(I2C1_INST);
    DL_I2C_reset(I2C0_INST);
    DL_ADC12_reset(ADC_0_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_I2C_enablePower(I2C1_INST);
    DL_I2C_enablePower(I2C0_INST);
    DL_ADC12_enablePower(ADC_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C0_IOMUX_SDA,
        GPIO_I2C0_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C0_IOMUX_SCL,
        GPIO_I2C0_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C0_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C0_IOMUX_SCL);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setPowerPolicyRUN0SLEEP0();
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_CLK_init(void)
{
    while ((DL_SYSCTL_getClockStatus() & (DL_SYSCTL_CLK_STATUS_LFOSC_GOOD)) !=
           (DL_SYSCTL_CLK_STATUS_LFOSC_GOOD)) {
        ;
    }
}

void SYSCFG_DL_I2C_init(void) 
{
    DL_I2C_setClockConfig(I2C0_INST,
        (DL_I2C_ClockConfig *) &gI2CClockConfig);
    DL_I2C_disableAnalogGlitchFilter(I2C0_INST);

    /* Configure Target Mode */
    DL_I2C_setTargetOwnAddress(I2C0_INST, 0x50);
    DL_I2C_setTargetTXFIFOThreshold(I2C0_INST, DL_I2C_TX_FIFO_LEVEL_BYTES_2);
    DL_I2C_setTargetRXFIFOThreshold(I2C0_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableTargetTXEmptyOnTXRequest(I2C0_INST);
    DL_I2C_disableTargetTXTriggerInTXMode(I2C0_INST);
    /* Hold SCL when TX FIFO is stale so firmware can refill before 0xFF idle
     * bytes are shifted out (complements clock stretching below). */
    DL_I2C_enableTargetTXWaitWhenTXFIFOStale(I2C0_INST);

    /* Clock stretching MUST stay enabled: with a low TX FIFO trigger threshold
     * (BYTES_2) the refill window is ~45us @400kHz. If the ISR is late (e.g.
     * during downstream I2C1 traffic when cascaded), the TX FIFO underflows.
     * With stretching enabled the target holds SCL low until firmware refills
     * instead of shifting out idle 0xFF, which is what corrupted the MODBUS CRC
     * frame on the master side. */
    DL_I2C_enableTargetClockStretching(I2C0_INST);

    /* Workaround for errata I2C_ERR_04 */
    DL_I2C_disableTargetWakeup(I2C0_INST);
    /* Configure Interrupts.
     * TXFIFO_EMPTY is intentionally NOT enabled: with clock stretching plus
     * TXWaitWhenTXFIFOStale the FIFO is held (SCL low) before it ever drains
     * to empty, so this source never fires (observed TXEMPTY=0) and only adds
     * latent ISR load. TXFIFO_TRIGGER (BYTES_2) handles the steady refill and
     * TXFIFO_UNDERFLOW remains as the safety net. Removing it lowers the I2C0
     * ISR rate so the I2C1 controller's completion interrupt is serviced in
     * time during cascaded traffic. */
    DL_I2C_enableInterrupt(I2C0_INST,
                           DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
                           DL_I2C_INTERRUPT_TARGET_START |
                           DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
                           DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW |
                           DL_I2C_INTERRUPT_TARGET_STOP);


    /* Enable module */
    DL_I2C_enableTarget(I2C0_INST);
}
