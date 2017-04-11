/*
 * Copyright (C) 2017 Sean Nyekjaer
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_samd20
 * @{
 *
 * @file
 * @brief       Implementation of the CPU initialization
 *
 * @author      Sean Nyekjaer <sean@nyekjaer.dk>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @}
 */

#include "cpu.h"
#include "periph_conf.h"
#include "periph/init.h"

/**
 * @brief   Configure clock sources and the cpu frequency
 */
static void clk_init(void)
{
#if CLOCK_USE_DFLL
    uint32_t coarse;
    uint32_t fine;
#endif

    /* enable clocks for the power, sysctrl and gclk modules */
    PM->APBAMASK.reg = (PM_APBAMASK_PM | PM_APBAMASK_SYSCTRL |
                        PM_APBAMASK_GCLK);

    /* adjust NVM wait states, see table 42.30 (p. 1070) in the datasheet */
    PM->APBAMASK.reg |= PM_AHBMASK_NVMCTRL;
    NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS(1);
    PM->APBAMASK.reg &= ~PM_AHBMASK_NVMCTRL;

    /* configure internal 8MHz oscillator to run without prescaler */
    SYSCTRL->OSC8M.bit.PRESC = 0;
    SYSCTRL->OSC8M.bit.ONDEMAND = 1;
    SYSCTRL->OSC8M.bit.RUNSTDBY = 0;
    SYSCTRL->OSC8M.bit.ENABLE = 1;
    while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC8MRDY)) {}

#if CLOCK_USE_DFLL
    /* reset the GCLK module so it is in a known state */
    GCLK->CTRL.reg = GCLK_CTRL_SWRST;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* setup generic clock 1 to feed DPLL with 1MHz */
    GCLK->GENDIV.reg = (GCLK_GENDIV_DIV(1) |
                        GCLK_GENDIV_ID(1));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN |
                         GCLK_GENCTRL_SRC_OSC8M |
                         GCLK_GENCTRL_ID(1));
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* errata_9905 */
    SYSCTRL->DFLLCTRL.reg = SYSCTRL->DFLLCTRL.reg & ~SYSCTRL_DFLLCTRL_ONDEMAND;
    while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY)) {}

    coarse = (*(uint32_t *)FUSES_DFLL48M_COARSE_CAL_ADDR)
             >> FUSES_DFLL48M_COARSE_CAL_Pos
             & FUSES_DFLL48M_COARSE_CAL_Msk;

    fine = (*(uint32_t *)FUSES_DFLL48M_FINE_CAL_ADDR)
           >> FUSES_DFLL48M_FINE_CAL_Pos
           & FUSES_DFLL48M_FINE_CAL_Msk;

    SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse)
                           | SYSCTRL_DFLLVAL_FINE(fine);
    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
    while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY)) {}

    /* select the DFLL as source for clock generator 0 (CPU core clock) */
    GCLK->GENDIV.reg =  (GCLK_GENDIV_DIV(CLOCK_DFLL_DIV) |
                        GCLK_GENDIV_ID(0));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN |
                         GCLK_GENCTRL_SRC_DFLL48M |
                         GCLK_GENCTRL_ID(0));
#else /* do not use DFLL, use internal 8MHz oscillator directly */
    GCLK->GENDIV.reg =  (GCLK_GENDIV_DIV(CLOCK_DIV) |
                        GCLK_GENDIV_ID(0));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN |
                         GCLK_GENCTRL_SRC_OSC8M |
                         GCLK_GENCTRL_ID(0));
#endif

    /* make sure we synchronize clock generator 0 before we go on */
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* Setup Clock generator 2 with divider 1 (32.768kHz) */
    GCLK->GENDIV.reg  = (GCLK_GENDIV_ID(2)  | GCLK_GENDIV_DIV(0));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN |
                        GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_SRC_OSCULP32K);

    while (GCLK->STATUS.bit.SYNCBUSY) {}

    /* redirect all peripherals to a disabled clock generator (7) by default */
    for (int i = 0x3; i <= 0x1b; i++) {
        GCLK->CLKCTRL.reg = ( GCLK_CLKCTRL_ID(i) | GCLK_CLKCTRL_GEN_GCLK7 );
        while (GCLK->STATUS.bit.SYNCBUSY) {}
    }
}

void cpu_init(void)
{
    /* disable the watchdog timer */
    WDT->CTRL.bit.ENABLE = 0;
    /* initialize the Cortex-M core */
    cortexm_init();
    /* Initialise clock sources and generic clocks */
    clk_init();
    /* trigger static peripheral initialization */
    periph_init();
}
