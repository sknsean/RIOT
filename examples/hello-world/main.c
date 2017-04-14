/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Hello World application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "xtimer.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "pm_layered.h"
#include "board.h"

void isr_wdt(void)
{
    WDT->INTFLAG.reg = WDT_INTFLAG_EW;
    cortexm_isr_end();
}

void deepsleep (uint8_t count) {
    //powerdown();
    PM->APBAMASK.reg |= PM_APBAMASK_WDT;

    GCLK->CLKCTRL.reg = (WDT_GCLK_ID |
                         GCLK_CLKCTRL_CLKEN |
                         GCLK_CLKCTRL_GEN_GCLK2);
    while (GCLK->STATUS.bit.SYNCBUSY) {}

    WDT->CONFIG.reg = WDT_CONFIG_PER_16K;
    WDT->EWCTRL.reg = WDT_EWCTRL_EWOFFSET_8K;
    WDT->INTENSET.reg = WDT_INTENSET_EW;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY) {}
    WDT->CTRL.reg = WDT_CTRL_ENABLE;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY) {}

    NVIC_EnableIRQ(WDT_IRQn);

    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN |
                         GCLK_GENCTRL_SRC_OSCULP32K |
                         GCLK_GENCTRL_ID(0));
    while (GCLK->STATUS.bit.SYNCBUSY) {}


    while (count-- > 0) {
        WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
        while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY) {}
        pm_set(0);
    }
    //powerup();
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN |
                         GCLK_GENCTRL_SRC_DFLL48M |
                         GCLK_GENCTRL_ID(0));
    while (GCLK->STATUS.bit.SYNCBUSY) {}

    WDT->CTRL.reg &= ~WDT_CTRL_ENABLE;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY) {}
}

int main(void)
{
    char i;
    puts("Hello World!");

    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);

    //while(1) {
    for(i=0; i<5; i++) {
        leds_toggle();
        xtimer_usleep(1000000);
    }


uart_poweroff(UART_DEV(0));
    deepsleep(2);
uart_poweron(UART_DEV(0));

    while(1) {
        leds_toggle();
        xtimer_usleep(500000);
    }
    return 0;
}
