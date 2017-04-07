/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     driver_periph
 * @{
 *
 * @file
 * @brief       Low-level UART driver implementation
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Troels Hoffmeyer <troels.d.hoffmeyer@gmail.com>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "cpu.h"

#include "periph/uart.h"
#include "periph/gpio.h"

#define SHIFT 32

/**
 * @brief   Allocate memory to store the callback functions
 */
static uart_isr_ctx_t uart_ctx[UART_NUMOF];

/**
 * @brief   Get the pointer to the base register of the given UART device
 *
 * @param[in] dev       UART device identifier
 *
 * @return              base register address
 */
static inline SercomUsart *_uart(uart_t dev)
{
    return uart_config[dev].dev;
}

static int init_base(uart_t uart, uint32_t baudrate);

int uart_init(uart_t uart, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    /* initialize basic functionality */
    int res = init_base(uart, baudrate);
    if (res != UART_OK) {
        return res;
    }

    /* register callbacks */
    uart_ctx[uart].rx_cb = rx_cb;
    uart_ctx[uart].arg = arg;
    /* configure interrupts and enable RX interrupt */
    _uart(uart)->INTENSET.reg = SERCOM_USART_INTENSET_RXC;
    NVIC_EnableIRQ(SERCOM0_IRQn + sercom_id(_uart(uart)));
    return UART_OK;
}

static uint64_t long_division(uint64_t n, uint64_t d)
{
    int32_t i;
    uint64_t q = 0, r = 0, bit_shift;
    for (i = 63; i >= 0; i--) {
        bit_shift = (uint64_t)1 << i;

        r = r << 1;

        if (n & bit_shift) {
            r |= 0x01;
        }

        if (r >= d) {
            r = r - d;
            q |= bit_shift;
        }
    }

    return q;
}

static int init_base(uart_t uart, uint32_t baudrate)
{
    uint32_t baud;
    SercomUsart *dev;
    uint64_t temp1;
    uint64_t ratio = 0;
    uint64_t scale = 0;

    if ((unsigned int)uart >= UART_NUMOF) {
        return UART_NODEV;
    }

    /* get the devices base register */
    dev = _uart(uart);
    /* calculate baudrate */
    temp1 = ((16 * (uint64_t)baudrate) << SHIFT);
    ratio = long_division(temp1, CLOCK_CORECLOCK);
    scale = ((uint64_t)1 << SHIFT) - ratio;
    baud = (65536 * scale) >> SHIFT;
    /* enable sync and async clocks */
    uart_poweron(uart);
    /* configure pins */
    gpio_init(uart_config[uart].rx_pin, GPIO_IN);
    gpio_init_mux(uart_config[uart].rx_pin, uart_config[uart].mux);
    gpio_init(uart_config[uart].tx_pin, GPIO_OUT);
    gpio_init_mux(uart_config[uart].tx_pin, uart_config[uart].mux);
    /* reset the UART device */
    dev->CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (dev->STATUS.reg & SERCOM_USART_STATUS_SYNCBUSY) {}

    /* set asynchronous mode w/o parity, LSB first, TX and RX pad as specified
     * by the board in the periph_conf.h, x16 sampling and use internal clock */
    dev->CTRLA.reg = (SERCOM_USART_CTRLA_DORD |
                      uart_config[uart].tx_pad << SERCOM_USART_CTRLA_TXPO_Pos |
                      SERCOM_USART_CTRLA_RXPO(uart_config[uart].rx_pad) |
                      SERCOM_USART_CTRLA_MODE_USART_INT_CLK);
    /* set baudrate */
    dev->BAUD.reg = baud;
    /* enable receiver and transmitter, use 1 stop bit */
    dev->CTRLB.reg = (SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);
    /* finally, enable the device */
    dev->CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
    return UART_OK;
}

void uart_write(uart_t uart, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        while (!(_uart(uart)->INTFLAG.reg & SERCOM_USART_INTFLAG_DRE)) {}
        _uart(uart)->DATA.reg = data[i];
    }
}

void uart_poweron(uart_t uart)
{
    PM->APBCMASK.reg |= (PM_APBCMASK_SERCOM0 << sercom_id(_uart(uart)));
    GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_CLKEN |
                         GCLK_CLKCTRL_GEN_GCLK0 |
                         (SERCOM0_GCLK_ID_CORE + sercom_id(_uart(uart))) <<
                          GCLK_CLKCTRL_ID_Pos);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}
}

void uart_poweroff(uart_t uart)
{
    PM->APBCMASK.reg &= ~(PM_APBCMASK_SERCOM0 << sercom_id(_uart(uart)));
    GCLK->CLKCTRL.reg = ((SERCOM0_GCLK_ID_CORE + sercom_id(_uart(uart))) <<
                          GCLK_CLKCTRL_ID_Pos);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}
}

static inline void irq_handler(int dev)
{
    SercomUsart *uart = _uart(dev);

    if (uart->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) {
        /* interrupt flag is cleared by reading the data register */
        uart_ctx[dev].rx_cb(uart_ctx[dev].arg, (uint8_t)(uart->DATA.reg));
    }
    cortexm_isr_end();
}

#ifdef UART_0_ISR
void UART_0_ISR(void)
{
    irq_handler(0);
}
#endif

#ifdef UART_1_ISR
void UART_1_ISR(void)
{
    irq_handler(1);
}
#endif

#ifdef UART_2_ISR
void UART_2_ISR(void)
{
    irq_handler(2);
}
#endif

#ifdef UART_3_ISR
void UART_3_ISR(void)
{
    irq_handler(3);
}
#endif

#ifdef UART_4_ISR
void UART_4_ISR(void)
{
    irq_handler(4);
}
#endif

#ifdef UART_5_ISR
void UART_5_ISR(void)
{
    irq_handler(5);
}
#endif
