/*
    \file  main.c
*/
/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/
#include "gd32e23x_hal.h"
#include "gd32e23x_hal_init.h"
/* user code [global 0] begin */
#define ARRAY_SIZE(arr_name) (uint32_t)(sizeof(arr_name) / sizeof(*(arr_name)))
#define TRANSMIT_SIZE(arr_name) (ARRAY_SIZE(arr_name) - 1)

/* TIMER15 update interrupt user callback */
void timer_update_callback(hal_timer_dev_struct *timer_dev);

/* TIMER15 interrupt user callback function pointer structure */
hal_timer_irq_struct timer15_irq_handle;

void tx_complete_callback(hal_uart_dev_struct *uart);

uint8_t transmitter_buffer[] = "led toggled!\r\n";
__IO FlagStatus tx_end = RESET;
/* user code [global 0] end */

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void) {
    /* user code [local 0] begin */

    /* user code [local 0] end */

    msd_system_init();
    msd_clock_init();
    /* user code [local 1] begin */

    /* user code [local 1] end */
    msd_gpio_init();
    msd_adc_init();
    msd_timer15_init();
    msd_usart0_init();

    /* user code [local 2] begin */
    hal_gpio_bit_reset(LED_Pin_GPIO_PORT, LED_Pin_PIN);

    /* configure TIMER15 interrupt user callback function pointer structure */
    timer15_irq_handle.update_usercb = timer_update_callback;
    /* start TIMER counter and update interrupt */
    hal_timer_irq_handle_set(&timer15_info, &timer15_irq_handle);

    /* 使能外设 */
    hal_timer_start_counter_interrupt(&timer15_info);
    hal_uart_start(&uart0_info);
    /* user code [local 2] end */

    while (1) {
        /* user code [local 3] begin */

        /* user code [local 3] end */
    }
}

/* user code [global 1] begin */
void timer_update_callback(hal_timer_dev_struct *timer_dev) {
    if (timer_dev == &timer15_info) {
        hal_gpio_bit_toggle(LED_Pin_GPIO_PORT, LED_Pin_PIN);
        tx_end = RESET; // 我都不知道这个有什么意义
        hal_uart_transmit_interrupt(&uart0_info, transmitter_buffer, TRANSMIT_SIZE(transmitter_buffer),
                                    NULL);
    }
}
/* user code [global 1] end */	
