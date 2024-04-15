/*
    \file  gd32e23x_hal_init.c
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
#include "gd32e23x_hal_init.h"
/* user code [global 0] begin */

/* user code [global 0] end */
hal_adc_dev_struct adc_info;
hal_timer_dev_struct timer15_info;
hal_uart_dev_struct uart0_info;

void msd_system_init(void)
{
    /* user code [system_init local 0] begin */
    /* user code [system_init local 0] end */
    hal_fmc_prefetch_enable();
    hal_rcu_periph_clk_enable(RCU_CFGCMP);
    hal_nvic_periph_irq_enable(NonMaskableInt_IRQn, 0);
    hal_nvic_periph_irq_enable(HardFault_IRQn, 0);
    hal_nvic_periph_irq_enable(SVCall_IRQn, 0);
    hal_nvic_periph_irq_enable(PendSV_IRQn, 0);
    hal_nvic_periph_irq_enable(SysTick_IRQn, 0);
    hal_basetick_init(HAL_BASETICK_SOURCE_SYSTICK);

    /* user code [system_init local 1] begin */
    /* user code [system_init local 1] end */
}

void msd_clock_init(void)
{
    /* user code [clock_init local 0] begin */
    /* user code [clock_init local 0] end */
    hal_rcu_clk_struct rcu_clk_parameter;
    hal_rcu_osci_struct rcu_osci_parameter;
    hal_rcu_periphclk_struct rcu_periphclk_parameter;

    hal_rcu_struct_init(HAL_RCU_CLK_STRUCT, &rcu_clk_parameter);
    hal_rcu_struct_init(HAL_RCU_OSCI_STRUCT, &rcu_osci_parameter);
    hal_rcu_struct_init(HAL_RCU_PERIPHCLK_STRUCT, &rcu_periphclk_parameter);

    rcu_osci_parameter.hxtal.need_configure = ENABLE;
    rcu_osci_parameter.hxtal.state = RCU_OSC_ON;
    rcu_osci_parameter.irc8m.need_configure = ENABLE;
    rcu_osci_parameter.irc8m.state = RCU_OSC_ON;
    rcu_osci_parameter.irc8m.adjust_value = 0;
    rcu_osci_parameter.pll.need_configure = ENABLE;
    rcu_osci_parameter.pll.state = RCU_OSC_ON;
    rcu_osci_parameter.pll.pll_source = RCU_PLL_SRC_HXTAL;
    rcu_osci_parameter.pll.pll_mul = RCU_PLL_MULT18;
    rcu_osci_parameter.pll.pre_div = RCU_PLL_PREDIV2;
    if(HAL_ERR_NONE != hal_rcu_osci_config(&rcu_osci_parameter)){
        while(1);
    }

    rcu_clk_parameter.clock_type = RCU_CLKTYPE_SYSCLK | RCU_CLKTYPE_AHBCLK | RCU_CLKTYPE_APB1CLK | RCU_CLKTYPE_APB2CLK;
    rcu_clk_parameter.sysclk_source = RCU_SYSCLK_SRC_PLL;
    rcu_clk_parameter.ahbclk_divider = RCU_SYSCLK_AHBDIV1;
    rcu_clk_parameter.apb1clk_divider = RCU_AHBCLK_APB1DIV1;
    rcu_clk_parameter.apb2clk_divider = RCU_AHBCLK_APB2DIV1;
    if(HAL_ERR_NONE != hal_rcu_clock_config(&rcu_clk_parameter, WS_WSCNT_2)){
        while(1);
    }

    rcu_periphclk_parameter.periph_clock_type = RCU_PERIPH_CLKTYPE_ADC;
    rcu_periphclk_parameter.adc_clock_source = RCU_ADCCK_APB2_DIV2;
    if(HAL_ERR_NONE != hal_rcu_periph_clock_config(&rcu_periphclk_parameter)){
        while(1);
    }

    rcu_periphclk_parameter.periph_clock_type = RCU_PERIPH_CLKTYPE_USART0;
    rcu_periphclk_parameter.usart0_clock_source = RCU_USART0_CLKSRC_APB2;
    if(HAL_ERR_NONE != hal_rcu_periph_clock_config(&rcu_periphclk_parameter)){
        while(1);
    }

    /* user code [clock_init local 1] begin */
    /* user code [clock_init local 1] end */
}

void msd_gpio_init(void)
{
    /* user code [gpio_init local 0] begin */
    /* user code [gpio_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;

    hal_rcu_periph_clk_enable(RCU_GPIOC);
    hal_rcu_periph_clk_enable(RCU_GPIOF);
    hal_rcu_periph_clk_enable(RCU_GPIOA);
    hal_gpio_struct_init(&gpio_init_parameter);

    hal_gpio_bit_reset(LED_Pin_GPIO_PORT, LED_Pin_PIN);
    gpio_init_parameter.mode = HAL_GPIO_MODE_OUTPUT_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(LED_Pin_GPIO_PORT, LED_Pin_PIN, &gpio_init_parameter);

    /* user code [gpio_init local 1] begin */
    /* user code [gpio_init local 1] end */
}

void msd_gpio_deinit(void)
{
    /* user code [gpio_deinit local 0] begin */
    /* user code [gpio_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_GPIOC);
    hal_rcu_periph_clk_disable(RCU_GPIOF);
    hal_rcu_periph_clk_disable(RCU_GPIOA);
    hal_gpio_deinit(LED_Pin_GPIO_PORT, LED_Pin_PIN);
    /* user code [gpio_deinit local 1] begin */
    /* user code [gpio_deinit local 1] end */
}

void msd_adc_init(void)
{
    /* user code [adc_init local 0] begin */
    /* user code [adc_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;
    hal_adc_init_struct adc_init_parameter;

    hal_rcu_periph_clk_enable(RCU_ADC);
    hal_gpio_struct_init(&gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_1, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_0, &gpio_init_parameter);

    hal_adc_struct_init(HAL_ADC_DEV_STRUCT, &adc_info);
    hal_adc_struct_init(HAL_ADC_INIT_STRUCT, &adc_init_parameter);

    adc_init_parameter.resolution_select = ADC_RESOLUTION_12B;
    adc_init_parameter.data_alignment = ADC_DATAALIGN_RIGHT;
    adc_init_parameter.scan_mode = DISABLE;
    adc_init_parameter.oversample_config.oversample_mode = DISABLE;
    hal_adc_init(&adc_info, &adc_init_parameter);

    hal_adc_calibration(&adc_info);
    /* user code [adc_init local 1] begin */
    /* user code [adc_init local 1] end */
}

void msd_adc_deinit(void)
{
    /* user code [adc_deinit local 0] begin */
    /* user code [adc_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_ADC);
    hal_gpio_deinit(GPIOA, GPIO_PIN_1);
    hal_gpio_deinit(GPIOA, GPIO_PIN_0);
    hal_adc_deinit(&adc_info);
    /* user code [adc_deinit local 1] begin */
    /* user code [adc_deinit local 1] end */
}

void msd_timer15_init(void)
{
    /* user code [timer15_init local 0] begin */
    /* user code [timer15_init local 0] end */
    hal_timer_basic_struct timer15_basic_parameter;

    hal_rcu_periph_clk_enable(RCU_TIMER15);
    hal_timer_struct_init(HAL_TIMER_DEV_STRUCT, &timer15_info);
    hal_timer_struct_init(HAL_TIMER_BASIC_STRUCT, &timer15_basic_parameter);

    timer15_basic_parameter.prescaler = 7199;
    timer15_basic_parameter.alignedmode = TIMER_COUNTER_EDGE;
    timer15_basic_parameter.counterdirection = TIMER_COUNTER_UP;
    timer15_basic_parameter.period = 9999;
    timer15_basic_parameter.clockdivision = TIMER_CKDIV_DIV1;
    timer15_basic_parameter.repetitioncounter = 0;
    timer15_basic_parameter.autoreload_shadow = AUTO_RELOAD_SHADOW_DISABLE;
    hal_timer_basic_init(&timer15_info, TIMER15, &timer15_basic_parameter);

    hal_nvic_periph_irq_enable(TIMER15_IRQn, 0);
    /* user code [timer15_init local 1] begin */
    /* user code [timer15_init local 1] end */
}

void msd_timer15_deinit(void)
{
    /* user code [timer15_deinit local 0] begin */
    /* user code [timer15_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_TIMER15);
    hal_timer_deinit(&timer15_info);
    /* user code [timer15_deinit local 1] begin */
    /* user code [timer15_deinit local 1] end */
}

void msd_usart0_init(void)
{
    /* user code [usart0_init local 0] begin */
    /* user code [usart0_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;
    hal_uart_init_struct uart0_init_parameter;

    hal_rcu_periph_clk_enable(RCU_USART0);
    hal_gpio_struct_init(&gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_1;
    hal_gpio_init(GPIOA, GPIO_PIN_10, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_1;
    hal_gpio_init(GPIOA, GPIO_PIN_9, &gpio_init_parameter);

    hal_uart_struct_init(HAL_UART_DEV_STRUCT, &uart0_info);
    hal_uart_struct_init(HAL_UART_INIT_STRUCT, &uart0_init_parameter);

    uart0_init_parameter.baudrate = 115200;
    uart0_init_parameter.parity = UART_PARITY_NONE;
    uart0_init_parameter.word_length = UART_WORD_LENGTH_8BIT;
    uart0_init_parameter.direction = UART_DIRECTION_RX_TX;
    uart0_init_parameter.stop_bit = UART_STOP_BIT_1;
    uart0_init_parameter.over_sample = UART_OVER_SAMPLE_16;
    uart0_init_parameter.hardware_flow = UART_HARDWARE_FLOW_NONE;
    uart0_init_parameter.work_mode = UART_WORK_MODE_ASYN;
    uart0_init_parameter.sample_method = UART_THREE_SAMPLE_BIT;
    hal_uart_init(&uart0_info, USART0, &uart0_init_parameter);

    hal_nvic_periph_irq_enable(USART0_IRQn, 0);
    /* user code [usart0_init local 1] begin */
    /* user code [usart0_init local 1] end */
}

void msd_usart0_deinit(void)
{
    /* user code [usart0_deinit local 0] begin */
    /* user code [usart0_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_USART0);
    hal_gpio_deinit(GPIOA, GPIO_PIN_10);
    hal_gpio_deinit(GPIOA, GPIO_PIN_9);
    hal_uart_deinit(&uart0_info);
    /* user code [usart0_deinit local 1] begin */
    /* user code [usart0_deinit local 1] end */
}

/* user code [global 1] begin */

/* user code [global 1] end */
