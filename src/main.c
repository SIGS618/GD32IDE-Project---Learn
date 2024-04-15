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
// 要注意参数一致吗?
void timer_update_irq_handler(hal_timer_dev_struct *timer_dev)
{
	if (timer_dev == &timer15_info) // 判断中断来源?
		gpio_bit_toggle(LED_Pin_GPIO_PORT, LED_Pin_PIN);
}
/* user code [global 0] end */

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
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

    /* hal_xxx_irq()函数实现了外设模块所有中断标志位的判断及清中断操作，这样用户就不需
	要关心各类中断应该怎样处理，仅需要将对应中断来临时希望被调用到的函数以回调函数指针
    的形式传入“设备中断回调函数结构体”中即可。 */
    // 1. 声明 Timer 中断函数结构体
    hal_timer_irq_struct timer_irq_struct;
    // 2. 将希望响应的中断函数设置为自定义函数
    timer_irq_struct.update_usercb = timer_update_irq_handler;
    // 3. 设置“设备中断回调函数结构体”
    hal_timer_irq_handle_set(&timer15_info, &timer_irq_struct);
    /* user code [local 2] end */

    while(1){
        /* user code [local 3] begin */

        /* user code [local 3] end */
    }
}
/* user code [global 1] begin */

/* user code [global 1] end */	
