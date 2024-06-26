/*!
    \file    gd32e23x_hal_nvic.h
    \brief   definitions for the NVIC
    
    \version 2019-03-14, V1.0.0, firmware for GD32E23x
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

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

#ifndef GD32E23X_HAL_NVIC_H
#define GD32E23X_HAL_NVIC_H

#include "gd32e23x_hal.h"

/* constants definitions */
/* set the RAM and FLASH base address */
#define NVIC_VECTTAB_RAM            ((uint32_t)0x20000000)                      /*!< RAM base address */
#define NVIC_VECTTAB_FLASH          ((uint32_t)0x08000000)                      /*!< Flash base address */

/* set the NVIC vector table offset mask */
#define NVIC_VECTTAB_OFFSET_MASK    ((uint32_t)0x1FFFFF80)                      /*!< NVIC vector table offset mask */

/* function declarations */
/* set NVIC request priority */
int32_t hal_nvic_irq_priority_set(IRQn_Type irqn, uint8_t priority);
/* get NVIC request priority */
uint32_t hal_nvic_irq_priority_get(IRQn_Type irqn);
/* reset NVIC system */
void hal_nvic_system_reset(void);
/* enable peripherals NVIC request */
int32_t hal_nvic_periph_irq_enable(IRQn_Type irqn, uint8_t priority);
/* disable NVIC request */
void hal_nvic_periph_irq_disable(IRQn_Type irqn);
/* set the NVIC vector table base address */
int32_t hal_nvic_vector_table_set(uint32_t nvic_vict_tab, uint32_t offset);

#endif /* GD32E23X_HAL_NVIC_H */
