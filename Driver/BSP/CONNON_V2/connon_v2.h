/** 
  ******************************************************************************
  * @file    connon_v2.h
  * @author  Inc
  * @version V1.0.0
  * @date    13-3-2016
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on connnon_v2
  *            from juma_board
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 Inc</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONNON_V2_H
#define __CONNON_V2_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
   
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup CONNON_V2
  * @{
  */

typedef enum 
{
  LED1 = 0,
  LED_GREEN = LED1,
}Led_TypeDef;



/**
  * @}
  */ 

#define LEDn                                    1

#define LED1_PIN                                GPIO_PIN_3
#define LED1_GPIO_PORT                          GPIOB
#define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()  
 

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) == 0) {__HAL_RCC_GPIOB_CLK_ENABLE();} else\
                                                                    {__HAL_RCC_GPIOB_CLK_ENABLE();   }} while(0)	
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  do { if((__INDEX__) == 0) {__HAL_RCC_GPIOB_CLK_DISABLE();} else\
                                                                    {__HAL_RCC_GPIOB_CLK_DISABLE();   }} while(0)	
 
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);



#ifdef __cplusplus
}
#endif

#endif /* __CONNON_V2_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
