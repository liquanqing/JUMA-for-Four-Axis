/**
  ******************************************************************************
  * @file    connon_v2.c
  * @author  Inc
  * @version V1.0.0
  * @date    12-3-2016
  * @brief   This file provides set of firmware functions to manage LEDs
  *          available on connon_v2 board.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "connon_v2.h"

/** @defgroup BSP BSP
  * @{
  */ 


GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, 
                                };

const uint16_t GPIO_PIN[LEDn] = {LED1_PIN,
                                 };

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  */
void BSP_LED_Init(Led_TypeDef Led)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable the GPIO_LED Clock */
    LEDx_GPIO_CLK_ENABLE(Led);

    /* Configure the GPIO_LED pin */
    GPIO_InitStruct.Pin = GPIO_PIN[Led];
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

    HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}


/************************ (C) COPYRIGHT Inc *****END OF FILE****/
