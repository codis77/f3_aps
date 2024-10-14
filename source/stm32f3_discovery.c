/**
  ******************************************************************************
  * @file    stm32f3_discovery.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   This file provides set of firmware functions to manage Leds and
  *          push-button available on STM32F3-DISCOVERY Kit from STMicroelectronics.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3_discovery.h"

/**  STM32F3_DISCOVERY_LOW_LEVEL
  *  This file provides set of firmware functions to manage Leds and push-button
  *  available on STM32F3-Discovery Kit from STMicroelectronics.
  *  STM32F3_DISCOVERY_LOW_LEVEL_Private_TypesDefinitions
  */


/** STM32F3_DISCOVERY_LOW_LEVEL_Private_Variables
  */
GPIO_TypeDef *GPIO_PORT[LEDn] = {
    LED3_GPIO_PORT, LED4_GPIO_PORT, LED5_GPIO_PORT, LED6_GPIO_PORT,
    LED7_GPIO_PORT, LED8_GPIO_PORT, LED9_GPIO_PORT, LED10_GPIO_PORT
};
const uint16_t GPIO_PIN[LEDn] = {
    LED3_PIN, LED4_PIN, LED5_PIN, LED6_PIN,
    LED7_PIN, LED8_PIN, LED9_PIN, LED10_PIN
};
const uint32_t GPIO_CLK[LEDn] = {
    LED3_GPIO_CLK, LED4_GPIO_CLK, LED5_GPIO_CLK, LED6_GPIO_CLK,
    LED7_GPIO_CLK, LED8_GPIO_CLK, LED9_GPIO_CLK, LED10_GPIO_CLK
};
GPIO_TypeDef   *BUTTON_PORT[BUTTONn]        = { USER_BUTTON_GPIO_PORT };
const uint16_t  BUTTON_PIN[BUTTONn]         = { USER_BUTTON_PIN };
const uint32_t  BUTTON_CLK[BUTTONn]         = { USER_BUTTON_GPIO_CLK };
const uint16_t  BUTTON_EXTI_LINE[BUTTONn]   = { USER_BUTTON_EXTI_LINE };
const uint8_t   BUTTON_PORT_SOURCE[BUTTONn] = { USER_BUTTON_EXTI_PORT_SOURCE };
const uint8_t   BUTTON_PIN_SOURCE[BUTTONn]  = { USER_BUTTON_EXTI_PIN_SOURCE };
const uint8_t   BUTTON_IRQn[BUTTONn]        = { USER_BUTTON_EXTI_IRQn };


/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED3 ... LED10
  * @retval None
  */
void  STM_EVAL_LEDInit (Led_TypeDef Led)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_AHBPeriphClockCmd (GPIO_CLK[Led], ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin   = GPIO_PIN[Led];
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (GPIO_PORT[Led], &GPIO_InitStructure);
}



/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED3 ... LED10
  * @retval None
  */
void  STM_EVAL_LEDOn (Led_TypeDef Led)
{
    GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
}


/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED3 ... LED10
  * @retval None
  */
void  STM_EVAL_LEDOff (Led_TypeDef Led)
{
    GPIO_PORT[Led]->BRR = GPIO_PIN[Led];
}


/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED3 ... LED10
  * @retval None
  */
void  STM_EVAL_LEDToggle (Led_TypeDef Led)
{
    GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}


/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  * @retval None
  */
void  STM_EVAL_PBInit (Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    EXTI_InitTypeDef  EXTI_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* Enable the BUTTON Clock */
    RCC_AHBPeriphClockCmd (BUTTON_CLK[Button], ENABLE);
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_SYSCFG, ENABLE);

    /* Configure Button pin as input */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin  = BUTTON_PIN[Button];
    GPIO_Init (BUTTON_PORT[Button], &GPIO_InitStructure);

    if (Button_Mode == BUTTON_MODE_EXTI)
    {
        /* Connect Button EXTI Line to Button GPIO Pin */
        SYSCFG_EXTILineConfig (BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

        /* Configure Button EXTI line */
        EXTI_InitStructure.EXTI_Line    = BUTTON_EXTI_LINE[Button];
        EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init (&EXTI_InitStructure);

        /* Enable and set Button EXTI Interrupt to the lowest priority */
        NVIC_InitStructure.NVIC_IRQChannel                   = BUTTON_IRQn[Button];
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0F;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init (&NVIC_InitStructure);
    }
}


/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *         This parameter should be: BUTTON_USER
  * @retval The Button GPIO pin value.
  */
uint32_t  STM_EVAL_PBGetState (Button_TypeDef Button)
{
    return GPIO_ReadInputDataBit (BUTTON_PORT[Button], BUTTON_PIN[Button]);
}


/* *****END OF FILE****/
