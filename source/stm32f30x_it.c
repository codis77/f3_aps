/**
  ******************************************************************************
  * @file    stm32f30x_it.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_it.h"
#include "stm32f3_discovery.h"
#include "stm32f30x_conf.h"
#include "main.h"
#include "timer.h"
#include <math.h>

#ifdef USE_TIM_INTERRUPT
  #warning "Timer3 operation with interrupt !"
#else
  #warning "Timer3 operating without interrupts / direct ADC trigger"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UART_ERR_INT_MASK     0x0000015F        /* UART error interrupt flags */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint32_t      TickCount = 0;

extern volatile uint32_t      TimingCounter;
extern volatile uint32_t      setOutput;
extern volatile uint32_t      transmitRunning;

extern volatile char          txBuffer[TX_BUF_SIZE];
extern volatile uint32_t      txSendCount;
extern volatile uint32_t      txIndex;

extern volatile uint32_t      newAdcValue;
extern volatile uint32_t      AudioValue;

extern volatile uint32_t      ubtnChange;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/** handle NMI exceptions
  */
void NMI_Handler(void)
{
}

/** handle Hard Fault exceptions
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/** handle Memory Manage exceptions
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/** handle Bus Fault exceptions
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/** handle Usage Fault exceptions
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/** handle SVCall exceptions
  */
void SVC_Handler(void)
{
}

/** handle Debug Monitor exceptions
  */
void DebugMon_Handler(void)
{
}

/** handles PendSVC exceptions
  */
void PendSV_Handler(void)
{
}

/** -- SysTick Handler --
  */
void SysTick_Handler(void)
{
    TimingCounter++;
}


/******************************************************************************/
/*                 STM32F30x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f30x.s).                                            */
/******************************************************************************/

/** EXTI0_IRQ Handler; rising only
  */
void EXTI0_IRQHandler(void)
{
    if (STM_EVAL_PBGetState(BUTTON_USER) != RESET)
        ubtnChange = 1;

    // cycle through output amplitude levels
    if (ubtnChange)
    {
    }

    /* Clear the EXTI line pending bit */
    EXTI_ClearITPendingBit (USER_BUTTON_EXTI_LINE);
}


/** TIM3 interrupt handler
  */
#ifdef USE_TIM_INTERRUPT
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus (TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit (TIM3, TIM_IT_Update);  /* clear pending flag   */
/** Debug: zum Messen des Timings eine LED umschalten (LED5, orange)
  */
STM_EVAL_LEDToggle (LED5);
        ADC_StartConversion (ADC1);                   /* Start ADC conversion */
    }
}
#endif


/** USART interrupt handler
 */
void  USART1_IRQHandler (void)
{
//volatile uint32_t delay;

    if (USART_GetITStatus (USART1, USART_IT_TXE) == SET)
    {
        if (txIndex < txSendCount)
            USART1->TDR = txBuffer[txIndex++];

        if (txIndex >= txSendCount)
        {
            txIndex = 0;
            USART_ITConfig (USART1, USART_IT_TXE, DISABLE);
            USART1->ICR     = 0x000FFFFF;
            transmitRunning = 0;
        }
    }

    if (USART_GetITStatus (USART1, USART_IT_RXNE) == SET)
    {
        uint32_t  dmy __attribute__((__unused__));

        dmy = (USART1->RDR & (uint16_t) 0x01FF);
        return;
    }
#if 0
#define FULL_ICR_MASK  0x000FFFFF
    /* Q&D - Fehlerbehandlung */
    if (USART1->ISR & UART_ERR_INT_MASK)
    {
        uint32_t  dummy;
        dummy        = USART1->ISR;
//      USART3->ICR &= ~UART_ERR_INT_MASK;
        USART1->ICR  = 0x000FFFFF;
        dummy        = USART1->RDR;
    }
#endif
}



extern volatile uint32_t      newAdcValue;
extern volatile uint32_t      apValue;

/** ADC1/2 Interruptroutine;
 * Einlesen der ADC-Werte, Ausgabe der DAC-Werte
 */
void  ADC1_2_IRQHandler (void)
{
    /* end of sequence -> ADC_IN4 (PA3) = Potentiometerdaten sind da */
    if (ADC_GetITStatus (ADC1, ADC_IT_EOS) != RESET)
    {
        apValue = ADC1->DR & (0x00FFFF);
        ADC_ClearITPendingBit (ADC1, ADC_IT_EOS);
    }

    /* EOC without EOS --> audio data (not used !) */
    else if (ADC_GetITStatus (ADC1, ADC_IT_EOC) != RESET)
    {
#if 0
        STM_EVAL_LEDToggle (LED4); /** Debug: zum Messen LED umschalten (LED4, blau) */
#endif
        apValue = ADC1->DR & (0x00FFFF);
    }

    /* ignore other interrupts, delete flags */
    if (ADC1->ISR)
        ADC1->ISR |= 0x07FF;
}

/* ***** END OF FILE *****
 */
