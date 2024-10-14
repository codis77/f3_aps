#include "stm32f30x_gpio.h"
#include "main.h"
#include "adc.h"


/* configure the Timer3 to generate the periodic ADC trigger
 * improved version, direct trigger without interrupt;
 * the actual association with the ADC happens in the ADC init
 * function, where TIM3 is selected as trigger source
 */
void  SampleTimerConfig (uint32_t smplperiod)
{
//  uint16_t                 PrescalerValue = 0;
#ifdef USE_TIM_INTERRUPT
    NVIC_InitTypeDef         NVIC_InitStructure;
#endif
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);   /* TIM3 clock enable */

    /* TIM3 Configuration as trigger for the ADC */
    TIM_DeInit (TIM3);
    TIM_TimeBaseStructInit (&TIM_TimeBaseStructure);
    TIM_OCStructInit (&TIM_OCInitStructure);

#ifdef USE_TIM_INTERRUPT
    /* enable TIM3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

    /* Time base configuration */
#warning "TIM3-Prescaler calculation changed !"
#if TIM3_BASE_10MHz
    TIM_TimeBaseStructure.TIM_Prescaler     = (SystemCoreClock / 10.000.000) - 1;   // 10 MHz
#else
    TIM_TimeBaseStructure.TIM_Prescaler     = 3;               // (4 - 1) --> 18 MHz = 55.55 ns
#endif
    TIM_TimeBaseStructure.TIM_Period        = smplperiod - 1;  // 18MHz/period = smpl.freq
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit (TIM3, &TIM_TimeBaseStructure);
    TIM_SelectOutputTrigger (TIM3, TIM_TRGOSource_Update);
    TIM_ARRPreloadConfig (TIM3, ENABLE);

    /* Timer and interrupt enable */
#ifdef USE_TIM_INTERRUPT
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
#endif
    TIM_Cmd (TIM3, ENABLE);
}

