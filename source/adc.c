
#include "stm32f30x_conf.h"
#include "stm32f3_discovery.h"
#include "main.h"

/* Private define --------------------------------------*/
#define ADC_CDR_ADDRESS    ((uint32_t)0x5000030C)

#define ENABLE_ADC_CALIBRATION

/* PA2 (ADCIN_3) und PA3 (ADCIN_4) als ADC-Eingaenge konfigurieren;
 * ADC_IN3 (PA2) fuer Audiodaten, ADC_IN4 (PA3) als Stellpotentiometer
 * setzt eine erfolgte Initialisierung des ADC-DMA voraus;
 * die Timer-getriggerte Wandlung wird am Ende der Initialisierung angestossen,
 * die ADC-Werte werden in der ADC-Interruptroutine ausgewertet;
 */
void  initADC (void)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
    ADC_InitTypeDef        ADC_InitStructure;
    ADC_CommonInitTypeDef  ADC_CommonInitStructure;
    NVIC_InitTypeDef       NVIC_InitStructure;
    volatile uint32_t      i;
    volatile uint16_t      calibration_value __attribute__((__unused__));

    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOA, ENABLE);

    /* Configure ADC Channels 3 & 4 as analog input */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init (GPIOA, &GPIO_InitStructure);

    RCC_ADCCLKConfig (RCC_ADC12PLLCLK_Div4);              /* configure ADC clock */
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_ADC12, ENABLE);  /* enable ADC1 clock   */

    // try setting to the AHB clock instead of directly from PLL
    ADC_DeInit(ADC1);
    ADC1_2->CCR |= ADC12_CCR_CKMODE;  // set both CKMODE[1:0] bits

    /* Calibration procedure;
     * a delay of 10 s is required after turning on the voltage regulator */
    ADC_VoltageRegulatorCmd (ADC1, ENABLE);
    for (i=0; i<100000; i++);          //  Delay(10);

#ifdef ENABLE_ADC_CALIBRATION
    ADC_SelectCalibrationMode (ADC1, ADC_CalibrationMode_Single);
    ADC_StartCalibration (ADC1);
#warning "Verzoegerung fuer ADC_VoltageRegulatorCmd() nachpruefen !"

    while (ADC_GetCalibrationStatus (ADC1) != RESET);
    calibration_value = ADC_GetCalibrationValue (ADC1);
#endif

    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Clock            = ADC_Clock_AsynClkMode;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_DMAMode          = ADC_DMAMode_Circular;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = 2;
    ADC_CommonInit (ADC1, &ADC_CommonInitStructure);

    ADC_StructInit (&ADC_InitStructure);
    ADC_InitStructure.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Disable;
    ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
#ifdef USE_TIM_INTERRUPT     /** SW-Trigger aus der TIM3 Interrupthandler-Routine */
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0; // irrelevant
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
#else                        /** HW-Trigger direkt von TIM3 */
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_4; // entspricht ADC_ExternalTrigConv_T3_TRGO, TIM3_TRGO
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
#endif
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_OverrunMode           = ADC_OverrunMode_Disable;
    ADC_InitStructure.ADC_AutoInjMode           = ADC_AutoInjec_Disable;
    ADC_InitStructure.ADC_NbrOfRegChannel       = 2;
    ADC_Init (ADC1, &ADC_InitStructure);

    /* ADC1 regular channel3/4 configuration */
    ADC_RegularChannelConfig (ADC1, ADC_Channel_3, 1, ADC_SampleTime_181Cycles5);
    ADC_RegularChannelConfig (ADC1, ADC_Channel_4, 2, ADC_SampleTime_181Cycles5);

    /* Configure and enable ADC1 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init (&NVIC_InitStructure);

    ADC_Cmd (ADC1, ENABLE);                            /* Enable ADC1    */
    while (!ADC_GetFlagStatus (ADC1, ADC_FLAG_RDY));   /* wait for ADRDY */

    /* enable EOC interrupt */
    ADC_ITConfig (ADC1, ADC_IT_EOC, ENABLE);

    ADC_StartConversion (ADC1);                        /* Start Conversion */
}



void  startAdcConversion (void)
{
    /* Start ADC1 Software Conversion */
    ADC_StartConversion (ADC1);
}
