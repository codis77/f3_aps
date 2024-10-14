/*********************************************************************
*
* File :         main.c
*
*        infraSensor
*
* An application to collect infrasound data via the 2SMPP-03 analogue
* pressure sensor, and store them on a PC atached via RS232 line.
* The application is based upon the STM32F3 Discovery board, with
* the pressure sensor attached to the ADC input via glue logic
* (opamp preamlifier).
* 
* The sensor is interfaced via ADC, and 12-bit values representing
* a pressure range of -50P ... +50P are read outcyclically.
* The default sampling rate is 1kHz, and the default RS232 connection
* settings are 8N1@115200bps
*
**********************************************************************
*/

#include <math.h>
#include "stm32f30x_conf.h"
#include "stm32f3_discovery.h"
#include "main.h"
#include "adc.h"
#include "timer.h"


/* define ------------------------------------------------------------*/

/* typedefs ----------------------------------------------------------*/

/* global variables --------------------------------------------------*/
volatile uint32_t      TimingCounter   = 0;
volatile uint32_t      newAdcValue     = 0;    // "ready" flag
volatile uint32_t      apValue         = 0;    // actual ADC value
volatile uint32_t      transmitRunning = 0;

volatile char          txBuffer[TX_BUF_SIZE];
volatile uint32_t      txSendCount     = 0;
volatile uint32_t      txIndex         = 0;

volatile uint32_t      ubtnChange      = 0;  /* Flag fuer 'Userbutton' */

/* external variables -------------------------------------------------*/

/* private variables --------------------------------------------------*/

/* Private function prototypes ----------------------------------------*/
static void    initUSART    (void);
static void    sendHeader   (uint16_t sampleRate);
static void    sendDataItem (uint16_t data);
static void    evb_LedsInit (void);
static void    evbLedsOn    (void);
static void    evbLedsOff   (void);
static void    pbInit       (void);


//uint16_t  outValBuf[1000];

int  main (void)
{
    uint32_t           fs, i;
    uint16_t           tVal;
    RCC_ClocksTypeDef  RCC_Clocks;

    /* SysTick every millisecond */
    RCC_GetClocksFreq (&RCC_Clocks);
    fs = RCC_Clocks.HCLK_Frequency / 1000;
    SystemCoreClockUpdate ();
    SysTick_Config (fs);

    /* configure remaining periphery */
    initUSART ();
    evb_LedsInit ();
    pbInit ();
    initADC ();
    SampleTimerConfig (TIM3_TRGO_VALUE_1K0);   /* 1kHz sample rate */


    /* main loop;
     * get latest ADC value, format and send it via USART;
     */
    while (1)
    {
        if (newAdcValue)
        {
            newAdcValue = 0;
            tVal = (uint16_t) apValue;
            evbLedsOn ();
            sendDataItem (tVal);
            evbLedsOff ();
        }
    }
}



/* Configures the USART Peripheral;
 * using UART1 at escape routes PC4=TX, PC5=RX
 * (not UART3 at escape routes PB10=TX, PB11=RX)
 */
static void  initUSART (void)
{
    USART_InitTypeDef  USART_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOC, ENABLE);

    /* Enable USART clock */
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE);

    /* Connect PXx to USARTx_Tx and USARTx_Rx */
    GPIO_PinAFConfig (GPIOC, GPIO_PinSource4, GPIO_AF_7);
    GPIO_PinAFConfig (GPIOC, GPIO_PinSource5, GPIO_AF_7);

    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init (GPIOC, &GPIO_InitStructure);

    /* USART1-Konfiguration fuer die Kommunikation zum Hauptprozessor
     *  - 115200 Baud, 8 Bits, 1 Stopbit, keine Paritaet, RTS/CTS aus,
     *  - Senden und Empfangen
     */
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init (USART1, &USART_InitStructure);

    /* NVIC configuration; configure the Priority Group to 2 bits */
    NVIC_PriorityGroupConfig (NVIC_PriorityGroup_2);

    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init (&NVIC_InitStructure);

    /* USART und Interrupt freigeben */
    USART_ITConfig (USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd (USART1, ENABLE);
}



/* LEDs auf dem Discovery-Board initialisieren
 */
static void  evb_LedsInit (void)
{
    /* Initialize LEDs on the STM32F3-Discovery board */
    STM_EVAL_LEDInit (LED3);    /* rot    */
    STM_EVAL_LEDInit (LED4);    /* blau   */
    STM_EVAL_LEDInit (LED5);    /* orange */
    STM_EVAL_LEDInit (LED6);    /* gruen  */
    STM_EVAL_LEDInit (LED7);    /* gruen  */
    STM_EVAL_LEDInit (LED8);    /* orange */
    STM_EVAL_LEDInit (LED9);    /* blau   */
    STM_EVAL_LEDInit (LED10);   /* rot    */
}



/* alle User-LEDs auf dem Discovery-Board ausschalten
 */
static void  evbLedsOff (void)
{
    STM_EVAL_LEDOff (LED3);
    STM_EVAL_LEDOff (LED4);
    STM_EVAL_LEDOff (LED5);
    STM_EVAL_LEDOff (LED6);
    STM_EVAL_LEDOff (LED7);
    STM_EVAL_LEDOff (LED8);
    STM_EVAL_LEDOff (LED9);
    STM_EVAL_LEDOff (LED10);
}



/* alle User-LEDs auf dem Discovery-Board einschalten
 */
static void  evbLedsOn (void)
{
    STM_EVAL_LEDOn (LED3);
    STM_EVAL_LEDOn (LED4);
    STM_EVAL_LEDOn (LED5);
    STM_EVAL_LEDOn (LED6);
    STM_EVAL_LEDOn (LED7);
    STM_EVAL_LEDOn (LED8);
    STM_EVAL_LEDOn (LED9);
    STM_EVAL_LEDOn (LED10);
}



/* Amplitudenanzeige aktualisieren;
 * Reihenfolge :
 *   0 -> 1 -> 2 ->  3 -> 4 -> 5 -> 6 -> 7 -> 8
 *   - ->LD4->LD6->LD8->LD10->LD9->LD7->LD5->LD3
 *   0%  12%  25%  37%   50%  63%  75%  87%  100%  
 */
static void  setAmpLevelDisplay (uint32_t aLevel)
{
    // ON-Sequenz
    if (aLevel > 0)
        STM_EVAL_LEDOn (LED4);
    if (aLevel > 1)
        STM_EVAL_LEDOn (LED6);
    if (aLevel > 2)
        STM_EVAL_LEDOn (LED8);
    if (aLevel > 3)
        STM_EVAL_LEDOn (LED10);
    if (aLevel > 4)
        STM_EVAL_LEDOn (LED9);
    if (aLevel > 5)
        STM_EVAL_LEDOn (LED7);
    if (aLevel > 6)
        STM_EVAL_LEDOn (LED5);
    if (aLevel > 7)
        STM_EVAL_LEDOn (LED3);

    // OFF-Sequenz
    if (aLevel < 8)
        STM_EVAL_LEDOff (LED3);
    if (aLevel < 7)
        STM_EVAL_LEDOff (LED5);
    if (aLevel < 6)
        STM_EVAL_LEDOff (LED7);
    if (aLevel < 5)
        STM_EVAL_LEDOff (LED9);
    if (aLevel < 4)
        STM_EVAL_LEDOff (LED10);
    if (aLevel < 3)
        STM_EVAL_LEDOff (LED8);
    if (aLevel < 2)
        STM_EVAL_LEDOff (LED6);
    if (aLevel < 1)
        STM_EVAL_LEDOff (LED4);
}



/* init "User" button
 */
static void  pbInit (void)
{
    STM_EVAL_PBInit (BUTTON_USER, BUTTON_MODE_EXTI);
}



/* send data header;
 * sequential, access USART1 directly
 */
static void  sendHeader (uint16_t sampleRate)
{
    int  sl, index;

    sl = index = 0;

    sl = sprintf (txBuffer, "#infra_%d @%hu\n", PROTOCOL_VERSION, sampleRate);
    if (sl <= 0)  // an unlikely sprintf() error
        return;

    // send string in a busy loop
    while (index < sl)
    {
        USART1->TDR = txBuffer[index++];
        while ((USART1->ISR & USART_FLAG_TXE)==0);
    }
}


/* initialize the transmission of a data item via serial line;
 * function is executed in "real time", and supposedly finishes
 * long before the 6.6ms sample cycle (ca. 10Byte/ms throughput);
 * otherwise Tx data could overlap / be corrupted;
 */
static void  sendDataItem (uint16_t data)
{
    sprintf (txBuffer, "%hu\n", data);

    // initialize UART TXE interrupt, and send first character
    USART1->CR1 |= USART_CR1_TXEIE;
    txIndex      = 1;
    USART1->TDR  = txBuffer[0];
}
