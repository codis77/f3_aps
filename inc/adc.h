/*
 * ---- Interface-Datei fuer adc.c ----
 */
#include "main.h"


/* - - - Daten - - -
 */
extern volatile uint16_t   adcValues[ADC_CHANNELS];


/* - - - Prototypen - - -
 */
extern void   initADC     (void);
extern void   initADC_DMA (void);

