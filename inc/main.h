
/* version number;
 */
#define SW_VERSION_MAJOR        0
#define SW_VERSION_MINOR        1

#define PROTOCOL_VERSION        1

/* numbers */
#define PI_X_2                 (6.2831853f)
#define TX_BUF_SIZE            64

#define ADC_CHANNELS           2
#define ADC_IDX_FMOD           0

#define ADC_MIN_12b            0
#define ADC_MAX_12b            4095
#define ADC_SCALE_OFFSET       50
#define ADC_SCALE_MIN          (ADC_MIN_12b+ADC_SCALE_OFFSET)
#define ADC_SCALE_MAX          (ADC_MAX_12b-ADC_SCALE_OFFSET)
#define ADC_SCALE_DELTA        (ADC_SCALE_MAX-ADC_SCALE_MIN)


/* amplitude display steps (0..100%) */
#define AMPLEVEL_MIN      0
#define AMPLEVEL_MAX      8


#define MULTIPRESS_DELTA       1000          /* keypress-delay */


/* Aktivierung/Deaktivierung diverser Features
 */
#define ENABLE_DEBUG_OUTPUT

#define _ENABLE_USART_
