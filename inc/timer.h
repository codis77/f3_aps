
/* TIM3 interrupt request handler
 */
#define CCR1_VALUE_44K1             23     // Timerwert fuer 44.100 kHz Audioabtastrate
#define CCR1_VALUE_22K05            45     // Timerwert fuer 22.050 kHz Audioabtastrate
#define CCR1_VALUE_11K025           91     // Timerwert fuer 11.025 kHz Audioabtastrate
#define CCR1_VALUE_8K0             125     // Timerwert fuer  8.000 kHz Audioabtastrate
#define CCR1_VALUE_4K0             250     // Timerwert fuer  4.000 kHz Audioabtastrate
#define CCR1_VALUE_2K0             500     // Timerwert fuer  2.000 kHz Audioabtastrate
#define CCR1_VALUE_1K0            1000     // Timerwert fuer  1.000 kHz Audioabtastrate

/* TIM3 TRGO update rate constants;
 * Periodendauer-Werte fuer die o.g. Abtastfrequenzen
 */
#if TIM3_BASE_10MHz
#define TIM3_TRGO_VALUE_44K1       227     // 44.100 kHz (226.8)
#define TIM3_TRGO_VALUE_22K05      454     // 22.050 kHz (453.51)
#define TIM3_TRGO_VALUE_11K025     907     // 11.025 kHz (907.03)
#define TIM3_TRGO_VALUE_8K0       1250     // 8.000 kHz  ( * )
#define TIM3_TRGO_VALUE_4K0       2500     // 4.000 kHz  ( * )
#define TIM3_TRGO_VALUE_2K0       5000     // 2.000 kHz  ( * )
#define TIM3_TRGO_VALUE_1K0      10000     // 1.000 kHz  ( * )
#else // 16MHz = 64MHz / 4
  #define TIM3_TRGO_VALUE_44K1     363  /* 362.81  */ //  408   // 44.100 kHz (408.16)
  #define TIM3_TRGO_VALUE_22K05    726  /* 725.62  */ //  816   // 22.050 kHz (816.32)
  #define TIM3_TRGO_VALUE_11K025  1451  /* 1451.25 */ //  1633  // 11.025 kHz (1632.65)
  #define TIM3_TRGO_VALUE_8K0     2000                //  2250  // 8.000 kHz  ( * )
  #define TIM3_TRGO_VALUE_4K0     4000                //  4500  // 4.000 kHz  ( * )
  #define TIM3_TRGO_VALUE_2K0     8000                //  2.000 kHz
  #define TIM3_TRGO_VALUE_1K0    16000                //  1.000 kHz
#endif

/* -------- Prototypen --------
 */
void       SampleTimerConfig   (uint32_t smplperiod);
