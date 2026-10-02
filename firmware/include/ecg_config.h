/*
 * ecg_config.h - central build configuration for the ECG acquisition node.
 *
 * Every tunable that couples the analog front end, the timer/DMA sampler, the
 * digital filter chain and the uplink protocol lives here so that the same
 * translation units build for the STM32 target and for the PC test harness.
 *
 * Target : STM32F103C8T6 (Cortex-M3, 72 MHz, 12-bit ADC, 2 x DMA)
 * Host   : any C99 compiler (used by the unit-test suite)
 */
#ifndef ECG_CONFIG_H
#define ECG_CONFIG_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* Math constants (M_PI is not part of ISO C99, so it is defined here)  */
/* ------------------------------------------------------------------ */
#define ECG_PI      3.14159265358979323846
#define ECG_TWO_PI  6.28318530717958647692
#define ECG_DEG2RAD (ECG_PI / 180.0)

/* ------------------------------------------------------------------ */
/* Fixed-point / floating-point policy                                 */
/* ------------------------------------------------------------------ */
/*
 * The filter kernels are written against ecg_real_t so the same code runs as
 * double on the host (bit-exact reference) and as single precision on a
 * Cortex-M4F/M7 with an FPU. Only the *design* routines (least-squares
 * coefficient solving) are pinned to double.
 */
#ifdef ECG_USE_SINGLE_PRECISION
typedef float ecg_real_t;
#define ECG_SQRT(x)  sqrtf(x)
#define ECG_FABS(x)  fabsf(x)
#else
typedef double ecg_real_t;
#define ECG_SQRT(x)  sqrt(x)
#define ECG_FABS(x)  fabs(x)
#endif

/* ------------------------------------------------------------------ */
/* Acquisition timing                                                  */
/* ------------------------------------------------------------------ */
/* TIM2 update event is the single time base of the whole system.      */
#define ECG_SAMPLE_RATE_HZ      250u
#define ECG_SAMPLE_PERIOD_US    (1000000u / ECG_SAMPLE_RATE_HZ)   /* 4000 us */
#define ECG_TIM2_CLOCK_HZ       72000000u
/* 72 MHz / (250 Hz * 1) -> prescaler 287, period 999 (72e6/288/1000 = 250) */
#define ECG_TIM2_PRESCALER      (ECG_TIM2_CLOCK_HZ / (ECG_SAMPLE_RATE_HZ * 1000u) - 1u)
#define ECG_TIM2_PERIOD         999u
#define ECG_TIM2_TICK_HZ        1000u   /* update event = internal trigger rate */

/* ------------------------------------------------------------------ */
/* Analog channel + ADC scaling                                        */
/* ------------------------------------------------------------------ */
#define ECG_ADC_BITS            12u
#define ECG_ADC_FULL_SCALE      (1u << ECG_ADC_BITS)          /* 4096 counts   */
#define ECG_ADC_VREF_MV         3300.0   /* external REF3033 on VDDA         */
#define ECG_ADC_LSB_UV          (ECG_ADC_VREF_MV * 1000.0 / (double)ECG_ADC_FULL_SCALE)

#define ECG_AFE_GAIN_NOMINAL    1000.0   /* INA front end, V/V               */
#define ECG_AFE_REF_MV          1650.0   /* mid-rail level-shift, mV         */
/* uV at the electrode per ADC count with the nominal gain */
#define ECG_ELECTRODE_UV_PER_LSB (ECG_ADC_LSB_UV / ECG_AFE_GAIN_NOMINAL)

/* Analog channel corner frequencies (documented, mirrored in docs/DESIGN.md) */
#define ECG_AFE_HP_FIRST_HZ     0.05    /* two-pole 0.05 Hz anti-saturation */
#define ECG_AFE_HP_SECOND_HZ    0.5     /* second high-pass stage          */
#define ECG_AFE_LP_HZ           100.0   /* 2nd order Sallen-Key low-pass   */
#define ECG_AFE_RLD_HZ          150.0   /* right-leg-drive loop bandwidth  */

/* ------------------------------------------------------------------ */
/* Digital filter chain                                                */
/* ------------------------------------------------------------------ */
#define ECG_NOTCH_F0_HZ         50.0    /* mains fundamental                */
#define ECG_NOTCH_Q             8.0     /* notch quality factor             */
#define ECG_NOTCH_STAGES        2u      /* cascaded biquads (deeper stopband)*/
#define ECG_NOTCH_MAX_STAGES    4u

#define ECG_BASELINE_HP_HZ      0.5     /* residual wander removal corner   */
#define ECG_BASELINE_HP_SECTIONS 2u     /* cascaded 1-pole sections (=2 poles)*/

/* Default Savitzky-Golay operating point, selected by the parameter sweep
 * implemented in test_filters.c (see docs/DESIGN.md section 5).          */
#define ECG_SG_DEFAULT_ORDER    4u
#define ECG_SG_DEFAULT_WINDOW   17u
#define ECG_SG_MAX_ORDER        6u
#define ECG_SG_MAX_WINDOW       41u

/* ------------------------------------------------------------------ */
/* Heart-rate detector                                                 */
/* ------------------------------------------------------------------ */
#define ECG_HR_INTEG_WINDOW     37u     /* 150 ms moving-average integrator */
#define ECG_HR_PEAK_SEARCH      31u     /* +-125 ms R-peak refinement window*/
#define ECG_HR_RR_HISTORY       5u      /* accepted RR intervals in the median */
#define ECG_HR_RR_SERIES        128u    /* RR series kept for the HRV statistics */
#define ECG_HR_RR_LEARN         3u      /* intervals needed before RR outliers  */
                                        /* are rejected (avoids prior dead-lock)*/
#define ECG_HR_REFRACTORY_MS    200u
#define ECG_HR_TWAVE_MS         360u
#define ECG_HR_RR_MIN_MS        300u    /* 200 BPM ceiling                  */
#define ECG_HR_RR_MAX_MS        2000u   /* 30 BPM floor                     */
#define ECG_HR_ECTOPIC_PCT      40u     /* RR deviating more than this is   */
                                        /* flagged and kept out of the mean */
#define ECG_HR_BPM_MIN          30.0
#define ECG_HR_BPM_MAX          220.0

/* ------------------------------------------------------------------ */
/* Ring buffer / DMA                                                   */
/* ------------------------------------------------------------------ */
#define ECG_DMA_BLOCK_SIZE      64u     /* counts per half-transfer          */
#define ECG_DMA_BUFFER_SIZE     (ECG_DMA_BLOCK_SIZE * 2u)
#define ECG_RING_CAPACITY       1024u   /* power of two, samples              */
#define ECG_MAX_SAMPLES_PER_TICK 256u

/* ------------------------------------------------------------------ */
/* Uplink protocol                                                     */
/* ------------------------------------------------------------------ */
#define ECG_PROTO_VERSION       1u
#define ECG_FRAME_HDR0          0xA5u
#define ECG_FRAME_HDR1          0x5Au
#define ECG_FRAME_MAX_PAYLOAD   255u
/* hdr(2) + type(1) + len(1) + seq(2) + crc(2) */
#define ECG_FRAME_OVERHEAD      8u
#define ECG_FRAME_MAX_SIZE      (ECG_FRAME_OVERHEAD + ECG_FRAME_MAX_PAYLOAD)
#define ECG_FRAME_SAMPLES       24u     /* samples packed per ADC frame      */
#define ECG_UPLINK_UART_BAUD    460800u
#define ECG_TX_QUEUE_SIZE       2048u

/* ------------------------------------------------------------------ */
/* Calibration                                                         */
/* ------------------------------------------------------------------ */
#define ECG_CAL_MAX_POINTS      8u
#define ECG_CAL_STEP_MV         1.0     /* 1 mV standard square wave         */
#define ECG_CAL_SETTLE_MS       50u
#define ECG_CAL_AVERAGE_SAMPLES 40u

/* ------------------------------------------------------------------ */
/* Diagnostics                                                         */
/* ------------------------------------------------------------------ */
#define ECG_ENABLE_ASSERT       1
#if ECG_ENABLE_ASSERT
void ecg_assert_failed(const char *file, int line);
#define ECG_ASSERT(cond) do { if (!(cond)) { ecg_assert_failed(__FILE__, __LINE__); } } while (0)
#else
#define ECG_ASSERT(cond) ((void)0)
#endif

#endif /* ECG_CONFIG_H */
