/*
 * hal_stub.h - PC simulation of the ECG node hardware.
 *
 * The stub replaces the three things the firmware cannot do on a PC:
 *
 *   1. the sample-rate timer + circular ADC/DMA  -> hal_sim_run() replays the
 *      exact interrupt sequence (half transfer, full transfer) at 250 Hz;
 *   2. the analog front end and the patient      -> hal_sim_adc_count()
 *      synthesises P-QRS-T + 50 Hz hum + respiration wander + white noise,
 *      applies the instrumentation-amplifier gain and the mid-rail level
 *      shift, and quantises to 12 bits;
 *   3. the uplink UART                           -> every byte handed to
 *      hal_uart_write() is appended to a capture buffer that the tests decode
 *      with the real frame synchroniser.
 *
 * The synthetic waveform is generated analytically from Gaussian pulses, so a
 * test always knows the ground truth heart rate, the exact R-peak positions
 * and the exact signal-to-noise ratio that was injected.
 */
#ifndef HAL_STUB_H
#define HAL_STUB_H

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double   sample_rate_hz;
    double   heart_rate_bpm;      /* ground truth for the test suite        */
    double   r_amp_mv;            /* R wave amplitude at the electrode      */
    double   p_amp_mv;
    double   q_amp_mv;
    double   s_amp_mv;
    double   t_amp_mv;
    double   p_offset_s;          /* relative to the R peak                 */
    double   q_offset_s;
    double   s_offset_s;
    double   t_offset_s;
    double   p_sigma_s;
    double   qrs_sigma_s;
    double   t_sigma_s;
    double   hum_amp_mv;          /* 50 Hz mains pickup at the electrode    */
    double   hum_freq_hz;
    double   wander_amp_mv;       /* respiration baseline wander            */
    double   wander_freq_hz;
    double   noise_rms_mv;        /* white noise at the electrode           */
    int      enable_hum;
    int      enable_wander;
    int      enable_noise;
    double   afe_gain;            /* instrumentation amplifier gain, V/V    */
    double   afe_ref_mv;          /* mid-rail level shift at the AFE output */
    double   adc_vref_mv;
    int      adc_bits;
    uint32_t seed;
} hal_sim_cfg_t;

typedef void (*hal_sim_task_fn)(void *user);

/* ---- configuration ---- */
void hal_sim_default_cfg(hal_sim_cfg_t *cfg);
void hal_sim_set_cfg(const hal_sim_cfg_t *cfg);
const hal_sim_cfg_t *hal_sim_get_cfg(void);
void hal_sim_reset(void);

/* ---- signal model ---- */
/* Clean ECG in mV at the electrode, t in seconds. */
double   hal_sim_ecg_mv(const hal_sim_cfg_t *cfg, double t_s);
/* Analog channel output in mV at the ADC pin (bias + gain*ecg + hum + wander
 * + noise) for sample index n. Not const: the white-noise generator advances
 * the configuration seed. */
double   hal_sim_channel_mv(hal_sim_cfg_t *cfg, uint32_t n);
/* Quantised 12-bit reading for sample index n. */
uint16_t hal_sim_adc_count(hal_sim_cfg_t *cfg, uint32_t n);
/* Exact R-peak time of beat number k (k = 0 is the first beat after t = 0). */
double   hal_sim_beat_time_s(const hal_sim_cfg_t *cfg, uint32_t k);
/* True sample index of the R peak closest to sample n, or -1 when the nearest
 * R peak is further away than `span` samples. */
long     hal_sim_nearest_r_peak(const hal_sim_cfg_t *cfg, uint32_t n, uint32_t span);

uint32_t hal_sim_rng(uint32_t *state);
double   hal_sim_gauss(hal_sim_cfg_t *cfg);

/* Fill `out` with `n` raw counts for a steady input of `mv` millivolts at the
 * electrode. The production firmware calls this twice per calibration point
 * (the high and the low plateau of the standard square wave) and takes the
 * difference as the peak-to-peak count swing. */
void     hal_sim_capture_plateau(hal_sim_cfg_t *cfg, double mv, uint16_t *out, size_t n);

/* ---- DMA / timer emulation ---- */
/* Push n_samples through the registered half/full callbacks using the buffer
 * that was armed with hal_adc_start_dma(). `task`, when not NULL, is called
 * every `task_period_samples` samples to emulate the main loop.
 * Returns the number of DMA callbacks fired. */
uint32_t hal_sim_run(hal_sim_cfg_t *cfg, uint32_t n_samples,
                     uint32_t task_period_samples, hal_sim_task_fn task, void *user);
uint32_t hal_sim_isr_count(void);
uint32_t hal_sim_timer_starts(void);
uint32_t hal_sim_adc_starts(void);

/* ---- observations ---- */
const uint8_t *hal_sim_uart_buffer(void);
uint32_t       hal_sim_uart_len(void);
void           hal_sim_uart_reset(void);
/* Limit how many bytes the mock UART accepts per call (0 = unlimited). */
void           hal_sim_uart_set_chunk(uint32_t bytes);
int            hal_sim_pin_level(hal_pin_t pin);
uint32_t       hal_sim_tick(void);
void           hal_sim_advance_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* HAL_STUB_H */
