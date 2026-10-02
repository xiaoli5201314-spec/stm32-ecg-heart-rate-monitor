/*
 * ecg_pipeline.h - system integration layer.
 *
 *   DMA ISR --> ecg_ring_t --> main loop --> DSP chain --> frame builder --> UART
 *                                              |
 *                                              +--> QRS detector --> HR frames
 *
 * DSP chain, in the order it runs per sample:
 *
 *   raw ADC count
 *     -> calibration        counts -> uV at the electrode (1 mV square wave fit)
 *     -> baseline tracker   0.5 Hz high-pass, removes residual wander
 *     -> 50 Hz notch        ECG_NOTCH_STAGES cascaded biquads
 *     -> Savitzky-Golay     least-squares smoothing, R peak preserving
 *     -> QRS detector       envelope + adaptive threshold + RR validation
 */
#ifndef ECG_PIPELINE_H
#define ECG_PIPELINE_H

#include "ecg_config.h"
#include "ring_buffer.h"
#include "iir_notch.h"
#include "savgol.h"
#include "heart_rate.h"
#include "frame_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Calibration                                                         */
/* ------------------------------------------------------------------ */
typedef struct {
    ecg_real_t mv;       /* nominal peak-to-peak input at the electrode, mV */
    ecg_real_t counts;   /* measured peak-to-peak ADC count swing           */
} ecg_cal_point_t;

typedef struct {
    ecg_real_t slope_counts_per_mv;   /* least-squares gain   a  */
    ecg_real_t intercept_counts;      /* least-squares offset b  */
    ecg_real_t r2;                    /* coefficient of determination */
    ecg_real_t nonlinearity_pct_fs;   /* max |residual| / span * 100   */
    ecg_real_t afe_gain;              /* measured analog gain, V/V     */
    ecg_real_t electrode_uv_per_lsb;  /* 1000 / slope                  */
    uint8_t    n_points;
    uint8_t    valid;
} ecg_cal_t;

/* Least-squares fit of counts = a*mv + b over the 1 mV square-wave points. */
int        ecg_cal_fit(ecg_cal_t *cal, const ecg_cal_point_t *points, size_t n);
void       ecg_cal_defaults(ecg_cal_t *cal);
ecg_real_t ecg_cal_counts_to_uv(const ecg_cal_t *cal, ecg_real_t counts);

/* ------------------------------------------------------------------ */
/* Pure DSP chain                                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t            sample_rate_hz;
    ecg_cal_t           cal;
    ecg_hpf1_t          baseline[ECG_BASELINE_HP_SECTIONS];
    ecg_notch_t         notch;
    ecg_savgol_stream_t sg;
    ecg_hr_t            hr;

    ecg_real_t          last_raw_counts;
    ecg_real_t          last_dc_uv;
    ecg_real_t          last_notch_uv;
    ecg_real_t          last_uv;        /* final filtered output        */
    uint32_t            processed;
    uint32_t            sg_prime_count;
} ecg_pipeline_t;

void       ecg_pipeline_init(ecg_pipeline_t *p, uint32_t sample_rate_hz, const ecg_cal_t *cal);
void       ecg_pipeline_reset(ecg_pipeline_t *p);
/* Runs one raw ADC count through the whole chain.
 * Returns the filtered value in uV (electrode referred); *beat_out may be NULL. */
ecg_real_t ecg_pipeline_process(ecg_pipeline_t *p, uint16_t raw_count, ecg_hr_beat_t *beat_out);
void       ecg_pipeline_process_counts(ecg_pipeline_t *p, const uint16_t *counts, size_t n,
                                       ecg_real_t *out_uv);

/* ------------------------------------------------------------------ */
/* Device: DMA + ring buffer + framing + UART                          */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t isr_blocks;
    uint32_t samples_acquired;
    uint32_t samples_dropped;
    uint32_t frames_tx;
    uint32_t bytes_tx;
    uint32_t frames_dropped;
    uint32_t bytes_dropped;
    uint32_t beats;
    uint32_t ring_overflows;
} ecg_dev_stats_t;

#define ECG_TX_QUEUE_CAPACITY ECG_TX_QUEUE_SIZE   /* must stay a power of two */

typedef struct {
    ecg_pipeline_t dsp;
    ecg_ring_t     adc_ring;
    uint16_t       adc_ring_storage[ECG_RING_CAPACITY];
    uint16_t       dma_buffer[ECG_DMA_BUFFER_SIZE];
    ecg_ring_t     tx_ring;
    uint8_t        tx_ring_storage[ECG_TX_QUEUE_CAPACITY];

    uint16_t       seq;
    uint8_t        status_flags;
    uint32_t       last_hr_report_ms;
    uint32_t       last_status_ms;
    uint32_t       started_ms;
    ecg_dev_stats_t stats;

    int16_t        sample_block[ECG_FRAME_SAMPLES];
    uint32_t       sample_block_fill;
    uint8_t        frame_scratch[ECG_FRAME_MAX_SIZE];

    void         (*on_beat)(const ecg_hr_beat_t *beat, void *user);
    void          *on_beat_user;
} ecg_device_t;

int  ecg_device_init(ecg_device_t *dev);
int  ecg_device_start(ecg_device_t *dev);      /* hal_init + timer + ADC/DMA */
void ecg_device_stop(ecg_device_t *dev);

/* Interrupt context: called by the platform DMA driver. */
void ecg_device_on_dma_half(const uint16_t *samples, uint32_t count);
void ecg_device_on_dma_full(const uint16_t *samples, uint32_t count);

/* Main loop: drains the ring, filters, detects beats, emits frames and pushes
 * them out of the UART. Returns the number of filtered samples written to
 * out_uv (out_max entries available, may be NULL). */
uint32_t ecg_device_task(ecg_device_t *dev, ecg_real_t *out_uv, uint32_t out_max);
uint32_t ecg_device_flush_tx(ecg_device_t *dev);

const ecg_dev_stats_t *ecg_device_stats(const ecg_device_t *dev);
int  ecg_device_apply_calibration(ecg_device_t *dev, const ecg_cal_point_t *points, size_t n);
void ecg_device_set_beat_callback(ecg_device_t *dev,
                                  void (*cb)(const ecg_hr_beat_t *, void *), void *user);

/* ------------------------------------------------------------------ */
/* Self test / diagnostics                                             */
/* ------------------------------------------------------------------ */
/* Averages one captured plateau of raw ADC counts.  The production firmware
 * fills the array while the 1 mV calibrator drives the input; the mean is
 * taken after a 3-point median pre-filter so that a single glitch cannot bias
 * the calibration point. */
ecg_real_t ecg_cal_plateau_mean(const uint16_t *counts, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* ECG_PIPELINE_H */
