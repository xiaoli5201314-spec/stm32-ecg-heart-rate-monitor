/*
 * heart_rate.h - QRS detection and heart-rate statistics.
 *
 * Detector chain (all original code, classic QRS-enhancement pipeline):
 *
 *   x[n] --> 2-sample difference  d[n] = x[n] - x[n-2]        (5..20 Hz boost)
 *        --> square              e[n] = d[n]^2
 *        --> 150 ms moving average  env[n]                          (envelope)
 *        --> adaptive threshold on env[n]
 *        --> refractory + T-wave discrimination
 *        --> R-peak refinement on |x| inside +-125 ms
 *        --> RR interval validation (range + ectopic outlier rejection)
 *        --> BPM from the median of the last 5 accepted RR intervals
 *
 * The two running levels spki (signal peak) and npki (noise peak) are updated
 * with a first order IIR every time the envelope crosses a local maximum:
 *
 *     spki = 0.125*peak + 0.875*spki        (peak >= threshold)
 *     npki = 0.125*peak + 0.875*npki        (peak <  threshold)
 *     thr  = npki + 0.25*(spki - npki)
 *
 * If no beat is found for longer than 1.66 * RR_average the threshold is
 * halved for the next samples (search-back), which is what recovers beats
 * whose amplitude collapsed because of electrode motion.
 */
#ifndef HEART_RATE_H
#define HEART_RATE_H

#include "ecg_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ECG_BEAT_NONE = 0,
    ECG_BEAT_NORMAL,
    ECG_BEAT_ECTOPIC,      /* accepted but RR is far off the running median */
    ECG_BEAT_RECOVERED     /* found by the threshold search-back            */
} ecg_beat_class_t;

typedef struct {
    uint32_t         sample_index;   /* absolute index of the refined R peak */
    uint32_t         rr_samples;
    ecg_real_t       rr_ms;
    ecg_real_t       bpm_instant;
    ecg_real_t       bpm_average;
    ecg_real_t       amplitude_uv;   /* |x| at the R peak                    */
    ecg_real_t       envelope;       /* env value that triggered the beat    */
    ecg_beat_class_t klass;
    int              rr_accepted;    /* 1 when used for the RR statistics    */
} ecg_hr_beat_t;

typedef struct {
    /* configuration */
    ecg_real_t fs_hz;
    uint32_t   refractory_samples;
    uint32_t   twave_samples;
    uint32_t   rr_min_samples;
    uint32_t   rr_max_samples;
    /* input delay line */
    ecg_real_t x_hist[4];            /* x[n-1] .. x[n-4]                     */
    int        x_primed;             /* derivative primed on the first sample */
    /* envelope integrator */
    ecg_real_t integ[ECG_HR_INTEG_WINDOW];
    uint32_t   integ_index;
    ecg_real_t integ_sum;
    /* peak search history */
    ecg_real_t peak_hist[ECG_HR_PEAK_SEARCH];
    uint32_t   peak_index;
    /* adaptive levels */
    ecg_real_t spki, npki, threshold, threshold_back;
    ecg_real_t env_prev, env_prev2;
    ecg_real_t warmup_max;
    uint32_t   warmup_samples;
    uint32_t   warmup_settle;        /* samples ignored before the learning    */
    int        warmup_done;
    /* beat bookkeeping */
    uint32_t   sample_count;
    uint32_t   last_beat_sample;
    int        have_last_beat;
    ecg_real_t last_beat_env;
    ecg_real_t rr_series[ECG_HR_RR_SERIES];  /* ring of accepted RR intervals  */
    uint32_t   rr_series_count;
    uint32_t   rr_series_write;
    ecg_real_t rr_median;                    /* median of the last 5 accepted  */
    ecg_real_t bpm_avg;
    /* statistics */
    uint32_t   beats_total;
    uint32_t   beats_rejected;
    uint32_t   beats_ectopic;
    uint32_t   beats_recovered;
    uint32_t   beats_twave_rejected;
    ecg_real_t peak_uv_max;
    ecg_real_t noise_uv_rms;         /* rough noise estimate for the SQI    */
    ecg_real_t signal_uv_rms;
} ecg_hr_t;

typedef struct {
    uint32_t   beats;
    ecg_real_t mean_rr_ms;
    ecg_real_t sdnn_ms;      /* standard deviation of RR intervals  */
    ecg_real_t rmssd_ms;     /* root mean square of successive diffs */
    ecg_real_t pnn50_pct;    /* percentage of |dRR| > 50 ms          */
    ecg_real_t bpm_mean;
} ecg_hr_hrv_t;

void       ecg_hr_init(ecg_hr_t *hr, ecg_real_t fs_hz);
void       ecg_hr_reset(ecg_hr_t *hr, ecg_real_t fs_hz);
/* Feed one filtered sample. Returns ECG_BEAT_NONE or the beat class; when a
 * beat is produced *beat_out (may be NULL) is filled in. */
ecg_beat_class_t ecg_hr_process(ecg_hr_t *hr, ecg_real_t x, ecg_hr_beat_t *beat_out);
/* Convenience for offline analysis. */
void       ecg_hr_process_block(ecg_hr_t *hr, const ecg_real_t *x, size_t n,
                                ecg_hr_beat_t *beats, size_t max_beats, size_t *n_beats);
ecg_real_t ecg_hr_bpm(const ecg_hr_t *hr);
/* Quality index 0..100 derived from the SPKI/NPKI ratio. */
uint8_t    ecg_hr_quality(const ecg_hr_t *hr);
void       ecg_hr_get_hrv(const ecg_hr_t *hr, ecg_hr_hrv_t *hrv);

#ifdef __cplusplus
}
#endif

#endif /* HEART_RATE_H */
