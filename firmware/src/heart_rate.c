/*
 * heart_rate.c - QRS detection, RR validation and heart-rate statistics.
 *
 * See heart_rate.h for the block diagram.  Every stage below is written from
 * the standard QRS-enhancement idea (derivative -> square -> moving-window
 * integration -> adaptive threshold); the peak refinement, the T-wave
 * discrimination and the RR outlier logic are specific to this implementation.
 *
 * Timing budget at 250 Hz:
 *   derivative          d[n] = x[n] - x[n-2]        ~ 1 sample  (4 ms)
 *   integrator group delay  (W-1)/2 = 18 samples    ~ 72 ms
 *   R-peak refinement   searches back 31 samples    ~ 124 ms
 * so the detector reports a beat about 72 ms after the R peak.  R-R intervals
 * are computed from the *refined* peak positions, so that constant latency
 * cancels out and does not bias the BPM.
 */
#include "heart_rate.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* small helpers                                                       */
/* ------------------------------------------------------------------ */
static ecg_real_t rr_median_of(const ecg_hr_t *hr, uint32_t n)
{
    ecg_real_t tmp[ECG_HR_RR_HISTORY];
    uint32_t i, j, count, idx;
    ecg_real_t key;

    if (hr->rr_series_count == 0u || n == 0u) {
        return 0.0;
    }
    count = (n < hr->rr_series_count) ? n : hr->rr_series_count;
    if (count > ECG_HR_RR_HISTORY) {
        count = ECG_HR_RR_HISTORY;
    }
    /* copy the most recent `count` intervals, oldest first */
    for (i = 0u; i < count; i++) {
        idx = (hr->rr_series_write + ECG_HR_RR_SERIES - count + i) % ECG_HR_RR_SERIES;
        tmp[i] = hr->rr_series[idx];
    }
    /* insertion sort: count <= 5 */
    for (i = 1u; i < count; i++) {
        key = tmp[i];
        j = i;
        while (j > 0u && tmp[j - 1u] > key) {
            tmp[j] = tmp[j - 1u];
            j--;
        }
        tmp[j] = key;
    }
    if ((count & 1u) != 0u) {
        return tmp[count / 2u];
    }
    return (tmp[(count / 2u) - 1u] + tmp[count / 2u]) * 0.5;
}

static void rr_push(ecg_hr_t *hr, ecg_real_t rr)
{
    hr->rr_series[hr->rr_series_write] = rr;
    hr->rr_series_write = (hr->rr_series_write + 1u) % ECG_HR_RR_SERIES;
    if (hr->rr_series_count < ECG_HR_RR_SERIES) {
        hr->rr_series_count++;
    }
}

/* ------------------------------------------------------------------ */
/* init / reset                                                        */
/* ------------------------------------------------------------------ */
void ecg_hr_reset(ecg_hr_t *hr, ecg_real_t fs_hz)
{
    uint32_t i;

    if (hr == NULL) {
        return;
    }
    memset(hr, 0, sizeof(*hr));
    hr->fs_hz = (fs_hz > 0.0) ? fs_hz : (ecg_real_t)ECG_SAMPLE_RATE_HZ;

    hr->refractory_samples = (uint32_t)((hr->fs_hz * (ecg_real_t)ECG_HR_REFRACTORY_MS) / 1000.0 + 0.5);
    hr->twave_samples      = (uint32_t)((hr->fs_hz * (ecg_real_t)ECG_HR_TWAVE_MS) / 1000.0 + 0.5);
    hr->rr_min_samples     = (uint32_t)((hr->fs_hz * (ecg_real_t)ECG_HR_RR_MIN_MS) / 1000.0 + 0.5);
    hr->rr_max_samples     = (uint32_t)((hr->fs_hz * (ecg_real_t)ECG_HR_RR_MAX_MS) / 1000.0 + 0.5);
    hr->warmup_samples     = (uint32_t)(hr->fs_hz * 2.0);   /* 2 s learning phase */
    hr->warmup_settle      = (uint32_t)(hr->fs_hz * 0.25);  /* ignore the first 250 ms */

    for (i = 0u; i < ECG_HR_INTEG_WINDOW; i++) {
        hr->integ[i] = (ecg_real_t)0.0;
    }
    for (i = 0u; i < ECG_HR_PEAK_SEARCH; i++) {
        hr->peak_hist[i] = (ecg_real_t)0.0;
    }
    hr->rr_median = (ecg_real_t)(0.8 * (double)hr->fs_hz);   /* 75 BPM assumption */
    hr->bpm_avg   = (ecg_real_t)75.0;
}

void ecg_hr_init(ecg_hr_t *hr, ecg_real_t fs_hz)
{
    ecg_hr_reset(hr, fs_hz);
}

/* ------------------------------------------------------------------ */
/* main entry point                                                    */
/* ------------------------------------------------------------------ */
ecg_beat_class_t ecg_hr_process(ecg_hr_t *hr, ecg_real_t x, ecg_hr_beat_t *beat_out)
{
    ecg_real_t d, e, env, peak, best, val;
    uint32_t   cand, since, lag, lag_best, search;
    int        is_local_max;
    int        allow_npki = 1;
    ecg_beat_class_t result = ECG_BEAT_NONE;

    if (hr == NULL) {
        return ECG_BEAT_NONE;
    }

    /* ---- 1. derivative ----------------------------------------------------
     * Five-point derivative  d[n] = (2x[n] + x[n-1] - x[n-3] - 2x[n-4]) / 8.
     * It is a band-pass with zeros at DC and at Nyquist peaking near 0.16*fs
     * (40 Hz at 250 Hz), so it emphasises the QRS slope while rejecting both
     * the baseline and the residual high-frequency noise. A plain two-sample
     * difference would put its pass band right up at Nyquist, which turns the
     * R->S fall and the S->iso-electric recovery into two separate envelope
     * maxima and pollutes the adaptive noise estimate.
     * The delay line is primed with the first sample: starting from zeros would
     * turn the very first sample into a full-scale step and inject an impulse
     * of ~A^2 into the envelope integrator, which would then dominate the
     * learning phase and desensitise the detector for the whole record. */
    if (hr->x_primed == 0) {
        hr->x_primed = 1;
        hr->x_hist[0] = x;
        hr->x_hist[1] = x;
        hr->x_hist[2] = x;
        hr->x_hist[3] = x;
    }
    d = ((ecg_real_t)2.0 * x + hr->x_hist[0] - hr->x_hist[2] -
         (ecg_real_t)2.0 * hr->x_hist[3]) / (ecg_real_t)8.0;
    hr->x_hist[3] = hr->x_hist[2];
    hr->x_hist[2] = hr->x_hist[1];
    hr->x_hist[1] = hr->x_hist[0];
    hr->x_hist[0] = x;

    /* ---- 2. square ---- */
    e = d * d;

    /* ---- 3. moving-window integration (150 ms) ---- */
    hr->integ_sum -= hr->integ[hr->integ_index];
    hr->integ[hr->integ_index] = e;
    hr->integ_sum += e;
    hr->integ_index = (hr->integ_index + 1u) % ECG_HR_INTEG_WINDOW;
    if (hr->integ_sum < (ecg_real_t)0.0) {
        hr->integ_sum = (ecg_real_t)0.0;      /* guard against fp drift */
    }
    env = hr->integ_sum / (ecg_real_t)ECG_HR_INTEG_WINDOW;

    /* ---- 4. keep the raw samples for the R-peak refinement ---- */
    hr->peak_hist[hr->peak_index] = x;
    hr->peak_index = (hr->peak_index + 1u) % ECG_HR_PEAK_SEARCH;

    hr->sample_count++;

    /* ---- 5. learning phase: derive the initial thresholds from the max ---- */
    if (hr->warmup_done == 0) {
        /* the first 250 ms are discarded: electrode settling and the DC-blocker
         * start-up would otherwise inflate the initial signal estimate and
         * desensitise the detector for the whole record */
        if (hr->sample_count >= hr->warmup_settle && env > hr->warmup_max) {
            hr->warmup_max = env;
        }
        if (hr->sample_count >= hr->warmup_samples) {
            hr->spki = hr->warmup_max * (ecg_real_t)0.60;
            hr->npki = hr->warmup_max * (ecg_real_t)0.02;
            hr->threshold      = hr->npki + (ecg_real_t)0.25 * (hr->spki - hr->npki);
            hr->threshold_back = hr->threshold * (ecg_real_t)0.5;
            hr->warmup_done = 1;
        }
        hr->env_prev2 = hr->env_prev;
        hr->env_prev  = env;
        return ECG_BEAT_NONE;
    }

    /* ---- 6. local maximum of the envelope? ---- */
    is_local_max = ((hr->env_prev > hr->env_prev2) && (hr->env_prev >= env)) ? 1 : 0;

    if (is_local_max != 0) {
        ecg_real_t thr_eff;
        int        recovered = 0;

        peak = hr->env_prev;                    /* envelope value at sample n-1 */
        cand = hr->sample_count - 1u;

        if (hr->have_last_beat != 0) {
            since = cand - hr->last_beat_sample;
        } else {
            since = hr->sample_count;           /* never had a beat */
        }

        /* search-back: nothing for longer than 1.66 * RR -> halve the bar */
        thr_eff = hr->threshold;
        if (hr->have_last_beat != 0 &&
            since > (uint32_t)((ecg_real_t)1.66 * hr->rr_median)) {
            thr_eff = hr->threshold_back;
            recovered = 1;
        }

        /* Everything within ~360 ms after an R peak belongs to the QRS-T
         * complex: the shoulders of the QRS envelope and the T wave are both
         * local maxima that are systematically far larger than the real noise
         * floor. Feeding them into npki would raise the adaptive threshold and
         * eventually swallow low-amplitude QRS complexes, so that region is
         * excluded from the noise estimate.
         * The gate is capped at half the running RR interval so that a fast
         * rhythm still contributes samples to the noise estimate. */
        {
            uint32_t gate = hr->twave_samples;
            uint32_t half_rr = (uint32_t)(hr->rr_median * (ecg_real_t)0.5);
            if (half_rr < gate) {
                gate = half_rr;
            }
            allow_npki = (hr->have_last_beat == 0 || since >= gate) ? 1 : 0;
        }

        if (peak > thr_eff) {
            int accept = 1;

            if (hr->have_last_beat != 0 && since < hr->refractory_samples) {
                /* inside the refractory period: muscle noise spike, not a QRS */
                accept = 0;
            } else if (hr->have_last_beat != 0 && since < hr->twave_samples &&
                       peak < (ecg_real_t)0.5 * hr->last_beat_env) {
                /* a T wave that followed closely is much smaller than the QRS */
                hr->beats_twave_rejected++;
                accept = 0;
            }

            if (accept != 0) {
                /* --- refine the R peak on |x| inside the latency window --- */
                search = ECG_HR_PEAK_SEARCH - 1u;
                if (hr->have_last_beat != 0 && since < search) {
                    search = since;
                }
                if (search == 0u) {
                    search = 1u;
                }
                best    = (ecg_real_t)-1.0;
                lag_best = 1u;
                for (lag = 1u; lag <= search; lag++) {
                    uint32_t slot = (hr->peak_index + ECG_HR_PEAK_SEARCH - 1u - lag) %
                                    ECG_HR_PEAK_SEARCH;
                    val = ECG_FABS(hr->peak_hist[slot]);
                    if (val > best) {
                        best = val;
                        lag_best = lag;
                    }
                }
                {
                    uint32_t r_index = hr->sample_count - lag_best;
                    uint32_t rr      = 0u;

                    if (hr->have_last_beat != 0) {
                        rr = r_index - hr->last_beat_sample;
                    }

                    if (hr->have_last_beat != 0 &&
                        (rr < hr->rr_min_samples || rr > hr->rr_max_samples)) {
                        /* physiologically impossible interval: reject and treat
                         * the candidate as noise so it cannot poison rr_median */
                        hr->beats_rejected++;
                        if (allow_npki != 0) {
                            hr->npki = (ecg_real_t)0.125 * peak + (ecg_real_t)0.875 * hr->npki;
                        }
                    } else {
                        ecg_real_t rr_ms;
                        int ectopic = 0;
                        int had_prev = hr->have_last_beat;

                        if (had_prev != 0) {
                            /* The ectopic test compares against the running
                             * median, which is only meaningful once a few real
                             * intervals have been measured. Applying it from
                             * the very first beat would dead-lock against the
                             * 75 BPM prior: a 45 BPM or a 150 BPM rhythm would
                             * be flagged as ectopic forever and never reach the
                             * RR series, so bpm_avg would stay at the prior. */
                            if (hr->rr_series_count >= ECG_HR_RR_LEARN) {
                                ecg_real_t dev = ECG_FABS((ecg_real_t)rr - hr->rr_median);
                                if (dev > ((ecg_real_t)ECG_HR_ECTOPIC_PCT / 100.0) *
                                          hr->rr_median) {
                                    ectopic = 1;
                                    hr->beats_ectopic++;
                                }
                            }
                            rr_ms = ((ecg_real_t)rr * 1000.0) / hr->fs_hz;
                        } else {
                            rr_ms = 0.0;
                        }

                        hr->spki = (ecg_real_t)0.125 * peak + (ecg_real_t)0.875 * hr->spki;
                        hr->last_beat_sample = r_index;
                        hr->last_beat_env    = peak;
                        hr->have_last_beat   = 1;
                        hr->beats_total++;

                        if (had_prev != 0 && rr != 0u && ectopic == 0) {
                            rr_push(hr, (ecg_real_t)rr);
                            hr->rr_median = rr_median_of(hr, ECG_HR_RR_HISTORY);
                            if (hr->rr_median > (ecg_real_t)0.0) {
                                hr->bpm_avg = (ecg_real_t)((60.0 * (double)hr->fs_hz) /
                                                           (double)hr->rr_median);
                            }
                        }

                        if (beat_out != NULL) {
                            memset(beat_out, 0, sizeof(*beat_out));
                            beat_out->sample_index = r_index;
                            beat_out->rr_samples   = rr;
                            beat_out->rr_ms        = rr_ms;
                            beat_out->bpm_instant  = (rr != 0u)
                                ? (ecg_real_t)((60.0 * (double)hr->fs_hz) / (double)rr)
                                : hr->bpm_avg;
                            beat_out->bpm_average  = hr->bpm_avg;
                            beat_out->amplitude_uv = best;
                            beat_out->envelope     = peak;
                            beat_out->rr_accepted  = (rr != 0u && ectopic == 0) ? 1 : 0;
                            if (ectopic != 0) {
                                beat_out->klass = ECG_BEAT_ECTOPIC;
                            } else if (recovered != 0) {
                                beat_out->klass = ECG_BEAT_RECOVERED;
                                hr->beats_recovered++;
                            } else {
                                beat_out->klass = ECG_BEAT_NORMAL;
                            }
                            result = beat_out->klass;
                        } else {
                            result = (ectopic != 0) ? ECG_BEAT_ECTOPIC : ECG_BEAT_NORMAL;
                        }
                        if (best > hr->peak_uv_max) {
                            hr->peak_uv_max = best;
                        }
                    }
                }
            } else if (allow_npki != 0) {
                hr->npki = (ecg_real_t)0.125 * peak + (ecg_real_t)0.875 * hr->npki;
            }
        } else if (allow_npki != 0) {
            hr->npki = (ecg_real_t)0.125 * peak + (ecg_real_t)0.875 * hr->npki;
        }

        /* refresh the adaptive threshold */
        hr->threshold      = hr->npki + (ecg_real_t)0.25 * (hr->spki - hr->npki);
        hr->threshold_back = hr->threshold * (ecg_real_t)0.5;
    }

    /* running signal / noise power estimate for the quality index */
    hr->signal_uv_rms += (ecg_real_t)0.0005 * ((x * x) - hr->signal_uv_rms);
    hr->noise_uv_rms  += (ecg_real_t)0.0005 * ((env * env) - hr->noise_uv_rms);

    hr->env_prev2 = hr->env_prev;
    hr->env_prev  = env;
    return result;
}

void ecg_hr_process_block(ecg_hr_t *hr, const ecg_real_t *x, size_t n,
                          ecg_hr_beat_t *beats, size_t max_beats, size_t *n_beats)
{
    size_t i, count = 0u;
    ecg_hr_beat_t tmp;

    if (hr == NULL || x == NULL) {
        if (n_beats != NULL) {
            *n_beats = 0u;
        }
        return;
    }
    for (i = 0u; i < n; i++) {
        ecg_beat_class_t c = ecg_hr_process(hr, x[i], &tmp);
        if (c != ECG_BEAT_NONE) {
            if (beats != NULL && count < max_beats) {
                beats[count] = tmp;
            }
            count++;
        }
    }
    if (n_beats != NULL) {
        *n_beats = count;
    }
}

ecg_real_t ecg_hr_bpm(const ecg_hr_t *hr)
{
    return (hr == NULL) ? (ecg_real_t)0.0 : hr->bpm_avg;
}

uint8_t ecg_hr_quality(const ecg_hr_t *hr)
{
    double q;
    if (hr == NULL) {
        return 0u;
    }
    if ((hr->spki + hr->npki) <= 0.0) {
        return 0u;
    }
    q = 100.0 * (double)hr->spki / (double)(hr->spki + hr->npki);
    if (q < 0.0) {
        q = 0.0;
    }
    if (q > 100.0) {
        q = 100.0;
    }
    return (uint8_t)(q + 0.5);
}

void ecg_hr_get_hrv(const ecg_hr_t *hr, ecg_hr_hrv_t *hrv)
{
    uint32_t i, n, idx, prev_idx;
    double   sum = 0.0, mean, acc = 0.0, dsum = 0.0;
    uint32_t nn50 = 0u, pairs = 0u;
    int      have_prev = 0;

    if (hrv == NULL) {
        return;
    }
    memset(hrv, 0, sizeof(*hrv));
    if (hr == NULL || hr->rr_series_count == 0u) {
        return;
    }
    n = hr->rr_series_count;
    hrv->beats = n;

    for (i = 0u; i < n; i++) {
        idx = (hr->rr_series_write + ECG_HR_RR_SERIES - n + i) % ECG_HR_RR_SERIES;
        sum += (double)hr->rr_series[idx];
    }
    mean = sum / (double)n;
    hrv->mean_rr_ms = (ecg_real_t)((mean * 1000.0) / (double)hr->fs_hz);
    hrv->bpm_mean   = (ecg_real_t)(60000.0 / ((mean * 1000.0) / (double)hr->fs_hz));

    for (i = 0u; i < n; i++) {
        double d;
        idx = (hr->rr_series_write + ECG_HR_RR_SERIES - n + i) % ECG_HR_RR_SERIES;
        d = (double)hr->rr_series[idx] - mean;
        acc += d * d;

        prev_idx = (hr->rr_series_write + ECG_HR_RR_SERIES - n + i - 1u) % ECG_HR_RR_SERIES;
        if (have_prev != 0) {
            double drr = ((double)hr->rr_series[idx] - (double)hr->rr_series[prev_idx]) *
                         (1000.0 / (double)hr->fs_hz);
            dsum += drr * drr;
            pairs++;
            if (ECG_FABS(drr) > 50.0) {
                nn50++;
            }
        }
        have_prev = 1;
    }
    hrv->sdnn_ms  = (ecg_real_t)ECG_SQRT(acc / (double)n);
    if (pairs != 0u) {
        hrv->rmssd_ms = (ecg_real_t)ECG_SQRT(dsum / (double)pairs);
        hrv->pnn50_pct = (ecg_real_t)((100.0 * (double)nn50) / (double)pairs);
    }
}
