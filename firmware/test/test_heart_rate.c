/*
 * test_heart_rate.c - QRS detection accuracy, refractory behaviour and RR
 * outlier rejection, all against a synthetic signal with a known ground truth.
 */
#include "test_util.h"
#include "heart_rate.h"
#include "ecg_pipeline.h"
#include "hal_stub.h"

#define FS          250u
#define HR_SECONDS  60u
#define HR_N        (FS * HR_SECONDS)
#define STEADY_FROM (30u * FS)          /* ignore the first 30 s of adaptation */
#define REFRACTORY_SAMPLES ((ECG_HR_REFRACTORY_MS * FS) / 1000u)

typedef struct {
    double   bpm_steady;
    double   bpm_error;
    uint32_t beats;
    uint32_t rejected;
    uint32_t ectopic;
    uint32_t twave;
    uint32_t recovered;
    double   rpeak_offset_mean;
    double   rpeak_offset_std;
    double   min_rr;
    uint32_t refractory_violations;
} hr_case_t;

/* Run `seconds` of simulated acquisition through the complete chain
 * (calibration -> baseline -> notch -> Savitzky-Golay -> QRS). */
static void run_hr_case(double true_bpm, int with_interference, uint32_t seed,
                        uint32_t seconds, hr_case_t *out)
{
    hal_sim_cfg_t cfg;
    ecg_pipeline_t p;
    uint32_t n, total = seconds * FS;
    double   sum = 0.0;
    uint32_t cnt = 0u;
    double   off_sum = 0.0, off_sq = 0.0;
    uint32_t off_cnt = 0u;
    uint32_t prev_index = 0u;

    memset(out, 0, sizeof(*out));
    out->min_rr = 1e9;
    hal_sim_default_cfg(&cfg);
    cfg.heart_rate_bpm = true_bpm;
    cfg.seed           = seed;
    if (with_interference == 0) {
        cfg.enable_hum    = 0;
        cfg.enable_wander = 0;
        cfg.enable_noise  = 0;
    }
    ecg_pipeline_init(&p, FS, NULL);

    for (n = 0u; n < total; n++) {
        ecg_hr_beat_t beat;
        uint16_t counts = hal_sim_adc_count(&cfg, n);
        (void)ecg_pipeline_process(&p, counts, &beat);

        if (beat.klass != ECG_BEAT_NONE) {
            if (n >= STEADY_FROM) {
                sum += (double)beat.bpm_average;
                cnt++;
            }
            if (beat.rr_samples != 0u && beat.rr_samples < REFRACTORY_SAMPLES) {
                out->refractory_violations++;
            }
            if (prev_index != 0u) {
                uint32_t rr = beat.sample_index - prev_index;
                if ((double)rr < out->min_rr) {
                    out->min_rr = (double)rr;
                }
            }
            prev_index = beat.sample_index;

            {
                long true_idx = hal_sim_nearest_r_peak(&cfg, beat.sample_index, FS / 2u);
                if (true_idx >= 0 && n >= STEADY_FROM) {
                    double off = (double)beat.sample_index - (double)true_idx;
                    off_sum += off;
                    off_sq  += off * off;
                    off_cnt++;
                }
            }
        }
    }

    out->bpm_steady = (cnt != 0u) ? (sum / (double)cnt) : 0.0;
    out->bpm_error  = fabs(out->bpm_steady - true_bpm);
    out->beats      = p.hr.beats_total;
    out->rejected   = p.hr.beats_rejected;
    out->ectopic    = p.hr.beats_ectopic;
    out->twave      = p.hr.beats_twave_rejected;
    out->recovered  = p.hr.beats_recovered;

    if (off_cnt != 0u) {
        double mean = off_sum / (double)off_cnt;
        double var  = (off_sq / (double)off_cnt) - (mean * mean);
        out->rpeak_offset_mean = mean;
        out->rpeak_offset_std  = (var > 0.0) ? sqrt(var) : 0.0;
    }
}

static void test_hr_accuracy_clean(void)
{
    static const double rates[] = { 45.0, 60.0, 72.0, 90.0, 120.0, 150.0, 180.0 };
    size_t i;
    double worst = 0.0;
    uint32_t total_beats = 0u;
    double   worst_offset_std = 0.0;

    tt_begin("heart_rate::accuracy_clean");

    printf("  %-10s %-12s %-12s %-9s %-11s\n",
           "true[BPM]", "measured", "error[BPM]", "beats", "Rpeaksigma");
    printf("  -----------------------------------------------------------\n");
    for (i = 0u; i < sizeof(rates) / sizeof(rates[0]); i++) {
        hr_case_t c;
        run_hr_case(rates[i], 0, (uint32_t)(0x1000u + i), HR_SECONDS, &c);
        printf("  %-10.1f %-12.3f %-12.3f %-9u %-11.2f\n",
               rates[i], c.bpm_steady, c.bpm_error, c.beats, c.rpeak_offset_std);
        TT_CHECK_MSG(c.bpm_error <= 2.0, "%.1f BPM -> %.3f BPM measured (error %.3f)",
                     rates[i], c.bpm_steady, c.bpm_error);
        TT_CHECK_MSG(c.refractory_violations == 0u,
                     "%.1f BPM: %u RR intervals shorter than the refractory period",
                     rates[i], c.refractory_violations);
        TT_CHECK_MSG(c.rpeak_offset_std <= 2.0,
                     "%.1f BPM: R peak jitter %.2f samples", rates[i], c.rpeak_offset_std);
        if (c.bpm_error > worst) {
            worst = c.bpm_error;
        }
        if (c.rpeak_offset_std > worst_offset_std) {
            worst_offset_std = c.rpeak_offset_std;
        }
        total_beats += c.beats;
    }
    printf("\n");

    /* the expected beat count for 60 s is 60 * HR / 60 = HR beats */
    tt_info("total detected beats over %u records: %u", (unsigned)(sizeof(rates) / sizeof(rates[0])),
            total_beats);
    tt_measure("hr_clean_worst_error_bpm", worst, "BPM");
    tt_measure("hr_clean_rpeak_jitter_samples", worst_offset_std, "samples");
    tt_info("worst heart-rate error (clean): %.4f BPM, worst R-peak jitter: %.2f samples",
            worst, worst_offset_std);
    TT_CHECK(worst <= 2.0);

    tt_end();
}

static void test_hr_accuracy_with_interference(void)
{
    static const double rates[] = { 50.0, 60.0, 75.0, 100.0, 140.0 };
    size_t   i;
    double   worst = 0.0;
    uint32_t seed = 0x77u;

    tt_begin("heart_rate::accuracy_with_hum_wander_noise");

    printf("  %-10s %-12s %-12s %-9s %-9s %-9s\n",
           "true[BPM]", "measured", "error[BPM]", "beats", "ectopic", "rejected");
    printf("  --------------------------------------------------------------------\n");
    for (i = 0u; i < sizeof(rates) / sizeof(rates[0]); i++) {
        hr_case_t c;
        hal_sim_cfg_t cfg;
        run_hr_case(rates[i], 1, seed + (uint32_t)i, HR_SECONDS, &c);
        hal_sim_default_cfg(&cfg);
        printf("  %-10.1f %-12.3f %-12.3f %-9u %-9u %-9u\n",
               rates[i], c.bpm_steady, c.bpm_error, c.beats, c.ectopic, c.rejected);
        TT_CHECK_MSG(c.bpm_error <= 2.0,
                     "%.1f BPM with 0.3 mV 50 Hz + 0.6 mV wander + 30 uV noise -> "
                     "%.3f BPM (error %.3f)", rates[i], c.bpm_steady, c.bpm_error);
        if (c.bpm_error > worst) {
            worst = c.bpm_error;
        }
        tt_info("   injected: %.1f mVpp hum @ %.0f Hz, %.1f mVpp wander @ %.2f Hz, %.0f uV rms noise",
                2.0 * cfg.hum_amp_mv, cfg.hum_freq_hz, 2.0 * cfg.wander_amp_mv,
                cfg.wander_freq_hz, cfg.noise_rms_mv * 1000.0);
    }
    printf("\n");
    tt_measure("hr_noisy_worst_error_bpm", worst, "BPM");
    tt_info("worst heart-rate error with full interference: %.4f BPM", worst);
    TT_CHECK(worst <= 2.0);

    tt_end();
}

/* Direct measurement of the refractory period invariant: no two consecutive
 * detected beats may be closer than ECG_HR_REFRACTORY_MS. */
static void test_hr_refractory_and_spike(void)
{
    hal_sim_cfg_t cfg;
    ecg_hr_t      hr;
    double       *sig;
    uint32_t      n, total = 20u * FS;
    uint32_t      spike_at;
    uint32_t      beats = 0u, near_spike = 0u;
    uint32_t      min_rr = 0xFFFFFFFFu, prev = 0u;
    int           have_prev = 0;

    tt_begin("heart_rate::refractory");

    sig = (double *)malloc(total * sizeof(double));
    TT_CHECK(sig != NULL);
    if (sig == NULL) {
        tt_end();
        return;
    }

    hal_sim_default_cfg(&cfg);
    cfg.heart_rate_bpm = 60.0;
    cfg.enable_hum = 0;
    cfg.enable_wander = 0;
    cfg.enable_noise = 0;

    /* the true R peaks sit at t = k * 1 s; put a 2 mV, 4-sample muscle spike
     * 100 ms after the R peak at t = 10 s, i.e. inside the refractory window */
    spike_at = (10u * FS) + (FS / 10u);

    for (n = 0u; n < total; n++) {
        double v = hal_sim_ecg_mv(&cfg, (double)n / (double)FS) * 1000.0;
        if (n >= spike_at - 2u && n <= spike_at + 2u) {
            v += 2000.0;
        }
        sig[n] = v;
    }

    ecg_hr_init(&hr, (double)FS);
    for (n = 0u; n < total; n++) {
        ecg_hr_beat_t beat;
        if (ecg_hr_process(&hr, sig[n], &beat) != ECG_BEAT_NONE) {
            beats++;
            if (have_prev != 0) {
                uint32_t rr = beat.sample_index - prev;
                if (rr < min_rr) {
                    min_rr = rr;
                }
            }
            prev = beat.sample_index;
            have_prev = 1;
            if (beat.sample_index + 6u >= spike_at && beat.sample_index <= spike_at + 6u) {
                near_spike++;
            }
        }
    }

    tt_measure("hr_refractory_min_rr_samples", (double)min_rr, "samples");
    tt_measure("hr_refractory_min_rr_ms", (double)min_rr * 1000.0 / (double)FS, "ms");
    tt_info("20 s at 60 BPM with a 2 mV spike inside the refractory window: "
            "%u beats, minimum RR %u samples (%.0f ms)",
            beats, min_rr, (double)min_rr * 1000.0 / (double)FS);

    TT_CHECK_MSG(min_rr >= REFRACTORY_SAMPLES,
                 "minimum RR %u samples < refractory %u", min_rr, REFRACTORY_SAMPLES);
    TT_CHECK_MSG(near_spike == 0u, "%u beats were triggered by the muscle spike",
                 near_spike);
    /* 20 s at 60 BPM is 19 intervals; the 2 s learning phase and the edge
     * effects cost one or two beats, so allow a small window */
    TT_CHECK(beats >= 17u && beats <= 20u);

    free(sig);
    tt_end();
}

/* A motion artefact 400 ms after an R peak is detected as a beat, but its RR
 * interval deviates far more than ECG_HR_ECTOPIC_PCT from the running median,
 * so it must be flagged and kept out of the RR statistics. */
static void test_hr_outlier_rejection(void)
{
    hal_sim_cfg_t cfg;
    ecg_hr_t      hr;
    ecg_hr_t      reference;
    double       *sig;
    double       *clean;
    uint32_t      n, total = HR_N;
    uint32_t      artifact_at = 5u * FS + (FS * 2u / 5u);   /* R at 5 s + 400 ms */
    uint32_t      beats = 0u;
    double        sum = 0.0;
    uint32_t      cnt = 0u;

    tt_begin("heart_rate::rr_outlier_rejection");

    sig   = (double *)malloc(total * sizeof(double));
    clean = (double *)malloc(total * sizeof(double));
    TT_CHECK(sig != NULL && clean != NULL);
    if (sig == NULL || clean == NULL) {
        free(sig);
        free(clean);
        tt_end();
        return;
    }

    hal_sim_default_cfg(&cfg);
    cfg.heart_rate_bpm = 60.0;
    cfg.enable_hum = 0;
    cfg.enable_wander = 0;
    cfg.enable_noise = 0;

    for (n = 0u; n < total; n++) {
        double v = hal_sim_ecg_mv(&cfg, (double)n / (double)FS) * 1000.0;
        clean[n] = v;
        sig[n]   = v;
        if (n >= artifact_at && n < artifact_at + 6u) {
            sig[n] += 3000.0;                  /* electrode motion artefact */
        }
    }

    /* reference run without the artefact */
    ecg_hr_init(&reference, (double)FS);
    for (n = 0u; n < total; n++) {
        (void)ecg_hr_process(&reference, clean[n], NULL);
    }

    ecg_hr_init(&hr, (double)FS);
    for (n = 0u; n < total; n++) {
        ecg_hr_beat_t beat;
        if (ecg_hr_process(&hr, sig[n], &beat) != ECG_BEAT_NONE) {
            beats++;
            if (n >= STEADY_FROM) {
                sum += (double)beat.bpm_average;
                cnt++;
            }
        }
    }

    {
        double bpm = (cnt != 0u) ? (sum / (double)cnt) : 0.0;
        double err = fabs(bpm - 60.0);
        tt_measure("hr_artifact_ectopic_beats", (double)hr.beats_ectopic, "count");
        tt_measure("hr_artifact_bpm_error", err, "BPM");
        tt_info("with a 3 mV motion artefact: %u beats, %u flagged ectopic, "
                "%u rejected, steady BPM %.3f (error %.3f)",
                beats, hr.beats_ectopic, hr.beats_rejected, bpm, err);
        tt_info("reference run without the artefact: %u beats, steady BPM %.3f",
                reference.beats_total, reference.bpm_avg);
        TT_CHECK_MSG(hr.beats_ectopic >= 1u,
                     "the artefact was not flagged as an ectopic interval");
        TT_CHECK_MSG(err <= 2.0,
                     "outlier rejection failed: %.3f BPM error", err);
    }

    free(sig);
    free(clean);
    tt_end();
}

static void test_hr_statistics(void)
{
    hal_sim_cfg_t cfg;
    ecg_hr_t      hr;
    uint32_t      n, total = HR_N;
    ecg_hr_hrv_t  hrv;
    uint8_t       quality;

    tt_begin("heart_rate::statistics");

    hal_sim_default_cfg(&cfg);
    cfg.heart_rate_bpm = 75.0;
    cfg.enable_hum = 0;
    cfg.enable_wander = 0;
    cfg.enable_noise = 0;
    ecg_hr_init(&hr, (double)FS);

    for (n = 0u; n < total; n++) {
        (void)ecg_hr_process(&hr, hal_sim_ecg_mv(&cfg, (double)n / (double)FS) * 1000.0, NULL);
    }

    TT_CHECK(hr.rr_series_count > 50u);
    TT_NEAR(hr.bpm_avg, 75.0, 2.0);

    ecg_hr_get_hrv(&hr, &hrv);
    quality = ecg_hr_quality(&hr);
    tt_measure("hr_rr_intervals_stored", (double)hr.rr_series_count, "count");
    tt_measure("hr_sdnn_ms", hrv.sdnn_ms, "ms");
    tt_measure("hr_rmssd_ms", hrv.rmssd_ms, "ms");
    tt_measure("hr_pnn50_pct", hrv.pnn50_pct, "%");
    tt_measure("hr_quality_index", (double)quality, "%");
    tt_info("%u RR intervals: mean %.1f ms, SDNN %.2f ms, RMSSD %.2f ms, pNN50 %.1f %%, SQI %u",
            hrv.beats, hrv.mean_rr_ms, hrv.sdnn_ms, hrv.rmssd_ms, hrv.pnn50_pct, quality);
    tt_info("beats %u, rejected %u, ectopic %u, T-wave rejects %u, search-back recoveries %u",
            hr.beats_total, hr.beats_rejected, hr.beats_ectopic,
            hr.beats_twave_rejected, hr.beats_recovered);
    tt_info("envelope levels: spki %.1f, npki %.1f, threshold %.1f, warmup_max %.1f",
            (double)hr.spki, (double)hr.npki, (double)hr.threshold, (double)hr.warmup_max);

    /* a perfectly periodic synthetic signal must show (almost) no variability */
    TT_CHECK_MSG(hrv.sdnn_ms < 2.0, "SDNN %.3f ms on a jitter-free signal", hrv.sdnn_ms);
    TT_CHECK(hrv.rmssd_ms < 2.0);
    TT_CHECK(quality > 80u);

    tt_end();
}

void test_heart_rate_all(void)
{
    test_hr_accuracy_clean();
    test_hr_accuracy_with_interference();
    test_hr_refractory_and_spike();
    test_hr_outlier_rejection();
    test_hr_statistics();
}
