/*
 * test_filters.c - IIR notch, Savitzky-Golay and calibration tests.
 *
 * Every headline number quoted in README.md / docs/DESIGN.md is produced here
 * by an actual measurement on synthetic data with a known ground truth.
 */
#include "test_util.h"
#include "iir_notch.h"
#include "savgol.h"
#include "ecg_pipeline.h"
#include "hal_stub.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FS            250u
#define SWEEP_SECONDS 60u
#define SWEEP_N       (FS * SWEEP_SECONDS)

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */
static double vec_rms(const double *v, size_t n)
{
    double acc = 0.0;
    size_t i;
    if (n == 0u) {
        return 0.0;
    }
    for (i = 0u; i < n; i++) {
        acc += v[i] * v[i];
    }
    return sqrt(acc / (double)n);
}

static double vec_max_abs(const double *v, size_t a, size_t b)
{
    double m = 0.0;
    size_t i;
    for (i = a; i <= b; i++) {
        double x = fabs(v[i]);
        if (x > m) {
            m = x;
        }
    }
    return m;
}

/* Clean ECG in microvolts at the electrode. */
static void make_clean_ecg(const hal_sim_cfg_t *cfg, double *out, size_t n)
{
    size_t i;
    for (i = 0u; i < n; i++) {
        out[i] = hal_sim_ecg_mv(cfg, (double)i / cfg->sample_rate_hz) * 1000.0;
    }
}

/* ------------------------------------------------------------------ */
/* 1. Savitzky-Golay coefficient derivation                            */
/* ------------------------------------------------------------------ */
static void test_sg_coefficients(void)
{
    ecg_savgol_t sg;
    uint32_t order, window, j;
    double   sum, centre;

    tt_begin("savgol::coefficient_derivation");

    /* -- order 0 must degenerate to the plain moving average -- */
    TT_CHECK(ecg_savgol_design(&sg, 0u, 9u) == 0);
    for (j = 0u; j < 9u; j++) {
        TT_NEAR(sg.coef[j], 1.0 / 9.0, 1e-12);
    }

    /* -- against the published Savitzky-Golay tables (independent oracle) -- */
    /* quadratic (order 2), 5 points: [-3, 12, 17, 12, -3] / 35 */
    {
        static const double expect5[5] = { -3.0 / 35.0, 12.0 / 35.0, 17.0 / 35.0,
                                           12.0 / 35.0, -3.0 / 35.0 };
        TT_CHECK(ecg_savgol_design(&sg, 2u, 5u) == 0);
        for (j = 0u; j < 5u; j++) {
            TT_NEAR(sg.coef[j], expect5[j], 1e-12);
        }
    }
    /* cubic (order 3), 7 points: [-2, 3, 6, 7, 6, 3, -2] / 21 */
    {
        static const double expect7[7] = { -2.0 / 21.0, 3.0 / 21.0, 6.0 / 21.0, 7.0 / 21.0,
                                           6.0 / 21.0, 3.0 / 21.0, -2.0 / 21.0 };
        TT_CHECK(ecg_savgol_design(&sg, 3u, 7u) == 0);
        for (j = 0u; j < 7u; j++) {
            TT_NEAR(sg.coef[j], expect7[j], 1e-12);
        }
    }

    /* -- general properties over the whole supported grid -- */
    for (order = 0u; order <= ECG_SG_MAX_ORDER; order++) {
        uint32_t first = order + 1u;
        if ((first & 1u) == 0u) {
            first++;                        /* window must be odd */
        }
        for (window = first; window <= ECG_SG_MAX_WINDOW; window += 2u) {
            if (ecg_savgol_design(&sg, order, window) != 0) {
                TT_CHECK_MSG(0, "design failed for order %u window %u", order, window);
                continue;
            }
            sum = 0.0;
            for (j = 0u; j < window; j++) {
                sum += sg.coef[j];
            }
            TT_CHECK_MSG(fabs(sum - 1.0) < 1e-10,
                         "DC gain order %u window %u = %.15f", order, window, sum);

            /* symmetric kernel: h[j] == h[window-1-j] (odd window =>
             * only even-order terms survive in the fit at x = 0) */
            for (j = 0u; j < window; j++) {
                TT_CHECK_MSG(fabs(sg.coef[j] - sg.coef[window - 1u - j]) < 1e-12,
                             "asymmetric kernel order %u window %u at %u", order, window, j);
            }
            centre = sg.coef[window / 2u];
            TT_CHECK_MSG(centre > 0.0, "negative centre coefficient order %u window %u",
                         order, window);
        }
    }

    /* -- invalid arguments are rejected -- */
    TT_CHECK(ecg_savgol_design(&sg, 2u, 6u) == -1);              /* even window      */
    TT_CHECK(ecg_savgol_design(&sg, 11u, 11u) == -1);            /* order >= window  */
    TT_CHECK(ecg_savgol_design(&sg, 3u, ECG_SG_MAX_WINDOW + 2u) == -1);
    TT_CHECK(ecg_savgol_design(NULL, 3u, 7u) == -1);

    /* -- the edge-aware variant evaluates the same polynomial off-centre -- */
    {
        double edge[7];
        TT_CHECK(ecg_savgol_design_at(edge, 0u, 7u, 0u) == 0);
        for (j = 0u; j < 7u; j++) {
            TT_NEAR(edge[j], 1.0 / 7.0, 1e-12);   /* order 0 is position independent */
        }
    }

    tt_end();
}

static void test_sg_polynomial_reproduction(void)
{
    ecg_savgol_t sg;
    double in[512];
    double out[512];
    size_t i;
    uint32_t order;

    tt_begin("savgol::polynomial_reproduction");

    /* A least-squares polynomial fit of order p reproduces any polynomial of
     * degree <= p exactly, anywhere inside the window. */
    for (order = 0u; order <= 4u; order++) {
        double max_err = 0.0;
        for (i = 0u; i < 512u; i++) {
            double x = (double)i;
            double y = 3.0;
            uint32_t k;
            for (k = 1u; k <= order; k++) {
                y += ((double)(k + 1) * 0.25) * pow(x, (double)k);
            }
            in[i] = y;
            out[i] = 0.0;
        }
        TT_CHECK(ecg_savgol_design(&sg, order, 11u) == 0);
        ecg_savgol_apply(&sg, in, out, 512u);
        /* the first/last (window-1)/2 samples use the shrinking edge windows,
         * which still reproduce the polynomial exactly */
        for (i = 0u; i < 512u; i++) {
            double e = fabs(out[i] - in[i]);
            if (e > max_err) {
                max_err = e;
            }
        }
        TT_CHECK_MSG(max_err < 1e-8 * (1.0 + fabs(in[256])),
                     "order %u reproduction error %.3e", order, max_err);
        tt_info("order %u: max |SG(poly) - poly| = %.3e", order, max_err);
    }

    tt_end();
}

static void test_sg_noise_gain(void)
{
    ecg_savgol_t sg;
    double  noise[200000];
    double  out[200000];
    size_t  i;
    uint32_t seed = 0xC0FFEEu;
    double  rms_in, rms_out, predicted;

    tt_begin("savgol::noise_gain");

    hal_sim_cfg_t cfg;
    hal_sim_default_cfg(&cfg);
    cfg.seed = seed;
    cfg.enable_hum = 0;
    cfg.enable_wander = 0;
    cfg.heart_rate_bpm = 0.0;

    for (i = 0u; i < 200000u; i++) {
        noise[i] = hal_sim_gauss(&cfg);
    }
    TT_CHECK(ecg_savgol_design(&sg, 2u, 11u) == 0);
    ecg_savgol_apply(&sg, noise, out, 200000u);

    rms_in    = vec_rms(noise, 200000u);
    rms_out   = vec_rms(&out[64], 200000u - 128u);
    predicted = ecg_savgol_noise_gain(&sg);

    tt_measure("sg_white_noise_std_ratio", rms_out / rms_in, "x");
    tt_measure("sg_predicted_noise_gain", predicted, "x");
    tt_info("white noise std: in %.4f, out %.4f, ratio %.4f (predicted %.4f), %.2f dB",
            rms_in, rms_out, rms_out / rms_in, predicted, 20.0 * log10(rms_out / rms_in));
    TT_NEAR(rms_out / rms_in, predicted, 0.01);

    tt_end();
}

static void test_sg_stream_matches_block(void)
{
    ecg_savgol_t sg;
    ecg_savgol_stream_t stream;
    double in[1000];
    double blk[1000];
    double str[1000];
    size_t i;
    uint32_t order, window;

    tt_begin("savgol::stream_vs_block");

    for (i = 0u; i < 1000u; i++) {
        in[i] = sin(0.01 * (double)i) + (0.3 * sin(0.31 * (double)i));
    }

    for (order = 1u; order <= 4u; order++) {
        for (window = 5u; window <= 21u; window += 4u) {
            size_t first = window;      /* stream is primed after window-1 pushes */
            size_t last  = 1000u - window;
            double max_diff = 0.0;

            TT_CHECK(ecg_savgol_design(&sg, order, window) == 0);
            ecg_savgol_apply(&sg, in, blk, 1000u);
            ecg_savgol_stream_init(&stream, &sg);
            for (i = 0u; i < 1000u; i++) {
                str[i] = ecg_savgol_stream_push(&stream, in[i], NULL);
            }
            /* the stream output is the centre of the window, so y_stream[n]
             * corresponds to blk[n - half]; after priming both must agree. */
            for (i = first; i < last; i++) {
                double d = fabs(str[i] - blk[i - sg.half]);
                if (d > max_diff) {
                    max_diff = d;
                }
            }
            TT_CHECK_MSG(max_diff < 1e-9, "stream/block mismatch order %u window %u: %.3e",
                         order, window, max_diff);
        }
    }
    tt_info("streaming and block implementations agree to < 1e-9 for every tested pair");

    tt_end();
}

/* ------------------------------------------------------------------ */
/* 2. Savitzky-Golay parameter sweep (order x window)                  */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t order;
    uint32_t window;
    double   cutoff_hz;
    double   noise_reduction_db;
    double   predicted_peak_pct;
    double   measured_peak_pct;
    double   worst_peak_pct;
    double   distortion_pct;
    double   total_error_pct;
} sg_sweep_row_t;

static void sg_evaluate(const hal_sim_cfg_t *cfg,
                        const double *clean, const double *noisy,
                        size_t n, uint32_t order, uint32_t window,
                        sg_sweep_row_t *row)
{
    ecg_savgol_t sg;
    double *filt_clean;
    double *filt_noisy;
    double  rr, amp, sum = 0.0, worst = 1e9;
    size_t  i;
    uint32_t k, count = 0u;
    double  dist_acc = 0.0, err_acc = 0.0;

    filt_clean = (double *)malloc(n * sizeof(double));
    filt_noisy = (double *)malloc(n * sizeof(double));
    if (filt_clean == NULL || filt_noisy == NULL) {
        free(filt_clean);
        free(filt_noisy);
        memset(row, 0, sizeof(*row));
        return;
    }

    memset(row, 0, sizeof(*row));
    row->order  = order;
    row->window = window;
    if (ecg_savgol_design(&sg, order, window) != 0) {
        free(filt_clean);
        free(filt_noisy);
        return;
    }
    ecg_savgol_apply(&sg, clean, filt_clean, n);
    ecg_savgol_apply(&sg, noisy, filt_noisy, n);

    row->cutoff_hz = ecg_savgol_cutoff_hz(&sg, (double)FS);
    row->noise_reduction_db = -20.0 * log10(ecg_savgol_noise_gain(&sg));
    row->predicted_peak_pct = 100.0 * ecg_savgol_gaussian_peak_retention(&sg, (double)FS,
                                                                        cfg->qrs_sigma_s);

    /* QRS peak retention on the real synthetic morphology */
    rr  = 60.0 / cfg->heart_rate_bpm;
    amp = cfg->r_amp_mv * 1000.0;
    for (k = 1u;; k++) {
        double  bt  = (double)k * rr;
        long    idx = (long)((bt * (double)FS) + 0.5);
        size_t  a, b;
        double  pc, pf, ratio;

        if (idx + (long)window >= (long)n) {
            break;
        }
        if (idx - (long)window < 0) {
            continue;
        }
        a = (size_t)idx - 1u;
        b = (size_t)idx + 1u;
        pc = vec_max_abs(clean, a, b);
        pf = vec_max_abs(filt_clean, a, b);
        if (pc <= 0.0) {
            continue;
        }
        ratio = pf / pc;
        sum += ratio;
        if (ratio < worst) {
            worst = ratio;
        }
        count++;
    }
    row->measured_peak_pct = (count != 0u) ? (100.0 * sum / (double)count) : 0.0;
    row->worst_peak_pct    = (count != 0u) ? (100.0 * worst) : 0.0;

    for (i = (size_t)window; i + window < n; i++) {
        double dc = filt_clean[i] - clean[i];
        double dn = filt_noisy[i] - clean[i];
        dist_acc += dc * dc;
        err_acc  += dn * dn;
    }
    {
        size_t m = n - (2u * window);
        row->distortion_pct  = 100.0 * sqrt(dist_acc / (double)m) / amp;
        row->total_error_pct = 100.0 * sqrt(err_acc / (double)m) / amp;
    }

    free(filt_clean);
    free(filt_noisy);
}

static void test_sg_parameter_sweep(void)
{
    static const uint32_t orders[]  = { 2u, 3u, 4u, 5u };
    static const uint32_t windows[] = { 5u, 7u, 9u, 11u, 13u, 15u, 17u, 21u, 25u, 31u };
    hal_sim_cfg_t cfg;
    double *clean;
    double *noisy;
    size_t  i, oi, wi;
    sg_sweep_row_t best;
    sg_sweep_row_t defrow;
    int     have_best = 0;

    tt_begin("savgol::parameter_sweep");

    hal_sim_default_cfg(&cfg);
    cfg.heart_rate_bpm = 72.0;
    cfg.enable_hum     = 0;
    cfg.enable_wander  = 0;
    cfg.enable_noise   = 1;
    cfg.noise_rms_mv   = 0.030;

    clean = (double *)malloc(SWEEP_N * sizeof(double));
    noisy = (double *)malloc(SWEEP_N * sizeof(double));
    TT_CHECK(clean != NULL && noisy != NULL);
    if (clean == NULL || noisy == NULL) {
        free(clean);
        free(noisy);
        tt_end();
        return;
    }
    make_clean_ecg(&cfg, clean, SWEEP_N);
    /* the noisy trace keeps the same morphology but adds white noise; the hum
     * and wander are removed by the analog/notch stages in the real chain, so
     * they are excluded from the smoothing trade-off study */
    for (i = 0u; i < SWEEP_N; i++) {
        noisy[i] = clean[i] + (cfg.noise_rms_mv * 1000.0 * hal_sim_gauss(&cfg));
    }

    printf("  %-6s %-7s %-10s %-10s %-11s %-11s %-11s %-11s\n",
           "order", "window", "-3dB[Hz]", "noise[dB]", "peak_pred[%]",
           "peak_meas[%]", "distort[%]", "error[%]");
    printf("  ---------------------------------------------------------------------------"
           "----------\n");

    memset(&best, 0, sizeof(best));
    memset(&defrow, 0, sizeof(defrow));

    for (oi = 0u; oi < sizeof(orders) / sizeof(orders[0]); oi++) {
        for (wi = 0u; wi < sizeof(windows) / sizeof(windows[0]); wi++) {
            sg_sweep_row_t row;
            if (windows[wi] <= orders[oi]) {
                continue;
            }
            sg_evaluate(&cfg, clean, noisy, SWEEP_N, orders[oi], windows[wi], &row);
            printf("  %-6u %-7u %-10.1f %-10.2f %-11.2f %-11.2f %-11.3f %-11.3f%s\n",
                   row.order, row.window, row.cutoff_hz, row.noise_reduction_db,
                   row.predicted_peak_pct, row.measured_peak_pct, row.distortion_pct,
                   row.total_error_pct,
                   (row.measured_peak_pct >= 95.0) ? "  <- peak ok" : "");

            /* the selected operating point: lowest residual error among the
             * parameter pairs that keep the QRS peak above 95 % */
            if (row.measured_peak_pct >= 95.0 &&
                (have_best == 0 || row.total_error_pct < best.total_error_pct)) {
                best = row;
                have_best = 1;
            }
            if (row.order == ECG_SG_DEFAULT_ORDER && row.window == ECG_SG_DEFAULT_WINDOW) {
                defrow = row;
            }
        }
    }

    printf("\n");
    TT_CHECK(have_best == 1);
    if (have_best != 0) {
        tt_info("sweep optimum  : order %u, window %u -> peak %.2f%%, residual %.3f%%",
                best.order, best.window, best.measured_peak_pct, best.total_error_pct);
    }
    tt_info("configured     : order %u, window %u -> peak %.2f%%, residual %.3f%%",
            ECG_SG_DEFAULT_ORDER, ECG_SG_DEFAULT_WINDOW,
            defrow.measured_peak_pct, defrow.total_error_pct);

    /* the shipped default must satisfy the QRS peak-preservation requirement */
    TT_CHECK_MSG(defrow.measured_peak_pct >= 95.0,
                 "default (order %u, window %u) only preserves %.2f%% of the QRS peak",
                 ECG_SG_DEFAULT_ORDER, ECG_SG_DEFAULT_WINDOW, defrow.measured_peak_pct);
    TT_CHECK_MSG(defrow.worst_peak_pct >= 93.0,
                 "worst individual beat only %.2f%%", defrow.worst_peak_pct);

    tt_measure("sg_default_peak_retention_pct", defrow.measured_peak_pct, "%");
    tt_measure("sg_default_worst_peak_retention_pct", defrow.worst_peak_pct, "%");
    tt_measure("sg_default_distortion_pct", defrow.distortion_pct, "%");
    tt_measure("sg_default_residual_error_pct", defrow.total_error_pct, "%");
    tt_measure("sg_default_cutoff_hz", defrow.cutoff_hz, "Hz");
    tt_measure("sg_default_noise_reduction_db", defrow.noise_reduction_db, "dB");
    tt_measure("sg_default_predicted_peak_pct", defrow.predicted_peak_pct, "%");
    tt_measure("sg_optimum_order", (double)best.order, "");
    tt_measure("sg_optimum_window", (double)best.window, "");

    free(clean);
    free(noisy);
    tt_end();
}

/* ------------------------------------------------------------------ */
/* 3. IIR notch                                                        */
/* ------------------------------------------------------------------ */
static void test_notch_coefficients(void)
{
    ecg_biquad_t bq;
    ecg_notch_t  notch;

    tt_begin("notch::coefficients");

    TT_CHECK(ecg_biquad_notch_design(&bq, 250.0, 50.0, 8.0) == 0);
    /* hand-derived values documented in iir_notch.c */
    TT_NEAR(bq.b0, 0.9438940, 1e-6);
    TT_NEAR(bq.b1, -0.5833586, 1e-6);
    TT_NEAR(bq.b2, 0.9438940, 1e-6);
    TT_NEAR(bq.a1, -0.5833586, 1e-6);
    TT_NEAR(bq.a2, 0.8877879, 1e-6);
    tt_info("fs=250 f0=50 Q=8 -> b=[%.7f %.7f %.7f] a=[1 %.7f %.7f]",
            bq.b0, bq.b1, bq.b2, bq.a1, bq.a2);

    /* zeros on the unit circle at exactly +-50 Hz -> numerical null */
    TT_CHECK_MSG(ecg_biquad_response_db(&bq, 50.0) < -250.0,
                 "response at 50 Hz is only %.2f dB", ecg_biquad_response_db(&bq, 50.0));

    /* the biquad is exactly 0 dB at DC and at Nyquist */
    TT_NEAR(ecg_biquad_response_db(&bq, 0.0), 0.0, 1e-3);
    TT_NEAR(ecg_biquad_response_db(&bq, 125.0), 0.0, 1e-3);

    /* 2-stage cascade doubles the depth at a fixed offset from the null */
    TT_CHECK(ecg_notch_init(&notch, 250.0, 50.0, 8.0, 2u) == 0);
    TT_CHECK(notch.n_stages == 2u);
    tt_info("cascade: 2 sections, -3 dB notch half width %.2f Hz",
            ecg_notch_stopband_edge_hz(&notch, -3.0));
    TT_NEAR(ecg_notch_response_db(&notch, 49.0),
            2.0 * ecg_biquad_response_db(&bq, 49.0), 1e-6);

    /* invalid designs are refused */
    TT_CHECK(ecg_biquad_notch_design(&bq, 250.0, 130.0, 8.0) == -1);
    TT_CHECK(ecg_biquad_notch_design(&bq, 0.0, 50.0, 8.0) == -1);
    TT_CHECK(ecg_biquad_notch_design(&bq, 250.0, 50.0, 0.0) == -1);

    tt_end();
}

/* Time-domain attenuation: sine in, RMS out, steady state only. */
static double measure_tone_attenuation_db(const ecg_notch_t *notch, double f_hz,
                                          uint32_t fs, uint32_t seconds, uint32_t skip_s)
{
    uint32_t n = fs * seconds;
    uint32_t skip = fs * skip_s;
    double  *in;
    double  *out;
    uint32_t i;
    double   acc_in = 0.0, acc_out = 0.0;
    double   db;

    in  = (double *)malloc(n * sizeof(double));
    out = (double *)malloc(n * sizeof(double));
    if (in == NULL || out == NULL) {
        free(in);
        free(out);
        return 0.0;
    }
    for (i = 0u; i < n; i++) {
        in[i] = sin(2.0 * M_PI * f_hz * (double)i / (double)fs);
    }
    {
        ecg_notch_t local = *notch;
        ecg_notch_reset(&local);
        ecg_notch_process_block(&local, in, out, n);
    }
    for (i = skip; i < n; i++) {
        acc_in  += in[i] * in[i];
        acc_out += out[i] * out[i];
    }
    if (acc_in <= 0.0 || acc_out <= 0.0) {
        free(in);
        free(out);
        return -300.0;
    }
    db = 10.0 * log10(acc_out / acc_in);
    free(in);
    free(out);
    return db;
}

static void test_notch_attenuation(void)
{
    ecg_notch_t notch;
    ecg_notch_t single;
    double worst = -1e9;      /* dB values are negative: start below all of them */
    double f;
    static const double probe[] = { 49.5, 49.8, 50.0, 50.2, 50.5 };

    tt_begin("notch::50Hz_attenuation");

    TT_CHECK(ecg_notch_init(&notch, (double)FS, ECG_NOTCH_F0_HZ, ECG_NOTCH_Q,
                            ECG_NOTCH_STAGES) == 0);
    TT_CHECK(ecg_notch_init(&single, (double)FS, ECG_NOTCH_F0_HZ, ECG_NOTCH_Q, 1u) == 0);

    printf("  %-10s %-14s %-14s\n", "freq[Hz]", "1 stage[dB]", "2 stages[dB]");
    printf("  --------------------------------------\n");
    for (f = 49.0; f <= 51.01; f += 0.5) {
        double a1 = measure_tone_attenuation_db(&single, f, FS, 12u, 4u);
        double a2 = measure_tone_attenuation_db(&notch, f, FS, 12u, 4u);
        printf("  %-10.2f %-14.2f %-14.2f\n", f, a1, a2);
    }
    printf("\n");

    /* headline: attenuation at exactly 50.00 Hz */
    {
        double a50 = measure_tone_attenuation_db(&notch, 50.0, FS, 12u, 4u);
        double a_single = measure_tone_attenuation_db(&single, 50.0, FS, 12u, 4u);
        tt_measure("notch_50Hz_attenuation_db", a50, "dB");
        tt_measure("notch_50Hz_attenuation_single_stage_db", a_single, "dB");
        tt_info("measured attenuation at 50.00 Hz: %.2f dB (1 stage %.2f dB)", a50, a_single);
        TT_CHECK_MSG(a50 <= -20.0, "50 Hz attenuation only %.2f dB", a50);
    }

    /* worst case over realistic mains drift +-0.5 Hz */
    for (f = 49.5; f <= 50.51; f += 0.25) {
        double a = measure_tone_attenuation_db(&notch, f, FS, 12u, 4u);
        if (a > worst) {
            worst = a;
        }
    }
    {
        double a = measure_tone_attenuation_db(&notch, 50.0, FS, 12u, 4u);
        if (a > worst) {
            worst = a;
        }
    }
    tt_measure("notch_worst_attenuation_49.5_50.5Hz_db", worst, "dB");
    tt_info("worst attenuation over 49.5..50.5 Hz: %.2f dB", worst);
    TT_CHECK_MSG(worst <= -20.0, "worst drift-case attenuation only %.2f dB", worst);

    /* the discrete probe list used in the README table */
    for (f = 0.0; f < (double)(sizeof(probe) / sizeof(probe[0])); f += 1.0) {
        double a = measure_tone_attenuation_db(&notch, probe[(int)f], FS, 12u, 4u);
        printf("  %-8.1f Hz -> %8.2f dB\n", probe[(int)f], a);
    }
    printf("\n");

    tt_end();
}

static void test_notch_passband(void)
{
    ecg_notch_t notch;
    static const double freqs[] = { 0.5, 1.0, 5.0, 10.0, 20.0, 40.0, 75.0, 100.0 };
    size_t i;
    double worst = -1e9;

    tt_begin("notch::passband");

    TT_CHECK(ecg_notch_init(&notch, (double)FS, ECG_NOTCH_F0_HZ, ECG_NOTCH_Q,
                            ECG_NOTCH_STAGES) == 0);

    printf("  %-10s %-14s %-14s\n", "freq[Hz]", "theory[dB]", "measured[dB]");
    printf("  --------------------------------------\n");
    for (i = 0u; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
        double th = ecg_notch_response_db(&notch, freqs[i]);
        double ms = measure_tone_attenuation_db(&notch, freqs[i], FS, 12u, 4u);
        printf("  %-10.1f %-14.3f %-14.3f\n", freqs[i], th, ms);
        if (ms > worst) {
            worst = ms;
        }
    }
    printf("\n");
    tt_measure("notch_passband_worst_attenuation_db", worst, "dB");
    tt_info("largest pass-band loss (0.5..100 Hz, excluding the notch): %.3f dB", worst);
    TT_CHECK_MSG(worst >= -0.5, "the notch eats %.3f dB of the ECG pass band", worst);

    tt_end();
}

static void test_hpf1(void)
{
    ecg_hpf1_t hp;
    double dc = 0.0;
    uint32_t i;

    tt_begin("highpass::residual_baseline");

    TT_CHECK(ecg_hpf1_init(&hp, (double)FS, ECG_BASELINE_HP_HZ) == 0);
    /* a 1 mV DC step must decay to (almost) nothing in 5 s */
    for (i = 0u; i < 5u * FS; i++) {
        dc = ecg_hpf1_process(&hp, 1000.0);
    }
    tt_info("1 mV DC step after 5 s: %.4f uV (%.1f dB)", dc,
            20.0 * log10(fabs(dc) / 1000.0 + 1e-30));
    TT_CHECK(fabs(dc) < 1.0);

    /* 0.25 Hz respiration wander is heavily attenuated, 50 Hz passes */
    TT_NEAR(ecg_hpf1_response_db(&hp, 0.25), -6.0, 1.5);
    TT_CHECK(ecg_hpf1_response_db(&hp, 10.0) > -0.1);

    tt_end();
}

/* ------------------------------------------------------------------ */
/* 4. calibration with the 1 mV standard square wave                   */
/* ------------------------------------------------------------------ */
static void test_calibration(void)
{
    hal_sim_cfg_t   cfg;
    ecg_cal_point_t pts[4];
    ecg_cal_t       cal;
    static const double mv_pp[4] = { 0.5, 1.0, 1.5, 2.0 };
    uint16_t hi[256];
    uint16_t lo[256];
    size_t   i;
    double   err_pct;

    tt_begin("calibration::1mV_square_wave");

    hal_sim_default_cfg(&cfg);
    cfg.enable_noise  = 1;
    cfg.noise_rms_mv  = 0.010;          /* 10 uV rms front-end noise */
    cfg.seed          = 0xBEEF01u;

    for (i = 0u; i < 4u; i++) {
        double hi_mv = mv_pp[i] / 2.0;
        double lo_mv = -mv_pp[i] / 2.0;
        double mean_hi, mean_lo;
        hal_sim_capture_plateau(&cfg, hi_mv, hi, 256u);
        hal_sim_capture_plateau(&cfg, lo_mv, lo, 256u);
        mean_hi = ecg_cal_plateau_mean(hi, 256u);
        mean_lo = ecg_cal_plateau_mean(lo, 256u);
        pts[i].mv     = mv_pp[i];
        pts[i].counts = mean_hi - mean_lo;      /* peak-to-peak swing */
        tt_info("%.1f mVpp -> %.1f counts pp (hi %.1f, lo %.1f)",
                mv_pp[i], pts[i].counts, mean_hi, mean_lo);
    }

    TT_CHECK(ecg_cal_fit(&cal, pts, 4u) == 0);
    TT_CHECK(cal.valid == 1u);
    TT_CHECK(cal.n_points == 4u);

    err_pct = 100.0 * fabs((double)cal.afe_gain - ECG_AFE_GAIN_NOMINAL) / ECG_AFE_GAIN_NOMINAL;

    tt_measure("cal_slope_counts_per_mv", cal.slope_counts_per_mv, "counts/mV");
    tt_measure("cal_measured_afe_gain", cal.afe_gain, "V/V");
    tt_measure("cal_gain_error_pct", err_pct, "%");
    tt_measure("cal_r2", cal.r2, "");
    tt_measure("cal_nonlinearity_pct_fs", cal.nonlinearity_pct_fs, "%FS");
    tt_measure("cal_uv_per_lsb", cal.electrode_uv_per_lsb, "uV/LSB");
    tt_info("fitted gain %.1f counts/mV -> analog gain %.1f V/V (nominal %.0f), R2=%.6f",
            cal.slope_counts_per_mv, cal.afe_gain, ECG_AFE_GAIN_NOMINAL, cal.r2);
    tt_info("nonlinearity %.3f %%FS, %.4f uV per ADC count at the electrode",
            cal.nonlinearity_pct_fs, cal.electrode_uv_per_lsb);

    TT_CHECK_MSG(err_pct < 1.0, "measured gain is %.2f %% off nominal", err_pct);
    TT_CHECK_MSG(cal.r2 > 0.999, "R2 = %.6f", cal.r2);
    TT_CHECK_MSG(cal.nonlinearity_pct_fs < 1.0, "nonlinearity %.3f %%FS",
                 cal.nonlinearity_pct_fs);

    /* the reverse conversion must invert the fit */
    TT_NEAR(ecg_cal_counts_to_uv(&cal, cal.intercept_counts + cal.slope_counts_per_mv),
            1000.0, 1.0);

    /* the median pre-filter must reject a single glitch */
    {
        uint16_t glitchy[64];
        double   clean_mean, glitch_mean;
        for (i = 0u; i < 64u; i++) {
            glitchy[i] = 2000u;
        }
        clean_mean = ecg_cal_plateau_mean(glitchy, 64u);
        glitchy[10] = 4000u;
        glitch_mean = ecg_cal_plateau_mean(glitchy, 64u);
        TT_NEAR(clean_mean, 2000.0, 0.001);
        TT_NEAR(glitch_mean, 2000.0, 0.001);   /* glitch removed by the median */
    }

    /* degenerate inputs */
    TT_CHECK(ecg_cal_fit(&cal, pts, 1u) == -1);
    TT_CHECK(ecg_cal_fit(NULL, pts, 4u) == -1);
    {
        ecg_cal_point_t same[3] = { { 1.0, 100.0 }, { 1.0, 100.0 }, { 1.0, 100.0 } };
        TT_CHECK(ecg_cal_fit(&cal, same, 3u) == -1);
    }

    ecg_cal_defaults(&cal);
    TT_CHECK(cal.valid == 0u);
    TT_NEAR(cal.afe_gain, ECG_AFE_GAIN_NOMINAL, 1e-9);

    tt_end();
}

void test_filters_all(void)
{
    test_sg_coefficients();
    test_sg_polynomial_reproduction();
    test_sg_noise_gain();
    test_sg_stream_matches_block();
    test_sg_parameter_sweep();
    test_notch_coefficients();
    test_notch_attenuation();
    test_notch_passband();
    test_hpf1();
    test_calibration();
}
