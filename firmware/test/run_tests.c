/*
 * run_tests.c - test harness, end-to-end integration test, demo and dump mode.
 *
 *   ./build/run_tests                 run every suite
 *   ./build/run_tests demo            acquisition demo incl. an ASCII waveform
 *   ./build/run_tests --dump build    write the CSV/coefficient files consumed
 *                                     by tools/verify_filters.py
 *
 * Exit status is 0 only when every check passed.
 */
#include "test_util.h"

#include <stdarg.h>
#include <time.h>

#include "hal_stub.h"
#include "ecg_pipeline.h"
#include "frame_protocol.h"

/* ------------------------------------------------------------------ */
/* harness                                                             */
/* ------------------------------------------------------------------ */
#define TT_MAX_MEASURE 160

typedef struct {
    char   key[56];
    double value;
    char   unit[16];
} tt_measurement_t;

static tt_measurement_t s_meas[TT_MAX_MEASURE];
static int s_meas_count = 0;
static int s_checks = 0;
static int s_failures = 0;
static int s_suites = 0;
static int s_suites_failed = 0;
static int s_suite_failed = 0;

void tt_begin(const char *suite)
{
    s_suites++;
    s_suite_failed = 0;
    printf("== %s\n", (suite != NULL) ? suite : "?");
}

void tt_end(void)
{
    if (s_suite_failed != 0) {
        s_suites_failed++;
        printf("   FAILED\n\n");
    } else {
        printf("   ok\n\n");
    }
}

int tt_suite_failed(void)
{
    return s_suite_failed;
}

void tt_check_impl(int ok, const char *expr, const char *file, int line)
{
    s_checks++;
    if (ok == 0) {
        s_failures++;
        s_suite_failed = 1;
        printf("   FAIL %s:%d  %s\n", (file != NULL) ? file : "?", line,
               (expr != NULL) ? expr : "?");
    }
}

void tt_near_impl(double a, double b, double tol, const char *ea, const char *eb,
                  const char *file, int line)
{
    int ok = (fabs(a - b) <= tol) ? 1 : 0;
    s_checks++;
    if (ok == 0) {
        s_failures++;
        s_suite_failed = 1;
        printf("   FAIL %s:%d  %s ~= %s  (%.12g vs %.12g, tol %.3g)\n",
               (file != NULL) ? file : "?", line,
               (ea != NULL) ? ea : "?", (eb != NULL) ? eb : "?", a, b, tol);
    }
}

void tt_info(const char *fmt, ...)
{
    va_list ap;
    printf("   ");
    va_start(ap, fmt);
    (void)vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}

void tt_measure(const char *key, double value, const char *unit)
{
    int i;
    if (key == NULL) {
        return;
    }
    for (i = 0; i < s_meas_count; i++) {
        if (strcmp(s_meas[i].key, key) == 0) {
            s_meas[i].value = value;
            return;
        }
    }
    if (s_meas_count >= TT_MAX_MEASURE) {
        return;
    }
    snprintf(s_meas[s_meas_count].key, sizeof(s_meas[s_meas_count].key), "%s", key);
    s_meas[s_meas_count].value = value;
    snprintf(s_meas[s_meas_count].unit, sizeof(s_meas[s_meas_count].unit), "%s",
             (unit != NULL) ? unit : "");
    s_meas_count++;
}

void tt_measure_reset(void)
{
    s_meas_count = 0;
}

/* ------------------------------------------------------------------ */
/* end-to-end integration: patient -> AFE -> ADC/DMA -> ring -> DSP -> */
/* frames -> UART -> host frame synchroniser                           */
/* ------------------------------------------------------------------ */
#define INTEG_SECONDS 60u
#define INTEG_SAMPLES (INTEG_SECONDS * ECG_SAMPLE_RATE_HZ)
/* Circular DMA only raises an interrupt when a whole half-buffer is complete,
 * so a run of INTEG_SAMPLES ends on a partial block that is never delivered.
 * The expected sample count is therefore rounded down to the block size. */
#define INTEG_DELIVERED ((INTEG_SAMPLES / ECG_DMA_BLOCK_SIZE) * ECG_DMA_BLOCK_SIZE)

typedef struct {
    ecg_device_t  dev;
    hal_sim_cfg_t cfg;
    double       *filtered;
    uint32_t      filtered_cap;
    uint32_t      filtered_n;
    uint32_t      task_calls;
} integ_ctx_t;

static void integ_main_loop(void *user)
{
    integ_ctx_t *c = (integ_ctx_t *)user;
    ecg_real_t   buf[256];
    uint32_t     n;

    c->task_calls++;
    n = ecg_device_task(&c->dev, buf, 256u);
    if (n != 0u && c->filtered != NULL && (c->filtered_n + n) <= c->filtered_cap) {
        memcpy(&c->filtered[c->filtered_n], buf, (size_t)n * sizeof(double));
    }
    c->filtered_n += n;
}

static void integ_run(integ_ctx_t *c, double bpm, int interference, uint32_t seed)
{
    hal_sim_reset();
    hal_sim_default_cfg(&c->cfg);
    c->cfg.heart_rate_bpm = bpm;
    c->cfg.seed           = seed;
    if (interference == 0) {
        c->cfg.enable_hum    = 0;
        c->cfg.enable_wander = 0;
        c->cfg.enable_noise  = 0;
    }
    hal_sim_set_cfg(&c->cfg);

    c->filtered_n = 0u;
    c->task_calls = 0u;
    TT_CHECK(ecg_device_init(&c->dev) == 0);
    TT_CHECK(ecg_device_start(&c->dev) == 0);
    TT_CHECK(hal_ops() != NULL);
    TT_CHECK(strcmp(hal_ops()->name, "pc-simulation-stub") == 0);

    (void)hal_sim_run(&c->cfg, INTEG_SAMPLES, 8u, integ_main_loop, c);
}

void test_pipeline_integration(void)
{
    static integ_ctx_t ctx;
    ecg_frame_sync_t   sync;
    ecg_frame_t        frame;
    ecg_sync_result_t  r;
    const uint8_t     *uart;
    uint32_t           uart_len, off;
    uint32_t           frames_by_type[16];
    uint32_t           total_frames = 0u;
    double             hr_sum = 0.0;
    uint32_t           hr_cnt = 0u;
    uint32_t           seq_errors = 0u;
    uint16_t           prev_seq = 0u;
    int                have_prev_seq = 0;
    uint32_t           decoded_samples = 0u;
    uint32_t           mismatch = 0u;
    uint32_t           k, i;
    int16_t            uv[ECG_SAMPLE_BLOCK_MAX];

    tt_begin("integration::dma_ring_filters_frames_uart");

    ctx.filtered_cap = INTEG_SAMPLES + 4096u;
    ctx.filtered     = (double *)malloc(ctx.filtered_cap * sizeof(double));
    TT_CHECK(ctx.filtered != NULL);
    if (ctx.filtered == NULL) {
        tt_end();
        return;
    }

    integ_run(&ctx, 72.0, 1, 0x9E3779B9u);

    /* ---- DMA / ring health ---- */
    {
        const ecg_dev_stats_t *st = ecg_device_stats(&ctx.dev);
        TT_CHECK(st != NULL);
        tt_info("ISR blocks %u, samples acquired %u, dropped %u, ring overflows %u",
                st->isr_blocks, st->samples_acquired, st->samples_dropped, st->ring_overflows);
        tt_measure("integ_isr_blocks", (double)st->isr_blocks, "count");
        tt_measure("integ_samples_acquired", (double)st->samples_acquired, "samples");
        tt_measure("integ_samples_dropped", (double)st->samples_dropped, "samples");
        tt_measure("integ_beats", (double)st->beats, "beats");
        tt_measure("integ_frames_tx", (double)st->frames_tx, "frames");
        tt_measure("integ_bytes_tx", (double)st->bytes_tx, "bytes");
        TT_CHECK_MSG(st->samples_dropped == 0u, "%u samples were dropped", st->samples_dropped);
        TT_CHECK_MSG(st->ring_overflows == 0u, "%u ring overflows", st->ring_overflows);
        TT_CHECK(st->isr_blocks == (INTEG_SAMPLES / (ECG_DMA_BUFFER_SIZE / 2u)));
        TT_CHECK_MSG(st->samples_acquired == INTEG_DELIVERED,
                     "%u samples acquired, expected %u", st->samples_acquired, INTEG_DELIVERED);
        TT_CHECK(ctx.filtered_n == INTEG_DELIVERED);
        TT_CHECK(st->beats >= 65u && st->beats <= 75u);   /* 60 s at 72 BPM */
        tt_measure("integ_dma_blocks_expected", (double)(INTEG_SAMPLES / ECG_DMA_BLOCK_SIZE), "blocks");
    }

    /* ---- decode the captured uplink with the real frame synchroniser,
     *      delivered in awkward chunk sizes to force half/whole packets ---- */
    memset(frames_by_type, 0, sizeof(frames_by_type));
    ecg_frame_sync_init(&sync);
    uart     = hal_sim_uart_buffer();
    uart_len = hal_sim_uart_len();
    TT_CHECK(uart_len > 1000u);

    off = 0u;
    while (off < uart_len) {
        static const uint32_t chunk_sizes[4] = { 1u, 7u, 53u, 4096u };
        uint32_t chunk = chunk_sizes[(off / 1u) % 4u];
        if (chunk > (uart_len - off)) {
            chunk = uart_len - off;
        }
        for (i = 0u; i < chunk; i++) {
            r = ecg_frame_sync_feed(&sync, uart[off + i], &frame);
            if (r == ECG_SYNC_FRAME) {
                total_frames++;
                /* the device assigns one monotonic sequence number per emitted
                 * frame regardless of type, so the decoded order must be
                 * gap-free: any gap means a lost or duplicated frame */
                if (have_prev_seq != 0 && frame.seq != (uint16_t)(prev_seq + 1u)) {
                    seq_errors++;
                }
                prev_seq = frame.seq;
                have_prev_seq = 1;

                if (frame.type < 16u) {
                    frames_by_type[frame.type]++;
                }
                if (frame.type == (uint8_t)ECG_FRAME_HR_REPORT) {
                    ecg_hr_report_t rep;
                    if (ecg_frame_parse_hr(&frame, &rep) == 0) {
                        hr_sum += (double)rep.bpm_avg_x10 / 10.0;
                        hr_cnt++;
                    }
                }
                if (frame.type == (uint8_t)ECG_FRAME_ADC_PACK12 ||
                    frame.type == (uint8_t)ECG_FRAME_ADC_DELTA) {
                    size_t cnt = 0u;
                    if (ecg_frame_parse_samples(&frame, uv, ECG_SAMPLE_BLOCK_MAX, &cnt) == 0) {
                        for (k = 0u; k < cnt; k++) {
                            uint32_t idx = decoded_samples + k;
                            if (idx < ctx.filtered_n) {
                                double expect = ctx.filtered[idx];
                                if (fabs((double)uv[k] - expect) > 1.0) {
                                    mismatch++;
                                }
                            }
                        }
                        decoded_samples += (uint32_t)cnt;
                    } else {
                        mismatch++;
                    }
                }
            }
        }
        off += chunk;
    }

    tt_info("decoded %u frames: %u sample blocks, %u HR reports, %u status, %u ack",
            total_frames, frames_by_type[ECG_FRAME_ADC_PACK12] +
                          frames_by_type[ECG_FRAME_ADC_DELTA],
            frames_by_type[ECG_FRAME_HR_REPORT], frames_by_type[ECG_FRAME_STATUS],
            frames_by_type[ECG_FRAME_ACK]);
    tt_info("frame synchroniser: ok %u, CRC errors %u, format errors %u, resync slips %u",
            sync.frames_ok, sync.frames_crc_err, sync.frames_fmt_err, sync.resync_slips);

    TT_CHECK_MSG(sync.frames_crc_err == 0u, "%u CRC errors on a clean link",
                 sync.frames_crc_err);
    TT_CHECK_MSG(sync.frames_fmt_err == 0u, "%u format errors", sync.frames_fmt_err);
    TT_CHECK(total_frames > 100u);
    TT_CHECK(frames_by_type[ECG_FRAME_HR_REPORT] > 30u);
    TT_CHECK(seq_errors == 0u);
    TT_CHECK_MSG(decoded_samples == INTEG_DELIVERED, "%u samples decoded, expected %u", decoded_samples, INTEG_DELIVERED);
    TT_CHECK_MSG(mismatch == 0u, "%u of %u sample values differ from the transmitted ones",
                 mismatch, decoded_samples);

    /* ---- the heart rate recovered from the uplink must match the truth ---- */
    {
        double hr = (hr_cnt != 0u) ? (hr_sum / (double)hr_cnt) : 0.0;
        tt_measure("integ_hr_from_uplink_bpm", hr, "BPM");
        tt_measure("integ_hr_reports", (double)hr_cnt, "count");
        tt_info("heart rate decoded from the uplink: %.3f BPM (truth 72.0), %u reports",
                hr, hr_cnt);
        TT_CHECK_MSG(fabs(hr - 72.0) <= 2.0, "uplink BPM %.3f vs truth 72.0", hr);
    }

    tt_measure("integ_frames_decoded", (double)total_frames, "frames");
    tt_measure("integ_sample_blocks", (double)(frames_by_type[ECG_FRAME_ADC_PACK12] +
                                              frames_by_type[ECG_FRAME_ADC_DELTA]), "frames");
    tt_measure("integ_uplink_bytes", (double)uart_len, "bytes");
    tt_measure("integ_uplink_bytes_per_second", (double)uart_len / (double)INTEG_SECONDS, "B/s");
    tt_info("uplink: %u bytes in %u s = %.1f B/s (%.0f bit/s at 8N1)",
            uart_len, INTEG_SECONDS, (double)uart_len / (double)INTEG_SECONDS,
            10.0 * (double)uart_len / (double)INTEG_SECONDS);

    free(ctx.filtered);
    ctx.filtered = NULL;
    tt_end();
}

/* ------------------------------------------------------------------ */
/* dump mode for tools/verify_filters.py                               */
/* ------------------------------------------------------------------ */
static void dump_coefficients(const char *dir)
{
    char  path[512];
    FILE *f;
    ecg_pipeline_t p;
    uint32_t j;

    snprintf(path, sizeof(path), "%s/coefficients.txt", dir);
    f = fopen(path, "w");
    if (f == NULL) {
        printf("cannot write %s\n", path);
        return;
    }
    ecg_pipeline_init(&p, ECG_SAMPLE_RATE_HZ, NULL);

    fprintf(f, "# generated by %s --dump, every value is what the C firmware uses\n", "run_tests");
    fprintf(f, "fs_hz %.10f\n", (double)ECG_SAMPLE_RATE_HZ);
    fprintf(f, "uv_per_lsb %.12f\n", (double)p.cal.electrode_uv_per_lsb);
    fprintf(f, "cal_slope_counts_per_mv %.12f\n", (double)p.cal.slope_counts_per_mv);
    fprintf(f, "cal_intercept_counts %.12f\n", (double)p.cal.intercept_counts);
    fprintf(f, "hpf_fc_hz %.10f\n", (double)ECG_BASELINE_HP_HZ);
    fprintf(f, "hpf_sections %u\n", (unsigned)ECG_BASELINE_HP_SECTIONS);
    fprintf(f, "hpf_a %.12f\n", (double)p.baseline[0].a);
    fprintf(f, "notch_f0_hz %.10f\n", p.notch.f0_hz);
    fprintf(f, "notch_q %.10f\n", p.notch.q);
    fprintf(f, "notch_stages %u\n", p.notch.n_stages);
    for (j = 0u; j < p.notch.n_stages; j++) {
        fprintf(f, "notch%u_b %.12f %.12f %.12f\n", j,
                (double)p.notch.stage[j].b0, (double)p.notch.stage[j].b1,
                (double)p.notch.stage[j].b2);
        fprintf(f, "notch%u_a1a2 %.12f %.12f\n", j,
                (double)p.notch.stage[j].a1, (double)p.notch.stage[j].a2);
    }
    fprintf(f, "sg_order %u\n", p.sg.kernel.order);
    fprintf(f, "sg_window %u\n", p.sg.kernel.window);
    fprintf(f, "sg_coef");
    for (j = 0u; j < p.sg.kernel.window; j++) {
        fprintf(f, " %.12f", p.sg.kernel.coef[j]);
    }
    fprintf(f, "\n");
    fclose(f);
}

static int dump_signals(const char *dir, uint32_t seconds)
{
    char           path[512];
    FILE          *fraw;
    FILE          *ffilt;
    hal_sim_cfg_t  cfg;
    ecg_pipeline_t p;
    uint32_t       n, total = seconds * ECG_SAMPLE_RATE_HZ;
    uint32_t       beats = 0u;

    snprintf(path, sizeof(path), "%s/raw_signal.csv", dir);
    fraw = fopen(path, "w");
    if (fraw == NULL) {
        printf("cannot write %s\n", path);
        return -1;
    }
    snprintf(path, sizeof(path), "%s/filtered_signal.csv", dir);
    ffilt = fopen(path, "w");
    if (ffilt == NULL) {
        fclose(fraw);
        printf("cannot write %s\n", path);
        return -1;
    }

    hal_sim_reset();
    hal_sim_default_cfg(&cfg);
    cfg.heart_rate_bpm = 72.0;
    cfg.seed           = 0x2468ACE0u;
    cfg.enable_hum     = 1;
    cfg.enable_wander  = 1;
    cfg.enable_noise   = 1;
    hal_sim_set_cfg(&cfg);
    ecg_pipeline_init(&p, ECG_SAMPLE_RATE_HZ, NULL);

    fprintf(fraw, "sample,adc_count,electrode_uv,clean_uv,hum_uv,wander_uv\n");
    fprintf(ffilt, "sample,notch_uv,filtered_uv,beat\n");

    for (n = 0u; n < total; n++) {
        ecg_hr_beat_t beat;
        uint16_t      counts = hal_sim_adc_count(&cfg, n);
        double        y      = (double)ecg_pipeline_process(&p, counts, &beat);
        double        t      = (double)n / (double)ECG_SAMPLE_RATE_HZ;
        double        electro = ((double)counts - (double)p.cal.intercept_counts) *
                                (double)p.cal.electrode_uv_per_lsb;
        double        clean = hal_sim_ecg_mv(&cfg, t) * 1000.0;
        double        hum   = cfg.hum_amp_mv * sin(ECG_TWO_PI * cfg.hum_freq_hz * t + 0.37) *
                              cfg.afe_gain;
        double        wand  = cfg.wander_amp_mv * sin(ECG_TWO_PI * cfg.wander_freq_hz * t + 0.11) *
                              cfg.afe_gain;
        int           is_beat = (beat.klass != ECG_BEAT_NONE) ? 1 : 0;

        if (is_beat != 0) {
            beats++;
        }
        fprintf(fraw, "%u,%u,%.6f,%.6f,%.6f,%.6f\n", n, (unsigned)counts, electro, clean, hum, wand);
        fprintf(ffilt, "%u,%.6f,%.6f,%d\n", n, (double)p.last_notch_uv, y, is_beat);
    }
    fclose(fraw);
    fclose(ffilt);

    printf("dumped %u samples (%u s) and %u beats\n", total, seconds, beats);
    return 0;
}

static int write_dump(const char *dir)
{
    dump_coefficients(dir);
    return dump_signals(dir, 30u);
}

/* ------------------------------------------------------------------ */
/* demo                                                                */
/* ------------------------------------------------------------------ */
static void run_demo(void)
{
    hal_sim_cfg_t  cfg;
    ecg_pipeline_t p;
    uint32_t       n, total = 10u * ECG_SAMPLE_RATE_HZ;
    uint32_t       beats = 0u;
    static double  sec_buf[ECG_SAMPLE_RATE_HZ];
    static const char ramp[] = " .:-=+*#%@";

    printf("----------------------------------------------------------------\n");
    printf("ECG node demo - 10 s of simulated acquisition at %.0f Hz\n",
           (double)ECG_SAMPLE_RATE_HZ);
    printf("----------------------------------------------------------------\n");

    hal_sim_default_cfg(&cfg);
    cfg.heart_rate_bpm = 72.0;
    ecg_pipeline_init(&p, ECG_SAMPLE_RATE_HZ, NULL);

    printf("  chain: ADC -> calibration -> 0.5 Hz baseline -> 50 Hz notch (2 x biquad)\n"
           "         -> Savitzky-Golay(order %u, window %u) -> QRS detector\n",
           ECG_SG_DEFAULT_ORDER, ECG_SG_DEFAULT_WINDOW);
    printf("  injected: %.2f mVpp 50 Hz hum, %.2f mVpp %.2f Hz wander, %.0f uV rms noise\n\n",
           2.0 * cfg.hum_amp_mv, 2.0 * cfg.wander_amp_mv, cfg.wander_freq_hz,
           cfg.noise_rms_mv * 1000.0);

    for (n = 0u; n < total; n++) {
        ecg_hr_beat_t beat;
        uint16_t      counts = hal_sim_adc_count(&cfg, n);
        double        y      = (double)ecg_pipeline_process(&p, counts, &beat);
        uint32_t      sec    = n / ECG_SAMPLE_RATE_HZ;

        sec_buf[n % ECG_SAMPLE_RATE_HZ] = y;

        if (beat.klass != ECG_BEAT_NONE) {
            beats++;
            printf("  t=%2u.%03u s  beat #%-3u  R=%-8.0f uV  RR=%4u ms  "
                   "BPM inst %6.1f / avg %6.1f  SQI %3u %s\n",
                   sec, (n % ECG_SAMPLE_RATE_HZ) * 1000u / ECG_SAMPLE_RATE_HZ,
                   beats, (double)beat.amplitude_uv, beat.rr_samples * 4u,
                   (double)beat.bpm_instant, (double)beat.bpm_average,
                   ecg_hr_quality(&p.hr),
                   (beat.klass == ECG_BEAT_ECTOPIC) ? "(ectopic)" : "");
        }

        /* one 80-column trace line per second of filtered signal */
        if ((n + 1u) % ECG_SAMPLE_RATE_HZ == 0u) {
            char     line[81];
            uint32_t c;
            for (c = 0u; c < 80u; c++) {
                uint32_t idx = (c * ECG_SAMPLE_RATE_HZ) / 80u;
                double   v   = sec_buf[idx];
                int      bin = (int)(((v + 200.0) / 1200.0) * 9.0);
                if (bin < 0) {
                    bin = 0;
                }
                if (bin > 9) {
                    bin = 9;
                }
                line[c] = ramp[bin];
            }
            line[80] = '\0';
            printf("  [%2u s] |%s|\n", sec, line);
        }
    }

    {
        ecg_hr_hrv_t hrv;
        ecg_hr_get_hrv(&p.hr, &hrv);
        printf("\n  beats %u, BPM %.2f, mean RR %.1f ms, SDNN %.2f ms, RMSSD %.2f ms, SQI %u\n",
               p.hr.beats_total, (double)p.hr.bpm_avg, (double)hrv.mean_rr_ms,
               (double)hrv.sdnn_ms, (double)hrv.rmssd_ms, ecg_hr_quality(&p.hr));
        printf("  processed samples %u, Savitzky-Golay primed after %u samples\n",
               p.processed, p.sg_prime_count);
        printf("  scale: ' ' = -200 uV, '@' = +1000 uV (one column = 3.1 ms)\n");
    }
}

static void run_uart_demo(void)
{
    static integ_ctx_t ctx;
    ecg_frame_sync_t   sync;
    ecg_frame_t        frame;
    ecg_sync_result_t  r;
    const uint8_t     *uart;
    uint32_t           uart_len, i;
    uint32_t           frames = 0u;
    uint32_t           sample_blocks = 0u;
    uint32_t           hr_reports = 0u;
    uint32_t           seconds = 20u;

    ctx.filtered_cap = seconds * ECG_SAMPLE_RATE_HZ + 4096u;
    ctx.filtered     = NULL;

    printf("----------------------------------------------------------------\n");
    printf("uplink demo - %u s through DMA -> ring -> frames -> UART\n", seconds);
    printf("----------------------------------------------------------------\n");

    hal_sim_reset();
    hal_sim_default_cfg(&ctx.cfg);
    ctx.cfg.heart_rate_bpm = 72.0;
    ctx.cfg.seed = 0x5A5A1234u;
    hal_sim_set_cfg(&ctx.cfg);
    ctx.filtered_n = 0u;
    (void)ecg_device_init(&ctx.dev);
    (void)ecg_device_start(&ctx.dev);
    (void)hal_sim_run(&ctx.cfg, seconds * ECG_SAMPLE_RATE_HZ, 8u, integ_main_loop, &ctx);

    uart     = hal_sim_uart_buffer();
    uart_len = hal_sim_uart_len();
    ecg_frame_sync_init(&sync);
    for (i = 0u; i < uart_len; i++) {
        r = ecg_frame_sync_feed(&sync, uart[i], &frame);
        if (r == ECG_SYNC_FRAME) {
            frames++;
            if (frame.type == (uint8_t)ECG_FRAME_ADC_PACK12 ||
                frame.type == (uint8_t)ECG_FRAME_ADC_DELTA) {
                sample_blocks++;
            } else if (frame.type == (uint8_t)ECG_FRAME_HR_REPORT) {
                ecg_hr_report_t rep;
                if (ecg_frame_parse_hr(&frame, &rep) == 0 && hr_reports < 6u) {
                    printf("  HR frame seq %-5u  inst %6.1f BPM  avg %6.1f BPM  "
                           "RR %4u ms  SQI %3u\n", frame.seq,
                           (double)rep.bpm_x10 / 10.0, (double)rep.bpm_avg_x10 / 10.0,
                           rep.rr_ms, rep.quality);
                    hr_reports++;
                } else if (ecg_frame_parse_hr(&frame, &rep) == 0) {
                    hr_reports++;
                }
            }
        }
    }
    {
        const ecg_dev_stats_t *st = ecg_device_stats(&ctx.dev);
        printf("\n  %u frames (%u sample blocks + %u HR reports) in %u bytes = %.1f B/s\n",
               frames, sample_blocks, hr_reports, uart_len,
               (double)uart_len / (double)seconds);
        printf("  raw int16 would need %u B/s, this protocol uses %.1f %% of it\n",
               ECG_SAMPLE_RATE_HZ * 2u,
               100.0 * (double)uart_len / (double)seconds / (double)(ECG_SAMPLE_RATE_HZ * 2u));
        printf("  CRC errors %u, format errors %u, resync slips %u\n",
               sync.frames_crc_err, sync.frames_fmt_err, sync.resync_slips);
        printf("  device: beats %u, samples %u, dropped %u\n",
               st->beats, st->samples_acquired, st->samples_dropped);
    }
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    const char *dump_dir = NULL;
    int         demo     = 0;
    int         uart_demo = 0;
    int         i;
    clock_t     t0, t1;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "demo") == 0) {
            demo = 1;
        } else if (strcmp(argv[i], "uart-demo") == 0) {
            uart_demo = 1;
        } else if (strcmp(argv[i], "--dump") == 0 && (i + 1) < argc) {
            dump_dir = argv[++i];
        }
    }

    printf("================================================================\n");
    printf(" STM32 ECG acquisition node - host test suite\n");
    printf(" %u Hz, 12-bit ADC, 50 Hz notch Q=%.0f, Savitzky-Golay order %u window %u\n",
           ECG_SAMPLE_RATE_HZ, (double)ECG_NOTCH_Q, ECG_SG_DEFAULT_ORDER,
           ECG_SG_DEFAULT_WINDOW);
    printf("================================================================\n\n");

    if (demo != 0 || uart_demo != 0) {
        if (demo != 0) {
            run_demo();
        }
        if (uart_demo != 0) {
            run_uart_demo();
        }
        return 0;
    }

    t0 = clock();
    test_ring_buffer_all();
    test_filters_all();
    test_protocol_all();
    test_heart_rate_all();
    test_pipeline_integration();

    if (dump_dir != NULL) {
        printf("== dump\n");
        if (write_dump(dump_dir) != 0) {
            printf("   FAILED to write the CSV dump\n\n");
            return 2;
        }
        printf("   ok\n\n");
    }

    t1 = clock();

    printf("================================================================\n");
    printf(" MEASURED VALUES\n");
    printf("================================================================\n");
    for (i = 0; i < s_meas_count; i++) {
        printf("  %-44s %14.4f %s\n", s_meas[i].key, s_meas[i].value, s_meas[i].unit);
    }
    printf("================================================================\n");
    printf(" suites %d, failed suites %d, checks %d, failed checks %d, %.2f s\n",
           s_suites, s_suites_failed, s_checks, s_failures,
           (double)(t1 - t0) / (double)CLOCKS_PER_SEC);
    printf(" RESULT: %s\n", (s_failures == 0) ? "ALL TESTS PASSED" : "FAILURES PRESENT");
    printf("================================================================\n");
    return (s_failures == 0) ? 0 : 1;
}
