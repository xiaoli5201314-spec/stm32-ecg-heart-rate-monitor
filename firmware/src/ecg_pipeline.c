/*
 * ecg_pipeline.c - calibration, DSP chain and device integration.
 *
 * Written as two layers on purpose:
 *
 *   ecg_pipeline_t  pure signal processing, no hardware, trivially testable
 *   ecg_device_t    DMA + ring buffer + framing + UART, one instance bound to
 *                   the interrupt vector (s_dev)
 */
#include "ecg_pipeline.h"
#include "hal.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* Calibration                                                         */
/* ------------------------------------------------------------------ */
void ecg_cal_defaults(ecg_cal_t *cal)
{
    if (cal == NULL) {
        return;
    }
    memset(cal, 0, sizeof(*cal));
    /* nominal: 1 mV at the electrode becomes ECG_AFE_GAIN_NOMINAL mV at the ADC
     * input, and one ADC count is ECG_ADC_LSB_UV microvolts. */
    cal->slope_counts_per_mv  = (ecg_real_t)(1000.0 / ECG_ELECTRODE_UV_PER_LSB);
    cal->intercept_counts     = (ecg_real_t)0.0;
    cal->r2                   = (ecg_real_t)1.0;
    cal->nonlinearity_pct_fs  = (ecg_real_t)0.0;
    cal->afe_gain             = (ecg_real_t)ECG_AFE_GAIN_NOMINAL;
    cal->electrode_uv_per_lsb = (ecg_real_t)ECG_ELECTRODE_UV_PER_LSB;
    cal->n_points             = 0u;
    cal->valid                = 0u;
}

int ecg_cal_fit(ecg_cal_t *cal, const ecg_cal_point_t *points, size_t n)
{
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0, syy = 0.0;
    double slope, intercept, mean_y, ss_res = 0.0, ss_tot = 0.0;
    double max_dev = 0.0, y_min, y_max, span, r2;
    size_t i;

    if (cal == NULL || points == NULL || n < 2u || n > (size_t)ECG_CAL_MAX_POINTS) {
        return -1;
    }

    for (i = 0u; i < n; i++) {
        double x = (double)points[i].mv;
        double y = (double)points[i].counts;
        sx  += x;
        sy  += y;
        sxx += x * x;
        sxy += x * y;
        syy += y * y;
    }
    {
        double dn     = (double)n;
        double denom  = (dn * sxx) - (sx * sx);
        if (ECG_FABS(denom) < 1e-12) {
            return -1;                     /* all calibration points identical */
        }
        slope     = ((dn * sxy) - (sx * sy)) / denom;
        intercept = (sy - (slope * sx)) / dn;
    }
    if (ECG_FABS(slope) < 1e-12) {
        return -1;
    }

    mean_y = sy / (double)n;
    y_min  = (double)points[0].counts;
    y_max  = y_min;
    for (i = 0u; i < n; i++) {
        double x   = (double)points[i].mv;
        double y   = (double)points[i].counts;
        double fit = (slope * x) + intercept;
        double dev = y - fit;
        double dt  = y - mean_y;

        if (ECG_FABS(dev) > max_dev) {
            max_dev = ECG_FABS(dev);
        }
        ss_res += dev * dev;
        ss_tot += dt * dt;
        if (y < y_min) {
            y_min = y;
        }
        if (y > y_max) {
            y_max = y;
        }
    }
    span = y_max - y_min;
    r2   = (ss_tot > 0.0) ? (1.0 - (ss_res / ss_tot)) : 1.0;

    cal->slope_counts_per_mv  = (ecg_real_t)slope;
    cal->intercept_counts     = (ecg_real_t)intercept;
    cal->r2                   = (ecg_real_t)r2;
    cal->nonlinearity_pct_fs  = (ecg_real_t)((span > 0.0) ? ((100.0 * max_dev) / span) : 0.0);
    /* measured analog gain: slope [counts/mV] * LSB [uV/count] / 1000 [uV/mV] */
    cal->afe_gain             = (ecg_real_t)((slope * ECG_ADC_LSB_UV) / 1000.0);
    cal->electrode_uv_per_lsb = (ecg_real_t)(1000.0 / slope);
    cal->n_points             = (uint8_t)n;
    cal->valid                = 1u;
    (void)syy;
    return 0;
}

ecg_real_t ecg_cal_counts_to_uv(const ecg_cal_t *cal, ecg_real_t counts)
{
    if (cal == NULL) {
        return counts;
    }
    return (counts - cal->intercept_counts) * cal->electrode_uv_per_lsb;
}

ecg_real_t ecg_cal_plateau_mean(const uint16_t *counts, size_t n)
{
    double   acc = 0.0;
    size_t   i;
    uint32_t count = 0u;

    if (counts == NULL || n == 0u) {
        return 0.0;
    }
    for (i = 0u; i < n; i++) {
        uint16_t a = counts[i];
        uint16_t b = counts[i];
        uint16_t c = counts[i];
        uint16_t med;

        if (i > 0u) {
            a = counts[i - 1u];
        }
        if (i + 1u < n) {
            c = counts[i + 1u];
        }
        /* 3-point median: kills isolated spikes / ADC glitches */
        if (a > b) { uint16_t t = a; a = b; b = t; }
        if (b > c) { uint16_t t = b; b = c; c = t; }
        if (a > b) { uint16_t t = a; a = b; b = t; }
        med = b;
        acc += (double)med;
        count++;
    }
    if (count == 0u) {
        return 0.0;
    }
    return (ecg_real_t)(acc / (double)count);
}

/* ------------------------------------------------------------------ */
/* DSP chain                                                           */
/* ------------------------------------------------------------------ */
void ecg_pipeline_init(ecg_pipeline_t *p, uint32_t sample_rate_hz, const ecg_cal_t *cal)
{
    uint32_t i;
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->sample_rate_hz = (sample_rate_hz != 0u) ? sample_rate_hz : ECG_SAMPLE_RATE_HZ;

    if (cal != NULL) {
        p->cal = *cal;
    } else {
        ecg_cal_defaults(&p->cal);
    }

    /* two cascaded 1-pole sections: 2 poles at 0.5 Hz reject respiration
     * wander (0.2..0.35 Hz) by ~14 dB while staying inside the 0.5 Hz
     * monitoring-mode high-pass allowed by IEC 60601-2-27 */
    for (i = 0u; i < ECG_BASELINE_HP_SECTIONS; i++) {
        (void)ecg_hpf1_init(&p->baseline[i], (double)p->sample_rate_hz,
                            ECG_BASELINE_HP_HZ);
    }
    (void)ecg_notch_init(&p->notch, (double)p->sample_rate_hz, ECG_NOTCH_F0_HZ,
                         ECG_NOTCH_Q, ECG_NOTCH_STAGES);
    {
        ecg_savgol_t sg;
        if (ecg_savgol_design(&sg, ECG_SG_DEFAULT_ORDER, ECG_SG_DEFAULT_WINDOW) == 0) {
            ecg_savgol_stream_init(&p->sg, &sg);
        }
    }
    ecg_hr_init(&p->hr, (ecg_real_t)p->sample_rate_hz);
}

void ecg_pipeline_reset(ecg_pipeline_t *p)
{
    uint32_t fs;
    ecg_cal_t cal;
    if (p == NULL) {
        return;
    }
    fs  = p->sample_rate_hz;
    cal = p->cal;
    ecg_pipeline_init(p, fs, &cal);
}

ecg_real_t ecg_pipeline_process(ecg_pipeline_t *p, uint16_t raw_count, ecg_hr_beat_t *beat_out)
{
    ecg_real_t uv, ac, notched, y;
    int        primed = 0;
    uint32_t   i;

    if (p == NULL) {
        return (ecg_real_t)0.0;
    }
    if (beat_out != NULL) {
        memset(beat_out, 0, sizeof(*beat_out));
        beat_out->klass = ECG_BEAT_NONE;
    }

    /* 1. counts -> uV at the electrode */
    uv = ecg_cal_counts_to_uv(&p->cal, (ecg_real_t)raw_count);
    p->last_raw_counts = (ecg_real_t)raw_count;

    /* 2. residual baseline wander (the analog channel already high-passes at
     *    0.05/0.5 Hz; this removes what is left after the ADC)              */
    ac = uv;
    for (i = 0u; i < ECG_BASELINE_HP_SECTIONS; i++) {
        ac = ecg_hpf1_process(&p->baseline[i], ac);
    }
    p->last_dc_uv = ac;

    /* 3. 50 Hz notch */
    notched = ecg_notch_process(&p->notch, ac);
    p->last_notch_uv = notched;

    /* 4. Savitzky-Golay smoothing */
    y = ecg_savgol_stream_push(&p->sg, notched, &primed);
    if (primed != 0 && p->sg_prime_count == 0u) {
        /* record how many input samples the window needed before the first
         * filtered output appeared: exactly ECG_SG_DEFAULT_WINDOW */
        p->sg_prime_count = p->processed + 1u;
    }
    p->last_uv = y;
    p->processed++;

    /* 5. QRS detection on the conditioned signal */
    (void)ecg_hr_process(&p->hr, y, beat_out);
    return y;
}

void ecg_pipeline_process_counts(ecg_pipeline_t *p, const uint16_t *counts, size_t n,
                                 ecg_real_t *out_uv)
{
    size_t i;
    if (p == NULL || counts == NULL) {
        return;
    }
    for (i = 0u; i < n; i++) {
        ecg_real_t y = ecg_pipeline_process(p, counts[i], NULL);
        if (out_uv != NULL) {
            out_uv[i] = y;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Device layer                                                        */
/* ------------------------------------------------------------------ */
static ecg_device_t *s_dev = NULL;      /* the single node bound to the ISR */

static int16_t sat_i16(double v)
{
    if (v > 32767.0) {
        return (int16_t)32767;
    }
    if (v < -32768.0) {
        return (int16_t)-32768;
    }
    return (int16_t)((v >= 0.0) ? (v + 0.5) : (v - 0.5));
}

static uint32_t dev_now_ms(const ecg_device_t *dev)
{
    return (uint32_t)(((uint64_t)dev->dsp.processed * 1000u) /
                      (uint64_t)dev->dsp.sample_rate_hz);
}

static void dev_emit(ecg_device_t *dev, size_t len)
{
    uint32_t written;
    if (dev == NULL || len == 0u) {
        return;
    }
    written = ecg_ring_write(&dev->tx_ring, dev->frame_scratch, (uint32_t)len);
    if (written == (uint32_t)len) {
        dev->stats.frames_tx++;
    } else {
        dev->stats.frames_dropped++;
    }
}

static void dev_emit_sample_block(ecg_device_t *dev)
{
    size_t len = 0u;
    uint8_t type;

    if (dev->sample_block_fill == 0u) {
        return;
    }
    type = ecg_frame_build_samples(dev->seq, dev->sample_block, dev->sample_block_fill,
                                   dev->frame_scratch, sizeof(dev->frame_scratch), &len);
    if (type == 0u || len == 0u) {
        dev->stats.frames_dropped++;
        return;
    }
    dev_emit(dev, len);
    dev->seq++;
}

static void dev_emit_hr(ecg_device_t *dev, const ecg_hr_beat_t *beat)
{
    ecg_hr_report_t rep;
    size_t          len;
    double          bpm_inst  = (double)beat->bpm_instant;
    double          bpm_avg   = (double)beat->bpm_average;
    long            inst_x10  = (long)((bpm_inst * 10.0) + 0.5);
    long            avg_x10   = (long)((bpm_avg * 10.0) + 0.5);
    unsigned long   rr_ms     = (unsigned long)(((double)beat->rr_samples * 1000.0) /
                                                (double)dev->dsp.sample_rate_hz + 0.5);

    memset(&rep, 0, sizeof(rep));
    rep.flags = ((beat->klass == ECG_BEAT_ECTOPIC) ? 0x02u : 0x00u) |
                ((beat->rr_accepted != 0) ? 0x01u : 0x00u);
    if (inst_x10 > 32767L) { inst_x10 = 32767L; }
    if (inst_x10 < -32768L) { inst_x10 = -32768L; }
    if (avg_x10 > 32767L) { avg_x10 = 32767L; }
    if (avg_x10 < -32768L) { avg_x10 = -32768L; }
    if (rr_ms > 65535UL) { rr_ms = 65535UL; }
    rep.bpm_x10     = (int16_t)inst_x10;
    rep.bpm_avg_x10 = (int16_t)avg_x10;
    rep.rr_ms       = (uint16_t)rr_ms;
    rep.quality     = ecg_hr_quality(&dev->dsp.hr);

    len = ecg_frame_build_hr(dev->seq, &rep, dev->frame_scratch, sizeof(dev->frame_scratch));
    if (len != 0u) {
        dev_emit(dev, len);
        dev->seq++;
    } else {
        dev->stats.frames_dropped++;
    }
}

static void dev_emit_status(ecg_device_t *dev)
{
    size_t len = ecg_frame_build_status(dev->seq, dev->status_flags,
                                        (uint16_t)ecg_ring_count(&dev->adc_ring),
                                        dev->stats.samples_dropped,
                                        dev->frame_scratch, sizeof(dev->frame_scratch));
    if (len != 0u) {
        dev_emit(dev, len);
        dev->seq++;
    }
}

int ecg_device_init(ecg_device_t *dev)
{
    if (dev == NULL) {
        return -1;
    }
    memset(dev, 0, sizeof(*dev));
    if (ecg_ring_init_u16(&dev->adc_ring, dev->adc_ring_storage, ECG_RING_CAPACITY) != 0) {
        return -1;
    }
    if (ecg_ring_init_u8(&dev->tx_ring, dev->tx_ring_storage, ECG_TX_QUEUE_CAPACITY) != 0) {
        return -1;
    }
    ecg_pipeline_init(&dev->dsp, ECG_SAMPLE_RATE_HZ, NULL);
    dev->seq = 0u;
    s_dev = dev;
    return 0;
}

int ecg_device_start(ecg_device_t *dev)
{
    int rc;

    if (dev == NULL) {
        return -1;
    }
    rc = hal_init();
    if (rc != 0) {
        return rc;
    }
    hal_set_adc_callbacks(ecg_device_on_dma_half, ecg_device_on_dma_full);

    /* Arm the DMA before the timer: the first conversion must land in a buffer
     * that is already owned by the DMA controller. */
    rc = hal_adc_start_dma(dev->dma_buffer, ECG_DMA_BUFFER_SIZE);
    if (rc != 0) {
        return rc;
    }
    rc = hal_timer_start(ECG_SAMPLE_RATE_HZ);
    if (rc != 0) {
        hal_adc_stop_dma();
        return rc;
    }
    dev->started_ms = hal_tick_ms();
    hal_gpio_write(HAL_PIN_STATUS_LED, 1);
    hal_gpio_write(HAL_PIN_AFE_ENABLE, 1);
    hal_gpio_write(HAL_PIN_RLD_ENABLE, 1);
    return 0;
}

void ecg_device_stop(ecg_device_t *dev)
{
    (void)dev;
    hal_timer_stop();
    hal_adc_stop_dma();
    hal_gpio_write(HAL_PIN_STATUS_LED, 0);
}

void ecg_device_on_dma_half(const uint16_t *samples, uint32_t count)
{
    ecg_device_t *dev = s_dev;
    uint32_t      written;

    if (dev == NULL || samples == NULL || count == 0u) {
        return;
    }
    dev->stats.isr_blocks++;
    written = ecg_ring_write_u16(&dev->adc_ring, samples, count);
    dev->stats.samples_acquired += written;
    if (written < count) {
        dev->stats.samples_dropped += (count - written);
        dev->stats.ring_overflows++;
    }
}

void ecg_device_on_dma_full(const uint16_t *samples, uint32_t count)
{
    ecg_device_on_dma_half(samples, count);
}

uint32_t ecg_device_flush_tx(ecg_device_t *dev)
{
    uint8_t  chunk[32];
    uint32_t total = 0u;

    if (dev == NULL) {
        return 0u;
    }
    /* hal_uart_write() is all-or-nothing: it either queues the whole buffer in
     * the driver TX ring or returns 0 when that ring is full. A short count can
     * therefore only mean a real overrun and is accounted as dropped bytes. */
    for (;;) {
        uint32_t got = ecg_ring_read(&dev->tx_ring, chunk, (uint32_t)sizeof(chunk));
        uint32_t sent;

        if (got == 0u) {
            break;
        }
        sent = hal_uart_write(chunk, got);
        total += sent;
        if (sent < got) {
            dev->stats.bytes_dropped += (got - sent);
            dev->stats.frames_dropped++;
            break;
        }
    }
    dev->stats.bytes_tx += total;
    return total;
}

uint32_t ecg_device_task(ecg_device_t *dev, ecg_real_t *out_uv, uint32_t out_max)
{
    uint16_t block[ECG_MAX_SAMPLES_PER_TICK];
    uint32_t produced = 0u;

    if (dev == NULL) {
        return 0u;
    }
    for (;;) {
        uint32_t got = ecg_ring_read_u16(&dev->adc_ring, block, ECG_MAX_SAMPLES_PER_TICK);
        uint32_t i;

        if (got == 0u) {
            break;
        }
        for (i = 0u; i < got; i++) {
            ecg_hr_beat_t beat;
            ecg_real_t    y = ecg_pipeline_process(&dev->dsp, block[i], &beat);

            if (out_uv != NULL && produced < out_max) {
                out_uv[produced] = y;
            }
            produced++;

            /* compact uplink: accumulate ECG samples into fixed size frames */
            dev->sample_block[dev->sample_block_fill] = sat_i16((double)y);
            dev->sample_block_fill++;
            if (dev->sample_block_fill >= ECG_FRAME_SAMPLES) {
                dev_emit_sample_block(dev);
                dev->sample_block_fill = 0u;
            }

            if (beat.klass != ECG_BEAT_NONE) {
                uint32_t now = dev_now_ms(dev);
                dev->stats.beats++;
                if (dev->on_beat != NULL) {
                    dev->on_beat(&beat, dev->on_beat_user);
                }
                if ((now - dev->last_hr_report_ms) >= 300u || dev->last_hr_report_ms == 0u) {
                    dev_emit_hr(dev, &beat);
                    dev->last_hr_report_ms = now;
                }
            }
        }
    }

    /* low-rate housekeeping frame, once every 5 s */
    {
        uint32_t now = dev_now_ms(dev);
        if ((now - dev->last_status_ms) >= 5000u) {
            dev_emit_status(dev);
            dev->last_status_ms = now;
        }
    }

    (void)ecg_device_flush_tx(dev);
    return produced;
}

const ecg_dev_stats_t *ecg_device_stats(const ecg_device_t *dev)
{
    return (dev == NULL) ? NULL : &dev->stats;
}

int ecg_device_apply_calibration(ecg_device_t *dev, const ecg_cal_point_t *points, size_t n)
{
    ecg_cal_t cal;
    if (dev == NULL || points == NULL) {
        return -1;
    }
    if (ecg_cal_fit(&cal, points, n) != 0) {
        return -1;
    }
    dev->dsp.cal = cal;
    return 0;
}

void ecg_device_set_beat_callback(ecg_device_t *dev,
                                  void (*cb)(const ecg_hr_beat_t *, void *), void *user)
{
    if (dev == NULL) {
        return;
    }
    dev->on_beat = cb;
    dev->on_beat_user = user;
}
