/*
 * hal_stub.c - PC simulation of the analog front end, ADC/DMA and UART.
 *
 * Also provides the *weak* default implementations of the hal_ops_t table, so
 * that a port which only wants to replace one operation can simply define its
 * own strong symbol (see hal.h).
 */
#include "hal_stub.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* shared state                                                        */
/* ------------------------------------------------------------------ */
#define SIM_UART_CAPACITY   (1u << 20)

static hal_sim_cfg_t   s_cfg;
static uint16_t       *s_dma_buf      = NULL;
static uint32_t        s_dma_len      = 0u;
static uint32_t        s_sample_index = 0u;
static uint32_t        s_isr_count    = 0u;
static uint32_t        s_timer_starts = 0u;
static uint32_t        s_adc_starts   = 0u;
static uint32_t        s_tick_ms      = 0u;
static int             s_pin[HAL_PIN_COUNT];
static uint8_t         s_uart_buf[SIM_UART_CAPACITY];
static uint32_t        s_uart_len     = 0u;
static uint32_t        s_uart_chunk   = 0u;
static uint32_t        s_rng_state    = 0x1234567u;
static int             s_gauss_ready  = 0;
static double          s_gauss_spare  = 0.0;
static hal_adc_callback_t s_cb_half   = NULL;
static hal_adc_callback_t s_cb_full   = NULL;

/* ------------------------------------------------------------------ */
/* configuration                                                       */
/* ------------------------------------------------------------------ */
void hal_sim_default_cfg(hal_sim_cfg_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->sample_rate_hz = (double)ECG_SAMPLE_RATE_HZ;
    cfg->heart_rate_bpm = 72.0;

    /* amplitudes from a textbook surface-ECG morphology, in mV at the
     * electrode (so ~1 mV R wave, which is what the calibration standard
     * square wave is defined against) */
    cfg->r_amp_mv  =  1.00;
    cfg->p_amp_mv  =  0.13;
    cfg->q_amp_mv  = -0.12;
    cfg->s_amp_mv  = -0.26;
    cfg->t_amp_mv  =  0.32;

    cfg->p_offset_s  = -0.160;
    cfg->q_offset_s  = -0.032;
    cfg->s_offset_s  =  0.036;
    cfg->t_offset_s  =  0.230;

    cfg->p_sigma_s   = 0.024;
    cfg->qrs_sigma_s = 0.015;    /* R wave half width ~30 ms, i.e. a sharp QRS */
    cfg->t_sigma_s   = 0.048;

    cfg->hum_amp_mv      = 0.30;   /* ~30 % of the R wave: worst-case mains */
    cfg->hum_freq_hz     = 50.0;
    cfg->wander_amp_mv   = 0.60;   /* respiration, 0.25 Hz                   */
    cfg->wander_freq_hz  = 0.25;
    cfg->noise_rms_mv    = 0.030;
    cfg->enable_hum      = 1;
    cfg->enable_wander   = 1;
    cfg->enable_noise    = 1;

    cfg->afe_gain    = ECG_AFE_GAIN_NOMINAL;
    cfg->afe_ref_mv  = ECG_AFE_REF_MV;
    cfg->adc_vref_mv = ECG_ADC_VREF_MV;
    cfg->adc_bits    = (int)ECG_ADC_BITS;
    cfg->seed        = 0x5EED1234u;
}

void hal_sim_set_cfg(const hal_sim_cfg_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    s_cfg = *cfg;
    s_rng_state   = (cfg->seed != 0u) ? cfg->seed : 1u;
    s_gauss_ready = 0;
    s_sample_index = 0u;
}

const hal_sim_cfg_t *hal_sim_get_cfg(void)
{
    return &s_cfg;
}

void hal_sim_reset(void)
{
    hal_sim_default_cfg(&s_cfg);
    s_rng_state    = s_cfg.seed;
    s_gauss_ready  = 0;
    s_dma_buf      = NULL;
    s_dma_len      = 0u;
    s_sample_index = 0u;
    s_isr_count    = 0u;
    s_timer_starts = 0u;
    s_adc_starts   = 0u;
    s_tick_ms      = 0u;
    s_uart_len     = 0u;
    s_uart_chunk   = 0u;
    s_cb_half      = NULL;
    s_cb_full      = NULL;
    memset(s_pin, 0, sizeof(s_pin));
}

/* ------------------------------------------------------------------ */
/* random numbers                                                      */
/* ------------------------------------------------------------------ */
uint32_t hal_sim_rng(uint32_t *state)
{
    uint32_t x = (state != NULL) ? *state : 0x1234567u;
    /* xorshift32: cheap, deterministic, good enough for a noise model */
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    if (state != NULL) {
        *state = x;
    }
    return x;
}

static double sim_uniform(hal_sim_cfg_t *cfg)
{
    uint32_t r = hal_sim_rng(&cfg->seed);
    return ((double)(r >> 8) + 0.5) / 16777216.0;   /* (0,1) */
}

double hal_sim_gauss(hal_sim_cfg_t *cfg)
{
    /* Box-Muller with a cached second value */
    double u1, u2, r, z;
    if (cfg == NULL) {
        return 0.0;
    }
    if (s_gauss_ready != 0) {
        s_gauss_ready = 0;
        return s_gauss_spare;
    }
    u1 = sim_uniform(cfg);
    u2 = sim_uniform(cfg);
    if (u1 < 1e-12) {
        u1 = 1e-12;
    }
    r = sqrt(-2.0 * log(u1));
    z = r * cos(ECG_TWO_PI * u2);
    s_gauss_spare = r * sin(ECG_TWO_PI * u2);
    s_gauss_ready = 1;
    return z;
}

/* ------------------------------------------------------------------ */
/* ECG waveform model                                                  */
/* ------------------------------------------------------------------ */
static double gauss_pulse(double x, double sigma)
{
    if (sigma <= 0.0) {
        return 0.0;
    }
    return exp(-(x * x) / (2.0 * sigma * sigma));
}

/* Gaussian pulse at `off` seconds relative to the R peak, wrapped over one
 * cardiac cycle so the shape is continuous across the beat boundary. */
static double wrapped_pulse(double phase, double rr, double off, double amp, double sigma)
{
    double d = phase - off;
    return amp * (gauss_pulse(d, sigma) +
                  gauss_pulse(d - rr, sigma) +
                  gauss_pulse(d + rr, sigma));
}

double hal_sim_beat_time_s(const hal_sim_cfg_t *cfg, uint32_t k)
{
    double rr;
    if (cfg == NULL || cfg->heart_rate_bpm <= 0.0) {
        return 0.0;
    }
    rr = 60.0 / cfg->heart_rate_bpm;
    return (double)k * rr;
}

double hal_sim_ecg_mv(const hal_sim_cfg_t *cfg, double t_s)
{
    double rr, phase, v = 0.0;

    if (cfg == NULL || cfg->heart_rate_bpm <= 0.0) {
        return 0.0;
    }
    rr    = 60.0 / cfg->heart_rate_bpm;
    phase = fmod(t_s, rr);
    if (phase < 0.0) {
        phase += rr;
    }

    v += wrapped_pulse(phase, rr, 0.0,             cfg->r_amp_mv, cfg->qrs_sigma_s);
    v += wrapped_pulse(phase, rr, cfg->q_offset_s, cfg->q_amp_mv, cfg->qrs_sigma_s * 0.6);
    v += wrapped_pulse(phase, rr, cfg->s_offset_s, cfg->s_amp_mv, cfg->qrs_sigma_s * 0.7);
    v += wrapped_pulse(phase, rr, cfg->p_offset_s, cfg->p_amp_mv, cfg->p_sigma_s);
    v += wrapped_pulse(phase, rr, cfg->t_offset_s, cfg->t_amp_mv, cfg->t_sigma_s);
    return v;
}

double hal_sim_channel_mv(hal_sim_cfg_t *cfg, uint32_t n)
{
    double t, ecg, hum = 0.0, wander = 0.0, noise = 0.0, out;

    if (cfg == NULL) {
        return 0.0;
    }
    t   = (double)n / cfg->sample_rate_hz;
    ecg = hal_sim_ecg_mv(cfg, t);

    if (cfg->enable_hum != 0) {
        /* mains pickup, phase-locked to the sample clock so the test is
         * reproducible; a small deliberate phase offset keeps it realistic */
        hum = cfg->hum_amp_mv * sin(ECG_TWO_PI * cfg->hum_freq_hz * t + 0.37);
    }
    if (cfg->enable_wander != 0) {
        wander = cfg->wander_amp_mv * sin(ECG_TWO_PI * cfg->wander_freq_hz * t + 0.11);
    }
    if (cfg->enable_noise != 0 && cfg->noise_rms_mv > 0.0) {
        noise = cfg->noise_rms_mv * hal_sim_gauss(cfg);
    }

    /* the instrumentation amplifier sees the electrode signal only (the DC
     * electrode offset is rejected by the AC-coupled first stage), then the
     * level-shift stage adds the mid-rail bias */
    out = cfg->afe_ref_mv + (cfg->afe_gain * (ecg + hum + wander + noise));
    return out;
}

uint16_t hal_sim_adc_count(hal_sim_cfg_t *cfg, uint32_t n)
{
    double   v_mv, lsb_mv, code;
    uint32_t full_scale;

    if (cfg == NULL) {
        return 0u;
    }
    v_mv       = hal_sim_channel_mv(cfg, n);
    full_scale = 1u << (uint32_t)cfg->adc_bits;
    lsb_mv     = cfg->adc_vref_mv / (double)full_scale;
    code       = v_mv / lsb_mv;                 /* ideal ADC, then saturate */

    if (code < 0.0) {
        code = 0.0;
    }
    if (code > (double)(full_scale - 1u)) {
        code = (double)(full_scale - 1u);
    }
    return (uint16_t)(code + 0.5);
}

long hal_sim_nearest_r_peak(const hal_sim_cfg_t *cfg, uint32_t n, uint32_t span)
{
    double   t, rr;
    long     k, best_k = -1;
    double   best_d = 1e30;
    uint32_t i;

    if (cfg == NULL || cfg->heart_rate_bpm <= 0.0) {
        return -1;
    }
    t  = (double)n / cfg->sample_rate_hz;
    rr = 60.0 / cfg->heart_rate_bpm;
    k  = (long)(t / rr);

    for (i = 0u; i < 3u; i++) {
        long   kk = k - 1 + (long)i;
        double bt, d;
        if (kk < 0) {
            continue;
        }
        bt = (double)kk * rr;
        d  = (t > bt) ? (t - bt) : (bt - t);
        if (d < best_d) {
            best_d = d;
            best_k = kk;
        }
    }
    if (best_k < 0) {
        return -1;
    }
    if ((double)span > 0.0 && best_d > ((double)span / cfg->sample_rate_hz)) {
        return -1;
    }
    return (long)((hal_sim_beat_time_s(cfg, (uint32_t)best_k) * cfg->sample_rate_hz) + 0.5);
}

void hal_sim_capture_plateau(hal_sim_cfg_t *cfg, double mv, uint16_t *out, size_t n)
{
    hal_sim_cfg_t local;
    size_t   i;
    double   saved_r;

    if (cfg == NULL || out == NULL || n == 0u) {
        return;
    }
    /* a calibration plateau has no cardiac activity, no hum and no wander:
     * only the calibrator step and the channel noise remain */
    local = *cfg;
    local.heart_rate_bpm = 0.0;
    local.enable_hum     = 0;
    local.enable_wander  = 0;
    local.r_amp_mv = local.p_amp_mv = local.q_amp_mv = 0.0;
    local.s_amp_mv = local.t_amp_mv = 0.0;

    saved_r = local.afe_ref_mv;
    for (i = 0u; i < n; i++) {
        double v_mv = saved_r + (local.afe_gain * mv) +
                      (local.noise_rms_mv * hal_sim_gauss(&local));
        double lsb  = local.adc_vref_mv / (double)(1u << (uint32_t)local.adc_bits);
        double code = v_mv / lsb;
        if (code < 0.0) {
            code = 0.0;
        }
        if (code > 4095.0) {
            code = 4095.0;
        }
        out[i] = (uint16_t)(code + 0.5);
    }
    /* keep the noise stream running across successive plateau captures so the
     * high and the low plateau are not driven by the same noise sequence */
    cfg->seed = local.seed;
}

/* ------------------------------------------------------------------ */
/* DMA / timer emulation                                               */
/* ------------------------------------------------------------------ */
uint32_t hal_sim_run(hal_sim_cfg_t *cfg, uint32_t n_samples,
                     uint32_t task_period_samples, hal_sim_task_fn task, void *user)
{
    uint32_t i, callbacks = 0u;

    if (cfg == NULL || s_dma_buf == NULL || s_dma_len == 0u) {
        return 0u;
    }
    for (i = 0u; i < n_samples; i++) {
        uint32_t slot = s_sample_index % s_dma_len;

        s_dma_buf[slot] = hal_sim_adc_count(cfg, s_sample_index);
        s_sample_index++;

        /* circular DMA: half-transfer when the first half is complete, then
         * transfer-complete when the second half is complete */
        if (((slot + 1u) % (s_dma_len / 2u)) == 0u) {
            int first_half = (slot + 1u) == (s_dma_len / 2u);
            callbacks++;
            s_isr_count++;
            if (first_half != 0) {
                if (s_cb_half != NULL) {
                    s_cb_half(&s_dma_buf[0], s_dma_len / 2u);
                }
            } else {
                if (s_cb_full != NULL) {
                    s_cb_full(&s_dma_buf[s_dma_len / 2u], s_dma_len / 2u);
                }
            }
        }
        if (task != NULL && task_period_samples != 0u &&
            ((i + 1u) % task_period_samples) == 0u) {
            task(user);
        }
    }
    return callbacks;
}

uint32_t hal_sim_isr_count(void)    { return s_isr_count; }
uint32_t hal_sim_timer_starts(void) { return s_timer_starts; }
uint32_t hal_sim_adc_starts(void)   { return s_adc_starts; }

/* ------------------------------------------------------------------ */
/* observations                                                        */
/* ------------------------------------------------------------------ */
const uint8_t *hal_sim_uart_buffer(void) { return s_uart_buf; }
uint32_t       hal_sim_uart_len(void)    { return s_uart_len; }

void hal_sim_uart_reset(void)
{
    s_uart_len = 0u;
    memset(s_uart_buf, 0, sizeof(s_uart_buf));
}

void hal_sim_uart_set_chunk(uint32_t bytes) { s_uart_chunk = bytes; }

int hal_sim_pin_level(hal_pin_t pin)
{
    if ((int)pin < 0 || (int)pin >= (int)HAL_PIN_COUNT) {
        return -1;
    }
    return s_pin[pin];
}

uint32_t hal_sim_tick(void) { return s_tick_ms; }

void hal_sim_advance_ms(uint32_t ms) { s_tick_ms += ms; }

/* ------------------------------------------------------------------ */
/* weak default operation table                                        */
/* ------------------------------------------------------------------ */
HAL_WEAK int hal_weak_init(void)
{
    /* nothing to bring up on a PC */
    s_tick_ms = 0u;
    return 0;
}

HAL_WEAK int hal_weak_timer_start(uint32_t sample_rate_hz)
{
    (void)sample_rate_hz;
    s_timer_starts++;
    return 0;
}

HAL_WEAK void hal_weak_timer_stop(void) { }

HAL_WEAK int hal_weak_adc_start_dma(uint16_t *buf, uint32_t total_len)
{
    if (buf == NULL || total_len == 0u || (total_len & 1u) != 0u) {
        return -1;
    }
    s_dma_buf = buf;
    s_dma_len = total_len;
    s_adc_starts++;
    return 0;
}

HAL_WEAK void hal_weak_adc_stop_dma(void)
{
    s_dma_buf = NULL;
    s_dma_len = 0u;
}

HAL_WEAK uint32_t hal_weak_uart_write(const uint8_t *data, uint32_t len)
{
    uint32_t n = len;

    if (data == NULL || len == 0u) {
        return 0u;
    }
    if (s_uart_chunk != 0u && n > s_uart_chunk) {
        n = s_uart_chunk;                 /* emulate a busy TX ring */
    }
    if (n > (SIM_UART_CAPACITY - s_uart_len)) {
        n = SIM_UART_CAPACITY - s_uart_len;
    }
    if (n == 0u) {
        return 0u;
    }
    memcpy(&s_uart_buf[s_uart_len], data, n);
    s_uart_len += n;
    return n;
}

HAL_WEAK int hal_weak_uart_tx_idle(void)
{
    return 1;
}

HAL_WEAK uint32_t hal_weak_tick_ms(void)
{
    return s_tick_ms;
}

HAL_WEAK void hal_weak_gpio_write(hal_pin_t pin, int level)
{
    if ((int)pin >= 0 && (int)pin < (int)HAL_PIN_COUNT) {
        s_pin[pin] = level;
    }
}

HAL_WEAK int hal_weak_gpio_read(hal_pin_t pin)
{
    if ((int)pin >= 0 && (int)pin < (int)HAL_PIN_COUNT) {
        return s_pin[pin];
    }
    return 0;
}

HAL_WEAK void hal_weak_delay_ms(uint32_t ms)
{
    s_tick_ms += ms;                      /* simulated time, no real sleep */
}

/* ------------------------------------------------------------------ */
/* operation table plumbing                                            */
/* ------------------------------------------------------------------ */
static const hal_ops_t s_weak_ops = {
    "pc-simulation-stub",
    hal_weak_init,
    hal_weak_timer_start,
    hal_weak_timer_stop,
    hal_weak_adc_start_dma,
    hal_weak_adc_stop_dma,
    hal_weak_uart_write,
    hal_weak_uart_tx_idle,
    hal_weak_tick_ms,
    hal_weak_gpio_write,
    hal_weak_gpio_read,
    hal_weak_delay_ms
};

static const hal_ops_t *s_ops = &s_weak_ops;

void hal_register_ops(const hal_ops_t *ops)
{
    s_ops = (ops != NULL) ? ops : &s_weak_ops;
}

const hal_ops_t *hal_ops(void)
{
    return s_ops;
}

void hal_set_adc_callbacks(hal_adc_callback_t on_half, hal_adc_callback_t on_full)
{
    s_cb_half = on_half;
    s_cb_full = on_full;
}

void hal_invoke_adc_half(const uint16_t *samples, uint32_t count)
{
    if (s_cb_half != NULL) {
        s_cb_half(samples, count);
    }
}

void hal_invoke_adc_full(const uint16_t *samples, uint32_t count)
{
    if (s_cb_full != NULL) {
        s_cb_full(samples, count);
    }
}

/* -------- convenience wrappers -------- */
int      hal_init(void)                              { return s_ops->init(); }
int      hal_timer_start(uint32_t sample_rate_hz)    { return s_ops->timer_start(sample_rate_hz); }
void     hal_timer_stop(void)                        { s_ops->timer_stop(); }
int      hal_adc_start_dma(uint16_t *b, uint32_t n)  { return s_ops->adc_start_dma(b, n); }
void     hal_adc_stop_dma(void)                      { s_ops->adc_stop_dma(); }
uint32_t hal_uart_write(const uint8_t *d, uint32_t n) { return s_ops->uart_write(d, n); }
int      hal_uart_tx_idle(void)                      { return s_ops->uart_tx_idle(); }
uint32_t hal_tick_ms(void)                           { return s_ops->tick_ms(); }
void     hal_gpio_write(hal_pin_t pin, int level)    { s_ops->gpio_write(pin, level); }
int      hal_gpio_read(hal_pin_t pin)                { return s_ops->gpio_read(pin); }
void     hal_delay_ms(uint32_t ms)                   { s_ops->delay_ms(ms); }

/* ------------------------------------------------------------------ */
/* assertion hook                                                      */
/* ------------------------------------------------------------------ */
void ecg_assert_failed(const char *file, int line)
{
    fprintf(stderr, "ECG ASSERT FAILED: %s:%d\n", (file != NULL) ? file : "?", line);
    abort();
}
