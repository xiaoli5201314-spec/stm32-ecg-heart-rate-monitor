/*
 * iir_notch.c - 50 Hz mains notch, cascaded transposed-DF-II biquads.
 *
 * ------------------------------------------------------------------
 * Coefficient derivation (this is the comment the design review asks for)
 * ------------------------------------------------------------------
 * Analogue prototype of a symmetrical notch centred on w0:
 *
 *      H(s) = (s^2 + w0^2) / (s^2 + (w0/Q) s + w0^2)
 *
 * |H(jw0)| = 0 (perfect null), |H(0)| = |H(inf)| = 1, and the -3 dB width of
 * the stop band is w0/Q rad/s.  The bilinear transform with frequency
 * pre-warping,
 *
 *      s = 2 fs (1 - z^-1) / (1 + z^-1),      w0 -> K = tan(pi f0 / fs)
 *
 * maps the analogue null at w0 exactly onto the digital null at
 * w0_d = 2 pi f0 / fs (pre-warping is what guarantees "exactly 50.00 Hz" and
 * not "50.6 Hz").  Substituting and normalising by the resulting a0 gives the
 * closed form used below:
 *
 *      alpha = sin(w0_d) / (2 Q)
 *      b0 =  1 / (1 + alpha)          a0 = 1
 *      b1 = -2 cos(w0_d) / (1 + alpha) a1 = -2 cos(w0_d) / (1 + alpha)
 *      b2 =  1 / (1 + alpha)          a2 = (1 - alpha) / (1 + alpha)
 *
 * Numbers for the operating point used by this project (fs = 250 Hz,
 * f0 = 50 Hz, Q = 8):
 *      w0_d  = 2 pi * 50 / 250 = 1.2566370614 rad
 *      cos   = 0.3090169944            sin = 0.9510565163
 *      alpha = 0.9510565163 / 16 = 0.0594410323
 *      1 + alpha = 1.0594410323
 *      b0 = b2 = 0.9438940, b1 = -0.5833586
 *      a1 = -0.5833586,     a2 = 0.8877879
 * The zeros sit exactly on the unit circle at +-w0_d (hence the ideal null),
 * the poles sit at radius sqrt(a2) = 0.9422241 and the same angle (hence the
 * finite 6.25 Hz -3 dB width).  Identical to MATLAB
 *      wo = 50/(250/2); bw = wo/8; [b,a] = iirnotch(wo,bw);
 *
 * Cascade: N identical sections multiply the stop-band depth by N and widen
 * the -3 dB notch by about sqrt(2) while each section is 0 dB at DC and at
 * Nyquist, so the diagnostic bandwidth of the ECG is not touched.
 */
#include "iir_notch.h"

/* ------------------------------------------------------------------ */
/* Generic biquad                                                      */
/* ------------------------------------------------------------------ */
void ecg_biquad_set(ecg_biquad_t *bq, double b0, double b1, double b2,
                    double a1, double a2, double fs_hz)
{
    if (bq == NULL) {
        return;
    }
    bq->b0 = (ecg_real_t)b0;
    bq->b1 = (ecg_real_t)b1;
    bq->b2 = (ecg_real_t)b2;
    bq->a1 = (ecg_real_t)a1;
    bq->a2 = (ecg_real_t)a2;
    bq->fs_hz = fs_hz;
    bq->f0_hz = 0.0;
    bq->q = 0.0;
    ecg_biquad_reset(bq);
}

void ecg_biquad_reset(ecg_biquad_t *bq)
{
    if (bq == NULL) {
        return;
    }
    bq->z1 = (ecg_real_t)0.0;
    bq->z2 = (ecg_real_t)0.0;
}

int ecg_biquad_notch_design(ecg_biquad_t *bq, double fs_hz, double f0_hz, double q)
{
    double w0, cos_w0, sin_w0, alpha, a0;

    if (bq == NULL) {
        return -1;
    }
    if (fs_hz <= 0.0 || f0_hz <= 0.0 || q <= 0.0) {
        return -1;
    }
    if (f0_hz >= (fs_hz * 0.5)) {           /* above Nyquist: nothing to do */
        return -1;
    }

    w0     = ECG_TWO_PI * f0_hz / fs_hz;    /* pre-warped, exact null */
    cos_w0 = cos(w0);
    sin_w0 = sin(w0);
    alpha  = sin_w0 / (2.0 * q);
    a0     = 1.0 + alpha;

    bq->b0    = (ecg_real_t)(1.0 / a0);
    bq->b1    = (ecg_real_t)(-2.0 * cos_w0 / a0);
    bq->b2    = (ecg_real_t)(1.0 / a0);
    bq->a1    = (ecg_real_t)(-2.0 * cos_w0 / a0);
    bq->a2    = (ecg_real_t)((1.0 - alpha) / a0);
    bq->fs_hz = fs_hz;
    bq->f0_hz = f0_hz;
    bq->q     = q;
    ecg_biquad_reset(bq);
    return 0;
}

ecg_real_t ecg_biquad_process(ecg_biquad_t *bq, ecg_real_t x)
{
    /* transposed direct form II:
     *   y  = b0*x + z1
     *   z1 = b1*x - a1*y + z2
     *   z2 = b2*x - a2*y                                        */
    ecg_real_t y = (bq->b0 * x) + bq->z1;
    bq->z1 = (bq->b1 * x) - (bq->a1 * y) + bq->z2;
    bq->z2 = (bq->b2 * x) - (bq->a2 * y);
    return y;
}

double ecg_biquad_response_db(const ecg_biquad_t *bq, double f_hz)
{
    double w, c1, s1, c2, s2, nr, ni, dr, di, num, den;

    if (bq == NULL || bq->fs_hz <= 0.0) {
        return 0.0;
    }
    w  = ECG_TWO_PI * f_hz / bq->fs_hz;
    c1 = cos(w);        s1 = sin(w);
    c2 = cos(2.0 * w);  s2 = sin(2.0 * w);

    nr = (double)bq->b0 + ((double)bq->b1 * c1) + ((double)bq->b2 * c2);
    ni = -(((double)bq->b1 * s1) + ((double)bq->b2 * s2));
    dr = 1.0 + ((double)bq->a1 * c1) + ((double)bq->a2 * c2);
    di = -(((double)bq->a1 * s1) + ((double)bq->a2 * s2));

    num = ECG_SQRT((nr * nr) + (ni * ni));
    den = ECG_SQRT((dr * dr) + (di * di));
    if (den <= 0.0) {
        return 0.0;
    }
    if (num <= 0.0) {
        return -300.0;                      /* exact null */
    }
    return 20.0 * log10(num / den);
}

/* ------------------------------------------------------------------ */
/* Notch cascade                                                       */
/* ------------------------------------------------------------------ */
int ecg_notch_init(ecg_notch_t *n, double fs_hz, double f0_hz, double q, uint32_t n_stages)
{
    uint32_t i;

    if (n == NULL) {
        return -1;
    }
    if (n_stages == 0u) {
        n_stages = 1u;
    }
    if (n_stages > ECG_NOTCH_MAX_STAGES) {
        n_stages = ECG_NOTCH_MAX_STAGES;
    }
    n->fs_hz    = fs_hz;
    n->f0_hz    = f0_hz;
    n->q        = q;
    n->n_stages = n_stages;

    for (i = 0u; i < n_stages; i++) {
        if (ecg_biquad_notch_design(&n->stage[i], fs_hz, f0_hz, q) != 0) {
            n->n_stages = 0u;
            return -1;
        }
    }
    return 0;
}

void ecg_notch_reset(ecg_notch_t *n)
{
    uint32_t i;
    if (n == NULL) {
        return;
    }
    for (i = 0u; i < n->n_stages; i++) {
        ecg_biquad_reset(&n->stage[i]);
    }
}

ecg_real_t ecg_notch_process(ecg_notch_t *n, ecg_real_t x)
{
    uint32_t i;
    ecg_real_t y = x;
    for (i = 0u; i < n->n_stages; i++) {
        y = ecg_biquad_process(&n->stage[i], y);
    }
    return y;
}

void ecg_notch_process_block(ecg_notch_t *n, const ecg_real_t *in, ecg_real_t *out, size_t count)
{
    size_t i;
    if (n == NULL || in == NULL || out == NULL) {
        return;
    }
    for (i = 0u; i < count; i++) {
        out[i] = ecg_notch_process(n, in[i]);
    }
}

double ecg_notch_response_db(const ecg_notch_t *n, double f_hz)
{
    double total = 0.0;
    uint32_t i;
    if (n == NULL) {
        return 0.0;
    }
    for (i = 0u; i < n->n_stages; i++) {
        total += ecg_biquad_response_db(&n->stage[i], f_hz);
    }
    return total;
}

double ecg_notch_stopband_edge_hz(const ecg_notch_t *n, double level_db)
{
    double f, step, limit, top;

    if (n == NULL || n->fs_hz <= 0.0) {
        return 0.0;
    }
    step  = 0.01;
    limit = n->fs_hz * 0.5;
    /* walk upwards from the null until the response comes back above level */
    for (f = n->f0_hz; f < limit; f += step) {
        if (ecg_notch_response_db(n, f) > level_db) {
            return f - n->f0_hz;
        }
    }
    top = limit;
    (void)top;
    return limit - n->f0_hz;
}

/* ------------------------------------------------------------------ */
/* Single-pole high-pass                                               */
/* ------------------------------------------------------------------ */
int ecg_hpf1_init(ecg_hpf1_t *hp, double fs_hz, double fc_hz)
{
    double rc;
    if (hp == NULL || fs_hz <= 0.0 || fc_hz <= 0.0) {
        return -1;
    }
    rc = 1.0 / (ECG_TWO_PI * fc_hz);            /* analogue time constant */
    hp->a     = (ecg_real_t)(rc / (rc + (1.0 / fs_hz)));
    hp->fs_hz = fs_hz;
    hp->fc_hz = fc_hz;
    ecg_hpf1_reset(hp);
    return 0;
}

void ecg_hpf1_reset(ecg_hpf1_t *hp)
{
    if (hp == NULL) {
        return;
    }
    hp->y      = (ecg_real_t)0.0;
    hp->x1     = (ecg_real_t)0.0;
    hp->primed = 0;
}

ecg_real_t ecg_hpf1_process(ecg_hpf1_t *hp, ecg_real_t x)
{
    ecg_real_t y;

    if (hp->primed == 0) {
        /* Prime on the very first sample: the mid-rail DC level is adopted as
         * x[n-1] instead of being treated as a step from zero. Without this the
         * 1.65 V level shift looks like a 2.65 mV input step and the resulting
         * transient dominates the QRS detector's learning phase. */
        hp->primed = 1;
        hp->x1     = x;
        hp->y      = (ecg_real_t)0.0;
        return (ecg_real_t)0.0;
    }
    y = hp->a * (hp->y + x - hp->x1);
    hp->x1 = x;
    hp->y  = y;
    return y;
}

double ecg_hpf1_response_db(const ecg_hpf1_t *hp, double f_hz)
{
    double a, w, nr, ni, dr, di, num, den;
    if (hp == NULL || hp->fs_hz <= 0.0) {
        return 0.0;
    }
    a  = (double)hp->a;
    w  = ECG_TWO_PI * f_hz / hp->fs_hz;
    nr = a * (1.0 - cos(w));
    ni = a * sin(w);
    dr = 1.0 - (a * cos(w));
    di = a * sin(w);
    num = ECG_SQRT((nr * nr) + (ni * ni));
    den = ECG_SQRT((dr * dr) + (di * di));
    if (den <= 0.0 || num <= 0.0) {
        return -300.0;
    }
    return 20.0 * log10(num / den);
}
