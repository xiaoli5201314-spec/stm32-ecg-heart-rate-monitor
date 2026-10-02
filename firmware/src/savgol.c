/*
 * savgol.c - Savitzky-Golay least-squares smoothing.
 *
 * ------------------------------------------------------------------
 * Derivation of the convolution kernel (nothing here is a lookup table)
 * ------------------------------------------------------------------
 * Sample the signal inside a window of length L = 2m+1 centred on the output
 * sample and fit a polynomial of order p <= L-1 by least squares:
 *
 *      p(x) = c0 + c1 x + ... + cp x^p ,      x_i = (i - m) / m  in [-1, +1]
 *
 *      min_c  sum_{i=0..L-1} ( y_i - p(x_i) )^2
 *
 * In matrix form with the Vandermonde matrix A (A[i][k] = x_i^k):
 *
 *      A c = y   (over-determined)  ->   c = (A^T A)^-1 A^T y
 *
 * The smoothed output is the polynomial evaluated at the output position:
 *
 *      y_out = p(x_pos) = e_pos^T c = e_pos^T (A^T A)^-1 A^T y = h^T y
 *      with  h = A (A^T A)^-1 e_pos,   e_pos = [1, x_pos, ..., x_pos^p]^T
 *
 * so the kernel h is obtained by solving one (p+1) x (p+1) symmetric system
 * (A^T A) u = e_pos  and then h_i = sum_k A[i][k] u_k.
 *
 * Two numerical details matter and are implemented below:
 *   - the abscissa is normalised by m, which reduces cond(A^T A) by ~m^(2p)
 *     and makes the solver reliable up to order 6 / window 41 in double;
 *   - the kernel is re-normalised so that sum(h) == 1 (exact in real
 *     arithmetic because the fit reproduces constants, enforced in floating
 *     point to keep the DC gain at exactly 1.000).
 *
 * p(x_pos) with x_pos = 0 gives the centre kernel used in real time; the same
 * routine with x_pos at the edge gives the edge-aware kernels used by
 * ecg_savgol_apply(), which is why the first and last (L-1)/2 samples are
 * filtered instead of dropped.
 */
#include "savgol.h"

#include <string.h>

#define SG_MAX_P1 (ECG_SG_MAX_ORDER + 1u)

/* Gauss-Jordan elimination with partial pivoting, in place.
 * `mat` is n*n row major, `rhs` has n entries. Returns 0 on success. */
static int sg_solve(double *mat, double *rhs, int n)
{
    int i, j, k, pivot;
    double tmp, factor, diag;

    for (i = 0; i < n; i++) {
        pivot = i;
        for (j = i + 1; j < n; j++) {
            if (ECG_FABS(mat[(j * n) + i]) > ECG_FABS(mat[(pivot * n) + i])) {
                pivot = j;
            }
        }
        if (ECG_FABS(mat[(pivot * n) + i]) < 1e-300) {
            return -1;                                  /* singular Gram matrix */
        }
        if (pivot != i) {
            for (k = 0; k < n; k++) {
                tmp = mat[(i * n) + k];
                mat[(i * n) + k] = mat[(pivot * n) + k];
                mat[(pivot * n) + k] = tmp;
            }
            tmp = rhs[i];
            rhs[i] = rhs[pivot];
            rhs[pivot] = tmp;
        }
        diag = mat[(i * n) + i];
        for (k = i; k < n; k++) {
            mat[(i * n) + k] /= diag;
        }
        rhs[i] /= diag;

        for (j = 0; j < n; j++) {
            if (j == i) {
                continue;
            }
            factor = mat[(j * n) + i];
            if (factor == 0.0) {
                continue;
            }
            for (k = i; k < n; k++) {
                mat[(j * n) + k] -= factor * mat[(i * n) + k];
            }
            rhs[j] -= factor * rhs[i];
        }
    }
    return 0;
}

int ecg_savgol_design_at(double *coef, uint32_t order, uint32_t window, uint32_t pos)
{
    double vander[ECG_SG_MAX_WINDOW][SG_MAX_P1];
    double gram[SG_MAX_P1 * SG_MAX_P1];
    double rhs[SG_MAX_P1];
    double u[SG_MAX_P1];
    double x_row[ECG_SG_MAX_WINDOW];
    double x, xp, sum;
    uint32_t m, i, k, l, p1;

    if (coef == NULL) {
        return -1;
    }
    if (window == 0u || (window & 1u) == 0u || window > ECG_SG_MAX_WINDOW) {
        return -1;                                      /* window must be odd */
    }
    if (order > ECG_SG_MAX_ORDER || order >= window) {
        return -1;
    }
    if (pos >= window) {
        return -1;
    }

    m  = (window - 1u) / 2u;
    p1 = order + 1u;

    /* --- Vandermonde rows, abscissa normalised to [-1, 1] --- */
    for (i = 0u; i < window; i++) {
        if (m == 0u) {
            x = 0.0;
        } else {
            x = ((double)i - (double)m) / (double)m;
        }
        x_row[i] = x;
        xp = 1.0;
        for (k = 0u; k < p1; k++) {
            vander[i][k] = xp;
            xp *= x;
        }
    }

    /* --- Gram matrix  G[k][l] = sum_i x_i^(k+l) --- */
    for (k = 0u; k < p1; k++) {
        for (l = 0u; l < p1; l++) {
            sum = 0.0;
            for (i = 0u; i < window; i++) {
                sum += vander[i][k] * vander[i][l];
            }
            gram[(k * p1) + l] = sum;
        }
    }

    /* --- right hand side: e_pos --- */
    xp = 1.0;
    for (k = 0u; k < p1; k++) {
        rhs[k] = xp;
        xp *= x_row[pos];
    }

    memcpy(u, rhs, sizeof(double) * p1);
    if (sg_solve(gram, u, (int)p1) != 0) {
        return -1;
    }

    /* --- h_i = sum_k A[i][k] * u_k --- */
    sum = 0.0;
    for (i = 0u; i < window; i++) {
        double acc = 0.0;
        for (k = 0u; k < p1; k++) {
            acc += vander[i][k] * u[k];
        }
        coef[i] = acc;
        sum += acc;
    }

    /* enforce unity DC gain exactly */
    if (ECG_FABS(sum) > 1e-12) {
        for (i = 0u; i < window; i++) {
            coef[i] /= sum;
        }
    }
    return 0;
}

int ecg_savgol_design(ecg_savgol_t *sg, uint32_t order, uint32_t window)
{
    uint32_t m;
    if (sg == NULL) {
        return -1;
    }
    m = (window - 1u) / 2u;
    if (ecg_savgol_design_at(sg->coef, order, window, m) != 0) {
        return -1;
    }
    sg->order  = order;
    sg->window = window;
    sg->half   = m;
    return 0;
}

void ecg_savgol_apply(const ecg_savgol_t *sg, const ecg_real_t *in, ecg_real_t *out, size_t n)
{
    size_t i;
    uint32_t j, len, start, pos;
    uint32_t m;
    double local[ECG_SG_MAX_WINDOW];
    const double *h;
    double acc;

    if (sg == NULL || in == NULL || out == NULL || n == 0u) {
        return;
    }
    m = sg->half;

    for (i = 0u; i < n; i++) {
        /* choose the window: full in the middle, shrunk and re-centred at the
         * two edges (the classic sgolayfilt convention)                       */
        if (i < (size_t)m) {
            len   = (uint32_t)(2u * (uint32_t)i) + 1u;
            start = 0u;
            pos   = (uint32_t)i;
        } else if (i + (size_t)m >= n) {
            len   = (uint32_t)(2u * (uint32_t)(n - 1u - i)) + 1u;
            start = (uint32_t)(n - (size_t)len);
            pos   = len / 2u;
        } else {
            len   = sg->window;
            start = (uint32_t)i - m;
            pos   = m;
        }

        if (len > sg->window) {                 /* signal shorter than kernel */
            len   = sg->window;
            start = 0u;
            pos   = (start + m < n) ? m : 0u;
        }
        if (len == sg->window && pos == m) {
            h = sg->coef;
        } else if (len <= sg->order) {
            /* not enough points for the requested order: fall back to a
             * straight copy of the original sample */
            out[i] = in[i];
            continue;
        } else {
            if (ecg_savgol_design_at(local, sg->order, len, pos) != 0) {
                out[i] = in[i];
                continue;
            }
            h = local;
        }

        acc = 0.0;
        for (j = 0u; j < len; j++) {
            acc += h[j] * (double)in[start + j];
        }
        out[i] = (ecg_real_t)acc;
    }
}

/* ------------------------------------------------------------------ */
/* Analysis helpers                                                    */
/* ------------------------------------------------------------------ */
double ecg_savgol_noise_gain(const ecg_savgol_t *sg)
{
    double s = 0.0;
    uint32_t i;
    if (sg == NULL) {
        return 1.0;
    }
    for (i = 0u; i < sg->window; i++) {
        s += sg->coef[i] * sg->coef[i];
    }
    return ECG_SQRT(s);        /* white noise std multiplier (power gain = s) */
}

double ecg_savgol_response_db(const ecg_savgol_t *sg, double fs_hz, double f_hz)
{
    double w, re, im, mag;
    uint32_t j;
    if (sg == NULL || fs_hz <= 0.0) {
        return 0.0;
    }
    w  = ECG_TWO_PI * f_hz / fs_hz;
    re = 0.0;
    im = 0.0;
    for (j = 0u; j < sg->window; j++) {
        double d = (double)j - (double)sg->half;      /* lag in samples */
        re += sg->coef[j] * cos(w * d);
        im -= sg->coef[j] * sin(w * d);
    }
    mag = ECG_SQRT((re * re) + (im * im));
    if (mag <= 0.0) {
        return -300.0;
    }
    return 20.0 * log10(mag);
}

double ecg_savgol_cutoff_hz(const ecg_savgol_t *sg, double fs_hz)
{
    double f, step, limit;
    if (sg == NULL || fs_hz <= 0.0) {
        return 0.0;
    }
    step  = 0.05;
    limit = fs_hz * 0.5;
    for (f = step; f < limit; f += step) {
        if (ecg_savgol_response_db(sg, fs_hz, f) <= -3.0103) {
            return f;
        }
    }
    return limit;
}

double ecg_savgol_group_delay_s(const ecg_savgol_t *sg, double fs_hz)
{
    if (sg == NULL || fs_hz <= 0.0) {
        return 0.0;
    }
    return (double)sg->half / fs_hz;        /* linear phase FIR: exactly m samples */
}

double ecg_savgol_gaussian_peak_retention(const ecg_savgol_t *sg, double fs_hz, double sigma_s)
{
    double dt, t, acc;
    uint32_t j;
    if (sg == NULL || fs_hz <= 0.0 || sigma_s <= 0.0) {
        return 0.0;
    }
    dt  = 1.0 / fs_hz;
    acc = 0.0;
    /* the pulse is centred on the output sample: the SG output at the peak is
     * simply the kernel weighted by the pulse shape                          */
    for (j = 0u; j < sg->window; j++) {
        t = ((double)j - (double)sg->half) * dt;
        acc += sg->coef[j] * exp(-(t * t) / (2.0 * sigma_s * sigma_s));
    }
    return acc;                              /* 1.0 = peak fully preserved */
}

/* ------------------------------------------------------------------ */
/* Streaming                                                           */
/* ------------------------------------------------------------------ */
void ecg_savgol_stream_init(ecg_savgol_stream_t *s, const ecg_savgol_t *sg)
{
    if (s == NULL || sg == NULL) {
        return;
    }
    s->kernel = *sg;
    ecg_savgol_stream_reset(s);
}

void ecg_savgol_stream_reset(ecg_savgol_stream_t *s)
{
    uint32_t i;
    if (s == NULL) {
        return;
    }
    for (i = 0u; i < ECG_SG_MAX_COEF; i++) {
        s->delay[i] = (ecg_real_t)0.0;
    }
    s->index  = 0u;
    s->filled = 0u;
    s->primed = 0u;
}

ecg_real_t ecg_savgol_stream_push(ecg_savgol_stream_t *s, ecg_real_t x, int *primed)
{
    uint32_t j, w;
    double acc = 0.0;

    if (s == NULL || s->kernel.window == 0u) {
        if (primed != NULL) {
            *primed = 0;
        }
        return x;
    }
    w = s->kernel.window;

    s->delay[s->index] = x;
    s->index = (s->index + 1u) % w;      /* index now addresses the oldest sample */
    if (s->filled < w) {
        s->filled++;
    }

    if (s->filled < w) {
        /* start-up: the window is not complete yet, pass the sample through.
         * This lasts (window-1) samples = 40 ms for window 11 at 250 Hz and is
         * irrelevant for the QRS detector, which needs a 2 s warm-up anyway. */
        if (primed != NULL) {
            *primed = 0;
        }
        return x;
    }

    for (j = 0u; j < w; j++) {
        acc += s->kernel.coef[j] * (double)s->delay[(s->index + j) % w];
    }
    s->primed = 1u;
    if (primed != NULL) {
        *primed = 1;
    }
    return (ecg_real_t)acc;
}

void ecg_savgol_stream_push_block(ecg_savgol_stream_t *s, const ecg_real_t *in,
                                  ecg_real_t *out, size_t n)
{
    size_t i;
    if (s == NULL || in == NULL || out == NULL) {
        return;
    }
    for (i = 0u; i < n; i++) {
        out[i] = ecg_savgol_stream_push(s, in[i], NULL);
    }
}
