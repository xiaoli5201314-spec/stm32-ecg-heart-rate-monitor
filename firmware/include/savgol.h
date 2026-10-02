/*
 * savgol.h - Savitzky-Golay least-squares smoothing filter.
 *
 * The convolution kernel is NOT a hard-coded table: ecg_savgol_design()
 * derives it from (polynomial order, window length) at run time by solving the
 * normal equations of the least-squares polynomial fit
 *
 *      min_c  sum_{i=-m..m} ( y[i] - sum_{k=0..p} c_k * i^k )^2
 *
 * The smoothed value at the window centre is c_0, therefore
 *
 *      c = (A^T A)^-1 A^T y        (A = Vandermonde matrix, A[i][k] = i^k)
 *      y_smooth[0] = c_0 = h^T y   with  h = A (A^T A)^-1 e_0
 *
 * i is scaled by 1/m before building A, which reduces the condition number of
 * the Gram matrix by orders of magnitude and keeps the solver stable up to
 * order 6 / window 41.  A Gauss-Jordan elimination with partial pivoting
 * solves the (p+1)x(p+1) system in double precision.  For p == 0 the kernel is
 * the plain moving average (a useful special case to cross-check against).
 *
 * The edge-aware variant ecg_savgol_design_at() evaluates the same polynomial
 * at sample `pos` instead of the centre, so the first/last (window-1)/2 output
 * samples are extrapolated by the local fit rather than being dropped or
 * zero-padded - that is what keeps the R peak at the very start of a record
 * from being clipped.
 */
#ifndef SAVGOL_H
#define SAVGOL_H

#include "ecg_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ECG_SG_MAX_COEF (ECG_SG_MAX_WINDOW)

typedef struct {
    uint32_t order;                     /* polynomial order p                 */
    uint32_t window;                    /* odd window length 2m+1             */
    uint32_t half;                      /* m                                  */
    double   coef[ECG_SG_MAX_COEF];     /* centre kernel, sum(coef) == 1      */
} ecg_savgol_t;

/* Streaming (block-boundary-free) realisation. */
typedef struct {
    ecg_savgol_t kernel;
    ecg_real_t   delay[ECG_SG_MAX_COEF]; /* circular history, `window` entries */
    uint32_t     index;                  /* next write slot                    */
    uint32_t     filled;                 /* samples pushed so far              */
    uint32_t     primed;                 /* 1 once the window is complete      */
} ecg_savgol_stream_t;

/* Derive the centre kernel. Returns 0 on success, -1 if
 * (window is even) || (window > ECG_SG_MAX_WINDOW) || (order >= window). */
int  ecg_savgol_design(ecg_savgol_t *sg, uint32_t order, uint32_t window);

/* Derive the kernel that evaluates the fitted polynomial at output position
 * `pos` (0 .. window-1) of a window of length `window`. Used for the edges. */
int  ecg_savgol_design_at(double *coef, uint32_t order, uint32_t window, uint32_t pos);

/* Apply to a whole buffer with edge-aware coefficients at both ends. */
void ecg_savgol_apply(const ecg_savgol_t *sg, const ecg_real_t *in, ecg_real_t *out, size_t n);

/* --- analysis helpers (used by the parameter sweep in the test suite) --- */
double ecg_savgol_noise_gain(const ecg_savgol_t *sg);                 /* sqrt(sum h^2)   */
double ecg_savgol_response_db(const ecg_savgol_t *sg, double fs_hz, double f_hz);
double ecg_savgol_cutoff_hz(const ecg_savgol_t *sg, double fs_hz);    /* -3 dB point     */
double ecg_savgol_group_delay_s(const ecg_savgol_t *sg, double fs_hz);
/* Peak attenuation predicted for a Gaussian pulse of the given sigma: the SG
 * kernel acts on the pulse as a discrete convolution, so the response at DC
 * weighted by the pulse spectrum gives the peak retention. */
double ecg_savgol_gaussian_peak_retention(const ecg_savgol_t *sg, double fs_hz, double sigma_s);

/* --- streaming API --- */
void       ecg_savgol_stream_init(ecg_savgol_stream_t *s, const ecg_savgol_t *sg);
void       ecg_savgol_stream_reset(ecg_savgol_stream_t *s);
/* Push one sample. *primed is set to 1 once the first full window has been
 * seen; until then the function returns the input unchanged (documented
 * start-up pass-through of (window-1) samples = 40 ms at 250 Hz). */
ecg_real_t ecg_savgol_stream_push(ecg_savgol_stream_t *s, ecg_real_t x, int *primed);
/* Push a block; identical to pushing sample by sample. */
void       ecg_savgol_stream_push_block(ecg_savgol_stream_t *s, const ecg_real_t *in,
                                        ecg_real_t *out, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* SAVGOL_H */
