/*
 * iir_notch.h - 50 Hz mains notch built from cascaded second-order sections.
 *
 * Design route (identical to the MATLAB fdatool / iirnotch flow used for the
 * parameter study, see docs/DESIGN.md section 4 for the full derivation):
 *
 *   analog prototype   H(s) = (s^2 + w0^2) / (s^2 + (w0/Q)*s + w0^2)
 *   bilinear transform with prewarping  s = (2/T) * (1 - z^-1)/(1 + z^-1)
 *   w0 <- 2*pi*f0,  K = tan(pi*f0/fs),  alpha = sin(w0_d)/(2Q)
 *
 *   b0 = (1 + alpha*Q ... )  ->  normalised to the closed form
 *   b = [ 1, -2*cos(w0_d),  1 ] / (1 + alpha)
 *   a = [ 1, -2*cos(w0_d)/(1+alpha), (1 - alpha)/(1 + alpha) ]
 *
 * Per section the -3 dB stop band is f0/Q wide; cascading N identical sections
 * multiplies the stop-band depth by N and widens the notch by ~sqrt(2) while
 * leaving the pass band essentially untouched (each section is 0 dB at DC and
 * at Nyquist).
 *
 * The filter is realised in transposed direct form II, which needs only two
 * state variables and has the best rounding behaviour of the direct forms for
 * fixed/float arithmetic.
 */
#ifndef IIR_NOTCH_H
#define IIR_NOTCH_H

#include "ecg_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ecg_real_t b0, b1, b2;   /* numerator,   b0 normalised to 1 */
    ecg_real_t a1, a2;       /* denominator, a0 == 1            */
    ecg_real_t z1, z2;       /* transposed DF-II state          */
    double     fs_hz;
    double     f0_hz;
    double     q;
} ecg_biquad_t;

typedef struct {
    ecg_biquad_t stage[ECG_NOTCH_MAX_STAGES];
    uint32_t     n_stages;
    double       fs_hz;
    double       f0_hz;
    double       q;
} ecg_notch_t;

/* Solve the notch coefficients for one biquad. Returns 0 on success. */
int  ecg_biquad_notch_design(ecg_biquad_t *bq, double fs_hz, double f0_hz, double q);
/* Generic helper: arbitrary biquad from normalised coefficients (b0..a2). */
void ecg_biquad_set(ecg_biquad_t *bq, double b0, double b1, double b2,
                    double a1, double a2, double fs_hz);
void ecg_biquad_reset(ecg_biquad_t *bq);
ecg_real_t ecg_biquad_process(ecg_biquad_t *bq, ecg_real_t x);
double ecg_biquad_response_db(const ecg_biquad_t *bq, double f_hz);

/* n_stages is clamped to [1, ECG_NOTCH_MAX_STAGES]. */
int  ecg_notch_init(ecg_notch_t *n, double fs_hz, double f0_hz, double q, uint32_t n_stages);
void ecg_notch_reset(ecg_notch_t *n);
ecg_real_t ecg_notch_process(ecg_notch_t *n, ecg_real_t x);
void ecg_notch_process_block(ecg_notch_t *n, const ecg_real_t *in, ecg_real_t *out, size_t count);
/* Magnitude response of the whole cascade, in dB. */
double ecg_notch_response_db(const ecg_notch_t *n, double f_hz);
/* Frequency (Hz) at which the cascade response first drops to `level_db`. */
double ecg_notch_stopband_edge_hz(const ecg_notch_t *n, double level_db);

/* Single-pole high-pass, used to strip the residual DC / respiration wander
 * that survives the analog channel. y[n] = a*(y[n-1] + x[n] - x[n-1]).
 *
 * The filter primes itself on its very first sample (x[n-1] := x[0], y := 0)
 * instead of starting from zero state. Without that, the 1.65 V mid-rail level
 * shift looks like a 2.6 mV step at the electrode and leaks a large transient
 * into the QRS detector's learning phase. */
typedef struct {
    ecg_real_t a;
    ecg_real_t y;
    ecg_real_t x1;
    int        primed;
    double     fs_hz;
    double     fc_hz;
} ecg_hpf1_t;

int        ecg_hpf1_init(ecg_hpf1_t *hp, double fs_hz, double fc_hz);
void       ecg_hpf1_reset(ecg_hpf1_t *hp);
ecg_real_t ecg_hpf1_process(ecg_hpf1_t *hp, ecg_real_t x);
double     ecg_hpf1_response_db(const ecg_hpf1_t *hp, double f_hz);

#ifdef __cplusplus
}
#endif

#endif /* IIR_NOTCH_H */
