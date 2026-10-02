/*
 * test_util.h - tiny assertion / measurement harness shared by the suites.
 *
 * The implementation lives in run_tests.c so that the firmware test directory
 * keeps exactly the files listed in the project layout.  Every suite reports
 * through these helpers, which is what lets run_tests.c print one consolidated
 * table of measured numbers at the end.
 */
#ifndef ECG_TEST_UTIL_H
#define ECG_TEST_UTIL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

void tt_begin(const char *suite);
void tt_end(void);
int  tt_suite_failed(void);
void tt_check_impl(int ok, const char *expr, const char *file, int line);
void tt_near_impl(double a, double b, double tol, const char *ea, const char *eb,
                  const char *file, int line);
void tt_measure(const char *key, double value, const char *unit);
void tt_info(const char *fmt, ...);
void tt_measure_reset(void);

#define TT_CHECK(expr) tt_check_impl(((expr) != 0) ? 1 : 0, #expr, __FILE__, __LINE__)

#define TT_CHECK_MSG(expr, ...)                              \
    do {                                                     \
        int tt_ok_ = ((expr) != 0) ? 1 : 0;                  \
        tt_check_impl(tt_ok_, #expr, __FILE__, __LINE__);    \
        if (tt_ok_ == 0) { tt_info(__VA_ARGS__); }           \
    } while (0)

#define TT_NEAR(a, b, tol) tt_near_impl((double)(a), (double)(b), (double)(tol), \
                                        #a, #b, __FILE__, __LINE__)

#define TT_TRUE(expr)  TT_CHECK(expr)
#define TT_FALSE(expr) TT_CHECK(!(expr))

/* Suite entry points. */
void test_ring_buffer_all(void);
void test_filters_all(void);
void test_protocol_all(void);
void test_heart_rate_all(void);
void test_pipeline_integration(void);

#ifdef __cplusplus
}
#endif

#endif /* ECG_TEST_UTIL_H */
