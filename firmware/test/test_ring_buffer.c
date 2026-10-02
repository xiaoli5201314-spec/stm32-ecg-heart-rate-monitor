/*
 * test_ring_buffer.c - SPSC ring buffer: wrap-around, full/empty, overflow,
 * the DMA producer pattern and a real two-thread stress test.
 */
#include "test_util.h"
#include "ring_buffer.h"

#if defined(ECG_HAVE_PTHREAD)
#include <pthread.h>
#endif

static uint16_t s_storage[64];
static uint8_t  s_bytes[128];

static void test_init_validation(void)
{
    ecg_ring_t rb;

    tt_begin("ring_buffer::init");

    TT_CHECK(ecg_ring_init_u16(&rb, s_storage, 64u) == 0);
    TT_CHECK(ecg_ring_capacity(&rb) == 64u);
    TT_CHECK(ecg_ring_is_empty(&rb) == 1);
    TT_CHECK(ecg_ring_is_full(&rb) == 0);
    TT_CHECK(ecg_ring_count(&rb) == 0u);
    TT_CHECK(ecg_ring_space(&rb) == 64u);

    /* capacity must be a power of two */
    TT_CHECK(ecg_ring_init_u16(&rb, s_storage, 60u) == -1);
    /* degenerate arguments */
    TT_CHECK(ecg_ring_init_u16(NULL, s_storage, 64u) == -1);
    TT_CHECK(ecg_ring_init_u16(&rb, NULL, 64u) == -1);
    TT_CHECK(ecg_ring_init_u16(&rb, s_storage, 1u) == -1);

    tt_end();
}

static void test_fifo_round_trip(void)
{
    ecg_ring_t rb;
    uint16_t   in[16];
    uint16_t   out[16];
    uint32_t   i;

    tt_begin("ring_buffer::fifo");

    (void)ecg_ring_init_u16(&rb, s_storage, 64u);
    for (i = 0u; i < 16u; i++) {
        in[i] = (uint16_t)(1000u + i);
    }
    TT_CHECK(ecg_ring_write_u16(&rb, in, 16u) == 16u);
    TT_CHECK(ecg_ring_count(&rb) == 16u);
    TT_CHECK(ecg_ring_space(&rb) == 48u);

    memset(out, 0, sizeof(out));
    TT_CHECK(ecg_ring_read_u16(&rb, out, 16u) == 16u);
    for (i = 0u; i < 16u; i++) {
        TT_CHECK(out[i] == in[i]);
    }
    TT_CHECK(ecg_ring_is_empty(&rb) == 1);

    /* partial read */
    (void)ecg_ring_write_u16(&rb, in, 4u);
    TT_CHECK(ecg_ring_read_u16(&rb, out, 8u) == 4u);
    TT_CHECK(ecg_ring_underflow_count(&rb) == 1u);

    tt_end();
}

static void test_wrap_around(void)
{
    ecg_ring_t rb;
    uint16_t   out[70];
    uint16_t   tmp[10];
    uint32_t   i, total = 0u;

    tt_begin("ring_buffer::wrap");

    (void)ecg_ring_init_u16(&rb, s_storage, 64u);

    /* push 640 samples (10 x capacity) one burst at a time, drain each burst */
    for (i = 0u; i < 64u; i++) {
        uint16_t v = (uint16_t)i;
        TT_CHECK(ecg_ring_write_u16(&rb, &v, 1u) == 1u);
        if (ecg_ring_count(&rb) > 0u) {
            TT_CHECK(ecg_ring_read_u16(&rb, tmp, 1u) == 1u);
            TT_CHECK(tmp[0] == v);
            total++;
        }
    }
    TT_CHECK(total == 64u);

    /* fill exactly to capacity, then one more write must be refused */
    for (i = 0u; i < 64u; i++) {
        tmp[0] = (uint16_t)(i + 7u);
        TT_CHECK(ecg_ring_write_u16(&rb, tmp, 1u) == 1u);
    }
    TT_CHECK(ecg_ring_is_full(&rb) == 1);
    TT_CHECK(ecg_ring_space(&rb) == 0u);
    tmp[0] = 0xFFFFu;
    TT_CHECK(ecg_ring_write_u16(&rb, tmp, 1u) == 0u);
    TT_CHECK(ecg_ring_overflow_count(&rb) == 1u);

    TT_CHECK(ecg_ring_read_u16(&rb, out, 70u) == 64u);
    for (i = 0u; i < 64u; i++) {
        TT_CHECK(out[i] == (uint16_t)(i + 7u));
    }
    TT_CHECK(ecg_ring_high_water(&rb) == 64u);

    tt_end();
}

/* Emulates the DMA producer pattern at the wrong rate: 64-sample bursts are
 * pushed by the ISR while the main loop only manages 8 samples per tick, so
 * the ring must overflow, report it, and keep the stream gap-free wherever it
 * does deliver data. */
static void test_dma_producer_pattern(void)
{
    ecg_ring_t rb;
    uint16_t   block[64];
    uint16_t   buf[8];
    uint32_t   burst, i;
    uint32_t   drained = 0u;
    uint32_t   written = 0u;
    uint32_t   last = 0u;
    int        have_last = 0;
    int        non_monotonic = 0;

    tt_begin("ring_buffer::dma_producer");

    (void)ecg_ring_init_u16(&rb, s_storage, 64u);

    for (burst = 0u; burst < 100u; burst++) {
        for (i = 0u; i < 64u; i++) {
            block[i] = (uint16_t)((burst * 64u) + i);
        }
        written += ecg_ring_write_u16(&rb, block, 64u);

        /* one (slow) consumer tick per burst */
        {
            uint32_t got = ecg_ring_read_u16(&rb, buf, 8u);
            uint32_t k;
            for (k = 0u; k < got; k++) {
                if (have_last != 0 && buf[k] <= (uint16_t)last) {
                    non_monotonic++;
                }
                last = buf[k];
                have_last = 1;
            }
            drained += got;
        }
    }

    TT_CHECK(ecg_ring_overflow_count(&rb) > 0u);
    TT_CHECK(written + ecg_ring_overflow_count(&rb) == 100u * 64u);
    TT_CHECK(drained + ecg_ring_count(&rb) == written);
    TT_CHECK(non_monotonic == 0);
    TT_CHECK(ecg_ring_high_water(&rb) == 64u);
    tt_measure("ring_overflow_events_slow_consumer", (double)ecg_ring_overflow_count(&rb), "samples");
    tt_measure("ring_high_water_slow_consumer", (double)ecg_ring_high_water(&rb), "samples");
    tt_info("slow consumer: %u samples refused, %u delivered, high water %u/64",
            ecg_ring_overflow_count(&rb), drained, ecg_ring_high_water(&rb));

    tt_end();
}

static void test_byte_ring(void)
{
    ecg_ring_t rb;
    uint8_t    out[40];
    uint8_t    in[10] = { 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u };

    tt_begin("ring_buffer::bytes");

    TT_CHECK(ecg_ring_init_u8(&rb, s_bytes, 128u) == 0);
    TT_CHECK(ecg_ring_write(&rb, in, 10u) == 10u);
    TT_CHECK(ecg_ring_count(&rb) == 10u);
    TT_CHECK(ecg_ring_read(&rb, out, 10u) == 10u);
    TT_CHECK(memcmp(in, out, 10u) == 0);
    TT_CHECK(ecg_ring_read(&rb, out, 1u) == 0u);

    tt_end();
}

#if defined(ECG_HAVE_PTHREAD)
#define SPSC_ITEMS 400000u

static ecg_ring_t  s_spsc;
static uint16_t    s_spsc_storage[256];
static volatile int s_spsc_stop = 0;
static uint32_t    s_spsc_produced = 0u;
static uint32_t    s_spsc_consumed = 0u;
static int         s_spsc_mismatch = 0;

static void *spsc_producer(void *arg)
{
    uint32_t i = 0u;
    (void)arg;
    while (i < SPSC_ITEMS) {
        uint16_t v = (uint16_t)(i & 0xFFFFu);
        if (ecg_ring_write_u16(&s_spsc, &v, 1u) == 1u) {
            i++;
            s_spsc_produced++;
        }
    }
    s_spsc_stop = 1;
    return NULL;
}

static void *spsc_consumer(void *arg)
{
    uint16_t expected = 0u;
    (void)arg;
    while (s_spsc_stop == 0 || ecg_ring_count(&s_spsc) > 0u) {
        uint16_t v;
        if (ecg_ring_read_u16(&s_spsc, &v, 1u) == 1u) {
            if (v != expected) {
                s_spsc_mismatch++;
            }
            expected = (uint16_t)(expected + 1u);
            s_spsc_consumed++;
        }
    }
    return NULL;
}

static void test_spsc_threads(void)
{
    pthread_t prod, cons;
    int rc;

    tt_begin("ring_buffer::spsc_threads");

    s_spsc_stop = 0;
    s_spsc_produced = 0u;
    s_spsc_consumed = 0u;
    s_spsc_mismatch = 0;
    (void)ecg_ring_init_u16(&s_spsc, s_spsc_storage, 256u);

    rc  = pthread_create(&prod, NULL, spsc_producer, NULL);
    rc |= pthread_create(&cons, NULL, spsc_consumer, NULL);
    if (rc != 0) {
        tt_info("pthread_create failed, skipping the threaded test");
        tt_end();
        return;
    }
    (void)pthread_join(prod, NULL);
    (void)pthread_join(cons, NULL);

    TT_CHECK(s_spsc_mismatch == 0);
    TT_CHECK(s_spsc_consumed == SPSC_ITEMS);
    tt_measure("spsc_items_transferred", (double)s_spsc_consumed, "samples");
    tt_info("transfered %u samples between two threads with %d ordering errors",
            s_spsc_consumed, s_spsc_mismatch);

    tt_end();
}
#endif /* ECG_HAVE_PTHREAD */

void test_ring_buffer_all(void)
{
    test_init_validation();
    test_fifo_round_trip();
    test_wrap_around();
    test_dma_producer_pattern();
    test_byte_ring();
#if defined(ECG_HAVE_PTHREAD)
    test_spsc_threads();
#endif
}
