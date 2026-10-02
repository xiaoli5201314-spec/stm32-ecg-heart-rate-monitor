/*
 * ring_buffer.h - lock-free single-producer / single-consumer ring buffer.
 *
 * The DMA half/full interrupt is the only producer, the main loop is the only
 * consumer.  Neither side ever blocks or disables interrupts, which is what
 * keeps the 250 Hz sample clock strictly equidistant: the ISR only has to
 * memcpy one block into the ring and return.
 *
 * Memory-ordering: the head index is owned by the producer and published with
 * release semantics, the tail index is owned by the consumer and published the
 * same way.  On GCC/Clang the C99-legal __atomic builtins are used; on other
 * toolchains (Keil/armcc, IAR) the ECGRB_BARRIER() compiler barrier is the
 * fallback, which is sufficient on a single-core Cortex-M where the DMA
 * writes are already ordered by the AHB bus.
 */
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "ecg_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) && (__GNUC__ >= 4)
#define ECGRB_LOAD_ACQUIRE(p)   __atomic_load_n((p), __ATOMIC_ACQUIRE)
#define ECGRB_STORE_RELEASE(p,v) __atomic_store_n((p), (v), __ATOMIC_RELEASE)
#define ECGRB_BARRIER()         __atomic_thread_fence(__ATOMIC_SEQ_CST)
#else
#define ECGRB_LOAD_ACQUIRE(p)   (*(volatile uint32_t *)(p))
#define ECGRB_STORE_RELEASE(p,v) do { *(volatile uint32_t *)(p) = (uint32_t)(v); ECGRB_BARRIER(); } while (0)
#define ECGRB_BARRIER()         do { } while (0)   /* single core: compiler order is enough */
#endif

typedef struct {
    uint8_t         *storage;      /* element storage, capacity * elem_size bytes */
    uint32_t         capacity;     /* number of elements, MUST be a power of two   */
    uint32_t         mask;         /* capacity - 1                                 */
    uint32_t         elem_size;    /* bytes per element                            */
    volatile uint32_t head;        /* producer write index (monotonic, masked)     */
    volatile uint32_t tail;        /* consumer read index  (monotonic, masked)     */
    volatile uint32_t overflow;    /* elements dropped by the producer             */
    volatile uint32_t underflow;   /* read attempts on an empty buffer             */
    volatile uint32_t high_water;  /* deepest fill level ever observed             */
} ecg_ring_t;

/* Attach static storage. `capacity` must be a power of two.
 * Returns 0 on success, -1 on invalid arguments. */
int      ecg_ring_init(ecg_ring_t *rb, void *storage, uint32_t elem_size, uint32_t capacity);
void     ecg_ring_reset(ecg_ring_t *rb);

/* --- producer side (DMA ISR) --- */
uint32_t ecg_ring_write(ecg_ring_t *rb, const void *src, uint32_t count);
uint32_t ecg_ring_write_u16(ecg_ring_t *rb, const uint16_t *src, uint32_t count);
/* --- consumer side (main loop) --- */
uint32_t ecg_ring_read(ecg_ring_t *rb, void *dst, uint32_t count);
uint32_t ecg_ring_read_u16(ecg_ring_t *rb, uint16_t *dst, uint32_t count);

/* --- queries, safe from either side --- */
uint32_t ecg_ring_count(const ecg_ring_t *rb);      /* elements available to read   */
uint32_t ecg_ring_space(const ecg_ring_t *rb);      /* free slots for the producer  */
int      ecg_ring_is_empty(const ecg_ring_t *rb);
int      ecg_ring_is_full(const ecg_ring_t *rb);
uint32_t ecg_ring_capacity(const ecg_ring_t *rb);
uint32_t ecg_ring_high_water(const ecg_ring_t *rb);
uint32_t ecg_ring_overflow_count(const ecg_ring_t *rb);
uint32_t ecg_ring_underflow_count(const ecg_ring_t *rb);

/* Typed constructors for the two rings used by the firmware. */
#define ECG_RING_DECLARE(name, type, cap) \
    static type name##_storage[(cap)];    \
    static ecg_ring_t name

int ecg_ring_init_u16(ecg_ring_t *rb, uint16_t *storage, uint32_t capacity);
int ecg_ring_init_u8(ecg_ring_t *rb, uint8_t *storage, uint32_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
