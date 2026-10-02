/*
 * ring_buffer.c - lock-free single-producer / single-consumer ring buffer.
 *
 * Only one writer (head) and one reader (tail) exist, so no critical section is
 * required: the producer publishes samples by storing head with release
 * ordering after the payload has been copied, the consumer reads head with
 * acquire ordering before touching the payload.
 */
#include "ring_buffer.h"

#include <string.h>

static uint8_t *rb_slot(const ecg_ring_t *rb, uint32_t index)
{
    return rb->storage + ((size_t)(index & rb->mask) * (size_t)rb->elem_size);
}

int ecg_ring_init(ecg_ring_t *rb, void *storage, uint32_t elem_size, uint32_t capacity)
{
    if (rb == NULL || storage == NULL || elem_size == 0u || capacity < 2u) {
        return -1;
    }
    /* power-of-two capacity lets the wrap-around be a mask instead of a
     * modulo, which matters on a Cortex-M3 without a hardware divider. */
    if ((capacity & (capacity - 1u)) != 0u) {
        return -1;
    }
    rb->storage   = (uint8_t *)storage;
    rb->capacity  = capacity;
    rb->mask      = capacity - 1u;
    rb->elem_size = elem_size;
    rb->head      = 0u;
    rb->tail      = 0u;
    rb->overflow  = 0u;
    rb->underflow = 0u;
    rb->high_water = 0u;
    return 0;
}

int ecg_ring_init_u16(ecg_ring_t *rb, uint16_t *storage, uint32_t capacity)
{
    return ecg_ring_init(rb, storage, (uint32_t)sizeof(uint16_t), capacity);
}

int ecg_ring_init_u8(ecg_ring_t *rb, uint8_t *storage, uint32_t capacity)
{
    return ecg_ring_init(rb, storage, (uint32_t)sizeof(uint8_t), capacity);
}

void ecg_ring_reset(ecg_ring_t *rb)
{
    if (rb == NULL) {
        return;
    }
    /* Only safe while both sides are idle (start-up / error recovery). */
    rb->tail = rb->head;
    rb->overflow = 0u;
    rb->underflow = 0u;
    rb->high_water = 0u;
}

uint32_t ecg_ring_count(const ecg_ring_t *rb)
{
    uint32_t head, tail;
    if (rb == NULL) {
        return 0u;
    }
    head = ECGRB_LOAD_ACQUIRE(&rb->head);
    tail = ECGRB_LOAD_ACQUIRE(&rb->tail);
    return head - tail;              /* unsigned arithmetic wraps correctly */
}

uint32_t ecg_ring_space(const ecg_ring_t *rb)
{
    if (rb == NULL) {
        return 0u;
    }
    return rb->capacity - ecg_ring_count(rb);
}

uint32_t ecg_ring_capacity(const ecg_ring_t *rb)
{
    return (rb == NULL) ? 0u : rb->capacity;
}

int ecg_ring_is_empty(const ecg_ring_t *rb)
{
    return (ecg_ring_count(rb) == 0u) ? 1 : 0;
}

int ecg_ring_is_full(const ecg_ring_t *rb)
{
    return (ecg_ring_count(rb) == rb->capacity) ? 1 : 0;
}

uint32_t ecg_ring_high_water(const ecg_ring_t *rb)
{
    return (rb == NULL) ? 0u : ECGRB_LOAD_ACQUIRE(&rb->high_water);
}

uint32_t ecg_ring_overflow_count(const ecg_ring_t *rb)
{
    return (rb == NULL) ? 0u : ECGRB_LOAD_ACQUIRE(&rb->overflow);
}

uint32_t ecg_ring_underflow_count(const ecg_ring_t *rb)
{
    return (rb == NULL) ? 0u : ECGRB_LOAD_ACQUIRE(&rb->underflow);
}

uint32_t ecg_ring_write(ecg_ring_t *rb, const void *src, uint32_t count)
{
    const uint8_t *in;
    uint32_t head, tail, used, space, n, i;

    if (rb == NULL || src == NULL || count == 0u) {
        return 0u;
    }

    in    = (const uint8_t *)src;
    head  = ECGRB_LOAD_ACQUIRE(&rb->head);   /* producer owns head */
    tail  = ECGRB_LOAD_ACQUIRE(&rb->tail);   /* published by consumer */
    used  = head - tail;
    space = rb->capacity - used;
    n     = (count > space) ? space : count;

    for (i = 0u; i < n; i++) {
        memcpy(rb_slot(rb, head + i), in + ((size_t)i * rb->elem_size), rb->elem_size);
    }

    if (n != 0u) {
        used += n;
        if (used > ECGRB_LOAD_ACQUIRE(&rb->high_water)) {
            ECGRB_STORE_RELEASE(&rb->high_water, used);
        }
        /* release: the payload above must be visible before the new head */
        ECGRB_STORE_RELEASE(&rb->head, head + n);
    }
    if (n < count) {
        ECGRB_STORE_RELEASE(&rb->overflow, ECGRB_LOAD_ACQUIRE(&rb->overflow) + (count - n));
    }
    return n;
}

uint32_t ecg_ring_write_u16(ecg_ring_t *rb, const uint16_t *src, uint32_t count)
{
    return ecg_ring_write(rb, src, count);
}

uint32_t ecg_ring_read(ecg_ring_t *rb, void *dst, uint32_t count)
{
    uint8_t *out;
    uint32_t head, tail, avail, n, i;

    if (rb == NULL || dst == NULL || count == 0u) {
        return 0u;
    }

    out   = (uint8_t *)dst;
    tail  = ECGRB_LOAD_ACQUIRE(&rb->tail);   /* consumer owns tail */
    head  = ECGRB_LOAD_ACQUIRE(&rb->head);   /* published by producer */
    avail = head - tail;
    n     = (count > avail) ? avail : count;

    for (i = 0u; i < n; i++) {
        memcpy(out + ((size_t)i * rb->elem_size), rb_slot(rb, tail + i), rb->elem_size);
    }

    if (n != 0u) {
        ECGRB_STORE_RELEASE(&rb->tail, tail + n);
    }
    if (n < count) {
        ECGRB_STORE_RELEASE(&rb->underflow, ECGRB_LOAD_ACQUIRE(&rb->underflow) + 1u);
    }
    return n;
}

uint32_t ecg_ring_read_u16(ecg_ring_t *rb, uint16_t *dst, uint32_t count)
{
    return ecg_ring_read(rb, dst, count);
}
