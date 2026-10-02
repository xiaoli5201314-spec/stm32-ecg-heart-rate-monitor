/*
 * hal.h - hardware abstraction for the ECG node.
 *
 * The signal-processing core never includes a vendor header.  Everything the
 * core needs from the silicon is expressed as one small operation table which
 * is filled in either by hal/hal_stm32.c (real target) or by
 * hal/hal_stub.c (PC simulation used by the unit tests).
 *
 * Two decoupling mechanisms are used on purpose:
 *
 *   1. Function-pointer table  -> runtime selection, lets the test suite swap
 *      a recording mock in and inspect every call.
 *   2. Weak default symbols    -> a port that only wants to replace one
 *      operation can override exactly that symbol instead of filling the
 *      whole table (see hal/hal_stub.c, "weak fallbacks").
 */
#ifndef HAL_H
#define HAL_H

#include "ecg_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Weak linkage: every default operation in hal/hal_stub.c is weak, so a port
 * can replace a single operation by defining a strong symbol with the same
 * name, or replace all of them by installing its own hal_ops_t table. */
#if defined(__GNUC__)
#define HAL_WEAK __attribute__((weak))
#else
#define HAL_WEAK
#endif

/* Board LEDs / control lines exposed to the firmware. */
typedef enum {
    HAL_PIN_STATUS_LED = 0,
    HAL_PIN_LEAD_OFF_LED,
    HAL_PIN_AFE_ENABLE,
    HAL_PIN_RLD_ENABLE,
    HAL_PIN_COUNT
} hal_pin_t;

typedef struct {
    const char *name;

    /* Bring up clocks, GPIO, ADC, DMA, USART. 0 = ok. */
    int      (*init)(void);
    /* Start the sample-rate timer; its update event triggers the ADC. */
    int      (*timer_start)(uint32_t sample_rate_hz);
    void     (*timer_stop)(void);
    /* Arm the ADC in circular mode on the given buffer (2 x block). */
    int      (*adc_start_dma)(uint16_t *buf, uint32_t total_len);
    void     (*adc_stop_dma)(void);
    /* Queue a byte stream on the uplink UART. Contract: all-or-nothing, i.e.
     * the driver accepts the whole buffer into its own TX ring and returns
     * `len`, or returns 0 when that ring is full. A short count is a genuine
     * overrun and is accounted as dropped bytes by the caller. */
    uint32_t (*uart_write)(const uint8_t *data, uint32_t len);
    int      (*uart_tx_idle)(void);
    /* Free-running millisecond counter. */
    uint32_t (*tick_ms)(void);
    void     (*gpio_write)(hal_pin_t pin, int level);
    int      (*gpio_read)(hal_pin_t pin);
    void     (*delay_ms)(uint32_t ms);
} hal_ops_t;

/* Called from the DMA interrupt context. */
typedef void (*hal_adc_callback_t)(const uint16_t *samples, uint32_t count);

/* Install an operation table (NULL restores the weak defaults). */
void             hal_register_ops(const hal_ops_t *ops);
const hal_ops_t *hal_ops(void);

/* Register the two DMA completion callbacks: first half and second half. */
void hal_set_adc_callbacks(hal_adc_callback_t on_half, hal_adc_callback_t on_full);
void hal_invoke_adc_half(const uint16_t *samples, uint32_t count);
void hal_invoke_adc_full(const uint16_t *samples, uint32_t count);

/* -------- thin convenience wrappers used by the application -------- */
int      hal_init(void);
int      hal_timer_start(uint32_t sample_rate_hz);
void     hal_timer_stop(void);
int      hal_adc_start_dma(uint16_t *buf, uint32_t total_len);
void     hal_adc_stop_dma(void);
uint32_t hal_uart_write(const uint8_t *data, uint32_t len);
int      hal_uart_tx_idle(void);
uint32_t hal_tick_ms(void);
void     hal_gpio_write(hal_pin_t pin, int level);
int      hal_gpio_read(hal_pin_t pin);
void     hal_delay_ms(uint32_t ms);

/* -------- weak default implementations live in hal/hal_stub.c -------- */
HAL_WEAK int      hal_weak_init(void);
HAL_WEAK int      hal_weak_timer_start(uint32_t sample_rate_hz);
HAL_WEAK void     hal_weak_timer_stop(void);
HAL_WEAK int      hal_weak_adc_start_dma(uint16_t *buf, uint32_t total_len);
HAL_WEAK void     hal_weak_adc_stop_dma(void);
HAL_WEAK uint32_t hal_weak_uart_write(const uint8_t *data, uint32_t len);
HAL_WEAK int      hal_weak_uart_tx_idle(void);
HAL_WEAK uint32_t hal_weak_tick_ms(void);
HAL_WEAK void     hal_weak_gpio_write(hal_pin_t pin, int level);
HAL_WEAK int      hal_weak_gpio_read(hal_pin_t pin);
HAL_WEAK void     hal_weak_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* HAL_H */
