#pragma once
#include <stdbool.h>
#include <stdint.h>

void     radio_hal_init(void);
void     radio_hal_set_speed(uint32_t hz);
uint32_t radio_hal_get_speed(void);
bool     radio_hal_wait_busy_low(uint32_t timeout_us);
bool     radio_hal_irq_pin(void);
uint32_t radio_hal_busy_timeouts(void);

// One 6-byte transaction: ClearIrq(0xFFFFFFFF). MISO returns IrqStatus in bytes 2..5.
// Returns the IRQ flags that were pending (and are now cleared).
uint32_t radio_hal_read_clear_irq_fast(void);
