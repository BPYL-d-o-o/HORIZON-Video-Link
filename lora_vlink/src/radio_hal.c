// Fast lr20xx HAL for RP2350.
// - NSS is a plain GPIO held low across opcode + data (one transaction per command).
// - BUSY is polled with a tight gpio_get loop (BUSY rises on NSS falling edge, so no race).
#include "radio_hal.h"

#include <string.h>

#include "board.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "lr20xx_hal.h"
#include "pico/stdlib.h"

static uint32_t s_spi_hz;
static uint32_t s_busy_timeouts;

#define BUSY_SPIN_LIMIT 30000000u  // ~ >1 s of spinning: "never" on the hot path

static inline void nss_low(void) { gpio_put(PIN_RADIO_NSS, 0); }
static inline void nss_high(void) { gpio_put(PIN_RADIO_NSS, 1); }

static inline void wait_busy(void) {
    uint32_t n = 0;
    while (gpio_get(PIN_RADIO_BUSY)) {
        if (++n > BUSY_SPIN_LIMIT) {
            s_busy_timeouts++;
            return;
        }
    }
}

void radio_hal_init(void) {
#ifdef PIN_RADIO_CE
    // Module power (its LDO enable). Off -> on at every init, so a re-init also power-cycles a stuck module.
    gpio_init(PIN_RADIO_CE);
    gpio_set_dir(PIN_RADIO_CE, GPIO_OUT);
    gpio_put(PIN_RADIO_CE, 0);
    sleep_ms(5);
    gpio_put(PIN_RADIO_CE, 1);
    sleep_ms(5);
#endif
    gpio_init(PIN_RADIO_NSS);
    gpio_put(PIN_RADIO_NSS, 1);
    gpio_set_dir(PIN_RADIO_NSS, GPIO_OUT);

    gpio_init(PIN_RADIO_RESET);
    gpio_put(PIN_RADIO_RESET, 1);
    gpio_set_dir(PIN_RADIO_RESET, GPIO_OUT);

    gpio_init(PIN_RADIO_BUSY);
    gpio_set_dir(PIN_RADIO_BUSY, GPIO_IN);
    gpio_disable_pulls(PIN_RADIO_BUSY);

    gpio_init(PIN_RADIO_IRQ);
    gpio_set_dir(PIN_RADIO_IRQ, GPIO_IN);
    gpio_disable_pulls(PIN_RADIO_IRQ);

    s_spi_hz = spi_init(RADIO_SPI, RADIO_SPI_HZ_SLOW);
    spi_set_format(RADIO_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_RADIO_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_RADIO_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_RADIO_MOSI, GPIO_FUNC_SPI);

    gpio_set_drive_strength(PIN_RADIO_SCK, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_RADIO_MOSI, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_RADIO_NSS, GPIO_DRIVE_STRENGTH_2MA);
}

void radio_hal_set_speed(uint32_t hz) { s_spi_hz = spi_set_baudrate(RADIO_SPI, hz); }
uint32_t radio_hal_get_speed(void) { return s_spi_hz; }
bool radio_hal_irq_pin(void) { return gpio_get(PIN_RADIO_IRQ); }
uint32_t radio_hal_busy_timeouts(void) { return s_busy_timeouts; }

bool radio_hal_wait_busy_low(uint32_t timeout_us) {
    uint32_t t0 = time_us_32();
    while (gpio_get(PIN_RADIO_BUSY)) {
        if (time_us_32() - t0 > timeout_us) return false;
    }
    return true;
}

uint32_t radio_hal_read_clear_irq_fast(void) {
    static const uint8_t tx[6] = {0x01, 0x16, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx[6];
    wait_busy();
    nss_low();
    spi_write_read_blocking(RADIO_SPI, tx, rx, 6);
    nss_high();
    return ((uint32_t)rx[2] << 24) | ((uint32_t)rx[3] << 16) | ((uint32_t)rx[4] << 8) | rx[5];
}

// ---------------------------------------------------------------------------
// lr20xx_hal.h implementation
// ---------------------------------------------------------------------------

lr20xx_hal_status_t lr20xx_hal_reset(const void* context) {
    (void)context;
    gpio_put(PIN_RADIO_RESET, 0);
    sleep_ms(2);
    gpio_put(PIN_RADIO_RESET, 1);
    sleep_ms(10);
    return radio_hal_wait_busy_low(500000) ? LR20XX_HAL_STATUS_OK : LR20XX_HAL_STATUS_ERROR;
}

lr20xx_hal_status_t lr20xx_hal_wakeup(const void* context) {
    (void)context;
    nss_low();
    sleep_us(100);
    nss_high();
    return radio_hal_wait_busy_low(100000) ? LR20XX_HAL_STATUS_OK : LR20XX_HAL_STATUS_ERROR;
}

lr20xx_hal_status_t lr20xx_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length) {
    (void)context;
    wait_busy();
    nss_low();
    spi_write_blocking(RADIO_SPI, command, command_length);
    if (data_length) spi_write_blocking(RADIO_SPI, data, data_length);
    nss_high();
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length) {
    (void)context;
    wait_busy();
    nss_low();
    spi_write_blocking(RADIO_SPI, command, command_length);
    nss_high();
    if (data_length) {
        uint8_t dummy[2];
        wait_busy();
        nss_low();
        spi_read_blocking(RADIO_SPI, 0x00, dummy, 2);  // stat1, stat2
        spi_read_blocking(RADIO_SPI, 0x00, data, data_length);
        nss_high();
    }
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read(const void* context, uint8_t* data, const uint16_t data_length) {
    (void)context;
    wait_busy();
    nss_low();
    spi_read_blocking(RADIO_SPI, 0x00, data, data_length);
    nss_high();
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read_fifo(const void* context, const uint8_t* command,
                                                const uint16_t command_length, uint8_t* data,
                                                const uint16_t data_length) {
    (void)context;
    wait_busy();
    nss_low();
    spi_write_blocking(RADIO_SPI, command, command_length);
    if (data_length) spi_read_blocking(RADIO_SPI, 0x00, data, data_length);
    nss_high();
    return LR20XX_HAL_STATUS_OK;
}
