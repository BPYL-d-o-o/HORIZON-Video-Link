// One-time LR2021 configuration (identical on both nodes).
#include "radio.h"

#include <stdio.h>

#include "board.h"
#include "lr20xx_pram_load.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_system.h"
#include "radio_hal.h"
#include "pico/time.h"

const void* const radio = 0;

#define CHK(step, expr)                                         \
    do {                                                        \
        if ((expr) != LR20XX_STATUS_OK) {                       \
            printf("radio_init: step %d failed: %s\n", step, #expr); \
            return -(step);                                     \
        }                                                       \
    } while (0)

bool radio_set_freq(uint32_t hz) { return lr20xx_radio_common_set_rf_freq(radio, hz) == LR20XX_STATUS_OK; }

static bool g_hf;
static uint32_t g_init_hz = RF_FREQ_HZ;
static int8_t g_req_power = TX_POWER_HALF_DBM;
bool radio_is_hf(void) { return g_hf; }
static lr20xx_radio_common_rx_path_t rx_path(void) { return g_hf ? LR20XX_RADIO_COMMON_RX_PATH_HF : LR20XX_RADIO_COMMON_RX_PATH_LF; }
// A retune into the other band: the radio is set up again from reset on that band's path (~60 ms; between packets).
static bool band_for(uint32_t hz) {
    bool hf = hz >= RADIO_HF_MIN_HZ;
    if (hf == g_hf) return true;
    g_hf = hf;
    g_init_hz = hz;
    int rc = radio_init();
    printf("radio: %s band at %lu Hz%s\n", hf ? "2.4 GHz" : "sub-GHz", (unsigned long)hz, rc ? " FAILED" : "");
    return rc == 0;
}

bool radio_set_payload_len(uint8_t len) {
    const lr20xx_radio_lora_pkt_params_t pkt = {
        .preamble_len_in_symb = LORA_PREAMBLE,
        .pkt_mode             = LR20XX_RADIO_LORA_PKT_IMPLICIT,
        .pld_len_in_bytes     = len,
        .crc                  = LR20XX_RADIO_LORA_CRC_ENABLED,
        .iq                   = LR20XX_RADIO_LORA_IQ_STANDARD,
    };
    return lr20xx_radio_lora_set_packet_params(radio, &pkt) == LR20XX_STATUS_OK;
}

bool radio_set_power(int8_t half_dbm) {
    if (half_dbm < -19) half_dbm = -19;  // the sub-GHz PA's minimum (-9.5 dBm); the 2.4 GHz path uses RADIO_HF_POWER
    g_req_power = half_dbm;
    return lr20xx_radio_common_set_tx_params(radio, g_hf ? RADIO_HF_POWER : half_dbm, LR20XX_RADIO_COMMON_RAMP_48_US) ==
           LR20XX_STATUS_OK;
}

bool radio_retune(uint32_t hz) {
    if (!band_for(hz)) return false;
    if (lr20xx_system_set_standby_mode(radio, LR20XX_SYSTEM_STANDBY_MODE_XOSC) != LR20XX_STATUS_OK) return false;
    if (!radio_set_freq(hz)) return false;
    const lr20xx_radio_common_front_end_calibration_value_t fe_cal[3] = {
        {.rx_path = rx_path(), .frequency_in_hertz = hz - 10000000u},
        {.rx_path = rx_path(), .frequency_in_hertz = hz},
        {.rx_path = rx_path(), .frequency_in_hertz = hz + 10000000u},
    };
    bool ok = lr20xx_radio_common_calibrate_front_end_helper(radio, fe_cal, 3) == LR20XX_STATUS_OK;
    lr20xx_system_clear_errors(radio);
    lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
    lr20xx_radio_fifo_clear_rx(radio);
    lr20xx_radio_fifo_clear_tx(radio);
    lr20xx_system_set_fs_mode(radio);
    return ok;
}

bool radio_retune_span(uint32_t hz, uint32_t lo, uint32_t hi) {
    if (hz - lo <= 10000000u && hi - hz <= 10000000u) return radio_retune(hz);
    if (!band_for(hz)) return false;
    if (lr20xx_system_set_standby_mode(radio, LR20XX_SYSTEM_STANDBY_MODE_XOSC) != LR20XX_STATUS_OK) return false;
    if (!radio_set_freq(hz)) return false;
    const lr20xx_radio_common_front_end_calibration_value_t fe_cal[3] = {
        {.rx_path = rx_path(), .frequency_in_hertz = hz - 10000000u < lo ? hz - 10000000u : lo},
        {.rx_path = rx_path(), .frequency_in_hertz = hz},
        {.rx_path = rx_path(), .frequency_in_hertz = hz + 10000000u > hi ? hz + 10000000u : hi},
    };
    bool ok = lr20xx_radio_common_calibrate_front_end_helper(radio, fe_cal, 3) == LR20XX_STATUS_OK;
    lr20xx_system_clear_errors(radio);
    lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
    lr20xx_radio_fifo_clear_rx(radio);
    lr20xx_radio_fifo_clear_tx(radio);
    lr20xx_system_set_fs_mode(radio);
    return ok;
}

const char* radio_fail_cause(int rc) {
    switch (-rc) {
    case 0: return "ok";
    case RADIO_STEP_RESET: return "busy_stuck_high";
    case RADIO_STEP_VERSION: return "spi_no_response";
    case RADIO_STEP_TCXO: return "tcxo_start";
    default: return "init_failed";
    }
}

int radio_init(void) {
    radio_hal_init();
    radio_hal_set_speed(RADIO_SPI_HZ_SLOW);

    CHK(1, lr20xx_system_reset(radio));  // HAL: NRESET low 2 ms, wait 10 ms, wait BUSY low

    lr20xx_system_version_t ver = {0};
    CHK(2, lr20xx_system_get_version(radio, &ver));
    printf("radio: LR20xx fw version %u.%u\n", ver.major, ver.minor);
    if ((ver.major == 0 && ver.minor == 0) || (ver.major == 0xFF && ver.minor == 0xFF)) {
        printf("radio: version looks bogus -> SPI wiring problem?\n");
        return -2;
    }

    CHK(3, lr20xx_pram_load_pram_lr2021(radio));
    CHK(4, lr20xx_system_set_reg_mode(radio, LR20XX_SYSTEM_REG_MODE_LDO));
    // TCXO supply per module (board.h RADIO_TCXO_CTRL): 1.8 V on the NiceRF modules, 3.0 V on the Seeed Wio-LR2021
    CHK(5, lr20xx_system_set_tcxo_mode(radio, RADIO_TCXO_CTRL, 320000));  // 10 ms in 32 MHz steps
    CHK(6, lr20xx_system_clear_errors(radio));
    CHK(7, lr20xx_system_calibrate(radio, 0x6F));  // after TCXO is up (as in Semtech RAL)
    {
        // A TCXO that did not start (wrong supply voltage: firmware for another module) fails calibration. The Ground
        // Unit stops here with a cause the host can show (radio_fail_cause()); the NiceRF boards only report it.
        lr20xx_system_errors_t xerr = 0;
        lr20xx_system_get_errors(radio, &xerr);
        if (xerr & LR20XX_SYSTEM_ERRORS_HF_XOSC_START_MASK) {
            printf("radio: chip errors after calibration: 0x%04x (HF_XOSC_START: the TCXO did not start)\n", xerr);
#if HZ_BOARD == HZ_BOARD_GROUND_UNIT
            return -RADIO_STEP_TCXO;
#endif
        }
    }

#ifndef RADIO_NO_FRONT_END
    // RF switch / external PA enable (TXEN) on DIO6, high only when transmitting on the LF path.
    CHK(8, lr20xx_system_set_dio_function(radio, LR20XX_SYSTEM_DIO_6, LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH,
                                          LR20XX_SYSTEM_DIO_DRIVE_PULL_DOWN));
    CHK(9, lr20xx_system_set_dio_rf_switch_cfg(radio, LR20XX_SYSTEM_DIO_6, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_LF));
    // NiceRF LoRa2021F33-2G4, 2.4 GHz: DIO7 powers the front end, DIO8 switches it to its PA while transmitting, DIO5
    // high turns its LNA on (low = bypass, 12 dB less sensitive). On sub-GHz these pins must stay at their reset
    // state: DIO7 driven low cuts the sub-GHz output by about 20 dB.
    if (g_hf) {
        CHK(27, lr20xx_system_set_dio_function(radio, LR20XX_SYSTEM_DIO_7, LR20XX_SYSTEM_DIO_FUNC_GPIO_HIGH,
                                               LR20XX_SYSTEM_DIO_DRIVE_PULL_DOWN));
        CHK(28, lr20xx_system_set_dio_function(radio, LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH,
                                               LR20XX_SYSTEM_DIO_DRIVE_PULL_DOWN));
        CHK(29, lr20xx_system_set_dio_rf_switch_cfg(radio, LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF));
        CHK(30, lr20xx_system_set_dio_function(radio, LR20XX_SYSTEM_DIO_5, LR20XX_SYSTEM_DIO_FUNC_GPIO_HIGH,
                                               LR20XX_SYSTEM_DIO_DRIVE_PULL_UP));
        sleep_ms(2);  // front-end supply settling
    }
#endif  // RADIO_NO_FRONT_END (the Wio-LR2021: no RF switch or front end on its DIOs)

    CHK(10, lr20xx_radio_common_set_pkt_type(radio, LR20XX_RADIO_COMMON_PKT_TYPE_LORA));
    // RX boost: 1 dB better sub-GHz sensitivity for 1.2 mA. Transmitters only listen for commands: off.
#ifndef LIVE_TX
#define RX_BOOST    LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_7
#define RX_BOOST_HF LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_4  // as lora_modem.ino on the 2.4 GHz path
#else
#define RX_BOOST    LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE
#define RX_BOOST_HF LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE
#endif
    CHK(11, lr20xx_radio_common_set_rx_path(radio, rx_path(), g_hf ? RX_BOOST_HF : RX_BOOST));
    CHK(12, lr20xx_radio_common_set_rf_freq(radio, g_init_hz));

    const lr20xx_radio_common_front_end_calibration_value_t fe_cal_hf[3] = {
        {.rx_path = LR20XX_RADIO_COMMON_RX_PATH_HF, .frequency_in_hertz = g_init_hz - 10000000u},
        {.rx_path = LR20XX_RADIO_COMMON_RX_PATH_HF, .frequency_in_hertz = g_init_hz},
        {.rx_path = LR20XX_RADIO_COMMON_RX_PATH_HF, .frequency_in_hertz = g_init_hz + 10000000u},
    };
    const lr20xx_radio_common_front_end_calibration_value_t fe_cal[3] = {
        {.rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF, .frequency_in_hertz = FIXED_FREQ_HZ - 10000000u},
        {.rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF, .frequency_in_hertz = FIXED_FREQ_HZ},
        {.rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF, .frequency_in_hertz = FIXED_FREQ_HZ + 10000000u},
    };
    CHK(13, lr20xx_radio_common_calibrate_front_end_helper(radio, g_hf ? fe_cal_hf : fe_cal, 3));

    const lr20xx_radio_lora_mod_params_t mod = {
        .sf  = LR20XX_RADIO_LORA_SF5,
        .bw  = LR20XX_RADIO_LORA_BW_1000,
        // The lightest coding, for the shortest packet (convolutional 4/6: ~1 dB more sensitive, 20.7 ms per packet).
        .cr  = LR20XX_RADIO_LORA_CR_LI_4_5,
        .ppm = LR20XX_RADIO_LORA_NO_PPM,
    };
    CHK(14, lr20xx_radio_lora_set_modulation_params(radio, &mod));

    const lr20xx_radio_lora_pkt_params_t pkt = {
        .preamble_len_in_symb = LORA_PREAMBLE,
        .pkt_mode             = LR20XX_RADIO_LORA_PKT_IMPLICIT,
        .pld_len_in_bytes     = PAYLOAD_LEN,
        .crc                  = LR20XX_RADIO_LORA_CRC_ENABLED,
        .iq                   = LR20XX_RADIO_LORA_IQ_STANDARD,
    };
    CHK(15, lr20xx_radio_lora_set_packet_params(radio, &pkt));
    CHK(16, lr20xx_radio_lora_set_syncword(radio, LORA_SYNCWORD));

    const lr20xx_radio_common_pa_cfg_t pa = {
        .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
        .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
        .pa_lf_duty_cycle = 2,
        .pa_lf_slices     = 5,
        .pa_hf_duty_cycle = 16,
    };
    // 2.4 GHz: the HF PA, settings as lora_modem.ino (it only feeds the module's front-end PA)
    const lr20xx_radio_common_pa_cfg_t pa_hf = {
        .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
        .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
        .pa_lf_duty_cycle = 6,
        .pa_lf_slices     = 7,
        .pa_hf_duty_cycle = 30,
    };
    CHK(17, lr20xx_radio_common_set_pa_cfg(radio, g_hf ? &pa_hf : &pa));
    CHK(18, lr20xx_radio_common_set_tx_params(radio, g_hf ? RADIO_HF_POWER : g_req_power, LR20XX_RADIO_COMMON_RAMP_48_US));

    // Stay in FS between TX/RX so the PLL stays locked (fast turnaround on a single carrier).
    CHK(19, lr20xx_radio_common_set_rx_tx_fallback_mode(radio, LR20XX_RADIO_FALLBACK_FS));

    // The IRQ DIO (board.h RADIO_IRQ_DIO: DIO9 on the NiceRF modules, DIO11 on the Wio-LR2021) must be switched to the
    // IRQ function; SetDioIrqConfig alone does not drive the pin.
    CHK(26, lr20xx_system_set_dio_function(radio, RADIO_IRQ_DIO, LR20XX_SYSTEM_DIO_FUNC_IRQ, LR20XX_SYSTEM_DIO_DRIVE_NONE));
    CHK(20, lr20xx_system_set_dio_irq_cfg(radio, RADIO_IRQ_DIO, RADIO_IRQS));
    CHK(21, lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK));
    CHK(22, lr20xx_radio_fifo_clear_rx(radio));
    CHK(23, lr20xx_radio_fifo_clear_tx(radio));

    lr20xx_system_errors_t err = 0;
    CHK(24, lr20xx_system_get_errors(radio, &err));
    if (err) printf("radio: chip errors after init: 0x%04x\n", err);
    CHK(25, lr20xx_system_clear_errors(radio));

    radio_hal_set_speed(RADIO_SPI_HZ_FAST);
    printf("radio: init ok, SPI %lu Hz, IRQ pin=%d\n", (unsigned long)radio_hal_get_speed(), radio_hal_irq_pin());
    return 0;
}
