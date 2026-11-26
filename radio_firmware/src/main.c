// Horizon radio firmware (RP2350 / RP2354): one build is one role on one board (CMakeLists.txt). The transmitter
// (hat_tx: the Horizon HAT on a Raspberry Pi 5; air_radio: the Horizon Dev Air's radio controller) sends the video its
// host hands over; the receiver (hat_rx: the Horizon HAT on its own; ground_unit) forwards every packet to its computer
// over USB. The over-the-air format: format/README.md.
//   common.h / common.c  shared by both roles: the protocol, hopping, the link ID, callsign packets, identity
//   tx.c                 the transmitter: host link, slot loop, identification, firmware update over the host link
//   rx.c                 the receiver: acquisition and tracking, packets to USB, pairing profile, commands
#include "common.h"

int main(void) {
    hz_guard_start();
    stdio_init_all();
#ifdef PIN_LED
    gpio_init(PIN_LED);  // the status LED: off until the role turns it on
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);
#endif
    pico_get_unique_board_id_string(g_id, sizeof g_id);
    sleep_ms(1500);
    who_print();
#ifdef LIVE_TX
    tx_main();
#else
    rx_main();
#endif
}
