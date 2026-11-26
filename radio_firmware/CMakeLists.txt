cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board")
include(pico_sdk_import.cmake)

project(horizon_radio C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

file(GLOB LR20XX_SRC ${CMAKE_CURRENT_LIST_DIR}/third_party/lr20xx_driver/src/*.c)

# Firmware identity (src/common.h "Identity"): every build reports its target name and the commit it was built from
# (taken when CMake configures; "-dirty" = uncommitted changes then). The environment variable HZ_FW_GIT overrides
# it: the git hooks build an exported commit, which is not a git checkout (luckfox_image/air_build.sh).
if(NOT "$ENV{HZ_FW_GIT}" STREQUAL "")
    set(HZ_FW_GIT "$ENV{HZ_FW_GIT}")
else()
    execute_process(COMMAND git rev-parse --short=8 HEAD WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                    OUTPUT_VARIABLE HZ_FW_GIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE git_rc)
    if(NOT git_rc EQUAL 0 OR HZ_FW_GIT STREQUAL "")
        set(HZ_FW_GIT "unknown")
    else()
        execute_process(COMMAND git status --porcelain --untracked-files=no WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                        OUTPUT_VARIABLE git_dirty OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(NOT git_dirty STREQUAL "")
            set(HZ_FW_GIT "${HZ_FW_GIT}-dirty")
        endif()
    endif()
endif()
message(STATUS "firmware identity: git=${HZ_FW_GIT}")
# Only common.c uses it: a new commit recompiles that one file, not every target's sources.
set_source_files_properties(src/common.c PROPERTIES COMPILE_DEFINITIONS "HZ_FW_GIT=\"${HZ_FW_GIT}\"")

# Boot RF frequency of the live link (src/link_config.h); empty = its default (868 MHz).
set(HZ_FREQ_HZ "" CACHE STRING "boot frequency of every target in Hz (empty: 868000000)")
if(HZ_FREQ_HZ)
    add_compile_definitions(HZ_FREQ_HZ=${HZ_FREQ_HZ}u)
endif()

# USB product name per firmware (what the OS and the browser's port chooser show; tools find the boards by the
# Raspberry Pi vendor id 2e8a and the USB serial number = the chip id, not by these names).
set(HZ_USB_PRODUCT_hat_rx "Horizon HAT receiver")
set(HZ_USB_PRODUCT_ground_unit "Horizon Dev Ground Unit")
set(HZ_USB_PRODUCT_hat_tx "Horizon HAT transmitter")
set(HZ_USB_PRODUCT_air_radio "Horizon Dev Air radio")

function(hz_target name)
    add_executable(${name} src/main.c src/common.c src/tx.c src/rx.c src/radio.c src/radio_hal.c src/usb_descriptors.c ${LR20XX_SRC} ${ARGN})
    target_include_directories(${name} PRIVATE src third_party/lr20xx_driver/inc)
    target_compile_definitions(${name} PRIVATE LR2021 PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000
                               HZ_FW_TARGET="${name}"
                               USBD_MANUFACTURER="BPYL" USBD_PRODUCT="${HZ_USB_PRODUCT_${name}}"
                               PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS=0)  # src/usb_descriptors.c instead
    target_compile_options(${name} PRIVATE -O2)
    target_link_libraries(${name} pico_stdlib pico_unique_id hardware_spi hardware_exception hardware_watchdog)
    pico_enable_stdio_usb(${name} 1)
    pico_enable_stdio_uart(${name} 0)
    pico_add_extra_outputs(${name})
endfunction()

# Live video link on ONE fixed frequency, set in src/link_config.h (HZ_FREQ_HZ).
# hat_tx: the Horizon HAT on a Raspberry Pi 5 (transmitter/, hz_vtx built with build_pi.sh); records from the Pi
#           over SPI (hardware SPI1 slave), status back the same way; its USB port takes text commands.
# hat_rx: receiver, forwards packets to the host over USB (the receiver apps).
# Boot TX power (0.5 dBm units, -19..44): 44 = 22 dBm needs a proper supply; a board powered from a Pi's USB port
# should be built with e.g. -DHZ_HAT_TX_POWER=-19 (hz_vtx --tx-power changes it at run time either way).
set(HZ_HAT_TX_POWER 44 CACHE STRING "hat_tx boot TX power, 0.5 dBm units (-19..44)")
hz_target(hat_tx)
target_compile_definitions(hat_tx PRIVATE LIVE_TX TX_POWER_HALF_DBM=\(${HZ_HAT_TX_POWER}\))
target_link_libraries(hat_tx hardware_dma hardware_flash)
hz_target(hat_rx)
target_link_libraries(hat_rx hardware_flash)  # pairing profile in flash (src/rx.c, "Pairing profile")

# ground_unit: the same receiver on the Horizon Dev Ground Unit (rev B): RP2354A (2 MB in-package flash, so
# the pairing profile sits at the end of 2 MB) and a Seeed Wio-LR2021 module (pins, IRQ DIO11, TCXO 3.0 V, BUSY
# pull-up: src/board.h HZ_BOARD 4). Radio settings, slots and over-the-air format as hat_rx.
hz_target(ground_unit)
target_compile_definitions(ground_unit PRIVATE HZ_BOARD=4 PICO_FLASH_SIZE_BYTES=2097152)
target_link_libraries(ground_unit hardware_flash)

# air_radio: the same transmitter on the Horizon Dev Air (Luckfox Pico, RP2354A and NiceRF LoRa2021F33 on
# one board; pins in src/board.h, HZ_BOARD 3). Identical radio, slots, command slots and over-the-air format as
# hat_tx; the host is the Luckfox (transmitter/), its SPI a PIO slave (HZ_HOST_PIO); radio on SPI1 with the
# module's CE driven high. RP2354A: the pico2 board config already selects the A package; only the flash size (2 MB
# in-package) is set here. It has no USB connection on this board: the Luckfox flashes it over SPI or SWD (the Air
# image does it at boot; a test build: luckfox_image/rp2354_update.sh flash).
# Boot TX power (0.5 dBm units). Default -19 = -9.5 dBm, the LR2021 minimum: the bench boards ran from the Raspberry
# Pi 5's USB ports, and at 44 (22 dBm) the PA current tripped the Pi's USB over-current protection. Raise it at run
# time instead: from the Luckfox (`service.sh power <half_dbm>`, or hz_vtx --tx-power in horizon_vtx.conf) or from
# the receiver over the air. On a proper supply, build with e.g. -DHZ_AIR_TX_POWER=44.
set(HZ_AIR_TX_POWER -19 CACHE STRING "air_radio boot TX power, 0.5 dBm units (-19..44)")
hz_target(air_radio)
target_compile_definitions(air_radio PRIVATE LIVE_TX HZ_HOST_PIO HZ_BOARD=3 PICO_FLASH_SIZE_BYTES=2097152
                           TX_POWER_HALF_DBM=\(${HZ_AIR_TX_POWER}\))
target_link_libraries(air_radio hardware_dma hardware_pio hardware_flash)
# The Air's RP2354A runs at 160 MHz (stock 150): the radio SPI then gets exactly 16 MHz (160 / 10, the LR2021's
# limit) instead of 15, and the PIO host link answers ~7 % sooner after each SCK edge. PLL: 12 MHz x 120 = 1440 MHz
# VCO / 3 / 3. clk_peri follows clk_sys, so SPI and UART rates are computed from 160 MHz; timers are unaffected.
target_compile_definitions(air_radio PRIVATE SYS_CLK_HZ=160000000 PLL_SYS_VCO_FREQ_HZ=1440000000
                           PLL_SYS_POSTDIV1=3 PLL_SYS_POSTDIV2=3)
