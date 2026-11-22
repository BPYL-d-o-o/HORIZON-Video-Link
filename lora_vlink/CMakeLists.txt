cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board")
include(pico_sdk_import.cmake)

project(lr2021_vlink C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

file(GLOB LR20XX_SRC ${CMAKE_CURRENT_LIST_DIR}/third_party/lr20xx_driver/src/*.c)
set(VIDEO_IMAGE ${CMAKE_CURRENT_LIST_DIR}/video/video_image.bin)

# Firmware identity (src/main.c "Identity"): every build reports its target name and the commit it was built from
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
# Only main.c uses it: a new commit recompiles that one file, not every target's sources.
set_source_files_properties(src/main.c PROPERTIES COMPILE_DEFINITIONS "HZ_FW_GIT=\"${HZ_FW_GIT}\"")

# Boot RF frequency of the live link (src/vlive_config.h); empty = its default (868 MHz).
set(VLIVE_FREQ_HZ "" CACHE STRING "boot frequency of vlive_* in Hz (empty: 868000000)")
if(VLIVE_FREQ_HZ)
    add_compile_definitions(VLIVE_FREQ_HZ=${VLIVE_FREQ_HZ}u)
endif()

# USB product name per firmware (what the OS and the browser's port chooser show; tools find the boards by the
# Raspberry Pi vendor id 2e8a and the USB serial number = the chip id, not by these names).
set(HZ_USB_PRODUCT_vlive_rx "Horizon Dev Ground rev-1A")
set(HZ_USB_PRODUCT_vlive_tx "Horizon Pi transmitter")
set(HZ_USB_PRODUCT_vlive_tx_luckfox "Horizon Dev Air radio")
set(HZ_USB_PRODUCT_vlive_tx_video_tx "Horizon Dev Air radio")

function(vlink_target name)
    add_executable(${name} src/main.c src/radio.c src/radio_hal.c src/usb_descriptors.c ${LR20XX_SRC} ${ARGN})
    target_include_directories(${name} PRIVATE src third_party/lr20xx_driver/inc)
    if(DEFINED HZ_USB_PRODUCT_${name})
        set(product "${HZ_USB_PRODUCT_${name}}")
    else()
        set(product "Horizon test ${name}")
    endif()
    target_compile_definitions(${name} PRIVATE LR2021 PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000
                               HZ_FW_TARGET="${name}"
                               USBD_MANUFACTURER="BPYL" USBD_PRODUCT="${product}"
                               PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS=0)  # src/usb_descriptors.c instead
    target_compile_options(${name} PRIVATE -O2)
    target_link_libraries(${name} pico_stdlib pico_unique_id hardware_spi)
    pico_enable_stdio_usb(${name} 1)
    pico_enable_stdio_uart(${name} 0)
    pico_add_extra_outputs(${name})
endfunction()

# A: transmitter with the flash-resident packet image.
vlink_target(vlink_tx src/video_image.S)
target_compile_definitions(vlink_tx PRIVATE VIDEO_TX VIDEO_IMAGE_PATH="${VIDEO_IMAGE}")
set_source_files_properties(src/video_image.S PROPERTIES OBJECT_DEPENDS ${VIDEO_IMAGE})

# B: receiver, forwards packets to the host over USB.
vlink_target(vlink_rx)

# Fixed 915 MHz (no hopping) pair; must be used together.
vlink_target(vlink_tx_915 src/video_image.S)
target_compile_definitions(vlink_tx_915 PRIVATE VIDEO_TX VIDEO_IMAGE_PATH="${VIDEO_IMAGE}" FIXED_FREQ_HZ=915000000u)
vlink_target(vlink_rx_915)
target_compile_definitions(vlink_rx_915 PRIVATE FIXED_FREQ_HZ=915000000u)

# Live video link on ONE fixed frequency, set in src/vlive_config.h (VLIVE_FREQ_HZ).
# vlive_tx: Raspberry Pi transmitter (pi5_vtx/live_sender.py); records from the Pi over SPI (hardware SPI1 slave)
#           or over the USB serial port (live_sender.py --link usb), status back the same way.
# vlive_rx: receiver, forwards packets to the host over USB (the receiver apps).
# Boot TX power (0.5 dBm units, -19..44): 44 = 22 dBm needs a proper supply; a board powered from a Pi's USB port
# should be built with e.g. -DVLIVE_TX_POWER=-19 (live_sender.py --tx-power changes it at run time either way).
set(VLIVE_TX_POWER 44 CACHE STRING "vlive_tx boot TX power, 0.5 dBm units (-19..44)")
vlink_target(vlive_tx)
target_compile_definitions(vlive_tx PRIVATE VLIVE LIVE_TX TX_POWER_HALF_DBM=\(${VLIVE_TX_POWER}\))
target_link_libraries(vlive_tx hardware_dma hardware_flash)
vlink_target(vlive_rx)
target_compile_definitions(vlive_rx PRIVATE VLIVE)
target_link_libraries(vlive_rx hardware_flash)  # pairing profile in flash (src/main.c, "Pairing profile")

# vlive_tx_luckfox: the same transmitter for a Luckfox Pico host (luckfox_vtx/) on the Luckfox carrier.
# Identical radio, slots, command slots and over-the-air format as vlive_tx (works with the same vlive_rx /
# receiver apps); only the host SPI is a PIO slave on the carrier's pins (src/board.h, VLIVE_HOST_PIO).
# Board: RP2354A (RP2350 A package, 30 GPIOs, 2 MB in-package flash). The pico2 board config already
# selects the A package (PICO_RP2350A=1); only the flash size differs, set here for this target.
vlink_target(vlive_tx_luckfox)
# Boot TX power of this target (0.5 dBm units). Default -19 = -9.5 dBm, the LR2021 minimum: the bench boards run
# from the Raspberry Pi 5's USB ports, and at 44 (22 dBm) the PA current tripped the Pi's USB over-current
# protection. Raise it at run time instead: from the Luckfox (`service.sh power <half_dbm>`, or hz_vtx --tx-power in
# horizon_vtx.conf) or from the receiver over the air. On a proper supply, build with e.g. -DVLIVE_LUCKFOX_TX_POWER=44.
set(VLIVE_LUCKFOX_TX_POWER -19 CACHE STRING "vlive_tx_luckfox boot TX power, 0.5 dBm units (-19..44)")
target_compile_definitions(vlive_tx_luckfox PRIVATE VLIVE LIVE_TX VLIVE_HOST_PIO HZ_BOARD=2 PICO_FLASH_SIZE_BYTES=2097152
                           TX_POWER_HALF_DBM=\(${VLIVE_LUCKFOX_TX_POWER}\))
target_link_libraries(vlive_tx_luckfox hardware_dma hardware_pio hardware_flash)

# vlive_tx_video_tx: the same transmitter on the integrated video TX board (Luckfox Pico, RP2354A and NiceRF
# LoRa2021F33 on one carrier; pins in src/board.h, HZ_BOARD 3). Host = the Luckfox (luckfox_vtx/), PIO SPI slave as
# vlive_tx_luckfox; radio on SPI1 with the module's CE driven high. The RP2354A has no USB connection on this board:
# flash it over SWD from the Luckfox (the Air image does it at boot; a test build: luckfox_vtx/flash_rp2354.sh). Boot TX power as vlive_tx_luckfox.
vlink_target(vlive_tx_video_tx)
target_compile_definitions(vlive_tx_video_tx PRIVATE VLIVE LIVE_TX VLIVE_HOST_PIO HZ_BOARD=3 PICO_FLASH_SIZE_BYTES=2097152
                           TX_POWER_HALF_DBM=\(${VLIVE_LUCKFOX_TX_POWER}\))
target_link_libraries(vlive_tx_video_tx hardware_dma hardware_pio hardware_flash)
# The Air's RP2354A runs at 160 MHz (stock 150): the radio SPI then gets exactly 16 MHz (160 / 10, the LR2021's
# limit) instead of 15, and the PIO host link answers ~7 % sooner after each SCK edge. PLL: 12 MHz x 120 = 1440 MHz
# VCO / 3 / 3. clk_peri follows clk_sys, so SPI and UART rates are computed from 160 MHz; timers are unaffected.
target_compile_definitions(vlive_tx_video_tx PRIVATE SYS_CLK_HZ=160000000 PLL_SYS_VCO_FREQ_HZ=1440000000
                           PLL_SYS_POSTDIV1=3 PLL_SYS_POSTDIV2=3)
