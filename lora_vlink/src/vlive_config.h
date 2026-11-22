// Live video link (vlive_tx / vlive_rx): boot RF frequency (the receiver changes it at run time over the link).
// Default 868 MHz (Horizon Dev, Europe); a build can set another with  cmake -DVLIVE_FREQ_HZ=915000000
// (CMakeLists.txt). TX and RX must boot on the same value to find each other.
#pragma once
#ifndef VLIVE_FREQ_HZ
#define VLIVE_FREQ_HZ 868000000u
#endif
