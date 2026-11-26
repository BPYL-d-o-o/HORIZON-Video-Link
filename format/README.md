# The Horizon over-the-air format

How a Horizon transmitter (the Air, or a Raspberry Pi 5 with the Horizon HAT) sends video to a Ground, and what is in
every radio packet. This folder holds the format's reference code; every other implementation is checked against it.

| File | What |
| --- | --- |
| `packet.py` | The reference: packet layout, CRC-16, the in-packet Reed-Solomon code, the callsign blob |
| `block_fec.py` | The reference block FEC across packets |
| `packet.h`, `block_fec.h` | The same in C, for `hz_vtx` (`transmitter/`, both Air units) and the radio firmware (`radio_firmware/`) |

The receivers have their own ports: Rust in `desktop_receiver/src/engine.rs`, JavaScript in
`setup_site/js/lib/swfec.js`. The tests keep every copy bit-identical:

- `transmitter/test/test_swfec.py`: the C code and the transmitters' scheduler against the Python reference.
- `test/block_fec.sh`: Rust against JavaScript on a capture with lost and damaged packets.

The radio firmware reports the format number it speaks (`fmt=` in its `W` line); this is format 5.

## The radio

LoRa on a Semtech LR2021: spreading factor 5, 1 MHz bandwidth, the long interleaver at coding rate 4/5, 12-symbol
preamble, sync word 0x12, implicit header, the radio's own CRC on. Every packet is 255 bytes and takes 17.2 ms on air. Sub-GHz
(150–960 MHz) or the 2.4 GHz band (2400–2483.5 MHz).

## Slots and sync

Time is divided into **slots of 17.54 ms**, one packet per slot, numbered by a 16-bit slot counter (`seq16`). The
transmitter's radio controller sends a packet in every slot, so the receiver always has something to lock onto:

- video, or block FEC parity, when the host has some queued;
- otherwise an **idle packet** (entry type 0xFF), which carries the Air status (below).

The first two bytes of every packet are `seq16`, little-endian. The radio controller adds them; hosts never see
them.

**Sync.** A receiver that has no lock **parks** on one frequency (the centre of the plan) and listens continuously.
The first valid packet tells it the slot number and when that slot ended. From then on it **tracks**: it opens one
receive window per slot, from 0.4 ms before the predicted start to 0.5 ms after the predicted end, on the channel that
slot uses. After a packet it re-arms at once, since the gap between packets is only about 0.3 ms. After 32 missed
slots in a row (about 0.56 s) it goes back to parking. While parked with no packets, it also tries the previous
frequency or plan and the boot default, so it finds a transmitter that changed frequency while the receiver missed the
announcement.

**Command slots.** Every 64th slot (`seq16 % 64 == 0`, about every 1.1 s) carries no video. The transmitter listens in
it, and uses it for its callsign identification. A receiver can send a short command in it (change frequency, power,
hop plan or callsign). The transmitter acknowledges a command in 3 of the following slots, and both sides apply the
change 6 slots after the command slot. A frequency change leaves 2 slots silent for the retune and calibration. Without
an acknowledgement nothing changes, and the receiver tries again at the next command slot.

## Frequency hopping

A **plan** is a centre frequency, a channel count `n` (0 = one fixed frequency, else 2–128) and a channel spacing.
Channel `k` is `centre + (k - n/2) × spacing`, so the centre is always a channel. Slot `seq16` uses channel
`hop_tab[seq16 % n]`, one packet per channel. `hop_tab` is a fixed pseudo-random order of the `n` channels that
depends only on `n` (a shuffle seeded with `0x48F0C3A5 ^ n`). Both sides compute the channel from the slot number every
packet already carries, so hopping costs nothing on air.

A hopping transmitter announces its plan on the centre channel about every 2.5 s (every 144 slots). A receiver that
parks on the centre learns the plan from that announcement and starts hopping with it. Plans come from pairing on the
setup site; both sides keep them across restarts.

## Packet contents

```
byte  0..1      seq16 (little-endian), added by the transmitter's radio controller
byte  2..254    the entry, 253 bytes, built by the host (`hz_vtx`, on the Air and on the Pi)
```

A video entry (offsets inside the entry):

| Bytes | Field |
| --- | --- |
| 0 | type: 0x00 video, 0x02 block FEC parity (with the interleaving depth in bits 4..6), 0xFF idle / acknowledgement / announcement |
| 1..2 | `id16`: the video packet number (little-endian, consecutive, so the receiver sees every lost one) |
| 3 | `aux`: 0 for video; for parity, the block size and parity index (below) |
| 4..234 | the symbol: the next 231 bytes of the video byte stream |
| 235..236 | CRC-16/MODBUS of bytes 0..234, little-endian, keyed with the link ID (below) |
| 237..252 | the Reed-Solomon parity: 16 bytes over bytes 0..236, keyed with the link ID |

**The video byte stream.** H.265 access units, one after the other. Each one is its slices, plus VPS/SPS/PPS when
they are due or changed (with a keyframe, at most once per 2 s by default), plus an optional timestamp NAL, then an
access unit delimiter (AUD). Packets are never padded while video waits: one packet can carry the end of one picture
and the start of the next, so a picture's last bytes never wait for the next picture. The receiver cuts the stream
at the AUDs.

**Link ID.** A 16-bit key derived from the callsign: FNV-1a over the callsign blob, folded to 16 bits, never 0. The
transmitter's radio controller XORs it into the entry's CRC, and the matching change into the Reed-Solomon parity.
The parity stays a valid codeword, because both codes are linear. A receiver paired with another callsign repairs
the packet as usual, then the CRC fails, so it drops other systems' packets as if they were damaged. Commands,
acknowledgements and the Air status carry the link ID too. Callsign identification packets are not keyed: anyone may
read who is transmitting.

## In-packet FEC

Every entry carries its own error correction: RS(253, 237) over GF(256), polynomial 0x11d, roots α^0..α^15, 16
parity bytes. A packet that failed the radio's CRC is not thrown away. The receiver repairs:

- up to 8 wrong bytes;
- or up to 16 bytes known to be bad, for example where two receivers' copies of the same packet differ.

It then keeps the packet only if the CRC-16 matches. The CRC-16 (polynomial 0x8005, reflected) differs from the
radio's own CRC-16 (CCITT 0x1021), so it also catches what the radio's misses, and it rejects wrong repairs.

## Block FEC across packets

On by default: 8 + 4. After every **block of k video packets**, **m parity packets** follow. Any k of the k + m packets
give back all k, so up to m lost packets per block are rebuilt.

- **The code:** systematic Cauchy Reed-Solomon over GF(256), polynomial 0x11d. Parity row j, column i is
  `1 / ((k + j) XOR i)`; it encodes the 231-byte symbols, the rest of each entry is built as for video.
- **Interleaving:** `d` blocks (1–8) can be interleaved. Consecutive video packets then go to different blocks (block
  i takes packets i, i+d, i+2d…), and the d blocks' parity follows the group. A fade of up to d × m packets in a row
  is then rebuilt too, at the cost of about d times the wait.
- **Parity entry:** type `0x02 | (d - 1) << 4`, `id16` the block's first video packet, `aux` `(k - 1) << 4 | j` (j =
  the parity index), symbol the parity bytes.
- **Short blocks:** a block still open when no video waits is closed early once it has waited k slots, with parity over
  the packets it has (a smaller k).
- **Limits:** k and m are 1–16 each. The video bitrate is scaled to k / (k + m) of the link.

## Callsign identification

Amateur rules ask a station to identify at least every 5 minutes. The callsign is kept as a **blob**:

```
blob[0]   bit 7: packed (1) or raw (0); bits 0..6: character count n (0 = no callsign)
packed:   A-Z = 1..26, 0-9 = 27..36, "/" 37, "-" 38, " " 39; three characters per 16-bit little-endian word
          (c0 × 1600 + c1 × 40 + c2): 1 + 2 × ceil(n / 3) bytes, e.g. "S57ABC" is 5 bytes
raw:      n bytes of UTF-8, for anything outside the packed alphabet: 1 + n bytes
```

It goes out in a **command slot**, so it costs no video. The identification packet is 16 bytes (about 2.2 ms on air),
sent 1 ms into the slot:

```
seq16 | 1D C5 | part << 4 | (parts - 1) | 10 blob bytes | xor(bytes 2..14) ^ 0x5A
```

A packed callsign of up to 12 characters fits in one packet. The transmitter identifies:

- 5 s after it starts;
- every 4 min 50 s while it transmits (plus the wait for a command slot, this stays under 5 minutes);
- right after a new callsign is set;
- once more when its service is stopped, after which it stays silent.

Without a callsign stored it identifies as the debugging default N0CALL. With a callsign cleared on purpose it does
not transmit at all.

## Air status

Idle packets (entry type 0xFF FF FF) carry the transmitter's state in bytes that are otherwise zero, so they cost no
airtime. When the picture stops, the receivers use it to say why: the camera does not start, the host stopped sending
video, the service was stopped. The layout is in `radio_firmware/src/common.h` ("Air status").
