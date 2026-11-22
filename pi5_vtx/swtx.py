"""HORIZON over-the-air format v4, transmit side for the Raspberry Pi sender (live_sender.py).

Python port of the Luckfox transmitter's packet scheduler (luckfox_vtx/hz_sched.h + hz_swfec.h); the packet layout,
CRC-16 and Reed-Solomon parity come from the format's reference module (swfec.py), which the receivers are checked
against. test_swtx.py checks the records byte for byte against the C scheduler.

The video byte stream (HEVC access units, each followed by an AUD) is cut into 231-byte source symbols, one per radio
packet, with optional block FEC parity packets in between (Scheduler(fec=...), blockfec.py). One record is built at a
time, just before the RP2350 needs it:
    A5 5A | entry[253] | xor8(entry)          entry = type | id16 | aux | symbol[231] | CRC-16 | RS parity[16]
A source takes up to 231 waiting bytes, across a frame boundary when the next frame is already queued; only when the
queue runs dry is a packet's tail padded with zeros (a slot that would otherwise carry the RP2350's filler).
"""
import collections
import os
import sys
import threading
import time

import numpy as np

import swfec as S  # format v4 layout, CRC-16, RS, callsign blob

CMD_PERIOD = 64          # the RP2350's command slot: seq16 % 64 == 0 (lora_vlink/src/main.c CMD_PERIOD)
REC_LEN = 2 + S.ENTRY + 1


def is_cmd_slot(slot):
    return (slot & 0xFFFF) % CMD_PERIOD == 0


def make_record(entry):
    x = int(np.bitwise_xor.reduce(np.frombuffer(entry, dtype=np.uint8)))
    return b"\xa5\x5a" + bytes(entry) + bytes([x])


def call_record(blob):
    """Host record that hands the callsign blob to the RP2350 (A5 1D n blob xor)."""
    x = 0x5A ^ len(blob)
    for b in blob:
        x ^= b
    return bytes([0xA5, 0x1D, len(blob)]) + bytes(blob) + bytes([x])


def ident_record(mode):
    """Host record: identify now (1), or identify and then stop transmitting = end of transmission (2)."""
    return bytes([0xA5, 0x1E, mode, ~mode & 0xFF])


class Scheduler:
    """Thread-safe: push() from the encoder loop, next_record() from the feeder.

    fec=(k, m) or (k, m, d): optional block FEC (blockfec.py): m parity packets for every block of k video packets.
    d > 1 interleaves d blocks: consecutive video packets go to different blocks (block i of a group takes packets
    i, i+d, i+2d, ...), so a fade of up to d*m packets costs every block at most m. The d blocks' parity follows the
    group. A group still open when no video waits is closed early (parity over what it has) once it has waited a
    block's worth of slots, so a recovery does not wait long for idle time."""

    def __init__(self, fec=None):
        self.lock = threading.Lock()
        self.fifo = bytearray()
        self.marks = collections.deque()   # (end offset of a frame in the byte stream, push time)
        self.pushed = self.consumed = 0
        self.next_src = 0
        if fec is not None:
            fec = tuple(fec) + (1,) * (3 - len(fec))
            if not (1 <= fec[0] <= 16 and 1 <= fec[1] <= 16 and 1 <= fec[2] <= 8):
                raise ValueError("fec: k and m must be 1..16, d 1..8")
        self.fec = fec
        self.fec_syms = []                 # per interleaved block of the open group: its video symbols
        self.fec_first = 0                 # id of the group's first video packet
        self.fec_n = 0                     # video packets in the open group
        self.fec_age = 0                   # slots since the open group began
        self.fec_q = collections.deque()   # parity records waiting for slots
        # statistics (reset by the reporter)
        self.bytes = self.pad = self.shards = self.parity = 0
        self.qdelay = []                   # ms from push until the frame's last byte left in a record

    # ---- encoder side
    def push(self, b):
        with self.lock:
            self.fifo += b
            self.pushed += len(b)
            self.marks.append((self.pushed, time.monotonic()))

    def backlog(self):
        with self.lock:
            return len(self.fifo)

    def queued_packets(self):
        with self.lock:
            return (len(self.fifo) + S.SYM - 1) // S.SYM

    # ---- feeder side
    def next_record(self):
        """The next record, or None when no video waits (the RP2350 then sends its filler)."""
        with self.lock:
            if self.fec_q:                 # parity of the last block goes first
                self.parity += 1
                return self.fec_q.popleft()
            take = min(S.SYM, len(self.fifo))
            if self.fec and self.fec_n:
                self.fec_age += 1                # slots (calls) since the open group began
            if take == 0:
                # no video waits: close the open group only once it has waited a block's worth of slots (closing at
                # every short gap between frames would send parity for nearly empty blocks)
                if self.fec and self.fec_n and self.fec_age >= self.fec[0]:
                    self._close_block()
                    self.parity += 1
                    return self.fec_q.popleft()
                return None
            sid = self.next_src
            self.next_src += 1
            sym = bytes(self.fifo[:take]) + bytes(S.SYM - take)
            del self.fifo[:take]
            self.consumed += take
            self.pad += S.SYM - take
            now = time.monotonic()
            while self.marks and self.marks[0][0] <= self.consumed:
                self.qdelay.append((now - self.marks.popleft()[1]) * 1000.0)
            self.bytes += take
            self.shards += 1
            if self.fec:
                k, m, d = self.fec
                if not self.fec_n:
                    self.fec_first = sid
                    self.fec_syms = [[] for _ in range(d)]
                    self.fec_age = 0
                self.fec_syms[self.fec_n % d].append(sym)
                self.fec_n += 1
                if self.fec_n == k * d:
                    self._close_block()
        return make_record(S.make_entry(0, sid & 0xFFFF, 0, sym))

    def _close_block(self):
        """Queue the parity records of the open group's d blocks (called with the lock held). Parity entry: type
        PARITY | (d - 1) << 4, id16 = the block's first video packet, aux = (k - 1) << 4 | j."""
        import blockfec
        _, m, d = self.fec
        for i, syms in enumerate(self.fec_syms):
            if not syms:
                continue
            k = len(syms)
            for j, par in enumerate(blockfec.encode(syms, m)):
                self.fec_q.append(make_record(S.make_entry(S.PARITY | ((d - 1) << 4), (self.fec_first + i) & 0xFFFF,
                                                           ((k - 1) << 4) | j, par)))
        self.fec_syms = []
        self.fec_n = 0
