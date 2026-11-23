"""Pi camera -> x265 -> over-the-air format v4 -> SPI or USB -> vlive_tx RP2350 (one fixed frequency, or hopping).

Sends the same over-the-air format as the Luckfox transmitter (luckfox_vtx/hz_vtx), so the same receivers work:
the Ground (receiver RP2350s on prebuilt/firmware/vlive_rx.uf2, single or diversity) with Horizon Receiver or the setup site.
  radio   SF5 / 1 MHz / long-interleaver 4/5, 255-byte packets in 17.54 ms slots (set by the vlive_tx firmware)
  packets every packet carries 231 bytes of video + CRC-16 + RS(253,237) (swtx.py; layout from the format reference
          swfec.py). Optional block FEC across packets (--fec K,M[,D], blockfec.py): the receiver rebuilds up to M
          lost packets per block of K; without it a lost packet costs the frames it carried.
  video   576x320 HEVC (x265, zero latency, one encode per frame), camera free running at --fps (30); no keyframes after
          the first: columns are coded intra in turn (x265 periodic intra refresh), the whole picture every --refresh
          frames (120 = 4 s); VPS/SPS/PPS in front of a frame once per --header-every s so a receiver can start late.
          --source FILE sends a test video instead of the camera (converted once, cached, looped).
  ident   --callsign: stored by the RP2350, sent every 5 minutes in a command slot; on SIGTERM / SIGINT (systemctl stop)
          the RP2350 identifies once more and then stops transmitting (end of transmission).
Stream (as the Luckfox): HEVC access units without parameter sets / AUD / SEI, each followed by an AUD, VPS/SPS/PPS
in-band when due.

Pi <-> TX RP2350 (--link): SPI on the 40-pin header (Link) or the RP2350's USB serial port (UsbLink); both carry the
same records and the same status frame. auto (default) uses SPI when the RP2350 answers there, else USB.

    python3 live_sender.py [--link auto|spi|usb] [--serial /dev/ttyACM0] [--tx-power -19] [--callsign S57ABC] [--stamp]
"""
import argparse
import math
import os
import signal
import struct
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import swtx  # noqa: E402
from sender import Camera, Encoder, FileSource, link_au  # noqa: E402

SLOT_S = 0.01754        # firmware SLOT_US (format v4); only the starting guess, the real slot is measured
SLOT_MARGIN_S = 0.0013  # air_s = slot - this: the feeder's timing reference (as the Luckfox's hz_link.h)
AIR_S = SLOT_S - SLOT_MARGIN_S
VPWR_MIN, VPWR_MAX = -19, 44  # TX power, 0.5 dBm units (-9.5 .. 22 dBm)

AUD = b"\x00\x00\x00\x01\x46\x01\x50"  # access unit delimiter (NAL 35)
STAMP_NAL = b"\x00\x00\x00\x01\x60\x01"   # unspecified NAL 48 + 8-byte wall-clock us (--stamp)


class Link:
    """The whole Pi <-> TX RP2350 link, over SPI only (no USB): video records go out on MOSI, and the
    RP2350 repeats a 32-byte status frame on MISO that every transfer also reads (queue depth, records
    accepted, counters, TX power, frequency, slot number). /dev/spidevX.Y, Pi = master, SPI mode 1, 8 bit, MSB first.
    Wiring (Pi GPIO -> RP2350 GPIO): CE0 8 -> 13, MOSI 10 -> 8, SCLK 11 -> 10, MISO 9 <- 11, plus GND.
    Uses the spidev ioctls directly, so no extra Python package is needed.
    Status frame: 0 C3  1 3C  2 queue depth  3 event counter  4..7 records accepted (u32)  8..17 sent fail late data
    idle (u16 each)  18 bad  19 TX power (half dBm, i8)  20..23 freq_hz  24..25 seq16  26 command slots
    27 command-slot IRQs  28 last command-slot flags (bit 7 flips after every identification)  29..30 arrival
    31 xor(2..30) ^ 0x5A"""

    SPI_IOC_WR_MODE = 0x40016B01
    SPI_IOC_WR_BITS_PER_WORD = 0x40016B03
    SPI_IOC_WR_MAX_SPEED_HZ = 0x40046B04
    SPI_IOC_MESSAGE_1 = 0x40206B00
    POLL_LEN = 72  # 8 stale prefetched bytes + 2 full status frames

    def __init__(self, dev, hz, start_poll=True):
        import ctypes
        import fcntl
        self._ctypes, self._fcntl = ctypes, fcntl
        self.hz = hz
        self.fd = self._open(dev)
        self.depth = None
        self.accepted = 0
        self.written = None
        self.last_t = ""
        self.events = []
        self.lock = threading.Lock()
        self.spi_lock = threading.Lock()
        self.prev = None
        self.last_stat_t = time.monotonic()
        self.evcount = None
        self.acc_change_t = time.monotonic()
        self.write_t = 0.0
        self.cmd = (0, 0, 0, 0)
        self.ident_toggle = None
        self.cmd_prev = None
        self.seq = None
        self.seq32 = 0          # seq unwrapped to a running slot count
        self.slot_t = None      # monotonic time at which the RP2350 was last seen starting a new slot
        self.slot_s = SLOT_S    # slot period: measured from the TX board's sequence counter once enough slots were seen
        self.air_s = AIR_S
        self.slot_known = False
        self._dts = []
        self._tl = threading.local()
        self.freq_hz = None
        self.pwr = None
        self.spi_errors = 0
        # link health (link_watch): transfers read, of them with a valid status frame, the last valid one, and the
        # first bytes of the newest transfer that had none
        self.dev = dev
        self.reads = self.frames_ok = self.no_frame = 0
        self.last_frame_t = time.monotonic()
        self.bad_raw = b""
        self.feed = {"sent": 0, "full": 0, "empty": 0, "blind": 0}   # feeder decisions (see feeder())
        if start_poll:
            threading.Thread(target=self._poll, daemon=True).start()

    def _open(self, dev):
        fd = os.open(dev, os.O_RDWR)
        self._fcntl.ioctl(fd, self.SPI_IOC_WR_MODE, struct.pack("B", 1))
        self._fcntl.ioctl(fd, self.SPI_IOC_WR_BITS_PER_WORD, struct.pack("B", 8))
        self._fcntl.ioctl(fd, self.SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("I", self.hz))
        return fd

    def _xfer(self, data):
        """Full-duplex transfer, CS low for its whole length. Returns the bytes clocked in."""
        ct = self._ctypes
        tx = ct.create_string_buffer(bytes(data), len(data))
        rx = ct.create_string_buffer(len(data))
        msg = struct.pack("=QQIIHBBBBBBB", ct.addressof(tx), ct.addressof(rx), len(data), self.hz, 0, 8, 0, 0, 0, 0, 0, 0)
        with self.spi_lock:
            self._tl.t = time.monotonic() + len(data) * 4.0 / self.hz  # middle of the transfer
            self._fcntl.ioctl(self.fd, self.SPI_IOC_MESSAGE_1, msg)
        return rx.raw

    def _poll(self):
        while not getattr(self, "closed", False):
            try:
                self._status(self._xfer(bytes(self.POLL_LEN)))
            except OSError as e:
                self.spi_errors += 1
                self.events.append("spi error: %s" % e)
                time.sleep(0.5)
            # Poll densely only around the expected start of the next slot (to time it to ~0.3 ms), otherwise rarely.
            d, st, now = 0.0005, self.slot_t, time.monotonic()
            if st is not None and now - st > 3 * self.slot_s + 0.05:   # the slots stopped / the firmware changed: learn the period again
                self.slot_known = False
                self._dts = []
            if st is not None and self.slot_known:   # (while the period is unknown, poll densely: a late change must not be missed)
                lead = st + self.slot_s * max(1, math.ceil((now - st) / self.slot_s - 0.05)) - now
                d = lead - 0.0015 if lead > 0.0025 else 0.0003
            time.sleep(d)

    def reopen(self):
        """Closes and reopens the SPI device and forgets the link state the status frames gave (stall recovery):
        records in flight, queue depth and the slot timing are learnt again from the next valid frame."""
        with self.spi_lock:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = self._open(self.dev)
        with self.lock:
            self.written = self.accepted
            self.depth = None
            self.slot_known = False
            self._dts = []

    def _status(self, rx):
        """Find the newest valid status frame in the bytes read and publish it."""
        self.reads += 1
        best = None
        for i in range(len(rx) - 31):
            if rx[i] == 0xC3 and rx[i + 1] == 0x3C:
                f = rx[i:i + 32]
                x = 0x5A
                for c in f[2:31]:
                    x ^= c
                if x == f[31]:
                    best = f
        if best is None:
            self.no_frame += 1
            self.bad_raw = bytes(rx[:40])
            return
        self.frames_ok += 1
        self.last_frame_t = time.monotonic()
        f = best
        depth, ev = f[2], f[3]
        acc = struct.unpack_from("<I", f, 4)[0]
        sent, fail, late, data, idle = struct.unpack_from("<5H", f, 8)
        bad = f[18]
        pwr = struct.unpack_from("<b", f, 19)[0]
        hz, seq = struct.unpack_from("<IH", f, 20)
        if not ((100000000 <= hz <= 1000000000 or 2400000000 <= hz <= 2483500000) and -39 <= pwr <= VPWR_MAX):  # corrupted frame that passed the xor (sub-GHz or the 2.4 GHz band)
            return
        with self.lock:
            if self.written is None or acc < self.accepted:  # first status or RP2350 restart
                self.written = acc
            if acc != self.accepted:
                self.acc_change_t = time.monotonic()
            self.depth, self.accepted = depth, acc
            if self.evcount is not None and ev != self.evcount:
                self.events.append("command / change event: freq_hz=%d pwr=%d" % (hz, pwr))
            self.evcount, self.freq_hz, self.pwr = ev, hz, pwr
            self.cmd = (f[26], f[27], f[28] & 0x7F, f[29] | f[30] << 8)  # command-slot diagnostics from the RP2350
            self.ident_toggle = f[28] >> 7
            if self.seq is not None and seq != self.seq:  # new slot: the RP2350 publishes seq when the packet starts
                t = getattr(self._tl, "t", time.monotonic())
                if self.slot_t is not None and (seq - self.seq) & 0xFFFF == 1 and 0.003 < t - self.slot_t < 1.0:
                    self._dts = (self._dts + [t - self.slot_t])[-64:]
                    if len(self._dts) >= 16:      # median of the last slots: robust against a late poll
                        m = sorted(self._dts)[len(self._dts) // 2]
                        near = [x for x in self._dts if abs(x - m) < 0.05 * m]   # drop the odd missed / doubled interval
                        m = sum(near) / len(near)
                        if abs(m - self.slot_s) > 0.0004:
                            self.slot_s, self.air_s = m, max(0.001, m - SLOT_MARGIN_S)
                        self.slot_known = True
                self.slot_t = t
            self.seq32 = self.seq32 + ((seq - self.seq) & 0xFFFF) if self.seq is not None else seq
            self.seq = seq
            now = time.monotonic()
            cur = (sent, fail, late, data, idle, bad)
            if self.prev is None:
                self.prev, self.last_stat_t = cur, now
            elif now - self.last_stat_t >= 5.0:
                d = [(a - b) & 0xFFFF for a, b in zip(cur[:5], self.prev[:5])] + [(cur[5] - self.prev[5]) & 0xFF]
                self.last_t = ("T seq=%d sent=%d fail=%d late=%d data=%d idle=%d bad=%d" % tuple([seq] + d))
                c, p = self.cmd, self.cmd_prev or self.cmd
                self.cmd_prev = c
                self.last_t += (" | cmd-slots %d heard %d last=%s at %.2f ms" %
                                ((c[0] - p[0]) & 255, (c[1] - p[1]) & 255, self._cmd_flags(c[2]), (c[3] * 16 - 2000) / 1000.0))
                self.prev, self.last_stat_t = cur, now

    @staticmethod
    def _cmd_flags(b):
        names = ((1, "rx_done"), (2, "crc_err"), (4, "len_err"), (8, "other_irq"), (16, "bad_len"), (32, "decoded"), (64, "not_a_command"))
        return "+".join(n for m, n in names if b & m) or "nothing"

    def pending(self):
        """Packets queued on the RP2350 or still in flight (None until the RP2350 reports)."""
        with self.lock:
            if self.depth is None:
                return None
            in_flight = self.written - self.accepted
            if in_flight > 64 or (in_flight > 0 and time.monotonic() - max(self.acc_change_t, self.write_t) > 0.04):
                # a record was lost on the wire (the RP2350 never counted it): resync rather than stall
                self.written = self.accepted
                in_flight = 0
            return self.depth + in_flight

    def snapshot(self):
        with self.lock:
            return self.slot_t, self.slot_s, self.air_s, self.slot_known, self.seq32

    def write(self, records, n):
        try:
            rx = self._xfer(records)
        except OSError as e:
            self.spi_errors += 1
            self.events.append("spi error: %s" % e)
            return
        self._status(rx)  # the same transfer also reads the status frame back
        with self.lock:
            self.written += n
            self.write_t = time.monotonic()

    def refresh(self):
        try:
            self._status(self._xfer(bytes(self.POLL_LEN)))
        except OSError as e:
            self.spi_errors += 1
            self.events.append("spi error: %s" % e)

    BOARDS = {1: "vlink", 2: "luckfox-carrier", 3: "video-tx"}

    def whoami(self, timeout=1.0):
        """Identity of the TX RP2350 (lora_vlink/src/main.c "Identity"): the record A5 1F 01 FE makes it show a 32-byte
        identity frame on MISO for 300 ms (0 C3 1 49 2 version 3 board 4 format 5..8 commit 9 dirty 10..17 chip id
        18..29 target without "vlive_" 31 xor). -> dict (board, board_id, fw, git, fmt, uid), or None (firmware older
        than 2026-11-20). Call before video starts (no status frame while the identity shows)."""
        t_end, t_send = time.monotonic() + timeout, 0.0
        while time.monotonic() < t_end:
            if time.monotonic() >= t_send:   # resend: a record right after the device is opened can be lost
                self._xfer(b"\xa5\x1f\x01\xfe\x00\x00\x00\x00")
                t_send = time.monotonic() + 0.1
            time.sleep(0.005)
            rx = self._xfer(bytes(self.POLL_LEN + 32))
            for i in range(len(rx) - 31):
                f = rx[i:i + 32]
                if f[0] != 0xC3 or f[1] != 0x49:
                    continue
                x = 0x5A
                for b in f[2:31]:
                    x ^= b
                if x != f[31]:
                    continue
                return {"board": self.BOARDS.get(f[3], "unknown"), "board_id": f[3],
                        "fw": "vlive_" + f[18:30].rstrip(b"\0").decode(errors="replace"),
                        "git": "%08x%s" % (int.from_bytes(f[5:9], "little"), "-dirty" if f[9] & 1 else ""),
                        "fmt": f[4], "uid": f[10:18].hex().upper()}
        return None

    def set_power(self, half_dbm):
        """TX-side power change: a 5-byte power record (A5 C7 pwr ~pwr pwr^5A) between video records. The RP2350
        applies it between two packets, reports it in the status frame and announces it over the air (downlink only),
        so the receivers show the new value. True once the status frame shows it (3 tries, ~3 s)."""
        if not VPWR_MIN <= half_dbm <= VPWR_MAX:
            return False
        p = half_dbm & 0xFF
        rec = bytes([0xA5, 0xC7, p, ~p & 0xFF, p ^ 0x5A, 0, 0, 0])
        for _ in range(3):
            try:
                self._status(self._xfer(rec))
            except OSError:
                self.spi_errors += 1
                continue
            t_end = time.monotonic() + 1.0
            while time.monotonic() < t_end:
                time.sleep(0.05)
                # on 2.4 GHz the RP2350 keeps its minimum and reports what the LR2021 puts out (-39 = -19.5 dBm)
                if self.pwr == half_dbm or (self.pwr == -39 and (self.freq_hz or 0) >= 1500000000):
                    return True
        return False


    def set_plan(self, hz, n=0, step_hz=0, timeout=5.0):
        """Frequency / hop plan for the RP2350 (record A5 1C hz[4] n step_khz[2] xor, as hz_vtx --freq/--hop): it
        announces the plan and switches with the receivers. n = 0: one fixed frequency. True once the status frame
        shows the new frequency."""
        k = step_hz // 1000
        rec = bytearray([0xA5, 0x1C]) + struct.pack("<IBH", hz, n, k)
        x = 0x5A
        for b in rec[2:9]:
            x ^= b
        rec = bytes(rec + bytes([x]))
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            self.send_record(rec)
            t_try = time.monotonic() + 1.5
            while time.monotonic() < t_try:
                time.sleep(0.05)
                self.refresh()
                if self.freq_hz == hz:
                    return True
        return False

    def send_record(self, rec):
        try:
            self._status(self._xfer(rec + bytes(4)))
        except OSError:
            self.spi_errors += 1

    def set_callsign(self, blob):
        """Callsign blob (swfec.encode_call) for the RP2350: it keeps it in flash and identifies every 5 minutes."""
        self.send_record(swtx.call_record(blob))

    def ident(self, mode, timeout=6.0):
        """Identify now (1), or identify and then stop transmitting (2). True once the status frame shows it was
        sent (byte 28 bit 7 flips)."""
        before = self.ident_toggle
        if before is None:
            return False
        t_end, t_send = time.monotonic() + timeout, 0.0
        while time.monotonic() < t_end:
            if time.monotonic() >= t_send:
                self.send_record(swtx.ident_record(mode))
                t_send = time.monotonic() + 1.5
            time.sleep(0.02)
            self.refresh()
            if self.ident_toggle is not None and self.ident_toggle != before:
                return True
        return False


class UsbLink(Link):
    """The same link over the TX RP2350's USB serial port (vlive_tx built 2026-11-18 or later): the same records go
    out as bytes on the serial port, and the RP2350 prints its status frame as "S <64 hex digits>" at every slot start
    (enabled with the text command "S 1"). USB adds ~1 ms of jitter to the arrival of the status lines, so the slot
    phase is taken from the earliest arrivals of a straight-line fit (slot number -> time) instead of the last line.
    Needs pyserial (python3-serial)."""

    def __init__(self, port):
        import serial
        Link.__init__(self, None, 0, start_poll=False)
        self.port = port
        self.ser = serial.Serial(port, 115200, timeout=0.2, write_timeout=0.5)
        self.ser.reset_input_buffer()
        self.ser.write(b"S 1\n")
        self._fit = []           # (seq32, arrival time) of recent status lines
        self._fit_n = 0
        self._model = None       # (t0, slot_s): slot n starts at t0 + n * slot_s (host clock)
        self._seq_u = None
        threading.Thread(target=self._reader, daemon=True).start()

    def _open(self, dev):
        return None

    def whoami(self, timeout=1.0):
        """Identity over USB: the text command "W" answers "W board=.. board_id=.. fw=.. git=.. fmt=.. uid=..";
        -> dict, or None (firmware older than 2026-11-20)."""
        self.who = None
        with self.spi_lock:
            self.ser.write(b"W\n")
        t_end = time.monotonic() + timeout
        while self.who is None and time.monotonic() < t_end:
            time.sleep(0.02)
        return self.who

    def _xfer(self, data):
        with self.spi_lock:
            self.ser.write(bytes(data))
        return b""

    def refresh(self):
        pass  # status lines stream in by themselves

    def _reader(self):
        import serial
        while True:
            try:
                line = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.spi_errors += 1
                self.events.append("usb error: %s" % e)
                time.sleep(0.5)
                continue
            t = time.monotonic()
            if line.startswith(b"S ") and len(line) >= 66:
                try:
                    f = bytes.fromhex(line[2:66].decode())
                except ValueError:
                    continue
                self._tl.t = self._phase(f, t)
                self._status(f)
            elif line.startswith(b"W "):
                try:
                    self.who = dict(x.split("=", 1) for x in line[2:].decode(errors="replace").split())
                except ValueError:
                    pass
            elif line.startswith((b"C ", b"T ", b"H ")):
                if line.startswith(b"C "):
                    self.events.append(line.decode(errors="replace").strip())
            elif line.startswith(b"Q "):   # status lines got switched off (RP2350 restarted): ask again
                try:
                    self.ser.write(b"S 1\n")
                except (serial.SerialException, OSError):
                    pass

    def _phase(self, f, t):
        """Arrival time -> slot start time on the host clock: fit seq32 -> time over the last ~6 s and shift the
        line down to the earliest arrivals (the ones USB delayed least)."""
        if f[0] != 0xC3 or f[1] != 0x3C:
            return t
        seq = f[24] | f[25] << 8
        self._seq_u = seq if self._seq_u is None else self._seq_u + ((seq - self._seq_u) & 0xFFFF)
        if self._fit and (self._fit[-1][0] >= self._seq_u or t - self._fit[-1][1] > 1.0):
            self._fit, self._model = [], None   # RP2350 restarted or the stream paused: start over
        self._fit.append((self._seq_u, t))
        if len(self._fit) > 540:
            del self._fit[:len(self._fit) - 540]
        self._fit_n += 1
        if len(self._fit) >= 32 and (self._model is None or self._fit_n % 16 == 0):
            a = np.array(self._fit, dtype=np.float64)
            n0 = a[0, 0]
            x, y = a[:, 0] - n0, a[:, 1]
            slope, icpt = np.polyfit(x, y, 1)
            r = y - (slope * x + icpt)
            off = np.percentile(r, 5)   # earliest arrivals (robust against one odd early line)
            self._model = (icpt + off - slope * n0, slope)
        if self._model is None:
            return t
        return self._model[0] + self._model[1] * self._seq_u


def open_link(args):
    """--link spi | usb | auto (SPI if the RP2350 answers there, else the first Pico serial port that sends status
    lines and that no other program has open)."""
    if args.link in ("spi", "auto") and not args.serial:
        try:
            link = Link(args.spi_dev, args.spi_hz)
        except OSError as e:
            if args.link == "spi":
                sys.exit("cannot open %s: %s (enable SPI: sudo raspi-config nonint do_spi 0, then reboot)" % (args.spi_dev, e))
            link = None
        if link is not None:
            t_end = time.monotonic() + (5 if args.link == "spi" else 1.5)
            while link.pending() is None and time.monotonic() < t_end:
                time.sleep(0.05)
            if link.pending() is not None or args.link == "spi":
                return link, "SPI %s" % args.spi_dev
            link.closed = True
            time.sleep(0.6)
            os.close(link.fd)
    ports = [args.serial] if args.serial and args.serial != "auto" else find_tx_ports()
    for port in ports:
        if port.startswith("sn:"):
            port = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_%s-if00" % port[3:]
        try:
            link = UsbLink(port)
        except Exception as e:  # noqa: BLE001  (missing pyserial, port gone, permission)
            print("live_sender: %s: %s" % (port, e), flush=True)
            continue
        t_end = time.monotonic() + (5 if args.serial else 1.0)
        while link.pending() is None and time.monotonic() < t_end:
            time.sleep(0.05)
        if link.pending() is not None:
            return link, "USB %s" % port
        link.ser.close()
    return None, None


def find_tx_ports():
    """Pico serial ports that no other process has open (a receiver used by a receiver app is skipped untouched)."""
    import glob
    ports = sorted(glob.glob("/dev/serial/by-id/usb-Raspberry_Pi_Pico_*")) or sorted(glob.glob("/dev/ttyACM*"))
    busy = set()
    for fd in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            busy.add(os.path.realpath(fd))
        except OSError:
            pass
    return [p for p in ports if os.path.realpath(p) not in busy]


def feeder(link, sched, low, lead_s, stop, tap=None):
    """Builds each record just before the RP2350 takes it (lead_s before the end of the current packet), from everything
    queued by then. Same timing logic as the Luckfox's hz_feeder_thread (luckfox_vtx/hz_link.h)."""
    done = None
    trace = bool(os.environ.get("HZ_TRACE_FEED"))

    def hand(rec):
        if tap:
            tap(rec)
        link.write(rec, 1)

    while not stop.is_set():
        now = time.monotonic()
        st, slot_s, air_s, known, seq32 = link.snapshot()
        if st is None or now - st > 0.5:  # slot phase unknown: hand over as soon as the queue is empty
            p = link.pending()
            rec = sched.next_record() if (p is not None and p <= low) else None
            if rec is None:
                time.sleep(0.0005)
            else:
                link.feed["blind"] += 1
                hand(rec)
            continue
        n = int((now - st) / slot_s)
        w = st + n * slot_s + air_s - lead_s
        if (st, n) == done or now < w - 0.0006:
            time.sleep(max(0.0003, min(w - now - 0.0005, 0.004)) if (st, n) != done else 0.001)
            continue
        link.refresh()
        done = (st, n)
        p = link.pending()
        if p is None or p > low:
            link.feed["full"] += 1
            if trace:
                print("feed %.4f SKIP pending=%s" % (time.monotonic(), p), file=sys.stderr)
            continue
        rec = sched.next_record()
        if rec is not None:
            link.feed["sent"] += 1
            hand(rec)
        else:
            link.feed["empty"] += 1
        if trace:
            print("feed %.4f %s" % (time.monotonic(), "-" if rec is None else "S"), file=sys.stderr)


STALL_S = 2.0        # video waits but the RP2350 has taken no record for this long: a stall
STALL_REOPEN_S = 3.0  # then: reopen SPI and relearn the link state
STALL_EXIT_S = 10.0   # still stalled: exit (systemd restarts the sender)


def link_watch(link, sched, stop):
    """Once a second, independent of the camera and the encoder: is the link moving? A stall (video waiting, the
    RP2350's accepted-record counter frozen) is logged with the link state (EVENT spi_stall), recovered by reopening SPI
    after STALL_REOPEN_S and, if that does not help, by exiting after STALL_EXIT_S (FATAL; systemd restarts the
    sender). Every 5 s a "link:" line, also while frames are skipped."""
    last_acc, progress_t = link.accepted, time.monotonic()
    stall_t, reopened = None, False
    prev = (link.reads, link.frames_ok, link.no_frame, dict(link.feed), link.accepted, link.spi_errors)
    rep_t = time.monotonic()

    def state():
        age = time.monotonic() - link.last_frame_t
        return ("depth=%s accepted=%s written=%s pending=%s queued=%d slot_known=%s seq=%s frame_age=%.2fs "
                "spi_errors=%d feed=%s" % (link.depth, link.accepted, link.written, link.pending(), sched.queued_packets(),
                                           link.slot_known, link.seq, age, link.spi_errors, link.feed))

    while not stop.wait(1.0):
        now = time.monotonic()
        acc = link.accepted
        if acc != last_acc:
            last_acc, progress_t = acc, now
            if stall_t is not None:
                print("EVENT spi_recovered after %.1fs%s" % (now - stall_t, " (SPI reopened)" if reopened else ""), flush=True)
                stall_t, reopened = None, False
        elif sched.queued_packets() > 0 and now - progress_t > STALL_S:
            if stall_t is None:
                stall_t = now
                print("EVENT spi_stall no record taken for %.1fs: %s last_bad_read=%s" % (now - progress_t, state(),
                                                                                         link.bad_raw.hex()), flush=True)
            if not reopened and now - stall_t >= STALL_REOPEN_S and getattr(link, "dev", None):  # (SPI only)
                reopened = True
                try:
                    link.reopen()
                    print("EVENT spi_reopen: %s" % state(), flush=True)
                except OSError as e:
                    print("ERROR spi_reopen failed: %s" % e, flush=True)
            if now - stall_t >= STALL_EXIT_S:
                print("FATAL spi_stall for %.0fs, not recovered by reopening SPI: %s last_bad_read=%s" % (
                    now - stall_t, state(), link.bad_raw.hex()), flush=True)
                os._exit(4)
        if now - rep_t >= 5.0:
            cur = (link.reads, link.frames_ok, link.no_frame, dict(link.feed), link.accepted, link.spi_errors)
            dt = now - rep_t
            feed = {k: cur[3][k] - prev[3].get(k, 0) for k in cur[3]}
            print("link: reads %.0f/s, status frames ok %d, none %d, records taken %d, feed sent %d full %d empty %d "
                  "blind %d, spi errors %d, depth=%s frame_age=%.2fs" % (
                      (cur[0] - prev[0]) / dt, cur[1] - prev[1], cur[2] - prev[2], cur[4] - prev[4], feed["sent"],
                      feed["full"], feed["empty"], feed["blind"], cur[5] - prev[5], link.depth,
                      now - link.last_frame_t), flush=True)
            prev, rep_t = cur, now


def control(path, link, stop):
    """Control FIFO (as the Luckfox's --ctl): "pwr <half_dbm>" lines change the TX power; PATH.pwr always holds the
    RP2350's current power (tx_power.sh reads it)."""
    try:
        if os.path.exists(path):
            os.unlink(path)
        os.mkfifo(path, 0o660)
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)  # RDWR: never sees EOF when a writer closes
    except OSError as e:
        print("live_sender: WARNING control FIFO %s unavailable (%s); TX power only via --tx-power / the receiver" % (path, e),
              flush=True)
        return
    import select
    buf, shown = b"", None
    while not stop.is_set():
        pwr = link.pwr
        if pwr is not None and pwr != shown:
            try:
                with open(path + ".pwr.tmp", "w") as f:
                    f.write("%d\n" % pwr)
                os.replace(path + ".pwr.tmp", path + ".pwr")
                shown = pwr
            except OSError:
                pass
        if not select.select([fd], [], [], 0.2)[0]:
            continue
        try:
            buf += os.read(fd, 128)
        except BlockingIOError:
            continue
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            words = line.decode(errors="replace").split()
            if len(words) == 2 and words[0] == "pwr" and words[1].lstrip("-").isdigit():
                v = int(words[1])
                if not VPWR_MIN <= v <= VPWR_MAX:
                    print("live_sender: control: TX power %d out of range (-19..44, 0.5 dBm units)" % v, flush=True)
                elif link.set_power(v):
                    print("live_sender: control: TX power set to %d (%.1f dBm)" % (v, v / 2.0), flush=True)
                else:
                    print("live_sender: control: TX power %d NOT confirmed by the RP2350" % v, flush=True)
            elif words:
                print("live_sender: control: unknown command %r (use: pwr <half_dbm>)" % line, flush=True)


def main(argv=None, link=None):
    ap = argparse.ArgumentParser(description="Horizon video link transmitter for Raspberry Pi (over-the-air format v4)")
    ap.add_argument("--link", default="auto", choices=("auto", "spi", "usb"),
                    help="how the Pi talks to the TX RP2350: spi (40-pin header), usb (its USB serial port), auto "
                         "(default: SPI if the RP2350 answers there, else USB)")
    ap.add_argument("--serial", default=None,
                    help="USB serial port of the TX RP2350 (/dev/ttyACM0, /dev/serial/by-id/..., or sn:<serial no.>); "
                         "implies --link usb. Default with --link usb/auto: the first free Pico port that answers")
    ap.add_argument("--spi-dev", default="/dev/spidev0.0", help="SPI device to the RP2350")
    ap.add_argument("--spi-hz", type=int, default=4000000, help="SPI clock (the RP2350 slave allows up to 12.5 MHz)")
    # 576x320: the highest VMAF on the test clip at this bitrate (vmaf_lab sweeps; 288x160 scored ~half)
    ap.add_argument("--width", type=int, default=576)
    ap.add_argument("--height", type=int, default=320)
    ap.add_argument("--fps", type=float, default=30.0, help="camera / video frame rate (free running, default 30)")
    ap.add_argument("--preset", default="superfast")
    ap.add_argument("--refresh", type=int, default=120,
                    help="intra refresh period in frames: the whole picture is repainted column by column every N "
                         "frames (120 = 4 s); there are no keyframes after the first. Longer: sharper on a clean "
                         "link, slower to heal after a loss")
    ap.add_argument("--util", type=float, default=0.9,
                    help="video bitrate as a fraction of the link's video capacity (231 B per slot)")
    ap.add_argument("--kbps", type=float, default=0, help="video bitrate in kbit/s (overrides --util)")
    ap.add_argument("--max-queue", type=int, default=4,
                    help="skip a camera frame when more than this many video packets are still queued")
    ap.add_argument("--header-every", type=float, default=1.0, help="seconds between in-band VPS/SPS/PPS")
    ap.add_argument("--low", type=int, default=0,
                    help="build the next packet when <= this many wait on the RP2350 (0 = lowest latency)")
    ap.add_argument("--lead-ms", type=float, default=4.0,
                    help="build each record this long before the RP2350 takes it (lower = fresher, riskier)")
    ap.add_argument("--tx-power", type=int, default=None,
                    help="TX power at start, 0.5 dBm units: -19 (-9.5 dBm) .. 44 (22 dBm); on 2.4 GHz the RP2350 always uses its minimum. Default: keep the "
                         "RP2350's (vlive_tx boots at 22 dBm)")
    ap.add_argument("--freq", type=int, default=None,
                    help="frequency (or hop centre) in Hz from the start, e.g. 868000000 or 2442167000 (150..960 MHz, "
                         "or the 2.4 GHz band 2400..2483.5 MHz); default: keep the RP2350's")
    ap.add_argument("--hop", default=None,
                    help="with --freq: N,SPACING_HZ hop over N channels (2..128) SPACING_HZ apart, one packet on each "
                         "(a third number, the old slots per channel, is ignored)")
    ap.add_argument("--callsign", default=None,
                    help="station identification (callsign or any text, up to 32 characters): stored by the RP2350 "
                         "and sent every 5 minutes; default: keep the RP2350's (set from the receiver)")
    ap.add_argument("--ctl", default=os.environ.get("LORA_TX_CTL") or
                    ("/run/lora-vlink-tx/ctl" if os.access("/run/lora-vlink-tx", os.W_OK) else "/tmp/lora-vlink-tx.ctl"),
                    help="control FIFO for TX power changes while running (tx_power.sh; default /run/lora-vlink-tx/ctl "
                         "under the service, else /tmp/lora-vlink-tx.ctl); '' = none")
    ap.add_argument("--x265-extra", default="", help="more x265 overrides, name=value:name=value")
    ap.add_argument("--fec", default="8,4",
                    help="block FEC K,M[,D] (default 8,4; e.g. 16,3 or 12,3,2; off = none): M parity packets for every block of K video packets; the "
                         "receiver rebuilds up to M lost packets per block. D > 1 interleaves D blocks, so a fade of "
                         "up to D*M packets in a row is rebuilt too (recovery waits ~D times longer). The video "
                         "bitrate is scaled to K/(K+M) of the link")
    ap.add_argument("--stamp", action="store_true",
                    help="add a wall-clock stamp per frame so the receiver can show the link latency")
    ap.add_argument("--source", default=None,
                    help="test video instead of the camera: any file ffmpeg reads (converted once to the encode size and frame rate, centre-cropped, cached in ~/.cache/lora-vlink) or a raw I420 .yuv; looped")
    ap.add_argument("--encoder", default=os.path.join(HERE, "frame_enc"))
    ap.add_argument("--duration", type=float, default=0, help=argparse.SUPPRESS)  # tests: stop after N s
    # older options (old service files): accepted and ignored
    for o in ("--k", "--m", "--keyint"):
        ap.add_argument(o, type=int, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--pack", default=None, help=argparse.SUPPRESS)
    ap.add_argument("--no-refresh", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    if args.k is not None or args.m is not None or args.pack or args.no_refresh or args.keyint is not None:
        print("live_sender: --k / --m / --pack / --keyint / --no-refresh are older-format options and are ignored "
              "(format v4: no cross-packet FEC, no keyframes)", flush=True)
    stop = threading.Event()
    sig = {"stop": False}

    # A worker thread that dies (feeder, SPI poll, encoder drain) would leave a sender that runs but sends nothing:
    # end the whole process instead, so systemd restarts it and the log says why.
    def thread_died(a):
        print("FATAL thread %s died: %s: %s" % (a.thread.name if a.thread else "?", a.exc_type.__name__, a.exc_value),
              flush=True)
        threading.__excepthook__(a)
        os._exit(3)
    threading.excepthook = thread_died

    def on_signal(*_):
        sig["stop"] = True
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    if link is None:
        link, where = open_link(args)
        if link is None:
            sys.exit("no TX RP2350 found: over SPI (%s) it must run vlive_tx.uf2, be powered and wired "
                     "(CE0/MOSI/SCLK/MISO/GND); over USB it must run the vlive_tx.uf2 of 2026-11-18 or later "
                     "(--link usb --serial /dev/ttyACM0 to name the port)" % args.spi_dev)
    else:
        where = "test link"
        t_end = time.monotonic() + 5
        while link.pending() is None:
            if time.monotonic() > t_end:
                sys.exit("no status from the RP2350")
            time.sleep(0.05)
    t_slot = time.monotonic() + 3.0
    while not link.slot_known and time.monotonic() < t_slot:   # learn the slot length the firmware really uses
        time.sleep(0.05)
    slot_s = link.slot_s
    if slot_s < 0.015:
        print("live_sender: WARNING the RP2350 uses %.1f ms slots: that is not the format v4 vlive_tx.uf2 (17.5 ms); "
              "flash prebuilt/firmware/vlive_tx.uf2, receivers will not decode this" % (slot_s * 1000), flush=True)
    who = link.whoami()   # which board and firmware this sender drives (for support and updates)
    if who:
        print("live_sender: RP2350 board=%s (%s) fw=%s git=%s format=%s uid=%s" % (
            who.get("board"), who.get("board_id"), who.get("fw"), who.get("git"), who.get("fmt"), who.get("uid")),
            flush=True)
        if str(who.get("fmt")) not in ("4", "5"):   # v5 = v4 + the link ID keyed in by the RP2350 (same host entries)
            print("live_sender: WARNING the RP2350 firmware uses over-the-air format %s, this sender formats 4-5"
                  % who.get("fmt"), flush=True)
    else:
        print("live_sender: RP2350 identity unknown (firmware older than 2026-11-20 does not report one)", flush=True)
    if args.callsign:
        blob = swtx.S.encode_call(args.callsign)
        link.set_callsign(blob)
        print("live_sender: callsign %r (%d bytes on air) given to the RP2350" % (args.callsign, len(blob)), flush=True)
    if args.freq:   # the paired plan: the RP2350 announces it and switches with the receivers
        n = step = 0
        try:
            if args.hop:
                n, step = (int(x) for x in args.hop.split(",")[:2])
            assert 150000000 <= args.freq <= 960000000 or 2400000000 <= args.freq <= 2483500000
            assert n == 0 or (2 <= n <= 128 and step > 0 and step % 1000 == 0)
        except (ValueError, AssertionError):
            print("FATAL bad --freq/--hop %r %r" % (args.freq, args.hop), flush=True)
            return 2
        if link.set_plan(args.freq, n, step):
            print("live_sender: plan freq_hz=%d hop=%d,%d in use" % (args.freq, n, step), flush=True)
        else:
            print("live_sender: WARNING plan freq_hz=%d not confirmed by the RP2350" % args.freq, flush=True)
    if args.tx_power is not None:
        ok = link.set_power(args.tx_power)
        print("live_sender: TX power %d (%.1f dBm) %s" % (args.tx_power, args.tx_power / 2.0,
                                                           "set" if ok else "NOT confirmed by the RP2350"), flush=True)
    if args.ctl:
        threading.Thread(target=control, args=(args.ctl, link, stop), daemon=True).start()

    fps = args.fps
    cap_bps = swtx.S.SYM * 8 / slot_s * 63 / 64
    fec = None
    if args.fec and args.fec not in ("0", "off"):
        try:
            fec = tuple(int(x) for x in args.fec.split(","))
            fec = fec + (1,) * (3 - len(fec))
            assert len(fec) == 3 and 1 <= fec[0] <= 16 and 1 <= fec[1] <= 16 and 1 <= fec[2] <= 8
        except (ValueError, AssertionError):
            print("FATAL bad --fec %r (K,M[,D] with 1 <= K, M <= 16, 1 <= D <= 8)" % args.fec, flush=True)
            return 2
        cap_bps = cap_bps * fec[0] / (fec[0] + fec[1])   # the slots left for video
        print("live_sender: block FEC %d+%d%s" % (fec[0], fec[1], ", interleaved x%d" % fec[2] if fec[2] > 1 else ""),
              flush=True)
    kbps = args.kbps or int(args.util * cap_bps / 1000)
    print("live_sender: format v4, %s, slot %.2f ms%s, video packets %.1f kbit/s, video %d kbit/s, %dx%d %.1f fps, "
          "refresh %d frames, TX %s" %
          (where, slot_s * 1000, "" if link.slot_known else " (assumed)", cap_bps / 1000, kbps,
           args.width, args.height, fps, args.refresh,
           "LR2021 %.1f dBm" % (link.pwr / 2.0) if link.pwr is not None else "power unknown"), flush=True)

    # Rate buffer 0.3 s of video. Gradual refresh that really heals (instead of keyframes): with constrained-intra,
    # temporal-mvp=0, ref=1 and no-deblock a picture damaged by a lost frame is bit-exact again after the next refresh
    # sweep (measured), as the Luckfox's hardware refresh.
    # rdoq-level=2 + psy-rdoq / psy-rd 1 + early-skip off + TU depth 2: +3-4 VMAF over plain superfast at this
    # bitrate for ~7 ms more encode time on a Pi 5 at 576x320 (vmaf_lab sweeps; slower presets lose: they turn SAO on)
    x265 = ("bitrate=%d:vbv-maxrate=%d:vbv-bufsize=%d:vbv-init=0.5:ctu=32:constrained-intra:temporal-mvp=0:ref=1:"
            "no-deblock:rdoq-level=2:psy-rdoq=1:psy-rd=1:early-skip=0:tu-intra-depth=2:tu-inter-depth=2"
            % (kbps, kbps, max(3, kbps * 3 // 10)))
    if args.x265_extra:
        x265 += ":" + args.x265_extra
    enc = Encoder(args.encoder, args.width, args.height, fps, args.preset, args.refresh, 1, x265)
    cam = FileSource(args.source, args.width, args.height, fps) if args.source else Camera(args.width, args.height, fps)
    frame_target = cap_bps / 8 / fps

    sched = swtx.Scheduler(fec=fec)
    feed_stop = threading.Event()
    th_feed = threading.Thread(target=feeder, args=(link, sched, args.low, args.lead_ms / 1000.0, feed_stop,
                                                     getattr(link, "tap", None)), daemon=True)
    th_feed.start()
    if hasattr(link, "frames_ok"):   # (a test's stand-in link has no counters)
        threading.Thread(target=link_watch, args=(link, sched, feed_stop), name="link_watch", daemon=True).start()

    period = 1.0 / fps
    next_t = time.monotonic()
    last_hdr = -1e9
    frames = skipped = 0
    fbytes = 0
    ages, encs, s2p = [], [], []
    t_start = last_rep = time.monotonic()
    seen = 0
    while not sig["stop"] and not (args.duration and time.monotonic() - t_start > args.duration):
        if args.source is None:
            frame, cstamp, seen = cam.wait_new(seen)   # the camera runs at --fps: each frame is encoded on arrival
            if frame is None:
                continue
        else:
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
            frame, cstamp = cam.grab()
            if frame is None:
                continue
        sensor_t = getattr(cam, "sensor_t", 0.0) or cstamp  # sensor timestamp (as the Luckfox's PTS); file: arrival
        ages.append((time.monotonic() - sensor_t) * 1000.0)  # how old the camera frame is when the encoder takes it
        if sched.queued_packets() > args.max_queue:  # link fell behind: skip a frame rather than add delay
            skipped += 1
            continue
        te = time.monotonic()
        try:
            au, qp, _, _ = enc.encode(frame, frame_target)
        except (RuntimeError, OSError, ValueError):
            if sig["stop"]:   # systemd stops the encoder with us: not an error
                break
            raise
        encs.append((time.monotonic() - te) * 1000.0)
        body = link_au(au)
        now = time.monotonic()
        if now - last_hdr >= args.header_every:
            body = enc.headers + body
            last_hdr = now
        if args.stamp:
            body += STAMP_NAL + struct.pack("<Q", int(time.time() * 1e6))
        body += AUD
        sched.push(body)
        s2p.append((time.monotonic() - sensor_t) * 1000.0)
        if hasattr(link, "on_push"):
            link.on_push(body, cam.frames - 1)
        frames += 1
        fbytes += len(body)
        if now - last_rep >= 5:
            dt = now - last_rep
            print("tx: %.1f fps, %.1f kbit/s video, %d B/frame avg, packets %d, pad %.0f%%, backlog %d B, skipped %d | %s" %
                  (frames / dt, sched.bytes * 8 / dt / 1000, fbytes / max(1, frames), sched.shards,
                   100.0 * sched.pad / max(1, sched.bytes + sched.pad), sched.backlog(), skipped, link.last_t),
                  flush=True)
            qd = sched.qdelay
            print("  latency (ms, mean/max): camera frame age %.0f/%.0f (from the sensor timestamp), encode %.1f/%.1f, "
                  "sensor to queued %.0f/%.0f, wait in sender queue %.0f/%.0f; radio adds ~%.0f slot wait + %.1f air" %
                  (np.mean(ages or [0]), max(ages or [0]), np.mean(encs or [0]), max(encs or [0]),
                   np.mean(s2p or [0]), max(s2p or [0]), np.mean(qd or [0]), max(qd or [0]),
                   link.slot_s * 500, link.slot_s * 1000 - 0.33), flush=True)
            print("  RP2350: freq_hz=%s pwr=%s (half dBm), link errors=%d" % (link.freq_hz, link.pwr, link.spi_errors),
                  flush=True)
            for e in link.events[-3:]:
                print("  RP2350:", e, flush=True)
            link.events.clear()
            ages.clear(); encs.clear(); qd.clear(); s2p.clear()
            sched.bytes = sched.pad = sched.shards = 0
            frames = skipped = fbytes = 0
            last_rep = now
    # let the queued frame go out, then stop feeding
    t_end = time.monotonic() + (1.0 if sig["stop"] else 10.0)
    while sched.backlog() and time.monotonic() < t_end:
        time.sleep(0.02)
    time.sleep(0.1)
    feed_stop.set()
    th_feed.join(timeout=1)
    if sig["stop"]:
        # stopped on request (systemctl stop / SIGTERM / Ctrl-C): end of transmission. The RP2350 identifies once more
        # with its callsign and then stops transmitting until video records arrive again. Not after a crash.
        print("live_sender: stop requested: final identification, then the transmitter goes silent", flush=True)
        if link.ident(2):
            print("live_sender: final identification sent, transmitter silent", flush=True)
        else:
            print("live_sender: WARNING final identification not confirmed by the RP2350 (old firmware?)", flush=True)
    stop.set()


if __name__ == "__main__":
    main()
