"""The plugged-in Horizon boards (each USB serial port asked who it is, "W"), and checks of them.

    python test/device.py list           DEVICE PORT IDENTITY per board ("-" for a port that gave no identity)
    python test/device.py check DEVICE   air | pi-hat | ground-unit | hat-rx
    python test/device.py pair           factory test: a Horizon Dev Air and a Ground (Ground Unit or HAT receiver)

check: the board names itself as DEVICE, its radio started, its link settings can be read; the Air also runs its
self-check (no part fails) and the SPI link test to its radio controller (the transmitter stops for about 10 s); a
receiver reports its status (locked or not: an Air need not be on).
pair: both checks, then the two use the same link settings (else: Pair on the setup site) and the Ground receives the
Air for 10 s: locked, at least 90 % of the slots, no CRC errors. Air powered with camera and antenna, the Ground with
its antenna, a short distance apart; to test one board, use a known good one as the other.
Prints EVENT / ERROR lines, RESULT PASS|FAIL. Needs pyserial.
"""
import re
import sys
import time

import serial
import serial.tools.list_ports

AIR_VID, GROUND_VID = 0x2207, 0x2E8A
SLOT_S = 0.01754  # one slot of the radio link
errors = []


def error(msg):
    errors.append(msg)
    print(f"ERROR {msg}", flush=True)


def kv(line):
    return dict(x.split("=", 1) for x in (line or "").split()[1:] if "=" in x)


class Board:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.2)
        self.s.dtr = True  # the Ground sends only while DTR is set
        self.rest = b""

    def lines(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rest += self.s.read(4096)
            while b"\n" in self.rest:
                line, self.rest = self.rest.split(b"\n", 1)
                yield line.decode(errors="replace").strip()

    def ask(self, cmd, until, seconds=5.0, collect=None):
        """Sends cmd; (the first line matching `until` or None after `seconds`, the lines matching `collect`)."""
        self.s.reset_input_buffer()
        self.rest = b""
        self.s.write((cmd + "\n").encode())
        kept = []
        for line in self.lines(seconds):
            if collect and re.match(collect, line):
                kept.append(line)
            if re.match(until, line):
                return line, kept
        return None, kept


def device_of(w):
    i = kv(w)
    fw, board = i.get("fw", ""), i.get("board", "")
    if "host" in i:
        return "air"
    fw = {"vlive_tx": "hat_tx", "vlive_rx": "hat_rx", "vlive_rx_ground_unit": "ground_unit"}.get(fw, fw)  # before 2026-11-25
    if board in ("hat", "vlink"):
        return {"hat_tx": "pi-hat", "hat_rx": "hat-rx"}.get(fw)
    return "ground-unit" if board == "ground-unit" and fw == "ground_unit" else None


def probe():
    """[(port, device or "-", identity or why there is none)] for every Horizon USB serial port."""
    out = []
    for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
        if p.vid not in (AIR_VID, GROUND_VID):
            continue
        try:
            b = Board(p.device)
        except Exception as e:
            why = ("in use by another program (a setup site tab, Horizon Receiver?)" if "denied" in str(e) else
                   "not answering on USB (its firmware stopped): unplug and replug it")
            out.append((p.device, "-", why))
            continue
        w, _ = b.ask("W", r"^W ", 3.0)
        b.s.close()
        out.append((p.device, (w and device_of(w)) or "-", w or "no answer to W"))
    return out


def check(found, devices):
    """Checks the one plugged-in board that is one of `devices`; returns (Board, its W line, its K line) or None."""
    mine = [f for f in found if f[1] in devices]
    if len(mine) != 1:
        error(f"{' or '.join(devices)}: {'not found' if not mine else 'several plugged in: unplug all but one'}")
        return None
    port, device, w = mine[0]
    b = Board(port)
    if device != "air" and kv(w).get("radio") != "ok":
        error(f"{device} radio={kv(w).get('radio')}: the LR2021 did not start")
    k, _ = b.ask("K", r"^K (profile|saved) ")
    if k:
        print(f"EVENT {device} link settings: {k[2:]}")
    else:
        error(f"{device}: no answer to K (link settings)")
    if device == "air":
        end, lines = b.ask("S", r"^CHECK result=", 20, collect=r"^CHECK \w+ ")
        end2, spi = b.ask("T", r"^T RESULT ", 60, collect=r"^T ")  # the transmitter stops for ~10 s
        for line in lines + spi:
            print(f"EVENT air {line}")
            if re.match(r"^CHECK \w+ fail ", line):
                error(f"air {line}")
        if not end or not end2 or "PASS" not in end2:
            error("air: the self-check or the SPI link test did not pass")
    elif device != "pi-hat":
        status = [line for line in b.lines(3.5) if line.startswith("S ok=")]
        if status:
            print(f"EVENT {device} {status[-1]}")
        else:
            error(f"{device}: no status lines in 3 s")
    return b, w, k


def pair(found):
    air, ground = check(found, ["air"]), check(found, ["ground-unit", "hat-rx"])
    if not air or not ground:
        return
    (_, wa, ka), (g, wg, kg) = air, ground
    if kv(wa).get("fmt") != kv(wg).get("fmt"):
        error(f"over-the-air formats differ: Air {kv(wa).get('fmt')}, Ground {kv(wg).get('fmt')}")
    pa, pg = kv(ka), kv(kg)
    if (pa.get("freq_hz"), pa.get("hop")) != (pg.get("freq_hz"), pg.get("hop")):
        error(f"not paired: Air {pa.get('freq_hz')} Hz hop {pa.get('hop')}, Ground {pg.get('freq_hz')} Hz hop "
              f"{pg.get('hop')} (Pair them on the setup site)")
        return
    time.sleep(10)  # the Air's transmitter restarts after the SPI link test
    g.s.reset_input_buffer()  # the status lines of the wait
    g.rest = b""
    first = last = None
    for line in g.lines(11.5):  # Ground status, one line per second: "S ok=.. crc=.. locked=.."
        if line.startswith("S ok="):
            last = {k: int(v) if v.lstrip("-").isdigit() else v for k, v in kv(line).items()} | {"t": time.monotonic()}
            first = first or last
    if not first or first is last:
        return error("the Ground reports no status")
    dt = last["t"] - first["t"]
    ok, crc = last["ok"] - first["ok"], last["crc"] - first["crc"]
    share = ok / (dt / SLOT_S)
    print(f"EVENT link {dt:.1f} s: received={ok} ({share * 100:.1f} % of the slots) crc_errors={crc} "
          f"locked={last.get('locked')}")
    if last.get("locked") != 1:
        error("the Ground is not locked to the Air")
    if share < 0.90:
        error(f"the Ground received only {share * 100:.0f} % of the slots (antennas, distance, or the radios)")
    if crc:
        error(f"{crc} packets with CRC errors")


if __name__ == "__main__":
    cmd = sys.argv[1:]
    if cmd in (["check", "air"], ["check", "pi-hat"], ["check", "ground-unit"], ["check", "hat-rx"], ["pair"]):
        found = probe()
        for port, dev, info in found:
            print(f"EVENT {port}: {dev} {info}")
        check(found, [cmd[1]]) if cmd[0] == "check" else pair(found)
        print(f"RESULT {'FAIL' if errors else 'PASS'}")
        sys.exit(1 if errors else 0)
    if cmd != ["list"]:
        sys.exit(__doc__)
    for port, dev, info in probe():
        print(dev, port, info)
