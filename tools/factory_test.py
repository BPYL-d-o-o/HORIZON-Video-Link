#!/usr/bin/env python3
"""Factory test for a Horizon Dev Air and a Horizon Dev Ground (rev-1A): both plugged into this computer by USB, the
Air powered with its camera and antenna, the Ground with its antenna, a short distance apart.

    python tools/factory_test.py                       both found by their USB names
    python tools/factory_test.py --air COM9 --ground COM8
    python tools/factory_test.py --air-serial d549a71bbdfc27a2 --ground-serial 1E44EA1F60F3FF45
    python tools/factory_test.py --leds                also asks the operator to confirm the LEDs

To test one board, use a known good one (a reference) as the other. Steps:
  1. identity: both answer with their firmware; the over-the-air formats match
  2. Air self-check (luckfox_image/horizon_check.sh): no part fails (warnings are listed; the N0CALL default is fine)
  3. pairing: both start on the same link settings (else: Pair on the setup site first)
  4. radio link: the Ground receives the Air for 10 s: locked, at least 90 % of the slots, no CRC errors
  5. Air SPI link to its radio controller at the transmitter's clock (horizon_pair.sh "T"; the Air stops sending for
     about 10 s, so this runs last)
  6. (--leds) the operator confirms both LEDs on the Air

Prints EVENT/ERROR lines and a final RESULT PASS|FAIL (exit status 0 / 1). Needs pyserial.
"""
import argparse
import re
import sys
import time

import serial
import serial.tools.list_ports

AIR_VID, GROUND_VID = 0x2207, 0x2E8A
SLOT_S = 0.01754  # one slot of the v4 link (SF5, 1 MHz, 255-byte packets)


def find_port(vid, name, port, serial_no):
    if port:
        return port
    found = [p for p in serial.tools.list_ports.comports() if p.vid == vid and
             (serial_no is None or (p.serial_number or "").lower() == serial_no.lower())]
    named = [p for p in found if name in (p.product or p.description or "")]
    pick = named or found
    if len(pick) != 1:
        raise SystemExit(f"FATAL {name}: {'not found' if not pick else 'several found: ' + ', '.join(p.device for p in pick)}"
                         f" (use --{'air' if vid == AIR_VID else 'ground'} PORT)")
    return pick[0].device


class Board:
    def __init__(self, port, name):
        self.name = name
        self.s = serial.Serial(port, 115200, timeout=0.2)
        self.s.dtr = True  # the Ground sends only while DTR is set
        self.rest = b""

    def lines(self, seconds):
        """Yields lines for up to `seconds`."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rest += self.s.read(4096)
            while b"\n" in self.rest:
                line, self.rest = self.rest.split(b"\n", 1)
                yield line.decode(errors="replace").strip()

    def ask(self, cmd, until, seconds=5.0, collect=None):
        """Sends cmd, returns the first line matching `until` (None after `seconds`); lines matching `collect` are kept."""
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


def kv(line):
    return dict(x.split("=", 1) for x in line.split()[1:] if "=" in x)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--air"), ap.add_argument("--ground")
    ap.add_argument("--air-serial"), ap.add_argument("--ground-serial")
    ap.add_argument("--seconds", type=float, default=10.0, help="radio link test length (default 10)")
    ap.add_argument("--leds", action="store_true", help="ask the operator to confirm the LEDs")
    a = ap.parse_args()
    fails = []

    def fail(msg):
        fails.append(msg)
        print(f"ERROR {msg}", flush=True)

    air = Board(find_port(AIR_VID, "Horizon Dev Air", a.air, a.air_serial), "Air")
    ground = Board(find_port(GROUND_VID, "Horizon Dev Ground", a.ground, a.ground_serial), "Ground")
    print(f"EVENT ports air={air.s.port} ground={ground.s.port}", flush=True)

    # 1. identity
    wa, _ = air.ask("W", r"^W ")
    wg, _ = ground.ask("W", r"^W ")
    if not wa:
        raise SystemExit("FATAL the Air does not answer on its USB serial port (card image too old, or still starting)")
    if not wg:
        raise SystemExit("FATAL the Ground does not answer (flash its firmware on the setup site)")
    ia, ig = kv(wa), kv(wg)
    print(f"EVENT air host={ia.get('host')} image={ia.get('image', '?')} hz_vtx={ia.get('hz_vtx')} rp2354={ia.get('rp2354')} format={ia.get('fmt')} boot={ia.get('boot', '?')} up_s={ia.get('up', '?')}")
    print(f"EVENT ground uid={ig.get('uid')} fw={ig.get('fw')} git={ig.get('git')} format={ig.get('fmt')}", flush=True)
    if not ig.get("fw", "").startswith("vlive_rx"):
        fail(f"the Ground runs {ig.get('fw')}, not the receiver firmware")
    if ia.get("fmt") != ig.get("fmt"):
        fail(f"over-the-air formats differ: Air {ia.get('fmt')}, Ground {ig.get('fmt')}")
    if ia.get("rp2354", "").replace("-dirty", "") != ig.get("git", "").replace("-dirty", ""):
        print(f"EVENT note: firmware versions differ (Air {ia.get('rp2354')}, Ground {ig.get('git')})")

    # 2. Air self-check
    end, checks = air.ask("S", r"^CHECK result=", 20, collect=r"^CHECK \w+ ")
    if not end:
        fail("the Air's self-check did not finish")
    for c in checks:
        m = re.match(r"^CHECK (\w+) (ok|warn|fail) (.*)$", c)
        if not m:
            continue
        part, level, text = m.groups()
        print(f"EVENT air check {part} {level}: {text}")
        if level == "fail":
            fail(f"Air {part}: {text}")
        elif level == "warn" and not (part == "callsign" and "N0CALL" in text):
            print(f"EVENT note: Air {part} warning (not a failure): {text}")

    # 3. pairing: both on the same plan
    ka, _ = air.ask("K", r"^K (profile|saved) ")
    kg, _ = ground.ask("K", r"^K (profile|saved) ")
    pa, pg = kv(ka or "K x"), kv(kg or "K x")
    if not ka or not kg:
        fail("could not read the link settings of both boards")
    elif (pa.get("freq_hz"), pa.get("hop")) != (pg.get("freq_hz"), pg.get("hop")):
        fail(f"not paired: Air {pa.get('freq_hz')} Hz hop {pa.get('hop')}, Ground {pg.get('freq_hz')} Hz hop {pg.get('hop')}"
             " (Pair them on the setup site)")
    else:
        print(f"EVENT paired freq_hz={pa.get('freq_hz')} hop={pa.get('hop')} pwr={pa.get('pwr')}", flush=True)

    # 4. radio link: Ground status lines ("S ok=.. crc=.. miss=.. locked=..", one per second)
    first = last = None
    for line in ground.lines(a.seconds + 1.5):
        if line.startswith("S ok="):
            v = {k: int(x) if x.lstrip("-").isdigit() else x for k, x in kv(line).items()}
            v["t"] = time.monotonic()
            first = first or v
            last = v
    if not first or first is last:
        fail("the Ground reports no status")
    else:
        dt = last["t"] - first["t"]
        ok, crc = last["ok"] - first["ok"], last["crc"] - first["crc"]
        share = ok / (dt / SLOT_S) if dt > 0 else 0
        print(f"EVENT link {dt:.1f} s: received={ok} ({share * 100:.1f} % of the slots) crc_errors={crc} locked={last.get('locked')}",
              flush=True)
        if last.get("locked") != 1:
            fail("the Ground is not locked to the Air")
        if share < 0.90:
            fail(f"the Ground received only {share * 100:.0f} % of the slots (antennas, distance, or the radios)")
        if crc:
            fail(f"{crc} packets with CRC errors")

    # 5. the Air's SPI link to its radio controller (stops the transmitter ~10 s, so last)
    end, tl = air.ask("T", r"^T RESULT ", 60, collect=r"^T ")
    for t in tl:
        print(f"EVENT air spi {t[2:]}")
    if not end:
        fail("the Air's SPI test did not finish (card image without it? update the Air)")
    elif "PASS" not in end:
        fail("the SPI link between the Luckfox and the radio controller has errors")

    # 6. LEDs (operator)
    if a.leds:
        print("Look at the Air (it restarted its transmitter; wait ~30 s):", flush=True)
        r1 = input("  Red LED by the radio controller steadily on (sending video)? [y/n] ").strip().lower()
        r2 = input("  Luckfox LED on, or blinking slowly (a warning such as the N0CALL default)? [y/n] ").strip().lower()
        if r1 != "y":
            fail("operator: the radio controller LED is not on")
        if r2 != "y":
            fail("operator: the Luckfox LED is not as expected")

    print(f"RESULT {'FAIL' if fails else 'PASS'}" + (f" ({len(fails)} problem{'s' if len(fails) > 1 else ''})" if fails else ""))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
