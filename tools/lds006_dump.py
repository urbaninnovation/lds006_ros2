#!/usr/bin/env python3
"""LDS-006 ohne ROS ansehen — zum Prüfen der Verkabelung am Raspberry.

    python3 lds006_dump.py --port /dev/ttyUSB0
    python3 lds006_dump.py --port /dev/ttyUSB0 --raw       # Hex-Mitschnitt
    python3 lds006_dump.py --datei mitschnitt.bin          # ohne Hardware
"""

from __future__ import annotations

import argparse
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from ant_lidar.lds006 import Lds006Parser, Scan, Status  # noqa: E402


def zeige(scan: Scan) -> None:
    gut = scan.valid_count
    bild = []
    for a0 in range(0, 360, 30):
        seg = [s.distance_mm for s in scan.samples[a0 : a0 + 30] if s.valid]
        bild.append(f"{a0:3d}:{sum(seg)//len(seg):5d}" if seg else f"{a0:3d}:    -")
    print(f"{scan.rpm:6.1f} rpm  {scan.packets:2d}/90 Pakete  {gut:3d}/360 gültig  "
          + " ".join(bild))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--datei", help="statt des Ports eine Aufzeichnung lesen")
    ap.add_argument("--raw", action="store_true", help="Rohbytes als Hex ausgeben")
    ap.add_argument("--kein-spiegel", action="store_true")
    ap.add_argument("--versatz", type=int, default=0, help="Grad")
    ap.add_argument("--anzahl", type=int, default=0, help="nach N Scans beenden")
    args = ap.parse_args()

    p = Lds006Parser(mirror=not args.kein_spiegel, offset_deg=args.versatz)

    if args.datei:
        daten = pathlib.Path(args.datei).read_bytes()
        for ev in p.feed(daten):
            if isinstance(ev, Scan):
                zeige(ev)
            elif args.raw:
                print(f"Status 0x{ev.frame_type:02X}  {ev.rpm:6.1f} rpm")
        print(f"\n{p.frames_ok} Rahmen ok, {p.frames_bad} Prüfsummenfehler, "
              f"{p.resyncs} Resync", file=sys.stderr)
        return 0

    from ant_lidar.serial_link import Lds006Serial

    n = 0
    with Lds006Serial(port=args.port, mirror=not args.kein_spiegel,
                      offset_deg=args.versatz) as link:
        print(f"{args.port} offen, Motor gestartet. Strg-C beendet.", file=sys.stderr)
        for ev in link.events():
            if isinstance(ev, Status):
                print(f"Status 0x{ev.frame_type:02X}  {ev.rpm:6.1f} rpm — "
                      f"{'läuft an/aus' if ev.rpm > 1 else 'steht'}")
            elif isinstance(ev, Scan):
                zeige(ev)
                n += 1
                if args.anzahl and n >= args.anzahl:
                    break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
