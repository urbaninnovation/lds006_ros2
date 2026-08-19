"""
Protokoll des LDS-006 — des LiDAR aus Ecovacs-Deebot-Saugrobotern.

Reiner Parser: keine Abhängigkeiten, kein I/O, keine ROS-Bezüge. Dadurch gegen
aufgezeichnete Sensordaten testbar, ohne dass Hardware angeschlossen sein muss
(siehe `test/data/`).

Der Rahmen ist XV-11-kompatibel (Neato), **die Auswertung nicht**. Zwei
Abweichungen, beide am Aufbau belegt und der Grund, warum fertige XV-11-Treiber
an diesem Sensor scheitern:

1. Die Prüfsumme ist eine einfache 16-Bit-Bytesumme über Byte 0..19 — nicht der
   Shift-Add-Algorithmus des XV-11. Der verwirft sonst *jedes* Paket.
2. Das Drehzahlfeld wird durch 100 geteilt, nicht durch 64.

Vollständige Herleitung in PROTOCOL.md.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, Sequence

# --- Rahmen ---------------------------------------------------------------
FRAME_LEN = 22
START_BYTE = 0xFA
INDEX_MIN = 0xA0          # erster Winkelrahmen
INDEX_MAX = 0xF9          # letzter Winkelrahmen -> 90 Stück je Umdrehung
STATUS_MIN = 0xFA         # ab hier Statusrahmen, ohne Messwerte
PACKETS_PER_REV = 90
SAMPLES_PER_PACKET = 4
POINTS_PER_REV = 360

# --- Felder ---------------------------------------------------------------
RPM_DIVISOR = 100.0       # NICHT 64 wie beim XV-11
FLAG_INVALID = 0x80       # Bit 7 des Distanz-Highbytes
FLAG_WEAK = 0x40          # Bit 6 — aus der XV-11-Konvention, hier nie gesehen
DIST_MASK = 0x3F

# --- Fehlercodes ----------------------------------------------------------
# Nur zwei, über alle Aufzeichnungen belegt. Sie stehen im Distanz-Lowbyte,
# sobald Bit 7 des Highbytes gesetzt ist.
ERR_NONE = 0x00
ERR_TOO_CLOSE = 0x88      # Ziel näher als die Nahgrenze
ERR_NO_ECHO = 0x99        # kein Rückstreusignal

# Füllmuster für „misst gerade nicht". Bit 7 ist dabei NICHT gesetzt, das Muster
# sieht also wie ein gültiger Messwert aus und ergibt 14199 mm.
#
# Es steht nicht nur in Statusrahmen: beim Hochlaufen dreht der Sensor bereits
# und zählt Winkel, misst aber noch nicht — dann kommt die Füllung in **regulären
# Winkelrahmen** mit Index < 0xFA. Ein Parser, der nur auf den Rahmentyp achtet,
# lässt sie durch.
#
# Die verlässliche Unterscheidung ist die Signalstärke: über alle Aufzeichnungen
# hat **kein echter Messwert die Stärke 0** (788 geprüft). Ein unmarkierter Wert
# mit Stärke 0 ist deshalb keine Messung.
STATUS_FILL = bytes((0x77, 0x77, 0x00, 0x00))
ERR_NOT_MEASURING = 0x77  # synthetisch: Füllung statt Messwert, kein Sensor-Fehlercode

# --- Schnittstelle --------------------------------------------------------
BAUDRATE = 115200
CMD_START = b"startlds$"  # ohne Zeilenende!
CMD_STOP = b"stoplds$"

RANGE_MIN_M = 0.15        # kleinster je gemessener Wert war 107 mm
RANGE_MAX_M = 8.0         # größter je gemessener Wert war 7337 mm


@dataclass(frozen=True)
class Sample:
    """Ein Messwert. `angle` ist bereits auf die Fahrzeugkonvention gedreht."""

    angle: int            # 0..359 Grad
    distance_mm: int      # 0, wenn ungültig
    strength: int         # Reflexionsgrad, Einheit unbekannt
    error: int            # 0 = gültig, sonst ERR_*

    @property
    def valid(self) -> bool:
        return self.error == ERR_NONE


@dataclass(frozen=True)
class Status:
    """Statusrahmen: Index außerhalb des Winkelbereichs, keine Messwerte.

    Der Sensor sendet sie beim An- und Auslaufen und trägt im Drehzahlfeld die
    momentane Hochlaufdrehzahl. Daran — und nicht am Ausbleiben von Daten — ist
    „läuft an" von „steht" und von „Kabel ab" zu unterscheiden.
    """

    rpm: float
    frame_type: int       # beobachtet ausschließlich 0xFB


@dataclass(frozen=True)
class Scan:
    """Eine vollständige Umdrehung. `samples[i]` ist der Messwert bei i Grad."""

    samples: tuple[Sample, ...]
    rpm: float
    packets: int          # gesehene Pakete, 90 = lückenlos

    @property
    def complete(self) -> bool:
        return self.packets == PACKETS_PER_REV

    @property
    def valid_count(self) -> int:
        return sum(1 for s in self.samples if s.valid)


def checksum(frame: Sequence[int]) -> int:
    """Arithmetische 16-Bit-Bytesumme über Byte 0..19."""
    return sum(frame[:20]) & 0xFFFF


def frame_checksum(frame: Sequence[int]) -> int:
    """Die im Rahmen mitgelieferte Prüfsumme (Byte 20/21, little endian)."""
    return frame[20] | (frame[21] << 8)


class Lds006Parser:
    """Zerlegt den Bytestrom in `Scan`- und `Status`-Ereignisse.

    Die Synchronisation prüft **zwei** Bytes: `0xFA` allein kommt auch mitten in
    Distanzdaten vor. Erst zusammen mit einem Index ab `0xA0` beginnt ein Rahmen.
    Pakete mit falscher Prüfsumme werden verworfen, nicht geraten.

    `mirror` und `offset_deg` bilden den rohen Protokollwinkel auf die
    Fahrzeugkonvention ab: **0 Grad voraus, Winkel im Uhrzeigersinn** (von oben
    gesehen). Gespiegelt werden muss, weil der Sensor von oben betrachtet gegen
    den Uhrzeigersinn läuft; ohne das steht das Bild seitenverkehrt — und zwar
    *plausibel* seitenverkehrt, was die Falle daran ist.
    """

    def __init__(self, mirror: bool = True, offset_deg: int = 0) -> None:
        self.mirror = mirror
        self.offset_deg = offset_deg
        self.frames_ok = 0
        self.frames_bad = 0
        self.resyncs = 0
        self._buf = bytearray()
        self._samples: list[Sample | None] = [None] * POINTS_PER_REV
        self._seen: set[int] = set()
        self._prev_index = -1
        self._rpm = 0.0

    # -- Winkelabbildung ---------------------------------------------------
    def map_angle(self, raw: int) -> int:
        a = (360 - raw) % 360 if self.mirror else raw
        return (a + self.offset_deg) % 360

    # -- Hauptschleife -----------------------------------------------------
    def feed(self, data: bytes) -> Iterator[Scan | Status]:
        """Bytes einspeisen, fertige Ereignisse herausgeben."""
        self._buf.extend(data)
        while True:
            start = self._find_start()
            if start is None:
                return
            if start:
                self.resyncs += 1
                del self._buf[:start]
            if len(self._buf) < FRAME_LEN:
                return
            frame = bytes(self._buf[:FRAME_LEN])
            if checksum(frame) != frame_checksum(frame):
                self.frames_bad += 1
                del self._buf[:1]          # nur ein Byte weiter, nicht raten
                continue
            self.frames_ok += 1
            del self._buf[:FRAME_LEN]
            event = self._handle(frame)
            if event is not None:
                yield event

    def _find_start(self) -> int | None:
        """Index des nächsten plausiblen Rahmenanfangs, None wenn keiner da."""
        b = self._buf
        for i in range(len(b) - 1):
            if b[i] == START_BYTE and b[i + 1] >= INDEX_MIN:
                return i
        del b[: max(0, len(b) - 1)]        # nichts Brauchbares mehr im Puffer
        return None

    def _handle(self, frame: bytes) -> Scan | Status | None:
        rpm = (frame[2] | (frame[3] << 8)) / RPM_DIVISOR

        if frame[1] >= STATUS_MIN:         # Statusrahmen, kein Winkel
            self._rpm = rpm
            return Status(rpm=rpm, frame_type=frame[1])

        self._rpm = rpm
        index = frame[1] - INDEX_MIN       # 0..89
        finished: Scan | None = None
        if self._prev_index >= 0 and index < self._prev_index:
            finished = self._finish_scan()
        self._prev_index = index
        self._seen.add(index)

        for k in range(SAMPLES_PER_PACKET):
            lo, hi, s_lo, s_hi = frame[4 + k * 4 : 8 + k * 4]
            angle = self.map_angle(index * 4 + k)
            strength = s_lo | (s_hi << 8)
            # Stärke 0 ohne gesetztes Ungültig-Bit ist die Füllung, keine Messung
            # — sonst käme beim Hochlaufen ein Kranz aus 14199 mm heraus.
            if hi & FLAG_INVALID or strength == 0:
                self._samples[angle] = Sample(angle, 0, 0, lo)
            else:
                self._samples[angle] = Sample(
                    angle, ((hi & DIST_MASK) << 8) | lo, strength, ERR_NONE
                )
        return finished

    def _finish_scan(self) -> Scan:
        samples = tuple(
            s if s is not None else Sample(a, 0, 0, ERR_NO_ECHO)
            for a, s in enumerate(self._samples)
        )
        scan = Scan(samples=samples, rpm=self._rpm, packets=len(self._seen))
        self._samples = [None] * POINTS_PER_REV
        self._seen = set()
        return scan
