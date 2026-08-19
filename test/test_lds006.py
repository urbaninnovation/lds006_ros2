"""Tests gegen **aufgezeichnete Sensordaten**, nicht gegen Annahmen.

`test/data/*.bin` sind Rohmitschnitte vom Aufbau: einmal Dauerbetrieb, einmal
der Anlauf nach `startlds$`. Damit prüfen die Tests genau die Aussagen, die
diesen Sensor von einem XV-11 unterscheiden — wer sie ändert, muss sie neu
belegen.
"""

import pathlib

import pytest

from ant_lidar.lds006 import (
    ERR_NO_ECHO,
    ERR_TOO_CLOSE,
    FRAME_LEN,
    STATUS_FILL,
    PACKETS_PER_REV,
    POINTS_PER_REV,
    Lds006Parser,
    Scan,
    Status,
    checksum,
    frame_checksum,
)

DATA = pathlib.Path(__file__).parent / "data"


@pytest.fixture(scope="module")
def dauerbetrieb() -> bytes:
    return (DATA / "lauf_dauerbetrieb.bin").read_bytes()


@pytest.fixture(scope="module")
def anlauf() -> bytes:
    return (DATA / "lauf_anlauf.bin").read_bytes()


# --- Die beiden Abweichungen vom XV-11 ------------------------------------

def test_pruefsumme_ist_bytesumme(dauerbetrieb):
    """Bytesumme über 0..19 — der XV-11-Shift-Add trifft hier NICHT."""
    p = Lds006Parser()
    list(p.feed(dauerbetrieb))
    assert p.frames_ok > 300
    assert p.frames_bad == 0


def test_xv11_pruefsumme_wuerde_alles_verwerfen(dauerbetrieb):
    """Gegenprobe: mit dem XV-11-Algorithmus käme kein einziges Paket durch."""

    def xv11(frame):
        c = 0
        for i in range(0, 20, 2):
            c = (c << 1) + (frame[i] | (frame[i + 1] << 8))
        return ((c & 0x7FFF) + (c >> 15)) & 0x7FFF

    start = next(
        i for i in range(len(dauerbetrieb) - FRAME_LEN)
        if dauerbetrieb[i] == 0xFA and dauerbetrieb[i + 1] >= 0xA0
        and checksum(dauerbetrieb[i : i + FRAME_LEN])
        == frame_checksum(dauerbetrieb[i : i + FRAME_LEN])
    )
    treffer = 0
    gesamt = 0
    for off in range(start, len(dauerbetrieb) - FRAME_LEN, FRAME_LEN):
        f = dauerbetrieb[off : off + FRAME_LEN]
        if f[0] != 0xFA or checksum(f) != frame_checksum(f):
            continue
        gesamt += 1
        if xv11(f) == frame_checksum(f):
            treffer += 1
    assert gesamt > 100
    assert treffer == 0


def test_drehzahl_teiler_ist_100(dauerbetrieb):
    """/100 ergibt ~300 rpm. /64 ergäbe ~470 und widerspräche der Datenrate."""
    p = Lds006Parser()
    scans = [e for e in p.feed(dauerbetrieb) if isinstance(e, Scan)]
    assert scans
    for s in scans:
        assert 250.0 < s.rpm < 360.0


# --- Rahmenaufbau ---------------------------------------------------------

def test_vollstaendige_umdrehungen(dauerbetrieb):
    p = Lds006Parser()
    scans = [e for e in p.feed(dauerbetrieb) if isinstance(e, Scan)]
    voll = [s for s in scans if s.complete]
    assert len(voll) >= 2
    for s in voll:
        assert s.packets == PACKETS_PER_REV
        assert len(s.samples) == POINTS_PER_REV


def test_winkel_sind_lueckenlos_und_eindeutig(dauerbetrieb):
    p = Lds006Parser()
    voll = [e for e in p.feed(dauerbetrieb) if isinstance(e, Scan) and e.complete]
    assert {s.angle for s in voll[0].samples} == set(range(POINTS_PER_REV))


def test_stueckweises_einspeisen_ergibt_dasselbe(dauerbetrieb):
    """Der Parser darf nicht davon abhängen, wie der Strom zerteilt ankommt."""
    ganz = Lds006Parser()
    a = [e for e in ganz.feed(dauerbetrieb) if isinstance(e, Scan)]
    stueck = Lds006Parser()
    b = []
    for i in range(0, len(dauerbetrieb), 7):      # krumme Blockgröße mit Absicht
        b.extend(e for e in stueck.feed(dauerbetrieb[i : i + 7]) if isinstance(e, Scan))
    assert ganz.frames_ok == stueck.frames_ok
    assert [s.samples for s in a] == [s.samples for s in b]


# --- Statusrahmen ---------------------------------------------------------

def test_anlauf_liefert_statusrahmen(anlauf):
    """Index außerhalb 0xA0..0xF9 — beim Anlauf, nie im Dauerbetrieb."""
    p = Lds006Parser()
    st = [e for e in p.feed(anlauf) if isinstance(e, Status)]
    assert len(st) > 50
    assert all(e.frame_type >= 0xFA for e in st)


def test_statusdrehzahl_steigt_beim_anlauf(anlauf):
    p = Lds006Parser()
    rpm = [e.rpm for e in p.feed(anlauf) if isinstance(e, Status)]
    assert rpm[-1] > rpm[0] + 50.0


def test_dauerbetrieb_ohne_statusrahmen(dauerbetrieb):
    p = Lds006Parser()
    assert not [e for e in p.feed(dauerbetrieb) if isinstance(e, Status)]


def _frames(daten):
    """Alle prüfsummenrichtigen Rahmen, mit Resynchronisation."""
    i = 0
    while i + FRAME_LEN <= len(daten):
        f = daten[i : i + FRAME_LEN]
        if f[0] == 0xFA and f[1] >= 0xA0 and checksum(f) == frame_checksum(f):
            yield f
            i += FRAME_LEN
        else:
            i += 1


def test_es_gibt_genau_zwei_fehlercodes(dauerbetrieb, anlauf):
    """Winkelrahmen tragen nur 0x88 und 0x99 — kein dritter Code."""
    codes = set()
    for daten in (dauerbetrieb, anlauf):
        for f in _frames(daten):
            if f[1] >= 0xFA:
                continue
            for k in range(4):
                r = f[4 + k * 4 : 8 + k * 4]
                if r[1] & 0x80:
                    codes.add(r[0])
    assert codes == {ERR_TOO_CLOSE, ERR_NO_ECHO}


def test_statusrahmen_tragen_fuellung_keinen_fehlercode(anlauf):
    """`77 77 00 00` hat Bit 7 NICHT gesetzt. Wer die Messfelder eines
    Statusrahmens auswertet, bekommt daraus 14199 mm untergeschoben."""
    gesehen = 0
    for f in _frames(anlauf):
        if f[1] < 0xFA:
            continue
        gesehen += 1
        for k in range(4):
            assert f[4 + k * 4 : 8 + k * 4] == STATUS_FILL
    assert gesehen > 20
    assert not (STATUS_FILL[1] & 0x80)


def _baue_rahmen(index: int, speed: int, samples) -> bytes:
    """Baut einen prüfsummenrichtigen Rahmen — für Fälle, die im Mitschnitt
    zufällig fehlen könnten."""
    f = bytearray([0xFA, index, speed & 0xFF, speed >> 8])
    for s in samples:
        f += bytes(s)
    c = sum(f) & 0xFFFF
    f += bytes((c & 0xFF, c >> 8))
    return bytes(f)


def test_fuellung_in_WINKELrahmen_wird_nicht_zur_messung():
    """Beim Hochlaufen steht die Füllung in regulären Winkelrahmen (Index < 0xFA).
    Wer nur auf den Rahmentyp prüft, gibt daraus 14199 mm aus."""
    p = Lds006Parser(mirror=False)
    daten = b"".join(
        _baue_rahmen(0xA0 + i, 50, [STATUS_FILL] * 4) for i in range(PACKETS_PER_REV)
    ) + _baue_rahmen(0xA0, 50, [STATUS_FILL] * 4)
    scans = [e for e in p.feed(daten) if isinstance(e, Scan)]
    assert p.frames_ok == PACKETS_PER_REV + 1
    assert scans, "eine Umdrehung hätte abgeschlossen werden müssen"
    assert scans[0].valid_count == 0
    assert all(s.distance_mm == 0 for s in scans[0].samples)


def test_staerke_null_gilt_nie_als_messung():
    p = Lds006Parser(mirror=False)
    daten = b"".join(
        _baue_rahmen(0xA0 + i, 30000, [bytes((0x2C, 0x01, 0x00, 0x00))] * 4)
        for i in range(PACKETS_PER_REV)
    ) + _baue_rahmen(0xA0, 30000, [bytes((0x2C, 0x01, 0x00, 0x00))] * 4)
    scan = next(e for e in p.feed(daten) if isinstance(e, Scan))
    assert scan.valid_count == 0        # 300 mm, aber Staerke 0 -> keine Messung


def test_echter_messwert_bleibt_gueltig():
    """Gegenprobe, damit die Regel nicht zu viel wegwirft."""
    p = Lds006Parser(mirror=False)
    daten = b"".join(
        _baue_rahmen(0xA0 + i, 30000, [bytes((0x2C, 0x01, 0x64, 0x00))] * 4)
        for i in range(PACKETS_PER_REV)
    ) + _baue_rahmen(0xA0, 30000, [bytes((0x2C, 0x01, 0x64, 0x00))] * 4)
    scan = next(e for e in p.feed(daten) if isinstance(e, Scan))
    assert scan.valid_count == POINTS_PER_REV
    assert scan.samples[0].distance_mm == 300
    assert scan.samples[0].strength == 100


def test_parser_gibt_statusrahmen_keine_messwerte_aus(anlauf):
    """Die Gegenprobe zum vorigen Test auf Ebene der Bibliothek."""
    p = Lds006Parser()
    for scan in (e for e in p.feed(anlauf) if isinstance(e, Scan)):
        for s in scan.samples:
            assert not (s.valid and s.distance_mm == 14199)


# --- Winkelabbildung ------------------------------------------------------

def test_spiegelung_laesst_null_stehen():
    p = Lds006Parser(mirror=True)
    assert p.map_angle(0) == 0
    assert p.map_angle(90) == 270
    assert p.map_angle(270) == 90


def test_ohne_spiegelung_unveraendert():
    p = Lds006Parser(mirror=False)
    assert [p.map_angle(a) for a in (0, 90, 180, 270)] == [0, 90, 180, 270]


def test_versatz_wirkt_nach_der_spiegelung():
    p = Lds006Parser(mirror=True, offset_deg=90)
    assert p.map_angle(0) == 90
    assert p.map_angle(90) == 0


# --- Robustheit -----------------------------------------------------------

def test_muell_vor_dem_strom_wird_uebersprungen(dauerbetrieb):
    p = Lds006Parser()
    scans = [e for e in p.feed(b"\x00\xff\xfa\x12" * 20 + dauerbetrieb)
             if isinstance(e, Scan)]
    assert scans
    assert p.frames_ok > 300


def test_leerer_strom_liefert_nichts():
    p = Lds006Parser()
    assert not list(p.feed(b""))
    assert not list(p.feed(b"\x00" * 500))
