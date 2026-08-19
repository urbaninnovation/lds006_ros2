"""Serieller Transport für den LDS-006.

Getrennt vom Parser, damit dieser ohne Hardware testbar bleibt. Die einzige
Abhängigkeit (`pyserial`) sitzt hier und nirgends sonst.
"""

from __future__ import annotations

from typing import Iterator

from .lds006 import BAUDRATE, CMD_START, CMD_STOP, Lds006Parser, Scan, Status


class Lds006Serial:
    """Öffnet den Port, startet den Motor und liefert Scans.

    Am Raspberry Pi `ant000test` führt der Weg über einen **USB-Seriell-Adapter**
    (`/dev/ttyUSB0`), nicht über den GPIO-UART: `/dev/ttyAMA0` ist dort vom
    DDSM-Antrieb belegt.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        mirror: bool = True,
        offset_deg: int = 0,
        timeout: float = 1.0,
    ) -> None:
        import serial  # lokal, damit der Parser ohne pyserial nutzbar bleibt

        self.parser = Lds006Parser(mirror=mirror, offset_deg=offset_deg)
        self.ser = serial.Serial(port, BAUDRATE, timeout=timeout)

    # -- Motorsteuerung ----------------------------------------------------
    def start(self) -> None:
        """Motor anwerfen. Ohne diesen Befehl kommt **kein einziges Byte**."""
        self.ser.write(CMD_START)

    def stop(self) -> None:
        self.ser.write(CMD_STOP)

    def restart(self) -> None:
        """Stop-Start. Kur, wenn der Befehlsleser des Sensors verklemmt ist —
        unerkannte Bytes auf der Leitung können ihn in einem angefangenen
        Rahmen hängen lassen, danach wirkt `startlds$` nicht mehr."""
        self.stop()
        self.ser.write(CMD_STOP)
        self.start()

    # -- Datenstrom --------------------------------------------------------
    def events(self, chunk: int = 512) -> Iterator[Scan | Status]:
        """Blockierender Generator über alle Ereignisse."""
        while True:
            data = self.ser.read(max(1, min(chunk, self.ser.in_waiting or 1)))
            if data:
                yield from self.parser.feed(data)

    def scans(self, only_complete: bool = True) -> Iterator[Scan]:
        for ev in self.events():
            if isinstance(ev, Scan) and (ev.complete or not only_complete):
                yield ev

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self.ser.close()

    def __enter__(self) -> "Lds006Serial":
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
