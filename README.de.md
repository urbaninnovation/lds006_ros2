# ant_lidar — LDS-006 als ROS-2-Scanner

Der LiDAR aus einem Ecovacs-Deebot-Saugroboter am Raspberry Pi `ant000test`,
veröffentlicht als `sensor_msgs/LaserScan`.

Protokoll und Verkabelung wurden am Test-ESP `bono` erarbeitet
(Projekt `263_Test`); die vollständige Spezifikation steht in
[PROTOCOL.md](PROTOCOL.md) — **auf Englisch**, weil sie das Dokument ist, das
nach außen gehen soll.

## Aufbau

```
ant_lidar/
├── lds006.py        reines Protokoll — keine Abhängigkeiten, kein I/O
├── serial_link.py   Transport (pyserial); die einzige Abhängigkeit sitzt hier
└── lidar_node.py    ROS-2-Knoten -> sensor_msgs/LaserScan
tools/lds006_dump.py CLI ohne ROS, zum Prüfen der Verkabelung
test/                17 Tests gegen aufgezeichnete Sensordaten
```

Die Dreiteilung hat einen Zweck: **`lds006.py` läuft ohne Sensor und ohne ROS.**
Deshalb prüfen die Tests das Protokoll gegen echte Mitschnitte in `test/data/`
statt gegen Annahmen, und deshalb ist die Datei auch außerhalb dieses Projekts
brauchbar — sie hängt von nichts ab.

## Anschluss am Pi

**USB-Seriell-Adapter, nicht GPIO.** `/dev/ttyAMA0` ist auf `ant000test` vom
DDSM-Antrieb belegt (siehe `ddsm/README.md`), und der Adapter liegt dem Sensor
ohnehin meist bei.

| Ader | Funktion | Adapter |
|---|---|---|
| schwarz | GND | GND |
| rot | 5 V | 5 V |
| blau | Sensor TX | RX |
| grün | Sensor RX | TX |

Nutzer `pi` muss in der Gruppe `dialout` sein — für den DDSM ist das schon
erledigt.

## Erst ohne ROS prüfen

```bash
python3 tools/lds006_dump.py --port /dev/ttyUSB0
```

Zeigt je Umdrehung Drehzahl, Paketzahl, gültige Punkte und ein grobes Rundumbild.
Ohne Hardware geht es auch gegen eine Aufzeichnung:

```bash
python3 tools/lds006_dump.py --datei test/data/lauf_dauerbetrieb.bin
```

Referenzwerte eines gesunden Laufs: **~300 rpm, 90/90 Pakete, 0 Prüfsummenfehler.**

## Bauen und starten

```bash
cd ~/ros2_ws && colcon build --packages-select ant_lidar --parallel-workers 1
```

```bash
source ~/ros2_ws/install/setup.bash && ros2 launch ant_lidar lds006.launch.py
```

| Parameter | Standard | Bedeutung |
|---|---|---|
| `port` | `/dev/ttyUSB0` | serieller Port |
| `frame_id` | `laser` | TF-Rahmen im Header |
| `mirror` | `true` | Drehsinn spiegeln — siehe unten |
| `offset_deg` | `0` | Nullrichtung verschieben |
| `only_complete` | `true` | nur lückenlose Umdrehungen veröffentlichen |

```bash
ros2 topic hz /scan
```

## Zwei Dinge, die man leicht falsch macht

**Der Drehsinn.** Der Sensor läuft von oben gesehen gegen den Uhrzeigersinn. Ohne
Spiegelung steht das Bild seitenverkehrt — und zwar *plausibel*: Entfernungen und
Raumform sehen richtig aus, nur die Drehrichtung stimmt nicht. Beim Fahren fällt
das erst auf, wenn die Karte sich nicht schließt, und dann verdächtigt man zuerst
die Odometrie. Der Knoten dreht zusätzlich von der Fahrzeugkonvention (im
Uhrzeigersinn) auf die ROS-Konvention nach REP 103 (gegen den Uhrzeigersinn).

**Die Ausfälle.** Ein fehlender Messwert darf **nicht** als `0.0` gemeldet werden
— 0 ist ein gültiger Wert dicht am Sensor. REP 117 sieht drei Werte vor, und die
beiden Fehlercodes passen genau darauf:

| Code | Bedeutung | `LaserScan` |
|---|---|---|
| `0x88` | näher als messbar | `-inf` |
| `0x99` | kein Rückstreusignal | `+inf` |
| sonst | unklar | `nan` |

## Tests

```bash
python3 -m pytest test/ -q
```

Sie prüfen genau die Aussagen, die diesen Sensor von einem XV-11 unterscheiden —
Prüfsumme als Bytesumme, Drehzahlteiler 100, Statusrahmen, und dass die Füllung
`77 77 00 00` **kein** Fehlercode ist. Wer eine davon ändert, muss sie neu
belegen.

## Offen

- **Nullrichtung** am Fahrzeug bestimmen und in `offset_deg` eintragen.
- **Entzerrung.** Eine Umdrehung dauert 200 ms; `slam_toolbox` entzerrt nicht.
  Die Gierrate müsste aus der IMU kommen.
- **Blinder Keil.** Am Versuchsaufbau fiel der Sektor 348°–32° dauerhaft aus —
  vermutlich ein Gegenstand dicht vor dem Sensor. Vor dem Einbau klären, ob so
  etwas an der Halterung sitzt: dann fährt der blinde Bereich mit.
