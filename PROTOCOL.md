# LDS-006 serial protocol

Reverse-engineered specification for the **LDS-006**, the 360° laser distance
sensor found in Ecovacs Deebot robot vacuums and sold as a spare part for
roughly €20–25.

Everything below was **measured on a physical unit** unless explicitly marked as
unverified. Where our findings contradict published sources, both are given.

Reference implementation: [`ant_lidar/lds006.py`](ant_lidar/lds006.py).
Tests run against recorded sensor output in [`test/data/`](test/data/).

---

## 1. Electrical

| Wire | Function | Level |
|---|---|---|
| black | GND | — |
| red | Vcc | **5 V** |
| blue | sensor TX (data out) | 3.3 V |
| green | sensor RX (commands in) | 3.3 V |

2.0 mm pitch connector. **115200 baud, 8N1.** No level shifter needed for a
3.3 V host. The motor needs no separate PWM line — speed is regulated
internally.

**Power: 5 V, 0.43–0.44 A** — printed on the unit's own label, so this is a
manufacturer figure rather than our measurement. About 2.2 W, which is more than
a dev board's 5 V pin should be asked to supply on top of its own draw.

![LDS-006 laser unit, label visible](docs/lds006.jpg)

*The model number and rating are printed on the top face, which makes this the
quickest way to identify the unit: `LDS-006`, Ecovacs Robotics Co., Ltd.*

Identification procedure if the wire colours differ: with only Vcc and GND
connected, measure both signal wires against ground. **The line that idles high
is the sensor's TX** — a UART rests high. This also settles the logic level in
the same measurement.

---

## 2. Commands

| Command | Effect |
|---|---|
| `startlds$` | motor on, data stream begins |
| `stoplds$` | motor off |

ASCII, **no line terminator**. Without `startlds$` the sensor sends nothing at
all — a silent sensor is not necessarily a wiring fault.

### There are no other commands

Probed on a quiet channel (motor stopped): `$`, `?$`, `help$`, `version$`,
`getversion$`, `info$`, `status$`, `getinfo$`, `reset$`, `lds$`, `getlds$`, and
the deliberate nonsense `xyzq$`. **Every one returned 0 bytes.**

The nonsense probe is the informative one: since even an invalid command
produces no error response, **the sensor has no back-channel**. There is nothing
to query and no way to discover a command set by probing.

### PWM motor control does not work on this unit

[opravdin/lds-006-reverse-engineering](https://github.com/opravdin/lds-006-reverse-engineering)
documents a frame `!` `<pwm>` `!` (0x21, value, 0x21). Measured across the full
range, it does not move the rotation speed:

| PWM value | measured |
|---:|---:|
| — (normal) | 300.5 rpm |
| 60 | 301.4 rpm |
| 120 | 302.8 rpm |
| 180 | 300.4 rpm |
| 255 | 303.6 rpm |

It also does not start the motor from standstill. That source itself calls the
protocol deprecated; this firmware appears to have dropped it entirely.

> **Unrecognised bytes can wedge the sensor's command reader.** After the PWM
> probes, `startlds$` had no effect for 20 seconds — the sensor was most likely
> still waiting for the closing `!` of an unfinished frame and swallowed the
> command. The cure is `stoplds$` followed by `startlds$`.

### Not present on this unit

A `5A A5` startup frame is described by
[jentsch.io](https://www.jentsch.io/lds-006-lidar-sensor-reverse-engineering/).
Zero occurrences across spin-up, steady state and spin-down.

---

## 3. Frame format

Fixed **22 bytes**. The framing is XV-11 (Neato) compatible; **the interpretation
is not** — see §4.

| Byte | Content |
|---|---|
| 0 | `0xFA` start byte |
| 1 | index |
| 2–3 | rotation speed, little endian |
| 4–7 | sample 0 |
| 8–11 | sample 1 |
| 12–15 | sample 2 |
| 16–19 | sample 3 |
| 20–21 | checksum, little endian |

### Index

| Range | Meaning |
|---|---|
| `0xA0`–`0xF9` | angle frame — 90 frames per revolution |
| `0xFA`–`0xFF` | **status frame**, no measurements (see §5) |

Angle of sample *k* in an angle frame:

```
degrees = (byte1 - 0xA0) * 4 + k        # 0..359
```

> **Synchronise on two bytes, not one.** `0xFA` occurs freely inside distance
> data. Only `0xFA` followed by an index ≥ `0xA0` starts a frame.

### Sample (4 bytes)

| Byte | Content |
|---|---|
| 0 | distance low byte |
| 1 | bit 7 = invalid, bit 6 = weak signal, bits 5–0 = distance high bits |
| 2–3 | signal strength ("reflectance"), little endian, unit unknown |

```
distance_mm = ((byte1 & 0x3F) << 8) | byte0
```

If bit 7 is set, byte 0 carries an **error code** instead of a distance.

*Bit 6 as "weak signal" is inherited from the XV-11 convention. It was never
observed on a real measurement here. It does appear — but only as part of the
filler pattern below, where `0x77` happens to have bit 6 set. Treat it as
unverified.*

> **Strength 0 is never a measurement.** Across all recordings, every genuine
> reading has a non-zero strength (788 checked). An unflagged sample with
> strength 0 is the filler described in §5 — see the trap there.

### Error codes

Exactly two, across all recordings:

| Code | Meaning | Evidence |
|---|---|---|
| `0x88` | target closer than the minimum range | valid neighbours median 148 mm |
| `0x99` | no return | valid neighbours median 493 mm |

The distance association is suggestive, not proven (51 vs 9 mixed angles). The
clean experiment would be to withdraw a wall from 5 cm to 3 m and watch which
code appears where.

---

## 4. Two deviations from the XV-11

**These are why off-the-shelf XV-11 drivers produce nothing on this sensor.**

### 4.1 The checksum is a plain byte sum

```c
uint16_t chk = 0;
for (int i = 0; i < 20; i++) chk += frame[i];   /* compare to frame[20] | frame[21]<<8 */
```

Measured over 371 recorded frames:

| Algorithm | Match rate |
|---|---|
| **sum of bytes 0..19, 16-bit** | **371 / 371 — 100 %** |
| XV-11 shift-add over 10 words | 0 / 371 — 0 % |
| word sum, byte sum from 1 or 2, XOR | 0 / 371 each |

An XV-11 driver therefore rejects **every** frame while appearing to sync
correctly — the failure mode that costs the most time to diagnose.

Independently corroborated by jentsch.io ("arithmetic sum of all bytes in the
frame except the check digit").

### 4.2 The speed field divides by 100, not 64

Raw values 29805–30497, mean 30121.

| Divisor | Result | |
|---:|---:|---|
| `/64` (XV-11) | 470.6 rpm | contradicts the data rate |
| **`/100`** | **301.2 rpm** | consistent |

Cross-check independent of the field's contents: 9900 byte/s ÷ 22 bytes ÷ 90
frames = **5.0 rev/s = 300 rpm**. This rules out `/64`.

*No prose source we found states this divisor, but it does appear in code: the
[LDS006ESP32](https://github.com/lemarsienvoyageur/LDS-006-ESP32) Arduino library
divides by 100, derived from
[Aluminum-z's STM32 driver](https://github.com/Aluminum-z/Laser-Radar-LDS-006-Drive-Test).*

---

## 5. Status frames — index outside the angle range

During spin-up and spin-down the sensor emits frames with an index **outside**
`0xA0`–`0xF9`. Only `0xFB` was ever observed.

They carry **no measurements**. All four sample slots hold the constant pattern
`77 77 00 00`. The speed field carries the **current ramp speed**:

```
spin-up:    210 -> 8048 -> 11396 -> ... -> 35218     each value exactly 6x
spin-down:  22814 -> 22377 -> ... -> 4551
steady state: none
```

> **`77 77 00 00` is not an error code**, and branching on the frame type is not
> enough to keep it out.
>
> Bit 7 of the second byte is *clear*, so it reads as a **valid** measurement of
> 14199 mm. The symptom is a ring of phantom walls at 14 m on every spin-up.
>
> The subtlety: the filler is **not confined to status frames**. While spinning
> up, the sensor is already turning and counting angles but not yet measuring, so
> the same pattern arrives in ordinary **angle frames with index < `0xFA`**. We
> shipped this bug ourselves — skipping status frames looked sufficient, and the
> recorded capture happened not to expose it because the phantom values were
> overwritten before the revolution boundary.
>
> The reliable rule is the strength field: **reject any unflagged sample whose
> strength is 0.** No genuine reading has strength 0 (788 checked), so this costs
> nothing and catches the filler wherever it appears.

### Deriving the operating state

Status frames make three situations distinguishable that otherwise all look
identical — namely "no data arriving":

| State | Condition |
|---|---|
| spinning | angle frame newer than 500 ms |
| spinning up | only status frames, speed rising |
| spinning down | only status frames, speed falling |
| stopped | nothing for 500 ms |

---

## 6. Measured characteristics

| | |
|---|---|
| Rotation | ~300 rpm = 5 rev/s |
| Points | 360 per revolution, 1° |
| Data rate | 9900 byte/s |
| Range | 107 mm … 7337 mm observed |
| Valid returns | ~88 % in a normal room; ~33 % on a desk |
| Checksum errors | 5 in 118 615 frames (0.004 %) |

Signal strength falls with distance, but **spreads by a factor of ~100 at the
same distance** — that residual is surface information (dark vs. light, grazing
vs. perpendicular):

| Distance | median | range |
|---|---:|---|
| 0–300 mm | 146 | 51…258 |
| 1500–2500 mm | 114 | 4…443 |
| 2500–4000 mm | 90 | 2…228 |
| 4000–8000 mm | 66 | 2…104 |

---

## 7. Mounting: rotation direction

Viewed from above the sensor scans **counter-clockwise**, so the protocol's index
direction and a top-down view are opposed. Mapping to a vehicle convention of
0° forward, angles increasing clockwise:

```
angle = (360 - raw_angle) % 360
```

`(360 - a) % 360` leaves 0° fixed, so a zero direction found once survives
toggling the mirror.

> Getting this wrong produces a *plausible* mirror image: distances and room
> shape look right, only the rotation direction is wrong. While driving it shows
> up only as a map that fails to close — and suspicion falls on odometry first.
> Verify by pairing two scans of the same scene across the change: we measured a
> **0.4 %** median relative deviation when paired mirrored, against **39.2 %**
> unmirrored.

---

## 8. ROS 2 notes

Publishing as `sensor_msgs/LaserScan` requires two conversions that are easy to
get wrong:

**Direction.** REP 103 has x forward, y left, yaw counter-clockwise. The vehicle
convention above is clockwise, so `ranges` must be reversed.

**Dropouts.** REP 117 defines three distinct values, and the sensor's two error
codes map onto them exactly:

| Code | `LaserScan` range |
|---|---|
| `0x88` closer than measurable | `-inf` |
| `0x99` no return | `+inf` |
| anything else | `nan` |

Emitting `0.0` for a dropout is wrong — 0 is a *valid* reading close to the
sensor. Reporting `0x99` as 0 builds obstacles at the origin; reporting `0x88`
as "nothing" paints free space where an obstacle is within reach.

Also fill `scan_time` and `time_increment`: one revolution takes 200 ms, so a
robot turning at 30°/s smears a scan by 6°. `slam_toolbox` does not de-skew.

---

## 9. Existing implementations

Read before writing another one. Both were checked in August 2026.

**[lemarsienvoyageur/LDS-006-ESP32](https://github.com/lemarsienvoyageur/LDS-006-ESP32)**
(Arduino, GPL-3.0) — starts and stops the motor correctly and confirms the `/100`
speed divisor. Four limitations worth knowing before adopting it: it does not
verify the checksum at all; its receive buffer is 15 bytes against a 22-byte
frame, so only **sample 0 of each packet** survives and you get 90 points per
revolution instead of 360; the invalid-data flag is never tested, so dropouts
vanish silently (they are filtered only incidentally, by a `strength > 0` test);
and status frames are not handled, saved by the same accident. Its guard
`0 < an < 360` parses as `(0 < an) < 360` and is always true. The README states
8N2 and centimetres; the code uses 8N1 and yields millimetres.

**[manuelilg/lds006_lidar_driver](https://github.com/manuelilg/lds006_lidar_driver)**
(C++, ROS 1) — untouched since 2023, single-line README, not evaluated further.

## 10. Hardware (from third-party teardowns, not verified here)

Controller **GD32F130P6F6**; UART on PA2/PA3 through 100 Ω resistors; motor PWM
on PA4; SWD port accessible. Power is inductively coupled to the spinning
assembly and data returns via an IR LED and receiver.
Source: [0x416c6578](https://0x416c6578.github.io/posts/005-LDS-006-Hacking.html).

---

## Licence

MIT. Corrections and additions welcome — particularly measurements from other
units, since firmware variants clearly exist (see §2).
