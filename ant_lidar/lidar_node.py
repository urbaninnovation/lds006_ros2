#!/usr/bin/env python3
"""ROS-2-Knoten: LDS-006 als `sensor_msgs/LaserScan`.

Zwei Umrechnungen sind hier entscheidend und werden gern falsch gemacht.

**Drehsinn.** Die Bibliothek liefert die Fahrzeugkonvention: 0 Grad voraus,
Winkel im *Uhrzeigersinn* von oben gesehen. ROS erwartet nach REP 103 das
Gegenteil — x voraus, y links, Gierwinkel *gegen* den Uhrzeigersinn. `ranges`
wird deshalb umgedreht. Ohne das schließt die Karte beim Fahren nicht, und man
verdächtigt zuerst die Odometrie.

**Ausfälle.** REP 117 sieht für fehlende Messwerte drei verschiedene Werte vor,
und die beiden Fehlercodes des Sensors passen genau darauf:

| Code | Bedeutung | LaserScan |
|------|-----------|-----------|
| 0x88 | näher als messbar | `-inf` |
| 0x99 | kein Rückstreusignal | `+inf` |
| sonst | unklar | `nan` |

Ein Ausfall als `0.0` auszugeben wäre falsch: 0 ist ein *gültiger* Messwert
dicht am Sensor. Wer `0x99` als 0 meldet, baut Hindernisse in den Ursprung;
wer `0x88` als „nichts" meldet, malt Freiraum, wo ein Hindernis in Greifweite
steht.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from .lds006 import (
    ERR_NO_ECHO,
    ERR_TOO_CLOSE,
    POINTS_PER_REV,
    RANGE_MAX_M,
    RANGE_MIN_M,
    Scan,
)
from .serial_link import Lds006Serial


def range_of(sample) -> float:
    """Messwert nach REP 117 in Meter übersetzen."""
    if sample.valid:
        return sample.distance_mm / 1000.0
    if sample.error == ERR_TOO_CLOSE:
        return float("-inf")
    if sample.error == ERR_NO_ECHO:
        return float("inf")
    return float("nan")


class Lds006Node(Node):
    def __init__(self) -> None:
        super().__init__("lds006")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("mirror", True)
        self.declare_parameter("offset_deg", 0)
        self.declare_parameter("only_complete", True)
        self.declare_parameter("publish_intensities", True)

        p = self.get_parameter
        self.frame_id = p("frame_id").value
        self.only_complete = p("only_complete").value
        self.with_intensities = p("publish_intensities").value

        # Best-Effort ist für Sensordaten der Standard: ein verlorener Scan ist
        # in 200 ms ersetzt, ein erneuter Versuch käme ohnehin zu spät.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(LaserScan, "scan", qos)

        self.link = Lds006Serial(
            port=p("port").value,
            mirror=p("mirror").value,
            offset_deg=p("offset_deg").value,
        )
        self.link.start()
        self.get_logger().info(
            f"LDS-006 an {p('port').value}, Rahmen '{self.frame_id}', "
            f"Spiegelung {p('mirror').value}, Versatz {p('offset_deg').value} Grad"
        )
        self.scans = 0
        self.create_timer(10.0, self._report)

    def _report(self) -> None:
        pa = self.link.parser
        self.get_logger().info(
            f"{self.scans} Scans, {pa.frames_ok} Rahmen ok, "
            f"{pa.frames_bad} Prüfsummenfehler, {pa.resyncs} Resync"
        )

    def publish(self, scan: Scan) -> None:
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        step = 2.0 * math.pi / POINTS_PER_REV
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - step
        msg.angle_increment = step
        msg.scan_time = 60.0 / scan.rpm if scan.rpm > 1.0 else 0.2
        msg.time_increment = msg.scan_time / POINTS_PER_REV
        msg.range_min = RANGE_MIN_M
        msg.range_max = RANGE_MAX_M

        # Uhrzeigersinn -> gegen den Uhrzeigersinn (REP 103)
        ordered = [scan.samples[(POINTS_PER_REV - i) % POINTS_PER_REV]
                   for i in range(POINTS_PER_REV)]
        msg.ranges = [range_of(s) for s in ordered]
        if self.with_intensities:
            msg.intensities = [float(s.strength) for s in ordered]
        self.pub.publish(msg)
        self.scans += 1

    def spin_serial(self) -> None:
        for scan in self.link.scans(only_complete=self.only_complete):
            self.publish(scan)
            rclpy.spin_once(self, timeout_sec=0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Lds006Node()
    try:
        node.spin_serial()
    except KeyboardInterrupt:
        pass
    finally:
        node.link.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
