"""
LiDAR interface.

Expected input:
    MAVLink OBSTACLE_DISTANCE messages.

The challenge specifies a 16-beam LiDAR. This class converts the
incoming MAVLink distance array into 16 angular measurements.

IMPORTANT:
The exact challenge LiDAR interface has not been provided, so this
file is intentionally isolated from the rest of the controller.
"""


import math
import time
from typing import List, Optional


class Lidar16:
    def __init__(self, vehicle, beam_count: int = 16,
                 max_distance: float = 30.0):
        self.vehicle = vehicle
        self.beam_count = beam_count
        self.max_distance = max_distance

        self.distances: List[float] = [
            max_distance
        ] * beam_count

        self.angles: List[float] = [
            -math.pi + (2.0 * math.pi * i / beam_count)
            for i in range(beam_count)
        ]

        self.last_update = 0.0

    def update(self, timeout: float = 0.0) -> bool:
        """
        Read the newest OBSTACLE_DISTANCE message.

        Returns True if new LiDAR data was received.
        """

        msg = self.vehicle.recv_match(
            type="OBSTACLE_DISTANCE",
            blocking=(timeout > 0),
            timeout=timeout
        )

        if msg is None:
            return False

        raw = list(msg.distances)

        if not raw:
            return False

        # MAVLink distances are centimetres.
        min_cm = getattr(msg, "min_distance", 0)
        max_cm = getattr(msg, "max_distance", 0)

        sensor_max = self.max_distance

        if max_cm and max_cm > 0:
            sensor_max = min(max_cm / 100.0, self.max_distance)

        # angle_offset/increment are normally degrees.
        angle_offset_deg = getattr(msg, "angle_offset", 0.0)
        increment_deg = getattr(msg, "increment", 0.0)

        if increment_deg <= 0:
            increment_deg = 360.0 / len(raw)

        values = []

        for i, value_cm in enumerate(raw):
            if value_cm is None:
                continue

            try:
                value_cm = float(value_cm)
            except (TypeError, ValueError):
                continue

            if value_cm <= 0:
                continue

            if min_cm and value_cm < min_cm:
                continue

            distance = value_cm / 100.0

            if distance > sensor_max:
                distance = sensor_max

            angle_deg = angle_offset_deg + i * increment_deg

            # Normalize angle to [-180, 180)
            angle_deg = ((angle_deg + 180.0) % 360.0) - 180.0

            values.append(
                (
                    math.radians(angle_deg),
                    distance
                )
            )

        if not values:
            return False

        # Convert arbitrary sensor samples into exactly beam_count
        # angular sectors.
        sector_values: List[List[float]] = [
            [] for _ in range(self.beam_count)
        ]

        for angle, distance in values:

            # -pi -> pi mapped to sector 0 -> beam_count-1
            normalized = (angle + math.pi) / (2.0 * math.pi)

            index = int(
                normalized * self.beam_count
            ) % self.beam_count

            sector_values[index].append(distance)

        new_distances = []

        for sector in sector_values:
            if sector:
                # Conservative choice: closest obstacle.
                new_distances.append(min(sector))
            else:
                new_distances.append(sensor_max)

        self.distances = new_distances

        self.angles = [
            -math.pi
            + (2.0 * math.pi * i / self.beam_count)
            for i in range(self.beam_count)
        ]

        self.last_update = time.monotonic()

        return True

    def age(self) -> float:
        """Return seconds since the last valid LiDAR update."""
        if self.last_update == 0:
            return float("inf")

        return time.monotonic() - self.last_update

    def valid(self, timeout: float) -> bool:
        """Return True when LiDAR data is recent enough."""
        return self.age() <= timeout

    def get(self) -> List[float]:
        """Return the current 16-beam distance array."""
        return list(self.distances)

    def sector(self, center_deg: float,
               width_deg: float = 30.0) -> float:
        """
        Return the closest obstacle within an angular sector.

        Angle convention:
            0°   = forward
            +90° = right
            -90° = left
            180° = rear
        """

        center = math.radians(center_deg)
        half_width = math.radians(width_deg / 2.0)

        candidates = []

        for angle, distance in zip(
            self.angles,
            self.distances
        ):
            difference = math.atan2(
                math.sin(angle - center),
                math.cos(angle - center)
            )

            if abs(difference) <= half_width:
                candidates.append(distance)

        if not candidates:
            return self.max_distance

        return min(candidates)

    def front(self) -> float:
        return self.sector(0, 45)

    def front_left(self) -> float:
        return self.sector(-45, 45)

    def front_right(self) -> float:
        return self.sector(45, 45)

    def left(self) -> float:
        return self.sector(-90, 60)

    def right(self) -> float:
        return self.sector(90, 60)

    def rear(self) -> float:
        return self.sector(180, 60)
