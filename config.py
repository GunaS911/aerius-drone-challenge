"""
Configuration for the Aerius Drone Functionality Challenge.

All distances are in metres.
Velocity is in metres/second.
"""

from dataclasses import dataclass


@dataclass
class Config:
    # MAVLink connection
    connection_string: str = "udp:127.0.0.1:14550"

    # Goal position relative to the EKF/local origin.
    # CHANGE THESE TO THE ACTUAL CHALLENGE DESTINATION.
    goal_x: float = 20.0
    goal_y: float = 0.0

    # Flight altitude
    takeoff_altitude: float = 5.0

    # Goal tolerance
    goal_tolerance: float = 1.5

    # Maximum horizontal speed
    max_speed: float = 2.0

    # Reduced speed when near obstacles
    slow_speed: float = 0.8

    # Safety distances
    emergency_distance: float = 1.2
    obstacle_distance: float = 2.5
    clear_distance: float = 4.0

    # How long to hover after reaching destination
    goal_hold_time: float = 2.0

    # Control loop
    control_rate_hz: float = 10.0

    # Expected LiDAR beams
    lidar_beams: int = 16

    # Maximum valid LiDAR distance
    lidar_max_distance: float = 30.0

    # If no valid LiDAR data has arrived after this period,
    # the controller stops instead of blindly flying.
    lidar_timeout: float = 1.0
