"""
Main entry point for the Aerius autonomous drone controller.

Usage:

    python3 main.py

or:

    python3 main.py --goal-x 20 --goal-y 5
"""


import argparse
import time

from pymavlink import mavutil

from config import Config
from lidar import Lidar16
from controller import AutonomousController


def connect_vehicle(connection_string):

    print(
        f"Connecting to vehicle: "
        f"{connection_string}"
    )

    vehicle = mavutil.mavlink_connection(
        connection_string
    )

    vehicle.wait_heartbeat()

    print(
        f"Heartbeat received. "
        f"System={vehicle.target_system}, "
        f"Component={vehicle.target_component}"
    )

    return vehicle


def set_mode_guided(vehicle):

    print("Setting GUIDED mode...")

    mode_mapping = vehicle.mode_mapping()

    if "GUIDED" not in mode_mapping:
        raise RuntimeError(
            "GUIDED mode is not available."
        )

    guided_mode = mode_mapping["GUIDED"]

    vehicle.set_mode(guided_mode)

    start = time.monotonic()

    while time.monotonic() - start < 10:

        heartbeat = vehicle.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1
        )

        if heartbeat is None:
            continue

        if (
            heartbeat.custom_mode
            == guided_mode
        ):
            print("GUIDED mode active.")
            return

    raise RuntimeError(
        "Failed to enter GUIDED mode."
    )


def arm(vehicle):

    print("Arming motors...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,

        0,

        1,      # arm
        0,

        0, 0, 0, 0, 0
    )

    vehicle.motors_armed_wait()

    print("Vehicle ARMED.")


def takeoff(vehicle, altitude):

    print(
        f"Taking off to {altitude:.1f} m..."
    )

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,

        0,

        0, 0, 0, 0,

        0,
        0,

        altitude
    )

    start = time.monotonic()

    while time.monotonic() - start < 30:

        msg = vehicle.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=1
        )

        if msg is None:
            continue

        altitude_m = (
            msg.relative_alt / 1000.0
        )

        print(
            f"\rAltitude: "
            f"{altitude_m:.2f} m",
            end=""
        )

        if altitude_m >= altitude * 0.85:
            print(
                "\nTakeoff altitude reached."
            )
            return

    print(
        "\nWarning: takeoff timeout."
    )


def land(vehicle):

    print("Landing...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_NAV_LAND,

        0,

        0, 0, 0, 0, 0, 0, 0
    )


def parse_arguments():

    parser = argparse.ArgumentParser(
        description=(
            "Aerius autonomous drone controller"
        )
    )

    parser.add_argument(
        "--connection",
        default=None,
        help=(
            "MAVLink connection string. "
            "Example: udp:127.0.0.1:14550"
        )
    )

    parser.add_argument(
        "--goal-x",
        type=float,
        default=None,
        help="Goal X/North coordinate in metres."
    )

    parser.add_argument(
        "--goal-y",
        type=float,
        default=None,
        help="Goal Y/East coordinate in metres."
    )

    parser.add_argument(
        "--altitude",
        type=float,
        default=None,
        help="Takeoff altitude in metres."
    )

    return parser.parse_args()


def main():

    args = parse_arguments()

    cfg = Config()

    if args.connection:
        cfg.connection_string = args.connection

    if args.goal_x is not None:
        cfg.goal_x = args.goal_x

    if args.goal_y is not None:
        cfg.goal_y = args.goal_y

    if args.altitude is not None:
        cfg.takeoff_altitude = args.altitude

    vehicle = None

    try:

        vehicle = connect_vehicle(
            cfg.connection_string
        )

        set_mode_guided(vehicle)

        arm(vehicle)

        takeoff(
            vehicle,
            cfg.takeoff_altitude
        )

        lidar = Lidar16(
            vehicle,
            beam_count=cfg.lidar_beams,
            max_distance=cfg.lidar_max_distance
        )

        controller = AutonomousController(
            vehicle,
            lidar,
            cfg
        )

        controller.run()

        print(
            "Navigation complete."
        )

        land(vehicle)

    except KeyboardInterrupt:

        print(
            "\nController interrupted."
        )

        if vehicle is not None:
            land(vehicle)

    except Exception as exc:

        print(
            f"\nController error: {exc}"
        )

        if vehicle is not None:
            land(vehicle)

        raise

    finally:

        if vehicle is not None:
            vehicle.close()


if __name__ == "__main__":
    main()
