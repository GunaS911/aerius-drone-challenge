"""
Autonomous navigation controller.

Strategy:

1. Move toward the destination.
2. Continuously inspect LiDAR.
3. If the path is clear, continue toward the goal.
4. If an obstacle appears in front:
       - stop/slow down
       - compare left/right clearance
       - move toward the clearer side
5. When sufficiently close to the goal, stop.

The vehicle is controlled using MAVLink Guided-mode velocity commands.
"""


import math
import time


class AutonomousController:

    def __init__(self, vehicle, lidar, config):
        self.vehicle = vehicle
        self.lidar = lidar
        self.cfg = config

        self.avoid_direction = 1.0
        self.last_command = time.monotonic()

    # ------------------------------------------------------------
    # VEHICLE STATE
    # ------------------------------------------------------------

    def get_position(self):
        """
        Return local NED position.

        x = North
        y = East
        z = Down
        """

        msg = self.vehicle.recv_match(
            type="LOCAL_POSITION_NED",
            blocking=False
        )

        if msg is None:
            return None

        return (
            float(msg.x),
            float(msg.y),
            float(msg.z)
        )

    # ------------------------------------------------------------
    # VELOCITY COMMAND
    # ------------------------------------------------------------

    def send_body_velocity(
        self,
        forward: float,
        right: float,
        down: float = 0.0
    ):
        """
        Send velocity relative to the drone's body.

        +forward = forward
        +right   = right
        +down    = downward
        """

        self.vehicle.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,

            self.vehicle.target_system,
            self.vehicle.target_component,

            9,          # MAV_FRAME_BODY_OFFSET_NED

            0b110111000111,  # velocity only

            0, 0, 0,

            forward,
            right,
            down,

            0, 0, 0,

            0,
            0
        )

        self.last_command = time.monotonic()

    def stop(self):
        self.send_body_velocity(0.0, 0.0, 0.0)

    # ------------------------------------------------------------
    # GOAL
    # ------------------------------------------------------------

    def distance_to_goal(self, position):
        x, y, _ = position

        dx = self.cfg.goal_x - x
        dy = self.cfg.goal_y - y

        return math.sqrt(
            dx * dx + dy * dy
        )

    def goal_direction(self, position):
        """
        Calculate direction to goal in local NED coordinates.

        Returns:
            dx, dy, distance
        """

        x, y, _ = position

        dx = self.cfg.goal_x - x
        dy = self.cfg.goal_y - y

        distance = math.sqrt(
            dx * dx + dy * dy
        )

        return dx, dy, distance

    # ------------------------------------------------------------
    # OBSTACLE AVOIDANCE
    # ------------------------------------------------------------

    def obstacle_avoidance(self):
        """
        Decide body-frame velocity based on LiDAR.

        Returns:
            forward_velocity,
            right_velocity
        """

        front = self.lidar.front()
        left = self.lidar.left()
        right = self.lidar.right()

        # Emergency obstacle directly ahead.
        if front < self.cfg.emergency_distance:

            # Choose the side with more clearance.
            if left > right:
                self.avoid_direction = -1.0
            else:
                self.avoid_direction = 1.0

            return (
                0.0,
                self.avoid_direction * self.cfg.max_speed
            )

        # Obstacle approaching.
        if front < self.cfg.obstacle_distance:

            if left > right:
                self.avoid_direction = -1.0
            else:
                self.avoid_direction = 1.0

            # Move slowly forward while shifting sideways.
            return (
                self.cfg.slow_speed * 0.35,
                self.avoid_direction * self.cfg.slow_speed
            )

        # Clear path.
        return None

    # ------------------------------------------------------------
    # GOAL MOVEMENT
    # ------------------------------------------------------------

    def move_toward_goal(self, position):
        """
        Convert goal direction into a velocity command.

        NOTE:
        This baseline assumes the drone's heading is generally
        aligned with the local navigation direction.

        For a challenge environment where arbitrary yaw is expected,
        the position/heading transform should be expanded.
        """

        dx, dy, distance = self.goal_direction(
            position
        )

        if distance <= self.cfg.goal_tolerance:
            self.stop()
            return True

        # Normalize.
        magnitude = max(
            math.sqrt(dx * dx + dy * dy),
            0.001
        )

        vx = dx / magnitude
        vy = dy / magnitude

        # Scale speed according to distance.
        speed = min(
            self.cfg.max_speed,
            max(0.5, distance)
        )

        self.send_body_velocity(
            vx * speed,
            vy * speed,
            0.0
        )

        return False

    # ------------------------------------------------------------
    # MAIN CONTROL LOOP
    # ------------------------------------------------------------

    def run(self):

        print("Autonomous controller started.")
        print(
            f"Goal: X={self.cfg.goal_x:.2f}, "
            f"Y={self.cfg.goal_y:.2f}"
        )

        loop_period = (
            1.0 / self.cfg.control_rate_hz
        )

        while True:

            loop_start = time.monotonic()

            # ------------------------------------------------
            # Update LiDAR
            # ------------------------------------------------

            self.lidar.update(timeout=0.01)

            if not self.lidar.valid(
                self.cfg.lidar_timeout
            ):
                print(
                    "WARNING: LiDAR data unavailable. "
                    "Stopping vehicle."
                )

                self.stop()

                time.sleep(loop_period)

                continue

            # ------------------------------------------------
            # Position
            # ------------------------------------------------

            position = self.get_position()

            if position is None:
                self.stop()
                time.sleep(loop_period)
                continue

            # ------------------------------------------------
            # Check goal
            # ------------------------------------------------

            distance = self.distance_to_goal(
                position
            )

            if distance <= self.cfg.goal_tolerance:

                print(
                    f"Destination reached "
                    f"(distance={distance:.2f}m)"
                )

                self.stop()

                time.sleep(
                    self.cfg.goal_hold_time
                )

                return

            # ------------------------------------------------
            # Obstacle avoidance
            # ------------------------------------------------

            avoidance = (
                self.obstacle_avoidance()
            )

            if avoidance is not None:

                forward, right = avoidance

                self.send_body_velocity(
                    forward,
                    right,
                    0.0
                )

            else:

                self.move_toward_goal(
                    position
                )

            # ------------------------------------------------
            # Maintain loop frequency
            # ------------------------------------------------

            elapsed = (
                time.monotonic()
                - loop_start
            )

            remaining = (
                loop_period - elapsed
            )

            if remaining > 0:
                time.sleep(remaining)
