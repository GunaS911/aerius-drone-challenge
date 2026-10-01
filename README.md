# Aerius Drone Functionality Challenge

Autonomous drone navigation and obstacle avoidance controller for the
Aerius Drone Functionality Challenge conducted by the UAV & Drone Club,
School of Engineering, JNU.

## Overview

The controller is designed around:

- Autonomous navigation
- MAVLink communication
- ArduPilot Guided mode
- 16-beam LiDAR obstacle sensing
- Reactive obstacle avoidance
- Velocity-based flight control
- Destination detection

The controller does not require manual flight commands once started.

## Architecture

```text
                 16-Beam LiDAR
                       |
                       v
              +----------------+
              | LiDAR Processor |
              +----------------+
                       |
                       v
              +----------------+
              | Obstacle Logic |
              +----------------+
                  /         \
                 /           \
        Clear path          Obstacle
             |                  |
             v                  v
      Goal navigation     Choose safer side
             |                  |
             +--------+---------+
                      |
                      v
              MAVLink Velocity
                      |
                      v
                ArduPilot
                      |
                      v
                    Drone
