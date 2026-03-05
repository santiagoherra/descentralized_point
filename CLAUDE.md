# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 1 (Noetic) Python package implementing **decentralized point control** for mobile robots (mecanum-wheeled real hardware and TurtleBot3 simulation). The control algorithm guides robots along predefined CSV trajectories using kinematic inverse matrices with PD feedback.

**Platform:** Ubuntu 20.04, ROS 1 Noetic, Python 3

## Running the Project

### Real Robot
```bash
roscore &
rosrun mecanumrob_roboclaw mecanum_base_node.py
rosrun mecanumrob_roboclaw descentralized_point_real.py
```

### Simulation
```bash
roscore &
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
python3 descentralized_point_simulation.py
```

### SLAM
```bash
roslaunch mecanumrob_roboclaw slam_karto.launch
```

### Tests
```bash
pytest test/test.py -v
```

No compilation step — this is a pure Python ROS package.

## Architecture

### Node Graph

```
[Real Robot]
mecanum_base_node.py  ←── motor/NE_pwm, motor/NW_pwm, motor/SW_pwm, motor/SE_pwm (std_msgs/Int8)
                      ←── cmd_wheels (mecanumrob_common/WheelSpeed)
                      ──► /odom (nav_msgs/Odometry)
                      ──► wheel_speed (mecanumrob_common/WheelSpeed)
                      ──► encoders (mecanumrob_common/EncTimed)
                      ──► TF: odom → base_link

descentralized_point_real.py  ←── wheel_speed
                              ──► motor/SW_pwm, motor/SE_pwm

[Simulation]
descentralized_point_simulation.py  ←── /odom (nav_msgs/Odometry from Gazebo)
                                    ──► /cmd_vel (geometry_msgs/Twist)
```

### Key Files

| File | Role |
|------|------|
| `mecanum_base_node.py` | Hardware driver for Roboclaw motor controllers. Reads encoders, runs PID velocity loop, publishes `/odom` and TF `odom→base_link`. Main loop at 60 Hz. Robot is a **differential-drive tricycle**: two motorized mecanum wheels at rear (SW=phi[2], SE=phi[3]) and a passive ball caster at front. Kinematics are differential, not holonomic. |
| `descentralized_point_real.py` | Control node for real robot. Reads CSV trajectory, runs PD control law, outputs PWM commands to two drive wheels (SW, SE only — differential drive subset). |
| `descentralized_point_simulation.py` | Same control algorithm for TurtleBot3 in Gazebo. Reads `/odom` directly, outputs `/cmd_vel` Twist. Supports trajectory looping. |
| `get_info.py` | Utility: subscribes to `/odom`, logs x/y/theta to CSV for analysis. |
| `slam_karto/` | Launch + config for slam_karto SLAM integration with RPLIDAR. |
| `trayectorias/` | CSV trajectory files (`x,y` columns). Available shapes: circle, square, lemniscate, zigzag, hourglass. |
| `test/test.py` | pytest unit tests — validates trajectory loading, control math, and timing performance. |

### Control Algorithm (both real and simulation)

The decentralized point law computes wheel commands as:

```
u = B_inv @ (KV * [dx, dy] + Kp * [ex, ey])
```

Where:
- `[dx, dy]` = velocity direction from trajectory derivative
- `[ex, ey]` = position error between current control point and trajectory target
- `B` = inverse kinematics matrix (wheel geometry)
- Waypoint advancement is distance-threshold based (not time-based)

### Odometry (`mecanum_base_node.py`)

- Encoder counts → wheel speeds (rad/s) via `ppv` (pulses/rev, default 3415)
- Wheel speeds → robot velocity via **differential drive** kinematics (2-wheel tricycle):
  - `vx = (r/2) * (w_SE + w_SW_corrected)` — SW motor is physically inverted (`w_izq = -phi_prime[2]`)
  - `wz = (r/L) * (w_SE - w_SW_corrected)`
  - `r = 0.0505 m` (wheel_radius), `L = 0.160 m` (wheel_base), both ROS params
- Integrates position incrementally; `normalizar_angulo()` keeps theta in `(-π, π]` per REP-103
- Publishes `nav_msgs/Odometry` on `/odom` and TF broadcast `odom→base_link` via `tf.TransformBroadcaster`
- `update_odom()` is called directly in the main loop; uses `rospy.Time.now()` with its own `last_odom_time` for dt

## External Dependencies

Custom ROS messages from `mecanumrob_common` package (must be on `ROS_PACKAGE_PATH`):
- `mecanumrob_common/WheelSpeed` — four wheel angular velocities
- `mecanumrob_common/EncTimed` — encoder values with timestamp

Hardware: 2× Roboclaw motor controllers connected via serial (`/dev/ttyACM*`). Configured via ROS params: `port_front`, `port_back`, `baudrate`.
