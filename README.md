# Proyecto Eléctrico - Decentralized Point Control for Mobile Robots

This repository contains a set of ROS 1 nodes developed on Ubuntu 20.04 to implement and simulate a decentralized point control algorithm for mobile robots using differential drive and mecanum wheels. The system includes both real-world and simulation modules for trajectory tracking and odometry data extraction.

---

## 🧠 Project Overview

The main objective is to guide a mobile robot along a predefined trajectory using a decentralized control approach. The robot calculates its next position based on its current odometry and a list of waypoints, adjusting its wheel velocities accordingly.

---

## 📁 Folder Structure

```
├── descentralized_point_real.py       # Real-world control for differential robot using encoders
├── descentralized_point_simulation.py # Simulated control using Gazebo and Odometry
├── get_info.py                        # Node to extract and save odometry data to CSV
├── mecanum_base_node.py               # Base node to handle Roboclaw motor control and encoders
```

---

## ⚙️ Nodes

### `descentralized_point_real.py`
Executes the decentralized point algorithm for a physical robot using PWM motor commands. Subscribes to wheel speed and publishes motor commands to Roboclaw interfaces.

### `descentralized_point_simulation.py`
Simulates the decentralized point control using the TurtleBot3 in Gazebo. Subscribes to `/odom` and publishes velocity commands to `/cmd_vel`.

### `get_info.py`
Logs robot odometry data (x, y, θ) to a CSV file for later analysis.

### `mecanum_base_node.py`
Handles communication with two Roboclaw motor controllers and publishes encoder and wheel speed data. Also supports optional PID control for velocity regulation.

---

## 🧪 Example Trajectories

- Square trajectory (`trayectoria_cudrado.csv`)
- Circular trajectory (`trayectoria_circulo.csv`)
- Zig-zag path (`trayectoria_zigzag.csv`)
- Lemniscate trajectory (`trayectoria_lemniscata.csv`)

These must be placed in the correct directory (`turtlebot3_gazebo/descentralized_point/trayectorias/`) and formatted as CSV files with header row `x,y`.

---

## 📦 Requirements

- ROS 1 (Noetic preferred)
- `mecanumrob_common` package
- Roboclaw motor controllers
- TurtleBot3 simulation environment (Gazebo)

---

## 🚀 How to Run

### Real Robot

```bash
roscore
rosrun mecanumrob_roboclaw mecanum_base_node.py
rosrun mecanumrob_roboclaw descentralized_point_real.py
```

### Simulation

```bash
roscore
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
rosrun turtlebot3_gazebo descentralized_point_simulation.py
```

### Odometry Logger

```bash
rosrun turtlebot3_gazebo get_info.py
```

---

## 📌 Notes

- The control logic computes a reference velocity and orientation error using proportional and derivative gains.
- A conversion matrix (`B`) is used for inverse kinematics to get wheel velocities.
- Motor signals are limited to prevent overflow (e.g., PWM max = 130).
- The trajectory index advances dynamically based on the distance to the current goal.

---

## 📚 Author

Santiago Herra Castro  
Electrical Engineering Student at Universidad de Costa Rica
Focus: Mobile Robotics with ROS on Ubuntu 20.04
email: santicastro60@gmail.com

---

## 📄 License

MIT License. See `LICENSE` file.