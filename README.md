# Decentralized Point Simulation

## Project Overview
This project implements a **decentralized control algorithm** for simulating the coordinated movement of multiple agents (points) in a 2D space. Inspired by bio-inspired algorithms and swarm intelligence, the agents interact locally with their neighbors to achieve a global formation or task without a centralized controller.

## Objectives
- Simulate multi-agent systems with decentralized control.
- Implement simple coordination rules based on relative positions.
- Visualize the evolution of agents' positions over time.
- Study emergent behaviors like formation, aggregation, or dispersion.

## Features
- Initialization of a configurable number of agents with random positions.
- Decentralized control law: each agent updates its position based on its neighbors.
- Visualization of trajectories and final positions.
- Adjustable parameters for distance thresholds, control gains, and simulation steps.

## Files
- `descentralized_point_simulation.py`: Main simulation script implementing the decentralized algorithm and visualization.
- `mecanum_base_node.py`: Robot communication node based on ROS1 software.


## Requirements
- Python 3.x
- Libraries:
  - `numpy`
  - `matplotlib`

Install dependencies with:
```bash
pip install -r requirements.txt
