# PID Autonomous Navigation

A Python simulation of a differential-drive robot that plans collision-free paths with A*, converts them into tracking waypoints, and follows them using a grid-search-tuned PID heading controller. The system is evaluated through step-response analysis, collision and clearance validation, and a multi-scenario robustness sweep.

The project investigates a single question: how well does a tuned PID controller perform when it is driven by an autonomous planner rather than hand-placed waypoints?

## Table of Contents

- [Overview](#overview)
- [Results Summary](#results-summary)
- [Navigation Pipeline](#navigation-pipeline)
- [Project Structure](#project-structure)
- [Methodology](#methodology)
- [Getting Started](#getting-started)
- [Running Experiments](#running-experiments)
- [Evaluation Metrics](#evaluation-metrics)
- [Tech Stack](#tech-stack)
- [Limitations](#limitations)
- [Future Work](#future-work)
- [Author](#author)
- [License](#license)

## Overview

The robot is modeled as a differential-drive vehicle operating in a 2D environment with obstacles. The codebase separates planning, control, vehicle dynamics, and analysis into independent modules, so the planner or controller can be replaced without changing the rest of the system.

Key capabilities:

- Differential-drive kinematic simulation
- Closed-loop PID heading control with derivative-kick protection
- Systematic grid-search PID tuning across 150 gain combinations
- Step-response performance analysis
- Waypoint-based autonomous navigation
- Grid-based A* path planning with obstacle awareness
- Path simplification into tracking waypoints
- Collision and obstacle-clearance validation
- Robustness testing across multiple speeds, layouts, and disturbance conditions
- Quantitative metrics exported to CSV

## Results Summary

| Area | Result |
|---|---|
| PID tuning | 150 combinations evaluated; selected Kp = 8.0, Ki = 0.2, Kd = 0.2 |
| Step response | 0.320 s rise time, 0.36% overshoot, 0.540 s settling time, 0.265 deg steady-state error |
| Waypoint navigation | 4 of 4 waypoints reached in 10.15 s, final distance 0.099 m |
| A* navigation | 61 raw path points reduced to 9 waypoints, goal reached, collision free |
| Robustness | 12 of 12 scenarios successful, 0 collisions, 0 planning failures |
| Final position error | 0.091 m average, 0.100 m maximum across robustness scenarios |

All figures are produced by the scripts in `experiments/` and can be reproduced locally.

## Navigation Pipeline

```
A* Path Planning
      |
Path Simplification
      |
Waypoint Generation
      |
Desired Heading
      |
PID Heading Controller
      |
Differential-Drive Kinematics
      |
Robot Motion
      |
Obstacle and Collision Validation
```

## Project Structure

```
PID-Autonomous-Navigation/
├── analysis/
│   └── metrics.py                # Step-response and navigation metrics
├── controllers/
│   ├── pid_controller.py         # PID implementation
│   └── heading_controller.py     # Heading error to angular velocity
├── navigation/
│   ├── astar_planner.py          # Grid-based A* search
│   ├── obstacle_avoidance.py     # Clearance and collision validation
│   └── waypoint_follower.py      # Waypoint switching logic
├── vehicle/
│   └── differential_drive.py     # Differential-drive kinematics
├── experiments/
│   ├── pid_step_response.py
│   ├── pid_tuning.py
│   ├── heading_control.py
│   ├── waypoint_navigation.py
│   ├── obstacle_navigation.py
│   └── robustness_test.py
├── results/                      # Experiment output (CSV and plots)
├── main.py                       # Final demonstration
├── test_robot.py
└── requirements.txt
```

## Methodology

### PID Controller

The controller implements the standard formulation:

```
u(t) = Kp * e(t) + Ki * integral(e(t) dt) + Kd * de(t)/dt
```

- Kp sets the proportional response to heading error
- Ki reduces steady-state error
- Kd responds to the rate of change of the error

The first control iteration is handled separately to prevent derivative kick.

### PID Tuning

Gains were selected by exhaustive search rather than manual adjustment.

| Gain | Values evaluated |
|---|---|
| Kp | 2, 3, 4, 5, 6, 8 |
| Ki | 0.2, 0.5, 1.0, 1.5, 2.0 |
| Kd | 0.2, 0.5, 0.8, 1.2, 1.5 |

Each of the 150 combinations was scored on rise time, overshoot, settling time, and steady-state error. The selected configuration was Kp = 8.0, Ki = 0.2, Kd = 0.2.

| Metric | Value |
|---|---|
| Rise time | 0.320 s |
| Overshoot | 0.36% |
| Settling time | 0.540 s |
| Steady-state error | 0.265 deg |

The selected Kp lies at the upper bound of the search range, so the result is optimal for this search space and objective rather than globally. Full results are in `results/pid_tuning_results.csv`.

### Waypoint Navigation

For each waypoint, the system:

1. Computes the vector from the robot to the waypoint and converts it to a desired heading.
2. Computes the heading error and applies the PID controller.
3. Converts the resulting linear and angular velocity into left and right wheel velocities.
4. Updates the differential-drive state.
5. Advances to the next waypoint once inside the tolerance region.

| Waypoint | Time reached |
|---|---|
| 1 | 2.15 s |
| 2 | 4.98 s |
| 3 | 7.19 s |
| 4 | 10.15 s |

Final distance to the last waypoint: 0.099 m.

### A* Obstacle-Aware Planning

The environment is represented as a grid. A* generates a collision-free path, which is simplified so the controller tracks a small set of turning points instead of every grid cell.

| Item | Value |
|---|---|
| Raw path points | 61 |
| Simplified waypoints | 9 |
| Obstacles | 2 |
| Final position | (5.92, 4.94) m |
| Target position | (6.00, 5.00) m |
| Final distance | 0.100 m |
| Minimum obstacle clearance | 0.131 m |
| Status | All waypoints reached, collision free |

### Robustness Testing

The robustness experiment runs 12 scenarios combining different robot speeds, navigation layouts, and small simulated disturbances.

| Metric | Result |
|---|---|
| Successful scenarios | 12 of 12 (100%) |
| Collisions | 0 |
| Planning failures | 0 |
| Average final error | 0.091 m |
| Maximum final error | 0.100 m |
| Minimum clearance | 0.175 m |
| Average travel time | 10.42 s |

Raw data is in `results/robustness_results.csv`.

## Getting Started

Requires Python 3.12.

```bash
git clone https://github.com/dakshchaudhary16/PID-Autonomous-Navigation.git
cd PID-Autonomous-Navigation

python3 -m venv venv
source venv/bin/activate          # Windows: venv\Scripts\activate

pip install -r requirements.txt
```

Run the main demonstration, which displays the robot trajectory, planned path, tracking waypoints, goal position, and heading tracking error:

```bash
python main.py
```

## Running Experiments

| Experiment | Command |
|---|---|
| PID step response | `python -m experiments.pid_step_response` |
| PID parameter tuning | `python -m experiments.pid_tuning` |
| Heading control | `python -m experiments.heading_control` |
| Waypoint navigation | `python -m experiments.waypoint_navigation` |
| Obstacle-aware navigation | `python -m experiments.obstacle_navigation` |
| Robustness testing | `python -m experiments.robustness_test` |

## Evaluation Metrics

| Metric | Definition |
|---|---|
| Rise time | Time for the response to move from 10% to 90% of the target |
| Overshoot | Percentage by which the response exceeds the target |
| Settling time | Time required to remain within the 2% target band |
| Steady-state error | Final difference between target and measured response |
| Final distance | Distance between robot and target at the end of a run |
| Obstacle clearance | Minimum distance maintained from an obstacle |
| Success rate | Percentage of scenarios completed without collision |
| Travel time | Time required to complete the planned navigation |

## Tech Stack

| Tool | Purpose |
|---|---|
| Python 3.12 | Core language |
| NumPy | Numerical computation |
| SciPy | Scientific computing |
| Pandas | Experiment data and CSV analysis |
| Matplotlib | Simulation visualization |

## Limitations

This project is a simulation and does not represent a physical robot. The model does not include:

- Wheel slip
- Motor dynamics
- Encoder quantization
- Sensor latency
- Actuator saturation beyond the simulated limits
- Real-world localization error
- Dynamic obstacles

The robustness results describe behavior within the defined scenarios only. Twelve scenarios is a small sample, so a 100% success rate should be read as encouraging evidence, not as a guarantee of physical-robot performance.

## Future Work

- Sensor simulation and noisy localization
- Dynamic obstacle avoidance
- Model Predictive Control (MPC) as a comparison against PID
- Real-time visualization
- ROS 2 integration
- Gazebo or Webots simulation
- Hardware deployment on a differential-drive platform

## Author

Daksh Chaudhary
B.Tech Computer Science, SRM Institute of Science and Technology

## License

This project is intended for educational and research purposes.