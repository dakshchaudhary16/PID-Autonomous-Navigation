# PID Autonomous Navigation

A Python-based simulation of an autonomous differential-drive robot combining closed-loop PID control, waypoint navigation, A* path planning, obstacle avoidance, and robustness testing.

The project evaluates how a tuned PID controller can be integrated with autonomous path planning to guide a simulated mobile robot through obstacle-constrained environments.

---

## Project Overview

The system models a differential-drive autonomous robot operating in a 2D environment.

The navigation pipeline is:

```text
A* Path Planning
       ↓
Path Simplification
       ↓
Waypoint Generation
       ↓
Desired Heading
       ↓
PID Heading Controller
       ↓
Differential-Drive Kinematics
       ↓
Robot Motion
       ↓
Obstacle / Collision Validation

The project also includes systematic PID parameter tuning and robustness experiments across different robot speeds and disturbance conditions.

Key Features
Differential-drive robot kinematic simulation
Closed-loop PID heading control
Systematic grid-search PID tuning
Step-response performance analysis
Waypoint-based autonomous navigation
A* grid-based obstacle-aware path planning
Path simplification into tracking waypoints
Obstacle clearance and collision validation
Robustness testing across multiple speeds and layouts
Quantitative navigation metrics and CSV results
Visualization of trajectories, tracking error, and obstacle clearance
Technology Stack
Python 3.12
NumPy — numerical computation
SciPy — scientific computing
Pandas — experiment data and CSV analysis
Matplotlib — simulation visualization
A* — grid-based path planning
PID Control — closed-loop heading control
Project Structure
PID-Autonomous-Navigation/
│
├── analysis/
│   ├── __init__.py
│   └── metrics.py
│
├── controllers/
│   ├── __init__.py
│   ├── heading_controller.py
│   └── pid_controller.py
│
├── experiments/
│   ├── __init__.py
│   ├── heading_control.py
│   ├── obstacle_navigation.py
│   ├── pid_step_response.py
│   ├── pid_tuning.py
│   ├── robustness_test.py
│   └── waypoint_navigation.py
│
├── navigation/
│   ├── __init__.py
│   ├── astar_planner.py
│   ├── obstacle_avoidance.py
│   └── waypoint_follower.py
│
├── results/
│   ├── obstacle_clearance_robustness.png
│   ├── pid_tuning_results.csv
│   ├── robustness_results.csv
│   └── success_rate_vs_speed.png
│
├── vehicle/
│   ├── __init__.py
│   └── differential_drive.py
│
├── main.py
├── test_robot.py
├── requirements.txt
└── .gitignore
PID Controller Design

The controller uses the standard PID formulation:

u(t) = Kp e(t) + Ki ∫e(t)dt + Kd de(t)/dt

where:

Kp controls the proportional response
Ki reduces steady-state error
Kd responds to changes in the error

The controller includes derivative-kick protection by handling the first control iteration separately.

Systematic PID Tuning

Instead of selecting PID gains manually, the project evaluates multiple combinations of:

Kp ∈ {2, 3, 4, 5, 6, 8}

Ki ∈ {0.2, 0.5, 1.0, 1.5, 2.0}

Kd ∈ {0.2, 0.5, 0.8, 1.2, 1.5}

This produces:

150 total PID combinations

The tuning experiment evaluates:

Rise time
Percentage overshoot
Settling time
Steady-state error

The selected configuration from the defined search space and objective was:

Kp = 8.0
Ki = 0.2
Kd = 0.2

Measured step-response performance:

Rise time:           0.320 s
Overshoot:           0.36 %
Settling time:       0.540 s
Steady-state error:  0.265°

The results are stored in:

results/pid_tuning_results.csv
Autonomous Waypoint Navigation

The robot follows a sequence of target positions.

For every waypoint:

Calculate the vector from the robot to the waypoint.
Convert that vector into a desired heading.
Calculate heading error.
Apply the tuned PID controller.
Convert the resulting linear/angular velocity into left and right wheel velocities.
Update the differential-drive robot state.
Move to the next waypoint after entering the waypoint tolerance region.

The waypoint navigation experiment successfully reached all four configured waypoints.

Example result:

Waypoint 1 reached: 2.15 s
Waypoint 2 reached: 4.98 s
Waypoint 3 reached: 7.19 s
Waypoint 4 reached: 10.15 s

Final distance: 0.099 m
Navigation status: ALL WAYPOINTS REACHED
A* Obstacle-Aware Navigation

The project uses A* search to generate collision-free paths through an environment containing obstacles.

The planning process is:

Start
  ↓
Grid representation
  ↓
Obstacle-aware A* search
  ↓
Raw path
  ↓
Path simplification
  ↓
Tracking waypoints
  ↓
PID-controlled robot

In the final A* experiment:

Raw path points:       61
Simplified waypoints:   9
Obstacles:              2

The robot successfully followed the planned path and reached the goal.

Measured result:

Final position:          (5.92, 4.94) m
Final target:            (6.00, 5.00) m
Final distance:          0.100 m
Minimum obstacle clearance: 0.131 m
Navigation status:       ALL WAYPOINTS REACHED
Collision status:        COLLISION FREE
Robustness Validation

The navigation system was evaluated under multiple combinations of:

Robot speed
Navigation layouts
Small simulated disturbances/noise

The robustness experiment contained:

12 total scenarios

Results:

Successful scenarios:  12 / 12
Success rate:          100%
Collisions:            0
Planning failures:     0
Average final error:   0.091 m
Maximum final error:    0.100 m
Minimum clearance:     0.175 m
Average travel time:   10.42 s

The minimum clearance remained above the configured robot-radius collision boundary of 0.15 m in the robustness experiments.

Results are stored in:

results/robustness_results.csv

and visualized using:

results/success_rate_vs_speed.png
results/obstacle_clearance_robustness.png
Running the Project
1. Clone the repository
git clone <repository-url>
cd PID-Autonomous-Navigation
2. Create a virtual environment
python3 -m venv venv
3. Activate the environment
macOS / Linux
source venv/bin/activate
Windows
venv\Scripts\activate
4. Install dependencies
pip install -r requirements.txt
Run the Main Demonstration
python main.py

This runs the final PID-controlled waypoint navigation demonstration and displays:

Robot trajectory
Planned path
Tracking waypoints
Goal position
Heading tracking error
Run Individual Experiments
PID step response
python -m experiments.pid_step_response
PID parameter tuning
python -m experiments.pid_tuning
Heading control
python -m experiments.heading_control
Waypoint navigation
python -m experiments.waypoint_navigation
Obstacle-aware navigation
python -m experiments.obstacle_navigation
Robustness testing
python -m experiments.robustness_test
Engineering Metrics

The project evaluates the navigation system using measurable performance indicators:

Metric	Description
Rise Time	Time required for the response to move from 10% to 90% of the target
Overshoot	Percentage by which the response exceeds the target
Settling Time	Time required to remain within the 2% target band
Steady-State Error	Final difference between target and measured response
Final Distance	Distance between robot and target
Obstacle Clearance	Minimum distance maintained from an obstacle
Success Rate	Percentage of navigation scenarios completed successfully
Travel Time	Time required to complete the planned navigation
Limitations

This project is a simulation and does not represent a physical robot.

The differential-drive model does not include detailed real-world effects such as:

Wheel slip
Motor dynamics
Encoder quantization
Sensor latency
Actuator saturation beyond the simulated limits
Real-world localization errors
Complex dynamic obstacles

The robustness results therefore demonstrate behavior within the defined simulation environment and test scenarios rather than guaranteeing equivalent physical-robot performance.

Future Improvements

Potential extensions include:

Model Predictive Control (MPC)
Dynamic obstacle avoidance
Sensor simulation
Noisy localization
Real-time visualization
ROS 2 integration
Gazebo / Webots simulation
Hardware deployment on a differential-drive platform
Author

Daksh Chaudhary

B.Tech Computer Science
SRM Institute of Science and Technology

License

This project is intended for educational and research purposes.