import numpy as np
import matplotlib.pyplot as plt

from vehicle.differential_drive import DifferentialDriveRobot
from navigation.waypoint_follower import WaypointFollower


# ============================================================
# Simulation Settings
# ============================================================

DT = 0.01
SIMULATION_TIME = 15.0

time = np.arange(
    0,
    SIMULATION_TIME,
    DT
)


# ============================================================
# Robot Configuration
# ============================================================

robot = DifferentialDriveRobot(
    wheel_base=0.5,
    x=0.0,
    y=0.0,
    theta=0.0
)


# ============================================================
# PID Parameters
# ============================================================

KP = 8.0
KI = 0.2
KD = 0.2


# ============================================================
# Navigation Parameters
# ============================================================

MAX_LINEAR_VELOCITY = 1.0
MAX_ANGULAR_VELOCITY = 2.0

WAYPOINT_TOLERANCE = 0.10

SLOWDOWN_DISTANCE = 0.75


# ============================================================
# Waypoint Path
# ============================================================

waypoints = np.array([
    [2.0, 1.0],
    [4.0, 3.0],
    [3.0, 5.0],
    [6.0, 5.0]
])


# ============================================================
# Create Waypoint Controller
# ============================================================

waypoint_follower = WaypointFollower(
    waypoints=waypoints,
    kp=KP,
    ki=KI,
    kd=KD,
    waypoint_tolerance=WAYPOINT_TOLERANCE,
    max_linear_velocity=MAX_LINEAR_VELOCITY,
    max_angular_velocity=MAX_ANGULAR_VELOCITY,
    slowdown_distance=SLOWDOWN_DISTANCE
)


# ============================================================
# Data Storage
# ============================================================

x_history = []
y_history = []
theta_history = []

distance_history = []
heading_error_history = []

linear_velocity_history = []
angular_velocity_history = []

waypoint_reached_times = []

previous_waypoint_index = 0


# ============================================================
# Simulation
# ============================================================

for current_time in time:

    # --------------------------------------------------------
    # Navigation complete
    # --------------------------------------------------------

    if waypoint_follower.completed:

        x_history.append(robot.x)
        y_history.append(robot.y)
        theta_history.append(robot.theta)

        distance_history.append(0.0)
        heading_error_history.append(0.0)

        linear_velocity_history.append(0.0)
        angular_velocity_history.append(0.0)

        continue


    # --------------------------------------------------------
    # Current waypoint
    # --------------------------------------------------------

    current_waypoint = (
        waypoint_follower.get_current_waypoint()
    )


    # --------------------------------------------------------
    # Distance to current waypoint
    # --------------------------------------------------------

    distance = (
        waypoint_follower.distance_to_waypoint(
            robot.x,
            robot.y
        )
    )


    # --------------------------------------------------------
    # Desired heading
    # --------------------------------------------------------

    desired_heading = np.arctan2(
        current_waypoint[1] - robot.y,
        current_waypoint[0] - robot.x
    )


    # --------------------------------------------------------
    # Heading error
    # --------------------------------------------------------

    heading_error = (
        waypoint_follower
        .heading_controller
        .normalize_angle(
            desired_heading - robot.theta
        )
    )


    # --------------------------------------------------------
    # Controller
    # --------------------------------------------------------

    (
        angular_velocity,
        linear_velocity,
        waypoint_reached
    ) = waypoint_follower.update(
        x=robot.x,
        y=robot.y,
        theta=robot.theta,
        dt=DT
    )


    # --------------------------------------------------------
    # Detect waypoint transition
    # --------------------------------------------------------

    current_waypoint_index = (
        waypoint_follower.current_waypoint_index
    )


    if current_waypoint_index != previous_waypoint_index:

        waypoint_reached_times.append(
            current_time
        )

        print(
            f"Waypoint "
            f"{previous_waypoint_index + 1} "
            f"reached at "
            f"{current_time:.2f} s"
        )

        previous_waypoint_index = (
            current_waypoint_index
        )


    # --------------------------------------------------------
    # Convert to wheel velocities
    # --------------------------------------------------------

    v_left = (
        linear_velocity
        -
        angular_velocity
        * robot.wheel_base
        / 2
    )

    v_right = (
        linear_velocity
        +
        angular_velocity
        * robot.wheel_base
        / 2
    )


    # --------------------------------------------------------
    # Wheel velocity limiting
    # --------------------------------------------------------

    MAX_WHEEL_VELOCITY = 1.5

    v_left = np.clip(
        v_left,
        -MAX_WHEEL_VELOCITY,
        MAX_WHEEL_VELOCITY
    )

    v_right = np.clip(
        v_right,
        -MAX_WHEEL_VELOCITY,
        MAX_WHEEL_VELOCITY
    )


    # --------------------------------------------------------
    # Update robot
    # --------------------------------------------------------

    robot.update(
        v_left=v_left,
        v_right=v_right,
        dt=DT
    )


    # --------------------------------------------------------
    # Store state
    # --------------------------------------------------------

    x_history.append(robot.x)
    y_history.append(robot.y)
    theta_history.append(robot.theta)

    distance_history.append(distance)

    heading_error_history.append(
        np.rad2deg(heading_error)
    )

    linear_velocity_history.append(
        linear_velocity
    )

    angular_velocity_history.append(
        angular_velocity
    )


# ============================================================
# Final Metrics
# ============================================================

final_waypoint = waypoints[-1]

final_distance = np.sqrt(
    (final_waypoint[0] - robot.x) ** 2
    +
    (final_waypoint[1] - robot.y) ** 2
)


# ============================================================
# Print Results
# ============================================================

print()
print("Waypoint Navigation")
print("===================")

print(
    f"Number of waypoints: "
    f"{len(waypoints)}"
)

print(
    f"Final position: "
    f"({robot.x:.2f}, {robot.y:.2f}) m"
)

print(
    f"Final target: "
    f"({final_waypoint[0]:.2f}, "
    f"{final_waypoint[1]:.2f}) m"
)

print(
    f"Final distance: "
    f"{final_distance:.3f} m"
)

print(
    f"Waypoint tolerance: "
    f"{WAYPOINT_TOLERANCE:.2f} m"
)

print(
    f"Maximum linear velocity: "
    f"{MAX_LINEAR_VELOCITY:.2f} m/s"
)

print(
    f"Maximum angular velocity: "
    f"{MAX_ANGULAR_VELOCITY:.2f} rad/s"
)

print(
    f"Maximum wheel velocity: "
    f"{MAX_WHEEL_VELOCITY:.2f} m/s"
)


if waypoint_follower.completed:

    print(
        "Navigation status: "
        "ALL WAYPOINTS REACHED"
    )

else:

    print(
        "Navigation status: "
        "NAVIGATION INCOMPLETE"
    )


# ============================================================
# Plot Navigation Path
# ============================================================

plt.figure(
    figsize=(10, 7)
)


# Actual trajectory

plt.plot(
    x_history,
    y_history,
    linewidth=2,
    label="Robot trajectory"
)


# Planned path

planned_x = np.concatenate(
    (
        [0.0],
        waypoints[:, 0]
    )
)

planned_y = np.concatenate(
    (
        [0.0],
        waypoints[:, 1]
    )
)

plt.plot(
    planned_x,
    planned_y,
    linestyle="--",
    linewidth=1.5,
    label="Planned path"
)


# Start

plt.scatter(
    0.0,
    0.0,
    s=100,
    label="Start"
)


# Waypoints

plt.scatter(
    waypoints[:, 0],
    waypoints[:, 1],
    s=100,
    marker="o",
    label="Waypoints"
)


# Waypoint labels

for index, waypoint in enumerate(
    waypoints
):

    plt.annotate(
        f"W{index + 1}",
        (
            waypoint[0],
            waypoint[1]
        ),
        xytext=(8, 8),
        textcoords="offset points"
    )


plt.xlabel(
    "X position (m)"
)

plt.ylabel(
    "Y position (m)"
)

plt.title(
    "Autonomous Waypoint Navigation"
)

plt.axis("equal")

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.show()


# ============================================================
# Plot Distance to Waypoint
# ============================================================

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    time,
    distance_history,
    linewidth=2
)

plt.axhline(
    WAYPOINT_TOLERANCE,
    linestyle="--",
    label="Waypoint tolerance"
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Distance (m)"
)

plt.title(
    "Distance to Current Waypoint"
)

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.show()


# ============================================================
# Plot Heading Error
# ============================================================

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    time,
    heading_error_history,
    linewidth=2
)

plt.axhline(
    0,
    linestyle="--"
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Heading error (degrees)"
)

plt.title(
    "Waypoint Heading Tracking Error"
)

plt.grid(True)

plt.tight_layout()

plt.show()


# ============================================================
# Plot Velocity Profile
# ============================================================

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    time,
    linear_velocity_history,
    linewidth=2,
    label="Linear velocity"
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Velocity (m/s)"
)

plt.title(
    "Robot Linear Velocity Profile"
)

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.show()