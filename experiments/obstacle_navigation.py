import numpy as np
import matplotlib.pyplot as plt

from vehicle.differential_drive import (
    DifferentialDriveRobot
)

from navigation.waypoint_follower import (
    WaypointFollower
)

from navigation.obstacle_avoidance import (
    CircularObstacle
)

from navigation.astar_planner import (
    AStarPlanner
)


# ============================================================
# Simulation Settings
# ============================================================

DT = 0.01
SIMULATION_TIME = 25.0

time = np.arange(
    0,
    SIMULATION_TIME,
    DT
)


# ============================================================
# Robot
# ============================================================

robot = DifferentialDriveRobot(
    wheel_base=0.5,
    x=0.0,
    y=0.0,
    theta=0.0
)

ROBOT_RADIUS = 0.15


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

MAX_WHEEL_VELOCITY = 1.5


# ============================================================
# Start and Final Goal
# ============================================================

start = np.array([
    0.0,
    0.0
])

goal = np.array([
    6.0,
    5.0
])


# ============================================================
# Obstacles
# ============================================================

obstacles = [

    CircularObstacle(
        x=3.0,
        y=2.0,
        radius=0.40
    ),

    CircularObstacle(
        x=3.5,
        y=3.75,
        radius=0.40
    )
]


# ============================================================
# A* Planner
# ============================================================

planner = AStarPlanner(
    obstacles=obstacles,
    x_limits=(-1.0, 7.0),
    y_limits=(-1.0, 6.0),
    resolution=0.10,
    robot_radius=ROBOT_RADIUS,
    safety_margin=0.10
)


# ============================================================
# Generate Path
# ============================================================

raw_path = planner.plan(
    start=start,
    goal=goal
)


planned_path = planner.simplify_path(
    raw_path
)


# Remove start point because the robot
# is already at the start.

waypoints = planned_path[1:]


print()
print("A* Path Planning")
print("=================")

print(
    f"Raw path points: "
    f"{len(raw_path)}"
)

print(
    f"Simplified waypoints: "
    f"{len(waypoints)}"
)


# ============================================================
# Waypoint Controller
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

clearance_history = []


previous_waypoint_index = 0

waypoint_reached_times = []


# ============================================================
# Simulation
# ============================================================

for current_time in time:

    # --------------------------------------------------------
    # Navigation completed
    # --------------------------------------------------------

    if waypoint_follower.completed:

        x_history.append(
            robot.x
        )

        y_history.append(
            robot.y
        )

        theta_history.append(
            robot.theta
        )

        distance_history.append(
            0.0
        )

        heading_error_history.append(
            0.0
        )

        linear_velocity_history.append(
            0.0
        )

        angular_velocity_history.append(
            0.0
        )

        continue


    # --------------------------------------------------------
    # Current waypoint
    # --------------------------------------------------------

    current_waypoint = (
        waypoint_follower
        .get_current_waypoint()
    )


    target_x = (
        current_waypoint[0]
    )

    target_y = (
        current_waypoint[1]
    )


    # --------------------------------------------------------
    # Distance to waypoint
    # --------------------------------------------------------

    distance = (
        waypoint_follower
        .distance_to_waypoint(
            robot.x,
            robot.y
        )
    )


    # --------------------------------------------------------
    # Desired heading
    # --------------------------------------------------------

    desired_heading = np.arctan2(
        target_y - robot.y,
        target_x - robot.x
    )


    # --------------------------------------------------------
    # Heading error
    # --------------------------------------------------------

    heading_error = (
        waypoint_follower
        .heading_controller
        .normalize_angle(
            desired_heading
            - robot.theta
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
        waypoint_follower
        .current_waypoint_index
    )


    if (
        current_waypoint_index
        != previous_waypoint_index
    ):

        waypoint_reached_times.append(
            current_time
        )

        print(
            f"Planned waypoint "
            f"{previous_waypoint_index + 1} "
            f"reached at "
            f"{current_time:.2f} s"
        )

        previous_waypoint_index = (
            current_waypoint_index
        )


    # --------------------------------------------------------
    # Wheel velocities
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
    # Wheel limits
    # --------------------------------------------------------

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
    # Calculate obstacle clearance
    # --------------------------------------------------------

    minimum_clearance = np.inf

    for obstacle in obstacles:

        distance_to_obstacle = np.sqrt(
            (
                robot.x
                - obstacle.x
            ) ** 2
            +
            (
                robot.y
                - obstacle.y
            ) ** 2
        )

        clearance = (
            distance_to_obstacle
            - obstacle.radius
            - ROBOT_RADIUS
        )

        minimum_clearance = min(
            minimum_clearance,
            clearance
        )


    # --------------------------------------------------------
    # Collision check
    # --------------------------------------------------------

    if minimum_clearance < 0:

        print()
        print(
            "COLLISION DETECTED"
        )

        break


    # --------------------------------------------------------
    # Store data
    # --------------------------------------------------------

    x_history.append(
        robot.x
    )

    y_history.append(
        robot.y
    )

    theta_history.append(
        robot.theta
    )

    distance_history.append(
        distance
    )

    heading_error_history.append(
        np.rad2deg(
            heading_error
        )
    )

    linear_velocity_history.append(
        linear_velocity
    )

    angular_velocity_history.append(
        angular_velocity
    )

    clearance_history.append(
        minimum_clearance
    )


# ============================================================
# Final Metrics
# ============================================================

final_distance = np.sqrt(
    (
        goal[0]
        - robot.x
    ) ** 2
    +
    (
        goal[1]
        - robot.y
    ) ** 2
)


if clearance_history:

    minimum_clearance = min(
        clearance_history
    )

else:

    minimum_clearance = np.inf


# ============================================================
# Results
# ============================================================

print()
print("Obstacle-Aware Autonomous Navigation")
print("=====================================")

print(
    f"Number of obstacles: "
    f"{len(obstacles)}"
)

print(
    f"Planned path points: "
    f"{len(raw_path)}"
)

print(
    f"Tracking waypoints: "
    f"{len(waypoints)}"
)

print(
    f"Final position: "
    f"({robot.x:.2f}, "
    f"{robot.y:.2f}) m"
)

print(
    f"Final target: "
    f"({goal[0]:.2f}, "
    f"{goal[1]:.2f}) m"
)

print(
    f"Final distance: "
    f"{final_distance:.3f} m"
)

print(
    f"Minimum obstacle clearance: "
    f"{minimum_clearance:.3f} m"
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


if minimum_clearance >= 0:

    print(
        "Collision status: "
        "COLLISION FREE"
    )

else:

    print(
        "Collision status: "
        "COLLISION DETECTED"
    )


# ============================================================
# Plot Environment and Planned Path
# ============================================================

plt.figure(
    figsize=(10, 7)
)


# ------------------------------------------------------------
# Obstacles
# ------------------------------------------------------------

for index, obstacle in enumerate(
    obstacles
):

    obstacle_circle = plt.Circle(
        (
            obstacle.x,
            obstacle.y
        ),
        obstacle.radius,
        fill=False,
        linewidth=2,
        label=(
            "Obstacle"
            if index == 0
            else None
        )
    )

    safety_circle = plt.Circle(
        (
            obstacle.x,
            obstacle.y
        ),
        (
            obstacle.radius
            + ROBOT_RADIUS
            + 0.10
        ),
        fill=False,
        linestyle=":",
        linewidth=1
    )

    plt.gca().add_patch(
        obstacle_circle
    )

    plt.gca().add_patch(
        safety_circle
    )


# ------------------------------------------------------------
# A* path
# ------------------------------------------------------------

plt.plot(
    raw_path[:, 0],
    raw_path[:, 1],
    linestyle="--",
    linewidth=1.2,
    label="A* path"
)


# ------------------------------------------------------------
# Actual trajectory
# ------------------------------------------------------------

plt.plot(
    x_history,
    y_history,
    linewidth=2,
    label="Robot trajectory"
)


# ------------------------------------------------------------
# Start and goal
# ------------------------------------------------------------

plt.scatter(
    start[0],
    start[1],
    s=100,
    label="Start"
)

plt.scatter(
    goal[0],
    goal[1],
    s=120,
    marker="*",
    label="Goal"
)


# ------------------------------------------------------------
# A* waypoints
# ------------------------------------------------------------

if len(waypoints) > 0:

    plt.scatter(
        waypoints[:, 0],
        waypoints[:, 1],
        s=60,
        label="Tracking waypoints"
    )


plt.xlabel(
    "X position (m)"
)

plt.ylabel(
    "Y position (m)"
)

plt.title(
    "A* Obstacle-Aware Autonomous Navigation"
)

plt.axis("equal")

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.show()


# ============================================================
# Plot Obstacle Clearance
# ============================================================

plt.figure(
    figsize=(10, 5)
)

clearance_time = time[
    :len(clearance_history)
]

plt.plot(
    clearance_time,
    clearance_history,
    linewidth=2,
    label="Minimum obstacle clearance"
)

plt.axhline(
    0,
    linestyle="--",
    label="Collision boundary"
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Clearance (m)"
)

plt.title(
    "Obstacle Clearance During Navigation"
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

error_time = time[
    :len(heading_error_history)
]

plt.plot(
    error_time,
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
    "A* Path Tracking Heading Error"
)

plt.grid(True)

plt.tight_layout()

plt.show()