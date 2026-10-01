import numpy as np
import matplotlib.pyplot as plt

from vehicle.differential_drive import DifferentialDriveRobot
from controllers.heading_controller import HeadingController


# ============================================================
# Simulation Settings
# ============================================================

DT = 0.01
SIMULATION_TIME = 8.0

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
# Tuned PID Parameters
# ============================================================

KP = 8.0
KI = 0.2
KD = 0.2


heading_controller = HeadingController(
    kp=KP,
    ki=KI,
    kd=KD
)


# ============================================================
# Navigation Target
# ============================================================

target_x = 4.0
target_y = 4.0

# Robot is considered to have reached
# the target when it is within this distance.
TARGET_TOLERANCE = 0.10


# ============================================================
# Data Storage
# ============================================================

x_history = []
y_history = []
theta_history = []

heading_error_history = []
angular_velocity_history = []
distance_history = []


# ============================================================
# Simulation
# ============================================================

target_reached = False
target_reached_time = None


for current_time in time:

    # --------------------------------------------------------
    # Calculate distance to target
    # --------------------------------------------------------

    distance_to_target = np.sqrt(
        (target_x - robot.x) ** 2
        +
        (target_y - robot.y) ** 2
    )

    distance_history.append(
        distance_to_target
    )


    # --------------------------------------------------------
    # Check whether target has been reached
    # --------------------------------------------------------

    if distance_to_target <= TARGET_TOLERANCE:

        target_reached = True

        if target_reached_time is None:
            target_reached_time = current_time

        # Stop the robot
        linear_velocity = 0.0
        angular_velocity = 0.0

        heading_error = 0.0

    else:

        # ----------------------------------------------------
        # Calculate desired heading
        # ----------------------------------------------------

        desired_heading = np.arctan2(
            target_y - robot.y,
            target_x - robot.x
        )

        # ----------------------------------------------------
        # PID heading control
        # ----------------------------------------------------

        angular_velocity = heading_controller.update(
            desired_heading,
            robot.theta,
            DT
        )

        # ----------------------------------------------------
        # Constant forward velocity
        # ----------------------------------------------------

        linear_velocity = 1.0

        # ----------------------------------------------------
        # Calculate heading error
        # ----------------------------------------------------

        heading_error = heading_controller.normalize_angle(
            desired_heading - robot.theta
        )


    # ========================================================
    # Convert Linear/Angular Velocity
    # to Wheel Velocities
    # ========================================================

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


    # ========================================================
    # Update Robot
    # ========================================================

    robot.update(
        v_left=v_left,
        v_right=v_right,
        dt=DT
    )


    # ========================================================
    # Store State
    # ========================================================

    x_history.append(robot.x)
    y_history.append(robot.y)

    theta_history.append(
        robot.theta
    )

    heading_error_history.append(
        np.rad2deg(heading_error)
    )

    angular_velocity_history.append(
        angular_velocity
    )


# ============================================================
# Final State
# ============================================================

final_distance = np.sqrt(
    (target_x - robot.x) ** 2
    +
    (target_y - robot.y) ** 2
)


# ============================================================
# Print Results
# ============================================================

print()
print("Closed-Loop Heading Control")
print("===========================")

print(
    f"Target position: "
    f"({target_x:.2f}, {target_y:.2f}) m"
)

print(
    f"Final position:  "
    f"({robot.x:.2f}, {robot.y:.2f}) m"
)

print(
    f"Final heading:   "
    f"{np.rad2deg(robot.theta):.2f} degrees"
)

print(
    f"Final distance:  "
    f"{final_distance:.3f} m"
)

print(
    f"Target tolerance: "
    f"{TARGET_TOLERANCE:.2f} m"
)


if target_reached_time is not None:

    print(
        f"Target reached at: "
        f"{target_reached_time:.2f} s"
    )

    print(
        "Navigation status: TARGET REACHED"
    )

else:

    print(
        "Navigation status: "
        "TARGET NOT REACHED"
    )


# ============================================================
# Plot Robot Trajectory
# ============================================================

plt.figure(
    figsize=(9, 7)
)

plt.plot(
    x_history,
    y_history,
    linewidth=2,
    label="Robot trajectory"
)

plt.scatter(
    x_history[0],
    y_history[0],
    s=100,
    label="Start"
)

plt.scatter(
    target_x,
    target_y,
    s=100,
    marker="*",
    label="Target"
)

# Target tolerance circle
circle = plt.Circle(
    (target_x, target_y),
    TARGET_TOLERANCE,
    fill=False,
    linestyle="--",
    label="Target tolerance"
)

plt.gca().add_patch(circle)

plt.xlabel(
    "X position (m)"
)

plt.ylabel(
    "Y position (m)"
)

plt.title(
    "Closed-Loop PID Heading Control"
)

plt.axis("equal")

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.show()


# ============================================================
# Plot Distance to Target
# ============================================================

plt.figure(
    figsize=(9, 5)
)

plt.plot(
    time,
    distance_history,
    linewidth=2,
    label="Distance to target"
)

plt.axhline(
    TARGET_TOLERANCE,
    linestyle="--",
    label="Target tolerance"
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Distance (m)"
)

plt.title(
    "Distance to Navigation Target"
)

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.show()


# ============================================================
# Plot Heading Error
# ============================================================

plt.figure(
    figsize=(9, 5)
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
    "Heading Tracking Error"
)

plt.grid(True)

plt.tight_layout()

plt.show()