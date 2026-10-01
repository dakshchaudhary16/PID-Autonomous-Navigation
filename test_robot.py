import matplotlib.pyplot as plt

from vehicle.differential_drive import DifferentialDriveRobot


robot = DifferentialDriveRobot()

dt = 0.1

# Store trajectory
trajectory = []


# -----------------------------
# Phase 1: Move straight
# -----------------------------
for _ in range(20):

    robot.update(
        v_left=1.0,
        v_right=1.0,
        dt=dt
    )

    trajectory.append(robot.get_state())


# -----------------------------
# Phase 2: Turn
# -----------------------------
for _ in range(10):

    robot.update(
        v_left=0.5,
        v_right=1.0,
        dt=dt
    )

    trajectory.append(robot.get_state())


# -----------------------------
# Phase 3: Move forward
# -----------------------------
for _ in range(20):

    robot.update(
        v_left=1.0,
        v_right=1.0,
        dt=dt
    )

    trajectory.append(robot.get_state())


# Convert trajectory into separate arrays
x = [state[0] for state in trajectory]
y = [state[1] for state in trajectory]


# -----------------------------
# Plot trajectory
# -----------------------------

plt.figure(figsize=(8, 6))

plt.plot(x, y, linewidth=2)

plt.scatter(x[0], y[0], s=100, label="Start")
plt.scatter(x[-1], y[-1], s=100, label="End")

plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")

plt.title("Differential-Drive Robot Trajectory")

plt.axis("equal")
plt.grid(True)
plt.legend()

plt.show()