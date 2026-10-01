import numpy as np


class DifferentialDriveRobot:
    def __init__(self, wheel_base=0.5, x=0.0, y=0.0, theta=0.0):
        """
        Differential-drive robot model.

        Parameters:
            wheel_base: Distance between left and right wheels (meters)
            x: Initial x position (meters)
            y: Initial y position (meters)
            theta: Initial orientation (radians)
        """

        self.wheel_base = wheel_base

        # Robot state
        self.x = x
        self.y = y
        self.theta = theta

    def update(self, v_left, v_right, dt):
        """
        Update robot position using left and right wheel velocities.

        Parameters:
            v_left: Left wheel velocity (m/s)
            v_right: Right wheel velocity (m/s)
            dt: Simulation time step (seconds)
        """

        # Differential-drive equations
        v = (v_right + v_left) / 2
        omega = (v_right - v_left) / self.wheel_base

        # Update position
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt

        # Keep theta between -pi and pi
        self.theta = np.arctan2(
            np.sin(self.theta),
            np.cos(self.theta)
        )

    def get_state(self):
        """Return the current robot state."""
        return self.x, self.y, self.theta