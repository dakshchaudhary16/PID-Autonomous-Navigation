import numpy as np

from controllers.pid_controller import PIDController


class HeadingController:
    """
    Closed-loop heading controller for a differential-drive robot.

    The controller calculates heading error and uses PID control
    to determine the required angular velocity.
    """

    def __init__(
        self,
        kp,
        ki,
        kd,
        max_angular_velocity=None
    ):
        """
        Parameters:
            kp, ki, kd:
                PID gains.

            max_angular_velocity:
                Maximum allowed angular velocity in rad/s.
                None means unlimited.
        """

        self.max_angular_velocity = (
            max_angular_velocity
        )

        self.pid = PIDController(
            kp=kp,
            ki=ki,
            kd=kd
        )


    @staticmethod
    def normalize_angle(angle):
        """
        Normalize an angle to [-pi, pi].
        """

        return np.arctan2(
            np.sin(angle),
            np.cos(angle)
        )


    def reset(self):
        """
        Reset the internal PID state.
        """

        self.pid.reset()


    def update(
        self,
        desired_heading,
        current_heading,
        dt
    ):
        """
        Calculate angular velocity command.
        """

        error = self.normalize_angle(
            desired_heading - current_heading
        )


        angular_velocity = self.pid.update(
            error,
            dt
        )


        # ----------------------------------------------------
        # Angular velocity limit
        # ----------------------------------------------------

        if self.max_angular_velocity is not None:

            angular_velocity = np.clip(
                angular_velocity,
                -self.max_angular_velocity,
                self.max_angular_velocity
            )


        return angular_velocity