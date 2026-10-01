class PIDController:

    def __init__(
        self,
        kp,
        ki,
        kd,
        output_min=None,
        output_max=None
    ):
        """
        PID controller with optional output limits.

        Parameters:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain

            output_min:
                Minimum controller output.
                None means no lower limit.

            output_max:
                Maximum controller output.
                None means no upper limit.
        """

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = None


    def reset(self):
        """
        Reset the internal PID state.

        This is useful when switching between
        different navigation waypoints.
        """

        self.integral = 0.0
        self.previous_error = None


    def update(self, error, dt):
        """
        Calculate PID control output.

        Parameters:
            error:
                Current control error.

            dt:
                Simulation time step.

        Returns:
            PID control output.
        """

        # ----------------------------------------------------
        # Proportional term
        # ----------------------------------------------------

        proportional = self.kp * error


        # ----------------------------------------------------
        # Integral term
        # ----------------------------------------------------

        self.integral += error * dt

        integral = self.ki * self.integral


        # ----------------------------------------------------
        # Derivative term
        # ----------------------------------------------------

        if self.previous_error is None:

            derivative_error = 0.0

        else:

            derivative_error = (
                (error - self.previous_error)
                / dt
            )

        derivative = (
            self.kd * derivative_error
        )


        # ----------------------------------------------------
        # PID output
        # ----------------------------------------------------

        output = (
            proportional
            + integral
            + derivative
        )


        # ----------------------------------------------------
        # Output limiting
        # ----------------------------------------------------

        if self.output_min is not None:

            output = max(
                self.output_min,
                output
            )


        if self.output_max is not None:

            output = min(
                self.output_max,
                output
            )


        # ----------------------------------------------------
        # Store current error
        # ----------------------------------------------------

        self.previous_error = error


        return output