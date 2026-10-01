import numpy as np

from controllers.heading_controller import HeadingController


class WaypointFollower:
    """
    Waypoint-following controller for a differential-drive robot.

    The controller:
        1. Selects the current waypoint.
        2. Calculates desired heading.
        3. Uses PID heading control.
        4. Limits angular velocity.
        5. Reduces forward speed near waypoints.
        6. Supports an optional externally supplied
           desired heading for obstacle avoidance.
    """

    def __init__(
        self,
        waypoints,
        kp,
        ki,
        kd,
        waypoint_tolerance=0.10,
        max_linear_velocity=1.0,
        max_angular_velocity=2.0,
        slowdown_distance=0.75
    ):

        self.waypoints = np.asarray(
            waypoints,
            dtype=float
        )

        self.waypoint_tolerance = (
            waypoint_tolerance
        )

        self.max_linear_velocity = (
            max_linear_velocity
        )

        self.max_angular_velocity = (
            max_angular_velocity
        )

        self.slowdown_distance = (
            slowdown_distance
        )

        self.current_waypoint_index = 0

        self.heading_controller = HeadingController(
            kp=kp,
            ki=ki,
            kd=kd,
            max_angular_velocity=max_angular_velocity
        )

        self.completed = False


    def get_current_waypoint(self):
        """
        Return the current target waypoint.
        """

        if self.completed:
            return None

        return self.waypoints[
            self.current_waypoint_index
        ]


    def distance_to_waypoint(
        self,
        x,
        y
    ):
        """
        Calculate distance to current waypoint.
        """

        waypoint = self.get_current_waypoint()

        if waypoint is None:
            return 0.0

        dx = waypoint[0] - x
        dy = waypoint[1] - y

        return np.sqrt(
            dx ** 2 + dy ** 2
        )


    def get_desired_heading(
        self,
        x,
        y
    ):
        """
        Calculate the direct heading toward
        the current waypoint.
        """

        waypoint = self.get_current_waypoint()

        if waypoint is None:
            return 0.0

        return np.arctan2(
            waypoint[1] - y,
            waypoint[0] - x
        )


    def update(
        self,
        x,
        y,
        theta,
        dt,
        desired_heading_override=None
    ):
        """
        Calculate navigation commands.

        Parameters:
            x, y:
                Current robot position.

            theta:
                Current robot heading.

            dt:
                Simulation time step.

            desired_heading_override:
                Optional heading supplied by an
                obstacle-avoidance system.

        Returns:
            angular_velocity
            linear_velocity
            waypoint_reached
        """

        if self.completed:

            return (
                0.0,
                0.0,
                False
            )


        waypoint_reached = False


        # ----------------------------------------------------
        # Distance to waypoint
        # ----------------------------------------------------

        distance = self.distance_to_waypoint(
            x,
            y
        )


        # ----------------------------------------------------
        # Check waypoint
        # ----------------------------------------------------

        if distance <= self.waypoint_tolerance:

            waypoint_reached = True

            self.current_waypoint_index += 1


            if (
                self.current_waypoint_index
                >= len(self.waypoints)
            ):

                self.completed = True

                return (
                    0.0,
                    0.0,
                    True
                )


            # Reset PID when switching waypoint

            self.heading_controller.reset()

            distance = self.distance_to_waypoint(
                x,
                y
            )


        # ----------------------------------------------------
        # Desired heading
        # ----------------------------------------------------

        if desired_heading_override is None:

            desired_heading = (
                self.get_desired_heading(
                    x,
                    y
                )
            )

        else:

            desired_heading = (
                desired_heading_override
            )


        # ----------------------------------------------------
        # Heading control
        # ----------------------------------------------------

        angular_velocity = (
            self.heading_controller.update(
                desired_heading,
                theta,
                dt
            )
        )


        # ----------------------------------------------------
        # Adaptive forward velocity
        # ----------------------------------------------------

        if distance < self.slowdown_distance:

            speed_scale = (
                distance
                / self.slowdown_distance
            )

            linear_velocity = (
                self.max_linear_velocity
                * speed_scale
            )

        else:

            linear_velocity = (
                self.max_linear_velocity
            )


        linear_velocity = np.clip(
            linear_velocity,
            0.0,
            self.max_linear_velocity
        )


        return (
            angular_velocity,
            linear_velocity,
            waypoint_reached
        )


    def get_progress(self):
        """
        Return current waypoint index
        and total waypoint count.
        """

        return (
            self.current_waypoint_index,
            len(self.waypoints)
        )