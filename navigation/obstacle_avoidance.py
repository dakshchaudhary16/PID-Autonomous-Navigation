import numpy as np


class CircularObstacle:
    """
    Circular obstacle represented by a center position
    and radius.
    """

    def __init__(self, x, y, radius):
        self.x = float(x)
        self.y = float(y)
        self.radius = float(radius)


class ObstacleAvoidance:
    """
    Reactive obstacle-avoidance system.

    The system combines:
        - attraction toward the navigation target
        - repulsion away from nearby obstacles

    The resulting vector determines an adjusted
    desired heading.
    """

    def __init__(
        self,
        obstacles,
        influence_distance=1.0,
        robot_radius=0.15,
        safety_margin=0.10,
        avoidance_gain=2.0
    ):
        """
        Parameters:
            obstacles:
                List of CircularObstacle objects.

            influence_distance:
                Distance around obstacles within which
                avoidance behavior becomes active.

            robot_radius:
                Approximate radius of the robot.

            safety_margin:
                Additional clearance around obstacles.

            avoidance_gain:
                Strength of obstacle repulsion.
        """

        self.obstacles = obstacles

        self.influence_distance = (
            influence_distance
        )

        self.robot_radius = robot_radius

        self.safety_margin = safety_margin

        self.avoidance_gain = avoidance_gain


    def get_distance_to_obstacle(
        self,
        x,
        y,
        obstacle
    ):
        """
        Calculate distance from robot center
        to obstacle center.
        """

        return np.sqrt(
            (x - obstacle.x) ** 2
            +
            (y - obstacle.y) ** 2
        )


    def check_collision(
        self,
        x,
        y
    ):
        """
        Check whether the robot is colliding
        with any obstacle.
        """

        for obstacle in self.obstacles:

            distance = (
                self.get_distance_to_obstacle(
                    x,
                    y,
                    obstacle
                )
            )

            collision_distance = (
                obstacle.radius
                + self.robot_radius
            )

            if distance <= collision_distance:
                return True

        return False


    def get_minimum_clearance(
        self,
        x,
        y
    ):
        """
        Return minimum clearance between the robot
        boundary and any obstacle boundary.
        """

        clearances = []

        for obstacle in self.obstacles:

            distance = (
                self.get_distance_to_obstacle(
                    x,
                    y,
                    obstacle
                )
            )

            clearance = (
                distance
                - obstacle.radius
                - self.robot_radius
            )

            clearances.append(clearance)

        if not clearances:
            return np.inf

        return min(clearances)


    def get_avoidance_heading(
        self,
        x,
        y,
        target_x,
        target_y
    ):
        """
        Calculate an obstacle-aware desired heading.

        Returns:
            adjusted heading in radians
            whether obstacle avoidance is active
        """

        # ----------------------------------------------------
        # Direction toward target
        # ----------------------------------------------------

        target_vector = np.array([
            target_x - x,
            target_y - y
        ])

        target_distance = np.linalg.norm(
            target_vector
        )

        if target_distance < 1e-9:

            return 0.0, False

        target_direction = (
            target_vector
            / target_distance
        )


        # ----------------------------------------------------
        # Obstacle repulsion
        # ----------------------------------------------------

        repulsion = np.zeros(2)

        avoidance_active = False


        for obstacle in self.obstacles:

            obstacle_vector = np.array([
                x - obstacle.x,
                y - obstacle.y
            ])

            distance = np.linalg.norm(
                obstacle_vector
            )


            if distance < 1e-9:

                # Robot is essentially at obstacle center.
                # Push in a deterministic direction.
                obstacle_direction = np.array([
                    1.0,
                    0.0
                ])

            else:

                obstacle_direction = (
                    obstacle_vector
                    / distance
                )


            # ------------------------------------------------
            # Distance at which avoidance begins
            # ------------------------------------------------

            influence_radius = (
                obstacle.radius
                + self.robot_radius
                + self.safety_margin
                + self.influence_distance
            )


            if distance < influence_radius:

                avoidance_active = True


                # ------------------------------------------------
                # Repulsion strength
                # ------------------------------------------------

                distance_from_boundary = (
                    distance
                    - obstacle.radius
                    - self.robot_radius
                    - self.safety_margin
                )


                if distance_from_boundary <= 0:

                    strength = 1.0

                else:

                    strength = (
                        (
                            influence_radius
                            - distance
                        )
                        / self.influence_distance
                    )


                strength = np.clip(
                    strength,
                    0.0,
                    1.0
                )


                repulsion += (
                    obstacle_direction
                    * strength
                )


        # ----------------------------------------------------
        # Combine target attraction and repulsion
        # ----------------------------------------------------

        combined_vector = (
            target_direction
            +
            self.avoidance_gain
            * repulsion
        )


        magnitude = np.linalg.norm(
            combined_vector
        )


        if magnitude < 1e-9:

            combined_vector = (
                target_direction
            )

            magnitude = np.linalg.norm(
                combined_vector
            )


        combined_vector /= magnitude


        # ----------------------------------------------------
        # Calculate adjusted heading
        # ----------------------------------------------------

        adjusted_heading = np.arctan2(
            combined_vector[1],
            combined_vector[0]
        )


        return (
            adjusted_heading,
            avoidance_active
        )