import heapq
import numpy as np


class AStarPlanner:
    """
    Grid-based A* path planner.

    The planner converts the continuous environment into
    a 2D occupancy grid and searches for a collision-free
    path from start to goal.
    """

    def __init__(
        self,
        obstacles,
        x_limits=(-1.0, 7.0),
        y_limits=(-1.0, 6.0),
        resolution=0.10,
        robot_radius=0.15,
        safety_margin=0.10
    ):
        self.obstacles = obstacles

        self.x_min = x_limits[0]
        self.x_max = x_limits[1]

        self.y_min = y_limits[0]
        self.y_max = y_limits[1]

        self.resolution = resolution

        self.robot_radius = robot_radius

        self.safety_margin = safety_margin

        self.x_size = int(
            np.ceil(
                (self.x_max - self.x_min)
                / self.resolution
            )
        ) + 1

        self.y_size = int(
            np.ceil(
                (self.y_max - self.y_min)
                / self.resolution
            )
        ) + 1

        self.occupancy_grid = (
            self._build_occupancy_grid()
        )


    # ========================================================
    # Coordinate Conversion
    # ========================================================

    def world_to_grid(self, x, y):
        gx = int(
            round(
                (x - self.x_min)
                / self.resolution
            )
        )

        gy = int(
            round(
                (y - self.y_min)
                / self.resolution
            )
        )

        return gx, gy


    def grid_to_world(self, gx, gy):
        x = (
            self.x_min
            + gx * self.resolution
        )

        y = (
            self.y_min
            + gy * self.resolution
        )

        return x, y


    # ========================================================
    # Occupancy Grid
    # ========================================================

    def _build_occupancy_grid(self):

        grid = np.zeros(
            (
                self.x_size,
                self.y_size
            ),
            dtype=bool
        )

        obstacle_clearance = (
            self.robot_radius
            + self.safety_margin
        )

        for gx in range(self.x_size):

            for gy in range(self.y_size):

                x, y = self.grid_to_world(
                    gx,
                    gy
                )

                for obstacle in self.obstacles:

                    distance = np.sqrt(
                        (x - obstacle.x) ** 2
                        +
                        (y - obstacle.y) ** 2
                    )

                    if distance <= (
                        obstacle.radius
                        + obstacle_clearance
                    ):

                        grid[gx, gy] = True

                        break

        return grid


    # ========================================================
    # Valid Grid Cell
    # ========================================================

    def is_valid(self, node):

        gx, gy = node

        if gx < 0 or gx >= self.x_size:
            return False

        if gy < 0 or gy >= self.y_size:
            return False

        if self.occupancy_grid[gx, gy]:
            return False

        return True


    # ========================================================
    # Heuristic
    # ========================================================

    @staticmethod
    def heuristic(node, goal):

        return np.sqrt(
            (node[0] - goal[0]) ** 2
            +
            (node[1] - goal[1]) ** 2
        )


    # ========================================================
    # A* Search
    # ========================================================

    def plan(
        self,
        start,
        goal
    ):
        """
        Generate a collision-free path.

        Parameters:
            start: (x, y)
            goal: (x, y)

        Returns:
            List of (x, y) path points.
        """

        start_node = self.world_to_grid(
            start[0],
            start[1]
        )

        goal_node = self.world_to_grid(
            goal[0],
            goal[1]
        )


        if not self.is_valid(start_node):

            raise ValueError(
                "Start position is inside "
                "an obstacle."
            )


        if not self.is_valid(goal_node):

            raise ValueError(
                "Goal position is inside "
                "an obstacle."
            )


        # Priority queue:
        # (f_score, node)

        open_set = []

        heapq.heappush(
            open_set,
            (
                0.0,
                start_node
            )
        )


        came_from = {}

        g_score = {
            start_node: 0.0
        }


        # 8-connected grid

        neighbors = [

            (-1, -1),
            (-1, 0),
            (-1, 1),

            (0, -1),
            (0, 1),

            (1, -1),
            (1, 0),
            (1, 1)

        ]


        while open_set:

            _, current = heapq.heappop(
                open_set
            )


            # ------------------------------------------------
            # Goal reached
            # ------------------------------------------------

            if current == goal_node:

                return self._reconstruct_path(
                    came_from,
                    current
                )


            # ------------------------------------------------
            # Explore neighbors
            # ------------------------------------------------

            for dx, dy in neighbors:

                neighbor = (
                    current[0] + dx,
                    current[1] + dy
                )


                if not self.is_valid(
                    neighbor
                ):
                    continue


                # Diagonal movement costs sqrt(2)

                if dx != 0 and dy != 0:

                    movement_cost = np.sqrt(2)

                else:

                    movement_cost = 1.0


                tentative_g = (
                    g_score[current]
                    + movement_cost
                )


                if (
                    neighbor not in g_score
                    or tentative_g
                    < g_score[neighbor]
                ):

                    came_from[
                        neighbor
                    ] = current

                    g_score[
                        neighbor
                    ] = tentative_g


                    f_score = (
                        tentative_g
                        +
                        self.heuristic(
                            neighbor,
                            goal_node
                        )
                    )


                    heapq.heappush(
                        open_set,
                        (
                            f_score,
                            neighbor
                        )
                    )


        raise RuntimeError(
            "A* could not find a "
            "collision-free path."
        )


    # ========================================================
    # Reconstruct Path
    # ========================================================

    def _reconstruct_path(
        self,
        came_from,
        current
    ):

        path = [current]


        while current in came_from:

            current = came_from[
                current
            ]

            path.append(current)


        path.reverse()


        world_path = []

        for node in path:

            world_path.append(
                self.grid_to_world(
                    node[0],
                    node[1]
                )
            )


        return np.asarray(
            world_path,
            dtype=float
        )


    # ========================================================
    # Path Simplification
    # ========================================================

    @staticmethod
    def simplify_path(
        path,
        tolerance=0.05
    ):
        """
        Reduce unnecessary points from an A*
        grid path.

        Points are retained whenever the direction
        of travel changes significantly.
        """

        if len(path) <= 2:

            return path


        simplified = [
            path[0]
        ]


        previous_direction = None


        for i in range(1, len(path) - 1):

            direction = (
                path[i + 1]
                - path[i]
            )

            magnitude = np.linalg.norm(
                direction
            )

            if magnitude < tolerance:
                continue

            direction = (
                direction / magnitude
            )


            if previous_direction is None:

                previous_direction = direction

                continue


            direction_change = (
                np.linalg.norm(
                    direction
                    - previous_direction
                )
            )


            if direction_change > 0.05:

                simplified.append(
                    path[i]
                )


            previous_direction = direction


        simplified.append(
            path[-1]
        )


        return np.asarray(
            simplified
        )