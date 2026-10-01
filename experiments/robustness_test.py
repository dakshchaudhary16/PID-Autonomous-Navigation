import heapq
import math

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from vehicle.differential_drive import DifferentialDriveRobot
from controllers.heading_controller import HeadingController


# ============================================================
# Global Simulation Settings
# ============================================================

DT = 0.02

START = (0.0, 0.0)
GOAL = (6.0, 5.0)

WHEEL_BASE = 0.5
ROBOT_RADIUS = 0.15

WAYPOINT_TOLERANCE = 0.10

KP = 8.0
KI = 0.2
KD = 0.2

MAX_ANGULAR_VELOCITY = 2.0

GRID_RESOLUTION = 0.10

# Safety margin added around obstacles during A* planning.
PLANNING_MARGIN = 0.20


# ============================================================
# Obstacle Layouts
# ============================================================

OBSTACLE_LAYOUTS = {

    "Layout_A": [
        (3.0, 2.0, 0.40),
        (3.5, 4.0, 0.40)
    ],

    "Layout_B": [
        (3.0, 2.5, 0.45),
        (4.0, 4.0, 0.45)
    ]

}


# ============================================================
# A* Helper Functions
# ============================================================

def heuristic(a, b):
    """Euclidean distance heuristic."""

    return math.hypot(
        b[0] - a[0],
        b[1] - a[1]
    )


def point_to_grid(point):
    """Convert world coordinates to grid coordinates."""

    return (
        int(round(point[0] / GRID_RESOLUTION)),
        int(round(point[1] / GRID_RESOLUTION))
    )


def grid_to_point(grid_point):
    """Convert grid coordinates to world coordinates."""

    return (
        grid_point[0] * GRID_RESOLUTION,
        grid_point[1] * GRID_RESOLUTION
    )


def is_collision(point, obstacles):
    """
    Check whether a point lies inside an obstacle
    including robot/planning safety margin.
    """

    x, y = point

    for ox, oy, radius in obstacles:

        effective_radius = (
            radius
            + ROBOT_RADIUS
            + PLANNING_MARGIN
        )

        distance = math.hypot(
            x - ox,
            y - oy
        )

        if distance <= effective_radius:
            return True

    return False


def build_grid_bounds():
    """Return finite planning area."""

    return (
        -1.0,
        7.0,
        -1.0,
        6.0
    )


def a_star(start, goal, obstacles):
    """
    A* grid-based path planner.

    Returns a list of world-coordinate path points.
    """

    min_x, max_x, min_y, max_y = build_grid_bounds()

    start_grid = point_to_grid(start)
    goal_grid = point_to_grid(goal)

    min_grid_x = int(min_x / GRID_RESOLUTION)
    max_grid_x = int(max_x / GRID_RESOLUTION)

    min_grid_y = int(min_y / GRID_RESOLUTION)
    max_grid_y = int(max_y / GRID_RESOLUTION)

    open_set = []

    heapq.heappush(
        open_set,
        (
            heuristic(start_grid, goal_grid),
            0.0,
            start_grid
        )
    )

    came_from = {}

    g_score = {
        start_grid: 0.0
    }

    directions = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1)
    ]

    while open_set:

        _, current_cost, current = heapq.heappop(
            open_set
        )

        if current == goal_grid:

            path = [current]

            while current in came_from:

                current = came_from[current]

                path.append(current)

            path.reverse()

            return [
                grid_to_point(point)
                for point in path
            ]

        for dx, dy in directions:

            neighbor = (
                current[0] + dx,
                current[1] + dy
            )

            if not (
                min_grid_x
                <= neighbor[0]
                <= max_grid_x
            ):
                continue

            if not (
                min_grid_y
                <= neighbor[1]
                <= max_grid_y
            ):
                continue

            neighbor_world = grid_to_point(
                neighbor
            )

            if is_collision(
                neighbor_world,
                obstacles
            ):
                continue

            movement_cost = math.hypot(
                dx,
                dy
            )

            tentative_g = (
                current_cost
                + movement_cost
            )

            if (
                neighbor not in g_score
                or tentative_g < g_score[neighbor]
            ):

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g

                f_score = (
                    tentative_g
                    + heuristic(
                        neighbor,
                        goal_grid
                    )
                )

                heapq.heappush(
                    open_set,
                    (
                        f_score,
                        tentative_g,
                        neighbor
                    )
                )

    return []


# ============================================================
# Path Simplification
# ============================================================

def simplify_path(path, obstacles):
    """
    Reduce dense A* grid path into tracking waypoints.

    A point is retained when the direct line from the
    previous retained point to that point can no longer
    safely bypass all obstacles.
    """

    if len(path) <= 2:
        return path

    simplified = [path[0]]

    anchor_index = 0

    for i in range(2, len(path)):

        anchor = path[anchor_index]
        candidate = path[i]

        steps = max(
            2,
            int(
                math.hypot(
                    candidate[0] - anchor[0],
                    candidate[1] - anchor[1]
                )
                / GRID_RESOLUTION
            )
        )

        clear = True

        for j in range(steps + 1):

            ratio = j / steps

            x = (
                anchor[0]
                + ratio
                * (candidate[0] - anchor[0])
            )

            y = (
                anchor[1]
                + ratio
                * (candidate[1] - anchor[1])
            )

            if is_collision(
                (x, y),
                obstacles
            ):

                clear = False
                break

        if not clear:

            simplified.append(
                path[i - 1]
            )

            anchor_index = i - 1

    simplified.append(path[-1])

    return simplified


# ============================================================
# Distance From Point To Obstacles
# ============================================================

def minimum_obstacle_clearance(
    x,
    y,
    obstacles
):
    """
    Calculate distance between robot center and
    nearest obstacle boundary.
    """

    minimum = float("inf")

    for ox, oy, radius in obstacles:

        center_distance = math.hypot(
            x - ox,
            y - oy
        )

        clearance = (
            center_distance
            - radius
            - ROBOT_RADIUS
        )

        minimum = min(
            minimum,
            clearance
        )

    return minimum


# ============================================================
# Run One Navigation Experiment
# ============================================================

def run_experiment(
    speed,
    noise_std,
    layout_name,
    obstacles
):
    """
    Run one complete A* + PID navigation experiment.
    """

    path = a_star(
        START,
        GOAL,
        obstacles
    )

    if not path:

        return {
            "Layout": layout_name,
            "Speed (m/s)": speed,
            "Noise Std (m)": noise_std,
            "Status": "PLANNING FAILED",
            "Final Distance (m)": np.nan,
            "Minimum Clearance (m)": np.nan,
            "Travel Time (s)": np.nan,
            "Path Length (m)": np.nan,
            "Waypoints": 0
        }

    waypoints = simplify_path(
        path,
        obstacles
    )

    robot = DifferentialDriveRobot(
        wheel_base=WHEEL_BASE,
        x=START[0],
        y=START[1],
        theta=0.0
    )

    heading_controller = HeadingController(
        kp=KP,
        ki=KI,
        kd=KD
    )

    current_waypoint = 0

    x_history = []
    y_history = []

    minimum_clearance = float("inf")

    simulation_time = 0.0

    max_simulation_time = 30.0

    while (
        simulation_time
        < max_simulation_time
    ):

        if current_waypoint >= len(waypoints):
            break

        target_x, target_y = waypoints[
            current_waypoint
        ]

        # ----------------------------------------------------
        # Simulated noisy position measurement
        # ----------------------------------------------------

        measured_x = (
            robot.x
            + np.random.normal(
                0.0,
                noise_std
            )
        )

        measured_y = (
            robot.y
            + np.random.normal(
                0.0,
                noise_std
            )
        )

        # ----------------------------------------------------
        # Distance to current waypoint
        # ----------------------------------------------------

        distance = math.hypot(
            target_x - measured_x,
            target_y - measured_y
        )

        # ----------------------------------------------------
        # Waypoint reached
        # ----------------------------------------------------

        if distance <= WAYPOINT_TOLERANCE:

            current_waypoint += 1

            continue

        # ----------------------------------------------------
        # Desired heading
        # ----------------------------------------------------

        desired_heading = math.atan2(
            target_y - measured_y,
            target_x - measured_x
        )

        # ----------------------------------------------------
        # PID heading control
        # ----------------------------------------------------

        angular_velocity = (
            heading_controller.update(
                desired_heading,
                robot.theta,
                DT
            )
        )

        angular_velocity = np.clip(
            angular_velocity,
            -MAX_ANGULAR_VELOCITY,
            MAX_ANGULAR_VELOCITY
        )

        # ----------------------------------------------------
        # Reduce speed when heading error is large
        # ----------------------------------------------------

        heading_error = heading_controller.normalize_angle(
            desired_heading - robot.theta
        )

        heading_factor = max(
            0.25,
            math.cos(heading_error)
        )

        linear_velocity = (
            speed
            * heading_factor
        )

        # ----------------------------------------------------
        # Differential-drive conversion
        # ----------------------------------------------------

        v_left = (
            linear_velocity
            - (
                angular_velocity
                * WHEEL_BASE
                / 2
            )
        )

        v_right = (
            linear_velocity
            + (
                angular_velocity
                * WHEEL_BASE
                / 2
            )
        )

        # ----------------------------------------------------
        # Update robot
        # ----------------------------------------------------

        robot.update(
            v_left=v_left,
            v_right=v_right,
            dt=DT
        )

        # ----------------------------------------------------
        # Safety monitoring
        # ----------------------------------------------------

        clearance = minimum_obstacle_clearance(
            robot.x,
            robot.y,
            obstacles
        )

        minimum_clearance = min(
            minimum_clearance,
            clearance
        )

        # Collision check
        if clearance < 0:

            return {
                "Layout": layout_name,
                "Speed (m/s)": speed,
                "Noise Std (m)": noise_std,
                "Status": "COLLISION",
                "Final Distance (m)": math.hypot(
                    GOAL[0] - robot.x,
                    GOAL[1] - robot.y
                ),
                "Minimum Clearance (m)": minimum_clearance,
                "Travel Time (s)": simulation_time,
                "Path Length (m)": np.nan,
                "Waypoints": len(waypoints)
            }

        x_history.append(robot.x)
        y_history.append(robot.y)

        simulation_time += DT

    # --------------------------------------------------------
    # Final metrics
    # --------------------------------------------------------

    final_distance = math.hypot(
        GOAL[0] - robot.x,
        GOAL[1] - robot.y
    )

    if len(x_history) > 1:

        path_length = np.sum(
            np.sqrt(
                np.diff(x_history) ** 2
                +
                np.diff(y_history) ** 2
            )
        )

    else:

        path_length = 0.0

    if current_waypoint >= len(waypoints):

        status = "SUCCESS"

    else:

        status = "INCOMPLETE"

    return {
        "Layout": layout_name,
        "Speed (m/s)": speed,
        "Noise Std (m)": noise_std,
        "Status": status,
        "Final Distance (m)": final_distance,
        "Minimum Clearance (m)": minimum_clearance,
        "Travel Time (s)": simulation_time,
        "Path Length (m)": path_length,
        "Waypoints": len(waypoints)
    }


# ============================================================
# Main Robustness Experiment
# ============================================================

def main():

    np.random.seed(42)

    speeds = [
        0.6,
        0.8,
        1.0
    ]

    noise_levels = [
        0.00,
        0.01
    ]

    results = []

    total_tests = (
        len(OBSTACLE_LAYOUTS)
        * len(speeds)
        * len(noise_levels)
    )

    test_number = 0

    print()
    print("Autonomous Navigation Robustness Test")
    print("======================================")
    print()
    print(
        f"Total experiments: {total_tests}"
    )
    print()

    for layout_name, obstacles in OBSTACLE_LAYOUTS.items():

        for speed in speeds:

            for noise_std in noise_levels:

                test_number += 1

                result = run_experiment(
                    speed=speed,
                    noise_std=noise_std,
                    layout_name=layout_name,
                    obstacles=obstacles
                )

                results.append(result)

                print(
                    f"[{test_number:02d}/{total_tests}] "
                    f"{layout_name} | "
                    f"Speed={speed:.1f} m/s | "
                    f"Noise={noise_std:.2f} m | "
                    f"{result['Status']}"
                )


    # ========================================================
    # Create Results Table
    # ========================================================

    results_df = pd.DataFrame(
        results
    )

    results_df.to_csv(
        "results/robustness_results.csv",
        index=False
    )


    # ========================================================
    # Summary
    # ========================================================

    successful = (
        results_df["Status"]
        == "SUCCESS"
    ).sum()

    collisions = (
        results_df["Status"]
        == "COLLISION"
    ).sum()

    incomplete = (
        results_df["Status"]
        == "INCOMPLETE"
    ).sum()

    planning_failed = (
        results_df["Status"]
        == "PLANNING FAILED"
    ).sum()

    success_rate = (
        successful
        / len(results_df)
        * 100
    )


    print()
    print("Robustness Summary")
    print("==================")

    print(
        f"Total tests:       {len(results_df)}"
    )

    print(
        f"Successful:        {successful}"
    )

    print(
        f"Incomplete:        {incomplete}"
    )

    print(
        f"Collisions:        {collisions}"
    )

    print(
        f"Planning failures: {planning_failed}"
    )

    print(
        f"Success rate:      {success_rate:.1f}%"
    )


    successful_results = results_df[
        results_df["Status"] == "SUCCESS"
    ]

    if len(successful_results) > 0:

        print()
        print("Successful Test Statistics")
        print("==========================")

        print(
            "Average final distance: "
            f"{successful_results['Final Distance (m)'].mean():.3f} m"
        )

        print(
            "Maximum final distance: "
            f"{successful_results['Final Distance (m)'].max():.3f} m"
        )

        print(
            "Minimum obstacle clearance: "
            f"{successful_results['Minimum Clearance (m)'].min():.3f} m"
        )

        print(
            "Average travel time: "
            f"{successful_results['Travel Time (s)'].mean():.2f} s"
        )


    print()
    print(
        "Results saved to:"
    )

    print(
        "results/robustness_results.csv"
    )


    # ========================================================
    # Plot 1: Success Rate By Speed
    # ========================================================

    speed_success = []

    for speed in speeds:

        subset = results_df[
            results_df["Speed (m/s)"] == speed
        ]

        rate = (
            (
                subset["Status"]
                == "SUCCESS"
            ).mean()
            * 100
        )

        speed_success.append(rate)


    plt.figure(
        figsize=(9, 5)
    )

    plt.plot(
        speeds,
        speed_success,
        marker="o",
        linewidth=2
    )

    plt.xlabel(
        "Robot speed (m/s)"
    )

    plt.ylabel(
        "Success rate (%)"
    )

    plt.title(
        "Navigation Success Rate vs Robot Speed"
    )

    plt.ylim(
        0,
        105
    )

    plt.grid(True)

    plt.tight_layout()

    plt.savefig(
        "results/success_rate_vs_speed.png",
        dpi=200
    )

    plt.show()


    # ========================================================
    # Plot 2: Minimum Obstacle Clearance
    # ========================================================

    labels = []

    clearance_values = []

    for _, row in results_df.iterrows():

        labels.append(
            (
                f"{row['Layout']}\n"
                f"{row['Speed (m/s)']} m/s\n"
                f"N={row['Noise Std (m)']}"
            )
        )

        clearance_values.append(
            row["Minimum Clearance (m)"]
        )


    plt.figure(
        figsize=(12, 6)
    )

    plt.plot(
        range(len(clearance_values)),
        clearance_values,
        marker="o",
        linewidth=2
    )

    plt.axhline(
        ROBOT_RADIUS,
        linestyle="--",
        label="Robot radius"
    )

    plt.xlabel(
        "Experiment"
    )

    plt.ylabel(
        "Minimum obstacle clearance (m)"
    )

    plt.title(
        "Obstacle Clearance Across Robustness Tests"
    )

    plt.xticks(
        range(len(labels)),
        labels,
        rotation=45,
        ha="right"
    )

    plt.grid(True)

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        "results/obstacle_clearance_robustness.png",
        dpi=200
    )

    plt.show()


# ============================================================
# Entry Point
# ============================================================

if __name__ == "__main__":
    main()