import numpy as np
import pandas as pd

from controllers.pid_controller import PIDController
from analysis.metrics import calculate_step_metrics


# ============================================================
# Simulation Settings
# ============================================================

DT = 0.01
SIMULATION_TIME = 10.0

TARGET_ANGLE_DEG = 90.0

time = np.arange(
    0,
    SIMULATION_TIME,
    DT
)

target_angle = np.deg2rad(
    TARGET_ANGLE_DEG
)


# ============================================================
# PID Parameter Search Space
# ============================================================

KP_VALUES = [
    2.0,
    3.0,
    4.0,
    5.0,
    6.0,
    8.0
]

KI_VALUES = [
    0.2,
    0.5,
    1.0,
    1.5,
    2.0
]

KD_VALUES = [
    0.2,
    0.5,
    0.8,
    1.2,
    1.5
]


# ============================================================
# Run One PID Simulation
# ============================================================

def simulate_pid(kp, ki, kd):

    pid = PIDController(
        kp=kp,
        ki=ki,
        kd=kd
    )

    angle = 0.0

    angles = []

    for _ in time:

        error = target_angle - angle

        control = pid.update(
            error,
            DT
        )

        angle += control * DT

        angles.append(angle)

    angles_deg = np.rad2deg(
        angles
    )

    metrics = calculate_step_metrics(
        time,
        angles_deg,
        TARGET_ANGLE_DEG
    )

    return metrics


# ============================================================
# Grid Search
# ============================================================

results = []

total_combinations = (
    len(KP_VALUES)
    * len(KI_VALUES)
    * len(KD_VALUES)
)

combination_number = 0


print()
print("PID Systematic Tuning")
print("=====================")

print(
    f"Testing {total_combinations} "
    f"PID combinations..."
)

print()


for kp in KP_VALUES:

    for ki in KI_VALUES:

        for kd in KD_VALUES:

            combination_number += 1

            metrics = simulate_pid(
                kp,
                ki,
                kd
            )

            results.append({
                "Kp": kp,
                "Ki": ki,
                "Kd": kd,
                "Rise Time": metrics["rise_time"],
                "Overshoot": metrics["overshoot"],
                "Settling Time": metrics["settling_time"],
                "Steady-State Error": (
                    metrics["steady_state_error"]
                )
            })


# ============================================================
# Create DataFrame
# ============================================================

results_df = pd.DataFrame(
    results
)


# ============================================================
# Remove Invalid Results
# ============================================================

valid_results = results_df.dropna(
    subset=[
        "Rise Time",
        "Settling Time",
        "Steady-State Error"
    ]
).copy()


# ============================================================
# Define Tuning Requirements
# ============================================================

MAX_OVERSHOOT = 5.0
MAX_STEADY_STATE_ERROR = 2.0


qualified_results = valid_results[
    (valid_results["Overshoot"] <= MAX_OVERSHOOT)
    &
    (
        valid_results["Steady-State Error"]
        <= MAX_STEADY_STATE_ERROR
    )
].copy()


# ============================================================
# Define Optimization Objective
# ============================================================

if len(qualified_results) > 0:

    # Normalize the two main time metrics.
    rise_max = qualified_results["Rise Time"].max()
    settling_max = qualified_results["Settling Time"].max()

    if rise_max > 0:
        normalized_rise = (
            qualified_results["Rise Time"]
            / rise_max
        )
    else:
        normalized_rise = 0

    if settling_max > 0:
        normalized_settling = (
            qualified_results["Settling Time"]
            / settling_max
        )
    else:
        normalized_settling = 0

    # Weighted objective:
    # 30% rise time
    # 50% settling time
    # 20% steady-state error
    qualified_results["Objective"] = (
        0.30 * normalized_rise
        +
        0.50 * normalized_settling
        +
        0.20 * (
            qualified_results["Steady-State Error"]
            / MAX_STEADY_STATE_ERROR
        )
    )

    tuned_result = qualified_results.sort_values(
        "Objective"
    ).iloc[0]

else:

    tuned_result = None


# ============================================================
# Save All Results
# ============================================================

results_df.to_csv(
    "results/pid_tuning_results.csv",
    index=False
)


# ============================================================
# Print Summary
# ============================================================

print(
    f"Valid combinations: "
    f"{len(valid_results)}"
)

print(
    f"Qualified combinations: "
    f"{len(qualified_results)}"
)

print()


if tuned_result is not None:

    print("Selected PID Parameters")
    print("=======================")

    print(
        f"Kp: {tuned_result['Kp']:.2f}"
    )

    print(
        f"Ki: {tuned_result['Ki']:.2f}"
    )

    print(
        f"Kd: {tuned_result['Kd']:.2f}"
    )

    print()

    print("Performance")
    print("-----------")

    print(
        f"Rise Time:          "
        f"{tuned_result['Rise Time']:.3f} s"
    )

    print(
        f"Overshoot:          "
        f"{tuned_result['Overshoot']:.2f} %"
    )

    print(
        f"Settling Time:      "
        f"{tuned_result['Settling Time']:.3f} s"
    )

    print(
        f"Steady-State Error: "
        f"{tuned_result['Steady-State Error']:.3f} degrees"
    )

    print()

    print(
        "Results saved to:"
    )

    print(
        "results/pid_tuning_results.csv"
    )

else:

    print(
        "No PID combination satisfied "
        "the specified performance requirements."
    )

    print()
    print(
        "Consider expanding the search range."
    )