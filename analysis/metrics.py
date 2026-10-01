import numpy as np


def calculate_step_metrics(time, response, target):
    """
    Calculate standard step-response performance metrics.

    Metrics:
        - Rise time (10% to 90%)
        - Percentage overshoot
        - Settling time (2% band)
        - Steady-state error
    """

    time = np.asarray(time)
    response = np.asarray(response)

    # -----------------------------
    # Rise Time
    # -----------------------------

    lower_threshold = 0.10 * target
    upper_threshold = 0.90 * target

    lower_indices = np.where(response >= lower_threshold)[0]
    upper_indices = np.where(response >= upper_threshold)[0]

    if len(lower_indices) > 0 and len(upper_indices) > 0:
        t10 = time[lower_indices[0]]
        t90 = time[upper_indices[0]]
        rise_time = t90 - t10
    else:
        rise_time = np.nan

    # -----------------------------
    # Overshoot
    # -----------------------------

    peak = np.max(response)

    overshoot = max(
        0.0,
        ((peak - target) / abs(target)) * 100
    )

    # -----------------------------
    # Settling Time
    # 2% tolerance band
    # -----------------------------

    tolerance = 0.02 * abs(target)

    lower_bound = target - tolerance
    upper_bound = target + tolerance

    outside_band = np.where(
        (response < lower_bound) |
        (response > upper_bound)
    )[0]

    if len(outside_band) > 0:
        last_outside_index = outside_band[-1]

        if last_outside_index < len(time) - 1:
            settling_time = time[last_outside_index + 1]
        else:
            settling_time = np.nan
    else:
        settling_time = 0.0

    # -----------------------------
    # Steady-State Error
    # -----------------------------

    steady_state_error = abs(
        target - response[-1]
    )

    return {
        "rise_time": rise_time,
        "overshoot": overshoot,
        "settling_time": settling_time,
        "steady_state_error": steady_state_error
    }