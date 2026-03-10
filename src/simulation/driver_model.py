"""
Driver model module for lap time simulation.

Provides pure functions and dataclasses that compute driver input channels
(throttle, brake, steering, gear, RPM) from kinematic simulation outputs.

Design principles
-----------------
- Pure functions: no global state, fully unit-testable.
- Decoupled from solver: DriverModel consumes solver outputs (v, ax, radius)
  and produces driver inputs, not the other way around.
- Aligned with Pi Toolbox / MoTeC channel naming convention used in
  Porsche Carrera Cup Brasil telemetry (ref: Pi Toolbox Apostila, 2014).

References
----------
Segers, J. (2014). Analysis Techniques for Racecar Data Acquisition,
  2nd Ed. SAE International.
Milliken, W.F. & Milliken, D.L. (1995). Race Car Vehicle Dynamics.
  SAE International.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from src.vehicle.parameters import VehicleParams


# ---------------------------------------------------------------------------
# Output container
# ---------------------------------------------------------------------------

@dataclass
class DriverInputs:
    """
    Driver input channels for one complete simulation lap.

    All arrays are 1-D numpy arrays of length n (number of track points).
    Channel names and units are aligned with Pi Toolbox / MoTeC output.

    Attributes
    ----------
    throttle_pct : np.ndarray
        Throttle pedal demand [0–100 %].
    brake_pct : np.ndarray
        Brake pedal demand [0–100 %].
    steering_deg : np.ndarray
        Steering wheel angle [deg]. Positive = left turn.
    gear : np.ndarray
        Engaged gear number (integer array).
    rpm : np.ndarray
        Engine rotational speed [rev/min].
    """
    throttle_pct: np.ndarray
    brake_pct: np.ndarray
    steering_deg: np.ndarray
    gear: np.ndarray
    rpm: np.ndarray


# ---------------------------------------------------------------------------
# DriverModel dataclass
# ---------------------------------------------------------------------------

@dataclass
class DriverModel:
    """
    Configurable driver model parameters.

    Attributes
    ----------
    steering_ratio : float
        Steering system ratio (steering wheel angle / road wheel angle).
        Typical GT3: 13–16; Truck: 18–22.
    throttle_lag : float
        First-order lag time constant for throttle response [s].
        Not applied in quasi-steady-state mode (reserved for transient).
    brake_lag : float
        First-order lag time constant for brake response [s].
    smooth_window : int
        Moving-average window length for smoothing input channels.
        1 = no smoothing. Odd number recommended.
    """
    steering_ratio: float = 15.0
    throttle_lag: float = 0.05
    brake_lag: float = 0.03
    smooth_window: int = 1


# ---------------------------------------------------------------------------
# Pure computation functions
# ---------------------------------------------------------------------------

def compute_throttle_brake(
    ax_long: np.ndarray,
    smooth_window: int = 1,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Derive throttle_pct and brake_pct from longitudinal acceleration.

    Uses a normalised inverse model:
    - throttle proportional to positive ax, normalised by peak acceleration.
    - brake proportional to |negative ax|, normalised by peak deceleration.

    Parameters
    ----------
    ax_long : np.ndarray
        Longitudinal acceleration [m/s²]. Positive = accelerating.
    smooth_window : int
        Moving-average smoothing window (1 = disabled).

    Returns
    -------
    throttle_pct, brake_pct : tuple[np.ndarray, np.ndarray]
        Each in range [0, 100] %.
    """
    a_pos = np.clip(ax_long, 0.0, None)
    a_neg = np.clip(-ax_long, 0.0, None)

    peak_accel = max(float(np.max(a_pos)), 1e-6)
    peak_brake = max(float(np.max(a_neg)), 1e-6)

    throttle = np.clip(a_pos / peak_accel * 100.0, 0.0, 100.0)
    brake    = np.clip(a_neg / peak_brake * 100.0, 0.0, 100.0)

    if smooth_window > 1:
        kernel = np.ones(smooth_window) / smooth_window
        throttle = np.convolve(throttle, kernel, mode='same')
        brake    = np.convolve(brake,    kernel, mode='same')
        throttle = np.clip(throttle, 0.0, 100.0)
        brake    = np.clip(brake,    0.0, 100.0)

    return throttle, brake


def compute_gear(
    v_ms: np.ndarray,
    params: VehicleParams,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Select optimal gear at each track point to maximise drive force.

    Strategy: keep RPM in [rpm_idle*1.5, rpm_max*0.90]. Among gears
    within this band, select the one producing the highest wheel force.
    If no gear satisfies the band, select the gear with highest torque
    within rpm_max.

    Parameters
    ----------
    v_ms : np.ndarray
        Vehicle speed [m/s].
    params : VehicleParams
        Structured vehicle parameters.

    Returns
    -------
    gear_arr, rpm_arr : tuple[np.ndarray, np.ndarray]
        Gear (int) and RPM at each point.
    """
    n = len(v_ms)
    gear_arr = np.ones(n, dtype=int)
    rpm_arr  = np.zeros(n)

    # Unpack transmission
    gear_ratios  = params.transmission.gear_ratios
    final_drive  = params.transmission.final_drive_ratio
    r_wheel      = params.tire.wheel_radius
    rpm_idle     = params.engine.rpm_idle
    rpm_max      = params.engine.rpm_max
    rpm_min_opt  = rpm_idle * 1.5
    rpm_max_opt  = rpm_max  * 0.90
    n_gears      = params.transmission.num_gears

    # Torque function: use interpolated map if available, else constant
    use_map = len(params.engine.torque_curve_rpm) > 0
    t_map_rpm = np.array(params.engine.torque_curve_rpm) if use_map else None
    t_map_nm  = np.array(params.engine.torque_curve_nm)  if use_map else None

    def _torque(rpm: float) -> float:
        if use_map:
            return float(np.interp(
                np.clip(rpm, t_map_rpm[0], t_map_rpm[-1]),
                t_map_rpm, t_map_nm
            ))
        # Diesel fallback
        rpm_peak = 1300.0
        T_max = params.engine.max_torque
        if rpm < rpm_idle:
            return 0.0
        elif rpm <= rpm_peak:
            return T_max * (rpm - rpm_idle) / (rpm_peak - rpm_idle)
        elif rpm <= rpm_max:
            return T_max * float(np.exp(-0.0015 * (rpm - rpm_peak) ** 1.2))
        return 0.0

    for i, v in enumerate(v_ms):
        best_gear, best_force, best_rpm = 1, -1.0, rpm_idle
        for g in range(1, n_gears + 1):
            ratio = gear_ratios[g - 1] * final_drive
            rpm = (max(v, 0.01) / r_wheel) * ratio * 60.0 / (2.0 * np.pi)
            if rpm > rpm_max:
                continue
            rpm = max(rpm, rpm_idle)
            T = _torque(rpm)
            F = T * ratio / r_wheel
            in_band = rpm_min_opt <= rpm <= rpm_max_opt
            if in_band and F > best_force:
                best_force = F
                best_gear  = g
                best_rpm   = rpm
            elif best_force < 0.0:  # no in-band gear found yet
                if F > best_force:
                    best_force = F
                    best_gear  = g
                    best_rpm   = rpm
        gear_arr[i] = best_gear
        rpm_arr[i]  = best_rpm

    return gear_arr, rpm_arr


def compute_steering(
    radius: np.ndarray,
    wheelbase: float,
    steering_ratio: float = 15.0,
) -> np.ndarray:
    """
    Estimate steering wheel angle from Ackermann geometry.

    delta_wheel [rad] = L / R  (small angle approximation)
    steering_wheel [deg] = delta_wheel * steering_ratio * (180/pi)

    Sign convention: positive = left turn (positive curvature).
    For quasi-steady-state, sign is unsigned (absolute corner demand);
    lateral sign should be applied externally if needed.

    Parameters
    ----------
    radius : np.ndarray
        Corner radius at each track point [m]. Must be > 0.
    wheelbase : float
        Vehicle wheelbase L [m].
    steering_ratio : float
        Steering gear ratio [-].

    Returns
    -------
    np.ndarray
        Steering wheel angle [deg], always positive (absolute demand).
    """
    delta_rad = wheelbase / np.maximum(radius, 1.0)
    return np.degrees(delta_rad) * steering_ratio


def compute_driver_inputs(
    v_ms: np.ndarray,
    ax_long: np.ndarray,
    radius: np.ndarray,
    params: VehicleParams,
    driver: Optional[DriverModel] = None,
) -> DriverInputs:
    """
    Compute all driver input channels from kinematic solver outputs.

    Convenience wrapper that calls compute_throttle_brake(),
    compute_gear(), and compute_steering() and returns a DriverInputs
    container.

    Parameters
    ----------
    v_ms : np.ndarray
        Vehicle speed [m/s].
    ax_long : np.ndarray
        Longitudinal acceleration [m/s²].
    radius : np.ndarray
        Corner radius [m].
    params : VehicleParams
        Structured vehicle parameters.
    driver : DriverModel | None
        Driver model configuration. Uses defaults if None.

    Returns
    -------
    DriverInputs
    """
    if driver is None:
        driver = DriverModel()

    throttle, brake = compute_throttle_brake(ax_long, driver.smooth_window)
    gear, rpm       = compute_gear(v_ms, params)
    steering        = compute_steering(
        radius,
        wheelbase=params.mass_geometry.wheelbase,
        steering_ratio=driver.steering_ratio,
    )

    return DriverInputs(
        throttle_pct=throttle,
        brake_pct=brake,
        steering_deg=steering,
        gear=gear,
        rpm=rpm,
    )
