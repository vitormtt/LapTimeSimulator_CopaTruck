"""
Simulation modes module for lap time simulator.

Defines the SimulationMode enum and SimulationConfig dataclass that
control how the GGV solver is initialised and executed for each
simulation scenario.

References
----------
- Segers, J. (2014). Analysis Techniques for Racecar Data Acquisition,
  2nd Ed. SAE International.
- Brayshaw, D.L. & Harrison, M.F. (2005). A quasi steady state approach
  to race car lap simulation. Proc. IMechE, Part D.
- Pi Toolbox Apostila de Treinamento — Porsche Carrera Cup Brasil (2014).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

from ..vehicle.setup import VehicleSetup, get_default_setup


class SimulationMode(Enum):
    """
    Enumeration of available simulation scenarios.

    QUALIFYING    : single fastest lap from equilibrium speed.
    FLYING_LAP    : lap from a prescribed constant entry speed.
    STANDING_START: lap from standstill with launch sequence.
    """
    QUALIFYING     = auto()
    FLYING_LAP     = auto()
    STANDING_START = auto()


@dataclass
class SimulationConfig:
    """
    Full configuration for a single simulation run.

    Parameters
    ----------
    mode : SimulationMode
    setup : VehicleSetup
    n_laps : int
    v_entry_kmh : float
        Initial speed for FLYING_LAP [km/h].
    launch_rpm : float
        Clutch-drop RPM for STANDING_START [rev/min].
    track_temperature_c : float
        Track surface temperature [degC].
    tyre_compound : str
    export_driver_inputs : bool
    notes : str
    """
    mode: SimulationMode = SimulationMode.QUALIFYING
    setup: VehicleSetup = field(default_factory=get_default_setup)

    n_laps: int = 1
    track_temperature_c: float = 35.0
    tyre_compound: str = "slick_dry"
    export_driver_inputs: bool = True
    notes: str = ""

    v_entry_kmh: float = 100.0
    launch_rpm: float = 4500.0
    wheelspin_limit_slip: float = 0.25

    def is_qualifying(self) -> bool:
        return self.mode == SimulationMode.QUALIFYING

    def is_flying_lap(self) -> bool:
        return self.mode == SimulationMode.FLYING_LAP

    def is_standing_start(self) -> bool:
        return self.mode == SimulationMode.STANDING_START

    def describe(self) -> str:
        """Human-readable summary string for logging."""
        base = (
            f"[{self.mode.name}] Setup='{self.setup.name}' "
            f"Tyres={self.tyre_compound} T_track={self.track_temperature_c}\u00b0C"
        )
        if self.is_flying_lap():
            base += f" v_entry={self.v_entry_kmh:.1f} km/h"
        if self.is_standing_start():
            base += f" launch_rpm={self.launch_rpm:.0f} rpm"
        return base


def get_default_config(
    mode: SimulationMode = SimulationMode.QUALIFYING,
    setup: Optional[VehicleSetup] = None,
) -> SimulationConfig:
    """Return a ready-to-use SimulationConfig with sensible defaults."""
    return SimulationConfig(
        mode=mode,
        setup=setup if setup is not None else get_default_setup(),
    )


# ---------------------------------------------------------------------------
# Driver input channel specification
# ---------------------------------------------------------------------------

DRIVER_INPUT_CHANNELS = [
    ("distance_m",   "m",    "Cumulative distance along track centreline"),
    ("lap_time_s",   "s",    "Cumulative lap time"),
    ("v_kmh",        "km/h", "Vehicle speed"),
    ("ax_long_g",    "g",    "Longitudinal acceleration (+ = accel, - = braking)"),
    ("ay_lat_g",     "g",    "Lateral acceleration (+ = left, - = right)"),
    ("throttle_pct", "%",    "Throttle pedal / drive torque request [0-100]"),
    ("brake_pct",    "%",    "Brake pedal pressure request [0-100]"),
    ("steering_deg", "deg",  "Steering wheel angle (+ = left)"),
    ("gear",         "-",    "Engaged gear number"),
    ("rpm",          "rpm",  "Engine rotational speed"),
]

DRIVER_INPUT_CHANNEL_NAMES: list = [ch[0] for ch in DRIVER_INPUT_CHANNELS]
