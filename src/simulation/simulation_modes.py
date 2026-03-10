"""
Simulation modes definition for the lap time simulator.

Defines the available simulation scenarios and their configuration:
    - STANDING_START  : launch from rest (v0 = 0), includes clutch slip model
    - ROLLING_START   : constant-velocity launch (v0 = configurable)
    - QUALIFYING      : single fastest lap, no fuel/tyre degradation
    - RACE            : multi-lap with tyre/fuel degradation (future)

For each mode, the SimulationConfig dataclass holds:
    - Initial conditions
    - Driver model parameters (shift RPM, brake aggressiveness, etc.)
    - Target KPIs (what the optimizer minimizes)
    - Telemetry channels to log

Author: Lap Time Simulator Team
Date: 2026-03-10
References:
    - Hakewill, J. (2010). Lap Time Simulation Model for Racing Cars.
    - Jain et al. (2020). Computing the racing line using Bayesian optimization. arXiv:2002.04794
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class SimulationMode(Enum):
    """
    Enumeration of supported simulation scenarios.

    Each mode determines:
        - Initial velocity condition
        - Whether tyre/fuel degradation is active
        - Which driver model parameters are used
        - Target optimization metric
    """
    STANDING_START = "standing_start"   # Largada parada (v0 = 0)
    ROLLING_START  = "rolling_start"    # Largada em velocidade constante
    QUALIFYING     = "qualifying"        # Volta única mais rápida
    RACE           = "race"             # Corrida com degradação (futuro)


@dataclass
class DriverInputChannels:
    """
    Telemetry channels for driver input monitoring and optimization.

    These are the outputs to be logged at each simulation time step
    and compared against real telemetry during validation.

    All channels are time-series arrays aligned with the simulation time vector.
    """
    # Control inputs (0–1 normalized unless stated)
    throttle: List[float] = field(default_factory=list)       # [0–1]  — acelerador
    brake: List[float] = field(default_factory=list)          # [0–1]  — freio normalizado
    steering_angle: List[float] = field(default_factory=list) # [rad]  — ângulo de volante
    gear: List[int] = field(default_factory=list)             # [1–6]  — marcha engajada

    # Timing channels
    shift_events: List[float] = field(default_factory=list)   # [s]    — timestamps de trocas
    braking_points: List[float] = field(default_factory=list) # [m]    — distância de frenagem

    # Derived dynamics
    long_g: List[float] = field(default_factory=list)         # [m/s²] — acc. longitudinal
    lat_g: List[float] = field(default_factory=list)          # [m/s²] — acc. lateral
    speed: List[float] = field(default_factory=list)          # [km/h] — velocidade
    rpm: List[float] = field(default_factory=list)            # [rpm]  — rotação do motor


@dataclass
class SimulationConfig:
    """
    Full configuration for a single simulation run.

    Bundles the simulation mode, initial conditions, and driver model
    parameters into a single object passed to the solver.

    Attributes:
        mode            : SimulationMode — scenario type
        v0              : Initial velocity [m/s] (0 for standing start)
        lap_count       : Number of laps to simulate
        dt              : Integration time step [s]
        upshift_rpm     : RPM threshold for automatic upshift
        downshift_rpm   : RPM threshold for automatic downshift
        brake_efficiency: Driver braking efficiency scalar [0–1]
        throttle_smoothing: Low-pass filter coefficient for throttle [0–1]
        track_id        : Track identifier string (e.g., 'interlagos')
        notes           : Free-text annotation for this run
    """
    mode: SimulationMode = SimulationMode.QUALIFYING

    # Initial conditions
    v0: float = 0.0          # [m/s] — initial velocity
    lap_count: int = 1       # number of laps (>1 activates degradation in RACE mode)

    # Solver parameters
    dt: float = 0.01         # [s] — integration timestep (100 Hz)

    # Driver model — shift strategy
    upshift_rpm: Optional[float] = None    # None → use vehicle default
    downshift_rpm: Optional[float] = None  # None → use vehicle default

    # Driver model — inputs
    brake_efficiency: float = 1.0          # [-] 1.0 = optimal braking
    throttle_smoothing: float = 0.05       # [-] lower = sharper response

    # Standing start parameters (only used in STANDING_START mode)
    clutch_slip_duration: float = 0.4      # [s] — clutch engagement ramp
    launch_rpm: float = 4500.0             # [rpm] — launch control target RPM

    # Metadata
    track_id: str = "interlagos"
    notes: str = ""

    def __post_init__(self) -> None:
        """Enforce mode-specific initial condition constraints."""
        if self.mode == SimulationMode.STANDING_START and self.v0 != 0.0:
            self.v0 = 0.0  # force zero initial velocity
        if self.mode == SimulationMode.QUALIFYING:
            self.lap_count = 1  # qualifying = always single lap

    @classmethod
    def qualifying(cls, track_id: str = "interlagos", dt: float = 0.01) -> 'SimulationConfig':
        """Shortcut constructor for qualifying simulation."""
        return cls(
            mode=SimulationMode.QUALIFYING,
            v0=0.0,
            lap_count=1,
            dt=dt,
            track_id=track_id,
        )

    @classmethod
    def standing_start(cls, track_id: str = "interlagos", dt: float = 0.01) -> 'SimulationConfig':
        """Shortcut constructor for standing start simulation."""
        return cls(
            mode=SimulationMode.STANDING_START,
            v0=0.0,
            lap_count=1,
            dt=dt,
            track_id=track_id,
        )

    @classmethod
    def rolling_start(cls, v0_kmh: float, track_id: str = "interlagos") -> 'SimulationConfig':
        """Shortcut constructor for rolling start at constant speed."""
        return cls(
            mode=SimulationMode.ROLLING_START,
            v0=v0_kmh / 3.6,
            lap_count=1,
            track_id=track_id,
        )
