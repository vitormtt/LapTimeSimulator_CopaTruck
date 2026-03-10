"""
Porsche 911 GT3 Cup (991 Fase 1) — vehicle parameters for lap time simulation.

Model year: 2013–2016 (Carrera Cup Brasil / GT3 Cup Challenge Brasil)
Engine:     3.8L NA flat-6, 338 kW (460 hp) @ 7.500 rpm
Transmission: 6-speed sequential pneumatic (APS paddle shift)
Category:   GT3 Cup

Parameter sources:
    - Porsche Motorsport GT3 Cup Type 991 Technical Description (IMSA, 2017)
    - ENG210319 Manual 991.1 — Carrera Cup Brasil (2021) [internal]
    - Estimativas baseadas em literatura: Rajamani (2012), Pacejka (2012)
    - Massa com piloto ~1.050 kg (mín. regulamento 2014: 1.025 kg + ~25 kg piloto)

Author: Lap Time Simulator Team
Date: 2026-03-10
"""

from ..parameters import (
    VehicleParams,
    VehicleMassGeometry,
    TireParams,
    AeroParams,
    EngineParams,
    TransmissionParams,
    BrakeParams,
)


def porsche_gt3_cup_991_1() -> VehicleParams:
    """
    Default parameters for Porsche 911 GT3 Cup 991 Fase 1.

    ARB and wing set to mid-range defaults (position 4/7 front, 4/7 rear, wing 5/9).
    Apply VehicleSetup via setup.apply_setup() to override.

    Returns:
        VehicleParams: Complete vehicle parameter set.
    """
    return VehicleParams(
        mass_geometry=VehicleMassGeometry(
            mass=1050.0,           # [kg] — includes driver (~80 kg)
            lf=1.33,               # [m]  — CG to front axle (rear-engine: shorter front)
            lr=1.52,               # [m]  — CG to rear axle
            wheelbase=2.455,       # [m]  — lf + lr (Porsche 911 reference)
            track_width_front=1.580,  # [m]  — Carrera Cup front track
            track_width_rear=1.550,   # [m]  — Carrera Cup rear track
            cg_height=0.480,       # [m]  — estimated GT3 Cup (low CG, Porsche 911)
            Iz=1250.0,             # [kg·m²] — yaw inertia (estimated, Rajamani 2012)
            Ix=380.0,              # [kg·m²] — roll inertia (for future 3DOF+)
            Iy=1100.0,             # [kg·m²] — pitch inertia (for future models)
        ),
        tire=TireParams(
            cornering_stiffness_front=75000.0,   # [N/rad] — Michelin slick GT3
            cornering_stiffness_rear=85000.0,    # [N/rad] — rear higher (rear-engine load)
            friction_coefficient=1.55,           # [-]     — slick tyre peak mu
            wheel_radius=0.330,                  # [m]     — 18" Michelin Cup2
            # Pacejka Magic Formula (Michelin Pilot Sport Cup 2 estimate)
            pacejka_B=12.0,
            pacejka_C=1.35,
            pacejka_D=1.55,
            pacejka_E=0.90,
        ),
        aero=AeroParams(
            drag_coefficient=0.38,    # [-]   — Cd base (wing pos. 5/9)
            frontal_area=1.90,        # [m²]  — 911 GT3 Cup frontal area
            lift_coefficient=-0.85,   # [-]   — Cl (negative = downforce), wing pos. 5/9
            # Breakdown: front splitter ~-0.20, rear wing ~-0.65
            air_density=1.225,        # [kg/m³]
        ),
        engine=EngineParams(
            max_power=338000.0,        # [W]       — 338 kW / 460 hp
            max_torque=440.0,          # [N·m]     — @ ~6.000 rpm
            rpm_max=8500.0,            # [rpm]     — hard rev limiter
            rpm_idle=900.0,            # [rpm]
            rpm_redline=8500.0,        # [rpm]
            # Torque curve (estimated from dyno data — Porsche 3.8L NA)
            torque_curve_rpm=[
                1000.0, 2000.0, 3000.0, 4000.0, 5000.0,
                6000.0, 6500.0, 7000.0, 7500.0, 8000.0, 8500.0
            ],
            torque_curve_nm=[
                200.0, 280.0, 350.0, 390.0, 420.0,
                440.0, 435.0, 425.0, 410.0, 385.0, 340.0
            ],
            max_coolant_temp=110.0,    # [°C] — alarm threshold (manual ENG210319)
            max_oil_temp=140.0,        # [°C] — alarm threshold
        ),
        transmission=TransmissionParams(
            num_gears=6,
            # 991 Fase 1 — Porsche PDK sequential 6-speed (estimated ratios)
            gear_ratios=[3.82, 2.30, 1.64, 1.27, 1.03, 0.86],
            final_drive_ratio=3.44,    # [-] — Porsche 911 GT3 Cup standard
            shift_time=0.050,          # [s] — APS pneumatic shift ~50 ms
            upshift_rpm=8300.0,        # [rpm]
            downshift_rpm=5500.0,      # [rpm]
            transmission_efficiency=0.96,
        ),
        brake=BrakeParams(
            max_brake_force=28000.0,   # [N]  — 4-piston Brembo, 380mm discs
            brake_balance=52.0,        # [%]  — default front bias (Porsche rec: -1.0 bias knob)
            max_deceleration=22.0,     # [m/s²] — slick tyre peak (~2.2 g)
            brake_response_time=0.05,  # [s]
            abs_enabled=False,         # 991 Fase 1 does not have ABS
            abs_slip_target=0.15,
        ),
        name="Porsche 911 GT3 Cup (991 Fase 1)",
        manufacturer="Porsche",
        year=2014,
        category="GT3_Cup",
    )
