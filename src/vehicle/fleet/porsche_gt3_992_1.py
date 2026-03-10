"""
Porsche 911 GT3 Cup (992 Fase 1) — vehicle parameters for lap time simulation.

Model year: 2021–present (Carrera Cup Brasil / GT3 Cup Challenge Brasil)
Engine:     4.0L NA flat-6, 373 kW (510 hp) @ 8.400 rpm
Transmission: 6-speed sequential (Porsche PDK-derived, updated logic)
Category:   GT3 Cup

Key differences vs. 991 Fase 2:
    - Power increase: 338 kW → 373 kW (+10.4%)
    - Higher RPM peak power: 8.400 rpm (vs. 7.600 rpm)
    - Updated aerodynamic package (improved Cl/Cd ratio)
    - Stiffer chassis (higher Iz estimated)
    - TPMS fully integrated with 4-corner pressure/temperature display

Parameter sources:
    - Porsche Motorsport 992 GT3 Cup Technical Description (2021)
    - Porsche Newsroom (2021 spec sheet)
    - Extrapolation from 991.2 parameters with documented delta values
    - Rajamani (2012), Pacejka (2012)

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


def porsche_gt3_cup_992_1() -> VehicleParams:
    """
    Default parameters for Porsche 911 GT3 Cup 992 Fase 1.

    Returns:
        VehicleParams: Complete vehicle parameter set.
    """
    return VehicleParams(
        mass_geometry=VehicleMassGeometry(
            mass=1080.0,              # [kg] — 992 mín. regulamento ~1.055 kg + piloto
            lf=1.34,
            lr=1.525,
            wheelbase=2.459,          # [m] — 992 tem wheelbase ligeiramente maior
            track_width_front=1.600,  # [m] — 992 track width aumentado
            track_width_rear=1.580,
            cg_height=0.475,          # [m] — CG ligeiramente mais baixo (chassi 992)
            Iz=1300.0,                # [kg·m²] — chassi mais rígido, massa distribuída
            Ix=390.0,
            Iy=1150.0,
        ),
        tire=TireParams(
            cornering_stiffness_front=78000.0,   # [N/rad] — pneu 18" Michelin atualizado
            cornering_stiffness_rear=88000.0,
            friction_coefficient=1.60,           # [-] — compound levemente superior
            wheel_radius=0.330,
            pacejka_B=12.5,
            pacejka_C=1.38,
            pacejka_D=1.60,
            pacejka_E=0.88,
        ),
        aero=AeroParams(
            drag_coefficient=0.36,     # [-]   — pacote aerodinâmico 992 mais eficiente
            frontal_area=1.93,         # [m²]  — 992 ligeiramente maior
            lift_coefficient=-1.05,    # [-]   — maior downforce vs. 991 (~+24%)
            air_density=1.225,
        ),
        engine=EngineParams(
            max_power=373000.0,        # [W]   — 373 kW / 510 hp
            max_torque=470.0,          # [N·m] — @ ~7.000 rpm
            rpm_max=8750.0,            # [rpm] — pico mais alto vs. 991
            rpm_idle=900.0,
            rpm_redline=8750.0,
            # Torque curve (4.0L NA 992 — otimizado para alto RPM)
            torque_curve_rpm=[
                1000.0, 2000.0, 3000.0, 4000.0, 5000.0,
                6000.0, 7000.0, 7500.0, 8000.0, 8400.0, 8750.0
            ],
            torque_curve_nm=[
                210.0, 290.0, 360.0, 410.0, 450.0,
                468.0, 470.0, 462.0, 445.0, 420.0, 370.0
            ],
            max_coolant_temp=110.0,
            max_oil_temp=140.0,
        ),
        transmission=TransmissionParams(
            num_gears=6,
            # 992 — ratios ajustados para maior potência e RPM de pico
            gear_ratios=[3.91, 2.35, 1.68, 1.30, 1.06, 0.88],
            final_drive_ratio=3.44,
            shift_time=0.045,          # [s] — sistema pneumático mais rápido no 992
            upshift_rpm=8550.0,
            downshift_rpm=5700.0,
            transmission_efficiency=0.96,
        ),
        brake=BrakeParams(
            max_brake_force=30000.0,   # [N]  — discos maiores + pinças atualizadas
            brake_balance=52.0,
            max_deceleration=23.0,     # [m/s²] — ~2.35 g (maior downforce)
            brake_response_time=0.045,
            abs_enabled=True,
            abs_slip_target=0.10,
        ),
        name="Porsche 911 GT3 Cup (992 Fase 1)",
        manufacturer="Porsche",
        year=2021,
        category="GT3_Cup",
    )
