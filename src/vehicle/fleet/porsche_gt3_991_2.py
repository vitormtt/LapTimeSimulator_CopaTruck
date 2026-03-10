"""
Porsche 911 GT3 Cup (991 Fase 2) — vehicle parameters for lap time simulation.

Model year: 2017–2019 (Carrera Cup Brasil / GT3 Cup Challenge Brasil)
Engine:     4.0L NA flat-6, 338 kW (460 hp) @ 7.600 rpm
Transmission: 6-speed sequential pneumatic (APS — revised logic vs. Fase 1)
Category:   GT3 Cup

Diferences vs. 991 Fase 1:
    - Novo motor 4.0L (maior deslocamento, curva de torque mais plana na faixa média)
    - Novo ABS com chave de 12 posições (seco/molhado) — parâmetro abs_enabled=True
    - Sensação de troca de marchas mais suave (shift_time ligeiramente maior)
    - TPMS integrado ao painel

Parameter sources:
    - ENG210319 Manual 991.2 — Carrera Cup Brasil (2021) [internal]
    - Porsche Motorsport Technical Specifications 991.2 (2017)
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


def porsche_gt3_cup_991_2() -> VehicleParams:
    """
    Default parameters for Porsche 911 GT3 Cup 991 Fase 2.

    Returns:
        VehicleParams: Complete vehicle parameter set.
    """
    return VehicleParams(
        mass_geometry=VehicleMassGeometry(
            mass=1050.0,              # [kg] — mesma massa mínima do regulamento
            lf=1.33,
            lr=1.52,
            wheelbase=2.455,
            track_width_front=1.580,
            track_width_rear=1.550,
            cg_height=0.480,
            Iz=1260.0,               # [kg·m²] — ligeiramente maior (motor 4.0L mais pesado)
            Ix=380.0,
            Iy=1110.0,
        ),
        tire=TireParams(
            cornering_stiffness_front=75000.0,
            cornering_stiffness_rear=85000.0,
            friction_coefficient=1.55,
            wheel_radius=0.330,
            pacejka_B=12.0,
            pacejka_C=1.35,
            pacejka_D=1.55,
            pacejka_E=0.90,
        ),
        aero=AeroParams(
            drag_coefficient=0.38,
            frontal_area=1.90,
            lift_coefficient=-0.85,
            air_density=1.225,
        ),
        engine=EngineParams(
            max_power=338000.0,       # [W]   — mantém 338 kW (classe equalizada)
            max_torque=460.0,         # [N·m] — motor 4.0L tem torque médio ligeiramente superior
            rpm_max=8500.0,
            rpm_idle=900.0,
            rpm_redline=8500.0,
            # Torque curve (4.0L NA — curva mais plana na faixa média vs. 3.8L)
            torque_curve_rpm=[
                1000.0, 2000.0, 3000.0, 4000.0, 5000.0,
                6000.0, 6500.0, 7000.0, 7500.0, 8000.0, 8500.0
            ],
            torque_curve_nm=[
                220.0, 300.0, 370.0, 410.0, 440.0,
                460.0, 455.0, 445.0, 430.0, 400.0, 350.0
            ],
            max_coolant_temp=110.0,
            max_oil_temp=140.0,
        ),
        transmission=TransmissionParams(
            num_gears=6,
            gear_ratios=[3.82, 2.30, 1.64, 1.27, 1.03, 0.86],  # iguais ao 991.1
            final_drive_ratio=3.44,
            shift_time=0.060,         # [s] — troca mais suave, ligeiramente maior
            upshift_rpm=8300.0,
            downshift_rpm=5500.0,
            transmission_efficiency=0.96,
        ),
        brake=BrakeParams(
            max_brake_force=28000.0,
            brake_balance=52.0,
            max_deceleration=22.0,
            brake_response_time=0.05,
            abs_enabled=True,         # 991 Fase 2 possui ABS com chave 12 posições
            abs_slip_target=0.10,     # posição 6 (pista seca) — referência ENG210319
        ),
        name="Porsche 911 GT3 Cup (991 Fase 2)",
        manufacturer="Porsche",
        year=2017,
        category="GT3_Cup",
    )
