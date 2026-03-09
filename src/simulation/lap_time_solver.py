"""
Simulador de Lap Time para Caminhão Copa Truck
Modelo Bicicleta 2-DOF (Modular) + Aceleração/Frenagem + Motor + Aerodinâmica
"""
import numpy as np
import pandas as pd
import logging
import time

# Correção dos imports (removendo 'src.' e usando o PYTHONPATH injetado da interface)
from vehicle.engine import ICEEngine
from vehicle.brakes import PneumaticBrake
from vehicle.transmission import Transmission
from vehicle.tires import PacejkaTire
from vehicle.vehicle_model import BicycleVehicle2DOF

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def build_modular_truck_from_dict(params_dict: dict) -> BicycleVehicle2DOF:
    """Constrói o caminhão modular a partir do dicionário."""
    engine = ICEEngine({
        'displacement': 12.0,
        'max_power_kw': params_dict.get('P_max', 600000) / 1000.0,
        'max_power_rpm': 2000,
        'max_torque_nm': params_dict.get('T_max', 3700),
        'max_torque_rpm': 1300,
        'rpm_max': params_dict.get('rpm_max', 2800),
        'rpm_idle': params_dict.get('rpm_idle', 800)
    })
    
    brakes = PneumaticBrake({
        'wheel_radius_m': params_dict.get('r_wheel', 0.65),
        'max_brake_torque_nm': params_dict.get('m', 5000) * 9.81 * params_dict.get('r_wheel', 0.65) * 1.5,
        'chamber_area_cm2': 800
    })
    
    trans = Transmission({
        'gear_ratios': params_dict.get('gear_ratios', [14.0, 10.5, 7.8, 5.9, 4.5, 3.5, 2.7, 2.1, 1.6, 1.25, 1.0, 0.78]),
        'final_drive': params_dict.get('final_drive', 5.33)
    })
    
    tires = PacejkaTire({
        'mu_y': params_dict.get('mu', 1.1),
        'pacejka_b_y': 10.0,
        'pacejka_c_y': 1.3
    })
    
    return BicycleVehicle2DOF(
        mass=params_dict.get('m', 5000.0),
        wheelbase=params_dict.get('lf', 2.1) + params_dict.get('lr', 2.3),
        a=params_dict.get('lf', 2.1),
        cg_height=params_dict.get('h_cg', 1.1),
        izz=params_dict.get('m', 5000.0) * (params_dict.get('lf', 2.1)**2 + params_dict.get('lr', 2.3)**2) / 2,
        engine_sys=engine,
        brake_sys=brakes,
        trans_sys=trans,
        tire_sys=tires
    )

def run_bicycle_model(params_dict, circuit, config, save_csv=True, out_path=None):
    """
    Simulação Quasi-Steady-State baseada na arquitetura modular.
    """
    start_time = time.time()
    
    truck = build_modular_truck_from_dict(params_dict)
    
    g = 9.81
    rho = 1.225
    mu_aderencia = config.get("coef_aderencia", truck.tires.mu_y)
    consumo_l_100km = config.get("consumo", 43.0)
    temp_pneu_ini = config.get("temp_pneu_ini", 65.0)
    
    Cx = params_dict.get('Cx', 0.85)
    A_front = params_dict.get('A_front', 8.7)
    Cl = params_dict.get('Cl', 0.0)
    max_decel = params_dict.get('max_decel', 7.5)
    
    # Pre-calculos vetorizados para ganhar performance
    x = circuit.centerline_x
    y = circuit.centerline_y
    n = len(x)
    
    ds = np.zeros(n)
    ds[1:] = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    s = np.cumsum(ds)
    
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2 + 1e-6)**1.5
    radius = np.where(np.abs(curvature) > 1e-6, 1.0 / np.abs(curvature), 1e6)
    radius = np.clip(radius, 10.0, 1e6)
    
    v_profile = np.zeros(n)
    a_long = np.zeros(n)
    a_lat = np.zeros(n)
    gear_profile = np.ones(n, dtype=int)
    rpm_profile = np.zeros(n)
    temp_pneu = np.ones(n) * temp_pneu_ini
    consumo_acum = np.zeros(n)
    
    # FORWARD PASS (Aceleração)
    v_profile[0] = 10.0  
    gear_profile[0] = 1
    
    for i in range(1, n):
        v_prev = v_profile[i-1]
        
        gear_current = truck.transmission.select_optimal_gear(v_prev, truck.brakes.wheel_radius)
        gear_profile[i] = gear_current
        
        ratio_total = truck.transmission.get_total_ratio(gear_current)
        rpm = (v_prev / truck.brakes.wheel_radius) * ratio_total * 60 / (2 * np.pi)
        rpm = np.clip(rpm, truck.engine.idle_rpm, truck.engine.redline_rpm)
        rpm_profile[i-1] = rpm
        
        truck.vx = max(v_prev, 0.1)
        derivadas = truck.calculate_derivatives(throttle=1.0, brake_pedal=0.0, steering_angle=0.0, current_rpm=rpm)
        
        F_traction = derivadas['Fx_total'] 
        
        F_drag = 0.5 * rho * Cx * A_front * v_prev**2
        F_downforce = 0.5 * rho * Cl * A_front * v_prev**2
        
        v_lat_max = np.sqrt(mu_aderencia * g * radius[i]) 
        
        a = (F_traction - F_drag) / truck.mass
        a_long[i-1] = a
        
        v_rpm_limit = (truck.engine.redline_rpm * 2 * np.pi * truck.brakes.wheel_radius) / (60 * truck.transmission.get_total_ratio(truck.transmission.gear_ratios[-1]))
        
        if ds[i] > 0:
            v_possible = np.sqrt(max(0, v_prev**2 + 2 * a * ds[i]))
            v_profile[i] = min(v_possible, v_lat_max, v_rpm_limit)
        else:
            v_profile[i] = v_prev
            
        a_total = np.sqrt(a**2 + (v_prev**2 / max(radius[i], 1))**2)
        temp_pneu[i] = temp_pneu[i-1] + 0.05 * a_total
        
    # BACKWARD PASS (Frenagem)
    for i in reversed(range(n-1)):
        v_next = v_profile[i+1]
        
        a_lat_next = v_next**2 / max(radius[i+1], 1.0) if v_next > 0 else 0
        a_total_available = mu_aderencia * g
        a_decel_max = np.sqrt(max(0, a_total_available**2 - a_lat_next**2))
        a_decel_max = min(a_decel_max, max_decel)
        
        if ds[i+1] > 0:
            v_brake_limit = np.sqrt(v_next**2 + 2 * a_decel_max * ds[i+1])
            v_profile[i] = min(v_profile[i], v_brake_limit)
            
    # TIME PASS
    time_profile = np.zeros(n)
    for i in range(n):
        a_lat[i] = v_profile[i]**2 / max(radius[i], 1.0)
        if i > 0 and v_profile[i] > 0:
            dt = ds[i] / v_profile[i] if v_profile[i] > 0 else 0
            time_profile[i] = time_profile[i-1] + dt
            
            potencia_kw = (F_traction * v_profile[i]) / 1000
            if potencia_kw > 0:
                consumo_seg = truck.engine.get_fuel_consumption(potencia_kw, dt)
                consumo_acum[i] = consumo_acum[i-1] + consumo_seg
            else:
                consumo_acum[i] = consumo_acum[i-1]
                
    lap_time = time_profile[-1]
    
    elapsed = time.time() - start_time
    logger.info(f"Simulação concluída em {elapsed:.4f}s. Tempo de volta: {lap_time:.2f}s")
    
    result = {
        "lap_time": lap_time,
        "distance": s,
        "v_profile": v_profile,
        "a_long": a_long,
        "a_lat": a_lat,
        "gear": gear_profile,
        "rpm": rpm_profile,
        "radius": radius,
        "time": time_profile,
        "temp_pneu": temp_pneu,
        "consumo": consumo_acum,
        "compute_time_s": elapsed
    }
    
    if save_csv and out_path:
        df = pd.DataFrame({
            "distance_m": s, "x_m": x, "y_m": y, "v_kmh": v_profile * 3.6,
            "a_long_ms2": a_long, "a_lat_ms2": a_lat, "gear": gear_profile,
            "rpm": rpm_profile, "radius_m": radius, "time_s": time_profile,
            "temp_pneu_c": temp_pneu, "consumo_l": consumo_acum
        })
        df.to_csv(out_path, index=False)
        
    return result
