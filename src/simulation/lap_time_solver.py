"""
Simulador de Lap Time para Caminhão Copa Truck
Modelo Bicicleta 2-DOF (Modular) + Aceleração/Frenagem + Motor + Aerodinâmica
"""
import numpy as np
import pandas as pd
import logging
import time
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from src.vehicle.engine import ICEEngine
from src.vehicle.brakes import PneumaticBrake
from src.vehicle.transmission import Transmission
from src.vehicle.tires import PacejkaTire
from src.vehicle.vehicle_model import BicycleVehicle2DOF

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def build_modular_truck_from_dict(params_dict: dict) -> BicycleVehicle2DOF:
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
    start_time = time.time()
    truck = build_modular_truck_from_dict(params_dict)
    
    g = 9.81
    rho = 1.225
    mu_aderencia = config.get("coef_aderencia", truck.tires.mu_y)
    
    Cx = params_dict.get('Cx', 0.85)
    A_front = params_dict.get('A_front', 8.7)
    Cl = params_dict.get('Cl', 0.0)
    
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
    consumo_acum = np.zeros(n)
    
    # PILAR 1: GGV DIAGRAM - Aderência Dinâmica Máxima Lateral
    v_lat_max_profile = np.zeros(n)
    for i in range(n):
        denominador = (truck.mass / radius[i]) - (0.5 * rho * Cl * A_front * mu_aderencia)
        if denominador > 0:
            v_lat_max_profile[i] = np.sqrt((mu_aderencia * truck.mass * g) / denominador)
        else:
            v_lat_max_profile[i] = 250.0 / 3.6 
            
    # Lógica de Marcha e Velocidade Limite Absoluta
    num_gears = len(truck.transmission.gear_ratios)
    highest_gear_ratio = truck.transmission.get_total_ratio(num_gears)
    absolute_v_rpm_limit = (truck.engine.redline_rpm * 2 * np.pi * truck.brakes.wheel_radius) / (60 * highest_gear_ratio)
    
    # FORWARD PASS (Aceleração e limites físicos de curva)
    # Larga de uma velocidade crível para corrida (20 m/s ~ 72 km/h)
    start_speed = 20.0
    v_profile[0] = min(start_speed, v_lat_max_profile[0]) 
    
    for i in range(1, n):
        v_prev = v_profile[i-1]
        
        gear_current = truck.transmission.select_optimal_gear(v_prev, truck.brakes.wheel_radius)
        if gear_current < 4:
            gear_current = 4
        gear_profile[i] = gear_current
        
        ratio_total = truck.transmission.get_total_ratio(gear_current)
        rpm = (v_prev / truck.brakes.wheel_radius) * ratio_total * 60 / (2 * np.pi)
        rpm = np.clip(rpm, truck.engine.idle_rpm, truck.engine.redline_rpm)
        rpm_profile[i-1] = rpm
        
        # O Motor está sempre a 100% no Forward Pass tentando achar o limite
        truck.vx = max(v_prev, 0.1)
        derivadas = truck.calculate_derivatives(throttle=1.0, brake_pedal=0.0, steering_angle=0.0, current_rpm=rpm)
        
        F_traction_engine = derivadas['Fx_total'] 
        
        # Aerodinâmica (Arrasto para segurar e Downforce para grudar)
        F_drag = 0.5 * rho * Cx * A_front * v_prev**2
        F_downforce = 0.5 * rho * Cl * A_front * v_prev**2
        F_normal_dynamic = (truck.mass * g) + F_downforce
        
        # O limite da força de tração NÃO é só o motor, é também o atrito do pneu traseiro contra o asfalto.
        # No forward pass de um RWD puro, ele empurra com tração nas rodas de trás, mas usamos a elipse toda para manter a lógica:
        max_total_grip = mu_aderencia * F_normal_dynamic
        F_lateral = truck.mass * (v_prev**2 / radius[i])
        
        if max_total_grip > F_lateral:
            available_long_grip = np.sqrt(max_total_grip**2 - F_lateral**2)
        else:
            available_long_grip = 0.0
            
        F_traction_actual = min(F_traction_engine, available_long_grip)
        
        # Aceleração Longitudinal instantânea
        a = (F_traction_actual - F_drag) / truck.mass
        
        # Limita aceleração a um valor fisicamente possível (arrancada absurda < 8m/s2 para caminhão)
        if a > 8.0: a = 8.0
        
        a_long[i-1] = a
        
        if ds[i] > 0:
            v_possible = np.sqrt(max(0, v_prev**2 + 2 * a * ds[i]))
            v_profile[i] = min(v_possible, v_lat_max_profile[i], absolute_v_rpm_limit)
        else:
            v_profile[i] = v_prev
            
    # BACKWARD PASS (Frenagem integrando GGV dinâmico + limites reais)
    v_profile[-1] = min(v_profile[-1], v_lat_max_profile[-1])
    for i in reversed(range(n-1)):
        v_next = v_profile[i+1]
        
        F_lateral_next = truck.mass * (v_next**2 / radius[i+1])
        F_downforce_next = 0.5 * rho * Cl * A_front * v_next**2
        F_normal_next = (truck.mass * g) + F_downforce_next
        
        max_total_grip = mu_aderencia * F_normal_next
        
        if max_total_grip > F_lateral_next:
            available_brake_grip = np.sqrt(max_total_grip**2 - F_lateral_next**2)
        else:
            available_brake_grip = 0.0
            
        a_decel_max_friction = available_brake_grip / truck.mass
        
        # O Freio tem um limite térmico/pneumático próprio (aqui chumbado os eixos em 60/40)
        a_decel_max_system = truck.brakes.get_max_deceleration(v_next, F_normal_next * 0.6, F_normal_next * 0.4)
        
        a_decel_brakes = min(a_decel_max_friction, a_decel_max_system)
        
        # Arrasto ajuda o freio a desacelerar livremente
        a_drag_next = (0.5 * rho * Cx * A_front * v_next**2) / truck.mass
        a_decel_effective = a_decel_brakes + a_drag_next
        
        # Trava para desacelerações impossíveis em caminhões (-8 m/s2 máximo)
        if a_decel_effective > 8.0: a_decel_effective = 8.0
        
        if ds[i+1] > 0:
            v_brake_limit = np.sqrt(v_next**2 + 2 * a_decel_effective * ds[i+1])
            v_profile[i] = min(v_profile[i], v_brake_limit)
            
    # TIME E CONSUMO PASS (Atualiza Acelerações reais resultantes e Consumo Termodinâmico)
    time_profile = np.zeros(n)
    for i in range(n):
        a_lat[i] = v_profile[i]**2 / radius[i]
        
        if i < n-1:
            a_long[i] = (v_profile[i+1]**2 - v_profile[i]**2) / (2 * ds[i+1] if ds[i+1] > 0 else 1)
            
        if i > 0 and v_profile[i] > 0:
            dt = ds[i] / v_profile[i]
            time_profile[i] = time_profile[i-1] + dt
            
            F_drag_inst = 0.5 * rho * Cx * A_front * v_profile[i]**2
            F_traction_req = truck.mass * max(0, a_long[i]) + F_drag_inst
            
            potencia_kw = (F_traction_req * v_profile[i]) / 1000
            if potencia_kw > 0:
                consumo_seg = truck.engine.get_fuel_consumption(potencia_kw, dt)
                consumo_acum[i] = consumo_acum[i-1] + consumo_seg
            else:
                consumo_acum[i] = consumo_acum[i-1]
                
    lap_time = time_profile[-1]
    elapsed = time.time() - start_time
    logger.info(f"GGV Solver Concluído em {elapsed:.4f}s. Tempo de volta: {lap_time:.2f}s | V_Max: {np.max(v_profile)*3.6:.1f} km/h")
    
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
        "consumo": consumo_acum,
        "compute_time_s": elapsed
    }
    
    if save_csv and out_path:
        temp_pneu = np.ones(n) * 65.0 
        result["temp_pneu"] = temp_pneu
        
        df = pd.DataFrame({
            "distance_m": s, "x_m": x, "y_m": y, "v_kmh": v_profile * 3.6,
            "a_long_ms2": a_long, "a_lat_ms2": a_lat, "gear": gear_profile,
            "rpm": rpm_profile, "radius_m": radius, "time_s": time_profile,
            "consumo_l": consumo_acum, "temp_pneu_c": temp_pneu
        })
        df.to_csv(out_path, index=False)
        
    return result
