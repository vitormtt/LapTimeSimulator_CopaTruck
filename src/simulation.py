# src/simulation.py
"""
Simulador de Lap Time para Caminhão Copa Truck
Modelo Bicicleta 2DOF + Aceleração/Frenagem + Motor + Aerodinâmica + Limitadores
"""
import numpy as np
import pandas as pd
from dataclasses import dataclass
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class VehicleParams:
    """Parâmetros do veículo"""
    m: float = 5000.0
    lf: float = 2.1
    lr: float = 2.3
    h_cg: float = 1.1
    Cf: float = 120000.0
    Cr: float = 120000.0
    mu: float = 1.1
    r_wheel: float = 0.65
    P_max: float = 600000.0
    T_max: float = 3700.0
    rpm_max: float = 2800.0
    rpm_idle: float = 800.0
    n_gears: int = 12
    gear_ratios: list = None
    final_drive: float = 5.33
    max_decel: float = 7.5
    Cx: float = 0.85
    A_front: float = 8.7
    Cl: float = 0.0
    
    def __post_init__(self):
        if self.gear_ratios is None:
            self.gear_ratios = [14.0, 10.5, 7.8, 5.9, 4.5, 3.5, 2.7, 2.1, 1.6, 1.25, 1.0, 0.78]
        self.L = self.lf + self.lr
        self.Iz = self.m * (self.lf**2 + self.lr**2) / 2

def torque_curve(rpm, params):
    """Curva de torque do motor (curva real diesel pesado)"""
    rpm_torque_max = 1300.0
    if rpm < params.rpm_idle:
        return 0.0
    elif rpm <= rpm_torque_max:
        return params.T_max * (rpm - params.rpm_idle) / (rpm_torque_max - params.rpm_idle)
    elif rpm <= params.rpm_max:
        # Decréscimo suave após pico
        return params.T_max * np.exp(-0.0015 * (rpm - rpm_torque_max)**1.2)
    else:
        return 0.0

def select_gear_optimal(v, rpm_current, params):
    """
    Seleciona marcha ótima para maximizar aceleração.
    Tenta manter RPM em faixa ótima (1200-2200 rpm).
    """
    rpm_min_opt = 1200.0
    rpm_max_opt = 2200.0
    
    best_gear = 1
    best_rpm = 0
    
    for gear in range(1, params.n_gears + 1):
        ratio_total = params.gear_ratios[gear - 1] * params.final_drive
        rpm = (v / params.r_wheel) * ratio_total * 60 / (2 * np.pi)
        rpm = np.clip(rpm, 0, params.rpm_max)
        
        # Penaliza se RPM sair da faixa ótima
        if rpm_min_opt <= rpm <= rpm_max_opt:
            if rpm > best_rpm:
                best_rpm = rpm
                best_gear = gear
        elif rpm > params.rpm_max:
            # Marcha muito alta, ignora
            continue
        else:
            best_gear = gear
            best_rpm = rpm
    
    return best_gear

def get_rpm_from_velocity(v, gear, params):
    """Calcula RPM dada velocidade e marcha"""
    if gear < 1 or gear > params.n_gears:
        return 0
    ratio_total = params.gear_ratios[gear - 1] * params.final_drive
    rpm = (v / params.r_wheel) * ratio_total * 60 / (2 * np.pi)
    return np.clip(rpm, 0, params.rpm_max)

def run_bicycle_model(params_dict, circuit, config, save_csv=True, out_path=None):
    """
    Simulação completa com limitadores, consumo, e temp pneus.
    """
    params = VehicleParams(**params_dict)
    g = 9.81
    rho = 1.225
    
    # Condições de simulação
    mu_aderencia = config.get("coef_aderencia", params.mu)
    consumo_l_100km = config.get("consumo", 43.0)
    temp_pneu_ini = config.get("temp_pneu_ini", 65.0)
    
    # Dados da pista
    x = circuit.centerline_x
    y = circuit.centerline_y
    n = len(x)
    
    ds = np.zeros(n)
    ds[1:] = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    s = np.cumsum(ds)
    
    # Curvatura
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2 + 1e-6)**1.5
    radius = np.where(np.abs(curvature) > 1e-6, 1.0 / np.abs(curvature), 1e6)
    radius = np.clip(radius, 10.0, 1e6)
    
    # Inicializa arrays
    v_profile = np.zeros(n)
    a_long = np.zeros(n)
    a_lat = np.zeros(n)
    gear_profile = np.ones(n, dtype=int)
    rpm_profile = np.zeros(n)
    temp_pneu = np.ones(n) * temp_pneu_ini
    consumo_acum = np.zeros(n)
    
    logger.info(f"Iniciando simulação com {n} pontos de pista (comprimento: {s[-1]:.1f}m)")
    
    # FORWARD PASS
    v_profile[0] = 10.0  # largada lenta
    gear_profile[0] = 1
    
    for i in range(1, n):
        v_prev = v_profile[i-1]
        gear_current = gear_profile[i-1]
        
        # Seleciona marcha ótima
        gear_current = select_gear_optimal(v_prev, 0, params)
        gear_profile[i] = gear_current
        
        # RPM atual
        rpm = get_rpm_from_velocity(v_prev, gear_current, params)
        rpm_profile[i-1] = rpm
        
        # Torque e força de tração
        T_engine = torque_curve(rpm, params)
        ratio_total = params.gear_ratios[gear_current - 1] * params.final_drive
        F_traction = T_engine * ratio_total / params.r_wheel
        
        # Forças resistivas
        F_drag = 0.5 * rho * params.Cx * params.A_front * v_prev**2
        F_downforce = 0.5 * rho * params.Cl * params.A_front * v_prev**2
        
        # Força normal com downforce
        F_normal = params.m * g + F_downforce
        
        # Limita tração por aderência (círculo de força)
        F_trac_max = mu_aderencia * F_normal
        F_traction = min(F_traction, F_trac_max)
        
        # Aceleração longitudinal
        a = (F_traction - F_drag) / params.m
        a_long[i-1] = a
        
        # Limita por RPM máxima
        v_rpm_limit = 150.0 / 3.6  # limite de velocidade para evitar overspeed
        
        # Velocidade lateral máxima
        v_lat_max = np.sqrt(mu_aderencia * g * radius[i])
        
        # Integra velocidade
        if ds[i] > 0:
            v_possible = np.sqrt(max(0, v_prev**2 + 2 * a * ds[i]))
            v_profile[i] = min(v_possible, v_lat_max, v_rpm_limit)
        else:
            v_profile[i] = v_prev
        
        # Temperatura dos pneus (simplificada)
        a_total = np.sqrt(a**2 + (v_prev**2 / max(radius[i], 1))**2)
        temp_pneu[i] = temp_pneu[i-1] + 0.05 * a_total  # aquecimento por aceleração
    
    # BACKWARD PASS (frenagem)
    for i in reversed(range(n-1)):
        v_next = v_profile[i+1]
        
        # Aceleração lateral no próximo ponto
        a_lat_next = v_next**2 / max(radius[i+1], 1.0) if v_next > 0 else 0
        
        # Desaceleração máxima (círculo de aderência)
        a_total_available = mu_aderencia * g
        a_decel_max = np.sqrt(max(0, a_total_available**2 - a_lat_next**2))
        a_decel_max = min(a_decel_max, params.max_decel)
        
        # Velocidade limite para frear a tempo
        if ds[i+1] > 0:
            v_brake_limit = np.sqrt(v_next**2 + 2 * a_decel_max * ds[i+1])
            v_profile[i] = min(v_profile[i], v_brake_limit)
    
    # Calcula aceleração lateral e tempo
    time_profile = np.zeros(n)
    for i in range(n):
        a_lat[i] = v_profile[i]**2 / max(radius[i], 1.0)
        if i > 0 and v_profile[i] > 0:
            dt = ds[i] / v_profile[i] if v_profile[i] > 0 else 0
            time_profile[i] = time_profile[i-1] + dt
            # Consumo (simplificado)
            consumo_acum[i] = consumo_acum[i-1] + (consumo_l_100km / 100.0) * (ds[i] / 1000.0)
    
    lap_time = time_profile[-1]
    
    logger.info(f"✓ Lap time: {lap_time:.2f}s | V_max: {np.max(v_profile)*3.6:.1f} km/h | V_média: {np.mean(v_profile)*3.6:.1f} km/h")
    logger.info(f"✓ Consumo total: {consumo_acum[-1]:.2f}L | Temp pneu máx: {np.max(temp_pneu):.1f}°C")
    
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
        "consumo": consumo_acum
    }
    
    if save_csv and out_path:
        df = pd.DataFrame({
            "distance_m": s,
            "x_m": x,
            "y_m": y,
            "v_kmh": v_profile * 3.6,
            "a_long_ms2": a_long,
            "a_lat_ms2": a_lat,
            "gear": gear_profile,
            "rpm": rpm_profile,
            "radius_m": radius,
            "time_s": time_profile,
            "temp_pneu_c": temp_pneu,
            "consumo_l": consumo_acum
        })
        df.to_csv(out_path, index=False)
        logger.info(f"[OK] Resultados salvos em: {out_path}")
    
    return result
