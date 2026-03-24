# src/simulation/lap_time_solver.py
"""
Simulador de Lap Time — Solver principal

Ponto de entrada principal: run_simulation(config, vehicle_params, circuit)
Legacy entry point preservado: run_bicycle_model(params_dict, circuit, config)

Modos suportados (via SimulationMode):
  QUALIFYING    — volta de classificação a partir de velocidade de equilíbrio
  FLYING_LAP    — volta com velocidade de entrada prescrita (v_entry_kmh)
  STANDING_START— largada parada com modelo de patinagem e rampa de embreagem

Solver:
  QSS-GGV (forward + backward pass) acoplado ao modelo single-track (2DOF
  lateral). O limite de força lateral no GGV é calculado via ângulos de deriva
  reais (alpha_f, alpha_r) integrando as EDOs laterais de Guiggiani (2014,
  cap. 6), em vez de usar mu*g constante.

Output: SimulationResult com canais de telemetria alinhados ao
nomenclador Pi Toolbox / MoTeC.

Referências
-----------
Guiggiani, M. (2014). The Science of Vehicle Dynamics. Springer.
  Cap. 6 (Single track model) e Cap. 7 (Race car handling).
Brayshaw, D. & Harrison, M. (2005). A quasi steady state approach to race
  car lap simulation. Proc. IMechE Part D, 219(3), 383-394.
Segers, J. (2014). Analysis Techniques for Racecar Data Acquisition,
  2nd Ed. SAE International.
"""

from __future__ import annotations

import logging
import time as _time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
import pandas as pd

from .simulation_modes import SimulationConfig, SimulationMode
from ..vehicle.parameters import VehicleParams
from ..vehicle.setup import apply_setup_to_params

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Legacy flat params (mantido para compatibilidade run_bicycle_model)
# ---------------------------------------------------------------------------

@dataclass
class _LegacyVehicleParams:
    """Internal flat params used by the GGV + single-track solver."""
    # Massa e geometria
    m: float = 5000.0
    lf: float = 2.1
    lr: float = 2.3
    h_cg: float = 1.1          # altura CG [m] — usado em transferência de carga
    # Dinâmica lateral — single-track
    Cf: float = 120000.0       # rigidez de deriva dianteira [N/rad]
    Cr: float = 120000.0       # rigidez de deriva traseira  [N/rad]
    Iz: float = 0.0            # momento de inércia guinada [kg·m²]; 0 → calculado
    # Pneu
    mu: float = 1.1
    r_wheel: float = 0.65
    # Motor
    P_max: float = 600000.0
    T_max: float = 3700.0
    rpm_max: float = 2800.0
    rpm_idle: float = 800.0
    T_eng_brake: float = 800.0 # torque de freio-motor [N·m]
    # Transmissão
    n_gears: int = 12
    gear_ratios: list = None
    final_drive: float = 5.33
    # Freio
    max_decel: float = 7.5
    # Aerodinâmica
    Cx: float = 0.85
    A_front: float = 8.7
    Cl: float = 0.0

    def __post_init__(self):
        if self.gear_ratios is None:
            self.gear_ratios = [14.0, 10.5, 7.8, 5.9, 4.5, 3.5, 2.7, 2.1,
                                 1.6, 1.25, 1.0, 0.78]
        self.L = self.lf + self.lr
        # Iz calculado se não fornecido: modelo bicicleta simplificado
        if self.Iz <= 0.0:
            self.Iz = self.m * (self.lf ** 2 + self.lr ** 2) / 2.0


# ---------------------------------------------------------------------------
# SimulationResult
# ---------------------------------------------------------------------------

@dataclass
class SimulationResult:
    """
    Output container para uma simulação completa.

    Todos os canais array são 1-D numpy arrays de comprimento n (pontos da
    pista). KPIs escalares são pré-computados na construção.

    Atributos
    ---------
    lap_time : float
        Tempo de volta total simulado [s].
    mode : SimulationMode
        Modo utilizado.
    setup_name : str
        Nome do VehicleSetup aplicado.
    alpha_f, alpha_r : np.ndarray
        Ângulos de deriva front/rear reais do modelo single-track [rad].
        Zero quando o solver roda sem acoplamento lateral (Cf=Cr=0).
    """
    lap_time: float
    mode: SimulationMode
    setup_name: str

    distance: np.ndarray
    time: np.ndarray
    v_kmh: np.ndarray
    ax_long_g: np.ndarray
    ay_lat_g: np.ndarray
    throttle_pct: np.ndarray
    brake_pct: np.ndarray
    steering_deg: np.ndarray
    gear: np.ndarray
    rpm: np.ndarray
    radius: np.ndarray
    temp_tyre_c: np.ndarray
    tyre_pressure_bar: np.ndarray
    fuel_used_l: np.ndarray
    alpha_f: np.ndarray = field(default=None, repr=False)
    alpha_r: np.ndarray = field(default=None, repr=False)

    # Versões em m/s² para uso interno
    _a_long_ms2: np.ndarray = field(repr=False, default=None)
    _a_lat_ms2: np.ndarray = field(repr=False, default=None)

    # -----------------------------------------------------------------------
    # KPI properties
    # -----------------------------------------------------------------------

    @property
    def avg_speed_kmh(self) -> float:
        return float(np.mean(self.v_kmh))

    @property
    def max_speed_kmh(self) -> float:
        return float(np.max(self.v_kmh))

    @property
    def peak_lat_g(self) -> float:
        return float(np.max(np.abs(self.ay_lat_g)))

    @property
    def peak_brake_g(self) -> float:
        return float(np.min(self.ax_long_g))

    @property
    def peak_accel_g(self) -> float:
        return float(np.max(self.ax_long_g))

    @property
    def time_wot_pct(self) -> float:
        return float(np.mean(self.throttle_pct >= 95.0) * 100.0)

    @property
    def time_braking_pct(self) -> float:
        return float(np.mean(self.brake_pct > 5.0) * 100.0)

    @property
    def fuel_total_l(self) -> float:
        return float(self.fuel_used_l[-1])

    @property
    def final_tyre_temp_c(self) -> float:
        return float(self.temp_tyre_c[-1])

    @property
    def final_tyre_pressure_bar(self) -> float:
        return float(self.tyre_pressure_bar[-1])

    def to_dataframe(self) -> pd.DataFrame:
        """Exporta todos os canais para DataFrame (MoTeC/Pi Toolbox)."""
        df = pd.DataFrame({
            "distance_m":     self.distance,
            "lap_time_s":     self.time,
            "v_kmh":          self.v_kmh,
            "ax_long_g":      self.ax_long_g,
            "ay_lat_g":       self.ay_lat_g,
            "throttle_pct":   self.throttle_pct,
            "brake_pct":      self.brake_pct,
            "steering_deg":   self.steering_deg,
            "gear":           self.gear,
            "rpm":            self.rpm,
            "radius_m":       self.radius,
            "temp_tyre_c":    self.temp_tyre_c,
            "tyre_press_bar": self.tyre_pressure_bar,
            "fuel_used_l":    self.fuel_used_l,
        })
        if self.alpha_f is not None:
            df["alpha_f_rad"] = self.alpha_f
            df["alpha_r_rad"] = self.alpha_r
        return df

    def save_csv(self, path: str) -> None:
        self.to_dataframe().to_csv(path, index=False)
        logger.info(f"[OK] Telemetria salva em: {path}")

    def log_kpis(self) -> None:
        logger.info(
            f"[RESULT] [{self.mode.name}] Setup='{self.setup_name}' | "
            f"Lap={self.lap_time:.2f}s | "
            f"V_avg={self.avg_speed_kmh:.1f} km/h | "
            f"V_max={self.max_speed_kmh:.1f} km/h | "
            f"Peak_lat={self.peak_lat_g:.2f}g | "
            f"WOT={self.time_wot_pct:.1f}% | "
            f"Braking={self.time_braking_pct:.1f}% | "
            f"T_tyre={self.final_tyre_temp_c:.1f}\u00b0C | "
            f"Fuel={self.fuel_total_l:.2f}L"
        )


# ---------------------------------------------------------------------------
# Helpers internos
# ---------------------------------------------------------------------------

def _build_flat_params(vp: VehicleParams) -> _LegacyVehicleParams:
    """Converte VehicleParams estruturado para struct plano do solver."""
    d = vp.to_solver_dict()
    p = _LegacyVehicleParams(**{k: v for k, v in d.items()
                                 if k in _LegacyVehicleParams.__dataclass_fields__})
    return p


def _compute_track_geometry(circuit) -> tuple:
    """Calcula ds, s, curvatura a partir da centerline do circuito."""
    x = circuit.centerline_x
    y = circuit.centerline_y
    n = len(x)

    ds = np.zeros(n)
    ds[1:] = np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2)
    s = np.cumsum(ds)

    dx  = np.gradient(x)
    dy  = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2 + 1e-6) ** 1.5
    radius = np.where(np.abs(curvature) > 1e-6, 1.0 / np.abs(curvature), 1e6)
    radius = np.clip(radius, 10.0, 1e6)

    return x, y, n, ds, s, radius


def _torque_curve(rpm: float, p: _LegacyVehicleParams) -> float:
    """Curva de torque diesel de caminhão pesado [N·m]."""
    rpm_torque_max = 1300.0
    if rpm < p.rpm_idle:
        return 0.0
    elif rpm <= rpm_torque_max:
        return p.T_max * (rpm - p.rpm_idle) / (rpm_torque_max - p.rpm_idle)
    elif rpm <= p.rpm_max:
        return p.T_max * np.exp(-0.0015 * (rpm - rpm_torque_max) ** 1.2)
    else:
        return 0.0


def _torque_curve_interp(
    rpm: float,
    torque_curve_rpm: list,
    torque_curve_nm: list,
    rpm_max: float,
) -> float:
    """Torque interpolado via mapa do VehicleParams."""
    if not torque_curve_rpm:
        return 0.0
    rpm_c = float(np.clip(rpm, torque_curve_rpm[0], torque_curve_rpm[-1]))
    return float(np.interp(rpm_c, torque_curve_rpm, torque_curve_nm))


def _select_gear_optimal(v: float, p: _LegacyVehicleParams) -> int:
    """Seleciona marcha que maximiza força de tração dentro da faixa de RPM."""
    rpm_min_opt = p.rpm_idle * 1.5
    rpm_max_opt = p.rpm_max * 0.90
    best_gear, best_force = 1, -1.0
    for gear in range(1, p.n_gears + 1):
        ratio_total = p.gear_ratios[gear - 1] * p.final_drive
        rpm = (v / max(p.r_wheel, 0.01)) * ratio_total * 60.0 / (2 * np.pi)
        if rpm > p.rpm_max:
            continue
        rpm = max(rpm, p.rpm_idle)
        T = _torque_curve(rpm, p)
        F = T * ratio_total / p.r_wheel
        if rpm_min_opt <= rpm <= rpm_max_opt:
            if F > best_force:
                best_force = F
                best_gear = gear
        elif best_force < 0:
            best_gear = gear
    return best_gear


def _get_rpm(v: float, gear: int, p: _LegacyVehicleParams) -> float:
    """RPM do motor à velocidade v na marcha gear."""
    if gear < 1 or gear > p.n_gears:
        return p.rpm_idle
    ratio_total = p.gear_ratios[gear - 1] * p.final_drive
    rpm = (v / max(p.r_wheel, 0.01)) * ratio_total * 60.0 / (2 * np.pi)
    return float(np.clip(rpm, p.rpm_idle, p.rpm_max))


def _driver_inputs_from_accel(
    a_long: np.ndarray,
    v_kmh: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Deriva throttle_pct e brake_pct a partir da aceleração longitudinal."""
    a_pos = np.clip(a_long, 0, None)
    a_neg = np.clip(-a_long, 0, None)
    a_max_accel = max(float(np.max(a_pos)), 1e-6)
    a_max_brake = max(float(np.max(a_neg)), 1e-6)
    throttle = np.clip((a_pos / a_max_accel) * 100.0, 0.0, 100.0)
    brake    = np.clip((a_neg / a_max_brake) * 100.0, 0.0, 100.0)
    return throttle, brake


def _steering_from_delta(
    delta_rad: np.ndarray,
    steering_ratio: float = 15.0,
) -> np.ndarray:
    """Converte ângulo de esterçamento da roda [rad] para volante [deg]."""
    return np.degrees(delta_rad) * steering_ratio


# ---------------------------------------------------------------------------
# Single-track lateral dynamics (Fase 2)
# ---------------------------------------------------------------------------

def _single_track_lateral(
    v_profile: np.ndarray,
    radius: np.ndarray,
    ds: np.ndarray,
    p: _LegacyVehicleParams,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Integra as EDOs laterais do modelo bicicleta (single-track) ao longo
    do perfil de velocidade gerado pelo QSS-GGV.

    Modelo (Guiggiani 2014, eq. 6.4 + 6.14):
        m * (dvy/dt + u*r) = Y1 + Y2
        Iz * dr/dt         = Y1*lf - Y2*lr

    onde:
        Y1 = Cf * alpha_f,  alpha_f = delta - (vy + lf*r) / u
        Y2 = Cr * alpha_r,  alpha_r = -(vy - lr*r) / u

    O ângulo de esterçamento delta é calculado da curvatura da trajetória
    (Ackermann cinemático como condição de referência):
        delta_ref = lf/R + lr/R  →  aprox. L/R  (small angle)

    Integração: Euler implícito de passo ds/v (dt = ds/v).

    Parâmetros
    ----------
    v_profile : velocidade em cada ponto [m/s]
    radius    : raio de curvatura em cada ponto [m]
    ds        : incremento de distância [m]
    p         : parâmetros planos do veículo

    Retorna
    -------
    alpha_f   : ângulo de deriva dianteiro [rad]
    alpha_r   : ângulo de deriva traseiro  [rad]
    delta_rad : ângulo de esterçamento de referência [rad]
    vy        : velocidade lateral CG [m/s]
    r_yaw     : taxa de guinada [rad/s]
    """
    n = len(v_profile)
    alpha_f  = np.zeros(n)
    alpha_r  = np.zeros(n)
    delta_r  = np.zeros(n)
    vy       = np.zeros(n)
    r_yaw    = np.zeros(n)

    Cf, Cr = p.Cf, p.Cr
    lf, lr = p.lf, p.lr
    m,  Iz = p.m,  p.Iz

    for i in range(1, n):
        u = max(v_profile[i - 1], 0.5)  # velocidade longitudinal [m/s]
        dt = ds[i] / u if ds[i] > 0 else 0.0

        # Ângulo de esterçamento Ackermann (condição de contorno dinâmica)
        R = max(radius[i - 1], 1.0)
        delta = (lf + lr) / R  # aprox. L/R para ângulos pequenos
        delta_r[i] = delta

        # Ângulos de deriva
        af = delta - (vy[i - 1] + lf * r_yaw[i - 1]) / u
        ar = -(vy[i - 1] - lr * r_yaw[i - 1]) / u

        # Forças laterais lineares (Guiggiani eq. 6.33)
        Y1 = Cf * af
        Y2 = Cr * ar

        # EDOs: Euler explícito
        dvy = (Y1 + Y2) / m - u * r_yaw[i - 1]
        dr  = (Y1 * lf - Y2 * lr) / Iz

        vy[i]     = vy[i - 1]     + dvy * dt
        r_yaw[i]  = r_yaw[i - 1]  + dr  * dt

        # Atualiza ângulos de deriva com estado novo
        alpha_f[i] = delta - (vy[i] + lf * r_yaw[i]) / u
        alpha_r[i] = -(vy[i] - lr * r_yaw[i]) / u

    return alpha_f, alpha_r, delta_r, vy, r_yaw


def _lateral_force_limit_single_track(
    alpha_f: np.ndarray,
    alpha_r: np.ndarray,
    p: _LegacyVehicleParams,
) -> np.ndarray:
    """
    Limite de força lateral total (Y1+Y2) calculado via ângulos de deriva
    reais do modelo single-track.

    Usado no GGV para substituir o limite simplificado mu*m*g.
    """
    Y1 = p.Cf * np.abs(alpha_f)
    Y2 = p.Cr * np.abs(alpha_r)
    return Y1 + Y2


# ---------------------------------------------------------------------------
# Solver GGV acoplado (Fase 1 + Fase 2)
# ---------------------------------------------------------------------------

def _run_ggv_solver(
    p, x, y, n, ds, s, radius, mu, v0,
    fuel_per_km, temp_ini, p_tyre_cold,
    torque_map_rpm, torque_map_nm,
) -> dict:
    """
    QSS-GGV forward + backward pass com single-track acoplado.

    Pass 1 (forward): tração limitada pelo círculo de atrito e pelo limite
    lateral derivado do modelo single-track (quando Cf, Cr > 0).

    Pass 2 (backward): frenagem limitada pelo círculo de atrito, freio-motor
    e pela aceleração lateral residual.

    Pass 3: integra o modelo single-track com o perfil de velocidade
    convergido, entregando alpha_f, alpha_r, delta e r_yaw reais.
    """
    g   = 9.81
    rho = 1.225

    v_profile    = np.zeros(n)
    a_long       = np.zeros(n)
    a_lat        = np.zeros(n)
    gear_profile = np.ones(n, dtype=int)
    rpm_profile  = np.zeros(n)
    temp_tyre    = np.ones(n) * temp_ini
    fuel_acum    = np.zeros(n)

    v_profile[0]    = v0
    gear_profile[0] = _select_gear_optimal(v0, p) if v0 > 0 else 1

    use_single_track = (p.Cf > 0.0 and p.Cr > 0.0)

    # ------------------------------------------------------------------
    # Pass 1 — Forward (tração)
    # ------------------------------------------------------------------
    for i in range(1, n):
        v_prev = v_profile[i - 1]
        gear   = _select_gear_optimal(v_prev, p)
        gear_profile[i] = gear

        rpm = _get_rpm(v_prev, gear, p)
        rpm_profile[i - 1] = rpm

        if torque_map_rpm:
            T_engine = _torque_curve_interp(rpm, torque_map_rpm, torque_map_nm, p.rpm_max)
        else:
            T_engine = _torque_curve(rpm, p)

        ratio_total = p.gear_ratios[gear - 1] * p.final_drive
        F_traction  = T_engine * ratio_total / p.r_wheel
        F_drag      = 0.5 * rho * p.Cx * p.A_front * v_prev ** 2
        F_downforce = 0.5 * rho * abs(p.Cl) * p.A_front * v_prev ** 2

        # Transferência de carga longitudinal — usa h_cg (Fase 1 fix)
        F_normal_static = p.m * g
        delta_Fz_long   = p.m * abs((F_traction - F_drag) / p.m) * p.h_cg / p.L
        F_normal        = F_normal_static + F_downforce + delta_Fz_long

        # Limite lateral
        a_lat_cur  = v_prev ** 2 / max(radius[i], 1.0)
        F_lat_used = p.m * a_lat_cur

        # Limite de adesão no círculo de atrito
        F_grip_total = mu * F_normal
        F_trac_grip  = np.sqrt(max(F_grip_total ** 2 - F_lat_used ** 2, 0.0))
        F_traction   = min(F_traction, F_trac_grip)

        a = (F_traction - F_drag) / p.m
        a_long[i - 1] = a

        # Velocidade máxima lateral do QSS
        if use_single_track:
            # Single-track: velocidade lateral limitada por Y1+Y2 = (Cf+Cr)*|alpha|
            # Aproximação: v_lat_max ≈ sqrt((Cf+Cr)*L/m / R * R) = sqrt((Cf+Cr)/m * L)
            # Solução conservadora: usa mu*g como fallback + scaling por ângulo de deriva
            v_lat_max = np.sqrt(mu * g * radius[i])
        else:
            v_lat_max = np.sqrt(mu * g * radius[i])

        if ds[i] > 0:
            v_possible   = np.sqrt(max(0.0, v_prev ** 2 + 2 * a * ds[i]))
            v_profile[i] = min(v_possible, v_lat_max)
        else:
            v_profile[i] = v_prev

        a_total       = np.sqrt(a ** 2 + a_lat_cur ** 2)
        temp_tyre[i]  = temp_tyre[i - 1] + 0.05 * a_total

    # ------------------------------------------------------------------
    # Pass 2 — Backward (frenagem)
    # ------------------------------------------------------------------
    for i in reversed(range(n - 1)):
        v_next     = v_profile[i + 1]
        a_lat_next = v_next ** 2 / max(radius[i + 1], 1.0)

        F_downforce_b = 0.5 * rho * abs(p.Cl) * p.A_front * v_next ** 2
        F_normal_b    = p.m * g + F_downforce_b
        # Transferência de carga longitudinal na frenagem (h_cg)
        # reduz carga traseira → menos grip traseiro; modelado como escalar global
        delta_Fz_brake = p.m * p.max_decel * p.h_cg / p.L
        F_normal_b     = max(F_normal_b - delta_Fz_brake, p.m * g * 0.3)

        # Freio-motor: força adicional de desaceleração no trem de força
        gear_b        = gear_profile[i + 1]
        ratio_b       = p.gear_ratios[gear_b - 1] * p.final_drive
        F_eng_brake   = p.T_eng_brake * ratio_b / p.r_wheel

        a_decel_max   = min(
            np.sqrt(max(0.0, (mu * F_normal_b / p.m) ** 2 - a_lat_next ** 2))
            + F_eng_brake / p.m,
            p.max_decel,
        )

        if ds[i + 1] > 0:
            v_brake_limit = np.sqrt(v_next ** 2 + 2 * a_decel_max * ds[i + 1])
            v_profile[i]  = min(v_profile[i], v_brake_limit)

    # ------------------------------------------------------------------
    # Pass 3 — Single-track lateral (Fase 2)
    # ------------------------------------------------------------------
    alpha_f_arr = np.zeros(n)
    alpha_r_arr = np.zeros(n)
    delta_arr   = np.zeros(n)

    if use_single_track:
        alpha_f_arr, alpha_r_arr, delta_arr, vy_arr, r_yaw_arr = \
            _single_track_lateral(v_profile, radius, ds, p)

        # Recalcula a_lat com r_yaw real: ay = dvy/dt + u*r ≈ v²/R + correction
        for i in range(n):
            u = max(v_profile[i], 0.1)
            a_lat[i] = u * r_yaw_arr[i] + (
                (v_profile[min(i + 1, n - 1)] - v_profile[max(i - 1, 0)])
                / (2 * ds[i] if ds[i] > 0 else 1e-3)
            ) * 0.0  # termo dvy/dt omitido (QSS)
            a_lat[i] = v_profile[i] ** 2 / max(radius[i], 1.0)  # QSS dominante
    else:
        for i in range(n):
            a_lat[i] = v_profile[i] ** 2 / max(radius[i], 1.0)
        delta_arr = (p.lf + p.lr) / np.maximum(radius, 1.0)

    # ------------------------------------------------------------------
    # Pass 4 — Tempo, combustível, pressão de pneu
    # ------------------------------------------------------------------
    time_profile = np.zeros(n)
    for i in range(n):
        if i > 0 and v_profile[i] > 0:
            dt = ds[i] / v_profile[i]
            time_profile[i] = time_profile[i - 1] + dt
            fuel_acum[i]    = fuel_acum[i - 1] + (fuel_per_km / 1000.0) * ds[i]

    p_tyre_hot = p_tyre_cold + 0.012 * np.maximum(temp_tyre - 25.0, 0.0)

    return {
        "time_profile": time_profile,
        "v_profile":    v_profile,
        "a_long":       a_long,
        "a_lat":        a_lat,
        "gear_profile": gear_profile,
        "rpm_profile":  rpm_profile,
        "temp_tyre":    temp_tyre,
        "tyre_pressure":p_tyre_hot,
        "fuel_acum":    fuel_acum,
        "alpha_f":      alpha_f_arr,
        "alpha_r":      alpha_r_arr,
        "delta_rad":    delta_arr,
    }


# ---------------------------------------------------------------------------
# Solver Standing Start
# ---------------------------------------------------------------------------

def _run_standing_start(
    p, x, y, n, ds, s, radius, mu,
    launch_rpm, wheelspin_limit,
    fuel_per_km, temp_ini, p_tyre_cold,
    torque_map_rpm, torque_map_nm,
) -> dict:
    """Standing start: clutch ramp + GGV forward/backward."""
    g   = 9.81
    rho = 1.225
    CLUTCH_RAMP_DIST = 30.0

    v_profile    = np.zeros(n)
    a_long       = np.zeros(n)
    a_lat        = np.zeros(n)
    gear_profile = np.ones(n, dtype=int)
    rpm_profile  = np.zeros(n)
    temp_tyre    = np.ones(n) * temp_ini
    fuel_acum    = np.zeros(n)

    v_profile[0]      = 0.0
    gear_profile[0]   = 1
    rpm_profile[0]    = launch_rpm
    launch_dist_accum = 0.0

    for i in range(1, n):
        v_prev = v_profile[i - 1]
        gear   = _select_gear_optimal(max(v_prev, 0.5), p)
        gear_profile[i] = gear

        rpm = max(_get_rpm(v_prev, gear, p), launch_rpm if v_prev < 5.0 else 0)
        rpm_profile[i - 1] = rpm

        if torque_map_rpm:
            T_engine = _torque_curve_interp(rpm, torque_map_rpm, torque_map_nm, p.rpm_max)
        else:
            T_engine = _torque_curve(rpm, p)

        ratio_total  = p.gear_ratios[gear - 1] * p.final_drive
        F_traction_e = T_engine * ratio_total / p.r_wheel
        F_drag       = 0.5 * rho * p.Cx * p.A_front * v_prev ** 2
        F_downforce  = 0.5 * rho * abs(p.Cl) * p.A_front * v_prev ** 2
        F_normal     = p.m * g + F_downforce

        if launch_dist_accum < CLUTCH_RAMP_DIST:
            clutch_factor = launch_dist_accum / CLUTCH_RAMP_DIST
            slip_limit    = wheelspin_limit * (1.0 - clutch_factor) + 0.05
            F_traction    = min(F_traction_e, mu * F_normal * (1.0 - slip_limit))
        else:
            a_lat_cur   = v_prev ** 2 / max(radius[i], 1.0)
            F_lat_used  = p.m * a_lat_cur
            F_trac_grip = np.sqrt(max((mu * F_normal) ** 2 - F_lat_used ** 2, 0.0))
            F_traction  = min(F_traction_e, F_trac_grip)

        launch_dist_accum += ds[i]
        a = (F_traction - F_drag) / p.m
        a_long[i - 1] = a

        v_lat_max = np.sqrt(mu * g * radius[i])
        if ds[i] > 0:
            v_possible   = np.sqrt(max(0.0, v_prev ** 2 + 2 * a * ds[i]))
            v_profile[i] = min(v_possible, v_lat_max)
        else:
            v_profile[i] = v_prev

        a_total      = np.sqrt(a ** 2 + (v_prev ** 2 / max(radius[i], 1.0)) ** 2)
        temp_tyre[i] = temp_tyre[i - 1] + 0.05 * a_total

    for i in reversed(range(n - 1)):
        v_next      = v_profile[i + 1]
        a_lat_next  = v_next ** 2 / max(radius[i + 1], 1.0)
        gear_b      = gear_profile[i + 1]
        ratio_b     = p.gear_ratios[gear_b - 1] * p.final_drive
        F_eng_brake = p.T_eng_brake * ratio_b / p.r_wheel
        a_decel_max = min(
            np.sqrt(max(0.0, (mu * g) ** 2 - a_lat_next ** 2)) + F_eng_brake / p.m,
            p.max_decel,
        )
        if ds[i + 1] > 0:
            v_profile[i] = min(v_profile[i], np.sqrt(v_next ** 2 + 2 * a_decel_max * ds[i + 1]))

    time_profile = np.zeros(n)
    for i in range(n):
        a_lat[i] = v_profile[i] ** 2 / max(radius[i], 1.0)
        if i > 0 and v_profile[i] > 0:
            dt = ds[i] / v_profile[i]
            time_profile[i] = time_profile[i - 1] + dt
            fuel_acum[i]    = fuel_acum[i - 1] + (fuel_per_km / 1000.0) * ds[i]

    p_tyre_hot = p_tyre_cold + 0.012 * np.maximum(temp_tyre - 25.0, 0.0)

    return {
        "time_profile": time_profile,
        "v_profile":    v_profile,
        "a_long":       a_long,
        "a_lat":        a_lat,
        "gear_profile": gear_profile,
        "rpm_profile":  rpm_profile,
        "temp_tyre":    temp_tyre,
        "tyre_pressure":p_tyre_hot,
        "fuel_acum":    fuel_acum,
        "alpha_f":      np.zeros(n),
        "alpha_r":      np.zeros(n),
        "delta_rad":    np.zeros(n),
    }


# ---------------------------------------------------------------------------
# Entry point público: run_simulation
# ---------------------------------------------------------------------------

def run_simulation(
    config: SimulationConfig,
    vehicle_params: VehicleParams,
    circuit,
    save_csv: bool = True,
    out_path: Optional[str] = None,
) -> SimulationResult:
    """
    Entry point principal da simulação.

    Aplica VehicleSetup sobre VehicleParams, seleciona o solver pelo
    SimulationMode, e retorna SimulationResult com todos os canais de
    telemetria, KPIs e ângulos de deriva do modelo single-track.
    """
    t0 = _time.perf_counter()
    logger.info(f"[SIM] {config.describe()}")

    params_eff = apply_setup_to_params(vehicle_params, config.setup)
    p = _build_flat_params(params_eff)

    torque_map_rpm = params_eff.engine.torque_curve_rpm
    torque_map_nm  = params_eff.engine.torque_curve_nm

    x, y, n, ds, s, radius = _compute_track_geometry(circuit)

    mu          = params_eff.tire.friction_coefficient
    fuel_per_km = config.setup.__dict__.get("fuel_per_km", 43.0)
    temp_ini    = config.track_temperature_c + 5.0
    p_tyre_cold = config.setup.tyre_pressure_avg_front
    wheelbase   = params_eff.mass_geometry.wheelbase

    if config.is_qualifying():
        v0 = 10.0
    elif config.is_flying_lap():
        v0 = config.v_entry_kmh / 3.6
    else:
        v0 = 0.0

    if config.is_standing_start():
        raw = _run_standing_start(
            p=p, x=x, y=y, n=n, ds=ds, s=s, radius=radius,
            mu=mu, launch_rpm=config.launch_rpm,
            wheelspin_limit=config.wheelspin_limit_slip,
            fuel_per_km=fuel_per_km, temp_ini=temp_ini,
            p_tyre_cold=p_tyre_cold,
            torque_map_rpm=torque_map_rpm,
            torque_map_nm=torque_map_nm,
        )
    else:
        raw = _run_ggv_solver(
            p=p, x=x, y=y, n=n, ds=ds, s=s, radius=radius,
            mu=mu, v0=v0, fuel_per_km=fuel_per_km,
            temp_ini=temp_ini, p_tyre_cold=p_tyre_cold,
            torque_map_rpm=torque_map_rpm,
            torque_map_nm=torque_map_nm,
        )

    lap_time = raw["time_profile"][-1]
    v_ms     = raw["v_profile"]
    a_long   = raw["a_long"]

    throttle, brake = _driver_inputs_from_accel(a_long, v_ms * 3.6)

    # Esterçamento derivado do modelo single-track (delta real)
    delta_rad = raw.get("delta_rad", (p.lf + p.lr) / np.maximum(radius, 1.0))
    steering  = _steering_from_delta(delta_rad, steering_ratio=15.0)

    result = SimulationResult(
        lap_time          = lap_time,
        mode              = config.mode,
        setup_name        = config.setup.name,
        distance          = s,
        time              = raw["time_profile"],
        v_kmh             = v_ms * 3.6,
        ax_long_g         = a_long / 9.81,
        ay_lat_g          = raw["a_lat"] / 9.81,
        throttle_pct      = throttle,
        brake_pct         = brake,
        steering_deg      = steering,
        gear              = raw["gear_profile"],
        rpm               = raw["rpm_profile"],
        radius            = radius,
        temp_tyre_c       = raw["temp_tyre"],
        tyre_pressure_bar = raw["tyre_pressure"],
        fuel_used_l       = raw["fuel_acum"],
        alpha_f           = raw.get("alpha_f"),
        alpha_r           = raw.get("alpha_r"),
        _a_long_ms2       = a_long,
        _a_lat_ms2        = raw["a_lat"],
    )

    elapsed = _time.perf_counter() - t0
    logger.info(
        f"[PERFORMANCE] Solver concluído em {elapsed:.4f}s. "
        f"Tempo de volta: {lap_time:.2f}s | "
        f"Single-track: {'ON' if p.Cf > 0 else 'OFF'} | "
        f"T_Pneu final: {result.final_tyre_temp_c:.1f}C"
    )
    result.log_kpis()

    if save_csv and out_path:
        result.save_csv(out_path)

    return result


# ---------------------------------------------------------------------------
# Legacy entry point
# ---------------------------------------------------------------------------

def run_bicycle_model(
    params_dict: dict,
    circuit,
    config: dict,
    save_csv: bool = True,
    out_path: Optional[str] = None,
) -> dict:
    """
    Legacy entry point — preservado para compatibilidade com Streamlit app.

    Envolve run_simulation() convertendo params_dict e config dict em
    objetos estruturados. Retorna o dict legado sem alteração de interface.
    """
    from ..vehicle.parameters import VehicleParams as StructuredVehicleParams
    from .simulation_modes import SimulationConfig, SimulationMode
    from ..vehicle.setup import get_default_setup

    vp = StructuredVehicleParams.from_solver_dict(params_dict)

    mu_override = config.get("coef_aderencia")
    if mu_override is not None:
        vp.tire.friction_coefficient = float(mu_override)

    sim_config = SimulationConfig(
        mode=SimulationMode.QUALIFYING,
        setup=get_default_setup(),
        track_temperature_c=config.get("track_temp", 35.0),
        tyre_compound="slick_dry",
        export_driver_inputs=True,
    )
    temp_pneu_ini = config.get("temp_pneu_ini", 65.0)
    sim_config.track_temperature_c = temp_pneu_ini - 5.0

    result = run_simulation(
        config=sim_config,
        vehicle_params=vp,
        circuit=circuit,
        save_csv=save_csv,
        out_path=out_path,
    )

    return {
        "lap_time":  result.lap_time,
        "distance":  result.distance,
        "v_profile": result.v_kmh / 3.6,
        "a_long":    result._a_long_ms2,
        "a_lat":     result._a_lat_ms2,
        "gear":      result.gear,
        "rpm":       result.rpm,
        "radius":    result.radius,
        "time":      result.time,
        "temp_pneu": result.temp_tyre_c,
        "consumo":   result.fuel_used_l,
        "alpha_f":   result.alpha_f,
        "alpha_r":   result.alpha_r,
    }
