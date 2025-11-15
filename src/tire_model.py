# src/tire_model.py
"""
Modelo de pneu com dinâmica térmica e pressão
Baseado em: Magic Formula (Pacejka simplificado) + térmica
"""
import numpy as np

class TireModel:
    """Modelo de pneu linear com dinâmica térmica"""
    
    def __init__(self, Cf, mu, p_ref=220.0, T_ref=70.0):
        """
        Cf: rigidez lateral nominal [N/rad]
        mu: coeficiente de atrito
        p_ref: pressão de referência [kPa]
        T_ref: temperatura de referência [°C]
        """
        self.Cf_ref = Cf
        self.mu_ref = mu
        self.p_ref = p_ref
        self.T_ref = T_ref
        
        # Parâmetros térmicos (Rally/Competição)
        self.c_thermal = 1500.0  # capacidade térmica [J/K]
        self.h_conv = 50.0       # coef. convecção [W/m²K]
        self.A_tire = 0.15       # área efetiva [m²]
        
    def slip_angle(self, v_lat, v_long):
        """Calcula ângulo de escorregamento [rad]"""
        if v_long < 0.1:
            return 0.0
        return np.arctan2(v_lat, v_long)
    
    def lateral_force_linear(self, alpha, Fz, T_tire=70.0, p_tire=220.0):
        """
        Força lateral usando modelo linear com variação térmica/pressão
        
        Fy = Cf * α * (p/p_ref) * (T_ref/T) [aproximação simplificada]
        """
        # Efeito de pressão (tipicamente +3% força por 10 kPa)
        k_p = 1.0 + 0.003 * (p_tire - self.p_ref)
        
        # Efeito de temperatura (tipicamente -0.5% por °C acima de referência)
        k_t = 1.0 - 0.005 * (T_tire - self.T_ref)
        
        # Força lateral
        Fy = self.Cf_ref * alpha * k_p * k_t
        
        # Limita por aderência
        Fy_max = self.mu_ref * Fz
        return np.clip(Fy, -Fy_max, Fy_max)
    
    def thermal_dynamics(self, alpha, v_long, T_ambient, T_tire, dt=0.01):
        """
        Dinâmica térmica do pneu
        dT/dt = (Q_friction - Q_convection) / c_thermal
        """
        # Calor por fricção (proporcional ao trabalho dissipado)
        slip_velocity = v_long * np.abs(alpha)  # velocidade de escorregamento simplificada
        Q_friction = 0.8 * self.mu_ref * slip_velocity * 1000.0  # [W]
        
        # Calor perdido por convecção
        Q_conv = self.h_conv * self.A_tire * (T_tire - T_ambient)  # [W]
        
        # Integra temperatura
        dT_dt = (Q_friction - Q_conv) / self.c_thermal
        T_new = T_tire + dT_dt * dt
        
        return np.clip(T_new, T_ambient, 110.0)  # limita entre ambiente e máximo
    
    def pressure_dynamics(self, T_tire, p_tire, p_ref_temp=70.0):
        """
        Dinâmica de pressão (Gay-Lussac simplificado)
        p/T = const -> p_new = p_ref * (T_new / T_ref)
        """
        p_new = p_tire * (T_tire + 273.15) / (p_ref_temp + 273.15)
        return np.clip(p_new, 180.0, 260.0)  # limita range prático
