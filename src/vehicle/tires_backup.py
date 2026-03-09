"""
Módulo de modelos de Pneus.
Contém modelo Linear (validação rápida) e Pacejka Magic Formula.
"""
from abc import ABC, abstractmethod
import numpy as np

class BaseTire(ABC):
    """Interface abstrata para modelo de pneus"""
    
    def __init__(self, config: dict):
        self.config = config
        self.mu_x = config.get('mu_x', 0.9)
        self.mu_y = config.get('mu_y', 0.9)
        
    @abstractmethod
    def get_lateral_force(self, slip_angle: float, normal_load: float) -> float:
        """Calcula força lateral (Fy) com base no slip angle (rad) e carga (N)"""
        pass
        
    @abstractmethod
    def get_longitudinal_force(self, slip_ratio: float, normal_load: float) -> float:
        """Calcula força longitudinal (Fx) com base no slip ratio e carga (N)"""
        pass


class LinearTire(BaseTire):
    """Modelo Linear - Usado para validação do modelo 2-DOF em baixas acelerações"""
    
    def __init__(self, config: dict):
        super().__init__(config)
        self.cornering_stiffness = config.get('cornering_stiffness', 80000) # N/rad
        self.longitudinal_stiffness = config.get('longitudinal_stiffness', 100000) # N/slip_ratio
        
    def get_lateral_force(self, slip_angle: float, normal_load: float) -> float:
        fy = -self.cornering_stiffness * slip_angle
        # Saturação brusca baseada no atrito
        max_fy = self.mu_y * normal_load
        return np.clip(fy, -max_fy, max_fy)
        
    def get_longitudinal_force(self, slip_ratio: float, normal_load: float) -> float:
        fx = self.longitudinal_stiffness * slip_ratio
        max_fx = self.mu_x * normal_load
        return np.clip(fx, -max_fx, max_fx)


class PacejkaTire(BaseTire):
    """Modelo Magic Formula de Pacejka (Simplificado)"""
    
    def __init__(self, config: dict):
        super().__init__(config)
        # Pacejka coefs para Força Lateral
        self.B_y = config.get('pacejka_b_y', 10.0) # Stiffness factor
        self.C_y = config.get('pacejka_c_y', 1.3)  # Shape factor
        self.D_y = config.get('pacejka_d_y', 1.0)  # Peak factor (multiplicador do mi)
        self.E_y = config.get('pacejka_e_y', 0.0)  # Curvature factor
        
    def get_lateral_force(self, slip_angle: float, normal_load: float) -> float:
        """
        F_y = D * sin(C * arctan(B * alpha - E * (B * alpha - arctan(B * alpha))))
        """
        alpha_deg = np.degrees(slip_angle) # Geralmente formula pacejka recebe deg dependendo dos coefs
        D = self.mu_y * normal_load * self.D_y
        
        Bx = self.B_y * alpha_deg
        E_term = self.E_y * (Bx - np.arctan(Bx))
        
        fy = D * np.sin(self.C_y * np.arctan(Bx - E_term))
        return -fy # Convenção de sinal (slip positivo gera força negativa)
        
    def get_longitudinal_force(self, slip_ratio: float, normal_load: float) -> float:
        # Implementação futura se necessário slip dinâmico longitudinal real
        pass
