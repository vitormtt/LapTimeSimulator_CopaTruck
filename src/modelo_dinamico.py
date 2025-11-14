"""
Módulo de Modelo Dinâmico

Este módulo contém as equações dinâmicas do veículo para simulação de tempo de volta.
Inclui modelos de forças, aceleração, velocidade e dinâmica do veículo.
"""

import numpy as np


class ModeloDinamicoVeiculo:
    """
    Classe que representa o modelo dinâmico de um veículo de corrida.
    
    Attributes:
        massa (float): Massa total do veículo em kg
        potencia_maxima (float): Potência máxima do motor em kW
        area_frontal (float): Área frontal do veículo em m²
        coef_arrasto (float): Coeficiente de arrasto aerodinâmico
    """
    
    def __init__(self, massa=5000, potencia_maxima=800, area_frontal=8.0, coef_arrasto=0.8):
        """
        Inicializa o modelo dinâmico do veículo.
        
        Args:
            massa: Massa do veículo em kg (padrão: 5000)
            potencia_maxima: Potência máxima em kW (padrão: 800)
            area_frontal: Área frontal em m² (padrão: 8.0)
            coef_arrasto: Coeficiente de arrasto (padrão: 0.8)
        """
        self.massa = massa
        self.potencia_maxima = potencia_maxima
        self.area_frontal = area_frontal
        self.coef_arrasto = coef_arrasto
        self.densidade_ar = 1.225  # kg/m³
        
    def calcular_forca_arrasto(self, velocidade):
        """
        Calcula a força de arrasto aerodinâmico.
        
        Args:
            velocidade: Velocidade do veículo em m/s
            
        Returns:
            float: Força de arrasto em N
        """
        return 0.5 * self.densidade_ar * self.area_frontal * self.coef_arrasto * velocidade**2
    
    def calcular_aceleracao(self, velocidade, forca_motor):
        """
        Calcula a aceleração do veículo.
        
        Args:
            velocidade: Velocidade atual em m/s
            forca_motor: Força aplicada pelo motor em N
            
        Returns:
            float: Aceleração em m/s²
        """
        forca_arrasto = self.calcular_forca_arrasto(velocidade)
        forca_resultante = forca_motor - forca_arrasto
        return forca_resultante / self.massa
