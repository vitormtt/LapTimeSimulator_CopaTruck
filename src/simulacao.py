"""
Módulo de Simulação

Este módulo gerencia a execução da simulação de tempo de volta,
integrando o modelo dinâmico e o sistema de controle.
"""

import numpy as np
from typing import Dict, List, Tuple


class SimuladorTempoVolta:
    """
    Classe principal para simulação de tempo de volta.
    
    Coordena o modelo dinâmico e o controlador para simular
    uma volta completa no circuito.
    """
    
    def __init__(self, modelo_dinamico=None, controlador=None):
        """
        Inicializa o simulador.
        
        Args:
            modelo_dinamico: Instância do modelo dinâmico do veículo
            controlador: Instância do controlador do veículo
        """
        self.modelo_dinamico = modelo_dinamico
        self.controlador = controlador
        self.tempo_simulacao = 0
        self.historico = {
            'tempo': [],
            'velocidade': [],
            'posicao': [],
            'aceleracao': []
        }
        
    def resetar(self):
        """Reseta o estado da simulação."""
        self.tempo_simulacao = 0
        self.historico = {
            'tempo': [],
            'velocidade': [],
            'posicao': [],
            'aceleracao': []
        }
        
    def executar_passo(self, dt: float, velocidade_atual: float, posicao_atual: float) -> Tuple[float, float]:
        """
        Executa um passo da simulação.
        
        Args:
            dt: Intervalo de tempo em segundos
            velocidade_atual: Velocidade atual em m/s
            posicao_atual: Posição atual em metros
            
        Returns:
            Tuple[float, float]: Nova velocidade e posição
        """
        # Atualizar histórico
        self.historico['tempo'].append(self.tempo_simulacao)
        self.historico['velocidade'].append(velocidade_atual)
        self.historico['posicao'].append(posicao_atual)
        
        # Calcular aceleração (simplificado)
        aceleracao = 2.0  # m/s² (valor placeholder)
        self.historico['aceleracao'].append(aceleracao)
        
        # Atualizar estado
        nova_velocidade = velocidade_atual + aceleracao * dt
        nova_posicao = posicao_atual + velocidade_atual * dt + 0.5 * aceleracao * dt**2
        
        self.tempo_simulacao += dt
        
        return nova_velocidade, nova_posicao
    
    def simular_volta_completa(self, comprimento_pista: float = 3000, dt: float = 0.1) -> Dict:
        """
        Simula uma volta completa no circuito.
        
        Args:
            comprimento_pista: Comprimento da pista em metros (padrão: 3000)
            dt: Intervalo de tempo da simulação em segundos (padrão: 0.1)
            
        Returns:
            Dict: Dicionário com resultados da simulação
        """
        self.resetar()
        
        velocidade = 0
        posicao = 0
        
        while posicao < comprimento_pista:
            velocidade, posicao = self.executar_passo(dt, velocidade, posicao)
            
            # Limitar velocidade máxima (simulação simplificada)
            if velocidade > 55.56:  # ~200 km/h
                velocidade = 55.56
        
        return {
            'tempo_volta': self.tempo_simulacao,
            'historico': self.historico,
            'velocidade_media': np.mean(self.historico['velocidade']),
            'velocidade_maxima': np.max(self.historico['velocidade'])
        }
