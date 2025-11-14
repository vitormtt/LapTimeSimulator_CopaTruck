"""
Módulo de Utilidades

Este módulo contém funções auxiliares e utilitárias
usadas em todo o projeto.
"""

import numpy as np
from typing import List, Tuple


def kmh_para_ms(velocidade_kmh: float) -> float:
    """
    Converte velocidade de km/h para m/s.
    
    Args:
        velocidade_kmh: Velocidade em km/h
        
    Returns:
        float: Velocidade em m/s
    """
    return velocidade_kmh / 3.6


def ms_para_kmh(velocidade_ms: float) -> float:
    """
    Converte velocidade de m/s para km/h.
    
    Args:
        velocidade_ms: Velocidade em m/s
        
    Returns:
        float: Velocidade em km/h
    """
    return velocidade_ms * 3.6


def calcular_tempo_formatado(tempo_segundos: float) -> str:
    """
    Formata tempo em segundos para formato MM:SS.mmm.
    
    Args:
        tempo_segundos: Tempo em segundos
        
    Returns:
        str: Tempo formatado (ex: "1:35.420")
    """
    minutos = int(tempo_segundos // 60)
    segundos = tempo_segundos % 60
    return f"{minutos}:{segundos:06.3f}"


def interpolar_pontos(pontos: List[Tuple[float, float]], num_pontos: int) -> np.ndarray:
    """
    Interpola pontos para criar uma curva suave.
    
    Args:
        pontos: Lista de tuplas (x, y) representando pontos
        num_pontos: Número de pontos interpolados desejados
        
    Returns:
        np.ndarray: Array com pontos interpolados
    """
    if len(pontos) < 2:
        return np.array(pontos)
    
    pontos_array = np.array(pontos)
    t = np.linspace(0, 1, len(pontos))
    t_novo = np.linspace(0, 1, num_pontos)
    
    x_interp = np.interp(t_novo, t, pontos_array[:, 0])
    y_interp = np.interp(t_novo, t, pontos_array[:, 1])
    
    return np.column_stack([x_interp, y_interp])


def calcular_distancia_euclidiana(ponto1: Tuple[float, float], ponto2: Tuple[float, float]) -> float:
    """
    Calcula a distância euclidiana entre dois pontos.
    
    Args:
        ponto1: Tupla (x, y) do primeiro ponto
        ponto2: Tupla (x, y) do segundo ponto
        
    Returns:
        float: Distância entre os pontos
    """
    return np.sqrt((ponto2[0] - ponto1[0])**2 + (ponto2[1] - ponto1[1])**2)


def validar_parametros_veiculo(massa: float, potencia: float, area_frontal: float) -> bool:
    """
    Valida se os parâmetros do veículo estão dentro de limites aceitáveis.
    
    Args:
        massa: Massa do veículo em kg
        potencia: Potência em kW
        area_frontal: Área frontal em m²
        
    Returns:
        bool: True se os parâmetros são válidos
    """
    if massa <= 0 or massa > 10000:
        return False
    if potencia <= 0 or potencia > 2000:
        return False
    if area_frontal <= 0 or area_frontal > 20:
        return False
    return True


def gerar_relatorio_texto(resultados: dict) -> str:
    """
    Gera um relatório em texto dos resultados da simulação.
    
    Args:
        resultados: Dicionário com resultados da simulação
        
    Returns:
        str: Relatório formatado em texto
    """
    relatorio = "=" * 50 + "\n"
    relatorio += "RELATÓRIO DE SIMULAÇÃO - COPA TRUCK\n"
    relatorio += "=" * 50 + "\n\n"
    
    if 'tempo_volta' in resultados:
        relatorio += f"Tempo de Volta: {calcular_tempo_formatado(resultados['tempo_volta'])}\n"
    
    if 'velocidade_media' in resultados:
        relatorio += f"Velocidade Média: {ms_para_kmh(resultados['velocidade_media']):.2f} km/h\n"
    
    if 'velocidade_maxima' in resultados:
        relatorio += f"Velocidade Máxima: {ms_para_kmh(resultados['velocidade_maxima']):.2f} km/h\n"
    
    relatorio += "\n" + "=" * 50 + "\n"
    
    return relatorio
