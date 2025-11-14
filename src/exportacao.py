"""
Módulo de Exportação

Este módulo fornece funcionalidades para exportar resultados
da simulação em diferentes formatos (CSV, JSON, Excel).
"""

import pandas as pd
import json
from pathlib import Path
from typing import Dict, Any


class ExportadorResultados:
    """
    Classe responsável por exportar resultados da simulação.
    """
    
    def __init__(self, diretorio_saida: str = "../results"):
        """
        Inicializa o exportador.
        
        Args:
            diretorio_saida: Diretório onde os arquivos serão salvos
        """
        self.diretorio_saida = Path(diretorio_saida)
        self.diretorio_saida.mkdir(parents=True, exist_ok=True)
        
    def exportar_csv(self, dados: Dict[str, Any], nome_arquivo: str) -> str:
        """
        Exporta dados para formato CSV.
        
        Args:
            dados: Dicionário com dados a serem exportados
            nome_arquivo: Nome do arquivo (sem extensão)
            
        Returns:
            str: Caminho do arquivo criado
        """
        df = pd.DataFrame(dados)
        caminho = self.diretorio_saida / f"{nome_arquivo}.csv"
        df.to_csv(caminho, index=False)
        return str(caminho)
    
    def exportar_json(self, dados: Dict[str, Any], nome_arquivo: str) -> str:
        """
        Exporta dados para formato JSON.
        
        Args:
            dados: Dicionário com dados a serem exportados
            nome_arquivo: Nome do arquivo (sem extensão)
            
        Returns:
            str: Caminho do arquivo criado
        """
        caminho = self.diretorio_saida / f"{nome_arquivo}.json"
        with open(caminho, 'w', encoding='utf-8') as f:
            json.dump(dados, f, indent=4, ensure_ascii=False)
        return str(caminho)
    
    def exportar_excel(self, dados: Dict[str, Any], nome_arquivo: str) -> str:
        """
        Exporta dados para formato Excel.
        
        Args:
            dados: Dicionário com dados a serem exportados
            nome_arquivo: Nome do arquivo (sem extensão)
            
        Returns:
            str: Caminho do arquivo criado
        """
        df = pd.DataFrame(dados)
        caminho = self.diretorio_saida / f"{nome_arquivo}.xlsx"
        df.to_excel(caminho, index=False, engine='openpyxl')
        return str(caminho)
    
    def exportar_relatorio_completo(self, resultados: Dict[str, Any], nome_base: str) -> Dict[str, str]:
        """
        Exporta um relatório completo em múltiplos formatos.
        
        Args:
            resultados: Dicionário com resultados da simulação
            nome_base: Nome base para os arquivos
            
        Returns:
            Dict[str, str]: Dicionário com caminhos dos arquivos criados
        """
        arquivos_criados = {}
        
        # Exportar histórico em CSV
        if 'historico' in resultados:
            arquivos_criados['csv'] = self.exportar_csv(
                resultados['historico'], 
                f"{nome_base}_historico"
            )
        
        # Exportar resumo em JSON
        resumo = {k: v for k, v in resultados.items() if k != 'historico'}
        arquivos_criados['json'] = self.exportar_json(
            resumo,
            f"{nome_base}_resumo"
        )
        
        return arquivos_criados
