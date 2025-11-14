"""
Módulo de Interface Streamlit

Este módulo implementa a interface gráfica do usuário usando Streamlit
para interação com o simulador de tempo de volta.
"""

import streamlit as st
import plotly.graph_objects as go
import pandas as pd


def criar_interface():
    """
    Cria a interface principal do aplicativo Streamlit.
    """
    st.set_page_config(
        page_title="LapTime Simulator - Copa Truck",
        page_icon="🏁",
        layout="wide"
    )
    
    st.title("🏁 Simulador de Tempo de Volta - Copa Truck")
    st.markdown("""
    Bem-vindo ao simulador de tempo de volta para competições de Copa Truck.
    Configure os parâmetros do veículo e da pista para analisar o desempenho.
    """)
    
    # Sidebar para parâmetros
    st.sidebar.header("Parâmetros de Simulação")
    
    # Parâmetros do veículo
    st.sidebar.subheader("Veículo")
    massa = st.sidebar.slider("Massa (kg)", 4000, 6000, 5000, 100)
    potencia = st.sidebar.slider("Potência (kW)", 500, 1000, 800, 50)
    coef_arrasto = st.sidebar.slider("Coeficiente de Arrasto", 0.5, 1.2, 0.8, 0.1)
    
    # Parâmetros da pista
    st.sidebar.subheader("Pista")
    comprimento = st.sidebar.slider("Comprimento (m)", 2000, 5000, 3000, 100)
    num_curvas = st.sidebar.slider("Número de Curvas", 5, 20, 10, 1)
    
    # Botão de simulação
    if st.sidebar.button("🚀 Executar Simulação", type="primary"):
        simular_e_exibir_resultados(massa, potencia, coef_arrasto, comprimento, num_curvas)
    
    # Informações iniciais
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.metric("Massa do Veículo", f"{massa} kg")
    
    with col2:
        st.metric("Potência", f"{potencia} kW")
    
    with col3:
        st.metric("Comprimento da Pista", f"{comprimento} m")


def simular_e_exibir_resultados(massa, potencia, coef_arrasto, comprimento, num_curvas):
    """
    Executa a simulação e exibe os resultados.
    
    Args:
        massa: Massa do veículo em kg
        potencia: Potência do motor em kW
        coef_arrasto: Coeficiente de arrasto
        comprimento: Comprimento da pista em metros
        num_curvas: Número de curvas no circuito
    """
    with st.spinner("Executando simulação..."):
        # Placeholder para resultados da simulação
        import time
        time.sleep(1)  # Simular processamento
        
        st.success("✅ Simulação concluída!")
        
        # Exibir resultados
        st.header("Resultados da Simulação")
        
        col1, col2, col3 = st.columns(3)
        
        with col1:
            st.metric("Tempo de Volta", "1:35.420", delta="-0.350s")
        
        with col2:
            st.metric("Velocidade Média", "165.4 km/h", delta="+3.2 km/h")
        
        with col3:
            st.metric("Velocidade Máxima", "198.5 km/h")
        
        # Gráfico de velocidade vs tempo (placeholder)
        st.subheader("Velocidade ao Longo da Volta")
        
        # Dados de exemplo
        import numpy as np
        tempo = np.linspace(0, 95.42, 100)
        velocidade = 120 + 30 * np.sin(tempo / 10) + np.random.normal(0, 2, 100)
        
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=tempo,
            y=velocidade,
            mode='lines',
            name='Velocidade',
            line=dict(color='#1f77b4', width=2)
        ))
        
        fig.update_layout(
            xaxis_title="Tempo (s)",
            yaxis_title="Velocidade (km/h)",
            hovermode='x unified',
            height=400
        )
        
        st.plotly_chart(fig, use_container_width=True)
        
        # Tabela de setores
        st.subheader("Análise por Setores")
        df_setores = pd.DataFrame({
            'Setor': ['Setor 1', 'Setor 2', 'Setor 3'],
            'Tempo (s)': [32.145, 31.890, 31.385],
            'Velocidade Média (km/h)': [162.3, 168.1, 165.8],
            'Velocidade Máxima (km/h)': [195.2, 198.5, 192.3]
        })
        
        st.dataframe(df_setores, use_container_width=True)


def main():
    """Função principal do aplicativo."""
    criar_interface()


if __name__ == "__main__":
    main()
