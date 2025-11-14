import streamlit as st
import pandas as pd
import h5py
import plotly.express as px
import os
import time

DATA_PATH = r"C:\Users\vitor\OneDrive\Desktop\Pastas\LapTimeSimulator_CopaTruck\data"


def log_time(section):
    def decorator(func):
        def wrapper(*args, **kwargs):
            start = time.time()
            result = func(*args, **kwargs)
            st.info(f"Seção '{section}' executada em {time.time() - start:.2f}s")
            return result
        return wrapper
    return decorator

@log_time("Pista: Leitura HDF5 e Gráfico")
def pista_page():
    st.header("Pista")
    pistas = [f for f in os.listdir(DATA_PATH) if f.endswith('.hdf5')]
    pista_selecionada = st.selectbox("Selecione uma pista", pistas)
    if pista_selecionada:
        caminho_pista = os.path.join(DATA_PATH, pista_selecionada)
        with h5py.File(caminho_pista, 'r') as f:
            data = f['track_points'][:]
            track_df = pd.DataFrame(data, columns=['x', 'y'])
        # Ajusta para que o primeiro ponto seja (0,0):
        offset_x, offset_y = track_df.loc[0, 'x'], track_df.loc[0, 'y']
        track_df['x'] -= offset_x
        track_df['y'] -= offset_y
        st.write("### Mapa da Pista (origem em 0,0)")
        fig = px.line(track_df, x='x', y='y', title=f'Mapa: {pista_selecionada}')
        fig.update_layout(xaxis_title="x (m)", yaxis_title="y (m)", width=600, height=400)
        st.plotly_chart(fig, use_container_width=True)
        st.dataframe(track_df.head())

@log_time("Parâmetros do Veículo")
def parametros_veiculo_page():
    st.header("Parâmetros do Veículo")
    massa = st.number_input("Massa do Veículo (kg)", 1000.0, 15000.0, 5000.0)
    potencia = st.number_input("Potência (kW)", 100.0, 2000.0, 500.0)
    entre_eixos = st.number_input("Entre-eixos (m)", 2.5, 7.0, 4.2)
    largura = st.number_input("Largura (m)", 2.0, 4.0, 2.5)
    # ... outros parâmetros
    st.write(f"Massa: {massa} kg, Potência: {potencia} kW, Entre-eixos: {entre_eixos} m, Largura: {largura} m")

@log_time("Condições de Simulação")
def condicoes_simulacao_page():
    st.header("Condições de Simulação")
    clima = st.selectbox("Clima", ["Seco", "Molhado", "Outro"])
    coef_aderencia = st.slider("Coeficiente de Aderência", 0.5, 2.5, 1.2)
    tempo_sim = st.number_input("Tempo Simulação (segundos)", 10, 3600, 120)
    objetivo = st.selectbox("Objetivo", ["Tempo mínimo", "Menor consumo", "Outro"])
    st.write(f"Clima: {clima}, μ={coef_aderencia}, Tempo: {tempo_sim}s, Objetivo: {objetivo}")

@log_time("Resultados")
def resultados_page():
    st.header("Resultados")
    st.write("Resultados, gráficos e KPIs aparecerão aqui após a simulação.")
    # Exemplo de tabela de KPIs (vazio por enquanto)
    # data = pd.DataFrame({"KPI": [], "Valor": []})
    # st.table(data)

PAGES = {
    "Parâmetros do Veículo": parametros_veiculo_page,
    "Pista": pista_page,
    "Condições de Simulação": condicoes_simulacao_page,
    "Resultados": resultados_page
}

st.set_page_config(page_title="LapTimeSimulator", layout="wide")
st.sidebar.title("LapTimeSimulator")
page = st.sidebar.radio("Escolha a página:", list(PAGES.keys()))
st.sidebar.markdown("<hr>", unsafe_allow_html=True)

# Chama a função correta para a página escolhida
display = PAGES[page]
display()
