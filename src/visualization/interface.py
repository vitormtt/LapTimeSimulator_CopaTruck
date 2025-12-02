# ============ CRITICAL: Must execute BEFORE any custom imports ============
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))
# ===========================================================================

# fmt: off
# isort: skip_file

# Standard library imports
import os
import json
from datetime import datetime

# Third-party imports
import streamlit as st
import numpy as np
import pandas as pd
import plotly.graph_objects as go

# Custom module imports (ONLY after sys.path is modified above)
from tracks.hdf5 import CircuitHDF5Reader
from simulation.lap_time_solver import run_bicycle_model
from vehicle.parameters import VehicleParams, copa_truck_2dof_default

# fmt: on

# ===== PATHS =====

DATA_PATH = r"C:\Users\vitor\OneDrive\Desktop\Pastas\LapTimeSimulator_CopaTruck\tracks"
RESULTS_PATH = r"C:\Users\vitor\OneDrive\Desktop\Pastas\LapTimeSimulator_CopaTruck\src\results"
MODELS_PATH = "data/vehicle_models.json"
os.makedirs(RESULTS_PATH, exist_ok=True)


def init_session_state():
    """Initialize session_state with VehicleParams dataclass"""
    if "vehicle_params" not in st.session_state:
        st.session_state.vehicle_params = copa_truck_2dof_default()
    if "config" not in st.session_state:
        st.session_state.config = {
            "tipo": "Qualificatória",
            "coef_aderencia": 1.09,
            "consumo": 43.0,
            "temp_pneu_ini": 65.0
        }
    if "circuit" not in st.session_state:
        st.session_state.circuit = None
    if "circuit_meta" not in st.session_state:
        st.session_state.circuit_meta = None
    if "resultados_prontos" not in st.session_state:
        st.session_state.resultados_prontos = False
    if "resultados" not in st.session_state:
        st.session_state.resultados = None
    if "last_csv_path" not in st.session_state:
        st.session_state.last_csv_path = None


def parametros_veiculo_page():
    """Vehicle parameters page with VehicleParams integration"""
    st.header("Vehicle Parameters - Copa Truck (2DOF)")

    vp = st.session_state.vehicle_params

    # Display current model info
    col1, col2, col3 = st.columns(3)
    with col1:
        st.metric("Manufacturer", vp.manufacturer)
    with col2:
        st.metric("Year", vp.year)
    with col3:
        st.metric("Power", f"{vp.engine.max_power/1000:.0f} kW")

    st.markdown("---")

    # Customization toggle
    aba = st.radio("Customize?", ["No", "Yes"],
                   horizontal=True, key="custom_radio")

    if aba == "Yes":
        section = st.radio("Section:", [
            "Mass/Geometry", "Tire", "Engine", "Transmission", "Brake", "Aerodynamics"
        ], horizontal=True)

        if section == "Mass/Geometry":
            vp.mass_geometry.mass = st.number_input(
                "Mass (kg)", 3500.0, 9000.0, vp.mass_geometry.mass
            )
            wheelbase = st.number_input(
                "Wheelbase (m)", 3.8, 5.5, vp.mass_geometry.wheelbase
            )
            vp.mass_geometry.lf = st.number_input(
                "Dist. CG → front (m)", 1.0, 3.0, vp.mass_geometry.lf
            )
            vp.mass_geometry.lr = wheelbase - vp.mass_geometry.lf
            vp.mass_geometry.wheelbase = wheelbase
            vp.mass_geometry.cg_height = st.number_input(
                "CG height (m)", 0.7, 1.5, vp.mass_geometry.cg_height
            )

        elif section == "Tire":
            vp.tire.cornering_stiffness_front = st.number_input(
                "Cf front (N/rad)", 60000.0, 250000.0, vp.tire.cornering_stiffness_front
            )
            vp.tire.cornering_stiffness_rear = st.number_input(
                "Cr rear (N/rad)", 60000.0, 250000.0, vp.tire.cornering_stiffness_rear
            )
            vp.tire.friction_coefficient = st.number_input(
                "μ (base)", 0.8, 1.5, vp.tire.friction_coefficient
            )
            vp.tire.wheel_radius = st.number_input(
                "Wheel radius (m)", 0.5, 1.25, vp.tire.wheel_radius
            )

        elif section == "Engine":
            vp.engine.max_power = st.number_input(
                "Power (kW)", 300.0, 900.0, vp.engine.max_power/1000.0
            ) * 1000.0
            vp.engine.max_torque = st.number_input(
                "Torque (Nm)", 2000.0, 6500.0, vp.engine.max_torque
            )
            vp.engine.rpm_max = st.number_input(
                "RPM max", 1800.0, 3500.0, vp.engine.rpm_max
            )
            vp.engine.rpm_idle = st.number_input(
                "RPM idle", 600.0, 1000.0, vp.engine.rpm_idle
            )

        elif section == "Transmission":
            vp.transmission.num_gears = st.slider(
                "Gears", 6, 16, vp.transmission.num_gears
            )
            vp.transmission.final_drive_ratio = st.number_input(
                "Final drive", 2.8, 7.5, vp.transmission.final_drive_ratio
            )

        elif section == "Brake":
            vp.brake.max_deceleration = st.slider(
                "Max decel (m/s²)", 3.0, 10.0, vp.brake.max_deceleration
            )

        elif section == "Aerodynamics":
            vp.aero.drag_coefficient = st.number_input(
                "Cd", 0.45, 1.20, vp.aero.drag_coefficient
            )
            vp.aero.frontal_area = st.number_input(
                "Frontal area (m²)", 7.0, 10.0, vp.aero.frontal_area
            )
            vp.aero.lift_coefficient = st.number_input(
                "Cl", -1.0, 1.0, vp.aero.lift_coefficient
            )


def pista_page():
    """Track selection page (unchanged)"""
    st.header("Track")
    if not os.path.isdir(DATA_PATH):
        st.warning(f"Folder not found: {DATA_PATH}")
        return

    pistas = [f for f in os.listdir(DATA_PATH) if f.endswith('.hdf5')]
    if not pistas:
        st.warning('No tracks found!')
        return

    pista_selecionada = st.selectbox("Select:", pistas)
    caminho_pista = os.path.join(DATA_PATH, pista_selecionada)
    circuit, meta = CircuitHDF5Reader(caminho_pista).read_circuit()

    st.session_state.circuit = circuit
    st.session_state.circuit_meta = meta

    # Plot (unchanged)
    x_c = -(circuit.centerline_y - circuit.centerline_y[0])
    y_c = circuit.centerline_x - circuit.centerline_x[0]
    left_x = -(circuit.left_boundary_y - circuit.centerline_y[0])
    left_y = circuit.left_boundary_x - circuit.centerline_x[0]
    right_x = -(circuit.right_boundary_y - circuit.centerline_y[0])
    right_y = circuit.right_boundary_x - circuit.centerline_x[0]

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=x_c, y=y_c, mode="lines",
                  name="Centerline", line=dict(color="blue", width=2)))
    fig.add_trace(go.Scatter(x=left_x, y=left_y, mode="lines",
                  name="Left", line=dict(color='green', dash='dot')))
    fig.add_trace(go.Scatter(x=right_x, y=right_y, mode="lines",
                  name="Right", line=dict(color='red', dash='dot')))
    fig.add_trace(go.Scatter(x=[x_c[0]], y=[y_c[0]], mode="markers", marker=dict(
        color="orange", size=16, symbol="x"), name="Start"))
    fig.update_layout(title=f"{meta['name']}",
                      xaxis_title="x (m)", yaxis_title="y (m)")
    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    st.plotly_chart(fig, use_container_width=True)

    st.success(f"✓ {meta['name']} | {meta['length']:.2f}m")


def simulacao_page():
    """Simulation configuration and execution page"""
    st.header("Simulation Configuration")

    if st.session_state.circuit is None:
        st.warning("⚠️ Select a track first!")
        return

    st.info(f"✓ Track: {st.session_state.circuit_meta['name']}")

    st.session_state.config["tipo"] = st.selectbox("Type:", [
        "Qualificatória", "Treino Livre", "Stint Longo", "Ultrapassagem", "Largada", "Aquecimento Pneus"
    ])
    st.session_state.config["coef_aderencia"] = st.slider(
        "μ (track condition)", 0.7, 1.4, st.session_state.config["coef_aderencia"]
    )
    st.session_state.config["consumo"] = st.number_input(
        "Fuel consumption (l/100km)", 20.0, 70.0, st.session_state.config["consumo"]
    )
    st.session_state.config["temp_pneu_ini"] = st.slider(
        "Tire temp (°C)", 30.0, 120.0, st.session_state.config["temp_pneu_ini"]
    )

    col_play, col_reset = st.columns(2)

    with col_play:
        if st.button("▶ Play (Simulate)", use_container_width=True):
            with st.spinner("🔄 Simulating..."):
                # Convert VehicleParams to solver dict
                params_dict = st.session_state.vehicle_params.to_solver_dict()
                # Override mu from config (track condition overrides vehicle base mu)
                params_dict["mu"] = st.session_state.config["coef_aderencia"]

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                pista_nome = st.session_state.circuit_meta["name"].replace(
                    " ", "_")
                csv_path = os.path.join(
                    RESULTS_PATH, f"lap_{pista_nome}_{timestamp}.csv")

                resultados = run_bicycle_model(
                    params_dict,
                    st.session_state.circuit,
                    st.session_state.config,
                    save_csv=True,
                    out_path=csv_path
                )

                st.session_state.resultados = resultados
                st.session_state.resultados_prontos = True
                st.session_state.last_csv_path = csv_path
                st.success(f"✓ Lap time: **{resultados['lap_time']:.2f}s**")

    with col_reset:
        if st.button("🔄 Reset Simulation", use_container_width=True):
            st.session_state.circuit = None
            st.session_state.circuit_meta = None
            st.session_state.resultados_prontos = False
            st.session_state.resultados = None
            st.session_state.last_csv_path = None
            st.info("✓ Reset OK. Select track again.")
            st.rerun()


def resultados_page():
    """Results page with KPIs and telemetry (unchanged logic, same as before)"""
    st.header("Results")

    if not st.session_state.get("resultados_prontos", False):
        st.warning("Run a simulation first!")
        return

    res = st.session_state.resultados

    # KPIs
    col1, col2, col3, col4, col5 = st.columns(5)
    with col1:
        st.metric("Lap time", f"{res['lap_time']:.2f}s")
    with col2:
        v_max = np.max(res['v_profile']) * 3.6
        st.metric("V. Max", f"{v_max:.1f} km/h")
    with col3:
        v_avg = np.mean(res['v_profile']) * 3.6
        st.metric("V. Avg", f"{v_avg:.1f} km/h")
    with col4:
        a_lat_max = np.max(np.abs(res['a_lat']))
        st.metric("a_lat Max", f"{a_lat_max:.2f} m/s²")
    with col5:
        a_long_max = np.max(np.abs(res['a_long']))
        st.metric("a_long Max", f"{a_long_max:.2f} m/s²")

    col6, col7, col8, col9, col10 = st.columns(5)
    with col6:
        t_max = np.max(res.get('temp_pneu', [70]))
        st.metric("T Tire", f"{t_max:.1f}°C")
    with col7:
        rpm_max = np.max(res['rpm']) if len(res['rpm']) > 0 else 0
        st.metric("RPM Max", f"{rpm_max:.0f}")
    with col8:
        cons = res['consumo'][-1] if len(res['consumo']) > 0 else 0
        st.metric("Fuel", f"{cons:.2f}L")
    with col9:
        gear_max = int(np.max(res['gear']))
        st.metric("Max Gear", gear_max)
    with col10:
        a_rms = np.sqrt(np.mean(res['a_long']**2 + res['a_lat']**2))
        st.metric("a_RMS", f"{a_rms:.2f} m/s²")

    st.markdown("---")
    st.subheader("📊 Detailed Telemetry")

    telemetry_data = {
        "Distance (m)": np.round(res['distance'], 1),
        "Time (s)": np.round(res['time'], 2),
        "Velocity (km/h)": np.round(res['v_profile'] * 3.6, 1),
        "a_long (m/s²)": np.round(res['a_long'], 2),
        "a_lat (m/s²)": np.round(res['a_lat'], 2),
        "Gear": res['gear'].astype(int),
        "RPM": np.round(res['rpm'], 0).astype(int),
        "Radius (m)": np.round(res['radius'], 0).astype(int),
        "Temp (°C)": np.round(res.get('temp_pneu', [70]*len(res['v_profile'])), 1),
        "Fuel (L)": np.round(res['consumo'], 3),
    }

    telemetry_df = pd.DataFrame(telemetry_data)

    with st.expander("View full table (downsample 1 every 5 points)"):
        st.dataframe(telemetry_df.iloc[::5],
                     use_container_width=True, height=400)

    st.markdown("---")
    st.subheader("📈 Charts")

    # Charts (same as before)
    col_g1, col_g2 = st.columns(2)
    with col_g1:
        fig_v = go.Figure()
        fig_v.add_trace(go.Scatter(x=res['distance'], y=res['v_profile']*3.6,
                        mode="lines", name="Velocity", line=dict(color="blue", width=2)))
        fig_v.update_layout(
            title="Velocity", xaxis_title="Distance (m)", yaxis_title="km/h", height=350)
        st.plotly_chart(fig_v, use_container_width=True)

    with col_g2:
        fig_a = go.Figure()
        fig_a.add_trace(go.Scatter(x=res['distance'], y=res['a_lat'],
                        mode="lines", name="a_lateral", line=dict(color="red", width=2)))
        fig_a.update_layout(title="Lateral Acceleration",
                            xaxis_title="Distance (m)", yaxis_title="m/s²", height=350)
        st.plotly_chart(fig_a, use_container_width=True)

    col_g3, col_g4 = st.columns(2)
    with col_g3:
        fig_gear = go.Figure()
        fig_gear.add_trace(go.Scatter(
            x=res['distance'], y=res['gear'], mode="lines", name="Gear", line=dict(color="green", width=2)))
        if 'rpm' in res and len(res['rpm']) > 0:
            fig_gear.add_trace(go.Scatter(x=res['distance'], y=np.array(
                res['rpm'])/200, mode="lines", name="RPM (÷200)", line=dict(color="orange", width=1, dash="dot")))
        fig_gear.update_layout(
            title="Gear & RPM", xaxis_title="Distance (m)", yaxis_title="Value", height=350)
        st.plotly_chart(fig_gear, use_container_width=True)

    with col_g4:
        if 'temp_pneu' in res:
            fig_temp = go.Figure()
            fig_temp.add_trace(go.Scatter(
                x=res['distance'], y=res['temp_pneu'], mode="lines", name="Temp", line=dict(color="purple", width=2)))
            fig_temp.update_layout(
                title="Tire Temperature", xaxis_title="Distance (m)", yaxis_title="°C", height=350)
            st.plotly_chart(fig_temp, use_container_width=True)

    st.markdown("---")

    # Download CSV
    if st.session_state.get("last_csv_path") and os.path.exists(st.session_state["last_csv_path"]):
        with open(st.session_state["last_csv_path"], "rb") as f:
            st.download_button(
                "📥 Download CSV", f, f"lap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")


PAGES = {
    "Parameters": parametros_veiculo_page,
    "Track": pista_page,
    "Simulation": simulacao_page,
    "Results": resultados_page
}


# ===== MAIN =====
st.set_page_config(page_title="LapTimeSimulator", layout="wide")
init_session_state()

st.sidebar.title("🏁 LapTimeSimulator")
page = st.sidebar.radio("Choose:", list(PAGES.keys()))
st.sidebar.markdown("<hr>", unsafe_allow_html=True)

with st.sidebar.expander("📋 Parameters"):
    vp = st.session_state.vehicle_params
    st.write(f"**Mass:** {vp.mass_geometry.mass:.0f} kg")
    st.write(f"**Power:** {vp.engine.max_power/1000.0:.0f} kW")
    st.write(f"**μ:** {st.session_state.config['coef_aderencia']:.2f}")
    if st.session_state.circuit_meta:
        st.write(f"**Track:** {st.session_state.circuit_meta['name']}")

PAGES[page]()
