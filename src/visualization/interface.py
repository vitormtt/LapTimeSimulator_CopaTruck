# ============ CRITICAL: Must execute BEFORE any custom imports ============
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))
# ===========================================================================

# fmt: off
# isort: skip_file

import os
import json
from datetime import datetime
from typing import Dict, List, Optional

import streamlit as st
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from tracks.hdf5 import CircuitHDF5Reader
from simulation.lap_time_solver import run_bicycle_model
from vehicle.parameters import VehicleParams, copa_truck_2dof_default
from visualization.kpi_dashboard import (
    build_kpi_dataframe,
    compare_lap_times,
    plot_gg_diagram,
    plot_speed_vs_distance,
    plot_channels_vs_distance,
)

# fmt: on

# ===== PATHS =====
# Resolve paths relative to the repo root to avoid absolute user-specific paths
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DATA_PATH   = str(_REPO_ROOT / "tracks")
RESULTS_PATH = str(_REPO_ROOT / "src" / "results")
os.makedirs(RESULTS_PATH, exist_ok=True)

# ===== THEME CONSTANTS =====
_PLOTLY_TEMPLATE = "plotly_dark"
_PRIMARY   = "#00C8FF"   # cyan accent
_SECONDARY = "#FF6B35"   # orange accent
_SUCCESS   = "#2ECC71"
_DANGER    = "#E74C3C"
_SURFACE   = "#1E1E2E"   # card background
_DIVIDER   = "rgba(255,255,255,0.08)"

_CSS = """
<style>
/* ── Global dark background ────────────────────────────── */
[data-testid="stAppViewContainer"] { background: #13131F; }
[data-testid="stSidebar"] { background: #1A1A2E; border-right: 1px solid rgba(255,255,255,0.06); }

/* ── Metric cards ──────────────────────────────────────── */
[data-testid="metric-container"] {
    background: #1E1E2E;
    border: 1px solid rgba(0,200,255,0.18);
    border-radius: 10px;
    padding: 14px 18px;
}
[data-testid="metric-container"] [data-testid="stMetricValue"] {
    font-size: 1.55rem;
    font-weight: 700;
    color: #00C8FF;
}
[data-testid="metric-container"] [data-testid="stMetricLabel"] {
    font-size: 0.72rem;
    color: rgba(255,255,255,0.55);
    text-transform: uppercase;
    letter-spacing: 0.07em;
}

/* ── Section headers ───────────────────────────────────── */
h2, h3 { color: #E0E0F0 !important; }

/* ── Sidebar radio labels ──────────────────────────────── */
[data-testid="stSidebar"] label { color: #C0C0D8 !important; font-size: 0.88rem; }

/* ── DataTable header ──────────────────────────────────── */
thead th { background: #1E1E2E !important; color: #00C8FF !important; }

/* ── Button primary ────────────────────────────────────── */
[data-testid="baseButton-primary"] {
    background: #00C8FF !important;
    color: #0D0D1A !important;
    border-radius: 8px;
    font-weight: 600;
}

/* ── Divider ────────────────────────────────────────────── */
.section-divider {
    border: none;
    border-top: 1px solid rgba(255,255,255,0.08);
    margin: 1.2rem 0;
}

/* ── Lap tag badge ──────────────────────────────────────── */
.lap-badge {
    display: inline-block;
    padding: 3px 10px;
    border-radius: 20px;
    font-size: 0.78rem;
    font-weight: 600;
    margin-right: 6px;
}
.lap-badge-ref  { background: rgba(0,200,255,0.18); color: #00C8FF; border: 1px solid #00C8FF44; }
.lap-badge-comp { background: rgba(255,107,53,0.18); color: #FF6B35; border: 1px solid #FF6B3544; }
</style>
"""


# ===== SESSION STATE =====

def init_session_state() -> None:
    """Initialize all session_state keys with safe defaults."""
    defaults: Dict = {
        "vehicle_params":     copa_truck_2dof_default(),
        "config":             {"tipo": "Qualificatória", "coef_aderencia": 1.09,
                               "consumo": 43.0, "temp_pneu_ini": 65.0},
        "circuit":            None,
        "circuit_meta":       None,
        "resultados_prontos": False,
        "resultados":         None,
        "last_csv_path":      None,
        # Lap session storage — list of dicts with {label, result, params_snapshot}
        "lap_session":        [],
        # Channel selector for telemetry multi-channel plot
        "selected_channels":  ["v_kmh", "throttle_pct", "brake_pct", "gear", "rpm", "temp_tyre_c"],
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v


# ===== HELPER UTILS =====

_COLORS = [_PRIMARY, _SECONDARY, "#A29BFE", "#FD79A8", "#55EFC4", "#FDCB6E"]


def _plotly_base_layout(title: str = "", height: int = 360) -> dict:
    """Return common Plotly layout kwargs for dark theme."""
    return dict(
        template=_PLOTLY_TEMPLATE,
        title=dict(text=title, font=dict(size=13, color="#C0C0D8")),
        height=height,
        margin=dict(l=48, r=20, t=44, b=36),
        legend=dict(orientation="h", yanchor="bottom", y=1.02,
                    font=dict(size=11), bgcolor="rgba(0,0,0,0)"),
        paper_bgcolor="#1E1E2E",
        plot_bgcolor="#1E1E2E",
    )


def _delta_style(delta: float) -> str:
    """Return HTML span with color-coded delta."""
    if delta == 0.0:
        return "<span style='color:#00C8FF;font-weight:700'>REF</span>"
    color = _DANGER if delta > 0 else _SUCCESS
    sign  = "+" if delta > 0 else ""
    return f"<span style='color:{color};font-weight:600'>{sign}{delta:.3f}s</span>"


# ===== SIDEBAR =====

def _render_sidebar() -> str:
    """Render sidebar navigation; returns selected page key."""
    st.sidebar.markdown(
        "<div style='text-align:center;padding:12px 0 6px'>"
        "<span style='font-size:1.6rem'>🏁</span>"
        "<p style='color:#00C8FF;font-weight:700;font-size:1.05rem;margin:4px 0 0'>LapTimeSimulator</p>"
        "<p style='color:#888;font-size:0.72rem;margin:0'>Copa Truck · v2.0</p>"
        "</div>",
        unsafe_allow_html=True,
    )
    st.sidebar.markdown("<hr style='border-color:rgba(255,255,255,0.08)'>", unsafe_allow_html=True)

    PAGES = {
        "⚙️  Parameters":   "parameters",
        "🏎️  Track":         "track",
        "▶️  Simulation":    "simulation",
        "📊  Results":       "results",
        "🔄  Compare Laps": "compare",
    }
    selection = st.sidebar.radio("", list(PAGES.keys()), label_visibility="collapsed")
    page_key  = PAGES[selection]

    st.sidebar.markdown("<hr style='border-color:rgba(255,255,255,0.08)'>", unsafe_allow_html=True)

    # Quick status card in sidebar
    with st.sidebar.expander("📋 Current Config", expanded=True):
        vp  = st.session_state.vehicle_params
        cfg = st.session_state.config
        st.markdown(f"**Mass:** {vp.mass_geometry.mass:.0f} kg")
        st.markdown(f"**Power:** {vp.engine.max_power/1000.0:.0f} kW")
        st.markdown(f"**μ track:** {cfg['coef_aderencia']:.2f}")
        if st.session_state.circuit_meta:
            st.markdown(f"**Track:** {st.session_state.circuit_meta['name']}")
        n_laps = len(st.session_state.lap_session)
        if n_laps:
            st.markdown(f"**Saved laps:** {n_laps}")

    return page_key


# ===== PAGE: PARAMETERS =====

def page_parameters() -> None:
    """Vehicle parameters configuration page."""
    st.header("Vehicle Parameters")
    st.caption("Copa Truck 2-DOF bicycle model")

    vp = st.session_state.vehicle_params

    # Summary metrics row
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Manufacturer", vp.manufacturer)
    c2.metric("Year", str(vp.year))
    c3.metric("Power", f"{vp.engine.max_power/1000:.0f} kW")
    c4.metric("Mass", f"{vp.mass_geometry.mass:.0f} kg")

    st.markdown("<div class='section-divider'></div>", unsafe_allow_html=True)

    SECTIONS = ["Mass / Geometry", "Tyre", "Engine", "Transmission", "Brakes", "Aerodynamics"]
    tabs      = st.tabs(SECTIONS)

    # ── Tab 0: Mass / Geometry ──────────────────────────────────
    with tabs[0]:
        col_a, col_b = st.columns(2)
        with col_a:
            vp.mass_geometry.mass = st.number_input(
                "Total mass (kg)", 3500.0, 9000.0, vp.mass_geometry.mass, step=50.0
            )
            wheelbase = st.number_input(
                "Wheelbase (m)", 3.8, 5.5, vp.mass_geometry.wheelbase, step=0.05
            )
            vp.mass_geometry.lf = st.number_input(
                "CG → front axle (m)", 1.0, 3.5, vp.mass_geometry.lf, step=0.05
            )
            vp.mass_geometry.lr        = wheelbase - vp.mass_geometry.lf
            vp.mass_geometry.wheelbase = wheelbase
        with col_b:
            vp.mass_geometry.cg_height = st.number_input(
                "CG height (m)", 0.5, 1.6, vp.mass_geometry.cg_height, step=0.02
            )
            st.metric("CG → rear axle (m)", f"{vp.mass_geometry.lr:.3f}")
            st.metric("Weight dist. F/R",
                      f"{vp.mass_geometry.lr/wheelbase*100:.1f}% / {vp.mass_geometry.lf/wheelbase*100:.1f}%")

    # ── Tab 1: Tyre ─────────────────────────────────────────────
    with tabs[1]:
        col_a, col_b = st.columns(2)
        with col_a:
            vp.tire.cornering_stiffness_front = st.number_input(
                "Cornering stiffness front Cf (N/rad)", 60_000.0, 350_000.0,
                vp.tire.cornering_stiffness_front, step=5_000.0
            )
            vp.tire.cornering_stiffness_rear = st.number_input(
                "Cornering stiffness rear Cr (N/rad)", 60_000.0, 350_000.0,
                vp.tire.cornering_stiffness_rear, step=5_000.0
            )
        with col_b:
            vp.tire.friction_coefficient = st.number_input(
                "Friction coefficient μ (base)", 0.70, 1.60, vp.tire.friction_coefficient, step=0.01
            )
            vp.tire.wheel_radius = st.number_input(
                "Wheel radius (m)", 0.45, 1.30, vp.tire.wheel_radius, step=0.01
            )

    # ── Tab 2: Engine ────────────────────────────────────────────
    with tabs[2]:
        col_a, col_b = st.columns(2)
        with col_a:
            vp.engine.max_power = st.number_input(
                "Max power (kW)", 300.0, 900.0, vp.engine.max_power / 1000.0, step=10.0
            ) * 1000.0
            vp.engine.max_torque = st.number_input(
                "Max torque (N·m)", 2000.0, 7000.0, vp.engine.max_torque, step=100.0
            )
        with col_b:
            vp.engine.rpm_max = st.number_input(
                "RPM max", 1800.0, 4000.0, vp.engine.rpm_max, step=100.0
            )
            vp.engine.rpm_idle = st.number_input(
                "RPM idle", 500.0, 1100.0, vp.engine.rpm_idle, step=50.0
            )

    # ── Tab 3: Transmission ──────────────────────────────────────
    with tabs[3]:
        col_a, col_b = st.columns(2)
        with col_a:
            vp.transmission.num_gears = st.slider("Number of gears", 6, 16, vp.transmission.num_gears)
        with col_b:
            vp.transmission.final_drive_ratio = st.number_input(
                "Final drive ratio", 2.5, 8.0, vp.transmission.final_drive_ratio, step=0.1
            )

    # ── Tab 4: Brakes ────────────────────────────────────────────
    with tabs[4]:
        vp.brake.max_deceleration = st.slider(
            "Max deceleration (m/s²)", 3.0, 10.0, vp.brake.max_deceleration, step=0.1
        )

    # ── Tab 5: Aerodynamics ──────────────────────────────────────
    with tabs[5]:
        col_a, col_b, col_c = st.columns(3)
        with col_a:
            vp.aero.drag_coefficient = st.number_input(
                "Drag coeff. Cd", 0.40, 1.30, vp.aero.drag_coefficient, step=0.01
            )
        with col_b:
            vp.aero.frontal_area = st.number_input(
                "Frontal area (m²)", 6.0, 11.0, vp.aero.frontal_area, step=0.1
            )
        with col_c:
            vp.aero.lift_coefficient = st.number_input(
                "Lift coeff. Cl", -1.5, 1.5, vp.aero.lift_coefficient, step=0.05
            )


# ===== PAGE: TRACK =====

def page_track() -> None:
    """Track selection and preview page."""
    st.header("Track")

    if not os.path.isdir(DATA_PATH):
        st.error(f"Tracks folder not found: `{DATA_PATH}`")
        st.info("Tip: set DATA_PATH in `interface.py` or add an env var `LTS_TRACKS_PATH`.")
        return

    pistas = sorted([f for f in os.listdir(DATA_PATH) if f.endswith(".hdf5")])
    if not pistas:
        st.warning("No `.hdf5` tracks found in the tracks folder.")
        return

    selected = st.selectbox("Select track", pistas)
    path     = os.path.join(DATA_PATH, selected)
    circuit, meta = CircuitHDF5Reader(path).read_circuit()
    st.session_state.circuit      = circuit
    st.session_state.circuit_meta = meta

    # ── Track plot ───────────────────────────────────────────────
    x_c     = -(circuit.centerline_y - circuit.centerline_y[0])
    y_c     =   circuit.centerline_x - circuit.centerline_x[0]
    left_x  = -(circuit.left_boundary_y  - circuit.centerline_y[0])
    left_y  =   circuit.left_boundary_x  - circuit.centerline_x[0]
    right_x = -(circuit.right_boundary_y - circuit.centerline_y[0])
    right_y =   circuit.right_boundary_x - circuit.centerline_x[0]

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=right_x, y=right_y, mode="lines",
                             name="Right", line=dict(color="#E74C3C", width=1.5, dash="dot")))
    fig.add_trace(go.Scatter(x=left_x,  y=left_y,  mode="lines",
                             name="Left",  line=dict(color="#2ECC71", width=1.5, dash="dot")))
    fig.add_trace(go.Scatter(x=x_c, y=y_c, mode="lines",
                             name="Centerline", line=dict(color=_PRIMARY, width=2.5)))
    fig.add_trace(go.Scatter(x=[x_c[0]], y=[y_c[0]], mode="markers",
                             marker=dict(color="#FDCB6E", size=14, symbol="star"),
                             name="Start / Finish"))
    fig.update_layout(
        **_plotly_base_layout(f"{meta['name']}", height=500),
        xaxis_title="x (m)",
        yaxis_title="y (m)",
        yaxis=dict(scaleanchor="x", scaleratio=1),
    )
    st.plotly_chart(fig, use_container_width=True)

    # ── Track meta cards ─────────────────────────────────────────
    c1, c2, c3 = st.columns(3)
    c1.metric("Circuit", meta["name"])
    c2.metric("Length", f"{meta['length']:.1f} m")
    c3.metric("Points", f"{len(circuit.centerline_x):,}")


# ===== PAGE: SIMULATION =====

def page_simulation() -> None:
    """Simulation config and execution page."""
    st.header("Simulation")

    if st.session_state.circuit is None:
        st.warning("⚠️ Select a track first (Track page).")
        return

    st.info(f"✓ Track loaded: **{st.session_state.circuit_meta['name']}** "
            f"· {st.session_state.circuit_meta['length']:.1f} m")

    st.markdown("<div class='section-divider'></div>", unsafe_allow_html=True)

    col_cfg, col_run = st.columns([3, 1])

    with col_cfg:
        col_a, col_b = st.columns(2)
        with col_a:
            st.session_state.config["tipo"] = st.selectbox(
                "Session type",
                ["Qualificatória", "Treino Livre", "Stint Longo",
                 "Ultrapassagem", "Largada", "Aquecimento Pneus"],
            )
            st.session_state.config["coef_aderencia"] = st.slider(
                "Track grip μ", 0.70, 1.40,
                st.session_state.config["coef_aderencia"], step=0.01
            )
        with col_b:
            st.session_state.config["consumo"] = st.number_input(
                "Fuel consumption (l/100 km)", 20.0, 70.0,
                st.session_state.config["consumo"], step=1.0
            )
            st.session_state.config["temp_pneu_ini"] = st.slider(
                "Initial tyre temp (°C)", 30.0, 120.0,
                st.session_state.config["temp_pneu_ini"], step=1.0
            )

        # Lap label for session storage
        lap_label = st.text_input(
            "Lap label (for Compare Laps)",
            value=f"{st.session_state.circuit_meta['name']} – {datetime.now().strftime('%H:%M:%S')}",
        )

    with col_run:
        st.markdown("<br><br>", unsafe_allow_html=True)
        run_btn   = st.button("▶  Simulate",  type="primary", use_container_width=True)
        reset_btn = st.button("🔄  Reset",    use_container_width=True)
        save_btn  = st.button("💾  Save Lap",  use_container_width=True)

    # ── Run simulation ───────────────────────────────────────────
    if run_btn:
        with st.spinner("Running simulation…"):
            params_dict       = st.session_state.vehicle_params.to_solver_dict()
            params_dict["mu"] = st.session_state.config["coef_aderencia"]

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            track_name = st.session_state.circuit_meta["name"].replace(" ", "_")
            csv_path   = os.path.join(RESULTS_PATH, f"lap_{track_name}_{timestamp}.csv")

            res = run_bicycle_model(
                params_dict,
                st.session_state.circuit,
                st.session_state.config,
                save_csv=True,
                out_path=csv_path,
            )
            st.session_state.resultados       = res
            st.session_state.resultados_prontos = True
            st.session_state.last_csv_path    = csv_path
            st.session_state["_last_lap_label"] = lap_label
            st.success(f"✅ Lap time: **{res['lap_time']:.3f} s** "
                       f"· V_max: **{np.max(res['v_profile'])*3.6:.1f} km/h**")

    # ── Save to session ──────────────────────────────────────────
    if save_btn:
        if not st.session_state.resultados_prontos:
            st.warning("Run simulation first.")
        else:
            label   = st.session_state.get("_last_lap_label", lap_label)
            entry   = {
                "label":  label,
                "result": st.session_state.resultados,
                "params": {
                    "mass":  st.session_state.vehicle_params.mass_geometry.mass,
                    "power": st.session_state.vehicle_params.engine.max_power / 1000,
                    "mu":    st.session_state.config["coef_aderencia"],
                    "track": st.session_state.circuit_meta["name"],
                },
            }
            st.session_state.lap_session.append(entry)
            st.success(f"💾 Lap '{label}' saved to session ({len(st.session_state.lap_session)} stored).")

    # ── Reset ────────────────────────────────────────────────────
    if reset_btn:
        for key in ["circuit", "circuit_meta", "resultados_prontos", "resultados", "last_csv_path"]:
            st.session_state[key] = None if key != "resultados_prontos" else False
        st.info("✓ Reset complete.")
        st.rerun()


# ===== PAGE: RESULTS =====

_ALL_CHANNELS = [
    "v_kmh", "throttle_pct", "brake_pct", "steering_deg",
    "gear", "rpm", "ax_long_g", "ay_lat_g", "temp_tyre_c",
    "tyre_pressure_bar", "fuel_used_l",
]


def _build_result_obj(res: dict):
    """Wrap the raw solver dict into a lightweight namespace for kpi_dashboard compat."""
    from types import SimpleNamespace
    v_kmh   = np.asarray(res["v_profile"]) * 3.6
    a_lat_g = np.asarray(res["a_lat"]) / 9.81
    a_long_g= np.asarray(res["a_long"]) / 9.81
    obj = SimpleNamespace(
        lap_time       = float(res["lap_time"]),
        setup_name     = res.get("setup_name", "Run"),
        mode           = SimpleNamespace(name=res.get("tipo", "Sim")),
        distance       = np.asarray(res["distance"]),
        v_kmh          = v_kmh,
        ay_lat_g       = a_lat_g,
        ax_long_g      = a_long_g,
        gear           = np.asarray(res["gear"]).astype(int),
        rpm            = np.asarray(res["rpm"]),
        fuel_used_l    = np.asarray(res["consumo"]),
        temp_tyre_c    = np.asarray(res.get("temp_pneu", np.full(len(v_kmh), 70.0))),
        tyre_pressure_bar = np.asarray(res.get("pressao_pneu", np.full(len(v_kmh), 8.5))),
        throttle_pct   = np.clip(a_long_g / np.max(np.abs(a_long_g) + 1e-9) * 100.0, 0, 100),
        brake_pct      = np.clip(-a_long_g / np.max(np.abs(a_long_g) + 1e-9) * 100.0, 0, 100),
        steering_deg   = np.zeros(len(v_kmh)),
        avg_speed_kmh  = float(np.mean(v_kmh)),
        max_speed_kmh  = float(np.max(v_kmh)),
        peak_lat_g     = float(np.max(np.abs(a_lat_g))),
        peak_accel_g   = float(np.max(a_long_g)),
        peak_brake_g   = float(np.min(a_long_g)),
        time_wot_pct   = float(np.mean(a_long_g > 0.05) * 100),
        time_braking_pct = float(np.mean(a_long_g < -0.05) * 100),
        fuel_total_l   = float(np.asarray(res["consumo"])[-1]),
        final_tyre_temp_c     = float(np.asarray(res.get("temp_pneu", [70.0]))[-1]),
        final_tyre_pressure_bar = float(np.asarray(res.get("pressao_pneu", [8.5]))[-1]),
    )
    return obj


def page_results() -> None:
    """Results analysis page: KPIs, telemetry channels, GG diagram."""
    st.header("Results")

    if not st.session_state.get("resultados_prontos"):
        st.warning("Run a simulation first (Simulation page).")
        return

    res = st.session_state.resultados
    obj = _build_result_obj(res)

    # ── KPI Row 1 ────────────────────────────────────────────────
    st.subheader("Key Performance Indicators")
    c1, c2, c3, c4, c5 = st.columns(5)
    c1.metric("Lap Time",   f"{obj.lap_time:.3f} s")
    c2.metric("V max",      f"{obj.max_speed_kmh:.1f} km/h")
    c3.metric("V avg",      f"{obj.avg_speed_kmh:.1f} km/h")
    c4.metric("Peak lat g", f"{obj.peak_lat_g:.3f} g")
    c5.metric("Fuel total", f"{obj.fuel_total_l:.2f} L")

    c6, c7, c8, c9, c10 = st.columns(5)
    c6.metric("Peak accel g",  f"{obj.peak_accel_g:.3f} g")
    c7.metric("Peak brake g",  f"{obj.peak_brake_g:.3f} g")
    c8.metric("WOT time",      f"{obj.time_wot_pct:.1f} %")
    c9.metric("Braking time",  f"{obj.time_braking_pct:.1f} %")
    c10.metric("Tyre temp",    f"{obj.final_tyre_temp_c:.1f} °C")

    st.markdown("<div class='section-divider'></div>", unsafe_allow_html=True)

    # ── Charts: Speed + GG side by side ─────────────────────────
    st.subheader("Telemetry Charts")
    col_spd, col_gg = st.columns([2, 1])

    with col_spd:
        fig_v = go.Figure()
        fig_v.add_trace(go.Scatter(
            x=obj.distance, y=obj.v_kmh, mode="lines",
            name="Speed", line=dict(color=_PRIMARY, width=2)
        ))
        fig_v.update_layout(**_plotly_base_layout("Speed vs Distance", height=320),
                            xaxis_title="Distance (m)", yaxis_title="km/h")
        st.plotly_chart(fig_v, use_container_width=True)

    with col_gg:
        fig_gg = plot_gg_diagram(obj, title="GG Diagram", height=320)
        fig_gg.update_layout(**_plotly_base_layout("GG Diagram", height=320))
        st.plotly_chart(fig_gg, use_container_width=True)

    # ── Multi-channel stacked telemetry ──────────────────────────
    st.subheader("Multi-channel Telemetry")
    available = [c for c in _ALL_CHANNELS if getattr(obj, c, None) is not None]
    selected  = st.multiselect(
        "Channels", available,
        default=st.session_state.selected_channels,
        key="channel_multiselect",
    )
    st.session_state.selected_channels = selected

    if selected:
        fig_ch = plot_channels_vs_distance(obj, channels=selected, height=max(520, len(selected)*110))
        fig_ch.update_layout(template=_PLOTLY_TEMPLATE, paper_bgcolor="#1E1E2E", plot_bgcolor="#1E1E2E")
        st.plotly_chart(fig_ch, use_container_width=True)

    st.markdown("<div class='section-divider'></div>", unsafe_allow_html=True)

    # ── Raw telemetry table ──────────────────────────────────────
    st.subheader("Raw Telemetry Table")
    skip = st.slider("Downsample (1 per N points)", 1, 20, 5)
    df_tele = pd.DataFrame({
        "Dist (m)":  np.round(res["distance"],   1),
        "Time (s)":  np.round(res["time"],        2),
        "V (km/h)":  np.round(obj.v_kmh,          1),
        "a_long (g)": np.round(obj.ax_long_g,     3),
        "a_lat (g)":  np.round(obj.ay_lat_g,      3),
        "Gear":       obj.gear,
        "RPM":        np.round(obj.rpm, 0).astype(int),
        "Tyre T (°C)": np.round(obj.temp_tyre_c,  1),
        "Fuel (L)":   np.round(obj.fuel_used_l,   3),
    })
    st.dataframe(df_tele.iloc[::skip], use_container_width=True, height=380)

    # ── CSV download ─────────────────────────────────────────────
    csv_path = st.session_state.get("last_csv_path")
    if csv_path and os.path.exists(csv_path):
        with open(csv_path, "rb") as f:
            st.download_button(
                "📥 Download CSV", f,
                file_name=f"lap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                mime="text/csv",
            )


# ===== PAGE: COMPARE LAPS =====

def page_compare() -> None:
    """Multi-lap comparative analysis page."""
    st.header("Compare Laps")

    session: List[dict] = st.session_state.lap_session
    if len(session) < 1:
        st.info("No laps saved yet. Run simulations and click **💾 Save Lap** on the Simulation page.")
        return

    labels = [e["label"] for e in session]

    # ── Lap management ───────────────────────────────────────────
    with st.expander("Manage saved laps", expanded=False):
        to_del = st.multiselect("Select laps to remove", labels)
        if st.button("🗑️  Remove selected") and to_del:
            st.session_state.lap_session = [e for e in session if e["label"] not in to_del]
            st.rerun()
        if st.button("🗑️  Clear ALL laps"):
            st.session_state.lap_session = []
            st.rerun()

    session = st.session_state.lap_session  # refresh after potential deletion
    if not session:
        return

    labels = [e["label"] for e in session]
    selected_labels = st.multiselect("Laps to compare", labels, default=labels[:min(4, len(labels))])
    selected_entries = [e for e in session if e["label"] in selected_labels]

    if not selected_entries:
        return

    objs = [_build_result_obj(e["result"]) for e in selected_entries]
    for i, (o, e) in enumerate(zip(objs, selected_entries)):
        o.setup_name = e["label"]
        o.mode       = type("M", (), {"name": e["params"].get("track", "")})()

    # ── Lap time table ───────────────────────────────────────────
    st.subheader("Lap Time Comparison")
    fastest_time = min(o.lap_time for o in objs)
    rows = []
    for o, e in zip(objs, selected_entries):
        delta = o.lap_time - fastest_time
        rows.append({
            "Lap":         e["label"],
            "Lap Time (s)": round(o.lap_time, 3),
            "Δ to fastest": f"+{delta:.3f}" if delta > 0 else "REF",
            "V max (km/h)": round(o.max_speed_kmh, 1),
            "V avg (km/h)": round(o.avg_speed_kmh, 1),
            "Peak lat (g)": round(o.peak_lat_g, 3),
            "Fuel (L)":     round(o.fuel_total_l, 2),
            "Mass (kg)":    e["params"].get("mass", "–"),
            "Power (kW)":   e["params"].get("power", "–"),
            "μ track":      e["params"].get("mu", "–"),
        })
    df_comp = pd.DataFrame(rows)
    st.dataframe(df_comp, use_container_width=True, hide_index=True)

    st.markdown("<div class='section-divider'></div>", unsafe_allow_html=True)

    # ── Speed overlay ────────────────────────────────────────────
    st.subheader("Speed Overlay")
    fig_ov = plot_speed_vs_distance(objs)
    fig_ov.update_layout(**_plotly_base_layout("Speed vs Distance — All Laps", height=380))
    st.plotly_chart(fig_ov, use_container_width=True)

    # ── GG diagrams grid ─────────────────────────────────────────
    st.subheader("GG Diagrams")
    n_objs = len(objs)
    gg_cols = st.columns(min(n_objs, 3))
    for i, obj in enumerate(objs):
        with gg_cols[i % 3]:
            fig_gg = plot_gg_diagram(obj, title=obj.setup_name[:30], height=320)
            fig_gg.update_layout(**_plotly_base_layout(obj.setup_name[:30], height=320))
            st.plotly_chart(fig_gg, use_container_width=True)

    # ── Full KPI table ───────────────────────────────────────────
    st.subheader("Full KPI Table")
    try:
        df_kpi = build_kpi_dataframe(objs)
        st.dataframe(df_kpi.T, use_container_width=True)
    except Exception as exc:
        st.warning(f"KPI table unavailable: {exc}")

    # ── Channel comparison for a chosen channel ──────────────────
    st.subheader("Channel Overlay")
    channel = st.selectbox("Channel", ["v_kmh", "ax_long_g", "ay_lat_g",
                                        "rpm", "gear", "temp_tyre_c", "fuel_used_l"])
    fig_ch = go.Figure()
    for i, obj in enumerate(objs):
        data = getattr(obj, channel, None)
        if data is not None:
            fig_ch.add_trace(go.Scatter(
                x=obj.distance, y=data, mode="lines",
                name=obj.setup_name, line=dict(width=1.8, color=_COLORS[i % len(_COLORS)]),
            ))
    fig_ch.update_layout(**_plotly_base_layout(f"{channel} — Overlay", height=360),
                         xaxis_title="Distance (m)")
    st.plotly_chart(fig_ch, use_container_width=True)


# ===== MAIN ENTRY POINT =====

st.set_page_config(
    page_title="LapTimeSimulator · Copa Truck",
    page_icon="🏁",
    layout="wide",
    initial_sidebar_state="expanded",
)
st.markdown(_CSS, unsafe_allow_html=True)
init_session_state()

page = _render_sidebar()

_PAGE_MAP = {
    "parameters": page_parameters,
    "track":       page_track,
    "simulation":  page_simulation,
    "results":     page_results,
    "compare":     page_compare,
}
_PAGE_MAP[page]()
