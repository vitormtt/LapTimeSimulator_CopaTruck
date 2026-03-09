import os
import sys
import numpy as np
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from src.tracks.circuit import Circuit
from src.tracks.hdf5 import CircuitHDF5Writer

def generate_oval_track(name: str, length_m: float, radius_m: float) -> Circuit:
    """Gera um oval simples genérico para fallback/testes se não houver OSM real"""
    print(f"Gerando geometria sintética: {name}")
    
    straight_len = (length_m - (2 * np.pi * radius_m)) / 2
    
    # Reta inferior
    x1 = np.linspace(0, straight_len, 500)
    y1 = np.zeros_like(x1)
    
    # Curva direita (180 deg)
    theta1 = np.linspace(-np.pi/2, np.pi/2, 500)
    x2 = straight_len + radius_m * np.cos(theta1)
    y2 = radius_m + radius_m * np.sin(theta1)
    
    # Reta superior
    x3 = np.linspace(straight_len, 0, 500)
    y3 = np.full_like(x3, 2 * radius_m)
    
    # Curva esquerda (180 deg)
    theta2 = np.linspace(np.pi/2, 3*np.pi/2, 500)
    x4 = radius_m * np.cos(theta2)
    y4 = radius_m + radius_m * np.sin(theta2)
    
    x = np.concatenate([x1, x2, x3, x4])
    y = np.concatenate([y1, y2, y3, y4])
    
    track_width = 12.0 # Largura da pista (m)
    
    c = Circuit(
        name=name,
        length=length_m,
        centerline_x=x,
        centerline_y=y,
        left_boundary_x=x, # Simplificação sem calcular vetor normal
        left_boundary_y=y + track_width/2,
        right_boundary_x=x,
        right_boundary_y=y - track_width/2
    )
    return c

def create_brazilian_tracks():
    tracks_dir = ROOT_DIR / "tracks"
    os.makedirs(tracks_dir, exist_ok=True)
    
    # Dicionário de autódromos (Comprimento e raio médio de curvas fechadas)
    # Valores reais aproximados (Velocitta: 3.4km, Campo Grande: 3.5km, Brasília: 5.4km)
    autodromos = {
        "velocitta": {"name": "Velocitta", "length": 3493.0, "radius": 45.0},
        "campo_grande": {"name": "Autódromo Internacional de Campo Grande", "length": 3533.0, "radius": 50.0},
        "brasilia_nelson_piquet": {"name": "Autódromo Nelson Piquet (Brasília)", "length": 5475.0, "radius": 70.0}
    }
    
    for key, data in autodromos.items():
        file_path = tracks_dir / f"{key}.hdf5"
        if not file_path.exists():
            print(f"[{key}] Arquivo HDF5 não encontrado. Gerando track map...")
            circuit = generate_oval_track(data["name"], data["length"], data["radius"])
            
            writer = CircuitHDF5Writer(str(file_path))
            writer.write_circuit(circuit)
            print(f"✓ Salvo em: {file_path}")
        else:
            print(f"[{key}] Pista já existe.")

if __name__ == "__main__":
    print("--- GERADOR DE PISTAS NACIONAIS (HDF5) ---")
    print("Nota: Na ausência da malha GPS/OSM real, este script gera malhas fechadas sintéticas")
    print("com o exato comprimento oficial para permitir que o lap_time_solver funcione imediatamente.\n")
    create_brazilian_tracks()
    print("\nProcesso concluído! Reinicie o Streamlit para ver as novas pistas na UI.")
