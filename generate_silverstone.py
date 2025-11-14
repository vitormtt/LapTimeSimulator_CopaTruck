import numpy as np
import h5py
import os

DATA_PATH = r"C:\Users\vitor\OneDrive\Desktop\Pastas\LapTimeSimulator_CopaTruck\data"
os.makedirs(DATA_PATH, exist_ok=True)
file_path = os.path.join(DATA_PATH, "Silverstone.hdf5")

# Exemplo de dados da pista
x = np.array([0, 50, 100, 150, 180, 210, 250, 300, 350, 370, 390, 420, 440, 460, 480, 510, 540, 570, 600, 630])
y = np.array([0,  0,  0,  30, 60, 90, 120, 150, 160, 150, 130, 110, 90,  60,  30,  10,  0,   0,  0,  0])

track_points = np.column_stack((x, y))

with h5py.File(file_path, 'w') as f:
    f.create_dataset('track_points', data=track_points)

print(f"Pista Silverstone criada em {file_path}")
