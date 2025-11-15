# arquivo: vehicle_params.py

from dataclasses import dataclass

@dataclass
class MassaGeometria:
    massa: float
    entre_eixos: float
    largura: float
    altura_cg: float
    dist_cg_dianteira: float

@dataclass
class Suspensao:
    rigidez_molas_frente: float
    rigidez_molas_tras: float
    damping_frente: float
    damping_tras: float

@dataclass
class Pneu:
    rigidez_lateral: float
    coef_atrito: float
    raio_efetivo: float

@dataclass
class Motor:
    potencia_kw: float
    torque_nm: float
    rpm_max: float

@dataclass
class Transmissao:
    num_marchas: int
    relacao_final: float

@dataclass
class Freio:
    area_pistao_cm2: float
    coef_friccao: float
    balance_freio_dianteira: float

@dataclass
class Aerodinamica:
    cx: float
    area_frontal_m2: float
    downforce_n_200kmh: float


def parametros_bicicleta_2dof_copa_truck():
    # Exemplo base para inicialização or referência
    
    massa_geom = MassaGeometria(
        massa=5000.0,
        entre_eixos=4.4,
        largura=2.55,
        altura_cg=1.1,
        dist_cg_dianteira=2.1
    )
    
    suspensao = Suspensao(
        rigidez_molas_frente=800000,
        rigidez_molas_tras=900000,
        damping_frente=20000,
        damping_tras=25000
    )
    
    pneu = Pneu(rigidez_lateral=120000, coef_atrito=1.1, raio_efetivo=0.65)
    
    motor = Motor(potencia_kw=600, torque_nm=3700, rpm_max=2800)
    
    transmissao = Transmissao(num_marchas=12, relacao_final=5.33)
    
    freio = Freio(area_pistao_cm2=55.0, coef_friccao=0.37, balance_freio_dianteira=58)
    
    aerodinamica = Aerodinamica(cx=0.85, area_frontal_m2=8.7, downforce_n_200kmh=2100)
    
    return {
        "massa_geom": massa_geom,
        "suspensao": suspensao,
        "pneu": pneu,
        "motor": motor,
        "transmissao": transmissao,
        "freio": freio,
        "aerodinamica": aerodinamica
    }
