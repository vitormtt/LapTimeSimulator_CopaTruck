"""
Módulo do Veículo (Bicycle 2-DOF) integrando Powertrain, Freios e Pneus modulares.
"""
import numpy as np

class BicycleVehicle2DOF:
    """
    Modelo dinâmico Bicycle 2-DOF (Yaw e Lateral).
    Utiliza arquitetura modular (engine, brakes, transmission, tires).
    """
    
    def __init__(self, mass: float, wheelbase: float, a: float, cg_height: float, izz: float, 
                 engine_sys, brake_sys, trans_sys, tire_sys):
        # Parâmetros de Inércia e Geometria
        self.mass = mass
        self.wheelbase = wheelbase
        self.a = a                  # Distância CG até eixo dianteiro
        self.b = wheelbase - a      # Distância CG até eixo traseiro
        self.cg_height = cg_height
        self.izz = izz              # Inércia polar Z
        
        # Módulos Dinâmicos
        self.engine = engine_sys
        self.brakes = brake_sys
        self.transmission = trans_sys
        self.tires = tire_sys
        
        # Estados do veículo
        self.vx = 0.1 # m/s (evitar divisões por zero)
        self.vy = 0.0
        self.yaw_rate = 0.0
        
    def _calculate_normal_loads(self, ax: float) -> tuple:
        """Calcula transferência de carga longitudinal"""
        weight = self.mass * 9.81
        delta_f = (self.mass * ax * self.cg_height) / self.wheelbase
        
        load_front = (weight * self.b / self.wheelbase) - delta_f
        load_rear = (weight * self.a / self.wheelbase) + delta_f
        return max(load_front, 0.0), max(load_rear, 0.0)

    def calculate_derivatives(self, throttle: float, brake_pedal: float, steering_angle: float, current_rpm: float):
        """
        Calcula as derivadas de estado (ax, ay, yaw_accel) com base nos inputs do piloto.
        """
        # 1. Transferência de carga estática/dinâmica (assumindo ax do t-1 ou zero para simplificar)
        Fz_f, Fz_r = self._calculate_normal_loads(ax=0.0) 
        
        # 2. Powertrain Longitudinal
        engine_torque = self.engine.get_max_torque(current_rpm) * throttle
        gear = self.transmission.select_optimal_gear(self.vx, wheel_radius_m=0.5)
        gear_ratio = self.transmission.get_total_ratio(gear)
        
        wheel_torque_drive = self.engine.get_wheel_torque(engine_torque, gear_ratio)
        fx_drive = wheel_torque_drive / 0.5 # Assumindo raio da roda 0.5m no momento
        
        # 3. Freios Longitudinal
        brake_forces = self.brakes.get_brake_force(brake_pedal, self.vx)
        fx_brake = -brake_forces['total'] # Força contrária
        
        # Força longitudinal total
        Fx_total = fx_drive + fx_brake
        ax = Fx_total / self.mass
        
        # 4. Dinâmica Lateral (Bicycle Model)
        # Slip angles
        alpha_f = steering_angle - np.arctan2((self.vy + self.a * self.yaw_rate), self.vx)
        alpha_r = -np.arctan2((self.vy - self.b * self.yaw_rate), self.vx)
        
        # Forças laterais usando os pneus modulares
        Fy_f = self.tires.get_lateral_force(alpha_f, Fz_f)
        Fy_r = self.tires.get_lateral_force(alpha_r, Fz_r)
        
        # 5. Acelerações (Derivadas)
        ay = (Fy_f * np.cos(steering_angle) + Fy_r) / self.mass - (self.vx * self.yaw_rate)
        yaw_accel = (self.a * Fy_f * np.cos(steering_angle) - self.b * Fy_r) / self.izz
        
        return {
            'ax': ax,
            'ay': ay,
            'yaw_accel': yaw_accel,
            'Fy_f': Fy_f,
            'Fy_r': Fy_r,
            'Fx_total': Fx_total
        }
