#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
from math import sqrt
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('follow_gap')

        # ---------------------------------------------------
        # CONFIGURACIÓN
        # ---------------------------------------------------
        self.max_speed = 3.0
        self.min_speed = 1.0
        
        # Ajustes del Gap Follower
        self.bubble_radius = 0.4        # Reducido un poco para evitar bloqueos en pasillos estrechos
        self.gap_threshold = 1.5        # Un hueco debe tener al menos esta profundidad (metros) para ser considerado
        self.preprocess_max_range = 4.0 # Rango máximo de visión
        
        # Historial para suavizado
        self.steering_history = [0.0] * 5

        # ---------------------------------------------------
        # VARIABLES DE CARRERA (LAP TIMER)
        # ---------------------------------------------------
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_pose_set = False
        
        self.lap_count = 0
        self.start_time = 0.0
        self.last_lap_time = 0.0
        
        # NUEVO: Bandera para obligar al robot a salir antes de contar vuelta
        self.has_left_start = False 

        # ---------------------------------------------------
        # SUCRIPCIONES
        # ---------------------------------------------------
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info("✅ Follow-The-Gap v2 (Logic Fix) Iniciado")

    # =======================================================
    # ODOMETRÍA Y CONTEO DE VUELTAS
    # =======================================================
    def odom_callback(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        # 1. Fijar punto de partida (solo la primera vez)
        if not self.start_pose_set:
            self.start_x = curr_x
            self.start_y = curr_y
            self.start_pose_set = True
            self.start_time = time.time()
            self.last_lap_time = time.time()
            self.get_logger().info(f"📍 Salida fijada en X={curr_x:.2f}, Y={curr_y:.2f}")
            return

        # 2. Calcular distancia al origen
        dist_to_start = sqrt((curr_x - self.start_x)**2 + (curr_y - self.start_y)**2)

        # 3. Lógica de "Armado": El robot debe alejarse > 2m para activar el sensor de meta
        if not self.has_left_start and dist_to_start > 2.0:
            self.has_left_start = True
            # self.get_logger().info("💨 El robot ha salido de la zona de inicio.")

        # 4. Detectar cruce de meta
        # Solo cuenta si YA salimos (has_left_start) y volvimos a entrar (< 1.0m)
        if self.has_left_start and dist_to_start < 1.0:
            current_time = time.time()
            lap_duration = current_time - self.last_lap_time
            
            # Evitar rebotes muy rápidos (mínimo 5 seg por vuelta)
            if lap_duration > 5.0:
                self.lap_count += 1
                self.get_logger().info(f"🏁 VUELTA {self.lap_count} — Tiempo: {lap_duration:.2f} s")
                
                # Resetear timers y bandera para la siguiente vuelta
                self.last_lap_time = current_time
                self.has_left_start = False # Debe volver a alejarse para contar la siguiente

    # =======================================================
    # PROCESAMIENTO LIDAR
    # =======================================================
    def preprocess_lidar(self, ranges):
        # Limpiar INF y NAN
        arr = np.array(ranges)
        arr = np.nan_to_num(arr, nan=0.0, posinf=self.preprocess_max_range)
        arr[arr > self.preprocess_max_range] = self.preprocess_max_range
        
        # IMPORTANTE: Asegurar que no tenemos ceros falsos (ruido) que paren el coche
        # Si un valor es 0.0, lo convertimos a algo pequeño pero no bloqueante, 
        # a menos que sea real. Aquí asumimos que 0.0 suele ser error de lectura o muy lejos.
        # Pero para FGM, mejor tratar 0.0 como "muy lejos" si es 'inf', o 'pared' si es collision.
        # En simulacion, a veces scan da 0.0 por error. Lo pasamos al maximo.
        # (Ajusta esto si tu lidar real da 0.0 al chocar)
        # arr[arr < 0.1] = self.preprocess_max_range 

        return arr

    def lidar_callback(self, scan):
        # --- 1. Preproceso ---
        ranges = self.preprocess_lidar(scan.ranges)
        
        # --- 2. Recorte de Visión (Field of View) ---
        # Solo nos importa lo que hay adelante (-90 a +90 grados aprox)
        # Asumiendo que el lidar tiene el indice 0 atrás o al lado, buscamos el centro.
        n = len(ranges)
        center_idx = n // 2
        field_of_view_idx = n // 3  # Tomamos el tercio central del scanner
        
        left_bound = center_idx - field_of_view_idx
        right_bound = center_idx + field_of_view_idx
        
        # Ponemos a 0 (pared) todo lo que está detrás para no distraernos
        # Nota: Copiamos el array para no modificar el original masivamente
        proc_ranges = np.zeros_like(ranges)
        # Solo copiamos la parte central
        proc_ranges[left_bound:right_bound] = ranges[left_bound:right_bound]

        # --- 3. Safety Bubble (Evitar Choques) ---
        # Buscamos el punto más cercano EN EL CAMPO DE VISIÓN
        # (Ignoramos ceros porque son áreas recortadas)
        masked_ranges = np.ma.masked_equal(proc_ranges, 0.0)
        
        if masked_ranges.count() == 0:
            # No veo nada válido, freno de emergencia
            self.publish_drive(0.0, 0.0)
            return

        min_dist = masked_ranges.min()
        closest_idx = np.argmin(masked_ranges)

        # FRENADO DE EMERGENCIA REAL
        if min_dist < 0.25: 
            self.get_logger().warn(f"🛑 Freno: Obstáculo a {min_dist:.2f}m")
            self.publish_drive(0.0, 0.0)
            return

        # Crear Burbuja (Inflar obstáculo)
        # Calcula radio en índices
        radius_idx = int(self.bubble_radius / (min_dist * scan.angle_increment))
        
        l_bub = max(0, closest_idx - radius_idx)
        r_bub = min(n, closest_idx + radius_idx)
        proc_ranges[l_bub:r_bub] = 0.0  # Marcar como obstáculo

        # --- 4. Encontrar Max Gap ---
        # Un gap es todo aquello que sea mayor al umbral
        gap_mask = proc_ranges > self.gap_threshold
        
        # Encontrar índices consecutivos de True
        zero_runs = np.diff(gap_mask.astype(int))
        starts = np.where(zero_runs == 1)[0] + 1
        ends = np.where(zero_runs == -1)[0] + 1
        
        # Ajuste de bordes
        if gap_mask[0]: starts = np.insert(starts, 0, 0)
        if gap_mask[-1]: ends = np.append(ends, n)

        if len(starts) == 0:
            self.get_logger().warn("⚠️ No se encontró hueco (Gap), girando a ciegas...")
            # Girar hacia donde haya un poquito más de espacio
            self.publish_drive(0.5, 0.3) # Giro lento a la izquierda por defecto
            return

        # Encontrar el gap más ancho
        durations = ends - starts
        best_gap_idx = np.argmax(durations)
        
        start_i = starts[best_gap_idx]
        end_i = ends[best_gap_idx]

        # --- 5. Escoger Mejor Punto (Goal) ---
        # Elegimos el punto más lejano dentro del gap seleccionado para ir más rápido
        # O el centro para ir más seguro. Usaremos un promedio ponderado.
        gap_slice = proc_ranges[start_i:end_i]
        
        # Opción A: Centro geométrico del hueco (Más seguro para no chocar bordes)
        best_idx = (start_i + end_i) // 2
        
        # Opción B (Avanzada): Punto más profundo del hueco
        # deep_idx = start_i + np.argmax(gap_slice)
        # best_idx = (best_idx + deep_idx) // 2  # Promedio entre centro y fondo

        # --- 6. Calcular Ángulo y Velocidad ---
        angle = (best_idx - center_idx) * scan.angle_increment

        # Suavizado
        self.steering_history.append(angle)
        self.steering_history.pop(0)
        smooth_angle = sum(self.steering_history) / len(self.steering_history)
        smooth_angle = np.clip(smooth_angle, -0.4, 0.4) # Limitar giro físico

        # Velocidad basada en el ángulo de giro
        # Si el ángulo es pequeño (recta), corre. Si es grande, frena.
        speed = self.max_speed if abs(smooth_angle) < 0.15 else self.min_speed
        
        self.publish_drive(speed, smooth_angle)

    def publish_drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(angle)
        self.drive_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
