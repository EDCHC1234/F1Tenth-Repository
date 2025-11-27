### Autor: Daniel Chávez

# 🏎️ Autonomous Racing: Reactive Follow the Gap (ROS 2 Humble)

Este repositorio contiene la implementación de un controlador reactivo **Follow the Gap Method (FGM)** para un vehículo autónomo tipo F1Tenth, desarrollado en **ROS 2 Humble**.

El objetivo del nodo es navegar por una pista desconocida a alta velocidad, evitando obstáculos estáticos mediante el análisis de datos LiDAR en tiempo real, sin necesidad de un mapa previo (SLAM).

---

## Instalación del simulador F1Tenth

Primero nos aseguramos tener en nuestro equipo el simulador F1tenth, lo podemos clonar [aqui](https://github.com/widegonz/F1Tenth-Repository?tab=readme-ov-file), en este repositorio hay informacion importante del simulador.

### Modificación del Workspace

Nos derigimos a la siguiente dirección"F1Tenth-Repository/src/controllers/controllers". Aqui vamos a encontrar un archivo llamado gap_node.py el cual sera reemplazado por el archivo de  subido en este repositorio.

En la direcció "F1Tenth-Repository/src/controllers" se encuentra un archivo setup.py el cual debemos modificar para que se vea de la siguiente manera:

entry_points={
        'console_scripts': [
            'ttc_node = controllers.ttc_node:main',
            'pid_node = controllers.pid_node:main',
            'gap_node = controllers.gap_node:main',
            'purepursuit_node = controllers.purepursuit_node:main',
            'rrt_node = controllers.rrt_node:main',
            'mpc_node = controllers.mpc_node:main',
            'reactive_gap = controllers.gap_node:main',
        ],
 

### Cambio de mapas

En el link del repositorio de descarga del simulador F1Tenth se indica como hacer el cambio de mapa, por lo cual se recomienda pegar en la carpeta maps los archivos .png y .yaml de este repositorio y hacer el cambio en el archivo sim.yaml para que la linea se vea asi:

```bash
    map_path: '/home/"tu-usuario"/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch_map'
```

Con todos estos cambios hechos en nuestros wrokspace podemos realizar la simulacion para que nuestro vehiculo se mueva de froma autónoma



---

## 📂 Estructura del Código

El nodo principal se encuentra en el archivo `gap_node.py`. A continuación se describen sus componentes clave:

### Clase `ReactiveFollowGap`

* **`__init__`**: Inicializa los suscriptores (`/scan`, `/ego_racecar/odom`) y publicadores (`/drive`). Define parámetros ajustables como `bubble_radius`, `max_speed` y `fov_angle`.

* **`preprocess_lidar(ranges)`**: 
    * Limpia los datos crudos del sensor.
    * Aplica el recorte de campo de visión (FOV) para centrar la atención del robot.

* **`odom_callback(msg)`**: 
    * **Lap Timer Automático:** Detecta la posición inicial del robot al arrancar.
    * Calcula la distancia euclidiana al origen para contar vueltas y medir tiempos automáticamente. Incluye lógica de "debounce" para evitar falsos positivos al cruzar la meta.

* **`lidar_callback(scan)`**: 
    * Es el bucle principal de control (recorre todos los pasos del algoritmo mencionados arriba).
    * Calcula y aplica la "Safety Bubble".
    * Determina el ángulo de dirección y publica el mensaje `AckermannDriveStamped`.

---

## 🚀 Instrucciones de Ejecución

### 1. Requisitos Previos
* Simulador F1Tenth funcionando.
* Workspace de ROS 2 configurado.

### 2. Compilación
Nos segúramos de estar en la raíz de tu workspace:

```bash
cd ~/F1Tenth-Repository 
colcon build --packages-select controllers
source install/setup.bash
```
 
 Luego usamos el siguiente comando para cargar el simulador:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

En otra terminal, para mover el carrito de forma autónoma :

```bash
cd ~/F1Tenth-Repository 
ros2 run controllers gap_nodet
```

<p align="center">
  <img src="img/Simulation.png" />
  <br />
  <em> vehiculo autónomo simulado</em>
</p>
