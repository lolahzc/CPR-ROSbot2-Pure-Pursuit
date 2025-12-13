# 🚗 CPR-ROSbot2-Pure-Pursuit: Seguimiento de Trayectorias y Evasión de Obstáculos

Este proyecto implementa el algoritmo **Pure Pursuit** para el robot **Husarion ROSbot 2.0** en el entorno de simulación **Gazebo** (ROS 2). Se incluye una **máquina de estados** robusta para gestionar la evasión de obstáculos.

## Funcionalidades y Mejoras

### `pure_pursuit_controller.cpp` (Lógica de Control)

El controlador principal ha sido extendido para incluir la lógica de evasión de obstáculos, manteniendo el seguimiento de la trayectoria Pure Pursuit como modo por defecto.

#### Máquina de Estados para Evasión (`enum AvoidanceState`)

| Estado | Descripción | Transición |
| :--- | :--- | :--- |
| `NORMAL` | El robot sigue la trayectoria Pure Pursuit. | Si detecta obstáculo $\rightarrow$ `STOPPED` |
| `STOPPED` | Detención del robot. Decisión de giro. | Después de decidir la dirección $\rightarrow$ `SEARCHING_LEFT/RIGHT` |
| `SEARCHING_LEFT/RIGHT` | El robot pivota sobre sí mismo buscando un espacio libre. | Si el frente está libre y seguro $\rightarrow$ `NORMAL` |

#### Funciones de Evasión Implementadas

* **`scanCallback()`**: Procesa las lecturas del sensor LIDAR con un **filtro de ruido** (ignora lecturas menores a **0.20m**, para evitar el chasis) y un ajuste de **$180^\circ$** (corrigiendo la orientación del sensor).
* **`checkFreeSpace()`**: Verifica si hay espacio frontal (`> min_free_distance`) para reanudar la marcha.
* **`findGapSide()`**: Determina hacia dónde pivotar (Izquierda o Derecha) en modo `STOPPED`.
* **`findClosestIndexForward()`**: Localiza el punto de la trayectoria más cercano y "adelantado" para reenganchar el camino correctamente.

#### Parámetros Clave (Configurables)

| Parámetro | Valor Defecto | Descripción |
| :--- | :--- | :--- |
| `obstacle_distance_threshold` | 0.9m | Distancia máxima para considerar un objeto como obstáculo y detener la marcha. |
| `scan_fov_degrees` | 50° | Campo de visión frontal del LIDAR utilizado para la detección. |
| `min_free_distance` | 1.2m | Distancia libre mínima requerida para reanudar la marcha. |
| `search_angular_vel` | 0.45 rad/s | Velocidad angular utilizada para pivotar y buscar un hueco libre. |

---

### `route_publisher.cpp` (Gestión de Trayectorias)

El `route_publisher` define, interpole (mediante **Splines Cúbicos Naturales**) y publica las trayectorias.

| Ruta (`selected_route`) | Nombre | Descripción |
| :--- | :--- | :--- |
| 1 | Original “Lola” | Ruta inicial de prueba. |
| 2 | Línea recta | Avanza en línea recta. |
| 3 | Zigzag | Movimiento en zigzag. |
| 4 | Ocho | Trayectoria en forma de ocho. |
| 5 | Espiral | Movimiento en espiral. |
| 6 | Curvas amplias | Curvas amplias que ocupan todo el plano. |
| 7 | Curvas cerradas | Curvas cerradas de distinto radio. |
| **8** | **Curva obstáculo** | **Ruta creada para la prueba de evasión (rodeando la pared).** |

---

## Ejecución y Uso

### 1. Compilación del Proyecto

Ejecutar la compilación (desde el directorio raíz del espacio de trabajo, ej. `proy_cpr`):

```bash
colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
