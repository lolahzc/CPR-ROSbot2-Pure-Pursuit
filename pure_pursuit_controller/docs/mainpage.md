# Documentación del Proyecto: Navegación Autónoma con Pure Pursuit {#mainpage}

## 1. Introducción
Bienvenido a la documentación técnica del sistema de navegación para robot móvil diferencial. Este proyecto implementa una solución completa en **ROS 2** que permite a un robot seguir trayectorias complejas y suavizadas, con capacidad de reacción ante obstáculos imprevistos.

El sistema se divide en una arquitectura modular de dos nodos principales: un **Generador de Rutas**  y un **Controlador de Trayectoria**.

@image html robot_pure_pursuit.png "Esquema conceptual: Robot siguiendo la ruta con Pure Pursuit" width=600px

---

## 2. Arquitectura del Sistema

El flujo de información funciona de la siguiente manera:

1.  **Selección de Ruta:** El nodo `RoutePublisher` selecciona una ruta (ej. zigzag, espiral, circuito) y genera una trayectoria suave.
2.  **Comunicación:** La ruta se publica en el tópico `/waypoints_path`.
3.  **Control:** El nodo `PurePursuitNode` recibe la ruta, calcula la velocidad necesaria y evita obstáculos si aparecen.

### Diagrama de Nodos
* **[RoutePublisher]** ---> `/waypoints_path` ---> **[PurePursuitNode]** ---> `/cmd_vel`

---

## 3. Descripción de los Módulos

### A. Nodo Publicador de Rutas (Planificador)
Este nodo es el encargado de la "inteligencia global" de la ruta. No mueve el robot, solo le dice por dónde ir.

* **Clase Principal:** @ref RoutePublisher
* **Funcionalidades Clave:**
    * **Interpolación:** Utiliza **Splines Cúbicos Naturales** para convertir una lista de puntos dispersos en una curva suave y continua.
    * **Visualización:** Publica marcadores (`visualization_msgs::msg::MarkerArray`) para ver la ruta y el objetivo en RViz.
    * **Gestión de Misiones:** Controla si la ruta debe repetirse en bucle o finalizar.

### B. Nodo Controlador (Pure Pursuit + Evasión)
Este es el nodo que interactúa con los motores y sensores del robot. Implementa el algoritmo de seguimiento y una máquina de estados para la seguridad.

* **Clase Principal:** @ref PurePursuitNode
* **Algoritmos:**
    * **Pure Pursuit:** Calcula la curvatura necesaria para alcanzar un punto "Lookahead" a una distancia dinámica LHD.
    * **Smart Start:** Al recibir una ruta, busca automáticamente el punto más cercano al robot para iniciar el seguimiento.
    * **Evasión de Obstáculos:** Utiliza LIDAR para detectar colisiones inminentes.
* **Máquina de Estados:**
    1.  `NORMAL`: Seguimiento de ruta.
    2.  `STOPPED`: Obstáculo detectado.
    3.  `SEARCHING`: Giro sobre el propio eje para buscar espacio libre.

---

## 4. Referencias

| Tópico | Tipo de Mensaje | Dirección | Descripción |
| :--- | :--- | :--- | :--- |
| `/waypoints_path` | `geometry_msgs/msg/PoseArray` | Pub -> Sub | Lista de puntos de la ruta suavizada. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Pub -> Sub | El objetivo final del segmento actual. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Sensor -> Sub | Datos del LIDAR para evasión. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Pub -> Robot | Comandos de velocidad lineal y angular. |
| `/goal_reached` | `std_msgs/msg/Bool` | Sub <- Pub | Confirmación de llegada al destino. |

---
**Autores:** Pedro Cabello Pulido | Gabriela Cano Azuaga  | Lola Hernández Canizares | Almudena Jin | Lucía Pérez Guerrero 
**Fecha:** 2026