# CPR-ROSbot2-Pure-Pursuit: Seguimiento de Trayectorias y Evasión de Obstáculos

Este proyecto implementa el algoritmo **Pure Pursuit** para el robot **Husarion ROSbot 2.0** en el entorno de simulación **Gazebo** (ROS 2). Se incluye una **máquina de estados** robusta para gestionar la evasión de obstáculos.

Aviso: puede haber diferencias de funcionamiento de los códigos según el ordenador en el que se ejecute. Si al hacer la trayectoria 6 con obstáculos se choca el vehículo, incrementar detour_offset a 1.5 metros.
  
---

## Instrucciones instalación
He hecho la instalación oficial de los paquetes y lo he subido todo en local sin ningún paquete embebido para mayor comodidad al trabajar juntos en paralelo.

Estos paquetes son los que se instalan al poner al variable de Husarion en **simulation**. No sé que se instalaría al poner la variable en **hardware**, que asumo que lo necesitaríamos para probar el código en el robot físico.
```
export HUSARION_ROS_BUILD_TYPE=simulation
```

#### Actualizar e instalar dependencias (deberíais tenerlas de las prácticas).
```
sudo apt-get update
sudo apt-get install -y python3-pip ros-dev-tools stm32flash
```

#### Clonar el repositorio.
```
mkdir proy_cpr
cd proy_cpr
mkdir src
cd src
git clone git@github.com:lolahzc/CPR-ROSbot2-Pure-Pursuit.git
```

#### Volver a carpeta raíz del workspace (proy_cpr) y compilar.
```
colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Creo que da un warning de uno de los paquetes por movidas de cosas deprecadas pero ignorarlo.

#### Lanzar repositorio
```
source install/setup.bash
```
Os recomiendo añadiros esto en el .bashrc para no tener que hacerlo cada vez que queráis trabajar en el proyecto.

```
ros2 launch rosbot_gazebo simulation.launch.py robot_model:=rosbot
```

Si se quiere lanzar un mundo sin obstáculos, en husarion_gz_worlds/launch/gz_sim.launch.py cambiar la línea:
```
[FindPackageShare("husarion_gz_worlds"), "worlds", "slam.sdf"]
```
Por:
```
[FindPackageShare("husarion_gz_worlds"), "worlds", "empty_with_plugins.sdf"]
```

El `route_publisher` define, interpole (mediante **Splines Cúbicos Naturales**) y publica las trayectorias.

---

## Paquete creado

**pure_pursuit_node.cpp**
- Es el responsable del seguimiento de la trayectoria y de la evitación de obstáculos. Las funcionalidades que tiene actualmente son:
    - Controlador pure pursuit
    - Lookahead distance variable según la distancia a la trayectoria
    - Control de velocidad linear (depende de cte y curvatura) y angular (depende de lookahead)
    - Datalogger

**route_publisher.cpp**
- Publicador de trayectorias, se crean unos waypoints y se interpola cúbicamente entre ellos
- Publicador de los markers para rvizz

**pure_pursuit.launch.py**
- Configuración de parámetros:
    - Velocidad lineal máxima
    - Velocidad angular máxima
    - Lookahead distance mínima
    - Lookahead distance máxima
    - Lookahead distance gamma
    - Cantidad de puntos interpolados entre waypoints

Se ha creado una configuración de Rvizz para visualizar el comportamiento del robot

Se ha creado un código de matlab ejecutable en Octave para analizar el comportamiento del robot, tanto en simulación como en la vida real. Su nombre es analyzer.m

Se han empleado grabaciones de rosbag que se han analizado con foxglove para comprobar el comportamiento del robot en la vida real

### `pure_pursuit_controller.cpp` (Lógica de Control)

El controlador principal ha sido extendido para incluir la lógica de evasión de obstáculos, manteniendo el seguimiento de la trayectoria Pure Pursuit como modo por defecto.


## Algoritmo de Evasión de Obstáculos

Este módulo implementa un algoritmo de navegación reactiva basado en LIDAR que gestiona la seguridad del robot y la generación de trayectorias alternativas mediante curvas de Bézier cuadráticas.

### Flujo de Funcionamiento

#### 1. Escaneo Adaptativo (LIDAR)
El sistema ajusta el campo de visión (FOV) según el contexto:
* **Navegación Normal:** FOV de `60°` (±30°).
* **Análisis de Desvío:** FOV ampliado a `190°` (±95°) para buscar rutas alternativas.

#### 2. Máquina de Estados (Proximidad)
Se evalúa la distancia mínima (`min_distance`) detectada para determinar el comportamiento:

| Distancia ($d$) | Estado | Acción |
| :--- | :--- | :--- |
| $d < 0.25m$ | 🔴 **EMERGENCY** | Parada inmediata (Velocidad 0%). |
| $d < 0.50m$ | 🟠 **OBSTACLE** | Inicio de maniobra de evasión (Velocidad 60%). |
| Resto | 🟢 **NORMAL** | Seguimiento de ruta estándar (Velocidad 100%). |

#### 3. Lógica de Evasión
Si se detecta un obstáculo, el sistema decide la dirección óptima calculando el "peso" de los obstáculos en cada hemisferio:
* `peso = Σ (1.0 / distancia_i)`
* Si `peso_izq < peso_der` → Desvío a la **Izquierda**.
* Si `peso_izq > peso_der` → Desvío a la **Derecha**.

#### 4. Generación de Trayectoria (Curvas de Bézier)
Se genera una ruta suave utilizando una curva de Bézier cuadrática definida por tres puntos de control $(P_0, P_1, P_2)$:

* **$P_0$ (Inicio):** Posición actual del robot.
* **$P_1$ (Ápex):** Punto de máximo desplazamiento lateral (calculado con `forward_offset` y `detour_offset`).
* **$P_2$ (Fin):** Punto de reincorporación a la ruta original (`rejoin_distance`).

---

## Ejecución y Uso

### 1. Compilación del Proyecto

Ejecutar la compilación (desde el directorio raíz del espacio de trabajo, ej. `proy_cpr`):

```bash
colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Cargar el entorno y lanzar el nodo Pure Pursuit (despues de haber compilado):

```bash
source install/setup.bash
```

Lanzar la simulacion. Se recomienda hacerlo en proy_cpr/src/CPR-ROSBOT2-PURE-PURSUIT para guardar los datos del log en la misma carpeta que el código de matlab
```bash
ros2 launch rosbot_gazebo simulation.launch.py robot_model:=rosbot
```

En otro terminal lanzar el control con la ruta seleccionada
```bash
ros2 launch pure_pursuit_controller pure_pursuit.launch.py selected_route:=<número de la ruta>
```

| Número | Nombre de la ruta | Descripción breve      |
| ------ | ----------------- | ---------------------- |
| 1      | Original “Lola”   | Ruta inicial de prueba |
| 2      | Línea recta       | Avanza en línea recta  |
| 3      | Zigzag            | Movimiento en zigzag   |
| 4      | Ocho              | Forma de 8             |
| 5      | Espiral           | Movimiento en espiral  |
| 6      | Curvas amplias    | Curvas amplias que ocupan todo el plano  |
| 7      | Curvas cerradas   | Curvas cerradas de distinto radio  |


### Finalización de la trayectoria

Cuando el robot complete el recorrido, verás:

```text
[pure_pursuit_node-1] [INFO] [...] [pure_pursuit_node]: Goal reached!
```

### Selección de ruta en el momento mediante la implementación de un parámetro dinámico
Para ello se ha creado un nuevo tópico denominado changeroute.
Para usarlo, hay que abrir un nuevo terminal, hacer source y enviar el siguiente mensaje:
```bash
ros2 topic pub /change_route std_msgs/msg/Int32 "{data: 4}"
```
en el topico de changeroute siendo 4 el número de la ruta seleccionada.

---

## Ejecución alternativa del robot - Automatizado Completamente

Para agilizar el proceso de pruebas se ha creado el script `lanzar_simulacion.sh`. Este script unifica todo el flujo de trabajo en una sola terminal: lanza Gazebo, inicia el controlador Pure Pursuit y, al finalizar, ejecuta automáticamente el análisis de datos.

### Funcionalidades del script
1. **Entorno:** Carga automáticamente `setup.bash` de ROS 2 y del workspace.
2. **Ejecución:** Lanza Gazebo y el nodo de Pure Pursuit en segundo plano.
3. **Gestión de procesos:** Al detener la simulación, se encarga de matar (kill) todos los procesos relacionados para evitar nodos "zombies".
4. **Análisis automático:** Tras cerrar la simulación, lanza inmediatamente `analyzer_automatico.m` en Octave para mostrar las gráficas del experimento recién concluido.

### Uso
Primero de todo, tanto `lanzar_simulacion.sh` como `analyzer_automatico.m` tienen que estar en la carpera raíz del proyecto, `/proy_cpr`. Ya que el script realiza los `source` necesarios. Por eso mismo, al lanzar el script desde esa carpeta los logs se guardarán también en dicha ubicación, por lo que el script de análisis también tiene que estar.

Asegúrate de dar permisos de ejecución al script (solo la primera vez):
```
chmod +x lanzar_simulacion.sh
```
Después, basta con realizar:
```
./lanzar_simulacion.sh <numero_de_ruta>
```
Esto lanzará todos los launch necesarios del simulador y del nodo de pure_pursuit, asegurándose de esperar un tiempo para que el simulador se lance por completo antes de lanzar el script de control. Cuando termine la simulación, habrá que realizar **Ctrl + C**, para acabar prueba y automaticamente se abrirán las gráficas resultantes del log de la prueba que acaba de terminar. Para cerrar, bastará con pulsar **Enter** en terminal.

Todas estas instrucciones se indican por terminal al lanzar el script. 

---

## Ejecución del robot real

Para trabajar con el robot real el primer paso es realizar la conexión por ssh con el mismo. Actualmente la configuración de red que tiene es la siguiente:

Red: ROB_6
Contraseña: robotica

Tras conectarlo (preferiblemente siempre al mismo dispositivo para que no cambie la dirección IP, ya que se necesitaría una pantalla para cambiar el archivo de configuración de red del robot), en el portátil se ejecuta el siguiente comando:
```bash
ssh husarion@10.55.55.163
```
Contraseña: husarion

El siguiente paso es comprobar la versión del proyecto que está subida en el robot. Para ello, en la carpeta home/husarion está clonado este repositorio y sólo habría que copiar los archivos de esta carpeta a la home/husarion/proy_cpr/src. Esto se hace con:
```bash
cp -r ~/CPR-ROSbot2-Pure-Pursuit/pure_pursuit_controller ~/proy_cpr/src
```

Después hay que compilar, pero para no hacerlo todo se puede hacer :
```bash
cd home/husarion/proy_cpr
```
```bash
colcon build --packages-select pure_pursuit_controller
```

Una vez que acabe, se vuelve a hacer source:
```bash
source install/setup.bash
```

Tras esto, ya se pueden mandar los comandos necesarios. Hacen falta dos terminales, una para cada comando:
```bash
ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_baudrate:=256000
```
```bash
ros2 launch pure_pursuit_controller pure_pursuit.launch.py selected_route:=<número de la ruta>
```

Además, si se quiere crear un rosbag de lo que está pasando para analizarlo posteriormente en foxglove:
```bash
ros2 bag record -a -o <nombre del archivo>
```

Para copiar los logs del robot al portátil para analizarlos hay que hacer, en nuestro portátil (cambiando la fecha del log para seleccionar el que se quiere):
```bash
scp husarion@10.13.222.163:~/proy_cpr/ruta_defecto_2026-01-16_10-25-36.csv .
```
Para resetear la posición del robot:
```bash
ros2 topic pub --once /set_pose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'odom'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```
