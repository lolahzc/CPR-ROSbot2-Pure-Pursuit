# CPR-ROSbot2-Pure-Pursuit
🚗 **Trayectoria con Gazebo — Pure Pursuit**

Este proyecto utiliza el algoritmo **Pure Pursuit** para que el robot siga una trayectoria en Gazebo.

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


## Paquete creado


**pure_pursuit_node.cpp**
- Es el responsable del seguimiento de la trayectoria y de la evitación de obstáculos. Las funcionalidades que tiene actualmente son:
    - Controlador pure pursuit
    - Lookahead distance variable según la distancia a la trayectoria
    - Velocidad lineal y angular variable

**route_publisher.cpp**
- Publicador de trayectorias, se crean unos waypoints y se interpola cúbicamente entre ellos

Se ha creado una configuración de Rvizz para visualizar el comportamiento del robot

---

## Ejecución del robot siguiendo la trayectoria

Compilar el proyecto (desde proy_cpr)

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
cp -r home/husarion/CPR-ROSbot2-Pure-Pursuit/pure_pursuit_controller home/husarion/proy_cpr/src
```

Después hay que compilar, pero para no hacerlo todo se puede hacer :
```bash
cd home/husarion/proy_cpr
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
