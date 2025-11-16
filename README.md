# CPR-ROSbot2-Pure-Pursuit

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

Para comprobar que se os mueve el robot correctamente

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```