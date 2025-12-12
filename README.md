# CPR-ROSbot2-Pure-Pursuit
🚗 **Trayectoria con Gazebo — Pure Pursuit**

Este proyecto utiliza el algoritmo **Pure Pursuit** para que el robot siga una trayectoria en Gazebo.

---

## Archivos modificados


**pure_pursuit.launch.py**
- Se ha modificado para que la llamada reciba el parámetro selected_route

**route_publisher.cpp**
- Se han creado trayectorias nuevas (anotaciones):
  1. Ruta original ✅ 
  2. Línea recta  ✅
  3. Zig-zag ❌ (cuando llega al final el robot sigue hacia la nada indefinidamente)
  4. Ocho  ❌ (solo se hace 1 de las ramas del 8)
  5. Espiral ✅
  6. Curvas abiertas en todo el plano en ambas direcciones ❌ (cuando llega al punto final se queda varias veces dando  vueltas y luego sigue moviéndose hasta que vuelve a detectar otro punto en medio de la trayectoria entrando así en bucle)
  7. Curvas cerradas ❌ No hace todos los checkpoints (puntos a gris) y al terminar la segunda vuelta se aleja y se va por completo del plano y las que son muy cerradas no lo hace con muchas precisión (ya todo depende de cuánto queramos afinarlo)

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

Lanzar la simulacion. Se recomienda hacerlo en proy_cpr>src>CPR-ROSBOT2-PURE-PURSUIT para guardar los datos del log en la misma carpeta que el código de matlab
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


## Control manual del robot y velocidades

mover el robot con el teleoperador
```bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

visualizar las velocidades enviadas al robot
```bash
ros2 topic echo /cmd_vel
```

### Selección de ruta en el momento mediante la implementación de un parámetro dinámico
Para ello se ha creado un nuevo tópico denominado changeroute.
Para usarlo, hay que abrir un nuevo terminal, hacer source y enviar el siguiente mensaje:

ros2 topic pub /change_route std_msgs/msg/Int32 "{data: 4}"

en el topico de changeroute siendo 4 el número de la ruta seleccionada.
