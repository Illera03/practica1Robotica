# Práctica 1 de Robótica: extracción de datos cinemáticos del UR3e

Nodo de ROS 2 que mueve el UR3e con MoveIt en tres trayectorias (joint, lineal y circular), extrae la posición, la velocidad y la aceleración de cada articulación y las guarda en un CSV por trayectoria. Los datos se publican además en un mensaje propio, `DatosCinematicos`.

**Autores:** Jorge Illera Rivera, Iker Argulo

## Estructura del repositorio

```
practica1Robotica/
├── README.md
├── resultados/                          # CSV obtenidos con la versión final del código
│   ├── datos_joint.csv
│   ├── datos_lineal.csv
│   └── datos_circular.csv
├── practica_1/                          # Paquete ament_python con el código
│   ├── package.xml
│   ├── setup.py                         # Define el ejecutable extractor_cinematico
│   ├── setup.cfg
│   ├── resource/
│   ├── test/                            # Tests generados por ros2 pkg create
│   └── practica_1/
│       ├── extractor_cinematico.py      # Código de la práctica
│       └── control_robot.py             # Clase ControlRobot proporcionada en clase (sin cambios)
└── practica_1_interfaces/               # Paquete ament_cmake con el mensaje personalizado
    ├── CMakeLists.txt
    ├── package.xml
    └── msg/
        └── DatosCinematicos.msg
```

El mensaje va en un paquete aparte porque los `.msg` solo se pueden generar desde un paquete `ament_cmake`.

## Dónde está cada requisito

Todo el código propio está en [`practica_1/practica_1/extractor_cinematico.py`](practica_1/practica_1/extractor_cinematico.py).

| Requisito | Dónde |
|---|---|
| Paquete propio de ROS 2 | [`practica_1/`](practica_1/) y [`practica_1_interfaces/`](practica_1_interfaces/) |
| **Mínimo 1.** Suscriptor a `/joint_states` | Clase `ExtractorCinematico`: la suscripción se crea en `__init__` y cada mensaje se procesa en `_callback_articulaciones` |
| **Mínimo 2.** Trayectorias joint, lineal y circular | Clase `SecuenciaTrayectorias`: métodos `_trayectoria_joint`, `_trayectoria_lineal` y `_trayectoria_circular` (los puntos del círculo se calculan en `_poses_circulo`) |
| **Mínimo 3.** Posición, velocidad y aceleración con marca de tiempo, separadas por trayectoria | La aceleración se calcula en `_callback_articulaciones`; cada trayectoria se etiqueta con `registrando()` y `guardar_csv()` escribe un CSV por trayectoria. Resultados en [`resultados/`](resultados/) |
| **Opcional 1.** Mensaje personalizado | Definición en [`practica_1_interfaces/msg/DatosCinematicos.msg`](practica_1_interfaces/msg/DatosCinematicos.msg); se publica en `/datos_cinematicos` desde `ExtractorCinematico._publicar` |
| **Opcional 2.** Ingesta solo con el robot en movimiento | `ExtractorCinematico._hay_movimiento`, con el umbral `UMBRAL_VELOCIDAD` |

## Funcionamiento

`ExtractorCinematico` recibe cada mensaje de `/joint_states` y calcula la aceleración por diferencias finitas:

```
aceleración = (velocidad_actual - velocidad_anterior) / (t_actual - t_anterior)
```

Solo guarda la muestra si hay una trayectoria activa y alguna articulación supera `UMBRAL_VELOCIDAD` (1e-3 rad/s). La velocidad anterior se actualiza aunque el robot esté parado, para que la primera aceleración de cada movimiento no se calcule con datos de varios segundos antes.

`SecuenciaTrayectorias` ejecuta las trayectorias en este orden:

1. **Joint:** va a la posición `INICIO` (todas las articulaciones a 0) sin registrar y graba el movimiento en espacio articular hasta `HOME`.
2. **Lineal:** desde `HOME`, baja el efector 10 cm y graba tres tramos rectos de 10 cm: subida en Z y avances en Y y en X.
3. **Circular:** desde `HOME`, baja el efector 10 cm y graba una vuelta de 3 cm de radio en el plano XY.

Al terminar, el robot vuelve a `HOME` y se escriben los CSV, también si alguna trayectoria ha fallado.

### Formato de los CSV

Una fila por articulación y por instante:

```
tiempo,articulacion,posicion,velocidad,aceleracion
```

`tiempo` en segundos (marca de tiempo de `/joint_states`), `posicion` en rad, `velocidad` en rad/s y `aceleracion` en rad/s².

### Mensaje `DatosCinematicos`

Contiene todas las articulaciones de un instante en arrays paralelos (el elemento `i` de cada array corresponde a `nombres[i]`):

```
std_msgs/Header header
string trayectoria
string[] nombres
float64[] posiciones
float64[] velocidades
float64[] aceleraciones
```

## Ejecución

Con la simulación del UR3e y MoveIt arrancadas, se compila desde la raíz del workspace:

```bash
cd ~/ros_workspace
colcon build --symlink-install --packages-select practica_1_interfaces practica_1
source install/setup.bash
```

Los CSV se generan en el directorio desde el que se lanza el nodo. Para que sobrescriban los de `resultados/`:

```bash
cd ~/ros_workspace/src/practica1Robotica/resultados
ros2 run practica_1 extractor_cinematico
```

Para ver el mensaje personalizado mientras se mueve el robot, en otra terminal:

```bash
source ~/ros_workspace/install/setup.bash
ros2 topic echo /datos_cinematicos
```
