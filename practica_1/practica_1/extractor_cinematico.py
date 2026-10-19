import rclpy
import threading
import copy
import csv
from control_robot import ControlRobot
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dataclasses import dataclass
from math import pi, sin, cos


@dataclass
class MuestraCinematica:
    """Medida cinemática de una articulación en un instante concreto."""
    tiempo: float
    articulacion: str
    posicion: float
    velocidad: float
    aceleracion: float


class ExtractorCinematico(Node):
    """Nodo suscriptor que extrae posición, velocidad y aceleración del UR3e."""

    def __init__(self) -> None:
        """
        Inicializa el nodo y se suscribe a /joint_states.
        """
        super().__init__("extractor_cinematico")
        # Nos suscribimos a /joint_states igual que hace control_robot.py
        self._suscriptor = self.create_subscription(
            JointState,
            "/joint_states",
            self._callback_articulaciones,
            10
        )
        # Variables para guardar el estado anterior y poder calcular la aceleración
        self._tiempo_anterior: float | None = None
        self._velocidades_anteriores: dict[str, float] = {}
        self.trayectoria_actual: str | None = None 
        self._muestras_por_trayectoria: dict[str, list[MuestraCinematica]] = {} # Diccionario de la trayectoria para la lista de muestras

    def _callback_articulaciones(self, msg: JointState) -> None:
        """
        Se ejecuta automáticamente cada vez que el robot publica su estado.
        """
        tiempo_actual: float = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        if self._tiempo_anterior is None: # La primera vez que recibimos un mensaje, no podemos calcular la aceleración
            self._tiempo_anterior = tiempo_actual
            self._velocidades_anteriores = dict(zip(msg.name, msg.velocity))
            return
        
        # 1. Calculamos el tiempo transcurrido (Delta t)
        dt = tiempo_actual - self._tiempo_anterior
        if dt <= 0:
            return

        # 2. Recorremos las 3 listas a la vez
        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            # Buscamos la velocidad que tenia la articulacion en el paso anterior
            velocidad_anterior = self._velocidades_anteriores.get(name, 0.0)

            # Calculamos la aceleración y guardamos los datos
            aceleracion = (velocity - velocidad_anterior)/ dt
            muestra = MuestraCinematica(
                tiempo=tiempo_actual,
                articulacion=name,
                posicion=position,
                velocidad=velocity,
                aceleracion=aceleracion,
            )
            if self.trayectoria_actual is not None:
                self._muestras_por_trayectoria.setdefault(self.trayectoria_actual, []).append(muestra)

            # Actualizamos _velocidades_anteriores
            self._velocidades_anteriores[name] = velocity

        # 3. Actualizamos _tiempo_anterior
        self._tiempo_anterior = tiempo_actual
        
HOME = [0, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0]
INICIO = [0, 0, 0, 0, 0, 0]

def ir_a_home(control: ControlRobot) -> None:
    """
    Mueve el robot a la posición inicial.
    """
    if not control.mover_articulaciones(HOME):
        print("MoveIt no pudo ir a la posición home")

def bajar_efector(control: ControlRobot) -> None:
    """
    Baja el efector final 10 cm en el eje Z.
    """
    pose = control.pose_actual()
    pose.position.z -= 0.1
    if not control.mover_a_pose(pose):
        print("MoveIt no pudo bajar el efector final")


def main() -> None:
    """
    Función principal que ejecuta la práctica.
    """
    rclpy.init()
    control = ControlRobot()
    extractor = ExtractorCinematico()

    ejecutor_extractor = rclpy.executors.SingleThreadedExecutor()
    ejecutor_extractor.add_node(extractor)
    hilo_extractor = threading.Thread(target=ejecutor_extractor.spin, daemon=True)
    hilo_extractor.start()

    try:
        # --- Trayectoria joint ---
        extractor.trayectoria_actual = "joint"
        pi_medios = pi / 2
        if not control.mover_articulaciones(INICIO):
            print("MoveIt no pudo mover las articulaciones")
        
        # Movimiento grabado: INICIO -> HOME
        if not control.mover_articulaciones(HOME):
            print("MoveIt no pudo mover las articulaciones")
        extractor.trayectoria_actual = None


        # --- Trayectoria lineal --- 
        ir_a_home(control)
        bajar_efector(control)
        poses = []
        # Pose 1
        pose_actual = control.pose_actual()
        pose_actual.position.z += 0.1
        poses.append(copy.deepcopy(pose_actual))
        # Pose 2
        pose_actual.position.y += 0.1
        poses.append(copy.deepcopy(pose_actual))
        # Pose 3 
        pose_actual.position.x += 0.1
        poses.append(copy.deepcopy(pose_actual))

        # Mover
        extractor.trayectoria_actual = "lineal"
        if not control.mover_trayectoria(poses):
            print("MoveIt no pudo planificar o ejecutar la trayectoria lineal")
        extractor.trayectoria_actual = None
        

        # --- Trayectoria circular ---
        ir_a_home(control)
        bajar_efector(control)
        pose_inicial = control.pose_actual()
        radio = 0.03
        poses = []  # lista nueva, sin los waypoints de la trayectoria lineal

        # Círculo en el plano XY que empieza y acaba en la pose inicial 
        for i in range(0, 361, 10):
            angulo = i * (pi / 180)
            pose = copy.deepcopy(pose_inicial)
            pose.position.x = pose_inicial.position.x + radio * (1 - cos(angulo))
            pose.position.y = pose_inicial.position.y + radio * sin(angulo)
            poses.append(pose)

        extractor.trayectoria_actual = "circular"
        if not control.mover_trayectoria(poses):
            print("MoveIt no pudo planificar o ejecutar la trayectoria circular")
        extractor.trayectoria_actual = None
        
        ir_a_home(control)
            
    finally:
        control.ejecutor.shutdown()
        ejecutor_extractor.shutdown()
        extractor.destroy_node()
        control.destroy_node()
        rclpy.try_shutdown()

    # Guardar un CSV por cada trayectoria registrada
    for trayectoria, muestras in extractor._muestras_por_trayectoria.items():
        with open(f"datos_{trayectoria}.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["tiempo", "articulacion", "posicion", "velocidad", "aceleracion"])
            for m in muestras:
                writer.writerow([m.tiempo, m.articulacion, m.posicion, m.velocidad, m.aceleracion])


if __name__ == '__main__':
    main()