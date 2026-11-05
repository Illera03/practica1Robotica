import rclpy
import threading
import copy
import csv
from practica_1.control_robot import ControlRobot
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dataclasses import dataclass
from math import pi, sin, cos
from practica_1_interfaces.msg import DatosCinematicos

TOPIC_DATOS_CINEMATICOS = "/datos_cinematicos"
# Por debajo de esta velocidad (rad/s) consideramos la articulación parada
UMBRAL_VELOCIDAD = 1e-3


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
        self._publicador = self.create_publisher(DatosCinematicos, TOPIC_DATOS_CINEMATICOS, 10)
        
    def _callback_articulaciones(self, msg: JointState) -> None:
        """
        Se ejecuta automáticamente cada vez que el robot publica su estado.
        """
        tiempo_actual: float = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        if self._tiempo_anterior is None: # La primera vez que recibimos un mensaje, no podemos calcular la aceleración
            self._tiempo_anterior = tiempo_actual
            self._velocidades_anteriores = dict(zip(msg.name, msg.velocity))
            return
        
        dt = tiempo_actual - self._tiempo_anterior
        if dt <= 0:
            return

        # Solo guardamos si hay trayectoria activa y alguna articulación se mueve
        trayectoria = self.trayectoria_actual
        en_movimiento = any(abs(v) > UMBRAL_VELOCIDAD for v in msg.velocity)
        guardar = trayectoria is not None and en_movimiento
        aceleraciones: list[float] = []

        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            velocidad_anterior = self._velocidades_anteriores.get(name, 0.0)
            aceleracion = (velocity - velocidad_anterior) / dt
            aceleraciones.append(aceleracion)
            if guardar:
                muestra = MuestraCinematica(
                    tiempo=tiempo_actual,
                    articulacion=name,
                    posicion=position,
                    velocidad=velocity,
                    aceleracion=aceleracion,
                )
                self._muestras_por_trayectoria.setdefault(trayectoria, []).append(muestra)
            # Se actualiza siempre, también en reposo, para que la primera
            # aceleración de cada movimiento use la velocidad inmediatamente anterior
            self._velocidades_anteriores[name] = velocity

        if guardar:
            self._publicar(msg, trayectoria, aceleraciones)

        self._tiempo_anterior = tiempo_actual

    def _publicar(self, msg: JointState, trayectoria: str, aceleraciones: list[float]) -> None:
        """Envía posición, velocidad y aceleración de todas las articulaciones en un solo mensaje."""
        datos = DatosCinematicos()
        datos.header = msg.header
        datos.trayectoria = trayectoria
        datos.nombres = list(msg.name)
        datos.posiciones = list(msg.position)
        datos.velocidades = list(msg.velocity)
        datos.aceleraciones = aceleraciones
        self._publicador.publish(datos)
        
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
        if not control.mover_articulaciones(INICIO):
            print("MoveIt no pudo ir a la posición de partida")

        extractor.trayectoria_actual = "joint"
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