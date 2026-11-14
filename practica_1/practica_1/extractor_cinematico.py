"""Extracción de datos cinemáticos del UR3e en trayectorias joint, lineal y circular.

ExtractorCinematico se suscribe a /joint_states, calcula la aceleración de cada
articulación y publica los resultados en un mensaje DatosCinematicos.
SecuenciaTrayectorias mueve el robot con MoveIt y etiqueta cada trayectoria para
que sus muestras se guarden por separado.
"""

import copy
import csv
import threading
from collections.abc import Iterator, Sequence
from contextlib import contextmanager
from dataclasses import astuple, dataclass
from math import cos, pi, radians, sin
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from practica_1.control_robot import ControlRobot
from practica_1_interfaces.msg import DatosCinematicos

# Comunicación ROS 2
TOPIC_ESTADO_ARTICULACIONES: str = "/joint_states"
TOPIC_DATOS_CINEMATICOS: str = "/datos_cinematicos"
PROFUNDIDAD_COLA: int = 10
NANOSEGUNDOS_POR_SEGUNDO: float = 1e9
# Velocidad (rad/s) por debajo de la cual una articulación se considera parada.
# El robot quieto publica 0.0 exacto; el margen absorbe ruido numérico.
UMBRAL_VELOCIDAD: float = 1e-3

# Configuraciones articulares (rad)
HOME: list[float] = [0.0, -pi / 2, -pi / 2, -pi / 2, pi / 2, 0.0]
# Brazo estirado. Solo sirve de partida para la trayectoria joint: es una
# configuración singular y desde ella fallan las trayectorias cartesianas.
INICIO: list[float] = [0.0] * 6

# Geometría de las trayectorias cartesianas (m)
DESCENSO_PREVIO: float = 0.1  # bajada del efector antes de cada trayectoria cartesiana
TRAMO_LINEAL: float = 0.1
RADIO_CIRCULO: float = 0.03
# Con r = 3 cm, 10° dan unos 5 mm de cuerda, por debajo del max_step de 1 cm de MoveIt
PASO_ANGULAR_GRADOS: int = 10
GRADOS_VUELTA: int = 360

CABECERA_CSV: tuple[str, ...] = (
    "tiempo", "articulacion", "posicion", "velocidad", "aceleracion"
)


@dataclass
class MuestraCinematica:
    """Medida cinemática de una articulación en un instante concreto.

    El orden de los campos coincide con CABECERA_CSV.
    """
    tiempo: float
    articulacion: str
    posicion: float
    velocidad: float
    aceleracion: float


class ExtractorCinematico(Node):
    """Nodo suscriptor que extrae posición, velocidad y aceleración del UR3e.

    Solo guarda y publica muestras mientras hay una trayectoria activa
    (ver ``registrando``) y alguna articulación se está moviendo.
    """

    def __init__(self) -> None:
        """Crea la suscripción a /joint_states y el publicador de DatosCinematicos."""
        super().__init__("extractor_cinematico")
        self.create_subscription(
            JointState,
            TOPIC_ESTADO_ARTICULACIONES,
            self._callback_articulaciones,
            PROFUNDIDAD_COLA,
        )
        self._publicador = self.create_publisher(
            DatosCinematicos, TOPIC_DATOS_CINEMATICOS, PROFUNDIDAD_COLA
        )
        self._trayectoria_actual: str | None = None
        self._muestras_por_trayectoria: dict[str, list[MuestraCinematica]] = {}
        # Estado del mensaje anterior, necesario para derivar la velocidad
        self._tiempo_anterior: float | None = None
        self._velocidades_anteriores: dict[str, float] = {}

    @contextmanager
    def registrando(self, trayectoria: str) -> Iterator[None]:
        """Etiqueta con ``trayectoria`` las muestras tomadas dentro del bloque ``with``.

        El ``finally`` retira la etiqueta aunque el movimiento lance una excepción,
        así que los movimientos posteriores nunca se mezclan con esta trayectoria.
        """
        self._trayectoria_actual = trayectoria
        try:
            yield
        finally:
            self._trayectoria_actual = None

    def guardar_csv(self, directorio: Path) -> None:
        """Escribe un fichero datos_<trayectoria>.csv por cada trayectoria registrada."""
        for trayectoria, muestras in self._muestras_por_trayectoria.items():
            ruta = directorio / f"datos_{trayectoria}.csv"
            with ruta.open("w", newline="") as fichero:
                escritor = csv.writer(fichero)
                escritor.writerow(CABECERA_CSV)
                escritor.writerows(astuple(muestra) for muestra in muestras)

    def _callback_articulaciones(self, msg: JointState) -> None:
        """Calcula la aceleración por diferencias finitas y guarda la muestra si procede."""
        tiempo_actual = msg.header.stamp.sec + msg.header.stamp.nanosec / NANOSEGUNDOS_POR_SEGUNDO
        if self._tiempo_anterior is None:
            # Primer mensaje: aún no hay velocidad anterior con la que derivar
            self._tiempo_anterior = tiempo_actual
            self._velocidades_anteriores = dict(zip(msg.name, msg.velocity))
            return

        dt = tiempo_actual - self._tiempo_anterior
        if dt <= 0.0:
            # Mensaje repetido o desordenado: la división daría infinito o signo erróneo
            return

        # Copia local: el hilo principal puede cambiar la etiqueta durante el callback
        trayectoria = self._trayectoria_actual
        guardar = trayectoria is not None and self._hay_movimiento(msg.velocity)
        aceleraciones: list[float] = []

        for nombre, posicion, velocidad in zip(msg.name, msg.position, msg.velocity):
            aceleracion = (velocidad - self._velocidades_anteriores.get(nombre, 0.0)) / dt
            aceleraciones.append(aceleracion)
            if guardar:
                muestra = MuestraCinematica(tiempo_actual, nombre, posicion, velocidad, aceleracion)
                self._muestras_por_trayectoria.setdefault(trayectoria, []).append(muestra)
            # Se actualiza también en reposo para que la primera aceleración de
            # cada movimiento use la velocidad inmediatamente anterior
            self._velocidades_anteriores[nombre] = velocidad

        if guardar:
            self._publicar(msg, trayectoria, aceleraciones)
        self._tiempo_anterior = tiempo_actual

    @staticmethod
    def _hay_movimiento(velocidades: Sequence[float]) -> bool:
        """Indica si alguna articulación supera el umbral de velocidad."""
        return any(abs(velocidad) > UMBRAL_VELOCIDAD for velocidad in velocidades)

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


class SecuenciaTrayectorias:
    """Ejecuta las trayectorias de la práctica y las etiqueta en el extractor."""

    def __init__(self, control: ControlRobot, extractor: ExtractorCinematico) -> None:
        """Recibe el controlador de MoveIt y el extractor que etiqueta las muestras."""
        self._control = control
        self._extractor = extractor

    def ejecutar(self) -> None:
        """Ejecuta las trayectorias joint, lineal y circular y deja el robot en HOME."""
        self._trayectoria_joint()
        self._trayectoria_lineal()
        self._trayectoria_circular()
        self._mover_articulaciones(HOME, "HOME")

    def _trayectoria_joint(self) -> None:
        """Movimiento en espacio articular de INICIO a HOME.

        Primero se va a INICIO sin registrar: si el robot ya estuviera en HOME,
        el movimiento grabado sería nulo.
        """
        self._mover_articulaciones(INICIO, "INICIO")
        with self._extractor.registrando("joint"):
            self._mover_articulaciones(HOME, "HOME")

    def _trayectoria_lineal(self) -> None:
        """Tres tramos rectos del efector final: subida en Z y avances en Y y en X."""
        self._preparar_trayectoria_cartesiana()
        pose = self._control.pose_actual()
        poses: list[Pose] = []
        pose.position.z += TRAMO_LINEAL
        poses.append(copy.deepcopy(pose))
        pose.position.y += TRAMO_LINEAL
        poses.append(copy.deepcopy(pose))
        pose.position.x += TRAMO_LINEAL
        poses.append(copy.deepcopy(pose))

        with self._extractor.registrando("lineal"):
            self._mover_cartesiano(poses, "lineal")

    def _trayectoria_circular(self) -> None:
        """Una vuelta completa del efector final en el plano XY."""
        self._preparar_trayectoria_cartesiana()
        poses = self._poses_circulo(self._control.pose_actual())

        with self._extractor.registrando("circular"):
            self._mover_cartesiano(poses, "circular")

    @staticmethod
    def _poses_circulo(pose_inicial: Pose) -> list[Pose]:
        """Puntos de un círculo en el plano XY que empieza y acaba en ``pose_inicial``.

        El centro está desplazado RADIO_CIRCULO en X, de modo que el ángulo 0
        coincide con la pose de partida. Cada punto se calcula desde la pose
        inicial y no desde el anterior, para no acumular desplazamientos.
        """
        poses: list[Pose] = []
        for grados in range(0, GRADOS_VUELTA + 1, PASO_ANGULAR_GRADOS):
            angulo = radians(grados)
            pose = copy.deepcopy(pose_inicial)
            pose.position.x = pose_inicial.position.x + RADIO_CIRCULO * (1 - cos(angulo))
            pose.position.y = pose_inicial.position.y + RADIO_CIRCULO * sin(angulo)
            poses.append(pose)
        return poses

    def _preparar_trayectoria_cartesiana(self) -> None:
        """Lleva el robot a HOME y baja el efector para partir siempre de la misma pose."""
        self._mover_articulaciones(HOME, "HOME")
        pose = self._control.pose_actual()
        pose.position.z -= DESCENSO_PREVIO
        if not self._control.mover_a_pose(pose):
            self._avisar_fallo("bajar el efector final")

    def _mover_articulaciones(self, objetivo: list[float], nombre: str) -> None:
        """Mueve el robot a una configuración articular y avisa si MoveIt falla."""
        if not self._control.mover_articulaciones(objetivo):
            self._avisar_fallo(f"mover las articulaciones a {nombre}")

    def _mover_cartesiano(self, poses: list[Pose], nombre: str) -> None:
        """Recorre las poses en línea recta y avisa si MoveIt falla."""
        if not self._control.mover_trayectoria(poses):
            self._avisar_fallo(f"planificar o ejecutar la trayectoria {nombre}")

    def _avisar_fallo(self, accion: str) -> None:
        """Registra en el log de ROS 2 que MoveIt no pudo completar una acción."""
        self._control.get_logger().warning(f"MoveIt no pudo {accion}")


def main() -> None:
    """Arranca los nodos, ejecuta la secuencia de trayectorias y guarda los CSV."""
    rclpy.init()
    control = ControlRobot()
    extractor = ExtractorCinematico()

    # ControlRobot atiende sus callbacks en su propio hilo. El extractor necesita
    # otro ejecutor para seguir recibiendo /joint_states mientras el hilo
    # principal está bloqueado esperando a MoveIt.
    ejecutor_extractor = SingleThreadedExecutor()
    ejecutor_extractor.add_node(extractor)
    threading.Thread(target=ejecutor_extractor.spin, daemon=True).start()

    try:
        SecuenciaTrayectorias(control, extractor).ejecutar()
    finally:
        control.ejecutor.shutdown()
        ejecutor_extractor.shutdown()
        # Se guarda aunque la secuencia falle a mitad, para no perder lo registrado
        extractor.guardar_csv(Path.cwd())
        extractor.destroy_node()
        control.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()