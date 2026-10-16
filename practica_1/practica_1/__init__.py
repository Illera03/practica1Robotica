import rclpy
import threading
from control_robot import ControlRobot
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dataclasses import dataclass



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
        """Se ejecuta automáticamente cada vez que el robot publica su estado."""
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

            #Actualizamos _velocidades_anteriores
            self._velocidades_anteriores[name] = velocity

        # 3. Actualizamos _tiempo_anterior
        self._tiempo_anterior = tiempo_actual


def main() -> None:
    rclpy.init()
    control = ControlRobot()
    extractor = ExtractorCinematico()

    # El extractor escucha en un hilo aparte, igual que en consumir_datos_camara.py
    threading.Thread(target=rclpy.spin, args=(extractor,), daemon=True).start()

    try:
        # --- Trayectoria joint ---
        # 1. Llevar el robot al punto de partida (SIN grabar)
        # 2. extractor.trayectoria_actual = "joint"
        # 3. control.mover_articulaciones(...)
        # 4. extractor.trayectoria_actual = None

        # --- Trayectoria lineal --- (mismos 4 pasos, con mover_trayectoria)
        # --- Trayectoria circular --- (mismos 4 pasos, con mover_trayectoria)
        pass
    finally:
        control.ejecutor.shutdown()
        control.destroy_node()
        extractor.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()