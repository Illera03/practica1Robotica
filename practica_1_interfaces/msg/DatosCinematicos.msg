# Estado cinemático de todas las articulaciones del robot en un instante.
# Los arrays van en paralelo: el elemento i de cada uno corresponde a nombres[i].

std_msgs/Header header      # marca de tiempo copiada de /joint_states
string trayectoria          # trayectoria a la que pertenece la muestra (joint, lineal, circular)
string[] nombres            # nombres de las articulaciones
float64[] posiciones        # rad
float64[] velocidades       # rad/s
float64[] aceleraciones     # rad/s², calculada por diferencias finitas
