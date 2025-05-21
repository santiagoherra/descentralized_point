import numpy as np
import pdb

current_target_idx = None

DISTANCIA_UMBRAL = 15  # Distancia a la que el robot esta fuera de rango
OFFSET           = 1  # Offset que determina los puntos hacia adelante de la trayectoria
                       # que depende de el cambio de distancia entre los puntos.
CONTINUIDAD = True     # Bandera que determina si una trayectoria es continua (True) o no (False)


def obtener_puntos():
    SKIP_ROWS = 1
    DELIMITER = ","
    WAYPOINTS_FILE  =  "/home/labautomatica05/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/circulo_5m_300pts.csv"
    waypoints = np.loadtxt(WAYPOINTS_FILE, delimiter=DELIMITER, skiprows=SKIP_ROWS)
    return waypoints

def obtener_trayectoria(waypoints, current_x, current_y):

    pdb.set_trace()

    global current_target_idx

    # Si no existe aún un índice de objetivo, inicializarlo al punto más cercano
    if current_target_idx is None:
        current_target_idx = encontrar_idx_mas_cercano(waypoints, current_x, current_y)
    
    # Obtener el punto objetivo actual
    punto_objetivo = waypoints[current_target_idx]
    
    # Calcular la distancia a ese punto
    dx = punto_objetivo[0] - current_x
    dy = punto_objetivo[1] - current_y
    distancia = np.sqrt(dx**2 + dy**2)
    
    if distancia <= DISTANCIA_UMBRAL:
        siguiente_idx = current_target_idx + OFFSET
        if siguiente_idx < len(waypoints):
            current_target_idx = siguiente_idx
        else:
            if CONTINUIDAD:
                current_target_idx = OFFSET  # reiniciar con salto desde inicio
            else:
                current_target_idx = len(waypoints) - 1  # quedarse al final


    # Actualizar los valores de la trayectoria
    trajectory_x = waypoints[current_target_idx][0]
    trajectory_y = waypoints[current_target_idx][1]

    return trajectory_x, trajectory_y
    
def encontrar_idx_mas_cercano(waypoints, current_x, current_y):

    posicion_actual = np.array([current_x, current_y])
    distancias = np.linalg.norm(waypoints - posicion_actual, axis=1)

    return np.argmin(distancias)

# Generar waypoints aleatorios (10x2)
def main():

    global current_target_idx
    current_target_idx = None  # ← ¡reinicio explícito!

    waypoints = np.array([
        [1.0,  2.0],
        [3.5,  5.0],
        [2.2,  8.1],
        [7.5,  1.5],
        [6.3,  4.8],
        [9.0,  0.5],
        [4.2,  7.3],
        [5.5,  3.2],
        [8.4,  6.6],
        [0.9,  9.0]
    ])

    while True:
        # Ejecutar prueba
        current_x = float(input("da el valor de x\n"))
        current_y = float(input("da el valor de y\n"))


        trajectory_x, trajectory_y = obtener_trayectoria(waypoints, current_x, current_y)

        print(f"Resultado -> trajectory_x: {trajectory_x}, trajectory_y: {trajectory_y}")


main()
