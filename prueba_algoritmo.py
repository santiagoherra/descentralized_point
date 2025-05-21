
import numpy as np

### PARAMETROS 

wheel_base      = 0.577  # Mitad de la distancia entre las ruedas (b)
R               = 0.511     # Radio de la rueda
lenght_g        = 0.625     # Distancia desde el centro al frente del robot (g)
KV_GAIN         = 10          # Ganancia de la velocidad lineal
KP_GAIN         = 5          # Ganancia proporcional de la posición
tiempo_ejecucion = 0.022     # Tiempo de reiteracion

def algortimo_pp(current_x, current_y, current_theta, trajectory_x, trajectory_y):
    # Derivada de la trayectoria
    trajectory_dx = 5
    trajectory_dy = 5

    print(f" trajectory_dx {trajectory_dx}, trajectory_dy {trajectory_dy}")

    # Componente proporcional a la velocidad de referencia
    vel_component = KV_GAIN * np.array([[trajectory_dx],
                                        [trajectory_dy]])

    # Componente proporcional a la posición
    error_x = trajectory_x - (current_x + lenght_g *
                            np.cos(current_theta))

    error_y = trajectory_y - (current_y + lenght_g *
                            np.sin(current_theta))

    krp_identidad = np.array([[KP_GAIN, 0],
                            [0, KP_GAIN]])

    e_krp = krp_identidad @ np.array([[error_x],
                                        [error_y]])

    # Resultado final del control cinemático
    control_cinematico = vel_component + e_krp

    # Matriz de conversión (cinemática inversa)
    B = (1 / lenght_g) * np.array([
        [lenght_g* np.cos(current_theta) + 0.5 * wheel_base * np.sin(current_theta),
            lenght_g * np.sin(current_theta) - 0.5 * wheel_base * np.cos(current_theta)],

        [lenght_g * np.cos(current_theta) - 0.5 * wheel_base * np.sin(current_theta),
            lenght_g * np.sin(current_theta) + 0.5 * wheel_base * np.cos(current_theta)]
    ])

    # Velocidades de referencia de las ruedas (lineales)
    v = B @ control_cinematico

    # Obtener velocidades de las dos ruedas
    v_izq = v[0]
    v_der = v[1]

    print(f"v_izq {v_izq}, v_der {v_der}\n")

    #Obtener velocidad lineal
    v_lineal = (v_izq + v_der) / 2

    omega = 1/R * v_lineal

def main():
    algortimo_pp(0, 0, 0, 3, 3)


main()