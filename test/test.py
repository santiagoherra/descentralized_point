#! /usr/bin/python3

# Librerias generales
import time
import pytest
import tf
import numpy as np
import descentralized_point_simulation
from descentralized_point_simulation import DescentralizedPoint, obtener_delta, definir_parametros

# CONSTANTES
wheel_base      = 0.160  # Distancia entre las ruedas (b)
lenght_g        = 0.138/2     # Distancia desde el centro al frente del robot (g)
KV_GAIN         = 0.3         # Ganancia derivativa
KP_X_GAIN       = 0.07        # Ganancia proporcional
KP_Y_GAIN       = 0.07 
tiempo_ejecucion = 0.0333     # Tiempo de reiteracion
DISTANCIA_UMBRAL = 8          # Distancia a la que el robot esta fuera de rango
DISTANCIA = 0.2              # Parametro de control de distancia
CONTINUIDAD = True            # Bandera que determina si una trayectoria es continua (True) o no (False)
V_LINEAL_MAX = 0.15           # Valor de velocidad linear maxima
W_ANGULAR_MAX = 2.6          # Valor de velocidad angular maxima 


class Prueba_externa():
    def __init__(self):
        self.delta = obtener_delta(descentralized_point_simulation.WAYPOINTS_FILE)

    def test_obtener_delta(self):
        assert isinstance(self.delta, (int, float))
        assert self.delta != 0
        

    def test_parametros(self):
        definir_parametros(self.delta)
        # Pruebas de que las variables globales si existen
        assert hasattr(descentralized_point_simulation, "DISTANCIA_ALTA")
        assert hasattr(descentralized_point_simulation, "OFFSET_ALTO")

        # Pruebas para saber si las variables son enteros
        assert isinstance(descentralized_point_simulation. DISTANCIA_ALTA, (int, float))
        assert isinstance(descentralized_point_simulation. OFFSET_ALTO, int)
        assert isinstance(descentralized_point_simulation. OFFSET_MEDIO, int)
        assert isinstance(descentralized_point_simulation. OFFSET_BAJO, int)

        # Pruebas para saber si las distancias y offset son positivos y mayores que 0
        assert descentralized_point_simulation.DISTANCIA_ALTA > 0
        assert descentralized_point_simulation.DISTANCIA_MEDIA > 0
        assert descentralized_point_simulation.DISTANCIA_BAJA > 0

        assert descentralized_point_simulation.OFFSET_ALTO > 0
        assert descentralized_point_simulation.OFFSET_MEDIO > 0
        assert descentralized_point_simulation.OFFSET_BAJO > 0

        # Verificar la distancia alta
        assert descentralized_point_simulation.DISTANCIA_ALTA == descentralized_point_simulation.DISTANCIA


@pytest.fixture

# Funcion para poder contar decimales para verificar precision de numeros
def contar_decimales(num):
    s = str(num)
    if "." in s:
        return len(s.split(".")[1])
    return 0

def dp():
    return DescentralizedPoint()

def test_obtener_puntos(dp):
    # Funcion para poder medir el tiempo de ejecucion de la funcion en prueba
    inicio_obtener_puntos = time.perf_counter()
    waypoint_test = dp.obtener_puntos()
    fin_obtener_puntos = time.perf_counter()

    duracion = fin_obtener_puntos - inicio_obtener_puntos

    # Prueba que el tiempo en leer el archivo sea menor a 0.3s
    print(f"Duracion de obtener_funcion(): {duracion:.4f} segundos")
    assert duracion < 0.3 

    # Probar si se recuperaron las coordenadas del archivo y si son numeros
    for i in waypoint_test:
        assert isinstance(waypoint_test[i], (int, float))
        if isinstance(waypoint_test[i], float):
            # Prueba si las coordenadas de X y Y son flotantes y si tienen 2 decimales
            assert contar_decimales(waypoint_test[i, 0]) == 2
            assert contar_decimales(waypoint_test[i, 1]) == 2

def test_obtener_trayectoria(dp):
    waypoints = [5,0] # Valores de prueba de waypoint

    # Valores de prueba de coordenadas
    dp.current_x = 0
    dp.current_y = 0
    dp.current_theta = 0
    dp.current_target_idx = 1 # Se pone en 1 por ejemplo

    dp.obtener_trayectoria(waypoints)
    
    # Prueba que las coordenadas sean numeros de tipo int o float
    assert isinstance(dp.trajectory_x, (int, float))
    assert isinstance(dp.trajectory_y, (int, float))

    # Prueba que el calculo de las coordenadas coincidan con los waypoint
    assert dp.trajectory_x == 5
    assert dp.trajectory_y == 0

def test_punto_descentralizado_externo(dp):
    # Se definen las variables para poder ejecutar el algoritmo completo
    # Valores de odometria del robot movil

    dp.current_x = 0.0
    dp.current_y = 0.0
    dp.current_theta = 0.0

    # Coordenadas de el siguiente punto en la trayectoria
    dp.trajectory_x = 0.0
    dp.trajectory_y = 0.0

    # Derivada de la trayectoria recorrida del robot
    dp.trajectory_dx = 0.0
    dp.trajectory_dy = 0.0

    # Valor de indice de punto de ruta
    dp.current_target_idx = 1

    # Se definen todas las variables de orientacion y posicion en el origen
    # simulando asi el odom_msg
    dp.punto_descentralizado([0, 0, 0, 0, 0, 0])
 
    v_lineal_prueba = dp.actuaction.linear.x
    w_lineal_prueba = dp.actuation.angular.z

    # Pruebas de tipo de variables las velocidades
    assert isinstance(v_lineal_prueba, (int, float))
    assert isinstance(w_lineal_prueba, (int, float))

    # Pruebas de valores validos permitidos en velocidad
    assert -V_LINEAL_MAX <= v_lineal_prueba <= V_LINEAL_MAX
    assert -W_ANGULAR_MAX <= w_lineal_prueba <= W_ANGULAR_MAX

    # Prueba para el valor correcto de salida velocidad angular y linear
    assert v_lineal_prueba == 0.13 # Esto debe de cambiar para el waypoint de WAYPOINT_FILE
    assert w_lineal_prueba == 2.1 # Esto debe de cambiar para el waypoint de WAYPOINT_FILE

def test_punto_descentralizado_interno(waypoint):

    # Se guarda el Quaternio de orientacion en orientation_list
    orientation_list = [0, 0, 0, 0]
    
    # Se determina los tres angulos de orientacion
    (roll, pitch, theta) = tf.transformations.euler_from_quaternion(orientation_list)

    current_x = 0
    current_y = 0
    current_theta = theta

    trajectory_x = waypoint[0]
    trajectory_y = waypoint[1]

    # Derivada de la trayectoria
    trajectory_dx = trajectory_x - current_x
    trajectory_dy = trajectory_y - current_y

    # Componente proporcional a la velocidad de referencia
    vel_component = KV_GAIN * np.array([[trajectory_dx],
                                        [trajectory_dy]])

    # Componente proporcional a la posición
    error_x = trajectory_x - (current_x + lenght_g *
                            np.cos(current_theta))

    error_y = trajectory_y - (current_y + lenght_g *
                                np.sin(current_theta))

    krp_identidad = np.array([[KP_X_GAIN, 0],
                            [0, KP_Y_GAIN]])

    e_krp = krp_identidad @ np.array([[error_x],
                                        [error_y]])

    # Resultado final del control cinemático
    control_cinematico = vel_component + e_krp

    # Matriz de conversión (cinemática inversa)
    B = (1 / lenght_g) * np.array([
        [lenght_g * np.cos(current_theta) + 0.5 * wheel_base * np.sin(current_theta),
            lenght_g * np.sin(current_theta) - 0.5 * wheel_base * np.cos(current_theta)],

        [lenght_g * np.cos(current_theta) - 0.5 * wheel_base * np.sin(current_theta),
            lenght_g * np.sin(current_theta) + 0.5 * wheel_base * np.cos(current_theta)]
    ])

    # Velocidades de referencia de las ruedas (lineales)
    v = B @ control_cinematico

    # obtener valores de la velocidad lineal y angular
 
    mod_cine_direc = np.array([[1/2, 1/2],
                    [-1/(wheel_base), 1/(wheel_base)]
                    ])

    v_w_lineal = mod_cine_direc @ v

    v_lineal = v_w_lineal[0]

    w_lineal = v_w_lineal[1]

    # Prueba









        