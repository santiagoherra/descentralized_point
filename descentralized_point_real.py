#! /usr/bin/python
# -*- coding: utf-8 -*-

##### Librerias y tipos de mensajes #####

# Librerias generales
import rospy
import numpy as np
import math
import csv

# Int8 es el tipo de mensaje por el que se publica el valor de PWM de los motores del robot
from std_msgs.msg import Int8

# WheelSpeed es el tipo de mensaje por el que se lee las velocidades angulares de las ruedas del robot
from mecanumrob_common.msg import WheelSpeed

### Parámetros ###
wheel_base      = 0.276  # Distancia entre las ruedas (b)
lenght_g        = 0.15     # Distancia desde el centro al frente del robot (g)
R               = 0.0505      # Radio de las ruedas del robot
KV_GAIN         = 0.9         # Ganancia derivativa
KP_X_GAIN       = 0.1        # Ganancia proporcional
KP_Y_GAIN       = 0.1
tiempo_ejecucion = 0.029     # Tiempo de reiteracion
DISTANCIA_UMBRAL = 8          # Distancia a la que el robot esta fuera de rango
DISTANCIA = 0.4              # Parametro de control de distancia
CONTINUIDAD = True            # Bandera que determina si una trayectoria es continua (True) o no (False)
V_LINEAL_MAX = 120           # Valor de velocidad linear maxima en PWM

# Direccion de archivo que contiene la ruta de la trayectoria
WAYPOINTS_FILE  =  ("/home/dif1/catkin_ws/src/MiniMecanum/mecanumrob_roboclaw/scripts/"
	            "trayectoria_cuadrado.csv"
                    )

# Parametros a definir por medio de sofware, definidos en 0 hasta la ejecucion del programa
OFFSET_ALTO = 0
OFFSET_MEDIO = 0
OFFSET_BAJO = 0
DISTANCIA_ALTA = 0
DISTANCIA_MEDIA = 0
DISTANCIA_BAJA = 0


# Topicos

odom_topic = "/odom" # Topico del mensaje de la posicion

### Algoritmo ###

class Descentralized_point():
    """
    Clase de Punto Descentralizado:

    Esta clase se encarga de ejecutar el algoritmo de punto descentralizado
    para un robot movil descentralizado.
    """

    def __init__(self):
        self.base_name = rospy.get_param("~BaseName", default ="")

        # Odometria del robot movil
        self.wheelbase     = wheel_base
        self.current_x     = 0
        self.current_y     = 0
        self.current_theta = 0

        # Coordenadas de trayectoria y su derivada
        self.trajectory_x = 0
        self.trajectory_y = 0

        self.trajectory_dx = 0
        self.trajectory_dy = 0

        # Indice de trayectoria
        self.current_target_idx = OFFSET_ALTO

	# Guardar valores de posicion del robot.
	    self.file_path = "/home/dif1/catkin_ws/src/MiniMecanum/mecanumrob_roboclaw/sensors_results/prueba3_26_6.csv"
	    with open(self.file_path, "wb") as file:
		    writer = csv.writer(file)
		    writer.writerow(["current_x", "current_y", "v_lineal", "w_lineal"])
	
        # Velocidades de las ruedas izquierda y derecha
        self.izq_pub = rospy.Publisher("%s/motor/SW_pwm" % self.base_name,
                                        Int8, queue_size=0)

        self.der_pub = rospy.Publisher("%s/motor/SE_pwm" % self.base_name,
                                        Int8, queue_size=0)
        
        # Velocidades que devuelve el encoder.
        self.enc_sub =  rospy.Subscriber("%s/wheel_speed" % self.base_name,
                                           WheelSpeed, self.punto_descentralizado, queue_size = 50)
        
    def obtener_puntos(self):
        SKIP_ROWS = 1
        DELIMITER = ","
        WAYPOINTS_FILE  = "/home/dif1/catkin_ws/src/MiniMecanum/mecanumrob_roboclaw/scripts/trayectoria_cuadrado.csv"
        waypoints = np.loadtxt(WAYPOINTS_FILE, delimiter=DELIMITER, skiprows=SKIP_ROWS)
        return waypoints
    
    def modificar_posicion(self, encoders):

        # Velocidad angular de la rueda izquierda y derecha
        w_der = encoders.phi[3]
        w_izq = - encoders.phi[2]

        # Velocidad lineal de la rueda izquierda y derecha
        v_der =  (w_der) * R 
        v_izq =  (w_izq) * R

        # Velocidad angular del robot a la mitad
        omega = ((v_der-v_izq) / (self.wheelbase))

        # Actualizando el angulo de orientacion del robot
        self.current_theta += omega * tiempo_ejecucion

        # Actualizando la posicion del robot.
        self.current_x += ((v_der+v_izq)/2) * math.cos(self.current_theta) * tiempo_ejecucion
        self.current_y += ((v_der+v_izq)/2) * math.sin(self.current_theta) * tiempo_ejecucion

    def obtener_trayectoria(self, waypoints):
        # global primer_ciclo

        # Inicializar índice objetivo solo una vez
        # if self.current_target_idx is None and not primer_ciclo:
            # self.current_target_idx = self.encontrar_idx_mas_cercano(waypoints)
            # primer_ciclo = True

        punto_objetivo = waypoints[self.current_target_idx]

        dx = punto_objetivo[0] - self.current_x
        dy = punto_objetivo[1] - self.current_y
        distancia = np.sqrt(dx**2 + dy**2)

        if distancia <= DISTANCIA_UMBRAL:

            if DISTANCIA_MEDIA < distancia <= DISTANCIA_ALTA:
                siguiente_idx = self.current_target_idx + OFFSET_ALTO
            elif DISTANCIA_BAJA < distancia <= DISTANCIA_MEDIA:
                siguiente_idx = self.current_target_idx + OFFSET_MEDIO
            elif distancia < DISTANCIA_BAJA:
                siguiente_idx = self.current_target_idx + OFFSET_BAJO
            else:
                siguiente_idx = self.current_target_idx

            if siguiente_idx < len(waypoints):
                self.current_target_idx = siguiente_idx
            else:
                if CONTINUIDAD:
                    self.current_target_idx = OFFSET_ALTO  # reiniciar
                else:
                    self.current_target_idx = len(waypoints) - 1  # quedarse al final

        # Actualizar valores de trayectoria siempre
        self.trajectory_x = waypoints[self.current_target_idx][0]
        self.trajectory_y = waypoints[self.current_target_idx][1]

    def encontrar_idx_mas_cercano(self, waypoints):
        posicion_actual = np.array([self.current_x, self.current_y])
        distancias = np.linalg.norm(waypoints - posicion_actual, axis=1)

        return np.argmin(distancias)

    def punto_descentralizado(self, encoders):

        # Ejecutar todas los metodos para poder correr el algoritmo
        # punto descentralizado.

        waypoints = self.obtener_puntos()

        self.modificar_posicion(encoders)


        self.obtener_trayectoria(waypoints)

        # Encontrar sguiente punto de la trayectoria en x y y
        #next_trayectory_x = waypoints[self.current_target_idx + 1][0] # Offset para encontrar derivada
        #next_trayectory_y = waypoints[self.current_target_idx + 1][1] # Offset para encontrar derivada

        # Derivada de la trayectoria
        self.trajectory_dx = self.trajectory_x - self.current_x
        self.trajectory_dy = self.trajectory_y - self.current_y

        # Componente proporcional a la velocidad de referencia
        vel_component = KV_GAIN * np.array([[self.trajectory_dx],
                                            [self.trajectory_dy]])

        # Componente proporcional a la posición
        error_x = self.trajectory_x - (self.current_x + lenght_g *
                                        np.cos(self.current_theta))

        error_y = self.trajectory_y - (self.current_y + lenght_g *
                                        np.sin(self.current_theta))

        krp_identidad = np.array([[KP_X_GAIN, 0],
                                [0, KP_Y_GAIN]])

        e_matriz = np.array([[error_x],
                             [error_y]])

	    e_krp = np.dot(krp_identidad, e_matriz)

        # Resultado final del control cinemático
        control_cinematico = vel_component + e_krp

        # Matriz de conversión (cinemática inversa)
        B = (1 / lenght_g) * np.array([
            [lenght_g * np.cos(self.current_theta) + 0.5 * wheel_base * np.sin(self.current_theta),
              lenght_g * np.sin(self.current_theta) - 0.5 * wheel_base * np.cos(self.current_theta)],

            [lenght_g * np.cos(self.current_theta) - 0.5 * wheel_base * np.sin(self.current_theta),
              lenght_g * np.sin(self.current_theta) + 0.5 * wheel_base * np.cos(self.current_theta)]
        ])

        # Velocidades de referencia de las ruedas (lineales)
        v = np.dot(B, control_cinematico)

        # Obtener velocidades de las dos ruedas en PWM
        v_izq_PWM = v[0] * 100
        v_der_PWM = v[1] * 100

	# Limitar las velocidades del algoritmo

        if(v_izq_PWM > V_LINEAL_MAX):
            v_izq_PWM = V_LINEAL_MAX

        if(v_der_PWM > V_LINEAL_MAX):
            v_der_PWM = V_LINEAL_MAX
          
        mod_cine_direc = np.array([[1/2, 1/2],
                        [-1/(wheel_base), 1/(wheel_base)]
                        ])
        
        v_w_lineal = np.dot(mod_cine_direc, v)

        v_lineal = v_w_lineal[0]
        w_lineal = v_w_lineal[1]

	# Escribir fila de valores de posicion X y Y
        with open(self.file_path, "ab") as file:
                writer = csv.writer(file)
                writer.writerow([self.current_x, self.current_y, v_lineal, w_lineal])


        # Imprimir los mensajes
        print(
            "Velocidad Izq: " + str(v_izq_PWM) + " | Velocidad Der: " + str(v_der_PWM) + "\n" +
            "Trajectory X: " + str(self.trajectory_x) + " | Trajectory Y: " + str(self.trajectory_y) + "\n" +
            "Dx Trajectory: " + str(self.trajectory_dx) + " | Dy Trajectory: " + str(self.trajectory_dy) + "\n" +
            "Current X: " + str(self.current_x) + " | Current Y: " + str(self.current_y) + "\n")

        # Se publican los mensajes
        self.izq_pub.publish(Int8(v_izq_PWM))
        self.der_pub.publish(Int8(-v_der_PWM))

        return None


def obtener_delta(ruta_csv):
    """
    Carga la cantidad de filas que hay en el archivo de trayectoria para obtener la cantidad de filas totales
    para poder obtener el desplazamiento del indice de la trayectoria.
    """

    with open(ruta_csv, 'r') as f:
        total_lineas = sum(1 for _ in f)
    
    # Restamos 1 por la cabecera
    cantidad_filas = max(total_lineas - 1, 0)
    
    delta = cantidad_filas * 0.05

    if delta == 0:
        delta = 1
        return delta
    else:
        return delta


def definir_parametros(delta):
    """
    Define los parámetros globales de distancia y desplazamiento (offset) según un valor 
    delta de separación entre puntos. Divide la distancia máxima en tres rangos (alta, media 
    y baja) y calcula los respectivos offsets redondeados para cada rango.
    """

    global OFFSET_ALTO, OFFSET_MEDIO, OFFSET_BAJO, DISTANCIA_ALTA, DISTANCIA_MEDIA, DISTANCIA_BAJA

    # Definiendo valores de distancia dividiendolas en 3
    DISTANCIA_ALTA = DISTANCIA
    DISTANCIA_MEDIA = DISTANCIA * (2/3)
    DISTANCIA_BAJA = DISTANCIA * (1/3)

    # Definiendo valores de offset para cada una de las distancias
    OFFSET_ALTO = round(delta)
    OFFSET_MEDIO = round(delta * 2/3)
    OFFSET_BAJO = round(delta * 1/3)


def main():
    """
    Función principal del nodo. Calcula el valor de delta x a partir de los primeros puntos de 
    la trayectoria, define los parámetros globales necesarios, inicializa el nodo ROS y el 
    objeto de control descentralizado, y mantiene el nodo en ejecución.
    """

    # Obteniendo el valor de desplazamiento para obtener parametros del algoritmo.
    delta = obtener_delta(WAYPOINTS_FILE)
    definir_parametros(delta)

    rospy.init_node("RoboclawTester", log_level=rospy.DEBUG)
    prueba = Descentralized_point()

    print("\n descentralized point real :) \n")

    rospy.spin()

if __name__ == "__main__":
    main()


        

        




        
        
