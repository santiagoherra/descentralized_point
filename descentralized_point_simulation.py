#! /usr/bin/python3

# Librerias generales
import rospy
import numpy as np
import math
import tf
import csv

import pdb

# Twist es el tipo de mensaje por el que se publica la velocidad del robot
from geometry_msgs.msg import Twist

# Odometry es el tipo de mensaje por el que se lee la posicion del robot
from nav_msgs.msg import Odometry

### Parámetros ###
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

# Direccion de archivo que contiene la ruta de la trayectoria
WAYPOINTS_FILE  =  ("/home/labautomatica05/catkin_ws/src/turtlebot3_simulations/"
                   "turtlebot3_gazebo/descentralized_point/trayectorias/trayectoria_reloj_arena.csv"
                    )

# Parametros a definir por medio de software, definidos en 0 hasta la ejecucion del programa
OFFSET_ALTO = 0
OFFSET_MEDIO = 0
OFFSET_BAJO = 0
DISTANCIA_ALTA = 0
DISTANCIA_MEDIA = 0
DISTANCIA_BAJA = 0

# Topicos para la obtencion de datos del nodo

drive_topic         = "/cmd_vel" 
odom_topic         = "/odom" 

### Clase ###
class DescentralizedPoint:
    def __init__(self):
        self.wheelbase = wheel_base

        # Valores de odometria del robot movil
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Coordenadas de el siguiente punto en la trayectoria
        self.trajectory_x = 0.0
        self.trajectory_y = 0.0

        # Derivada de la trayectoria recorrida del robot
        self.trajectory_dx = 0.0
        self.trajectory_dy = 0.0

        # Valor de indice de punto de ruta
        self.current_target_idx = OFFSET_ALTO

        # Clase para publicar las variables a controlar
        self.actuaction = Twist()

        # Variables de publicacion y suscripcion a topicos
        self.drive_pub = rospy.Publisher(drive_topic, Twist, queue_size=100)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.punto_descentralizado, queue_size=100)

        self.file_path = "/home/labautomatica05/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/descentralized_point/desempennos/prueba_falsa.csv"
        with open(self.file_path, "w", newline='') as file:  # Usar "w" en lugar de "wb" en Python 3
            writer = csv.writer(file)
            writer.writerow(["current_x", "current_y", "v_lineal", "w_lineal"])

    def obtener_puntos(self):
        """
        Carga los puntos de trayectoria desde el archivo CSV especificado en WAYPOINTS_FILE.
        Retorna:
        np.ndarray: Un arreglo con los puntos (x, y) de la trayectoria.
        """

        SKIP_ROWS = 1
        DELIMITER = ","
        waypoints = np.loadtxt(WAYPOINTS_FILE, delimiter=DELIMITER, skiprows=SKIP_ROWS)
        return waypoints

    def obtener_trayectoria(self, waypoints):
        """
        Determina el punto objetivo de la trayectoria basado en la posición actual del robot
        y actualiza los valores a seguir. Utiliza umbrales de distancia para decidir cuánto
        avanzar en el índice de la trayectoria. Si se alcanza el final, reinicia o se detiene según 
        la configuración de continuidad.
        """
        #global primer_ciclo

        # Inicializar índice objetivo solo una vez
        #if self.current_target_idx is None and not primer_ciclo:
            #self.current_target_idx = self.encontrar_idx_mas_cercano(waypoints)
            #primer_ciclo = True


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
        """
        Calcula el índice del waypoint más cercano a la posición actual del robot usando la 
        distancia euclidiana. Retorna el índice del punto más próximo en la trayectoria.
        """

        posicion_actual = np.array([self.current_x, self.current_y])
        distancias = np.linalg.norm(waypoints - posicion_actual, axis=1)

        return np.argmin(distancias)


    def punto_descentralizado(self, odom_msg):
        """
        Calcula y aplica la señal de control para el robot en base a su posición actual y la 
        trayectoria deseada. Convierte la orientación del robot desde cuaterniones a ángulos de 
        Euler, calcula errores de posición y velocidad, y genera las velocidades de rueda 
        utilizando un modelo cinemático inverso. Limita las señales de control y publica los 
        valores calculados para accionar el robot.
        """

        waypoints = self.obtener_puntos()

        # Se guarda el Quaternio de orientacion en orientation_list
        orientation_list = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                            odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        
        # Se determina los tres angulos de orientacion
        (roll, pitch, theta) = tf.transformations.euler_from_quaternion(orientation_list)

        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        self.current_theta = theta

        # Obtener los puntos de la trayectoria
        self.obtener_trayectoria(waypoints)

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

        e_krp = krp_identidad @ np.array([[error_x],
                                          [error_y]])

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
        v = B @ control_cinematico

        # obtener valores de la velocidad lineal y angular

        mod_cine_direc = np.array([[1/2, 1/2],
                        [-1/(wheel_base), 1/(wheel_base)]
                        ])

        v_w_lineal = mod_cine_direc @ v

        v_lineal = v_w_lineal[0]

        w_lineal = v_w_lineal[1]

        # Agregando limitaciones de la señal de control para prevenir errores:
        if(v_lineal > V_LINEAL_MAX): 
            v_lineal = V_LINEAL_MAX
            # Agregar la nueva velocidad de la rueda izquierda y derecha para poder imprimir la nueva velocidad de las ruedas si se pasa
            # tambien para la velocidad angular.

        if(w_lineal > W_ANGULAR_MAX):
            w_lineal = W_ANGULAR_MAX

        # Asignar actuacion
        self.actuaction.linear.x = v_lineal

        self.actuaction.angular.z = w_lineal

        with open(self.file_path, "a", newline='') as file:  # Usa "a" en Python 3
            writer = csv.writer(file)
            writer.writerow([self.current_x, self.current_y, v_lineal, w_lineal])


        # Se publica el mensaje y se imprime en terminal las variables
        self.drive_pub.publish(self.actuaction)
        

        print(
            "Velocidad Izq: " + str(v[0]) + " | Velocidad Der: " + str(v[1]) + "\n" +
            "Velocidad lineal: " + str(v_lineal) + "| Velocidad Angular: " + str(w_lineal) + "\n"
            "Trajectory X: " + str(self.trajectory_x) + " | Trajectory Y: " + str(self.trajectory_y) + "\n" +
            "Dx Trajectory: " + str(self.trajectory_dx) + " | Dy Trajectory: " + str(self.trajectory_dy) + "\n" +
            "Current X: " + str(self.current_x) + " | Current Y: " + str(self.current_y) + "\n")
        
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

    if delta < 1:
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

    #pdb.set_trace()

    # Iniciando objeto de punto descentralizado
    rospy.init_node("descentralized_point_simulation")
    dp = DescentralizedPoint()
    #dp.rate.sleep() 
    
    print("\n descentralized point simulation working :)")

    rospy.spin()

if __name__ == "__main__":
    main()
