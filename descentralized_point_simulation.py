#! /usr/bin/python3

# Librerias generales
import rospy
import numpy as np
import math
import tf

import pdb

# Twist es el tipo de mensaje por el que se publica la velocidad del robot
from geometry_msgs.msg import Twist

# Odometry es el tipo de mensaje por el que se lee la posicion del robot
from nav_msgs.msg import Odometry

### Parámetros ###
wheel_base      = 0.160/2  # Mitad de la distancia entre las ruedas (b)
lenght_g        = 0.138/2     # Distancia desde el centro al frente del robot (g)
KV_GAIN         = 1          # Ganancia de la velocidad lineal
KP_GAIN         = 1        # Ganancia proporcional de la posición
tiempo_ejecucion = 0.022     # Tiempo de reiteracion
DISTANCIA_UMBRAL = 8 # Distancia a la que el robot esta fuera de rango
DISTANCIA_ALTA = 1
DISTANCIA_MEDIA = 0.75
DISTANCIA_BAJA = 0.5
OFFSET_BAJO    = 15 # Offset que determina los puntos hacia adelante de la trayectoria
                       # que depende de el cambio de distancia entre los puntos.
OFFSET_MEDIO = 10
OFFSET_ALTO = 5
CONTINUIDAD = True     # Bandera que determina si una trayectoria es continua (True) o no (False)
contador_ciclo = 0
primer_ciclo = False

drive_topic         = "/cmd_vel" 
odom_topic         = "/odom" 

### Clase ###
class DescentralizedPoint:
    def __init__(self):
        self.wheelbase = wheel_base
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.trajectory_x = 0.0
        self.trajectory_y = 0.0

        self.trajectory_dx = 0.0
        self.trajectory_dy = 0.0

        self.current_target_idx = 0

        self.actuaction = Twist()

        self.drive_pub = rospy.Publisher(drive_topic, Twist, queue_size=100)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.punto_descentralizado, queue_size=100)

    def obtener_puntos(self):
        SKIP_ROWS = 1
        DELIMITER = ","
        WAYPOINTS_FILE  =  "/home/labautomatica05/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/descentralized_point/circulo_5m_300pts.csv"
        waypoints = np.loadtxt(WAYPOINTS_FILE, delimiter=DELIMITER, skiprows=SKIP_ROWS)
        return waypoints

    def obtener_trayectoria(self, waypoints):
        global primer_ciclo, contador_ciclo

        # Inicializar índice objetivo solo una vez
        if self.current_target_idx is None and not primer_ciclo:
            self.current_target_idx = self.encontrar_idx_mas_cercano(waypoints)
            primer_ciclo = True

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


    def punto_descentralizado(self, odom_msg):

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

        # Encontrar sguiente punto de la trayectoria en x y y
        next_trayectory_x = waypoints[self.current_target_idx + 1][0] # Offset para encontrar derivada
        next_trayectory_y = waypoints[self.current_target_idx + 1][1] # Offset para encontrar derivada

        # Derivada de la trayectoria
        #self.trajectory_dx = (next_trayectory_x - self.trajectory_x) / tiempo_ejecucion
        #self.trajectory_dy = (next_trayectory_y - self.trajectory_y) / tiempo_ejecucion

        self.trajectory_dx = 2
        self.trajectory_dy = 2

        # Componente proporcional a la velocidad de referencia
        vel_component = KV_GAIN * np.array([[self.trajectory_dx],
                                            [self.trajectory_dy]])

        # Componente proporcional a la posición
        error_x = self.trajectory_x - (self.current_x + lenght_g *
                                        np.cos(self.current_theta))

        error_y = self.trajectory_y - (self.current_y + lenght_g *
                                        np.sin(self.current_theta))

        krp_identidad = np.array([[KP_GAIN, 0],
                                [0, KP_GAIN]])

        e_krp = krp_identidad @ np.array([[error_x],
                                          [error_y]])

        # Resultado final del control cinemático
        control_cinematico = vel_component + e_krp

        # Matriz de conversión (cinemática inversa)
        B = (1 / lenght_g) * np.array([
            [lenght_g* np.cos(self.current_theta) + 0.5 * wheel_base * np.sin(self.current_theta),
              lenght_g * np.sin(self.current_theta) - 0.5 * wheel_base * np.cos(self.current_theta)],

            [lenght_g * np.cos(self.current_theta) - 0.5 * wheel_base * np.sin(self.current_theta),
              lenght_g * np.sin(self.current_theta) + 0.5 * wheel_base * np.cos(self.current_theta)]
        ])

        # Velocidades de referencia de las ruedas (lineales)
        v = B @ control_cinematico

        # Obtener velocidades de las dos ruedas
        v_izq = v[0]
        v_der = v[1]

        #Obtener velocidad lineal
        v_lineal = (v_izq + v_der) / 2

        # Asignar actuacion
        self.actuaction.linear.x = v_lineal

        # Velocidad angular 
        omega = ((v_der - v_izq) / (self.wheelbase))

        self.actuaction.angular.z = omega

        # Se publica el mensaje y se imprime en terminal las variables
        self.drive_pub.publish(self.actuaction)
        

        print(
            "Velocidad Izq: " + str(v_izq) + " | Velocidad Der: " + str(v_der) + "\n" +
            "Trajectory X: " + str(self.trajectory_x) + " | Trajectory Y: " + str(self.trajectory_y) + "\n" +
            "Dx Trajectory: " + str(self.trajectory_dx) + " | Dy Trajectory: " + str(self.trajectory_dy) + "\n" +
            "Current X: " + str(self.current_x) + " | Current Y: " + str(self.current_y))
        


def main():
    rospy.init_node("descentralized_point_simulation")
    DescentralizedPoint()
    
    print("\n descentralized point simulation working :)")

    rospy.spin()

if __name__ == "__main__":
    main()
