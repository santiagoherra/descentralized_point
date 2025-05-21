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
wheel_base      = 0.276 / 2  # Mitad de la distancia entre las ruedas (b)
R               = 0.0505     # Radio de la rueda
lenght_g        = 0.0505     # Distancia desde el centro al frente del robot (g)
KV_GAIN         = 0.05          # Ganancia de la velocidad lineal
KP_GAIN         = 0.05          # Ganancia proporcional de la posición
tiempo_ejecucion = 0.022     # Tiempo de reiteracion
DISTANCIA_UMBRAL = 1.5  # Distancia a la que el robot esta fuera de rango
OFFSET           = 10  # Offset que determina los puntos hacia adelante de la trayectoria
                       # que depende de el cambio de distancia entre los puntos.
CONTINUIDAD = True     # Bandera que determina si una trayectoria es continua (True) o no (False)

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
        WAYPOINTS_FILE  =  "/home/labautomatica05/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/circulo_5m_300pts.csv"
        waypoints = np.loadtxt(WAYPOINTS_FILE, delimiter=DELIMITER, skiprows=SKIP_ROWS)
        return waypoints

    def obtener_trayectoria(self, waypoints):
        # Si no existe aún un índice de objetivo, inicializarlo al punto más cercano
        if not hasattr(self, 'current_target_idx') or self.current_target_idx is None:
            self.current_target_idx = self.encontrar_idx_mas_cercano(waypoints)
        
        # Obtener el punto objetivo actual
        punto_objetivo = waypoints[self.current_target_idx]
        
        # Calcular la distancia a ese punto
        dx = punto_objetivo[0] - self.current_x
        dy = punto_objetivo[1] - self.current_y
        distancia = np.sqrt(dx**2 + dy**2)
        
        if distancia <= DISTANCIA_UMBRAL:
            siguiente_idx = self.current_target_idx + OFFSET
            if siguiente_idx < len(waypoints):
                self.current_target_idx = siguiente_idx
            else:
                if CONTINUIDAD:
                    self.current_target_idx = OFFSET  # reiniciar con salto desde inicio
                else:
                    self.current_target_idx = len(waypoints) - 1  # quedarse al final


        # Actualizar los valores de la trayectoria
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
        next_trayectory_x = waypoints[self.current_target_idx][0]
        next_trayectory_y = waypoints[self.current_target_idx][1]

        # Derivada de la trayectoria
        self.trajectory_dx = (self.trajectory_x - next_trayectory_x) / tiempo_ejecucion
        self.trajectory_dy = (self.trajectory_y - next_trayectory_y) / tiempo_ejecucion

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

        # Velocidad angular del robot a la mitad
        #omega = ((v_der - v_izq) / (self.wheelbase))

        omega = 1/R * v_lineal

        self.actuaction.angular.z = omega

        # Se publica el mensaje y se imprime en terminal la posicion
        self.drive_pub.publish(self.actuaction)
        print("Current x: {}    Current y: {}".format(self.current_x, self.current_y))



def main():
    rospy.init_node("descentralized_point_simulation")
    DescentralizedPoint()
    
    print("\n descentralized point simulation working :)")

    rospy.spin()

if __name__ == "__main__":
    main()
