#! /usr/bin/python3

# Librerias generales
import rospy
import numpy as np
import tf
import math

from pp_utils import get_current_waypoint

import pdb

# Twist es el tipo de mensaje por el que se publica la velocidad del robot
from geometry_msgs.msg import Twist

# Odometry es el tipo de mensaje por el que se lee la posicion del robot
from nav_msgs.msg import Odometry

### Parámetros ###
wheel_base      = 0.577  # Mitad de la distancia entre las ruedas (b)
R               = 0.511     # Radio de la rueda
lenght_g        = 0.625     # Distancia desde el centro al frente del robot (g)
KV_GAIN         = 10          # Ganancia de la velocidad lineal
KP_GAIN         = 5          # Ganancia proporcional de la posición
tiempo_ejecucion = 0.022     # Tiempo de reiteracion

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
        """
        Encuentra el waypoint más cercano a la posición actual y selecciona el siguiente en la lista como trayectoria.
        """

        # Posición actual
        current_position = np.array([self.current_x, self.current_y])

        # Inicializar variables para búsqueda lineal
        min_dist = float('inf')
        nearest_idx = 0

        # Búsqueda lineal del punto más cercano
        for i, point in enumerate(waypoints):
            dist = np.linalg.norm(current_position - point)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        # Offset: cuántos puntos adelante tomas como trayectoria
        offset = 20
        next_idx = (nearest_idx + offset) % len(waypoints)  # para hacer wraparound

        # Guardar la trayectoria seleccionada
        self.trajectory_x = waypoints[next_idx, 0]
        self.trajectory_y = waypoints[next_idx, 1]

        return None  # opcional, puede devolver (trajectory_x, trajectory_y) si prefieres


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

        self.obtener_trayectoria(waypoints)

        # Derivada de la trayectoria
        #self.trajectory_dx = (self.trajectory_x - self.current_x) / tiempo_ejecucion
        #self.trajectory_dy = (self.trajectory_y - self.current_y) / tiempo_ejecucion

        pdb.set_trace()

        self.trajectory_dx = 3
        self.trajectory_dy = 3

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

        #pdb.set_trace()

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
