#! /usr/bin/python3

##### Librerias y tipos de mensajes #####

# Librerias generales
import rospy
import numpy as np
import math

# Int8 es el tipo de mensaje por el que se publica el valor de PWM de los motores del robot
from std_msgs.msg import Int8

# WheelSpeed es el tipo de mensaje por el que se lee las velocidades angulares de las ruedas del robot
from mecanumrob_common.msg import WheelSpeed

### Parameters ###

wheel_base      = 0.276/2  # Mitad de la distancia entre las ruedas (b)
R               = 0.0505   # Radio de la rueda del robot
lenght_g        = 0.0505   # Distancia desde el punto medio del robot hacia el frente (g)
KV_GAIN         = 1        # Ganancia de la velocidad lineal
KP_GAIN         = 1        # Ganancia proporcional de la velocidad lineal
tiempo_muestreo = 0.0022   # Tiempo de muestreo del robot
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

        # Variables a publicar y suscribir

        # Velocidades de las ruedas izquierda y derecha
        self.izq_pub = rospy.Publisher("%s/motor/SW_pwm" % self.base_name,
                                        Int8, queue_size=0)

        self.der_pub = rospy.Publisher("%s/motor/SE_pwm" % self.base_name,
                                        Int8, queue_size=0)
        
        # Posicion
        self.enc_sub =  rospy.Subscriber("%s/wheel_speed" % self.base_name,
                                           WheelSpeed, queue_size = 50)
        
    def obtener_trayectoria(self):
        SKIP_ROWS = 1
        DELIMITER = ","
        WAYPOINTS_FILE  = "./trayectoria_circulo.csv"
        waypoints = np.loadtxt(WAYPOINTS_FILE, delimiter=DELIMITER, skiprows=SKIP_ROWS)
        return waypoints
    
    def modificar_posicion(self, encoders):

        # Velocidad angular de la rueda izquierda y derecha
        w_der = encoders.phi[3]
        w_izq = -encoders.phi[2]

        # Velocidad lineal de la rueda izquierda y derecha
        v_der =  (w_der) * R 
        v_izq =  (w_izq) * R

        # Velocidad angular del robot a la mitad
        omega = ((v_der-v_izq) / (self.wheelbase))

        # Actualizando el angulo de orientacion del robot
        self.current_theta += omega * tiempo_muestreo

        # Actualizando la posicion del robot.

        #Preguntar esto
        self.current_x += ((v_der+v_izq)/2) * math.cos(self.current_theta) * tiempo_muestreo
        self.current_y += ((v_der+v_izq)/2) * math.sin(self.current_theta) * tiempo_muestreo
    
    def punto_descentralizado(self):

        # Componente proporcional a la velocidad de referencia
        vel_component = KV_GAIN * np.array([self.trajectory_dx,
                                            self.trajectory_dy])

        # Componente proporcional a la posición
        error_x = self.trajectory_x - (self.current_x + lenght_g *
                                        np.cos(self.current_theta))

        error_y = self.trajectory_y - (self.current_y + lenght_g *
                                        np.sin(self.current_theta))

        krp_identidad = np.array([[KP_GAIN, 0],
                                [0, KP_GAIN]])

        e_krp = krp_identidad @ np.array([error_x, error_y])

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

        # Obtener velocidades de las dos ruedas.
        v_correc_izq = v[0]
        v_correc_der = v[1]

        

        




        
        
