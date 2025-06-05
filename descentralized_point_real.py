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

wheel_base      = 0.276  # Distancia entre las ruedas (b)
R               = 0.0505
lenght_g        = 0.0505   # Distancia desde el punto medio del robot hacia el frente (g)
KV_GAIN         = 0.09          # Ganancia derivativa
KP_X_GAIN       = 0.40        # Ganancia proporcional
KP_Y_GAIN       = 0.40 
tiempo_ejecucion = 0.1     # Tiempo de reiteracion
DISTANCIA_UMBRAL = 8 # Distancia a la que el robot esta fuera de rango
DISTANCIA_ALTA = 1.5/2
DISTANCIA_MEDIA = 1/2
DISTANCIA_BAJA = 0.5/2
OFFSET_BAJO    = 15 # Offset que determina los puntos hacia adelante de la trayectoria
                       # que depende de el cambio de distancia entre los puntos.
OFFSET_MEDIO = 10
OFFSET_ALTO = 5
CONTINUIDAD = True     # Bandera que determina si una trayectoria es continua (True) o no (False)

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
        self.current_target_idx = 0
        
        # Frecuencia de muestreo
        self.rate = rospy.Rate(10) # Frecuencia a 10Hz

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
        WAYPOINTS_FILE  = "/home/labautomatica05/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/descentralized_point/circulo_5m_300pts.csv"
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
        global primer_ciclo

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

    def punto_descentralizado(self, encoders):

        # Ejecutar todas los metodos para poder correr el algoritmo
        # punto descentralizado.

        waypoints = self.obtener_puntos()

        self.modificar_posicion(encoders)

        self.obtener_trayectoria(waypoints)

        # Encontrar sguiente punto de la trayectoria en x y y
        next_trayectory_x = waypoints[self.current_target_idx + 1][0] # Offset para encontrar derivada
        next_trayectory_y = waypoints[self.current_target_idx + 1][1] # Offset para encontrar derivada

        # Derivada de la trayectoria
        self.trajectory_dx = (next_trayectory_x - self.trajectory_x) / tiempo_ejecucion
        self.trajectory_dy = (next_trayectory_y - self.trajectory_y) / tiempo_ejecucion

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

        # Obtener velocidades de las dos ruedas en PWM
        v_izq_PWM = v[0] * 100
        v_der_PWM = v[1] * 100

        # Imprimir los mensajes
        print(
            "Velocidad Izq: " + str(v[0]) + " | Velocidad Der: " + str(v[1]) + "\n" +
            "Trajectory X: " + str(self.trajectory_x) + " | Trajectory Y: " + str(self.trajectory_y) + "\n" +
            "Dx Trajectory: " + str(self.trajectory_dx) + " | Dy Trajectory: " + str(self.trajectory_dy) + "\n" +
            "Current X: " + str(self.current_x) + " | Current Y: " + str(self.current_y) + "\n")

        # Se publican los mensajes
        self.izq_pub.publish(Int8(v_izq_PWM))
        self.der_pub.publish(Int8(-v_der_PWM))

        return None


def main():
    rospy.init_node("RoboclawTester", log_level=rospy.DEBUG)
    prueba = Descentralized_point()
    prueba.rate.sleep()

    print("\n descentralized point real :) \n")

    rospy.spin()

if __name__ == "__main__":
    main()

        

        




        
        
