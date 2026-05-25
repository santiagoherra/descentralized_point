#! /usr/bin/python
# -*- coding: utf-8 -*-

import csv
import math

import numpy as np
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from tf.transformations import euler_from_quaternion

### Parámetros ###
wheel_base = 0.276
lenght_g = 0.15
KV_GAIN = 0.9
KP_X_GAIN = 0.1
KP_Y_GAIN = 0.1
DISTANCIA_UMBRAL = 8
DISTANCIA = 0.4
CONTINUIDAD = True
V_LINEAL_MAX = 120

WAYPOINTS_FILE = (
    "/home/dif1/catkin_ws/src/MiniMecanum/mecanumrob_roboclaw/scripts/"
    "trayectoria_cuadrado.csv"
)

OFFSET_ALTO = 0
OFFSET_MEDIO = 0
OFFSET_BAJO = 0
DISTANCIA_ALTA = 0
DISTANCIA_MEDIA = 0
DISTANCIA_BAJA = 0

odom_topic = "/odom"
CONTROL_RATE_HZ = 60.0


def _clip_pwm(value):
    return max(min(int(value), V_LINEAL_MAX), -V_LINEAL_MAX)


def _orientacion_quaternion(q):
    if isinstance(q, Quaternion):
        quat = [q.x, q.y, q.z, q.w]
    else:
        quat = [q[0], q[1], q[2], q[3]]
    _, _, orientacion = euler_from_quaternion(quat)
    return orientacion


class Descentralized_point(object):
    """
    Clase de Punto Descentralizado:
    Este nodo calcula y publica PWM usando la odometría publicada por el nodo base.
    """

    def __init__(self):
        self.base_name = rospy.get_param("~BaseName", default="")

        # Estado del robot provisto por /odom
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_ready = False

        # Coordenadas de trayectoria y su derivada
        self.trajectory_x = 0.0
        self.trajectory_y = 0.0
        self.trajectory_dx = 0.0
        self.trajectory_dy = 0.0

        # Índice de trayectoria
        self.current_target_idx = OFFSET_ALTO

        self.file_path = (
            "/home/dif1/catkin_ws/src/MiniMecanum/mecanumrob_roboclaw/"
            "sensors_results/prueba3_26_6.csv"
        )
        with open(self.file_path, "wb") as file_obj:
            writer = csv.writer(file_obj)
            writer.writerow(["current_x", "current_y", "v_lineal", "w_lineal"])

        # Publicación PWM para ruedas motrices traseras
        self.izq_pub = rospy.Publisher("%s/motor/SW_pwm" % self.base_name, Int8, queue_size=0)
        self.der_pub = rospy.Publisher("%s/motor/SE_pwm" % self.base_name, Int8, queue_size=0)

        # /odom como única fuente de pose
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=10)

        # Control periódico desacoplado de wheel_speed.
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / CONTROL_RATE_HZ),
            self.punto_descentralizado,
        )

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = _orientacion_quaternion(msg.pose.pose.orientation)
        self.odom_ready = True

    def obtener_puntos(self):
        waypoints = np.loadtxt(WAYPOINTS_FILE, delimiter=",", skiprows=1)
        return waypoints

    def obtener_trayectoria(self, waypoints):
        punto_objetivo = waypoints[self.current_target_idx]
        dx = punto_objetivo[0] - self.current_x
        dy = punto_objetivo[1] - self.current_y
        distancia = np.sqrt(dx ** 2 + dy ** 2)

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
                    self.current_target_idx = OFFSET_ALTO
                else:
                    self.current_target_idx = len(waypoints) - 1

        self.trajectory_x = waypoints[self.current_target_idx][0]
        self.trajectory_y = waypoints[self.current_target_idx][1]

    def punto_descentralizado(self, _event=None):
        if not self.odom_ready:
            return None

        waypoints = self.obtener_puntos()
        self.obtener_trayectoria(waypoints)

        self.trajectory_dx = self.trajectory_x - self.current_x
        self.trajectory_dy = self.trajectory_y - self.current_y

        vel_component = KV_GAIN * np.array([[self.trajectory_dx], [self.trajectory_dy]])

        error_x = self.trajectory_x - (self.current_x + lenght_g * np.cos(self.current_theta))
        error_y = self.trajectory_y - (self.current_y + lenght_g * np.sin(self.current_theta))

        krp_identidad = np.array([[KP_X_GAIN, 0], [0, KP_Y_GAIN]])
        e_matriz = np.array([[error_x], [error_y]])
        e_krp = np.dot(krp_identidad, e_matriz)

        control_cinematico = vel_component + e_krp

        B = (1.0 / lenght_g) * np.array(
            [
                [
                    lenght_g * np.cos(self.current_theta)
                    + 0.5 * wheel_base * np.sin(self.current_theta),
                    lenght_g * np.sin(self.current_theta)
                    - 0.5 * wheel_base * np.cos(self.current_theta),
                ],
                [
                    lenght_g * np.cos(self.current_theta)
                    - 0.5 * wheel_base * np.sin(self.current_theta),
                    lenght_g * np.sin(self.current_theta)
                    + 0.5 * wheel_base * np.cos(self.current_theta),
                ],
            ]
        )

        v = np.dot(B, control_cinematico)
        v_izq_pwm = _clip_pwm(v[0] * 100.0)
        v_der_pwm = _clip_pwm(v[1] * 100.0)

        #mod_cine_direc = np.array([[0.5, 0.5], [-1.0 / wheel_base, 1.0 / wheel_base]])
        #v_w_lineal = np.dot(mod_cine_direc, v)
        #v_lineal = float(v_w_lineal[0])
        #w_lineal = float(v_w_lineal[1])

        print(
            "Velocidad Izq: %s | Velocidad Der: %s\n"
            "Trajectory X: %s | Trajectory Y: %s\n"
            "Dx Trajectory: %s | Dy Trajectory: %s\n"
            "Current X: %s | Current Y: %s\n"
            % (
                v_izq_pwm,
                v_der_pwm,
                self.trajectory_x,
                self.trajectory_y,
                self.trajectory_dx,
                self.trajectory_dy,
                self.current_x,
                self.current_y,
            )
        )

        self.izq_pub.publish(Int8(v_izq_pwm))
        self.der_pub.publish(Int8(-v_der_pwm))
        return None


def obtener_delta(ruta_csv):
    with open(ruta_csv, "r") as file_obj:
        total_lineas = sum(1 for _ in file_obj)

    cantidad_filas = max(total_lineas - 1, 0)
    delta = cantidad_filas * 0.05
    if delta == 0:
        return 1
    return delta


def definir_parametros(delta):
    global OFFSET_ALTO, OFFSET_MEDIO, OFFSET_BAJO, DISTANCIA_ALTA, DISTANCIA_MEDIA, DISTANCIA_BAJA

    DISTANCIA_ALTA = DISTANCIA
    DISTANCIA_MEDIA = DISTANCIA * (2.0 / 3.0)
    DISTANCIA_BAJA = DISTANCIA * (1.0 / 3.0)

    OFFSET_ALTO = round(delta)
    OFFSET_MEDIO = round(delta * 2.0 / 3.0)
    OFFSET_BAJO = round(delta * 1.0 / 3.0)


def main():
    delta = obtener_delta(WAYPOINTS_FILE)
    definir_parametros(delta)

    rospy.init_node("RoboclawTester", log_level=rospy.DEBUG)
    Descentralized_point()
    print("\n descentralized point real :) \n")
    rospy.spin()


if __name__ == "__main__":
    main()
