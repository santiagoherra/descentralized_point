#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np
import math
import traceback


from serial import SerialException
import errno

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int8, Int32, Float32
from roboclaw.roboclaw import Roboclaw
from mecanumrob_common.msg import EncTimed, WheelSpeed


#--------------------------------------------------#
#def debug_signal_handler(signal, frame):
#    """Permite depurar el programa en tiempo real con kill -USR1 <pid>"""
#    import pdb
#    pdb.set_trace()
#
#import signal
#signal.signal(signal.SIGUSR1, debug_signal_handler)
#--------------------------------------------------#


def clip(val, minval, maxval):
    r"""Acota una variable entre dos valores, p. ej. (-127,127)"""
    return max(min(val, maxval), minval)

def timeit(method):
    def timed(*args, **kw):

        inicio = rospy.Time.now()
        result = method(*args, **kw)
        fin = rospy.Time.now()

        t_total = (fin - inicio).to_nsec()
        rospy.logdebug("[profiling] %s : %d ns", method.__name__, t_total)

        return result
    return timed

class MecanumNode(object):
    r"""Nodo para una base del MecanumRob, con 2 controladores de motores Roboclaw.

    Attributes:
            baudrate (int): Velocidad de transmisión para la comunicacion serial,
                en baud/s, 115200 por defecto.
            port_front (str): Nombre/ruta del dispositivo para el motor delantero,
                por defecto `/dev/actuators/roboclaw_0`.
            port_back (str): Nombre/ruta del dispositivo para el motor trasero,
                por defecto `/dev/actuators/roboclaw_1`.
            frame_id (str): Nombre para el marco de referencia del robot, por
                defecto `mecanum`
            t_n (Rospy.Time): momento de la última lectura de los encoders.
            phi_n (numpy.float_): vector con la última lectura de los encoders.
            t_prev (Rospy.Time): momento de la penúltima lectura de los encoders.
            phi_prev (numpy.float_): vector con la penúltima lectura de los encoders.
            ppv (int): pulsos por vuelta de los encoders.

    Note:
        Para mantener el estándar con el modelo cinemático del MecanumRob, los
        vectores :attr:`phi_n` y :attr:`phi_prev` se definen con el siguiente
        orden `{NE, NW, SW, SE}`, además se define el vector de signo `_w_sign`
        para el cálculo de velocidad angular de las ruedas, de modo que un valor
        positivo se define según la ley de la mano derecha visto del frente del robot,
        (una rueda que avanza tiene velocidad positiva).

    """
    def __init__(self):

        # Crear el nodo y loggear informacion del Roboclaw
        rospy.init_node("MecanumRob_base", log_level=rospy.DEBUG)

        rosargs = rospy.myargv(argv=sys.argv)
        rospy.loginfo("Got the following node args: %s" % str(rosargs))

        # Obtener parametros
        self.baudrate = rospy.get_param("~baudrate", default=115200)
        self.port_front = rospy.get_param("~port_front", default="/dev/ttyACM0")
        self.port_back = rospy.get_param("~port_back", default="/dev/ttyACM1")
        self.frame_id = rospy.get_param("~frame_id", default="mecanum_base")
        self.ppv = rospy.get_param("~ppv", default=3415) #3415
        self.w_max = rospy.get_param("~w_max", default=12.35)
        self.pid_kp = rospy.get_param("~Kp", default=10.5932)
        self.pid_ki = rospy.get_param("~Ki", default=self.pid_kp/0.2445)
        self.pid_kd = rospy.get_param("~Kd", default=0.0)
        self.pid_ts = 1.0/60.0
        self.PWM_LIMIT = 100


        if len(rosargs) == 2 and rosargs[1] == '1':
            self.PID_mode = True
            rospy.loginfo("Base PID control on")
        else:
            self.PID_mode = False
            rospy.loginfo("Base PID control off")

        self.address = 0x80

        self.front = Roboclaw(comport=self.port_front, rate=self.baudrate)
        rospy.loginfo("Connected to front roboclaw on port %s", self.port_front)

        self.back = Roboclaw(comport=self.port_back, rate=self.baudrate)
        rospy.loginfo("Connected to back roboclaw on port %s", self.port_back)

        rospy.on_shutdown(self.shutdown)

        for motor in [self.front, self.back]:
            try:
                version = motor.ReadVersion(self.address)
                rospy.logdebug("Version " + str(repr(version[1])))
            except Exception as e:
                rospy.logwarn("Problem getting roboclaw version")
                rospy.logdebug(e)
                raise SerialException("Connectivity issue. Could not read version")

        # Crear publicaciones de los encoders
        #self.m1_enc_pub = rospy.Publisher("~encoder/front_left_enc", Int32, queue_size=0)
        #self.m2_enc_pub = rospy.Publisher("~encoder/front_right_enc", Int32, queue_size=0)
        #self.m3_enc_pub = rospy.Publisher("~encoder/back_left_enc", Int32, queue_size=0)
        #self.m4_enc_pub = rospy.Publisher("~encoder/back_right_enc", Int32, queue_size=0)
        self.enc_pub = rospy.Publisher("encoders", EncTimed, queue_size=0)
        self.phi_pub = rospy.Publisher("wheel_speed", numpy_msg(WheelSpeed), queue_size=0)
        self.pwm_pub = rospy.Publisher("pwm", EncTimed, queue_size=0)

        # Inicializar valores
        # Inicialmente los motores estan detenidos
        self.m1_pwm_cmd = 0
        self.m2_pwm_cmd = 0
        self.m3_pwm_cmd = 0
        self.m4_pwm_cmd = 0
        self.front.ResetEncoders(self.address)
        self.back.ResetEncoders(self.address)

        self.t_n = rospy.Time.now()
        self.phi_n = np.zeros(4, dtype=np.float64)
        self._w_sign = np.array([1, 1, 1, 1] , dtype=np.float64)

        # Estados para el controlador PID de velocidad
        self.phi_prime = np.zeros(4, dtype=np.float64)
        self.phi_prime_ref = np.zeros(4, dtype=np.float64)
        self.pwm_output = np.zeros(4, dtype=np.float64)
        self.pwm_min_1 = np.zeros(4, dtype=np.float64)
        self.current_error = np.zeros(4, dtype=np.float64)
        self.e_min_1 = np.zeros(4, dtype=np.float64)
        self.e_min_2 = np.zeros(4, dtype=np.float64)
        a = self.pid_kp + self.pid_ki*self.pid_ts/2.0 + self.pid_kd/self.pid_ts
        b = -self.pid_kp + self.pid_ki*self.pid_ts/2.0 - 2.0*self.pid_kd/self.pid_ts
        c = self.pid_kd/self.pid_ts
        self.pid_coef = np.array([1, a, b, c], dtype=np.float64)


        # Crear subscripciones, una para cada motor
        self.m1_pwm_sub = rospy.Subscriber("motor/NE_pwm", Int8, self.m1_pwm_callback, queue_size=1)
        self.m2_pwm_sub = rospy.Subscriber("motor/NW_pwm", Int8, self.m2_pwm_callback, queue_size=1)
        self.m3_pwm_sub = rospy.Subscriber("motor/SW_pwm", Int8, self.m3_pwm_callback, queue_size=1)
        self.m4_pwm_sub = rospy.Subscriber("motor/SE_pwm", Int8, self.m4_pwm_callback, queue_size=1)

        # Comandos para el robot
        self.command_sub = rospy.Subscriber("cmd_wheels", numpy_msg(WheelSpeed), self.cmd_wheel_callback, queue_size=1)


    def m1_pwm_callback(self, msg):
        # Verificacion de entrada
        self.m1_pwm_cmd = clip(msg.data, -127, 127)
        #rospy.logdebug("M1 PWM command = %d", self.m1_pwm_cmd)

    def m2_pwm_callback(self, msg):
        # Verificacion de entrada
        self.m2_pwm_cmd = clip(msg.data, -127, 127)
        #rospy.logdebug("M2 PWM command = %d", self.m2_pwm_cmd)

    def m3_pwm_callback(self, msg):
        # Verificacion de entrada
        self.m3_pwm_cmd = clip(msg.data, -127, 127)
        #rospy.logdebug("M3 PWM command = %d", self.m3_pwm_cmd)

    def m4_pwm_callback(self, msg):
        # Verificacion de entrada
        self.m4_pwm_cmd = clip(msg.data, -127, 127)
        #rospy.logdebug("M4 PWM command = %d", self.m4_pwm_cmd)

    def cmd_wheel_callback(self, msg):
        r"""Asigna el valor deseado de velocidad de las ruedas.
        Aplica el estándar de signos según la definición de :attr:`_w_sign`.
        """
        self.phi_prime_ref = self._w_sign * np.clip(msg.phi, -self.w_max, self.w_max)
        rospy.logdebug_throttle(2, "Wheel commands received: %.2f, %.2f, %.2f, %.2f" % (msg.phi[0], msg.phi[1], msg.phi[2], msg.phi[3]))


    def get_encoder_speed(self):
        r"""Lee las velocidades de los encoders.
        Los valores se dan en cuentas por segundo, en :attr:`enc_prime_n`
        """

        # TODO: La lectura parece hacerse de forma bloqueante, por lo que está
        # durando en promedio 12.3174 ms. Se podria mejorar considerablemente el
        # tiempo haciendolo no bloqueante, habria que cambiar el paquete roboclaw
        # y meterse con el uso de Serial. O tambien haciendo un hilo que se encargue
        # de la comunicacion serial.
        try:
            enc_m1 = self.front.ReadSpeedM2(self.address)
            enc_m2 = self.front.ReadSpeedM1(self.address)
            enc_m3 = self.back.ReadSpeedM1(self.address)
            enc_m4 = self.back.ReadSpeedM2(self.address)
        except OSError as e:
            rospy.logwarn("Roboclaw OSError: %d", e.errno)
            rospy.logdebug(e)

        #self.t_prev = self.t_n
        #self.t_n = rospy.Time.now()

        # self.phi_prev = self.phi_n
        # self.phi_n = np.array([enc_m1[1], enc_m2[1], enc_m3[1], enc_m4[1]], dtype=np.int32)

        # self.enc_prime_prev = self.enc_prime_n
        self.enc_prime_n = np.array([enc_m1[1], enc_m2[1], enc_m3[1], enc_m4[1]], dtype=np.int32)

        # norm = np.abs(self.enc_prime_n - self.enc_prime_prev)
        # for x in norm:
        #     if x > 7000:
        #         rospy.logerr("Unexpected speed values detected!")
        #         rospy.logerr(norm)

    def get_encoder_value(self):
        r"""Lee los encoders y guarda su valor en :attr:`enc_n`
        """

        # TODO: La lectura parece hacerse de forma bloqueante, por lo que está
        # durando en promedio 12.3174 ms. Se podria mejorar considerablemente el
        # tiempo haciendolo no bloqueante, habria que cambiar el paquete roboclaw
        # y meterse con el uso de Serial. O tambien haciendo un hilo que se encargue
        # de la comunicacion serial.
        try:
            enc_m1 = self.front.ReadEncM2(self.address)
            enc_m2 = self.front.ReadEncM1(self.address)
            enc_m3 = self.back.ReadEncM1(self.address)
            enc_m4 = self.back.ReadEncM2(self.address)
        except OSError as e:
            rospy.logwarn("Roboclaw OSError: %d", e.errno)
            rospy.logdebug(e)

        self.enc_n = np.array([enc_m1[1], enc_m2[1], enc_m3[1], enc_m4[1]], dtype=np.int32)


    def update_wheel_speed(self):
        """ Calcula la velocidad angular de las ruedas.

        Calcula la velocidad angular y aplica el estándar de signo definido por
        :attr:`_w_sign`. La forma de hacer el cálculo es:

        .. math:: \dot{\varphi}_n \approx \dot{Q}_n\cdot\frac{2\pi}{PPV}

        donde :math:`Q_i` es la lectura i-ésima de los encoders de cuadratura del Roboclaw.
        """
        self.phi_prime = self.enc_prime_n*2.0*math.pi/self.ppv

    def get_pwm_output_pid(self):
        """Aplica el lazo de control PID."""

        self.e_min_2 = self.e_min_1
        self.e_min_1 = self.current_error
        self.current_error = self.phi_prime_ref - self.phi_prime

        self.pwm_min_1 = self.pwm_output

        H = np.column_stack((self.pwm_min_1, self.current_error, self.e_min_1, self.e_min_2))
        self.pwm_output = np.clip(np.matmul(H, self.pid_coef), -self.PWM_LIMIT, self.PWM_LIMIT)

    #@timeit
    def send_pwm_cmd(self, pid=False):
        r"""Envía los comandos PWM a los motores.
        """
        # NOTE: en promedio 3.77628 ms enviando los comandos

        if pid:
            self.m1_pwm_cmd, self.m2_pwm_cmd, self.m3_pwm_cmd, self.m4_pwm_cmd = np.int_(self.pwm_output)
            rospy.logdebug_throttle(2, "PWM commands: %s" % str(self.pwm_output))

        try:
            if self.m1_pwm_cmd >= 0:
                self.front.ForwardM2(self.address, self.m1_pwm_cmd)
            else:
                self.front.BackwardM2(self.address, -self.m1_pwm_cmd)

            if self.m2_pwm_cmd >= 0:
                self.front.ForwardM1(self.address, self.m2_pwm_cmd)
            else:
                self.front.BackwardM1(self.address, -self.m2_pwm_cmd)

            if self.m3_pwm_cmd >= 0:
                self.back.ForwardM1(self.address, self.m3_pwm_cmd)
            else:
                self.back.BackwardM1(self.address, -self.m3_pwm_cmd)

            if self.m4_pwm_cmd >= 0:
                self.back.ForwardM2(self.address, self.m4_pwm_cmd)
            else:
                self.back.BackwardM2(self.address, -self.m4_pwm_cmd)

        except OSError as e:
            rospy.logwarn("Roboclaw OSError: %d", e.errno)
            rospy.logdebug(e)


    def run(self):
        r"""Lazo principal del nodo.

        Envia los comandos PWM a los motores, publica el valor de los encoders y
        la velocidad angular de las ruedas.
        """

        rospy.loginfo("Starting motor drive")

        r_time = rospy.Rate(60)

        while not rospy.is_shutdown():

            self.t_prev = self.t_n
            self.t_n = rospy.Time.now()

            try:
            	self.get_encoder_speed()
            	self.get_encoder_value()
            	self.update_wheel_speed()
            	self.get_pwm_output_pid()
            	self.send_pwm_cmd(self.PID_mode)
            	self._pub_wheel_speed()
            	self._pub_encoder_value()
            	self._pub_pwm()
            	r_time.sleep()
            except Exception as e:
                 if rospy.is_shutdown():
                     rospy.logwarn("Exception caught while shutting down")
                     rospy.logwarn(str(e))
                     self.__emergency_stop()
            
                 else:
                     rospy.logerr("Unhandled exception %s" % type(e))
                     rospy.logerr(e.args)
                     rospy.logerr(e.message)
                     traceback.print_exc()
                     pass
                     #raise e

    def _pub_encoder_value(self):
        """Publica el valor (raw) de los encoders"""

        msg = EncTimed()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.t_n
        msg.enc1, msg.enc2, msg.enc3, msg.enc4 = self.enc_n
        self.enc_pub.publish(msg)

    def _pub_wheel_speed(self):
        r"""Publica la velocidad angular de las ruedas [rad/s], aplicando el
        estándar de signo.
        """

        msg = WheelSpeed()
        msg.header.stamp = self.t_n
        msg.header.frame_id = self.frame_id
        msg.phi = self._w_sign * self.phi_prime
        self.phi_pub.publish(msg)

    def _pub_pwm(self):
        r"""Publica la acción de los motores (valor PWM raw)."""

        msg = EncTimed()
        msg.header.stamp = self.t_n
        msg.header.frame_id = self.frame_id
        msg.enc1, msg.enc2, msg.enc3, msg.enc4 = self.pwm_output
        self.pwm_pub.publish(msg)



    def __emergency_stop(self):
        """Apagar los motores y publicar velocidad 0 en todas las ruedas.
        """
        self.front.ForwardM1(self.address, 0)
        self.front.ForwardM2(self.address, 0)
        self.back.ForwardM1(self.address, 0)
        self.back.ForwardM2(self.address, 0)

        msg = WheelSpeed()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.phi = np.array([0., 0., 0., 0.], dtype=np.float64)
        self.phi_pub.publish(msg)
        rospy.sleep(0.5)

    def shutdown(self):
	"""Apaga el nodo"""

        rospy.loginfo("Shutting down")

        # No recibir mas comandos a los motores
        if hasattr(self, "m1_pwm_sub"):
            self.m1_pwm_sub.unregister()
        if hasattr(self, "m2_pwm_sub"):
            self.m2_pwm_sub.unregister()
        if hasattr(self, "m3_pwm_sub"):
            self.m3_pwm_sub.unregister()
        if hasattr(self, "m4_pwm_sub"):
            self.m4_pwm_sub.unregister()

        if hasattr(self, "command_sub"):
            self.command_sub.unregister()

        self.pwm_output = np.zeros(4)
        self.m1_pwm_cmd = 0
        self.m2_pwm_cmd = 0
        self.m3_pwm_cmd = 0
        self.m4_pwm_cmd = 0



        # Detener los motores
        self.__emergency_stop()
        rospy.sleep(0.2)
        try:
            self.__emergency_stop()
            rospy.loginfo("Closed Roboclaw serial connection")
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                self.__emergency_stop()
            except OSError as e:
                rospy.logfatal("Could not shutdown motors!!!!")
                rospy.logfatal(e)




if __name__ == "__main__":
    try:
        node = MecanumNode()
        node.run()
    except rospy.ROSInterruptException:
        pass



    rospy.loginfo("Exiting")


