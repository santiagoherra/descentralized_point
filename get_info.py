#! /usr/bin/python3

##### Librerias y tipos de mensajes #####

# Librerias generales
import rospy
import tf
import csv

# Odometry es el tipo de mensaje por el que se lee la posicion del robot
from nav_msgs.msg import Odometry

##### Parametros #####
ODOM_TOPIC = "/odom"      # Topico del mensaje de odometria
CSV_FILE   = open("/home/labautomatica05/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/descentralized_point/odom_results.csv", "w")   


##### Algoritmo #####
class GetOdom():
    """
    Clase de extracción de información

    self.csvwriter : Escritor de archivo CSV
    self.odom_sub  : Topico suscriptor por el cual se lee la posicion actual del robot en 'x' y 'y'
    """
    def __init__(self):
        self.csvwriter = csv.writer(CSV_FILE) 
        self.odom_sub  = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_callback, queue_size=100)

    def odom_callback(self, msg):
        # Se guarda el Quaternio de orientacion en orientation_list
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        # Se determina los tres angulos de orientacion
        (roll, pitch, theta) = tf.transformations.euler_from_quaternion(orientation_list)
        
        # Se guarda la pose del robot
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        self.csvwriter.writerow([current_x, current_y, theta])

        return 0

def main():
    rospy.init_node('get_odom_node')
    getodom = GetOdom()

    print("\n\tSaving odom into CSV!")

    rospy.spin()



if __name__ == '__main__':
    main()
