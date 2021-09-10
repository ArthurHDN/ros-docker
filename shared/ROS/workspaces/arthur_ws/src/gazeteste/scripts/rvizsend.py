#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
import sys

# Frequencia de simulacao no gazebo
global freq
freq = 20.0  # Hz

# Estados do robo
global x_n, y_n, theta_n
x_n = 0.1  # posicao x atual do robo
y_n = 0.2  # posicao y atual do robo
theta_n = 0.001  # orientacao atual do robo

# Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global x_n, y_n, theta_n

    x_n = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_n = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta_n = euler[2]  # orientaco 'theta' do robo no mundo


    #Atualiza uma transformacao rigida entre os sistemas de coordenadas do mundo e do robo
    #Necessario para o rviz apenas
    br = tf.TransformBroadcaster()
    br.sendTransform((x_n, y_n, 0), (x_q, y_q, z_q, w_q), rospy.Time.now(), "/base_pose_ground_truth", "world")

    return

# Rotina primaria
def example():
    global freq
    global x_n, y_n, theta_n
    global pub_rviz_ref, pub_rviz_pose
    i = 0
    rospy.init_node("rviznode") #inicializa o no "este no"
    rospy.Subscriber("/odom", Odometry, callback_pose) #declaracao do topico onde sera lido o estado do robo

    #Inicializa os nos para enviar os marcadores para o rviz
    pub_rviz_ref = rospy.Publisher("/visualization_marker_ref", Marker, queue_size=1) #rviz marcador de velocidade de referencia
    pub_rviz_pose = rospy.Publisher("/visualization_marker_pose", Marker, queue_size=1) #rviz marcador de velocidade do robo

    #Define uma variavel que controlar[a a frequencia de execucao deste no
    rate = rospy.Rate(freq)

    sleep(0.2)

    # O programa do no consiste no codigo dentro deste while
    while not rospy.is_shutdown(): #"Enquanto o programa nao ser assassinado"

        # Incrementa o tempo
        i = i + 1
        time = i / float(freq)
        # Necessario para o rviz apenas
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "/map", "world")
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "base_laser_link", "base_pose_ground_truth")
        #Espera por um tempo de forma a manter a frequencia desejada
        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------



# Funcao inicial
if __name__ == '__main__':
    try:
        example()
    except rospy.ROSInterruptException:
        pass
