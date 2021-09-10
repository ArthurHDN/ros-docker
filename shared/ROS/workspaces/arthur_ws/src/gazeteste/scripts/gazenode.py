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

global centro_x, centro_y, raio_x, raio_y, w
centro_x = 0
centro_y = 0
raio_x = 1
raio_y = 1
w = 0.1
# Frequencia de simulacao no gazebo
global freq
freq = 20.0  # Hz

# Velocidade de saturacao
global Usat
Usat = 10

# Estados do robo
global x_n, y_n, theta_n
x_n = 0.1  # posicao x atual do robo
y_n = 0.2  # posicao y atual do robo
theta_n = 0.001  # orientacao atual do robo

# Estados do robo
global x_goal, y_goal
x_goal = 0
y_goal = 0

# Relativo ao feedback linearization
global d
d = 0.20
# Relativo ao controlador (feedforward + ganho proporcional)
global Kp
Kp = 0.5




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


# ----------  ----------  ----------  ----------  ----------






# Rotina para a geracao da trajetoria de referencia
def refference_trajectory(time):
##MUDAR PARA CIRCULO
    global raio_x, raio_y, w
    x_ref = raio_x * cos(w*time) + centro_x
    y_ref = raio_y * sin(w*time) + centro_y
    Vx_ref = - raio_x*sin(w*time)*w
    Vy_ref = raio_x*cos(w*time)*w

    return (x_ref, y_ref, Vx_ref, Vy_ref)

# ----------  ----------  ----------  ----------  ----------




# Rotina para a geracao da entrada de controle
def trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref):
    global x_n, y_n, theta_n
    global Kp
    global Usat

    Ux = Vx_ref + Kp * (x_ref - x_n)
    Uy = Vy_ref + Kp * (y_ref - y_n)

    absU = sqrt(Ux ** 2 + Uy ** 2)

    if (absU > Usat):
        Ux = Usat * Ux / absU
        Uy = Usat * Uy / absU

    return (Ux, Uy)

# ----------  ----------  ----------  ----------  ----------




# Rotina feedback linearization
def feedback_linearization(Ux, Uy):
    global x_n, y_n, theta_n
    global d
    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy
    return (VX, WZ)

# ----------  ----------  ----------  ----------  ----------



# Rotina para piblicar informacoes no rviz
def send_marker_to_rviz(x_ref, y_ref, Ux, Uy):
    global x_n, y_n, theta_n
    global x_goal, y_goal
    global pub_rviz_ref, pub_rviz_pose

    #Inicializa mensagens do tipo Marker
    mark_ref = Marker()
    mark_vel = Marker()

    # Define um marcador para representar o alvo
    mark_ref.header.frame_id = "/world"
    mark_ref.header.stamp = rospy.Time.now()
    mark_ref.id = 0
    mark_ref.type = mark_ref.SPHERE
    mark_ref.action = mark_ref.ADD
    mark_ref.scale.x = 0.5
    mark_ref.scale.y = 0.5
    mark_ref.scale.z = 0.5
    mark_ref.color.a = 1.0
    mark_ref.color.r = 1.0
    mark_ref.color.g = 0.0
    mark_ref.color.b = 0.0
    mark_ref.pose.position.x = x_ref
    mark_ref.pose.position.y = y_ref
    mark_ref.pose.position.z = 0.25
    mark_ref.pose.orientation.x = 0
    mark_ref.pose.orientation.y = 0
    mark_ref.pose.orientation.z = 0
    mark_ref.pose.orientation.w = 1

    # Define um marcador para representar a "velocidade desejada"
    mark_vel.header.frame_id = "/world"
    mark_vel.header.stamp = rospy.Time.now()
    mark_vel.id = 1
    mark_vel.type = mark_vel.ARROW
    mark_vel.action = mark_vel.ADD
    mark_vel.scale.x = 0.5 * sqrt(Ux ** 2 + Uy ** 2)
    mark_vel.scale.y = 0.2
    mark_vel.scale.z = 0.2
    mark_vel.color.a = 0.5
    mark_vel.color.r = 1.0
    mark_vel.color.g = 1.0
    mark_vel.color.b = 1.0
    mark_vel.pose.position.x = x_n
    mark_vel.pose.position.y = y_n
    mark_vel.pose.position.z = 0.1
    quaternio = quaternion_from_euler(0, 0, atan2(Uy,Ux))
    mark_vel.pose.orientation.x = quaternio[0]
    mark_vel.pose.orientation.y = quaternio[1]
    mark_vel.pose.orientation.z = quaternio[2]
    mark_vel.pose.orientation.w = quaternio[3]

    #Publica os marcadores, que serao lidos pelo rviz
    pub_rviz_ref.publish(mark_ref)
    pub_rviz_pose.publish(mark_vel)

    return

# ----------  ----------  ----------  ----------  ----------

# Rotina primaria
def example():
    global freq
    global x_n, y_n, theta_n
    global pub_rviz_ref, pub_rviz_pose
    global x_goal, y_goal, raio_x, raio_y, centro_x, centro_y, w
    vel = Twist()
    i = 0
    pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1) #declaracao do topico para comando de velocidade
    rospy.init_node("gaze_node") #inicializa o no "este no"
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

        # Obtem a trajetoria de referencia "constante neste exemplo"
        [x_ref, y_ref, Vx_ref, Vy_ref] = refference_trajectory(time)

        #Aplica o controlador
        [Ux, Uy] = trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref)

        # Aplica o feedback linearization
        [V_forward, w_z] = feedback_linearization(Ux, Uy)


        # Publica as velocidades
        vel.linear.x = V_forward
        vel.angular.z = w_z
        pub_stage.publish(vel)

        # Atualiza uma transformacao rigida entre os sistemas de coordenadas do mapa e do mundo
        # Necessario para o rviz apenas
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "/map", "world")
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "base_laser_link", "base_pose_ground_truth")

        #Chama a funcao que envia os marcadores para o rviz
        send_marker_to_rviz(x_ref, y_ref, Ux, Uy)

        #Espera por um tempo de forma a manter a frequencia desejada
        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------



# Funcao inicial
if __name__ == '__main__':

    # Obtem os argumentos, no caso a posicao do alvo

    centro_x = int(sys.argv[1])
    centro_y = int(sys.argv[2])
    raio_x = int(sys.argv[3])
    raio_y = int(sys.argv[4])

    try:
        example()
    except rospy.ROSInterruptException:
        pass
