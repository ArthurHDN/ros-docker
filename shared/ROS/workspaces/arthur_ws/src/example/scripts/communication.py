#!/usr/bin/env python3
from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import tf
from example.srv import *

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'



# Rotina callback para a obtencao da pose do robo
def callback_pose(data, i):
    global x, y, theta
    x[i] = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y[i] = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta[i] = euler[2]  # orientaco 'theta' do robo no mundo




# Rotina primaria
def main_control(N_robots, groups, M_groups):
    freq = 50

    rospy.init_node('communication')
    # Estados do robo
    global x, y, theta
    x = N_robots*[0.1]  # posicao x atual do robo
    y = N_robots*[0.2]  # posicao y atual do robo
    theta = N_robots*[0.001]  # orientacao atual do robo 

    for robot_number in range(N_robots):
        rospy.Subscriber('robot_' + str(robot_number) + '/base_pose_ground_truth', Odometry, callback_pose, (robot_number))

    rate = rospy.Rate(freq)

    a_ant = 0

    collisions = 0

    while not rospy.is_shutdown():

        c = float(rospy.get_param('/c'))
        Rb = float(rospy.get_param('/Rb'))
        d = float(rospy.get_param('/d'))

        for i in range(N_robots):
            x_j_list = list()
            y_j_list = list()
            theta_j_list = list()
            in_a_circle_j_list = list()
            s_j_list = list()
            g_j_list = list()
            for j in range(i+1, N_robots):
                
                if i != j and (x[i] - x[j])**2 + (y[i] - y[j])**2 < (0.78*Rb)**2 and ( (x[i] + x[j]) > d/2 or (y[i] + y[j]) > d/2):
                    print(bcolors.FAIL + bcolors.BOLD + '*'*27)
                    print('COLLISION BETWEEN {} and {} at ({},{})'.format(i,j, (x[i] + x[j])/2, (y[i] + y[j])/2 ))
                    print('*'*27 + bcolors.ENDC)
                    collisions = collisions + 1

                if (x[i] - x[j])**2 + (y[i] - y[j])**2 < c**2:
                    try:
                        #rospy.loginfo('Robots {} and {} are close'.format(i,j))
                        get_h_i = rospy.ServiceProxy('/robot_' + str(i) + '/send_h_' + str(i),send_h_i)
                        h_i = get_h_i(i,j)
                        # rospy.loginfo('Robot {} info: {}'.format(i,respi.s[0]))
                        get_h_j = rospy.ServiceProxy('/robot_' + str(j) + '/send_h_' + str(j),send_h_i)
                        h_j = get_h_j(j,i)

                        set_h_i = rospy.ServiceProxy('/robot_' + str(i) + '/receive_h_' + str(i),receive_h_i)
                        resp = set_h_i(h_j.s, h_j.g, h_j.t, h_j.time, h_j.theta, h_j.m, h_j.ID)

                        set_h_j = rospy.ServiceProxy('/robot_' + str(j) + '/receive_h_' + str(j),receive_h_i)
                        resp = set_h_j(h_i.s, h_i.g, h_i.t, h_i.time, h_i.theta, h_i.m, h_i.ID)

                        # get_tj = rospy.ServiceProxy('/robot_' + str(j) + '/send_ti_' + str(j),send_theta_incircle)
                        # tj = get_tj(j,i)
                        # x_j_list.append(tj.x_i)
                        # y_j_list.append(tj.y_i)
                        # theta_j_list.append(tj.theta_i)
                        # in_a_circle_j_list.append(tj.in_a_circle_i)
                        # s_j_list.append(tj.s_i)
                        # g_j_list.append(tj.g_i)
                    except:
                        pass

            for j in range(N_robots):
                if i == j or (x[i] - x[j])**2 + (y[i] - y[j])**2 >= c**2:
                    continue
                try:
                    get_tj = rospy.ServiceProxy('/robot_' + str(j) + '/send_ti_' + str(j),send_theta_incircle)
                    tj = get_tj(j,i)
                    x_j_list.append(tj.x_i)
                    y_j_list.append(tj.y_i)
                    theta_j_list.append(tj.theta_i)
                    in_a_circle_j_list.append(tj.in_a_circle_i)
                    s_j_list.append(tj.s_i)
                    g_j_list.append(tj.g_i)
                except:
                    pass
            try:
                set_tj = rospy.ServiceProxy('/robot_' + str(i) + '/receive_tj_' + str(i),receive_theta_incircle)
                resp = set_tj(x_j_list, y_j_list, theta_j_list, in_a_circle_j_list, s_j_list, g_j_list)
            except:
                pass

        try:
        # if True:
            curves = list()
            S_max = list()
            S_min = list()
            for i in range(M_groups):
                S_max.append(1)
                S_min.append(10000)
            for i in range(N_robots):
                get_h_j = rospy.ServiceProxy('/robot_' + str(i) + '/send_h_' + str(i),send_h_i)
                h_j = get_h_j(0,0)
                curves.append(int(h_j.s[0]))
                if S_max[groups[i]] < curves[i]:
                    S_max[groups[i]] = curves[i]
                if S_min[groups[i]] > curves[i]:
                    S_min[groups[i]] = curves[i]

            a = 0
            for i in range(N_robots):
                group = groups[i]
                curve = curves[i]
                a_i = 0
                for k in range(M_groups):
                    if curve <= S_max[k] and curve >= S_min[k] and group != k:
                        # print('raise a')
                        a_i = a_i + 1
                a = a + a_i

            a = 100*float(a)/(N_robots*M_groups)
            if a != a_ant:
                print(bcolors.OKCYAN + bcolors.BOLD + '-'*9*3)
                print('SEGREGATION INDEX = {:.2f}%'.format(100-a))
                print('-'*9*3 + bcolors.ENDC)
                a_ant = a
                if collisions > 0:
                    print(bcolors.FAIL + bcolors.BOLD + '*'*27)
                    print('Collisions')
                    print('*'*27 + bcolors.ENDC)
            

        except:
            pass         
        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------



# Funcao inicial
if __name__ == '__main__':

    # N_robots = int(sys.argv[1])
    # M_groups = int(sys.argv[2])
    N_robots = int(len(sys.argv)) - 3
    groups = list()
    for aux in range(1, N_robots+1):
        groups.append(int(sys.argv[aux]))
    M_groups = max(groups)+1
    

    try:
        main_control(N_robots, groups, M_groups)
    except rospy.ROSInterruptException:
        pass
