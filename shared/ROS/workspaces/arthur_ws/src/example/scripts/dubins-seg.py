#!/usr/bin/env python3
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from math import sqrt, atan2, exp, log, atan, cos, sin, acos, pi, asin, atan2, copysign
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
import sys
from example.srv import *
import numpy as np

# Estados do robo
global x_i, y_i, theta_i
x_i = 0.1  
y_i = 0.2  
theta_i = 0.001  

global time
time = 0

class h():
    s = list()
    g = list()
    t = list()
    time = list()
    theta = list()
    m = list()
    ID = list()

global h_i, h_j
h_i = h()
h_j = h()

class ti():
    x_j = list()
    y_j = list()
    theta_j = list()
    in_a_circle_j = list()
    s_j = list()
    g_j = list()

global tij
tij = ti()

global in_a_curve
in_a_curve = 1

global robot_group
robot_group = 0

def ang_diff(a,b, type):
    if type == 'min':
        return min(2*pi - (a-b)%2*pi, (a-b)%2*pi)
    else:
        return max(2*pi - (a-b)%2*pi, (a-b)%2*pi)

def dr(i_group, j_group):
    ###SPACE_FACTOR = 1
    SPACE_FACTOR = 1.1
    return (SPACE_FACTOR*int(i_group == j_group) + 1)

def Sr(Rb, alfa, d):
    beta = acos(1 - 2*(Rb**2)/((alfa*d)**2))
    return beta*alfa*d
         
# Rotina callback para a obtencao da pose do robo
def callback_pose(data, inital_pos):
    NOISE_VAR = 0.5

    global x_i, y_i, theta_i
    x_ii = data.pose.pose.position.x  #+ inital_pos[0]# posicao 'x' do robo no mundo
    y_ii = data.pose.pose.position.y  #+ inital_pos[1]# posicao 'y' do robo no mundo

    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta_ii = euler[2] #+ inital_pos[2]

    x_i = x_ii + float(np.random.normal(0,NOISE_VAR ,1))
    y_i = y_ii + float(np.random.normal(0,NOISE_VAR ,1))
    theta_i = theta_ii + float(np.random.normal(0,NOISE_VAR,1))


    # if theta_ii > pi:
    #     theta_i = theta_ii - 2*pi# orientaco 'theta' do robo no mundo
    # elif theta_ii <= -pi:
    #     theta_i = theta_ii + 2*pi
    # else:
    #     theta_i = theta_ii

    # x_i = x_ii*cos(inital_pos[2]) - y_ii*sin(inital_pos[2])
    # y_i = x_ii*sin(inital_pos[2]) + y_ii*cos(inital_pos[2])

    # x_i = -(x_ii*sin(inital_pos[2]) + y_ii*cos(inital_pos[2])) 
    # y_i = x_ii*cos(inital_pos[2]) - y_ii*sin(inital_pos[2])

    # print('---')
    # print(inital_pos[0], inital_pos[1], inital_pos[2]*180/pi)

    # print(x_i, y_i, theta_i*180/pi)

# Rotina callback para a obtencao da pose do robo
def callback_time(data):
    global time
    time = data.clock.secs*1000 + data.clock.nsecs/1000000 #milisecs
    #time = data.clock.secs 

def handle_receive_h_j(req):
    #global h_j
    global h_i
    h_j.s = req.s
    h_j.g = req.g
    h_j.t = req.t
    h_j.time = req.time
    h_j.theta = req.theta
    h_j.m = req.m
    h_j.ID = req.ID
    for aux in range(len(h_j.ID)):
        if h_j.ID[aux] in h_i.ID:
            aux2 = h_i.ID.index(h_j.ID[aux])
            if h_i.time[aux2] < h_j.time[aux] or aux == 0:
                h_i.s[aux2] = h_j.s[aux]
                h_i.g[aux2] = h_j.g[aux] # Nao necessario
                h_i.t[aux2] = h_j.t[aux]
                h_i.time[aux2] = h_j.time[aux]
                h_i.theta[aux2] = h_j.theta[aux]
                h_i.m[aux2] = h_j.m[aux]
                h_i.ID[aux2] = h_j.ID[aux] # Nao necessario
        else:
            h_i.s.append(h_j.s[aux])
            h_i.g.append(h_j.g[aux])
            h_i.t.append(h_j.t[aux])
            h_i.time.append(h_j.time[aux])
            h_i.theta.append(h_j.theta[aux])
            h_i.m.append(h_j.m[aux])
            h_i.ID.append(h_j.ID[aux])
    return receive_h_iResponse(1,1)

def handle_send_h_i(req):
    global h_i
    return send_h_iResponse(h_i.s, h_i.g, h_i.t, h_i.time, h_i.theta, h_i.m, h_i.ID) 

def handle_receive_tj(req):
    global tij
    tij.x_j = req.x_j
    tij.y_j = req.y_j
    tij.theta_j = req.theta_j
    tij.in_a_circle_j = req.in_a_circle_j
    tij.s_j = req.s_j
    tij.g_j = req.g_j
    return receive_theta_incircleResponse(1,1)

def handle_send_ti(req):
    global theta_i, in_a_curve, robot_group
    return send_theta_incircleResponse(x_i, y_i, theta_i, in_a_curve, h_i.s[0], robot_group) 

# Rotina primaria
def main_control(robot_number, robot_group, inital_pos):
    global time, h_i, h_j, in_a_curve, tij
    freq = 10

    vel = Twist()

    # Publish and Subscribe
    rospy.init_node('controller')
    pub_vel = rospy.Publisher('/robot_' + robot_number + '/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/robot_' + robot_number + '/base_pose_ground_truth', Odometry, callback_pose, inital_pos)
    rospy.Subscriber('/clock', Clock, callback_time)

    # Services
    ss = rospy.Service('send_h_'+robot_number, send_h_i, handle_send_h_i)
    sr = rospy.Service('receive_h_'+robot_number, receive_h_i, handle_receive_h_j)
    ts = rospy.Service('send_ti_'+robot_number, send_theta_incircle, handle_send_ti)
    tr = rospy.Service('receive_tj_'+robot_number, receive_theta_incircle, handle_receive_tj)
    
    rate = rospy.Rate(freq)

    G = 0
    H = 1
    theta_error_int = 0
    theta_error_prev = 0
    Rb = float(rospy.get_param('/Rb'))
    d = float(rospy.get_param('/d'))
    c = float(rospy.get_param('/c'))
    ref_velocity = float(rospy.get_param('/ref_vel'))

    xc = 0
    yc = 0
    G2 = 1
    G_t = 1
    H_t = 0
    alpha_t = 0
    theta_error_int_t = 0
    theta_error_prev_t = 0

    move_inwards = False
    move_outwards = False

    WAIT_TIME_SEC = 5
    ###WAIT_TIME_CHANGE = 0.50 
    WAIT_TIME_CHANGE = 0.10
    CONVERGENCE_LIMIT = 0.19

    time_wait_change = 0

    while not rospy.is_shutdown():

        Rb = float(rospy.get_param('/Rb'))
        d = float(rospy.get_param('/d'))
        c = float(rospy.get_param('/c'))
        ref_velocity = float(rospy.get_param('/ref_vel'))

        if time < WAIT_TIME_SEC*1000 + 100*int(robot_number):
            in_a_curve = 1
            lap = False
            theta_s = theta_i
            time_s = time/1000

            closest_contour_line = round(sqrt(x_i**2 + y_i**2)/d)
            if closest_contour_line == 0:
                closest_contour_line = 1
            r_i = closest_contour_line*d*((-1)**(closest_contour_line))
            h_i.s = [closest_contour_line]
            h_i.g = [robot_group]
            h_i.t = [time]
            h_i.time = [time]
            h_i.theta = [theta_i]
            h_i.m = [0]
            h_i.ID = [int(robot_number)]

            vel.linear.x = 0
            vel.angular.z = 0
            pub_vel.publish(vel)
            rate.sleep()
            continue
        
        r_i = closest_contour_line*d*((-1)**(closest_contour_line))
        
        if in_a_curve == 1:

            if ( abs(theta_s - theta_i) <= 0.01 or abs(theta_s - theta_i) >= (2*pi-0.01) ) and time/1000 - time_s > 60*h_i.s[0]:
                lap = True
                time_s = time/1000
                theta_s = theta_i
                # print('Robot', robot_number, 'lapped')

            for aux in range(1, len(h_i.ID)):
                if h_i.ID[aux] == h_i.ID[0]:
                    continue

                if (h_i.s[0] - 1 == h_i.s[aux]) and (h_i.g[0] != h_i.g[aux]):
                    lap = False
                    theta_s = theta_i
                    time_s = time/1000

                if (h_i.s[0] > h_i.s[aux]) and (h_i.g[0] == h_i.g[aux]):
                    aux_is_alone = True
                    for aux2 in range(len(h_i.ID)):
                        if aux != aux2 and (h_i.s[aux] == h_i.s[aux2]) and (h_i.g[aux] != h_i.g[aux2]):
                            aux_is_alone = False
                    if aux_is_alone:
                        move_inwards = True

                if (h_i.s[0] == h_i.s[aux]) and (h_i.g[0] != h_i.g[aux]) and not move_inwards:
                    if (h_i.t[0] < h_i.t[aux]) or (h_i.t[0] == h_i.t[aux] and h_i.theta[0] > h_i.theta[aux]):
                        move_outwards = True

                if (h_i.m[aux] == 1) and (h_i.s[0] == h_i.s[aux] + 1):
                    move_outwards = True

            if h_i.s[0] > 1 and lap:
                move_inwards = True
                lap = False
                time_s = time/1000
                theta_s = theta_i

        if move_outwards:
            move_inwards = False

        if h_i.m[0] != int(move_outwards):
            h_i.time[0] = time
            h_i.m[0] = int(move_outwards)

        # if robot_number == '4':
        #     print('Robot', robot_number, 'in_a_curve =', in_a_curve, 'intention =', move_inwards, move_outwards)

        ###SAFETY_SPACE = 0
        SAFETY_SPACE = 0
        alfa = h_i.s[0]
        closest_robot_dist = 100000
        closest_robot = 'self'
        for aux in range(len(tij.in_a_circle_j)):
            # current_dist = sqrt((x_i - tij.x_j[aux])**2 + (y_i - tij.y_j[aux])**2)

            # if closest_robot_dist >= current_dist:
            #     closest_robot_dist = current_dist
            #     closest_robot = aux

            if move_inwards and h_i.s[0] == tij.s_j[aux]+1 and in_a_curve == 0.5:
                if ang_diff(theta_i-pi/2,tij.theta_j[aux]+pi/2,'min') <= (pi*d/2 + SAFETY_SPACE + dr(h_i.g[0], tij.g_j[aux])*Sr(Rb, alfa, d))/alfa*d:
                    move_inwards = False
                    in_a_curve = 1

            if move_outwards and h_i.s[0] == tij.s_j[aux]-1 and in_a_curve == 0.5:
                if ang_diff(theta_i-pi/2,tij.theta_j[aux]+pi/2,'min') <= (pi*d/2 + SAFETY_SPACE + dr(h_i.g[0], tij.g_j[aux])*Sr(Rb, alfa, d))/alfa*d:
                    move_outwards = False
                    in_a_curve = 1
                    # if robot_number == '4':
                    #     print('Robot', robot_number, 'outward will canceled, collission')

            if tij.in_a_circle_j[aux] == 0 and in_a_curve >= 0.5:
                move_inwards = False
                move_outwards = False
                in_a_curve = 1
                break

            if tij.in_a_circle_j[aux] == 0.5 and tij.theta_j[aux] >= theta_i and in_a_curve >= 0.5: 
                move_inwards = False
                move_outwards = False
                in_a_curve = 1
                break

        # Closed loop control
        Kp = 0.8
        Ki = 0.0
        Kd = 0.0
        KG = 8.0

        xp = x_i/abs(r_i)
        yp = y_i/abs(r_i)

        alpha = xp**2 + yp**2 - 1
        G = -2*atan(KG*alpha)/pi
        H = copysign(1,r_i)*sqrt(1-G**2+1e-6)  

        move_in_out_int = int(move_outwards) - int(move_inwards)

        if move_in_out_int != 0 and in_a_curve == 1:
            # print('Robot', robot_number, 'waiting to move', move_in_out_int)
            in_a_curve = 0.5
            time_wait_change = time/1000    

        if move_inwards and time/1000 - time_wait_change > WAIT_TIME_CHANGE:
            # print('Robot', robot_number, 'moving inwards')
            in_a_curve = 0
            move_inwards = False
            lap = False
            time_s = time/1000
            theta_s = theta_i
            closest_contour_line = closest_contour_line - 1
            if closest_contour_line == 0:
                closest_contour_line = 1
            r_i = closest_contour_line*d*((-1)**(closest_contour_line))
            xc = x_i*(1 - d/(2*sqrt(x_i**2 + y_i**2)))
            yc = y_i*(1 - d/(2*sqrt(x_i**2 + y_i**2)))
            dirc = copysign(1,-r_i)

            xp = x_i/abs(r_i)
            yp = y_i/abs(r_i)

            alpha = xp**2 + yp**2 - 1
            G = -2*atan(KG*alpha)/pi

        if move_outwards and time/1000 - time_wait_change > WAIT_TIME_CHANGE:
            # print('Robot', robot_number, 'moving outwards')
            in_a_curve = 0
            move_outwards = False
            lap = False
            time_s = time/1000
            theta_s = theta_i
            closest_contour_line = closest_contour_line + 1
            r_i = closest_contour_line*d*((-1)**(closest_contour_line))
            xc = x_i*(1 + d/(2*sqrt(x_i**2 + y_i**2)))
            yc = y_i*(1 + d/(2*sqrt(x_i**2 + y_i**2)))
            dirc = copysign(1,r_i)

            xp = x_i/abs(r_i)
            yp = y_i/abs(r_i)

            alpha = xp**2 + yp**2 - 1
            G = -2*atan(KG*alpha)/pi

        if in_a_curve == 0:

            xp = (x_i - xc)/(d/2)
            yp = (y_i - yc)/(d/2)

            KG_t = 5

            alpha_t = xp**2 + yp**2 - 1
            G_t = -2*atan(KG_t*alpha_t)/pi
            H_t = dirc*sqrt(1-G_t**2+1e-6)

            grad_alpha_x = 2*xp/(d/2)
            grad_alpha_y = 2*yp/(d/2)
            norm_grad_alpha = sqrt(grad_alpha_x**2 + grad_alpha_y**2) + 1e6
            f_x = (G_t*grad_alpha_x - H_t*grad_alpha_y)/norm_grad_alpha
            f_y = (G_t*grad_alpha_y + H_t*grad_alpha_x)/norm_grad_alpha
            theta_ref = atan2(f_y, f_x )
            theta_error_t = sin(theta_ref - theta_i)
            theta_error_t = asin(theta_error_t)

            theta_error_int_t = theta_error_int_t + theta_error_t
            theta_error_diff_t = theta_error_t - theta_error_prev_t
            theta_error_prev_t = theta_error_t

            control_input = Kp*theta_error_t + Ki*theta_error_int_t + Kd*theta_error_diff_t

            if abs(G) < CONVERGENCE_LIMIT:
                h_i.s = [closest_contour_line]
                h_i.g = [robot_group]
                h_i.t = [time]
                h_i.time = [time]
                h_i.theta = [theta_i]
                h_i.m = [0]
                h_i.ID = [int(robot_number)]

                move_outwards = False
                move_inwards = False

                theta_s = theta_i
                time_s = time/1000
                lap = False
                in_a_curve = 1
                # print('Robot', robot_number, 'arrived')

        else:

            xp = x_i/abs(r_i)
            yp = y_i/abs(r_i)

            alpha = xp**2 + yp**2 - 1
            G = -2*atan(KG*alpha)/pi
            
            H = copysign(1,r_i)*sqrt(1-G**2+1e-6)
            grad_alpha_x = 2*xp/abs(r_i)
            grad_alpha_y = 2*yp/abs(r_i)
            norm_grad_alpha = sqrt(grad_alpha_x**2 + grad_alpha_y**2) + 1e6
            f_x = (G*grad_alpha_x - H*grad_alpha_y)/norm_grad_alpha
            f_y = (G*grad_alpha_y + H*grad_alpha_x)/norm_grad_alpha
            theta_ref = atan2(f_y, f_x )
            theta_error = sin(theta_ref - theta_i)
            theta_error = asin(theta_error)

            theta_error_int = theta_error_int + theta_error
            theta_error_diff = theta_error - theta_error_prev
            theta_error_prev = theta_error
            control_input = Kp*theta_error + Ki*theta_error_int + Kd*theta_error_diff

        R = 0.9*d/2

        if control_input >   1/R:
            control_input =  1/R
        if control_input <  -1/R:
            control_input = -1/R

        # Publica as velocidades
        vel.linear.x = ref_velocity
        vel.angular.z = control_input
        pub_vel.publish(vel)

        #Espera por um tempo de forma a manter a frequencia desejada
        rate.sleep()

# ---------- !! ---------- !! ---------- !! ---------- !! ----------
# Funcao inicial
if __name__ == '__main__':
	#global robot_group
	robot_number = str(sys.argv[1])
	robot_group  = int(sys.argv[2])
	x_initial = float(sys.argv[3])
	y_initial = float(sys.argv[4])
	theta_initial = float(sys.argv[6])*pi/180
	inital_pos = (x_initial, y_initial, theta_initial)
	try:
		main_control(robot_number, robot_group, inital_pos)
	except rospy.ROSInterruptException:
		pass