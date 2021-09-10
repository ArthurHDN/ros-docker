#!/usr/bin/env python
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
in_a_curve = True

global robot_group
robot_group = 0

def ang_diff(a,b, type):
    if type == 'min':
        return min(2*pi - (a-b)%2*pi, (a-b)%2*pi)
    else:
        return max(2*pi - (a-b)%2*pi, (a-b)%2*pi)

def dr(i_group, j_group):
    SPACE_FACTOR = 1
    return (SPACE_FACTOR*int(i_group == j_group) + 1)

def Sr(Rb, alfa, d):
    beta = acos(1 - 2*(Rb**2)/((alfa*d)**2))
    return beta*alfa*d
         
# Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global x_i, y_i, theta_i
    x_i = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_i = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta_i = euler[2]  # orientaco 'theta' do robo no mundo

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
def main_control(robot_number, robot_group):
    global time, h_i, h_j, in_a_curve, tij
    freq = 10

    vel = Twist()

    # Publish and Subscribe
    rospy.init_node('controller')
    pub_vel = rospy.Publisher('/robot_' + robot_number + '/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/robot_' + robot_number + '/base_pose_ground_truth', Odometry, callback_pose)
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

    move_inwards = False
    move_outwards = False
    while not rospy.is_shutdown():

        WAIT_TIME_SEC = 5
        CONVERGENCE_LIMIT = 0.3

        if time < WAIT_TIME_SEC*1000 + 50*int(robot_number):
            in_a_curve = True
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

        # myself = h_i.ID.index(int(robot_number))
        # if myself != 0:
        #     print("ERROR: myself != 0")

        Rb = float(rospy.get_param('/Rb'))
        d = float(rospy.get_param('/d'))
        c = float(rospy.get_param('/c'))
        ref_velocity = float(rospy.get_param('/ref_vel'))

        # in_a_curve = (abs(G) < 0.05)
        r_i = closest_contour_line*d*((-1)**(closest_contour_line))
        
        if in_a_curve:
            # move_inwards = False
            # move_outwards = False

            if ( abs(theta_s - theta_i) <= 0.01 or abs(theta_s - theta_i) >= (2*pi-0.01) ) and time/1000 - time_s > 5:
                lap = True
                time_s = time/1000

            for aux in range(1, len(h_i.ID)):
                if h_i.ID[aux] == h_i.ID[0]:
                    continue
                if (h_i.s[0] - 1 == h_i.s[aux]) and (h_i.g[0] != h_i.g[aux]):
                    lap = False
                    theta_s = theta_i
                    time_s = time/1000

                if (h_i.s[0] > h_i.s[aux]) and (h_i.g[0] == h_i.g[aux]):
                    aux_is_alone = True
                    #print('Robot{} looking for friend_alone group {}'.format(robot_number, robot_group))
                    for aux2 in range(len(h_i.ID)):
                        if aux != aux2 and (h_i.s[aux] == h_i.s[aux2]) and (h_i.g[aux] != h_i.g[aux2]):
                            aux_is_alone = False
                    if aux_is_alone:
                        # print('Robot{} friend_alone, moving inwards=True'.format(robot_number))
                        move_inwards = True
                        # print('Robot', robot_number, 'inward will: friend alone')

                elif robot_number == '2':
                    pass
                    # print('Robot', robot_number, 'no frient inner')

                if (h_i.s[0] == h_i.s[aux]) and (h_i.g[0] != h_i.g[aux]) and not move_inwards:
                    if (h_i.t[0] < h_i.t[aux]) or (h_i.t[0] == h_i.t[aux] and h_i.theta[0] > h_i.theta[aux]):# or (h_i.t[0] == h_i.t[aux] and h_i.theta[0] == h_i.theta[aux] and h_i.g[0] > h_i.g[aux]):
                        move_outwards = True
                        # print('Robot', robot_number, 'outward will: enemy kicking')
                    if h_i.t[0] == h_i.t[aux] and h_i.theta[0] == h_i.theta[aux]:
                        print('ERROR NO ONE IS LEAVING')
                        print('Robot {} from {} wont leave'.format(robot_number, robot_group))

                if (h_i.m[aux] == 1) and (h_i.s[0] == h_i.s[aux] + 1):
                    move_outwards = True
                    # print('Robot', robot_number, 'outward will: inner robot comming out')


        if h_i.s[0] > 1 and lap:
            move_inwards = True
            # print('Robot', robot_number, 'inward will: lapped')
            lap = False

        # if move_inwards:
        #     move_outwards = False

        if h_i.m[0] != int(move_outwards):
            # h_i.s[0] = closest_contour_line
            # h_i.g[0] = robot_group
            # h_i.t[0] = time
            h_i.time[0] = time
            # h_i.theta[0] = theta_i
            # theta_s = theta_i
            # time_s = time/1000
            h_i.m[0] = int(move_outwards)
            # h_i.ID[0] = int(robot_number)

        # h_i.m[0] = int(move_outwards)

        SAFETY_SPACE = 0
        alfa = h_i.s[0]
        closest_robot_dist = 100000
        closest_robot = 'self'
        for aux in range(len(tij.in_a_circle_j)):
            current_dist = sqrt((x_i - tij.x_j[aux])**2 + (y_i - tij.y_j[aux])**2)

            if closest_robot_dist >= current_dist:
                closest_robot_dist = current_dist
                closest_robot = aux

            if move_inwards and h_i.s[0] == tij.s_j[aux]+1:
                if ang_diff(theta_i-pi/2,tij.theta_j[aux]+pi/2,'min') <= (pi*d/2 + SAFETY_SPACE + dr(h_i.g[0], tij.g_j[aux])*Sr(Rb, alfa, d))/alfa*d:
                    move_inwards = False
            if move_outwards and h_i.s[0] == tij.s_j[aux]-1:
                if ang_diff(theta_i-pi/2,tij.theta_j[aux]+pi/2,'min') <= (pi*d/2 + SAFETY_SPACE + dr(h_i.g[0], tij.g_j[aux])*Sr(Rb, alfa, d))/alfa*d:
                    move_outwards = False

            if tij.in_a_circle_j[aux] == 0:
                move_inwards = False
                move_outwards = False

        # Closed loop control
        Kp = 0.8
        Ki = 0.0
        Kd = 0.0
        KG = 5.0

        xp = x_i/abs(r_i)
        yp = y_i/abs(r_i)

        alpha = xp**2 + yp**2 - 1
        G = -2*atan(KG*alpha)/pi
        G2 = G
        # if not in_a_curve or abs(G) > CONVERGENCE_LIMIT:
        #     param_a = 2*Rb
        #     G_scale_factor = closest_robot_dist/param_a - 1
        #     if G_scale_factor > 1:
        #         G_scale_factor = 1

        #     if G_scale_factor < 1:
        #         move_inwards = False
        #         move_outwards = False

        #     G2 = G_scale_factor * G
        #     # if robot_number == '1':
        #     #     try:
        #     #         print('###Robot {} Scale factor = {}'.format(robot_number, G_scale_factor))
        #     #         print('###Robot {} is closest to {} by {} ({},{})'.format(robot_number, closest_robot, closest_robot_dist, tij.x_j[closest_robot], tij.y_j[closest_robot]))
        #     #     except:
        #     #         print('###Robot {} ...'.format(robot_number))
        #     #         G2 = G
        # else:
        #     G2 = G
        #     #G2 = sqrt(2)/2*copysign(1,G)
        #     # G2 = copysign(1,G)*sqrt(1-G**2+1e-6)

        if G2**2 > 1:
            H = 0
            G2 = G2/abs(G2)
        else:
            H = copysign(1,r_i)*sqrt(1-G2**2+1e-6)


        if move_outwards:
            move_inwards = False

        if move_inwards:
            print('Robot', robot_number, 'moving inwards')
            in_a_curve = False
            move_inwards = False
            time_s = time/1000
            lap = False
            # move_outwards = False
            closest_contour_line = closest_contour_line - 1
            if closest_contour_line == 0:
                closest_contour_line = 1
            r_i = closest_contour_line*d*((-1)**(closest_contour_line))
            xc = x_i*(1 + d/(2*sqrt(x_i**2 + y_i**2)))
            yc = y_i*(1 + d/(2*sqrt(x_i**2 + y_i**2)))

        if move_outwards:
            print('Robot', robot_number, 'moving outwards')
            in_a_curve = False
            # move_inwards = False
            move_outwards = False
            time_s = time/1000
            lap = False
            closest_contour_line = closest_contour_line + 1
            r_i = closest_contour_line*d*((-1)**(closest_contour_line))
            xc = x_i*(1 + d/(2*sqrt(x_i**2 + y_i**2)))
            yc = y_i*(1 + d/(2*sqrt(x_i**2 + y_i**2)))

        if not in_a_curve:

            xp = (x_i - xc)/(d/2)
            yp = (y_i - yc)/(d/2)

            KG_t = 5

            alpha_t = xp**2 + yp**2 - 1
            G_t = -2*atan(KG_t*alpha_t)/pi
            H_t = copysign(1,r_i)*sqrt(1-G_t**2+1e-6)

            grad_alpha_x = 2*xp/(d/2)
            grad_alpha_y = 2*yp/(d/2)
            norm_grad_alpha = sqrt(grad_alpha_x**2 + grad_alpha_y**2) + 1e6
            f_x = (G_t*grad_alpha_x - H_t*grad_alpha_y)/norm_grad_alpha
            f_y = (G_t*grad_alpha_y + H_t*grad_alpha_x)/norm_grad_alpha
            theta_ref = atan2(f_y, f_x )
            theta_error = sin(theta_ref - theta_i)
            theta_error = asin(theta_error)

            theta_error_int = theta_error_int + theta_error
            theta_error_diff = theta_error - theta_error_prev
            theta_error_prev = theta_error

            # if robot_number=='2':
            #     print('Robot', robot_number, 'G =', abs(G))

            # move_inwards = False
            # move_outwards = False
            if abs(G) < CONVERGENCE_LIMIT:
                # print('******\nRobot', robot_number, 'arrived')
                h_i.s = [closest_contour_line]
                h_i.g = [robot_group]
                h_i.t = [time]
                h_i.time = [time]
                h_i.theta = [theta_i]
                h_i.m = [0]
                h_i.ID = [int(robot_number)]

                theta_s = theta_i
                time_s = time/1000
                lap = False
                in_a_curve = True
                print('Robot', robot_number, 'arrived')
        else:
            # print('R{}: G ={:.2f}, H={:.2f}, {}, {}'.format(robot_number, G, H))
            grad_alpha_x = 2*xp/abs(r_i)
            grad_alpha_y = 2*yp/abs(r_i)
            norm_grad_alpha = sqrt(grad_alpha_x**2 + grad_alpha_y**2) + 1e6
            f_x = (G2*grad_alpha_x - H*grad_alpha_y)/norm_grad_alpha
            f_y = (G2*grad_alpha_y + H*grad_alpha_x)/norm_grad_alpha
            theta_ref = atan2(f_y, f_x )
            theta_error = sin(theta_ref - theta_i)
            theta_error = asin(theta_error)

            theta_error_int = theta_error_int + theta_error
            theta_error_diff = theta_error - theta_error_prev
            theta_error_prev = theta_error

        # Publica as velocidades
        vel.linear.x = ref_velocity
        vel.angular.z = Kp*theta_error + Ki*theta_error_int + Kd*theta_error_diff
        pub_vel.publish(vel)

        #Espera por um tempo de forma a manter a frequencia desejada
        rate.sleep()

# ---------- !! ---------- !! ---------- !! ---------- !! ----------

# Funcao inicial
if __name__ == '__main__':
    global robot_group

    robot_number = str(sys.argv[1])
    robot_group  = int(sys.argv[2])

    # Rb = 2.0 # robot radius
    # d  = Rb/0.2 # Rb < 0.5 d
    # c = 36 # d < 0.2890 c

    try:
        main_control(robot_number, robot_group)
    except rospy.ROSInterruptException:
        pass
