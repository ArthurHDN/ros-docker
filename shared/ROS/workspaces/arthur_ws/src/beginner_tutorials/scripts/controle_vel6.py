#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import cos, sin
x0 = 0.0
y0 = 0.0
T = 0.0
k = 1
d = 1
Err = 0.1
vel_lim = Twist()
vel_lim.linear.x = 3
vel_lim.angular.z = 3
def callback(data):
    global x0, y0, T, k, d

    x0 = data.x + d * cos(data.theta)
    y0 = data.y + d * sin(data.theta)
    T = data.theta
def talker():
    rospy.init_node('controle_vel', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/turtle1/pose', Pose, callback)
    rate = rospy.Rate(20)
    vel_msg = Twist()
    while not rospy.is_shutdown():
        xf, yf = raw_input('Digite a coordenada (x,y) desejada: ').split()
        xf, yf = [float(xf) for xf in [xf, yf]]
        if xf > 11 or xf < 0 or yf > 11 or yf < 0:
            rospy.loginfo('\033[31mCoordenadas invalidas, digite novamente...\033[m')
            continue
        lim = 0
        while abs(xf - x0) > Err or abs(yf - y0) > Err:
            vel_msg.linear.x = k * ( cos(T) * (xf - x0) + sin(T) * (yf - y0) )
            vel_msg.angular.z = k * (-(sin(T) * (xf - x0)) / d + (cos(T) * (yf - y0)) / d)
            if vel_msg.linear.x > vel_lim.linear.x:
                 lim = 1
                 vel_msg.linear.x = vel_lim.linear.x
            elif vel_msg.linear.x < -vel_lim.linear.x:
                 lim = 1
                 vel_msg.linear.x = -vel_lim.linear.x
            if vel_msg.angular.z > vel_lim.angular.z:
                 lim = 1
                 vel_msg.angular.z = vel_lim.angular.z
            elif vel_msg.angular.z < -vel_lim.angular.z:
                 lim = 1
                 vel_msg.angular.z = -vel_lim.angular.z
            pub.publish(vel_msg)
            rate.sleep()
        if lim == 1:
            rospy.loginfo('A Velocidade foi limitada')
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
