#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan, cos
x0 = 0.0
y0 = 0.0
xf = 0.0
yf = 0.0
Theta0 = 0.0
Err = 0.2
def atribuirvalorpos(data):
    global x0
    global y0
    global Theta0
    x0 = data.x
    y0 = data.y
    Theta0 = data.theta
def mover():
    global xf
    global yf
    global Err
    #Err = 0.2
    rospy.init_node('controle_vel', anonymous=True)
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    vel_msg = Twist()
    rospy.Subscriber('turtle1/pose', Pose, atribuirvalorpos)
    while not rospy.is_shutdown():
        rospy.loginfo('Digite a coordenada (x,y) desejada: ')
        #rospy.loginfo("Coordenadas x, y, theta: %s, %s, %s", x0, y0, Theta0)
        xf = float(input('x = '))
        if xf > 11 or xf < 0:
            continue
        yf = float(input('y = '))
        if yf > 11 or yf < 0:
            continue
        while abs(x0-xf) > Err or abs(y0-yf) > Err:
            if abs(xf - x0) < Err:
                vel_msg.angular.z = 1.570793 - Theta0
                vel_msg.linear.x = (yf - y0) # * cos(vel_msg.angular.z)
                rate.sleep()
            else:
                vel_msg.angular.z = atan( (yf-y0)/(xf-x0) ) - Theta0
                if xf > x0:
                    vel_msg.linear.x = 2 * abs(x0-xf) #* abs (y0-yf)
                else:
                    vel_msg.linear.x = -2 * abs(x0-xf) #* abs (y0-yf)
            if vel_msg.linear.x > 2:
                vel_msg.linear.x = 2
            elif vel_msg.linear.x < -2:
                vel_msg.linear.x = -2
            pub.publish(vel_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
