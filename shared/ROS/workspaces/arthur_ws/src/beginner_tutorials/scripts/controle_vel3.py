#!/usr/bin/env python
#########################ERRADO###########################
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan, cos, atan2, sqrt

x0 = 0.0
y0 = 0.0
xf = 0.0
yf = 0.0
Theta0 = 0.0
Err = 0.2

def mover():
    rospy.init_node('controle_vel', anonymous=True)
    rospy.Subscriber('turtle1/pose', Pose, callback)
    rospy.spin()

def callback(data):

    rate = rospy.Rate(1)
    vel_msg = Twist()
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.loginfo('Digite a coordenada (x,y) desejada: ')
    xf, yf = raw_input('Digite a coordenada (x,y) desejada: ').split()
    xf, yf = [float(xf) for xf in [xf, yf]]
    if xf > 11 or xf < 0:
        rospy.loginfo('\033[31mCoordenadas invalidas, digite novamente...\033[m')
        #continue
    if yf > 11 or yf < 0:
        rospy.loginfo('\033[31mCoordenadas invalidas, digite novamente...\033[m')
        #continue
    #while abs(x0 - xf) > Err or abs(y0 - yf) > Err:

    x0 = data.x
    y0 = data.y
    Theta0 = data.theta

    vel_msg.angular.z = atan2((yf - y0), (xf - x0)) - Theta0
    vel_msg.linear.x = 1 * sqrt(((xf - x0) ** 2 + (yf - y0) ** 2))  # * abs (y0-yf)
    if vel_msg.linear.x > 2:
       vel_msg.linear.x = 2
    elif vel_msg.linear.x < -2:
        vel_msg.linear.x = -2
    pub.publish(vel_msg)
    #rate.sleep()

if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
