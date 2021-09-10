#!/usr/bin/env python
##################BUGADO########################
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import cos, sin

x0 = 0.0
y0 = 0.0
T = 0.0
k = 1
d = 0.2
Err = 0.5


def callback(data):
    global x0, y0, T, k, d
    x0 = data.x
    y0 = data.y
    T = data.theta

def talker():
    rospy.init_node('controle_vel', anonymous=True)
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/Pose', Pose, callback)
    rate = rospy.Rate(10)
    vel_msg = Twist()
    while not rospy.is_shutdown():
        xf, yf = raw_input('Digite a coordenada (x,y) desejada: ').split()
        xf, yf = [float(xf) for xf in [xf, yf]]
        if xf > 11 or xf < 0 or yf > 11 or yf < 0:
            rospy.loginfo('\033[31mCoordenadas invalidas, digite novamente...\033[m')
            continue
        while abs(xf - x0) > Err or abs(yf - y0) > Err:
            vel_msg.linear.x = k * (cos(T) * (xf - x0) + sin(T) * (yf - y0))
            vel_msg.angular.z = k * (-(sin(T) * (xf - x0)) / d + (cos(T) * (yf - y0)) / d)
            pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
