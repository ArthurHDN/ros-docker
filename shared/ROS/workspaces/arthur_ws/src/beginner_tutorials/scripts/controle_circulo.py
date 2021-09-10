#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import cos, sin
x0 = 0.0
y0 = 0.0
T = 0.0
k = 0.2
d = 1
def callback(data):
    global x0, y0, T, k, d
    x0 = data.x + d * cos(data.theta)
    y0 = data.y + d * sin(data.theta)
    T = data.theta
def iniciar():
    rospy.init_node('controle_vel', anonymous=True)
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/turtle1/pose', Pose, callback)
    rate = rospy.Rate(20)
    vel_msg = Twist()
    cx, cy, rx, ry = input('Digite o centro (x,y) e os semieixos (a,b) ').split()
    cx, cy, rx, ry = [float(i) for i in [cx, cy, rx, ry]]
    t = 0
    w = 0.4
    while not rospy.is_shutdown():
        t += 0.05
        xf = rx * cos(w*t) + cx
        yf = ry * sin(w*t) + cy
        print(xf, yf)
        Vx = k * (xf - x0) - rx*sin(w*t)*w
        Vy = k * (yf - y0) + ry*cos(w*t)*w
        vel_msg.linear.x =  cos(T) * Vx + sin(T) * Vy
        vel_msg.angular.z = -(sin(T) * Vx) / d + (cos(T) * Vy) / d
        pub.publish(vel_msg)
        rate.sleep()
if __name__ == '__main__':
    try:
        iniciar()
    except rospy.ROSInterruptException:
        pass
