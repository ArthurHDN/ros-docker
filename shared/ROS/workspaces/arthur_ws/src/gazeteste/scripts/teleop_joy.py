#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def callback(data):
    global Kp, vel_pub
    Kp = 0.1
    vel_pub = Twist()
    if data.buttons[0] == 1:
        Kp +=0.5
    if data.buttons[2] == 1:
        Kp += 1
    vel_pub.linear.x = Kp*data.axes[1]
    vel_pub.angular.z = Kp*data.axes[0]
    if data.buttons[1] == 1:
        vel_pub.linear.x = 0
    pub.publish(vel_pub)
# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2robot', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
