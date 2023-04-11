#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class AckermannCmdNode:
    def __init__(self):
        self.ackermann_cmd = AckermannDriveStamped()
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=1)
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        

    def cmd_callback(self, data):
        self.ackermann_cmd.drive.speed = data.linear.x
        self.ackermann_cmd.drive.steering_angle = data.angular.z
        self.pub.publish(self.ackermann_cmd)


if __name__ == "__main__":
    rospy.init_node("ackermann_cmd")
    node = AckermannCmdNode()
    rospy.spin()
