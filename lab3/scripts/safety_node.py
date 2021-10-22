#!/usr/bin/env python
#from yaml import scan
import rospy

# TODO: import ROS msg types and libraries
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from math import cos

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.
        self.ackermann = rospy.Publisher("brake", AckermannDriveStamped, queue_size=10)
        self.bool = rospy.Publisher('brake_bool', Bool, queue_size=10)

        self.bVal = Bool()
        self.bVal.data = False
        self.brake = AckermannDriveStamped()
        
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('scan', LaserScan, self.scan_callback)
        pass

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        TTC = []
        for i in range(len(scan_msg.ranges)):
            if type(scan_msg.ranges[i]) == float and (scan_msg.ranges[i]) != 0: 
                theta = (0 - scan_msg.angle_min) + (scan_msg.angle_increment * i) 
                TTC.append(scan_msg.ranges[i] / max([-self.speed * cos(theta), 0.00000000000001]))
        # TODO: publish brake message and publish controller bool
        if min(TTC) <= 1:
            print(min(TTC))
            self.bVal.data = True
            self.ackermann.publish(self.brake)
            self.bool.publish(self.bVal)
            
        else:
            self.bVal.data = False
            self.bool.publish(self.bVal)
            
        pass


if __name__ == '__main__':
    rospy.init_node('safety_node')
    safety_node = Safety()
    rospy.spin()