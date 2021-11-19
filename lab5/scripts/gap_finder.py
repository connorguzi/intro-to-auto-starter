#!/usr/bin/env python

import rospy
import math
from numpy import percentile
from sensor_msgs.msg import LaserScan
from lab4.msg import PIDInput
from nav_msgs.msg import Odometry


# helpful constants:
ANGLE_RANGE_DEG = 270
ANGLE_RANGE_RAD = 4.72
CAR_LEN = 1.5


HEADING_ANGLE  = 135   # defining heading of the car

CAR_RAD = CAR_LEN / 2 

class GapFinder:
    def __init__(self, car_len=CAR_LEN):
        # set pid control vars
        self.car_len = car_len
        self.gap_ang = 0
        # initialize node to register under master
        rospy.init_node('dist_finder', anonymous=True)


        # TODO: CREATE AN PUBLISHER HANDLE TO PUBLISH TO THE /dist_error TOPIC
        self.error_pub = rospy.Publisher('/dist_error', PIDInput, queue_size=10)

        # subscribe to scan topic of simulator
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)



    def zero_and_gaps(self, data, radius, templength):
        gaps = {}
        length = percentile(data, 75)
        ranges = [None] * len(data)
        indexes_to_avoid = []
        gap = {}
        for i in range(len(data)):
            if data[i] < length:    #If the measurement is too short, then a circle of zeroes is created
                if gap:
                    gap_width = len(gap.values()) * 270/len(ranges) * min(gap.values())
                    gap_angle = (sum(gap.keys())/(len(gap.keys()))) * 270/len(ranges)
                    gaps[gap_width] = gap_angle
                gap = {}
                
                theta = math.degrees(radius / length)
                counter = int(math.ceil(theta/270))
                ranges[i] = 0
                for n in range(counter):
                    indexes_to_avoid.append(i+n)
                    indexes_to_avoid.append(i-n)
            else:
                if i in indexes_to_avoid:
                    ranges[i] = 0
                else:    
                    gap[i] = data[i]
                    ranges[i] = data[i]
        
        return gaps[max(gaps.keys())]


    def scan_callback(self, data):
        
        gap_angle = self.zero_and_gaps(data.ranges, CAR_RAD, 5)
        error =  HEADING_ANGLE - gap_angle
 

        # TODO: CREATE AN INSTANCE OF THE PIDInput() MESSAGE AND ASSIGN THE ANGLE ERROR
        inp = PIDInput()
        inp.angle_error = error

        # TODO: PUBLISH YOUR MESSAGE USING self.error_pub
        self.error_pub.publish(inp)
        



if __name__ == "__main__":
    # OPTIONAL: ADD USER INPUT TO DEFINE DESIRED DISTANCE FROM WALL
    GapFinder()

    rospy.spin()
