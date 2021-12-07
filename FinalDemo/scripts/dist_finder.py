#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from lab4.msg import PIDInput
from nav_msgs.msg import Odometry

# helpful constants:
ANGLE_RANGE_DEG = 270
ANGLE_RANGE_RAD = 4.72
CAR_LEN = 1.5


DESIRED_DIST_FROM_WALL = 1.25 # TODO: SET A DESIRED DISTANCE AWAY FROM THE WALL

WF_ZERO  = 45   # TODO: DEFINE WHAT YOUR "ZERO" ANGLE WILL BE TO FIND THE RANGE PERPENDICULAR TO THE CAR'S FORWARD VECTOR
WF_THETA = 60   # TODO: SET THETA VALUE (HINT: SHOULD BE > WF_ZERO)


class DistFinder:
    def __init__(self, wall_dist=DESIRED_DIST_FROM_WALL):
        # set pid control vars
        self.dist_desired = wall_dist

        # initialize node to register under master
        rospy.init_node('dist_finder', anonymous=True)


        # TODO: CREATE AN PUBLISHER HANDLE TO PUBLISH TO THE /dist_error TOPIC
        self.error_pub = rospy.Publisher('/dist_error', PIDInput, queue_size=10)

        # subscribe to the odom topic
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # subscribe to scan topic of simulator
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    

    # get_range converts a theta value in degrees to a range in the ranges array
    def get_range(self, data, theta):
        # TODO: USE THE MAPPING IN THE PRELAB OR A MAPPING OF YOUR CHOOSING TO 
        #       CONVERT FROM A THETA TO A RANGE IN THE DATA
        dist = data.ranges[int(theta * (len(data.ranges)/270))]
        return dist

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x


    def scan_callback(self, data):
        
        # TODO CALCULATE DISTANCE AWAY FROM THE WALL USING FORMULA SEEN IN PRELAB
        # B IS GIVEN AS AN EXAMPLE, AND YOU NEED TO FILL IN THE REST

        b = self.get_range(data, WF_ZERO)	
        a = self.get_range(data, WF_THETA)

        # math functions use radians, so use math.radians to convert for the alpha calculation
        theta_rad = math.radians(WF_THETA - WF_ZERO)

        ## Your code goes here to compute alpha, AB, and CD..and finally the error.
        # TODO: COMPUTE ALPHA, AB, CD AND THEN THE ERROR
        AB = b * math.cos(math.atan((a * math.cos(theta_rad) - b)/(a * math.sin(theta_rad))))
        AC = self.speed * 0.5
        CD = AB + AC * math.atan((a * math.cos(theta_rad) - b)/(a * math.sin(theta_rad)))
        #print(CD)

        error =  CD - DESIRED_DIST_FROM_WALL

        # TODO: CREATE AN INSTANCE OF THE PIDInput() MESSAGE AND ASSIGN THE ANGLE ERROR
        inp = PIDInput()
        inp.angle_error = error

        # TODO: PUBLISH YOUR MESSAGE USING self.error_pub
        self.error_pub.publish(inp)



if __name__ == "__main__":
    # OPTIONAL: ADD USER INPUT TO DEFINE DESIRED DISTANCE FROM WALL
    DistFinder()
    
    rospy.spin()
