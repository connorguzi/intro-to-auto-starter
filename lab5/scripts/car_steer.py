#!/usr/bin/env python

import rospy
from lab4.msg import PIDInput
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped

ERROR_SCALE = 1 # TODO: ASSIGN A CONSTANT FOR HOW MUCH TO SCALE THE RAW ERROR
MAX_STEERING_ANGLE = 4.18

class CarControl:
    def __init__(self, kP, kD):
        # initialize ros node
        rospy.init_node("car_control", anonymous=False)

        # get publisher handles for drive and pose topics 
        # TODO: CREATE A PUBLISHER FOR THE DRIVE TOPIC 
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        # pose topic is to reset car position when you reset/shutdown the node
        self.pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)
        self.past_error = 0
        # TODO: SUBSCRIBE TO dist_error MESSAGE FROM dist_finder.py NODE . ASSIGN THE CALLBACK TO run_pid()
        rospy.Subscriber('dist_error', PIDInput, self.run_pid)

        # TODO: INITIALIZE KP AND KD CLASS VARIABLES FROM CONSTRUCTOR PARAMETERS
        self.kp = kP
        self.kd = kD
        # assign shutdown functon 
        rospy.on_shutdown(self.on_shutdown)

    # callback for dist_error topic to handle pid calculation and publish to /drive. 
    def run_pid(self, pid_input):
        # TODO: SCALE ERROR BY SOME CONSTANT
        error = pid_input.angle_error
        error *= ERROR_SCALE

        # TODO: CALCULATE A STEERING ANGLE BASED BY PASSING THE ERROR INTO THE PID FUNCTION
        ut = self.kp * error + self.kd * (-self.past_error + error)
        # TODO: UPDATE PREVIOUS ERROR FOR DERIVATIVE TERM
        self.past_error = error

        # TODO: VALIDATE STEERING ANGLE
        steer = 3* ut/ MAX_STEERING_ANGLE
        velocity = 3

        ack_msg = AckermannDriveStamped()
        ack_msg.drive.steering_angle = -steer
        ack_msg.drive.speed = velocity
    
        self.drive_pub.publish(ack_msg)
        pass


    # shutdown function
    def on_shutdown(self):
        print("### END WALL FOLLOW ###")
        self.pose_pub.publish(PoseStamped())


if __name__ == "__main__":
    kP = 0.02
    kD = 0.0006
    print("### START FOLLOW THE GAP ###")

    # initial node
    CarControl(kP, kD)

    # spin ros
    rospy.spin()
