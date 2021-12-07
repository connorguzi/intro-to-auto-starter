#!/usr/bin/env python

import rospy
from lab4.msg import PIDInput
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
import skfuzzy as fuzz
import numpy as np
from skfuzzy import control as ctrl

ERROR_SCALE = 5 # TODO: ASSIGN A CONSTANT FOR HOW MUCH TO SCALE THE RAW ERROR
MAX_STEERING_ANGLE = 4.18

class CarControl:
    def __init__(self, kPmin, kPmax, kDmin, kDmax):
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
        self.kpmin = kPmin
        self.kpmax = kPmax
        self.kdmin = kDmin
        self.kdmax = kDmax
        # assign shutdown functon 
        rospy.on_shutdown(self.on_shutdown)

    # callback for dist_error topic to handle pid calculation and publish to /drive. 
    def run_pid(self, pid_input):
        # TODO: SCALE ERROR BY SOME CONSTANT
        error = pid_input.angle_error
        error *= ERROR_SCALE

        temp_vals = self.fuzzy(error, (-self.past_error + error))
        kp = (self.kpmax - self.kpmin) * temp_vals[0] + self.kpmin 
        kd = (self.kdmax - self.kdmin) * temp_vals[1] + self.kdmin 
        # TODO: CALCULATE A STEERING ANGLE BASED BY PASSING THE ERROR INTO THE PID FUNCTION
        ut = kp * error + kd * (-self.past_error + error)
        # TODO: UPDATE PREVIOUS ERROR FOR DERIVATIVE TERM
        self.past_error = error

        # TODO: VALIDATE STEERING ANGLE
        steer = 3* ut/ MAX_STEERING_ANGLE
        velocity = 14/abs((5*steer))
        # TODO: ASSIGN VALUES IN ACKERMANN MESSAGE 
        ack_msg = AckermannDriveStamped()
        ack_msg.drive.steering_angle = -steer
        ack_msg.drive.speed = velocity
    
        # TODO PUBLISH ACKERMANN DRIVE MESSAGE
        self.drive_pub.publish(ack_msg)
        pass


    # shutdown function
    def on_shutdown(self):
        print("### END WALL FOLLOW ###")

        # TODO PUBLISH AN EMPTY ACKERMANNDRIVESTAMPED MESSAGE TO THE DRIVE PUBLISHER HANDLE

        self.pose_pub.publish(PoseStamped())
    
    def fuzzy(self, error, derror):
        errormf = ctrl.Antecedent(np.arrange(-6, 6, 1), 'error')
        derrormf = ctrl.Antecedent(np.arrange(-2, 2, 1), 'derror')
        kpp = ctrl.Consequent(np.linspace(0, 1, 20), 'kpp')
        kdd = ctrl.Consequent(np.linspace(0, 1, 20), 'kdd')

        names = ['nb', 'nm', 'ns', 'zo', 'ps', 'pm', 'pb']
        errormf.automf(names=names)
        derrormf.automf(names=names)
        
        kpp['small'] = fuzz.zmf(kpp.universe, [0.1, 0.9])
        kpp['big'] = fuzz.smf(kpp.universe, [0.1, 0.9])

        kdd['small'] = fuzz.zmf(kdd.universe, [0.1, 0.9])
        kdd['big'] = fuzz.smf(kdd.universe, [0.1, 0.9])

        rule1 = ctrl.Rule(antecedent=(errormf['nb'] | errormf['pb']), consequent=(kpp['big'] & kdd['small']))
        rule2 = ctrl.Rule(antecedent=(((errormf['nm'] | errormf['pm']) &  (~derrormf['nb'] & ~derrormf['pb'])) |
                            ((errormf['ns'] | errormf['ps']) & (derrormf['ns'] | derrormf['ps'] | derrormf['zo'])) | 
                            (errormf['zo'] & derrormf['zo'])),
                             consequent=kpp['big'])
        rule3 = ctrl.Rule(antecedent=(~(((errormf['nm'] | errormf['pm']) &  (~derrormf['nb'] & ~derrormf['pb'])) |
                            ((errormf['ns'] | errormf['ps']) & (derrormf['ns'] | derrormf['ps'] | derrormf['zo'])) | 
                            (errormf['zo'] & derrormf['zo'])) & ~(errormf['nb'] | errormf['pb'])),
                             consequent=kpp['small'])
        rule4 = ctrl.Rule(antecedent=(((errormf['nm'] | errormf['pm']) & (derrormf['ns'] | derrormf['ps'] | derrormf['zo'])) |
                            (~errormf['zo'] & derrormf['zo'])),
                            consequent=kdd['small'])
        rule5 = ctrl.Rule(antecedent=(~(((errormf['nm'] | errormf['pm']) & (derrormf['ns'] | derrormf['ps'] | derrormf['zo'])) |
                            (~errormf['zo'] & derrormf['zo'])) & ~(errormf['nb'] | errormf['pb'])),
                            consequent=kdd['big']) 

        fuzzy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
        fuzzied = ctrl.ControlSystemSimulation(fuzzy_ctrl)

        fuzzied.input['errormf'] = error
        fuzzied.input['derrormf'] = derrormf
        fuzzied.compute()

        Kpp = fuzzied.output['kpp']
        Kdd = fuzzied.output['kdd']

        return[Kpp, Kdd]


if __name__ == "__main__":
    #  TODO: USE USER INPUT TO TUNE KP AND KD, THEN HARDCODE ONCE TUNED
    kP = 0.42
    kD = 0.11

    print("### START WALL FOLLOW ###")

    # initial node
    CarControl(kP, kD)

    # spin ros
    rospy.spin()
