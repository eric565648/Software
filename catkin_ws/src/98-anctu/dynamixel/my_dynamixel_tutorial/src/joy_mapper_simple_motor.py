#!/usr/bin/env python
import rospy
import math

from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

from __builtin__ import True

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        # Setup Parameters
        #self.v_gain = self.setupParam("~speed_gain", 0.41)
        #self.omega_gain = self.setupParam("~steer_gain", 8.3)
        #self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        #self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        #self.simulated_vehicle_length = self.setupParam("~simulated_vehicle_length", 0.18)
        self.currentjointID = 1
        self.panState = JointState()
        self.tiltState = JointState()
        self.graspState = JointState()
        self.velocityCtl = 1
        self.max_angle_delta = 0.1

        # Publications
        #self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        #self.pub_joy_override = rospy.Publisher("~joystick_override", BoolStamped, queue_size=1)
        #self.pub_parallel_autonomy = rospy.Publisher("~parallel_autonomy",BoolStamped, queue_size=1)
        #self.pub_anti_instagram = rospy.Publisher("anti_instagram_node/click",BoolStamped, queue_size=1)
        self.pub_e_stop = rospy.Publisher("wheels_driver_node/emergency_stop",BoolStamped,queue_size=1)
        #self.pub_avoidance = rospy.Publisher("~start_avoidance",BoolStamped,queue_size=1)
        self.pub_pan_ctl = rospy.Publisher("/pan_controller/command", Float64, queue_size=1)
        self.pub_tilt_ctl = rospy.Publisher("/tilt_controller/command", Float64, queue_size=1)
        self.pub_grasp_ctl = rospy.Publisher("/grasp_controller/command", Float64, queue_size=1)


        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_pan_state = rospy.Subscriber("/pan_controller/state", JointState, self.cbJoint, queue_size=1)
        self.sub_tilt_state = rospy.Subscriber("/tilt_controller/state", JointState, self.cbJoint, queue_size=1)
        self.sub_grasp_state = rospy.Subscriber("/grasp_controller/state", JointState, self.cbJoint, queue_size=1)

        # timer
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(0.3),self.cbParamTimer)
        #self.has_complained = False

        #self.state_parallel_autonomy = False
        #self.state_verbose = False

        '''
        pub_msg = BoolStamped()
        pub_msg.data = self.state_parallel_autonomy
        pub_msg.header.stamp = self.last_pub_time
        self.pub_parallel_autonomy.publish(pub_msg)
        '''

    def cbParamTimer(self,event):
        '''
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 10)
        '''
        self.velocityCtl = 1
        print "Can Publish now"

    '''
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
    '''

    def cbJoy(self, joy_msg):
        print 'Get joy'
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)

    def cbJoint(self, joint_msg):
        if (joint_msg.motor_ids[0] == 1):
            self.panState = joint_msg
        elif (joint_msg.motor_ids[0] == 2):
            self.tiltState = joint_msg
        elif (joint_msg.motor_ids[0] == 3):
            self.graspState = joint_msg
        else:
            print 'Error, joint no catch!'

    def publishControl(self):
        '''
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = self.joy.header.stamp
        car_cmd_msg.v = self.joy.axes[1] * self.v_gain #Left stick V-axis. Up is positive
        if self.bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = self.joy.axes[3] * self.steer_angle_gain
            car_cmd_msg.omega = car_cmd_msg.v / self.simulated_vehicle_length * math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = self.joy.axes[3] * self.omega_gain
        self.pub_car_cmd.publish(car_cmd_msg)
        '''
        if self.velocityCtl == 1 and math.fabs(self.joy.axes[1]) > 0.5:
            if self.currentjointID == 1:
                angle_msg = self.joy.axes[1] * self.max_angle_delta + self.panState.current_pos
                self.pub_pan_ctl.publish(angle_msg)
            elif self.currentjointID == 2:
                angle_msg = self.joy.axes[1] * self.max_angle_delta + self.tiltState.current_pos
                self.pub_tilt_ctl.publish(angle_msg)
            else:
                print "Wrong ID"


# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7,
# logitek = 8, left joy = 9, right joy = 10
# XXX: here we should use constants
    def processButtons(self, joy_msg):
        if (joy_msg.buttons[2] == 1): #X button Change Motor (ID)
            if self.currentjointID < 2:
                self.currentjointID+=1
            else:
                self.currentjointID = 1
            rospy.loginfo('Current ID is %d' % self.currentjointID)
            
        elif (joy_msg.buttons[0] == 1): #A button gripper control
            if self.graspState.is_moving == False:
                if self.graspState.current_pos < -1:
                    self.pub_grasp_ctl.publish(0)
                else:
                    self.pub_grasp_ctl.publish(-2.5)

        elif (joy_msg.buttons[8] == 1): #power button (middle)
            e_stop_msg = BoolStamped()
            e_stop_msg.header.stamp = self.joy.header.stamp
            e_stop_msg.data = True # note that this is toggle (actual value doesn't matter)
            rospy.loginfo('E-stop message')
            self.pub_e_stop.publish(e_stop_msg)

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))


if __name__ == "__main__":
    rospy.init_node("joy_mapper_simple_motor",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()

