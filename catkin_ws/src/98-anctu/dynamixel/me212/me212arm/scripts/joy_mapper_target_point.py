#!/usr/bin/env python
import rospy
import math

from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy

from __builtin__ import True

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        # Publications
        self.pub_target_point = rospy.Publisher("/arm_target_points",Point,queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)

        #Steady the publish speed
        #self.param_timer = rospy.Timer(rospy.Duration.from_sec(0.1),self.cbParamTimer)

        # Target Points (initiallize to (x, z) = (0, 0.2) meters)
        self.tp = Point()
        self.tp.x = 0
        self.tp.y = 0 #no need y axis for temporary
        self.tp.z = 0.20
        self.pub_target_point.publish(self.tp)

        #Gripper control
        self.gp_State = False # False == Open
        print "init"

    def cbParamTimer(self,event):
        self.velocityCtl = 1
        #print "Can Publish now"

    def cbJoy(self, joy_msg):
        #print 'Get joy'
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)

    def publishControl(self):
        #if self.velocityCtl == 1:
        tp = Point()
        #tp = self.tp
        tp.x = self.tp.x
        tp.y = 0
        tp.z = self.tp.z
        if self.joy.axes[0] > 0.5 or self.joy.axes[0] < -0.5:
            tp.z = -(math.copysign(0.002, self.joy.axes[0])) + self.tp.z
        if self.joy.axes[1] > 0.5 or self.joy.axes[1] < -0.5:
            tp.x = math.copysign(0.002, self.joy.axes[1]) + self.tp.x
        print 'tp', tp, 'self.tp', self.tp
        if tp != self.tp:
            print "different"
            self.pub_target_point.publish(tp)
        self.velocityCtl = 0
        self.tp = tp


# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7,
# logitek = 8, left joy = 9, right joy = 10
# XXX: here we should use constants
    def processButtons(self, joy_msg):
        if (joy_msg.buttons[2] == 1): #X button Change Motor (ID)
            rospy.loginfo('X button to  origin')
            self.tp.x = 0
            self.tp.z = 0.2
            self.pub_target_point.publish(self.tp)
            
        elif (joy_msg.buttons[0] == 1): #A button gripper control
            self.gp_State = ~self.gp_State
            if self.gp_State:
                rospy.loginfo('A button change gripper state = Open')
            else:
                rospy.loginfo('A button change gripper state = Close')

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))


if __name__ == "__main__":
    rospy.init_node("joy_mapper_target_points",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()

