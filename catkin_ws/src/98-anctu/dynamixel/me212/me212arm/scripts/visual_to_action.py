#!/usr/bin/python

# get visual points and transfer to target point
import rospy
import planner
import std_msgs.msg, sensor_msgs.msg
from geometry_msgs.msg import Point
import numpy as np

gate = [0.28,0.318,0.355,0.378,0.43]

x = [0.0547, 0.074344, 0.097668, 0.098621]
z = [0.196, 0.175251, 0.140303, 0.125963]
po = Point()
po.x = -0.123515
po.y = 0
po.z = 0.142277

g_open = 0.
g_close = -1.2

d = 1.5

rospy.init_node("visual_to_action")

pub_tp = rospy.Publisher('/arm_target_points', Point, queue_size=1)
pub_grip = rospy.Publisher('/grip_controller/command', std_msgs.msg.Float64, queue_size=1)

rospy.set_param('/execute', False)

def action(msg):
    print "Get point!"
    exe = rospy.get_param('/execute', False)
    if exe == False:
        return
    elif msg.z < gate[0] or msg.z > gate[4]:
        return
    print "Start exe!"
    
    rospy.set_param('/execute', False)
    pub_tp.publish(po)
    pub_grip.publish(g_open)
    rospy.sleep(d+2)
    
    p = Point()
    for i in range(len(gate)-1):
        if msg.z > gate[i] and msg.z < gate[i+1]:
            p.x = x[i]
            p.y = 0
            p.z = z[i]
            pub_tp.publish(p)
            rospy.sleep(d)
            pub_grip.publish(g_close)
            rospy.sleep(d)
            break
    pub_tp.publish(po)
    rospy.sleep(d)
    pub_grip.publish(g_open)



if __name__=="__main__":
    point_sub = rospy.Subscriber('/depth_detect_cir_node/obj_point', Point, action, queue_size=1)
    rospy.spin()