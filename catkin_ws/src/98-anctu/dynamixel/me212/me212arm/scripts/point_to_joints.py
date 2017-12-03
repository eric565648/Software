#!/usr/bin/python

# 2.12 Lab 5 trajectory planning
# Peter Yu Oct 2016
import rospy
import planner
import std_msgs.msg, sensor_msgs.msg
from geometry_msgs.msg import Point
import numpy as np

rospy.init_node("points_to_joints")

exec_joint1_pub = rospy.Publisher('/joint1_controller/command', std_msgs.msg.Float64, queue_size=1)
exec_joint2_pub = rospy.Publisher('/joint2_controller/command', std_msgs.msg.Float64, queue_size=1)
exec_joint_pub = rospy.Publisher('/virtual_joint_states', sensor_msgs.msg.JointState, queue_size=10)

use_real_arm = rospy.get_param('/real_arm', False)

def to_joints(msg):
    print "get points", msg.x, msg.z
    
    robotjoints = rospy.wait_for_message('/joint_states', sensor_msgs.msg.JointState)
    q0 = robotjoints.position[0:2]
    
    target_xz = []
    target_xz.append(msg.x)
    target_xz.append(msg.z)
    
    q_sol = planner.ik(target_xz, q0)
    #q_sol = [0, 0, 0]

    if q_sol is None:
        print 'no ik solution'
    else:
        if use_real_arm:
            q_1 = -1*q_sol[0] #Because I installed the arm inversely accidentally
            q_2 = -1*q_sol[1]
            print '(q_1,q_2)=', q_sol
            exec_joint1_pub.publish(std_msgs.msg.Float64(q_1))
            exec_joint2_pub.publish(std_msgs.msg.Float64(q_2))
        else:
            print '(q_1,q_2)=', q_sol
            js = sensor_msgs.msg.JointState(name=['joint1', 'joint2'], position = q_sol)
            exec_joint_pub.publish(js)
        q0 = q_sol


if __name__=="__main__":
    point_sub = rospy.Subscriber('/arm_target_points', Point, to_joints, queue_size=1)
    rospy.spin()