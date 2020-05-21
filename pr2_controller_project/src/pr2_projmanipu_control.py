#!/usr/bin/env python

import rospy
import std_msgs
from control_msgs.msg import FollowJointTrajectoryActionGoal
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

goal_pub = rospy.Publisher('/r_arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=100)

def joint_callback(data):
	joint_angles = FollowJointTrajectoryActionGoal()
	joints = data.desired
	joints.positions += (0.02, 0.02, 0.1, 0.02, 0.02, 0.02, 0.02)
	joints.velocities += (0,0,1,0,0,0,0)
	joints.accelerations += (0,0,1,0,0,0,0)
	print(joints)
	velocity = (0,0,1,0,0,0,0)
	joint_angles.goal.trajectory.points.positions.append((0.02, 0.02, 0.1, 0.02, 0.02, 0.02, 0.02)) #velocity
	goal_pub.publish(joint_angles)


if __name__ == '__main__':
	rospy.init_node('pr2_manipulation_proj_control', anonymous=True)
	joint_sub = rospy.Subscriber('/r_arm_controller/state', JointTrajectoryControllerState, joint_callback)
	rospy.spin()
	
