#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import actionlib
from fabric_grasping.msg import GraspAction, GraspResult, GraspFeedback, GraspGoal
from tams_diana7_tools.dual_diana_helper import DualDianaHelper
from geometry_msgs.msg import PoseStamped

rospy.init_node("diana7_grasping_from_table")

param_error_msg = "Please provide the following parameters: which arm= [l,r,a], x and y position of the grasp point in the table_surface frame"

if len(sys.argv) < 4:
    rospy.logerr(param_error_msg)
    sys.exit(1)

if sys.argv[1] not in ['l', 'r', 'a']:
    rospy.logerr(param_error_msg)
    sys.exit(1)

lr = sys.argv[1]
x = float(sys.argv[2])
y = float(sys.argv[3])

goal_pose = PoseStamped()
goal_pose.header.frame_id = 'table_surface'
goal_pose.pose.position.x = x-0.02
goal_pose.pose.position.y = y
goal_pose.pose.position.z = 0.18
goal_pose.pose.orientation.w = 1

dh = DualDianaHelper(load_moveit=True, arm_l_active=True, arm_r_active=True, no_grippers=False)

if lr == 'l':
    arm = dh.arm_l
elif lr == 'r':
    arm = dh.arm_r
else:
    rospy.logerr('lr must be l, r, or a')
    sys.exit(1)
arm.gripper.move_to(0.5)

grasp_client = actionlib.SimpleActionClient('grasp', GraspAction)

arm.load_position_trajectory_controller()
arm.move_arm_to_pose(goal_pose)

goal = GraspGoal()
if lr == 'r':
    goal.arm = GraspGoal.ARM_RIGHT
else:
    goal.arm = GraspGoal.ARM_LEFT
grasp_client.send_goal(goal)
grasp_client.wait_for_result()

arm.move_arm_with_velocity(vz=0.05, duration=1)
