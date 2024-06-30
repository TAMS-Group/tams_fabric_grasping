#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import actionlib
from fabric_grasping.msg import GraspAction, GraspResult, GraspFeedback, GraspGoal
from tams_diana7_tools.dual_diana_helper import DualDianaHelper

class GraspTester:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("diana7_grasping_demo")

        self.dh = DualDianaHelper(load_moveit=True, arm_l_active=True, arm_r_active=True, no_grippers=False)
        self.arm = self.dh.arm_l
        self.gripper = self.dh.gripper_l

        self.grasp_client = actionlib.SimpleActionClient('grasp', GraspAction)
        
        cont = ''
        
        while cont == '' and not rospy.is_shutdown():
            # setting everything up to start from scratch
            print('initializing...')
            self.arm.unload_controllers()
            self.arm.to_position_mode()
            self.move_to_init()

            goal = GraspGoal()
            goal.arm = GraspGoal.ARM_LEFT
            self.grasp_client.send_goal(goal)
            self.grasp_client.wait_for_result()
            
            self.arm.unload_controllers()
            self.arm.to_position_mode()
            self.arm.load_position_trajectory_controller()
            self.arm.move_arm_to_named_target('idle_user_high')
            input('drop it?')
            self.gripper.open()
            cont = input('continue?')
   

    def move_to_init(self):
        print("moving to init pose.")
        self.dh.load_position_trajectory_controllers()
        self.arm.move_arm_to_named_target('idle_user_high')
        self.gripper.open()

if __name__ == '__main__':
    try:
        g = GraspTester()
    except KeyboardInterrupt:
        print( "Received control-c, stopping..." )
        exit( 0 )
