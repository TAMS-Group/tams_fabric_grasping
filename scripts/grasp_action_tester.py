#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from controller_manager_msgs.srv import SwitchController
from diana7_msgs.srv import SetControlMode, SetControlModeRequest, SetImpedance, SetImpedanceRequest
from diana7_msgs.msg import CartesianState
from fabric_grasping.msg import GraspAction, GraspResult, GraspFeedback, GraspGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GraspTester:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("diana7_grasping_demo")

        self.robot = moveit_commander.RobotCommander()

        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switch_controllers = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        rospy.wait_for_service('/diana7_hardware_interface/set_control_mode')
        self.set_mode = rospy.ServiceProxy('/diana7_hardware_interface/set_control_mode', SetControlMode)
        rospy.wait_for_service('/diana7_hardware_interface/set_cartesian_impedance')
        self.set_cart_imp = rospy.ServiceProxy('/diana7_hardware_interface/set_cartesian_impedance', SetImpedance)
        rospy.wait_for_service('/diana7_hardware_interface/set_joint_impedance')
        self.set_joint_imp = rospy.ServiceProxy('/diana7_hardware_interface/set_joint_impedance', SetImpedance)


        self.use_softhand = False

        if self.use_softhand:
            self.softhand_goal_pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_motor_positions_trajectory_controller/command', JointTrajectory, queue_size=10)
        else:
            self.gripper_goal_pub = rospy.Publisher('diana_gripper/simple_goal', JointState, queue_size=10)

        self.arm_vel_pub = rospy.Publisher('/cartesian_twist_controller/command', Twist, queue_size=10)

        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        # self.gripper_group = moveit_commander.MoveGroupCommander('gripper')

        self.grasp_client = actionlib.SimpleActionClient('grasp', GraspAction)
        
        cont = ''
        
        while cont == '' and not rospy.is_shutdown():
            # setting everything up to start from scratch
            print('initializing...')
            self.unload_controllers()
            self.to_position_mode()
            self.load_position_controller()
            self.move_to_init()

            goal = GraspGoal()
            self.grasp_client.send_goal(goal)
            self.grasp_client.wait_for_result()
            
            self.unload_controllers()
            self.to_position_mode()
            self.load_position_controller()
            if self.use_softhand:
                self.move_arm_to_named_target('idle_user_softhand')
            else:
                self.move_arm_to_named_target('idle_user_high')
            input('drop it?')
            if self.use_softhand:
                self.send_softhand_command(0)
            else:
                self.send_gripper_command(0, 0, 0)
            self.unload_controllers()
            cont = input('continue?')


    def move_down(self, speed=0.004):
        speed = -abs(speed)
        msg = Twist()
        msg.linear.z = speed
        self.arm_vel_pub.publish(msg)


    def move_up(self, speed=0.004):
        speed = abs(speed)
        msg = Twist()
        msg.linear.z = speed
        self.arm_vel_pub.publish(msg)

    def stop(self):
        msg = Twist()
        self.arm_vel_pub.publish(msg)
    
    def load_position_controller(self):
        print('loading position controller')
        try:
            self.switch_controllers(['joint_position_trajectory_controller'], ['cartesian_twist_controller'], 1, False, 0)
        except rospy.ServiceException as e:
            print("Service did not process request: " + str(e))

    def load_twist_controller(self):
        print('loading twist controller')
        try:
            self.switch_controllers(['cartesian_twist_controller'], ['joint_position_trajectory_controller'], 1, False, 0)
        except rospy.ServiceException as e:
            print("Service did not process request: " + str(e))

    def unload_controllers(self):
        try:
            self.switch_controllers([], ['cartesian_twist_controller', 'joint_position_trajectory_controller'], 1, False, 0)
        except rospy.ServiceException as e:
            print("Service did not process request: " + str(e))

    def move_to_init(self):
        self.load_position_controller()
        print("moving to init pose.")
        # self.move_gripper_to_named_target('wide_open_parallel')
        # self.set_gripper_goal_position(-0.5)
        if self.use_softhand:
            self.send_softhand_command(0, 0)
            self.move_arm_to_named_target('idle_user_softhand')
        else:
            self.set_gripper_goal_position(0.2)
            self.send_gripper_goal()
            self.move_arm_to_named_target('idle_user_high')

    def set_gripper_goal_position(self, pos):
        self.gripper_goal = pos

    def change_gripper_goal_position(self, chng):
        self.gripper_goal += chng

    def send_gripper_goal(self):
        self.send_gripper_command(self.gripper_goal, 0, 0.06)

    def send_gripper_command(self, angle, proximal, distal):
        goal_msg = JointState()
        goal_msg.name = ['parallel_gripper_angle', 'parallel_gripper_proximal_tilt', 'parallel_gripper_distal_tilt']
        goal_msg.position = [angle, proximal, distal]
        self.gripper_goal_pub.publish(goal_msg)

    def send_softhand_command(self, motor1, motor2=0):
        msg = JointTrajectory()
        msg.joint_names = ['qbhand2m1_motor_1_joint', 'qbhand2m1_motor_2_joint']
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [motor1, motor2]
        msg.points[0].time_from_start.nsecs = 10000000
        self.softhand_goal_pub.publish(msg)

    def to_impedance_mode(self):
        print("switching to impedance mode...")
        self.unload_controllers()
        try:
            resp1 = self.set_mode(1)
            print(resp1)
            r2 = SetImpedanceRequest()
            r2.stiffness = [2000, 2000, 1700, 1700, 1000, 100, 700]
            r2.damping = [30, 30, 20, 20, 9, 2, 6]
            resp2 = self.set_joint_imp(r2)
            # r2.stiffness = [3000, 3000, 3000, 500, 500, 500]
            # r2.damping = [30, 30, 30, 5, 5, 5]
            # resp2 = self.set_cart_imp(r2)
            print(resp2)
        except rospy.ServiceException as e:
            print("Service did not process request: " + str(e))    

    def to_position_mode(self):
        print("switching to position mode...")
        try:
            resp1 = self.set_mode(0)
            print(resp1)
            # input('mode changed?')
        except rospy.ServiceException as e:
            print("Service did not process request: " + str(e))    

    def move_gripper_to_named_target(self, target_name):
        t = rospy.Duration(1)
        max_i = 2
        self.gripper_group.set_named_target(target_name)
        success = False
        i = 0
        while not success:
            success = self.gripper_group.go(wait=True)
            if i > max_i:
                x = input()
                if len(x) > 0:
                    sys.exit()
            print(f"retrying gripper motion to \"{target_name}\"")
            i += 1
            rospy.sleep(t)

    def move_arm_to_named_target(self, target_name):
        t = rospy.Duration(1)
        max_i = 2
        self.arm_group.set_named_target(target_name)
        success = False
        i = 0
        while not success:
            success = self.arm_group.go(wait=True)
            if i > max_i:
                x = input()
                if len(x) > 0:
                    sys.exit()
            print(f"retrying arm motion to \"{target_name}\"")
            i += 1
            rospy.sleep(t)

if __name__ == '__main__':
    try:
        g = GraspTester()
    except KeyboardInterrupt:
        g.stop()
        print( "Received control-c, stopping..." )
        exit( 0 )
    except rospy.ROSInterruptException:
        pass
