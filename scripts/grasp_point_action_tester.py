#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import ransac_gpd.msg
import numpy as np
import actionlib
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Joy, JointState
from controller_manager_msgs.srv import SwitchController
from diana7_msgs.srv import SetControlMode, SetControlModeRequest, SetImpedance, SetImpedanceRequest
from diana7_msgs.msg import CartesianState
from fabric_grasping.msg import GraspAction, GraspResult, GraspFeedback, GraspGoal

class GraspTester:
    def __init__(self):
        #moveit_commander.roscpp_initialize(sys.argv)
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

        self.gripper_goal_pub = rospy.Publisher('diana_gripper/simple_goal', JointState, queue_size=10)
        self.arm_vel_pub = rospy.Publisher('/cartesian_twist_controller/command', Twist, queue_size=10)

        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        self.gripper_group = moveit_commander.MoveGroupCommander('gripper')
        

        self.grasp_client = actionlib.SimpleActionClient('get_grasping_point', ransac_gpd.msg.get_grasping_pointAction)
        self.state = actionlib.SimpleGoalState()
        print("Wait for action server...")
        self.grasp_client.wait_for_server()
        
        cont = ''
        
        # setting everything up to start from scratch
            
        print('initializing...')
        self.scene.remove_world_object("textile_collision_box")
        self.unload_controllers()
        self.to_position_mode()
        self.load_position_controller()
        self.move_to_init()
        input("Beginn?")

        while cont == '' and not rospy.is_shutdown():
            goal = ransac_gpd.msg.get_grasping_pointGoal(grasping_action_goal=0)
            print("send goal...")
            self.grasp_client.send_goal(goal)
            print("wait for result...")
            self.grasp_client.wait_for_result()
            
            print(self.grasp_client.get_state())

            if self.grasp_client.get_state() == 4:
                print("Aborted!")

            if self.grasp_client.get_state() == 3:
                pose_result = self.grasp_client.get_result().grasping_pose

                print("result:", pose_result)
                self.load_twist_controller()
                rospy.sleep(0.5)
                self.send_gripper_command(0, 0, 0)
                input("Check pose! Continue?")

                self.load_position_controller()

                # publish collision box
                p = PoseStamped()
                p.header.frame_id = self.robot.get_planning_frame()
                p.pose.position.x = 0.4
                p.pose.position.y = -0.20
                p.pose.position.z = 0.8
                self.scene.add_box("textile_collision_box", p, (0.4, 0.6, 0.25))

                self.move_arm_to_pose_target(pose_result, z_offset=0.13)
                
                # remove collision box
                self.scene.remove_world_object("textile_collision_box")
                
                self.load_twist_controller()
                self.move_down_cm(13)
                self.send_gripper_command_z_correction(1.1, 0, 0.1)
                self.move_up_cm(13)

                self.load_position_controller()
                self.move_arm_to_named_target('idle_corner_very_high')
                input('Drop it? ')
                
            self.send_gripper_command(0, 0, 0)
            rospy.sleep(1.5)
            self.move_arm_to_named_target('idle_room_high')
            cont = input('Continue? If not, enter something.')
            
    def move_up_cm(self, dist=0.0):
        self.move_up(0.05)
        rospy.sleep(dist*0.2)
        self.stop()

    def move_down_cm(self, dist=0.0):
        self.move_down(0.05)
        rospy.sleep(dist*0.2)
        self.stop()
            
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
        # self.set_gripper_goal_position(-0.5)ge, a PoseStamped message or a 
        self.set_gripper_goal_position(0.2)
        self.send_gripper_goal()
        self.move_arm_to_named_target('idle_room_high')

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

    def send_gripper_command_z_correction(self, angle, proximal, distal):
        goal_msg = JointState()
        goal_msg.name = ['parallel_gripper_angle', 'parallel_gripper_proximal_tilt', 'parallel_gripper_distal_tilt']
        goal_msg.position = [angle, proximal, distal]
        self.gripper_goal_pub.publish(goal_msg)
        self.move_down(0.021)
        rospy.sleep(1.35)
        self.stop()

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

    def move_arm_to_pose_target(self, pose:PoseStamped, z_offset=0):
        t = rospy.Duration(1)
        max_i = 2
        goal_pose = copy.deepcopy(pose)
        goal_pose.pose.position.z += z_offset
        self.arm_group.set_pose_target(goal_pose)
        success = False
        i = 0
        while not success:
            success = self.arm_group.go(wait=True)
            if i > max_i:
                x = input()
                if len(x) > 0:
                    sys.exit()
            print(f"retrying arm motion to \"{goal_pose}\"")
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