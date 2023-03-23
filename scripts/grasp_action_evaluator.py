#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import actionlib
from datetime import datetime
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int32
from controller_manager_msgs.srv import SwitchController
from diana7_msgs.srv import SetControlMode, SetControlModeRequest, SetImpedance, SetImpedanceRequest
from diana7_msgs.msg import CartesianState
from fabric_grasping.msg import GraspAction, GraspResult, GraspFeedback, GraspGoal

class GraspTester:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("diana7_grasping_demo")

        self.move_up_speed = 0.008
        self.pull_min_threshold = 100
        self.pull_max_threshold = 12000
        self.pull_loss_threshold = 50

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
        
        self.log = list()
        self.recording = False
        
        self.joint_state_msg = None
        self.forces_msg = None
        self.torques_msg = None
        self.gripper_currents_msg = None
        self.cartesian_state_msg = None
        self.tactile_sensor_1_msg = None
        self.tactile_sensor_2_msg = None
        self.load = None
        self.state = 'SETUP'
        self.joint_state_sub = rospy.Subscriber("/load", Int32, self.load_cb)
        self.joint_state_sub = rospy.Subscriber("/current", Int32, self.current_cb)
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        self.forces_sub = rospy.Subscriber("/diana_gripper/forces_raw", Joy, self.forces_cb)
        self.torques_sub = rospy.Subscriber("/diana_gripper/torques", Joy, self.torques_cb)
        self.torques_sub = rospy.Subscriber("/diana_gripper/currents", Joy, self.gripper_currents_cb)
        self.cartesian_state_sub = rospy.Subscriber("cartesian_state_controller/cartesian_state", CartesianState, self.cartesian_state_cb)
        self.tactile_sensor_sub_1 = rospy.Subscriber("tactile_sensor_data/1", TactileSensorArrayData, self.tactile_sensor_1_cb)
        self.tactile_sensor_sub_2 = rospy.Subscriber("tactile_sensor_data/2", TactileSensorArrayData, self.tactile_sensor_2_cb)

        self.grasp_client = actionlib.SimpleActionClient('grasp', GraspAction)

        cont = ''
        rospy.sleep(2)  # to fetch the messages first
        
        while cont == '' and not rospy.is_shutdown():
            # setting everything up to start from scratch
            self.recording = True
            self.state = 'INIT'
            print('initializing...')
            self.unload_controllers()
            self.to_position_mode()
            self.load_position_controller()
            self.move_to_init()

            self.state = 'GRASP'
            goal = GraspGoal()
            self.grasp_client.send_goal(goal)
            self.grasp_client.wait_for_result()
            
            self.state = 'PULL'
            self.unload_controllers()
            self.to_position_mode()
            self.load_twist_controller()
            self.move_up(speed=self.move_up_speed)

            max_load = 0
            had_load = False
            r = rospy.Rate(100)
            while self.cartesian_state_msg.pose.position.z < 0.85:  # as long as the arm is not too high up
                if self.load > self.pull_max_threshold:  # safety stop for too high load (don't want to rip the setup apart)
                    break
                if self.load > max_load:  # setting new max pull value
                    max_load = self.load
                elif max_load - self.load > self.pull_loss_threshold:  # stop pulling when load declined.
                    break
                r.sleep()
            self.stop()
            self.state = 'FINISHING'
            self.set_gripper_goal_position(0.2)
            self.send_gripper_goal()
            
            self.send_gripper_command(0, 0, 0)
            self.unload_controllers()
            self.recording = False
            cont = input('continue?')

    def log_timer_cb(self, event):
        if self.recording:
            self.log_state()

    def load_cb(self, msg):
        self.load = msg.data

    def current_cb(self, msg):
        self.current = msg.data

    def joint_state_cb(self, msg):
        # only save diana gripper joint states, ignore the robot joint states
        if 'diana_gripper/LFJ2' in msg.name:
            self.joint_state_msg = msg

    def forces_cb(self, msg):
        self.forces_msg = msg

    def torques_cb(self, msg):
        self.torques_msg = msg

    def gripper_currents_cb(self, msg):
        self.gripper_currents_msg = msg

    def cartesian_state_cb(self, msg):
        self.cartesian_state_msg = msg

    def tactile_sensor_1_cb(self, msg):
        self.tactile_sensor_1_msg = msg

    def tactile_sensor_2_cb(self, msg):
        self.tactile_sensor_2_msg = msg

    def log_state(self):
        self.log.append(', '.join([
            str(rospy.rostime.get_time()),  # timestamp
            self.state,  # state string
            str(self.load),  # pulling force
            str(self.current),  # gripper current
            str(self.cartesian_state_msg.pose.position.z),  # gripper height
            str(self.cartesian_state_msg.twist.linear.z),  # gripper up velocity
            ', '.join([str(e) for e in self.joint_state_msg.position]),  # servo positions (4 values)
            ', '.join([str(e) for e in self.joint_state_msg.velocity]),  # servo velocities (4 values)
            ', '.join([str(e) for e in self.joint_state_msg.effort]),  # servo efforts (4 values)
            ', '.join([str(e) for e in self.gripper_currents_msg.axes]),  # servo currents (4 values)
            ', '.join([str(e) for e in self.torques_msg.axes]),  # servo torques (4 values)
            ', '.join([str(e) for e in self.forces_msg.axes]),  # loadcell forces (4 values)
            ', '.join([str(e) for e in self.tactile_sensor_1_msg.data]),  # tactile sensor 1 data (36 values)
            ', '.join([str(e) for e in self.tactile_sensor_2_msg.data]),  # tactile sensor 2 data (36 values)
            ]))

    def write_log(self):
        dt = datetime.now()
        with open('logfile_' + dt.strftime('%y_%m_%d_%H_%M_%S'), 'w') as f:
            f.writelines(self.log)
        self.log = list()

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
