#! /usr/bin/env python

import sys
import copy
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from policies.policies import Policy, PositionPolicy
from tams_tactile_sensor_array.msg import TactileSensorArrayData
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from controller_manager_msgs.srv import SwitchController
from diana7_msgs.srv import SetControlMode, SetControlModeRequest, SetImpedance, SetImpedanceRequest
from diana7_msgs.msg import CartesianState
from fabric_grasping.msg import GraspAction, GraspResult, GraspFeedback, GraspGoal

class GraspServer:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("diana7_grasp_server")

        self.action_name = 'grasp'
        self.max_grasp_torque_2 = 200
        self.max_grasp_torque_3 = 100
        self.max_pressure = 15
        self.pressure_threshold = 5  # pressure hysteresis when approaching table

        self.approach_speed = 0.04
        self.retreat_speed = 0.01
        self.fine_speed = 0.005
        self.gripper_goal = 0.2

        self.policy = PositionPolicy(gripper_goal=1.5)
        self.policy.gripper_goal_cb(self.gripper_goal)
        self.new_tactile_data = False
        self.last_tactile_sensor_data_1 = None
        self.last_tactile_sensor_data_2 = None
        self.forces = None
        self.torques_2 = np.array([])
        self.torques_3 = np.array([])
        self.cartesian_state = None

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        self.gripper_group = moveit_commander.MoveGroupCommander('gripper')


        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switch_controllers = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        rospy.wait_for_service('/diana7_hardware_interface/set_control_mode')
        self.set_mode = rospy.ServiceProxy('/diana7_hardware_interface/set_control_mode', SetControlMode)
        rospy.wait_for_service('/diana7_hardware_interface/set_cartesian_impedance')
        self.set_cart_imp = rospy.ServiceProxy('/diana7_hardware_interface/set_cartesian_impedance', SetImpedance)
        rospy.wait_for_service('/diana7_hardware_interface/set_joint_impedance')
        self.set_joint_imp = rospy.ServiceProxy('/diana7_hardware_interface/set_joint_impedance', SetImpedance)

        self.forces_sub = rospy.Subscriber("/diana_gripper/forces_raw", Joy, self.forces_cb)
        self.torques_sub = rospy.Subscriber("/diana_gripper/torques", Joy, self.torques_cb)
        self.cartesian_state_sub = rospy.Subscriber("cartesian_state_controller/cartesian_state", CartesianState, self.cartesian_state_cb)
        self.tactile_sensor_sub_1 = rospy.Subscriber("tactile_sensor_data/1", TactileSensorArrayData, self.tactile_sensor_cb)
        self.tactile_sensor_sub_2 = rospy.Subscriber("tactile_sensor_data/2", TactileSensorArrayData, self.tactile_sensor_cb)


        self.gripper_goal_pub = rospy.Publisher('diana_gripper/simple_goal', JointState, queue_size=10)
        self.arm_vel_pub = rospy.Publisher('/cartesian_twist_controller/command', Twist, queue_size=10)

        self.grasp_action_server = actionlib.SimpleActionServer(self.action_name, GraspAction, execute_cb=self.grasp_cb, auto_start = False)
        self.grasp_action_server.start()


    def grasp_cb(self, goal: GraspGoal):


        
        self.load_twist_controller()

        print('start control loop.')

        grasp_finished = False
        first_contact = False
        backdrive = 0

        r = rospy.Rate(50) # 50hz
        while not grasp_finished and not rospy.is_shutdown():
            # contact with the piece (towards the table) has been made, slightly retracting and switching to impedance mode...
            if backdrive > 0: 
                self.move_up(self.retreat_speed)
                backdrive -= 1
                if backdrive == 0:
                    self.unload_controllers()
                    self.to_impedance_mode()
                    self.load_twist_controller()
            # not in backdrive mode, acting normally.
            else: 
                pressure = self.cartesian_state.wrench.linear.z
                print(pressure)
                if pressure > self.pressure_threshold:
                    # if not in impedance mode already
                    if not first_contact:
                        # move slightly back and switch to impedance mode
                        first_contact = True
                        backdrive = 25
                # move up in case the maximal pressure was surpassed
                if pressure > self.max_pressure + self.pressure_threshold:
                    print('up')
                    self.move_up(self.approach_speed)
                # move down if the pressure is too low
                elif pressure < self.max_pressure - self.pressure_threshold:
                    print('down')
                    # move slowly when first contact was made as collision is imminent.
                    if first_contact:
                        self.move_down(self.fine_speed)
                    else:
                        self.move_down(self.approach_speed)
                # pressure towards the table is in desired zone
                else:
                    self.stop()
                    print('stop')
                    # if len(self.torques_2) and self.torques_2.mean() < self.max_grasp_torque_2:
                    self.apply_policy()
                    self.send_gripper_goal()
                if self.policy.finished():  # self.torques_2.mean() > self.max_grasp_torque_2 or self.torques_3.mean() > self.max_grasp_torque_3:
                    print(self.torques_2)
                    print(self.torques_2.mean())
                    print(self.torques_3)
                    print(self.torques_3.mean())
                    grasp_finished = True
            r.sleep()
        self.grasp_action_server.set_succeeded(GraspResult())


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

    def forces_cb(self, msg):
        self.forces = msg
        self.policy.force_cb(msg)

    def torques_cb(self, msg):
        # self.torques_2 = np.concatenate((self.torques_2, [msg.axes[2]]))[-10:]
        # self.torques_3 = np.concatenate((self.torques_3, [msg.axes[3]]))[-10:]
        self.policy.torque_cb(msg)

    def tactile_sensor_cb(self, msg):
        self.policy.tactile_cb(msg)
        if msg.sensor_id == 1:
            self.last_tactile_sensor_data_1 = msg.data
        if msg.sensor_id == 2:
            self.last_tactile_sensor_data_2 = msg.data

    def cartesian_state_cb(self, msg):
        self.policy.state_cb(msg)
        self.cartesian_state = msg
        
    def set_gripper_goal_position(self, pos):
        self.gripper_goal = pos
        self.policy.gripper_goal_cb(self.gripper_goal)

    def apply_policy(self):
        self.change_gripper_goal_position(self.policy.decide_action() * self.policy.gripper_move_speed)

    def change_gripper_goal_position(self, chng):
        self.gripper_goal += chng
        self.policy.gripper_goal_cb(self.gripper_goal)

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
        g = GraspServer()
    except KeyboardInterrupt:
        g.stop()
        print( "Received control-c, stopping..." )
        exit( 0 )
    except rospy.ROSInterruptException:
        pass
