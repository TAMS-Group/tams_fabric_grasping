#! /usr/bin/env python

import sys
import copy
import rospy
import actionlib
import moveit_commander
from policies.policies import Policy, PositionPolicy, GripperCurrentPolicy, TactilePolicy
from diana7_msgs.msg import CartesianState
from fabric_grasping.msg import GraspAction, GraspResult, GraspFeedback, GraspGoal
from tams_diana7_tools.dual_diana_helper import DualDianaHelper

class GraspServer:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("diana7_grasp_server")

        self.action_name = 'grasp'
        self.max_grasp_torque_2 = 200
        self.max_grasp_torque_3 = 100
        self.max_pressure = 8
        self.pressure_threshold = 5  # pressure hysteresis when approaching table

        self.approach_speed = 0.02
        self.retreat_speed = 0.01
        self.fine_speed = 0.005
        self.gripper_goal = 0.5
        
        self.policy = PositionPolicy(gripper_goal=1, move_speed=0.01)
        # self.policy = GripperCurrentPolicy(gripper_current_goal=600)
        # self.policy = GripperCurrentPolicy(gripper_current_goal=1000)
        # self.policy = TactilePolicy(cell_active_threshold=900, max_gripper_current=1800, max_cell_loss=3)
        self.policy.gripper_goal_cb(self.gripper_goal)
    
    
        self.cartesian_state = {
            'r': None,
            'l': None
        }

        self.dh = DualDianaHelper(load_moveit=True, arm_l_active=True, arm_r_active=True, no_grippers=False)

        self.cartesian_state_sub_l = rospy.Subscriber("/arm_l/cartesian_state_controller/cartesian_state_controller/cartesian_state", CartesianState, self.cartesian_state_l_cb)
        self.cartesian_state_sub_r = rospy.Subscriber("/arm_r/cartesian_state_controller/cartesian_state_controller/cartesian_state", CartesianState, self.cartesian_state_r_cb)

        self.grasp_action_server = actionlib.SimpleActionServer(self.action_name, GraspAction, execute_cb=self.grasp_cb, auto_start = False)
        self.grasp_action_server.start()


    def grasp_cb(self, goal: GraspGoal):


        
        if goal.arm == GraspGoal.ARM_RIGHT:
            arm = self.dh.arm_r
            gripper = self.dh.gripper_r
            lr = 'r'
        elif goal.arm == GraspGoal.ARM_LEFT:
            arm = self.dh.arm_l
            gripper = self.dh.gripper_l
            lr = 'l'
        else:
            rospy.logerr('Invalid arm selection')
            return
        arm.load_twist_controller()
        

        print('start control loop.')

        self.policy.reset()

        self.gripper_goal = 0.5
        grasp_finished = False
        first_contact = False
        backdrive = 0

        r = rospy.Rate(50) # 50hz
        while not grasp_finished and not rospy.is_shutdown():
            # contact with the piece (towards the table) has been made, slightly retracting and switching to impedance mode...
            if backdrive > 0: 
                arm.set_arm_velocity(vz=self.retreat_speed)
                backdrive -= 1
                if backdrive == 0:
                    arm.unload_controllers()
                    arm.to_impedance_mode()
                    arm.load_twist_controller()
            # not in backdrive mode, acting normally.
            else: 
                pressure = self.cartesian_state[lr].wrench.linear.z
                # print(pressure)
                if pressure > self.pressure_threshold:
                    # if not in impedance mode already
                    if not first_contact:
                        # move slightly back and switch to impedance mode
                        first_contact = True
                        backdrive = 0
                # move up in case the maximal pressure was surpassed
                if pressure > self.max_pressure + self.pressure_threshold:
                    # print('up')
                    arm.set_arm_velocity(vz=self.approach_speed)
                # move down if the pressure is too low
                elif pressure < self.max_pressure - self.pressure_threshold:
                    # print('down')
                    # move slowly when first contact was made as collision is imminent.
                    if first_contact:
                        arm.set_arm_velocity(vz=-self.fine_speed)
                    else:
                        arm.set_arm_velocity(vz=-self.approach_speed)
                # pressure towards the table is in desired zone
                else:
                    arm.set_arm_velocity()
                    # print('stop')
                    # if len(self.torques_2) and self.torques_2.mean() < self.max_grasp_torque_2:
                    self.apply_policy()
                    gripper.move_to(self.gripper_goal)
                if self.policy.finished():  # self.torques_2.mean() > self.max_grasp_torque_2 or self.torques_3.mean() > self.max_grasp_torque_3:
                    grasp_finished = True
            r.sleep()
        self.grasp_action_server.set_succeeded(GraspResult())

    def cartesian_state_l_cb(self, msg):
        self.policy.state_cb(msg)
        self.cartesian_state['l'] = msg

    def cartesian_state_r_cb(self, msg):
        self.policy.state_cb(msg)
        self.cartesian_state['r'] = msg

    def set_gripper_goal_position(self, pos):
        self.gripper_goal = pos
        self.policy.gripper_goal_cb(self.gripper_goal)

    def apply_policy(self):
        self.change_gripper_goal_position(self.policy.decide_action() * self.policy.gripper_move_speed)

    def change_gripper_goal_position(self, chng):
        self.gripper_goal += chng
        self.policy.gripper_goal_cb(self.gripper_goal)


if __name__ == '__main__':
    try:
        g = GraspServer()
    except KeyboardInterrupt:
        print( "Received control-c, stopping..." )
        exit( 0 )
