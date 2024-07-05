#! /usr/bin/env python

#from __future__ import print_function
import sys
import rospy
import actionlib
import ransac_gpd.msg

def action_client():
    # Creates the SimpleActionClient
    client = actionlib.SimpleActionClient('get_grasping_point', ransac_gpd.msg.get_grasping_pointAction)

    # Waits until the action server has started up and started listening for goals
    print("Wait for action server...")
    client.wait_for_server()
    print("Action server found!")

    # Creates a goal to send to the action server
    goal = ransac_gpd.msg.get_grasping_pointGoal(grasping_action_goal=0)

    # Sends the goal to the action server
    client.send_goal(goal)
    print("Goal send...")
    # Waits for the server to finish performing the action
    client.wait_for_result()
    print("Result received!")

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS
        rospy.init_node('action_client_py')
        result = action_client()
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
