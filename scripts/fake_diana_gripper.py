#! /usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('fake_diana_gripper')
rospy.sleep(1)

js_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    
    js_msg = JointState()
    js_msg.header.stamp = rospy.Time.now()
    js_msg.header.frame_id = "Maerchenland"
    js_msg.name = ["diana_gripper/LFJ2", "diana_gripper/LFJ1", "diana_gripper/RFJ2", "diana_gripper/RFJ1"]
    js_msg.position = [0, 0, 0, 0]
    js_msg.velocity = [0, 0, 0, 0]
    js_msg.effort = [0, 0, 0, 0]
    js_pub.publish(js_msg)
    r.sleep()
