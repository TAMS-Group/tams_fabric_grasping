#! /usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SofthandTester:
    def __init__(self) -> None:
        rospy.init_node('SofthandTester')
        rospy.sleep(1)
        self.command_pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_motor_positions_trajectory_controller/command', JointTrajectory, queue_size=10)

        goal = 0.5
        growth = 0.02
        val = 0
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            while val < goal:
                self.send_command(val)
                val += growth
                r.sleep()
            input('open?')
            val = 0
            self.send_command(val)
            if not input('continue?') == '':
                break
    
    def send_command(self, value):
        msg = JointTrajectory()
        msg.joint_names = ['qbhand2m1_motor_1_joint', 'qbhand2m1_motor_2_joint']
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [value, 0]
        msg.points[0].time_from_start.nsecs = 100000000  # 1 nanosecond after start to do it immediately.
        self.command_pub.publish(msg)

if __name__ == '__main__':
    try:
        st = SofthandTester()
    except KeyboardInterrupt:
        print( "Received control-c, stopping..." )
        exit( 0 )
    except rospy.ROSInterruptException:
        pass
