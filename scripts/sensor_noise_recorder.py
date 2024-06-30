#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import numpy as np
import actionlib
from datetime import datetime
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int32
from tams_tactile_sensor_array.msg import TactileSensorArrayData

class NoiseRecorder:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("sensor_noise_recorder")

        self.log = list()
        self.recording = False
        

        self.tactile_sensor_1_msg = None
        self.tactile_sensor_2_msg = None
        self.tactile_sensor_1_change = None
        self.tactile_sensor_2_change = None

        self.tactile_sensor_sub_1 = rospy.Subscriber("tactile_sensor_data/1", TactileSensorArrayData, self.tactile_sensor_1_cb)
        self.tactile_sensor_sub_2 = rospy.Subscriber("tactile_sensor_data/2", TactileSensorArrayData, self.tactile_sensor_2_cb)

        rospy.sleep(2)  # to fetch the messages first
        log_timer = rospy.Timer(rospy.Duration(0.05), self.log_timer_cb)#20 hz

        self.write_log_header()
        self.recording = True

        while len(self.log) < 1000:
            print(len(self.log))
            rospy.sleep(1)
        self.write_log()

    def log_timer_cb(self, event):
        if self.recording:
            self.log_state()

    def tactile_sensor_1_cb(self, msg):
        if self.tactile_sensor_1_msg is not None:
            self.tactile_sensor_1_change = np.array(msg.data) - np.array(self.tactile_sensor_1_msg.data)
        self.tactile_sensor_1_msg = msg

    def tactile_sensor_2_cb(self, msg):
        if self.tactile_sensor_2_msg is not None:
            self.tactile_sensor_2_change = np.array(msg.data) - np.array(self.tactile_sensor_2_msg.data)
        self.tactile_sensor_2_msg = msg

    def log_state(self):
        cont = False
        if not self.tactile_sensor_1_msg: 
            print('tactile_sensor_1_msg missing!')
            cont = True
        if not self.tactile_sensor_2_msg: 
            print('tactile_sensor_2_msg missing!')
            cont = True
        if cont: 
            return
        self.log.append(', '.join([
            str(rospy.rostime.get_time()),  # timestamp
            ', '.join([str(e) for e in self.tactile_sensor_1_change]),
            ', '.join([str(e) for e in self.tactile_sensor_2_change]),
            ]) + '\n')

    def write_log_header(self):

        self.log.append(', '.join([
            'timestamp',
            ', '.join(['tactile_sensor_1_' + str(i) for i in range(36)]),
            ', '.join(['tactile_sensor_2_' + str(i) for i in range(36)]),
            ]) + '\n')

    def write_log(self):
        dt = datetime.now()
        with open('noisefile_' + dt.strftime('%y_%m_%d_%H_%M_%S') + '.csv', 'w') as f:
            f.writelines(self.log)
        self.log = list()


if __name__ == '__main__':
    try:
        nr = NoiseRecorder()
    except KeyboardInterrupt:
        nr.stop()
        print( "Received control-c, stopping..." )
        exit( 0 )
    except rospy.ROSInterruptException:
        pass
