import rospy

from controller_manager_msgs.srv import SwitchController

        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switch_controllers = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)