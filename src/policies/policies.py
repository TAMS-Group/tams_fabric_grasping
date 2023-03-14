import abc
from tams_tactile_sensor_array.msg import TactileSensorArrayData
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from diana7_msgs.msg import CartesianState

class Policy(abc.ABC):

    @abc.abstractmethod
    def tactile_cb(self, msg: TactileSensorArrayData):
        pass

    @abc.abstractmethod
    def force_cb(self, msg: Joy):
        pass

    @abc.abstractmethod
    def torque_cb(self, msg: Joy):
        pass

    @abc.abstractmethod
    def state_cb(self, msg: CartesianState):
        pass

    @abc.abstractmethod
    def gripper_goal_cb(self, goal: float):
        pass

    @abc.abstractmethod
    def decide_action(self) -> int:
        pass

    @abc.abstractmethod
    def finished(self) -> bool:
        pass

class PositionPolicy(Policy):

    def __init__(self, gripper_goal: float=1.5):
        self.overall_gripper_goal = gripper_goal
        self.gripper_goal = None
        self.state = None
        self.tactile1 = None
        self.tactile2 = None
        self.torque = None
        self.force = None
        self.gripper_move_speed = 0.005
        

    def tactile_cb(self, msg: TactileSensorArrayData):
        if msg.sensor_id == 1:
            self.tactile1 = msg.data
        if msg.sensor_id == 2:
            self.tactile2 = msg.data

    def force_cb(self, msg: Joy):
        self.force = msg

    def torque_cb(self, msg: Joy):
        self.torque = msg

    def state_cb(self, msg: CartesianState):
        self.state = msg

    def gripper_goal_cb(self, goal: int):
        self.gripper_goal = goal

    def decide_action(self) -> int:
        if self.gripper_goal and self.gripper_goal < self.overall_gripper_goal:
            return 1
        return 0

    def finished(self) -> bool:
        return self.gripper_goal and self.gripper_goal > self.overall_gripper_goal
