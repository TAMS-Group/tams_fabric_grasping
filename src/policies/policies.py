import abc
from tams_tactile_sensor_array.msg import TactileSensorArrayData
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from diana7_msgs.msg import CartesianState
import numpy as np

class Policy(abc.ABC):

    @abc.abstractmethod
    def tactile_cb(self, msg: TactileSensorArrayData):
        pass

    @abc.abstractmethod
    def gripper_current_cb(self, msg: TactileSensorArrayData):
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

    @abc.abstractmethod
    def reset(self) -> None:
        pass

class PositionPolicy(Policy):

    def __init__(self, gripper_goal: float=1.1, move_speed: float=0.005):
        self.overall_gripper_goal = gripper_goal
        self.gripper_move_speed = move_speed
        self.reset()

    def reset(self):
        self.gripper_goal = None
        self.gripper_current = None
        self.state = None
        self.tactile1 = None
        self.tactile2 = None
        self.torque = None
        self.force = None

    def tactile_cb(self, msg: TactileSensorArrayData):
        if msg.sensor_id == 1:
            self.tactile1 = msg.data
        if msg.sensor_id == 2:
            self.tactile2 = msg.data

    def gripper_current_cb(self, current: float):
        self.gripper_current = current

    def force_cb(self, msg: Joy):
        self.force = msg

    def torque_cb(self, msg: Joy):
        self.torque = msg

    def state_cb(self, msg: CartesianState):
        self.state = msg

    def gripper_goal_cb(self, goal: int):
        self.gripper_goal = goal

    def decide_action(self) -> int:
        if self.gripper_goal is not None and self.gripper_goal < self.overall_gripper_goal:
            return 1
        return 0

    def finished(self) -> bool:
        return self.gripper_goal and self.gripper_goal >= self.overall_gripper_goal


class GripperCurrentPolicy(Policy):

    def __init__(self, gripper_current_goal: float=1000):
        self.overall_gripper_current_goal = gripper_current_goal
        self.gripper_move_speed = 0.005
        self.reset()

    def reset(self):
        self.gripper_goal = None
        self.gripper_current = np.array([])
        self.state = None
        self.tactile1 = None
        self.tactile2 = None
        self.torque = None
        self.force = None
        

    def tactile_cb(self, msg: TactileSensorArrayData):
        if msg.sensor_id == 1:
            self.tactile1 = msg.data
        if msg.sensor_id == 2:
            self.tactile2 = msg.data

    def force_cb(self, msg: Joy):
        self.force = msg

    def torque_cb(self, msg: Joy):
        
        self.torque = msg

    def gripper_current_cb(self, current: float):
         
        self.gripper_current = np.concatenate((self.gripper_current, [current]))[-10:]
        
    def state_cb(self, msg: CartesianState):
        self.state = msg

    def gripper_goal_cb(self, goal: int):
        self.gripper_goal = goal

    def decide_action(self) -> int:
        if self.gripper_current.size > 9 and self.gripper_current.mean() < self.overall_gripper_current_goal:
            return 0.5
        if self.gripper_current.size > 9 and self.gripper_current.mean() > self.overall_gripper_current_goal:
            return -0.5
        return 0

    def finished(self) -> bool:
        print("sum")
        print(abs(self.gripper_current - self.overall_gripper_current_goal).sum())
        return self.gripper_current.size > 9 and abs(self.gripper_current - self.overall_gripper_current_goal).sum() < self.gripper_current.size * 100




class TactilePolicy(Policy):

    def __init__(self, cell_active_threshold: int=900, max_gripper_current: int=1800, max_cell_loss: int=4):
        self.gripper_move_speed = 0.005
        self.cell_active_threshold = cell_active_threshold
        self.max_gripper_current = max_gripper_current
        self.max_cell_loss = max_cell_loss
        self.reset()

    def reset(self):
        self.overall_gripper_goal = 0
        self.gripper_goal = None
        self.gripper_current = None
        self.state = None
        self.tactile1 = None
        self.tactile2 = None
        self.torque = None
        self.force = None
        self.tactile_results = dict()
        self.explore = True
    
    def num_cells_over_threshold(self):
        if self.tactile1 is not None and self.tactile2 is not None:
            return (np.array(self.tactile1 + self.tactile2) > self.cell_active_threshold).sum()
        return 0

    def tactile_cb(self, msg: TactileSensorArrayData):
        if msg.sensor_id == 1:
            self.tactile1 = 1023 - np.array(msg.data)
        if msg.sensor_id == 2:
            self.tactile2 = 1023 - np.array(msg.data)

    def gripper_current_cb(self, current: float):
        if self.explore:
            if self.max_gripper_current < current:
                # max load reached, stop exploring
                self.explore = False
                print(self.tactile_results)

                # set position with maximal tactile contact as goal
                self.overall_gripper_goal = self.tactile_results[max(self.tactile_results.keys())]
        self.gripper_current = current

    def force_cb(self, msg: Joy):
        self.force = msg

    def torque_cb(self, msg: Joy):
        self.torque = msg

    def state_cb(self, msg: CartesianState):
        self.state = msg

    def gripper_goal_cb(self, goal: int):
        if self.explore:
            cells = self.num_cells_over_threshold()
            if cells not in self.tactile_results:
                self.tactile_results[cells] = goal
            elif cells in self.tactile_results and self.tactile_results[cells] < goal:
                self.tactile_results[cells] = goal
        self.gripper_goal = goal

    def decide_action(self) -> int:
        print(self.num_cells_over_threshold())
        # close further and further when exploring
        if self.explore: 
            return 0.5

        cellcount = self.num_cells_over_threshold()
        # opened too far, start again
        if cellcount < 2:
            self.reset()
        """
        #  navigate to previously decided goal
        if self.gripper_goal and self.gripper_goal < self.overall_gripper_goal:
            return 0.5
        if self.gripper_goal and self.gripper_goal > self.overall_gripper_goal:
            return -0.5
        """

        # open up after exploration until satisfying result is reached
        if cellcount + self.max_cell_loss < max(self.tactile_results.keys()):
            return -0.5

        return 0

    def finished(self) -> bool:
        # stop when close enough to goal and exploration is done.
        # return not self.explore and self.gripper_goal and abs(self.gripper_goal - self.overall_gripper_goal) < 0.01
        done = not self.explore and self.num_cells_over_threshold() + self.max_cell_loss >= max(self.tactile_results.keys())
        if done:
            print(f'finishing with {self.num_cells_over_threshold()} cells after maximum with {max(self.tactile_results.keys())} cells')
        return done