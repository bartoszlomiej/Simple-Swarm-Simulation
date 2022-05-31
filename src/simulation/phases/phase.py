from abc import ABC, abstractmethod
from simulation.robot import RobotState
from utils import SpotNeighbor as spot
import math


class Phase(ABC):
    '''
    The Phase is an abstract base class.
    It's subclasses will perform the specific operations needed for different phases.
    '''
    def __init__(self, Robot):
        self.robot = Robot

    @abstractmethod
    def check_phase(self):
        '''
        If robot in the same AS have higher phase, then we update ours
        '''
        pass

    @abstractmethod
    def upgrade(self, next_phase):
        '''
        Upgrades the phase to further one.
        '''
        pass

    @abstractmethod
    def update(self):
        '''
        Performes all operations in the given phase
        '''
        pass

    def makeMove(self):
        self.robot.state = RobotState.MOVING
        self.robot.velocity.x = self.robot.direction.x * self.robot.velocity_level
        self.robot.velocity.y = self.robot.direction.y * self.robot.velocity_level
        spot.border_return(self.robot)


    def checkAngle(self, n1, robot, n2):
        angle = math.atan2(n1.position.y - robot.position.y,
                           n1.position.x - robot.position.x) - math.atan2(
                               n2.position.y - robot.position.y,
                               n2.position.x - robot.position.x)

        return math.degrees(abs(angle))
