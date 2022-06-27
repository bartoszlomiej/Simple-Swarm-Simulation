from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
#from simulation.phases.shapes.V_Shape import V_Shape
from simulation.phases.CountToTwo import CountToTwo
from utils import SpotNeighbor as spot
import math

class W_Shape(CountToTwo):
    def __init__(self, superAS):
        super().__init__(Robot, superAS, perpendicular_direction)
        self.perpendicular_direction = perpendicular_direction
        self.phase = 6

    def update(self):
        self.check_phase()
        self.robot.direction = Direction(1, 1)
        self._countToTwo()
        self.robot.velocity = Velocity(0, 0)
        self._keepStaticLine()
        self.robot.isAlloneInSupercluster()
        
    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 8:
                    robot.broadcast["superAS"] = self.robot.super_cluster_id
                    self.upgrade(m["Phase"], self.robot.super_cluster_id)
                    return
        self.robot.update_color()  #just for dbg                
                
    def upgrade(self, next_phase=4, superAS=None):
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 5:
            self.robot.faza = StepForward(self.robot, superAS)
