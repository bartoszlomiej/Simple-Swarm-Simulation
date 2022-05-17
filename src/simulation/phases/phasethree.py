import pygame as pg
import math
from utils import SpotNeighbor as spot
from simulation.phases.phase import Phase
import simulation.phases.phaseone as ph1
import simulation.phases.phasetwo as ph2
from simulation.robot import RobotState
from simulation.robot.Velocity import Velocity


class PhaseThree(Phase):
    def __init__(self, Robot, superAS):
        super().__init__(Robot)
        self.phase = 3
        self.robot.super_cluster_id = superAS
        self.isIncreased = False
        self.timerSet = False
        self.robot.velocity = Velocity(0, 0)
        self.robot.state = RobotState.STOPPED

    def __allowedAS(self):
        '''
        Returns the AS's that are in the same superAS that can be spot by the given robot.
        '''
        allowed = []
        allowed.append(self.robot.cluster_id)
        for n in self.robot.neighbors:
            if n.super_cluster_id == self.robot.super_cluster_id:
                if not n.cluster_id in allowed:
                    allowed.append(n.cluster_id)
        return allowed

    def __closestNeighborAS(self):
        '''
        Returns the closest neighbor's AS that in the same superAS.
        '''
        allowedAS = self.__allowedAS()
        best_neighbor, best_rd = spot.find_best_neighbor(self.robot, True, allowedAS)
        if not best_neighbor:
            return None #The leader don't have the best neighbor
        else:
            return best_neighbor.cluster_id

    def __countToTwo(self):
        '''
        Polish equivalent is "Do dwóch odlicz" - every second robot updates it's local AS to be equal AS + 1.
        '''
        previous_AS = self.__closestNeighborAS()
        if not previous_AS: #only leader should not have a previous AS
            if not self.timerSet:
                self.robot.setTimer(20)
                self.timerSet = True
            else:
                self.robot.timer.tick()
                if self.robot.timer.duration < 0:
                    self.upgrade(4, self.robot.super_cluster_id)
            return
        if previous_AS == self.robot.cluster_id:
            if not self.isIncreased:
                self.robot.cluster_id += 100 #creation of the new AS
                self.isIncreased = True
            else:
                self.robot.cluster_id -= 100
                self.isIncreased = False
        else:
            robot = self.robot
            check_me = robot.cluster_id
            red = check_me % 256
            green = math.floor(check_me / 4) % 256
            blue = math.floor(math.sqrt(check_me)) % 256
            color = (red, green, blue)
            pg.draw.circle(robot.image, color, (robot.radius, robot.radius),
                           robot.radius)


    def update(self):
        self.check_phase()
        self.__countToTwo()
        self.robot.velocity.x = 0
        self.robot.velocity.y = 0
        self.robot.broadcast["superAS"] = self.robot.super_cluster_id
        
    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 4:
                    robot.broadcast["superAS"] = self.robot.super_cluster_id
                    self.upgrade(m["Phase"], self.robot.super_cluster_id)
                    return
                
    def upgrade(self, next_phase=4, superAS=None):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        #        elif next_phase == 4:
        #            self.robot.faza = ph4.PhaseFour(self.robot, superAS)
