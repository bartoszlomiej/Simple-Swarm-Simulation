from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from simulation.phases.StaticLine import StaticLine
import simulation.phases.CountToTwo as ct
from simulation.robot.agreement.Downgrade import Downgrade
from simulation.robot.agreement.ThreeStateAgreement import SYN, SYN_ACK, ACK
import pygame as pg #dbg
from utils import SpotNeighbor as spot
import math

class W_Shape_Proxy(StaticLine):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        #        self.perpendicular_direction = perpendicular_direction
        self.phase = 5.5
        self.timerSet = False
        self.robot.agreement_state = SYN

    def paintItBlack(self):
        BLACK = (0, 0, 0)
        pg.draw.circle(self.robot.image, BLACK,
                       (self.robot.radius, self.robot.radius),
                       self.robot.radius)                

    def downgrade(self):
        self.robot.broadcast["Downgrade"] = 3.5
        '''
        self.robot.direction = self.robot.find_direction()
        self.robot.broadcast["Direction"] = self.robot.direction.copy()
        '''

    def checkForDowngrade(self):
        downgrade = Downgrade(self.robot.cluster_id,
                              self.robot.received_messages,
                              self.robot.broadcastMessage,
                              self.robot.repeatDirection)
        if self.robot.threeStateAgreement(downgrade):
            self.robot.is_downgrade = True
            if downgrade.state == ACK:
                self.robot.is_downgrade = False
                self.robot.communicationFinished()
                self.upgrade(3.5)
        else:
            self.__useTimerIfSet()

    def __setTimer(self):
        #        self.robot.setTimer(200)
        self.robot.setRandomTimer(100, 800)
        self.timerSet = True

    def __upgradeIfTimerFinished(self):
        if self.robot.timer.duration < 0: #here there should be 3 state agreement
            self.downgrade()

    def __useTimerIfSet(self):
        if not self.timerSet:
            self.__setTimer()
        else:
            self.robot.timer.tick()
            self.__upgradeIfTimerFinished()

    def _keepStaticLine(self):
        if self._isEdgeRobot():
            self.edgeRobotFunctionallity()
        else:
            self.insideRobotFunctionallity()
        
    def update(self):
        self.check_phase()
        self.robot.direction = Direction(1, 1)
        self.robot.velocity = Velocity(0, 0)
        self._keepStaticLine()
        self.robot.isAlloneInSupercluster()
        self.checkForDowngrade()
        
    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 6 or m["Phase"] == 3.5:
                    robot.broadcast["superAS"] = self.robot.super_cluster_id
                    self.upgrade(m["Phase"], self.robot.super_cluster_id)
                    return
        #        self.robot.update_color()  #just for dbg                
                
    def upgrade(self, next_phase=4, superAS=None):
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 3.5:
            self.robot.super_super_cluster_id = self.robot.super_cluster_id
            if self.robot.cluster_id != self.robot.super_cluster_id:
                self.robot.cluster_id += 2000
            if self.robot.divisions < 1:
                self.robot.faza = ct.CountToTwo(self.robot, self.robot.cluster_id) #self.robot.cluster_id as we create the new supercluster
