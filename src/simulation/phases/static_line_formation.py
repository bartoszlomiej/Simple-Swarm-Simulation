import pygame as pg
import math
from utils import SpotNeighbor as spot
from utils.colors import HORRIBLE_YELLOW, PURPLE, BLUE
from simulation.phases.phase import Phase
import simulation.phases.phaseone as ph1
import simulation.phases.phasetwo as ph2
from simulation.robot import RobotState
from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from simulation.robot.agreement.Flooding import Flooding
from simulation.phases.flooding.TimestampFlood import TimestampFlood
from simulation.robot.agreement.ThreeStateAgreement import SYN, SYN_ACK, ACK
from simulation.phases.CountToTwo import CountToTwo
from simulation.phases.StaticLine import StaticLine

import simulation.phases.merge_clusters_to_static_line as mg


class StaticLineFormation(StaticLine):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        self.timestamp_flood = TimestampFlood(self.robot.threeStateAgreement, \
                                              Flooding(superAS, self.robot.received_messages, self.robot.broadcastMessage))
        self.robot.agreement_state = SYN
        self.paintItBlack()


    def paintItBlack(self):
        BLACK = (0, 0, 0)
        pg.draw.circle(self.robot.image, BLACK,
                       (self.robot.radius, self.robot.radius),
                       self.robot.radius)

    def dbg_msg(self):
        print("TUTAJ JESTEM!!!")

    def updateColor(self, new_color):  #just for dbg
        pg.draw.circle(self.robot.image, new_color,
                       (self.robot.radius, self.robot.radius),
                       self.robot.radius)

    def __changeColorIfTimestamp(self, isEdgeRobot):  #just for dbg
        new_color = self.__checkForFlood(isEdgeRobot)
        if new_color > 0.05:
            self.updateColor(BLUE)
            self.upgrade(3.5, self.robot.super_cluster_id)

    def __checkForFlood(self, isEdgeRobot):
        if self.timestamp_flood.repeat():
            #            self.updateColor(HORRIBLE_YELLOW)
            return self.timestamp_flood.getTimeWhenFinished(isEdgeRobot)
        return 0

    def __startFlood(self):
        if self.timerSet and self.__isTimerFinished():
            self.timestamp_flood.spillOver()
            self.timerSet = False
        else:
            self.__setTimer()

    def __isTimerFinished(self):
        self.robot.timer.tick()
        if self.robot.timer.duration <= 0:
            return True
        return False

    def __setTimer(self):
        if not self.timerSet:
            self.timerSet = True
            self.robot.setTimer(100) #1500

    def __insideRobotFlooding(self):
        self.__changeColorIfTimestamp(False)
        self.insideRobotFunctionallity()

    def __edgeRobotFlooding(self):
        self.__startFlood()
        self.__changeColorIfTimestamp(self)
        self.edgeRobotFunctionallity(0.8)

    def getSameClusterMembers(self):
        self.same_cluster_neighbors.clear()
        for n in self.robot.neighbors:
            if n.cluster_id != self.robot.cluster_id:
                continue
            self.same_cluster_neighbors.append(n)
        return self.same_cluster_neighbors

    def __keepStaticLine(self):
        self.same_cluster_neighbors = self.getSameClusterMembers()
        if self._isEdgeRobot():
            self.__edgeRobotFlooding()
        else:
            self.__insideRobotFlooding()

    def update(self):
        self.check_phase()
        self.robot.velocity = Velocity(0, 0)
        self.timestamp_flood.agreement.updateMessages(
            self.robot.received_messages)
        self.__keepStaticLine()
        self.robot.is_allone()

    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] > 3:
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
        elif next_phase == 3:
            self.robot.faza = mg.MergeClustersToStaticLine(self.robot, superAS)
        elif next_phase == 3.5:
            self.robot.faza = CountToTwo(self.robot, superAS)
        #        elif next_phase == 4:
        #            self.robot.faza = ph4.PhaseFour(self.robot, superAS)
