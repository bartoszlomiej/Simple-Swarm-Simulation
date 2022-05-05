import sys
import numpy as np
import pygame as pg
import math
import Simulation as sim
import SpotNeighbor as spot
import phase as ph
import phaseone as ph1
import phasetwo as ph2
import phasefour as ph4


class PhaseThree(ph.Phase):
    def __init__(self, Robot, superAS):
        super().__init__(Robot)
        self.phase = 3
        self.robot.superAS = superAS
        self.isIncreased = False
        self.timerSet = False
        self.robot.velocity = [0, 0]

    def __allowedAS(self):
        '''
        Returns the AS's that are in the same superAS that can be spot by the given robot.
        '''
        allowed = []
        allowed.append(self.robot.AS)
        for n in self.robot.neighbors:
            if n.superAS == self.robot.superAS:
                if not n.AS in allowed:
                    allowed.append(n.AS)
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
            return best_neighbor.AS

    def __countToTwo(self):
        '''
        Polish equivalent is "Do dwóch odlicz" - every second robot updates it's local AS to be equal AS + 1.
        '''
        previous_AS = self.__closestNeighborAS()
        if not previous_AS: #only leader should not have a previous AS
            if not self.timerSet:
                self.robot.set_timer(20, False)
                self.timerSet = True
            else:
                self.robot.timer = (self.robot.timer[0], self.robot.timer[1] - 1, self.robot.timer[2])
                if self.robot.timer[1] < 0:
                    self.upgrade(4, self.robot.superAS)
            return
        if previous_AS == self.robot.AS:
            if not self.isIncreased:
                self.robot.AS += 100 #creation of the new AS
                self.isIncreased = True
            else:
                self.robot.AS -= 100
                self.isIncreased = False
        else:
            robot = self.robot
            check_me = robot.AS
            red = check_me % 256
            green = math.floor(check_me / 4) % 256
            blue = math.floor(math.sqrt(check_me)) % 256
            color = (red, green, blue)
            pg.draw.circle(robot.image, color, (robot.radius, robot.radius),
                           robot.radius)


    def update(self):
        self.check_phase()
        self.__countToTwo()
        self.robot.velocity[0] = 0
        self.robot.velocity[1] = 0
        self.robot.broadcast["superAS"] = self.robot.superAS
        
    def check_phase(self):
        robot = self.robot
        for m in robot.messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 4:
                    robot.broadcast["superAS"] = self.robot.superAS
                    self.upgrade(m["Phase"], self.robot.superAS)
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
