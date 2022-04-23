import sys
import numpy as np
import pygame as pg
import math
import Simulation as sim
import SpotNeighbor as spot
import phase as ph
import phaseone as ph1
import phasetwo as ph2
import phasethree as ph3


class PhaseFour(ph.Phase):
    def __init__(self, Robot, superAS):
        super().__init__(Robot)
        self.phase = 4
        self.robot.superAS = superAS

    def __checkPrivelage(self, neighborAS):
        '''
        returns True if this AS is higher than the neighbour in front of the given robot with regard to the direction of movement
        '''
        if neighborAS < self.robot.AS:
            return True
        return False

    def __findDirection(self):
        '''
        Returns the direction to go so as to avoid a robot of lower priority.
        If in front of us there is a robot of the same priority then stop!
        '''
        pass

    def __higherPriority(self):
        '''
        If check right is free, then go!

        TODO
        '''
        pass

    def __lowerPriority(self):
        '''
        If there is no robot of higher AS in the given direction then go!
        '''
        pass

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
        allnowedAS = self.__allowedAS()
        best_neighbor, best_rd = spot.find_best_neighbor(self.robot, True, allowedAS)
        if not best_neighbor:
            return None #The leader don't have the best neighbor
        else:
            return best_neighbor.AS

    def just_dbg(self):
        robot = self.robot
        color = (130, 21, 169)
        pg.draw.circle(robot.image, color, (robot.radius, robot.radius),
                           robot.radius)

    def update(self):
        self.just_dbg()
        self.robot.velocity[0] = 0
        self.robot.velocity[1] = 0
        self.robot.broadcast["superAS"] = self.robot.superAS
        
    def check_phase(self):
        robot = self.robot
        for m in robot.messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 4:
                    self.AS = m["AS"]
                    self.upgrade(m["Phase"])
                    return
                
    def upgrade(self, next_phase=5):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 3:
            self.robot.faza = ph2.PhaseFour(self.robot)
