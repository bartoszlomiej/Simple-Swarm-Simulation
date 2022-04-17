import sys
import numpy as np
import pygame as pg
import math
import Simulation as sim
import SpotNeighbor as spot
import phase as ph
import phaseone as ph1
import phasetwo as ph2


class PhaseThree(ph.Phase):
    def __init__(self, Robot):
        super().__init__(Robot)
        self.phase = 3
        self.superAS = Robot.AS

    def __closestNeighborAS(self):
        '''
        Returns the closest neighbor's AS that in the same superAS.
        '''
        best_neighbor, best_rd = spot.find_best_neighbor(self.robot, False)
        if not best_neighbor:
            return None #The leader don't have the best neighbor
        if best_neighbor.AS == self.superAS:
            self.robot.AS += 1
        else:
            self.robot.AS = self.superAS
            print(self.robot.dir_x,
                  self.robot.dir_y)  #direction should not be zero
            return None

    def __countToTwo(self):
        '''
        Polish equivalent is "Do dwóch odlicz" - every second robot updates it's local AS to be equal AS + 1.
        '''
        previous_AS = self.__closestNeighborAS()
        if not previous_AS:
            return  #I have know idea yet what should be done here:(
        if previous_AS == self.superAS:
            self.robot.AS += 1
            #just for dbg
            HORRIBLE_YELLOW = (190, 175, 50)
            robot = self.robot
            pg.draw.circle(robot.image, HORRIBLE_YELLOW,
                           (robot.radius, robot.radius), robot.radius)
        elif previous_AS != self.superAS + 1:  #this robot is not in my super cluster
            return

    def doubleRow(self):
        '''
        Changes single line formation to double row.
        '''

        pass

    def update(self):
        HORRIBLE_YELLOW = (190, 175, 50)
        robot = self.robot
        pg.draw.circle(robot.image, HORRIBLE_YELLOW,
                       (robot.radius, robot.radius), robot.radius)
        self.__countToTwo()
        self.robot.velocity[0] = 0
        self.robot.velocity[1] = 0
        
    def check_phase(self):
        robot = self.robot
        for m in robot.messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 3:
                    self.AS = m["AS"]
                    self.upgrade(m["Phase"])
                    return
                
    def upgrade(self, next_phase):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
