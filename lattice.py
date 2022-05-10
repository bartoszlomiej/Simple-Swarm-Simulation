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
import phasefive as ph5


class Lattice(ph.Phase):
    def __init__(self, Robot, superAS):
        super().__init__(Robot)
        self.phase = 4
        self.robot.superAS = superAS
        self.dir_x = self.robot.dir_x  #just for dbg
        self.dir_y = self.robot.dir_y  #just for dbg

    def __touchNeighbor(self):
        '''
        If new AS is comming to the existing one it stops relatively far from it.

        ***INITIALLY IT WILL BE TESTED ONLY ON THE AS.id != superAS.id***
        '''
        if self.robot.AS == self.robot.superAS:
            return
        self.robot.follower_msg()
        neighbor = self.__closestNeighbor()
        if not neighbor:  #leader doesn't have neighbors
            return
        self.__higherPriority(neighbor)

    def __checkPriority(self):
        '''
        returns True if this AS is higher than the neighbour in front of the given robot with regard to the direction of movement
        '''
        self.robot.follower_msg()
        neighbor = self.__closestNeighbor()
        if not neighbor:  #leader doesn't have neighbors
            return
        if neighbor.AS < self.robot.AS:
            self.__higherPriority(neighbor)
        else:
            self.__lowerPriority()
        self.robot.velocity[0] = self.robot.dir_x  # * 0.5
        self.robot.velocity[1] = self.robot.dir_y  # * 0.5

    def __perpendicularDirection(self, neighbor):
        '''
        Returns the direction the robot would like to follow

        Ax + By = 0 - initial direction from the direction_to_neighbor
        Bx - Ay = 0 - perpendicular to the above line
        '''
        vector = math.ceil(self.robot.k / 4)
        delta_x = (neighbor.x - self.robot.x
                   )  #it must be greater than 0 - robots cannot overlap
        delta_y = (neighbor.y - self.robot.y)

        suma = math.sqrt(delta_x**2 + delta_y**2)

        self.robot.dir_x = delta_y / suma
        self.robot.dir_y = -delta_x / suma
        #        return dir_x, dir_y  #collision avoidance must be implemented!!!

    def __higherPriority(self, neighbor):
        '''
        If check right is free, then go!
        '''
        if spot.is_follower(self.robot):
            spot.follower(self.robot)
            if self.robot.dir_x == 0 or self.robot.dir_y == 0:
                self.__perpendicularDirection(neighbor)
        else:
            self.__perpendicularDirection(neighbor)

    def __lowerPriority(self):
        '''
        If there is no robot of higher AS in the given direction then go!
        '''
        spot.follower(self.robot)
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

    def __closestNeighbor(self):
        '''
        Returns the closest neighbor's AS that is in the same superAS.
        '''
        allowedAS = self.__allowedAS()
        best_neighbor, best_rd = spot.find_best_neighbor(
            self.robot, True, allowedAS)
        if not best_neighbor:
            BLACK = (0, 0, 0)
            robot = self.robot
            pg.draw.circle(robot.image, BLACK, (robot.radius, robot.radius),
                           robot.radius)
            return None  #The leader don't have the best neighbor
        else:
            return best_neighbor

    def __cuddling(self):
        '''
        The group which joins superAS should go as close to AP and other group as possible
        '''
        pass

    def __minimal_distance(self):
        '''
        Checks if the minimal distance between robots is being kept.
        Returns true if minimal distance is being kept; otherwise returns false.
        '''
        robot = self.robot
        for n in robot.neighbors:
            if (abs(n.position[0] - robot.position[0]) <= robot.radius) and (
                    abs(n.position[1] - robot.position[1]) <= robot.radius):
                robot.dir_x, robot.dir_y = 0, 0
                return
        return

    def just_dbg(self):
        robot = self.robot
        color = (130, 21, 169)
        pg.draw.circle(robot.image, color, (robot.radius, robot.radius),
                       robot.radius)

    def update(self):
        #        self.__touchNeighbor()
        self.check_phase()
        self.__closestNeighbor()
        self.__minimal_distance()
        self.robot.velocity[0] = 0
        self.robot.velocity[1] = 0
        
        self.robot.broadcast["Direction"] = (self.dir_x, self.dir_y)
        self.robot.broadcast["superAS"] = self.robot.superAS

    def check_phase(self):
        robot = self.robot
        for m in robot.messages:
            if "Phase" in m.keys():
                if m["Phase"] > 4:
                    self.AS = m["AS"]
                    self.upgrade(m["Phase"])
                    return

    def upgrade(self, next_phase=5, superAS=None):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 3:
            self.robot.faza = ph3.PhaseThree(self.robot)
        elif next_phase == 5:
            self.robot.faza = ph5.PhaseFive(self.robot)
