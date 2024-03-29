import pygame as pg
import math
from utils import SpotNeighbor as spot
from simulation.phases.phase import Phase
import simulation.phases.phaseone as ph1
import simulation.phases.phasetwo as ph2
import simulation.phases.phasethree as ph3
import simulation.phases.phasefive as ph5


class Lattice(Phase):
    def __init__(self, Robot, superAS):
        super().__init__(Robot)
        self.phase = 4
        self.robot.super_cluster_id = superAS
        self.dir_x = self.robot.dir_x  #just for dbg
        self.dir_y = self.robot.dir_y  #just for dbg

    def __checkPriority(self):
        '''
        returns True if this AS is higher than the neighbour in front of the given robot with regard to the direction of movement
        '''
        self.robot.follower_msg()
        neighbor = self.__closestNeighbor()
        if not neighbor:  #leader doesn't have neighbors
            return
        if neighbor.AS < self.robot.cluster_id:
            self.__higherPriority(neighbor)
        else:
            self.__lowerPriority()
        self.robot.velocity.x = self.robot.dir_x  # * 0.5
        self.robot.velocity.y = self.robot.dir_y  # * 0.5

    def __perpendicularDirection(self, neighbor):
        '''
        Returns the direction the robot would like to follow

        Ax + By = 0 - initial direction from the direction_to_neighbor
        Bx - Ay = 0 - perpendicular to the above line
        '''
        vector = math.ceil(self.robot.sensors_number / 4)
        delta_x = (neighbor.position.x - self.robot.position.x
                   )  #it must be greater than 0 - robots cannot overlap
        delta_y = (neighbor.position.y - self.robot.position.y)

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
        allowed.append(self.robot.cluster_id)
        for n in self.robot.neighbors:
            if n.super_cluster_id == self.robot.super_cluster_id:
                if not n.cluster_id in allowed:
                    allowed.append(n.cluster_id)
        return allowed

    def __closestNeighbor(self):
        '''
        Returns the closest neighbor's AS that is in the same superAS.
        '''
        allowedAS = self.__allowedAS()
        best_neighbor, best_rd = spot.find_best_neighbor(
            self.robot, True, allowedAS)
        if not best_neighbor:
            return None  #The leader don't have the best neighbor
        else:
            return best_neighbor

    def __minimal_distance(self):
        '''
        Checks if the minimal distance between robots is being kept.
        Returns true if minimal distance is being kept; otherwise returns false.
        '''
        robot = self.robot
        for n in robot.neighbors:
            if (abs(n.position.x - robot.position.x) <= robot.radius) and (
                    abs(n.position.y - robot.position.y) <= robot.radius):
                robot.dir_x, robot.dir_y = 0, 0
                return
        return

    def just_dbg(self):
        robot = self.robot
        color = (130, 21, 169)
        pg.draw.circle(robot.image, color, (robot.radius, robot.radius),
                       robot.radius)

    def update(self):
        self.__checkPriority()
        self.__minimal_distance()
        self.robot.broadcast["Direction"] = (self.dir_x, self.dir_y)
        self.robot.broadcast["superAS"] = self.robot.super_cluster_id

    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
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
            self.robot.faza = ph3.PhaseThree(self.robot)
        elif next_phase == 5:
            self.robot.faza = ph5.PhaseFive(self.robot)
