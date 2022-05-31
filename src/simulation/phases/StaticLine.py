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
from simulation.robot.agreement.ThreeStateAgreement import SYN, SYN_ACK, ACK

import simulation.phases.merge_clusters_to_static_line as mg


class StaticLine(Phase):
    def __init__(self, Robot, superAS):
        super().__init__(Robot)
        self.phase = 3
        self.robot.cluster_id = superAS
        self.robot.super_cluster_id = superAS
        self.timerSet = False
        self.robot.velocity = Velocity(0, 0)
        self.robot.state = RobotState.STOPPED
        self.same_cluster_neighbors = []

    def _isEdgeRobot(self):
        iterator = 1
        delta = 20
        for n in self.same_cluster_neighbors:
            for i in range(iterator, len(self.same_cluster_neighbors)):
                if self.checkAngle(
                        n, self.robot,
                        self.same_cluster_neighbors[i]) > (90.0 + delta):
                    return False
            iterator += 1
        return True

    def insideRobotFunctionallity(self):
        closest_neighbor, closest_neighbor_distance = self._findClosestNeighbor(
        )
        opposite_neighbor, opposite_neighbor_distance = self.__findRobotOnOppositeSide(
            closest_neighbor)
        if not opposite_neighbor:
            return
        self.__equalizeDistances(closest_neighbor, opposite_neighbor)
        self.__repeatCommonDirection()

    def __repeatCommonDirection(self):
        self.robot.follower_msg()

    def __equalizeDistances(self, closest_neighbor, opposite_neighbor):
        point_x = (closest_neighbor.position.x +
                   opposite_neighbor.position.x) / 2
        point_y = (closest_neighbor.position.y +
                   opposite_neighbor.position.y) / 2
        delta_x = (point_x - self.robot.position.x)
        delta_y = (point_y - self.robot.position.y)
        if not delta_x or not delta_y:
            self.robot.direction.stop()
            return
        suma = math.sqrt(delta_x**2 + delta_y**2)
        self.robot.direction = Direction(delta_x / suma, delta_y / suma)
        self.__moveIfPathIsFree()

    def __findRobotOnOppositeSide(self, closest_neighbor):
        self.same_cluster_neighbors.remove(closest_neighbor)

        while self.same_cluster_neighbors:
            opposite_neighbor, distance = self._findClosestNeighbor()
            if self.checkAngle(closest_neighbor, self.robot,
                               opposite_neighbor) > 90.0:
                return opposite_neighbor, distance

            self.same_cluster_neighbors.remove(opposite_neighbor)

        self.upgrade(3, self.robot.super_cluster_id)
        return None, 0

    def edgeRobotFunctionallity(self, desired_distance=0.8):
        closest_neighbor, distance_to_neighbor = self._findClosestNeighbor()
        if not closest_neighbor:
            return
        self.__keepDistance(closest_neighbor, distance_to_neighbor,
                            desired_distance)
        self.__broadcastFreeDirection(closest_neighbor)

    def __broadcastFreeDirection(self, closest_neighbor):
        self.__oppositeDirectionTo(closest_neighbor)
        self.robot.broadcastMessage("Direction", self.robot.direction.copy())

    def __oppositeDirectionTo(self, closest_neighbor):
        spot.direction_to_neighbor(self.robot, closest_neighbor)
        self.robot.direction.negate()

    def _findClosestNeighbor(self):
        closest_distance = 10000
        closest_neighbor = None
        distance = None
        for n in self.same_cluster_neighbors:
            distance = spot.relative_distance(self.robot.position.x,
                                              self.robot.position.y,
                                              n.position.x, n.position.y)
            if distance < closest_distance:
                closest_distance = distance
                closest_neighbor = n
        return closest_neighbor, distance

    def __isOptimalDistance(self, distance_to_neighbor, distance_to_keep, epsilon):
        if distance_to_neighbor < distance_to_keep + epsilon and \
           distance_to_neighbor > distance_to_keep - epsilon:
            return True
        return False

    def __isTooSmallDistance(self, distance_to_neighbor, distance_to_keep, epsilon):
        if distance_to_neighbor < distance_to_keep - epsilon:
            return True
        return False

    def __isTooBigDistance(self, distance_to_neighbor, distance_to_keep, epsilon):
        if distance_to_neighbor > distance_to_keep + epsilon:
            return True
        return False
    
    def __keepDistance(self,
                       neighbor,
                       distance_to_neighbor,
                       desired_distance=0.8):
        distance_to_keep = desired_distance * (self.robot.sensor_range - self.robot.radius) + self.robot.radius
        spot.direction_to_neighbor(self.robot, neighbor)
        if self.__isOptimalDistance(distance_to_neighbor, distance_to_keep, 5):
            return
        elif self.__isTooSmallDistance(distance_to_neighbor, distance_to_keep, 5):
            self.robot.direction.negate()
            self.__moveIfPathIsFree()
        elif self.__isTooBigDistance(distance_to_neighbor, distance_to_keep, 5):
            self.__moveIfPathIsFree()


    '''
    def __keepDistance(self,
                       neighbor,
                       distance_to_neighbor,
                       desired_distance=0.8):
        if distance_to_neighbor < (
                desired_distance *
            (self.robot.sensor_range - self.robot.radius)) + self.robot.radius:
            spot.direction_to_neighbor(self.robot, neighbor)
            self.robot.direction.negate()
            self.__moveIfPathIsFree()
    '''
    def __moveIfPathIsFree(self):
        if not spot.is_any_collision(self.robot, 0.2):
            self.robot.direction.normalize()
            self.makeMove()

    def update(self):
        self.check_phase()
        self.robot.velocity = Velocity(0, 0)

        if self._isEdgeRobot():
            self.edgeRobotFunctionallity()
        else:

            self.insideRobotFunctionallity()
        self.robot.broadcast["superAS"] = self.robot.super_cluster_id
        self.robot.is_allone()

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
        elif next_phase == 3:
            self.robot.faza = mg.MergeClustersToStaticLine(self.robot, superAS)
        #        elif next_phase == 4:
        #            self.robot.faza = ph4.PhaseFour(self.robot, superAS)
