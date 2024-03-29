from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from simulation.phases.StaticLine import StaticLine
from utils import SpotNeighbor as spot

import pygame as pg #dbg

class Shape(StaticLine):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        self.phase = 5
        self.isIncreased = False
        #        self.robot.direction = Direction(1, 1)

    def paintItBlack(self, color = (0, 0, 0), robot=None):
        if not robot:
            robot = self.robot
        BLACK = color
        pg.draw.circle(robot.image, BLACK,
                       (robot.radius, robot.radius),
                       robot.radius)        

    def __getSuperclusterMembers(self):
        cluster_members = []
        for n in self.robot.neighbors:
            if n.super_cluster_id == self.robot.super_cluster_id:
                cluster_members.append(n)
        return cluster_members

    def __getSuperclusterMembersID(self):
        allowed = []
        allowed.append(self.robot.cluster_id)
        for n in self.robot.neighbors:
            if n.super_cluster_id == self.robot.super_cluster_id:
                if not n.cluster_id in allowed:
                    allowed.append(n.cluster_id)
        return allowed

    def _getSameClusterMembers(self):
        self.same_cluster_neighbors.clear()
        for n in self.robot.neighbors:
            if n.cluster_id != self.robot.cluster_id:
                continue
            self.same_cluster_neighbors.append(n)
        return self.same_cluster_neighbors


    def __isHigherClusterID(self, closest_neighbor):
        if closest_neighbor.cluster_id < self.robot.cluster_id:
            return True
        return False
            
    def __goCloserToPreviousNeighbor(self):
        spot.follower(self.robot)

    def __moveIfPathIsFree(self):
        if not spot.is_any_collision(self.robot, 0.15):
            self.robot.direction.normalize()
            self.makeMove(True)

    def __getOtherSuperclusterNeighborsClustersID(self):
        neighbors_cluster_id = []
        for n in self.robot.neighbors:
            if not n in self.same_cluster_neighbors:
                neighbors_cluster_id.append(n.cluster_id)
        return neighbors_cluster_id

    def _keepDistanceInsideSuperAS(self, max_distance=0.8, min_distance=0.3):
        neighbors_cluster_id = self.__getOtherSuperclusterNeighborsClustersID()
        closest_neighbor, rd = spot.find_best_neighbor(
            self.robot, True, neighbors_cluster_id)
        if not closest_neighbor:
            return
        self._keepDistance(closest_neighbor, rd, max_distance, min_distance)

    def _keepDistance(self,
                       neighbor,
                       distance_to_neighbor,
                       max_distance = 0.8,
                       min_distance = 0.3,
                       epsilon = 5):
        distance_to_keep = max_distance * (self.robot.sensor_range - self.robot.radius) + self.robot.radius
        min_distance_to_keep =  min_distance * (self.robot.sensor_range - self.robot.radius) + self.robot.radius
        spot.direction_to_neighbor(self.robot, neighbor)
        if self.__isTooBigDistance(distance_to_neighbor, distance_to_keep, epsilon):
            #self.paintItBlack()
            self.__moveIfPathIsFree()
        elif self.__isTooSmallDistance(distance_to_neighbor, min_distance_to_keep, epsilon):
            self.robot.direction.negate()
            self.__moveIfPathIsFree()

    def __isTooSmallDistance(self, distance_to_neighbor, distance_to_keep, epsilon):
        if distance_to_neighbor < distance_to_keep - epsilon:
            return True
        return False            
            
    def __isTooBigDistance(self, distance_to_neighbor, distance_to_keep, epsilon):
        if distance_to_neighbor > distance_to_keep + epsilon:
            return True
        return False            

    def _keepStaticLine(self):
        self.same_cluster_neighbors.clear()
        self.same_cluster_neighbors = self._getSameClusterMembers()
        if self._isEdgeRobot():
            self.edgeRobotFunctionallity(0.4)
            self._keepDistanceInsideSuperAS()
        else:
            self.insideRobotFunctionallity()

    def update(self):
        self.check_phase()
        self.robot.velocity = Velocity(0, 0)
        self.__checkForStepForward()
        self.__keepStaticLine()

        self.robot.isAlloneInSupercluster()

    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 4:
                    robot.broadcast["superAS"] = self.robot.super_cluster_id
                    self.upgrade(m["Phase"], self.robot.super_cluster_id)
                    return

    def upgrade(self, next_phase=4, superAS=None):
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        #        elif next_phase == 4:
        #            self.robot.faza = ph4.PhaseFour(self.robot, superAS)
