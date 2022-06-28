from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from simulation.phases.StaticLine import StaticLine
from simulation.phases.shapes.V_Shape import V_Shape
#from simulation.phases.shapes.P_Shape import P_Shape
#from simulation.phases.shapes.P_Shape_v2 import P_Shape_v2
from utils import SpotNeighbor as spot
from copy import deepcopy

import pygame as pg #dbg

class StepForward(StaticLine):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        self.phase = 4
        self.isIncreased = False
        self.robot.direction = Direction(1, 1)
        self.robot.cluster_id = superAS
        self.robot.divisions += 1

    def paintItBlack(self):
        BLACK = (0, 0, 0)
        pg.draw.circle(self.robot.image, BLACK,
                       (self.robot.radius, self.robot.radius),
                       self.robot.radius)        

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

    def __getSameClusterMembers(self):
        self.same_cluster_neighbors.clear()
        for n in self.robot.neighbors:
            if n.cluster_id != self.robot.cluster_id:
                continue
            self.same_cluster_neighbors.append(n)
        return self.same_cluster_neighbors

    def __checkForStepForward(self):
        self.robot.direction = Direction(1, 1)
        #self.robot.follower_msg()  #there is a need of direction --this might cause some problems
        closest_neighbor, rd = self.__getNeighborInDirection()
        '''
        closest_neighbor, rd = spot.find_best_neighbor(
            self.robot, False, self.__getSuperclusterMembersID())
        '''
        if not closest_neighbor: #this is the leader
            self.__leaderSpecialCase()
            return
        self.__stepForwardIfHigherPriority(closest_neighbor)

    def __leaderSpecialCase(self):
        self.robot.direction.negate()
        closest_neighbor, rd = spot.find_best_neighbor(
            self.robot, False, self.__getSuperclusterMembersID())
        self.__stepForwardIfHigherPriority(closest_neighbor, False)

    def __getNeighborInDirection(self):
        closest_neighbor, rd = spot.find_best_neighbor(
            self.robot, False, self.__getSuperclusterMembersID())
        if not closest_neighbor:
            return None, None
        if spot.isNeighborInDirection(self.robot, closest_neighbor):
            return closest_neighbor, rd
        else:
            self.robot.neighbors.remove(closest_neighbor)
            return self.__getNeighborInDirection()
        
    def __stepForwardIfHigherPriority(self, closest_neighbor, clockwise=True):
        if self.__isHigherClusterID(closest_neighbor):
            self.__tryPerpendicularMotion(closest_neighbor, clockwise)
            self.__moveIfPathIsFree()
        '''
        else:
            self.__goCloserToPreviousNeighbor()
        self.__moveIfPathIsFree()
        '''

    def __isHigherClusterID(self, closest_neighbor):
        if closest_neighbor.cluster_id < self.robot.cluster_id:
            return True
        return False
    
    def __tryPerpendicularMotion(self, closest_neighbor = None, clockwise=True):
        self.same_cluster_neighbors = self.__getSuperclusterMembers()
        if not self.same_cluster_neighbors:
            return
        if not closest_neighbor:
            best_neighbor, distance = self._findClosestNeighbor()
        else:
            best_neighbor = closest_neighbor
        spot.direction_to_neighbor(self.robot, best_neighbor)
        if clockwise:
            self.robot.direction.perpendicular()
        else:
            self.robot.direction.rightRotation()
            
    def __goCloserToPreviousNeighbor(self):
        spot.follower(self.robot)

    def __moveIfPathIsFree(self):
        if not spot.is_any_collision(self.robot, 0.15):
            self.robot.direction.normalize()
            self.makeMove(True)

    def __isInSameCluster(self, previous_robot_cluster_id):
        if previous_robot_cluster_id == self.robot.cluster_id:
            return True
        return False

    def __changeClusterIfSame(self, previous_robot_cluster_id):
        if self.__isInSameCluster(previous_robot_cluster_id):
            self.__updateClusterID()
        else:
            self.robot.update_color()  #just for dbg

    def __getOtherSuperclusterNeighborsClustersID(self):
        neighbors_cluster_id = []
        for n in self.robot.neighbors:
            if not n in self.same_cluster_neighbors:
                neighbors_cluster_id.append(n.cluster_id)
        return neighbors_cluster_id

    def __keepDistanceInsideSuperAS(self):
        neighbors_cluster_id = self.__getOtherSuperclusterNeighborsClustersID()
        closest_neighbor, rd = spot.find_best_neighbor(
            self.robot, True, neighbors_cluster_id)
        if not closest_neighbor:
            return
        self.__keepDistance(closest_neighbor, rd)

    def __keepDistance(self,
                       neighbor,
                       distance_to_neighbor,
                       max_distance = 0.8,
                       min_distance = 0.3,
                       epsilon = 5):
        distance_to_keep = max_distance * (self.robot.sensor_range - self.robot.radius) + self.robot.radius
        min_distance_to_keep =  min_distance * (self.robot.sensor_range - self.robot.radius) + self.robot.radius
        spot.direction_to_neighbor(self.robot, neighbor)
        if self.__isTooBigDistance(distance_to_neighbor, distance_to_keep, epsilon):
            self.paintItBlack()
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

    def __keepStaticLine(self):
        self.same_cluster_neighbors.clear()
        self.same_cluster_neighbors = self.__getSameClusterMembers()
        if self._isEdgeRobot():
            self.edgeRobotFunctionallity(0.4)
            self.__useTimerIfSet()
            self.__keepDistanceInsideSuperAS()
        else:
            self.insideRobotFunctionallity()

    def __setTimer(self):
        self.robot.setTimer(500) 
        self.timerSet = True

    def __upgradeIfTimerFinished(self):
        if self.robot.timer.duration < 0:
            self.upgrade(5, self.robot.super_cluster_id)
            pass

    def __useTimerIfSet(self):
        if not self.timerSet:
            self.__setTimer()
        else:
            self.robot.timer.tick()
            self.__upgradeIfTimerFinished()
            
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
                if m["Phase"] >= 5:
                    robot.broadcast["superAS"] = self.robot.super_cluster_id
                    self.upgrade(m["Phase"], self.robot.super_cluster_id)
                    return

    def upgrade(self, next_phase=4, superAS=None):
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 5:
            self.robot.faza = V_Shape(self.robot, superAS)
            #            self.robot.faza = P_Shape(self.robot, superAS)
            #            self.robot.faza = P_Shape_v2(self.robot, superAS)
