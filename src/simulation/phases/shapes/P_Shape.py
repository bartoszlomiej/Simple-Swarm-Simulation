from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from simulation.phases.shapes.Shape import Shape
from utils import SpotNeighbor as spot
import math

import pygame as pg #dbg

ANT = "Higher Priority"

class P_Shape(Shape):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        self.perpendicular_direction = None
        self.direction_to_neighbor = None
        self.robot.direction = Direction(1, 1)
        self.higher_priority = None

    def __executeInsidePriorities(self):
        if self.higher_priority:
            self.__makeArch()
            self.__keepAngleBetweenNeighbors()
        else:
            self.__stay()
            self.insideRobotFunctionallity()

    def __executeEdgePriorities(self):
        if self.higher_priority:
            if self.__checkIfAngleIsProper():
                self.robot.direction = self.direction_to_neighbor.copy()
                self.__moveIfPathIsFree()            
            self._keepDistanceInsideSuperAS(0.5, 0.4)
        else:
            self.__stay()

    def __findClosestOtherClusterNeighbor(self):
        other_neighbor = self.__getOtherSuperclusterNeighbors()
        closest_distance = 10000
        closest_neighbor = None
        distance = None
        for n in other_neighbor:
            distance = spot.relative_distance(self.robot.position.x,
                                              self.robot.position.y,
                                              n.position.x, n.position.y)
            if distance < closest_distance:
                closest_distance = distance
                closest_neighbor = n
        return closest_neighbor, closest_distance
            
    def __checkIfAngleIsProper(self, angle=140):
        closest_neighbor, closest_distance = self._findClosestNeighbor()
        other_cluster_neighbor, other_distance = self.__findClosestOtherClusterNeighbor()
        if not other_cluster_neighbor:
            return False
        if self.checkAngle(closest_neighbor, self.robot, other_cluster_neighbor) < angle:
            return True
        return False
            
    def __isPrioritySet(self):
        if self.higher_priority == None:
            self.__checkIfHigherPriority()
            return False
        return True
        
    def __makeArch(self):
        self.__insideRobotArchCreation()

    def __stay(self):
        pass

    def __checkIfHigherPriority(self):
        other_neighbor = self.__getOtherSuperclusterNeighbors()
        if not other_neighbor:
            self.__checkClusterPriorityInMessages()
        if other_neighbor[-1].cluster_id < self.robot.cluster_id:
            self.higher_priority = True
            self.robot.broadcastMessage(ANT, True)
        else:
            self.higher_priority = False
            self.robot.broadcastMessage(ANT, False)

    def __checkClusterPriorityInMessages(self):
        for m in self.robot.received_messages:
            if ANT in m.keys():
                self.higher_priority = m[ANT]
                return

    def __setTimer(self):
        self.robot.setTimer(500) 
        self.timerSet = True

    def __upgradeIfTimerFinished(self):
        if self.robot.timer.duration < 0:
            self.upgrade(6, self.robot.super_cluster_id)

    def __useTimerIfSet(self):
        if not self.timerSet:
            self.__setTimer()
        else:
            self.robot.timer.tick()
            self.__upgradeIfTimerFinished()        

    def __isCornerEdgeRobot(self):
        self.robot.direction = Direction(1, 1)
        neighbor, rd = spot.find_best_neighbor(self.robot, True)
        if not neighbor:
            return True
        return False

    def __cornerEdgeRobotFunctionallity(self):
        self._keepDistanceInsideSuperAS(0.25, 0.2)
        self.__useTimerIfSet()

    def __saveDirectionsIfEmpty(self):
        if not self.perpendicular_direction:
            self.__saveDirectionToNeighbor()
            self.__savePerpendicularDirection()

    def __nonCornerEdgeRobotFunctionallity(self):
        self.__saveDirectionsIfEmpty()

        
        if self.__isPrioritySet():
            self.__executeEdgePriorities()        
        #self.__moveUntilAngleIsObtained()

    def __saveDirectionToNeighbor(self):
        self.robot.direction = Direction(1, 1)
        spot.follower(self.robot)
        self.direction_to_neighbor = self.robot.direction.copy()

    def __savePerpendicularDirection(self):
        other_neighbor = self.__getOtherSuperclusterNeighbors()
        if not other_neighbor:
            return
        other_neighbor = other_neighbor[-1]
        self.perpendicular_direction = self.direction_to_neighbor.copy()
        self.perpendicular_direction.leftRotation()
        if self.__checkAngleFromDirectionToNeighbor(other_neighbor) < 0:
            self.perpendicular_direction.rightRotation()
            self.perpendicular_direction.rightRotation()

    def __checkAngleFromDirectionToNeighbor(self, n):
        current_direction = self.robot.direction.copy()
        spot.direction_to_neighbor(self.robot, n)
        angle = math.atan2(self.robot.direction.y,
                           self.robot.direction.x) - math.atan2(
                               self.direction_to_neighbor.y,
                               self.direction_to_neighbor.x)
        self.robot.direction = current_direction
        return math.degrees(angle)
    
    def __getOtherSuperclusterNeighbors(self):
        other_neighbors = []
        for n in self.robot.neighbors:
            if not n in self.same_cluster_neighbors:
                other_neighbors.append(n)
        return other_neighbors

    def __moveUntilAngleIsObtained(self):
        desired_angle = 100
        obtained_angle = self.__evaluateAngle()
        if obtained_angle < desired_angle:
            self.robot.direction = self.perpendicular_direction.copy()
            self.__moveIfPathIsFree()

    def __evaluateAngle(self):
        if self.direction_to_neighbor:
            self.robot.direction = self.direction_to_neighbor.copy()
        else:
            self.robot.direction = Direction(1, 1)
        spot.follower(self.robot)
        self.direction_to_neighbor = self.robot.direction.copy()
        
        angle = math.atan2(self.direction_to_neighbor.y, self.direction_to_neighbor.x) \
            - math.atan2(self.perpendicular_direction.y, self.perpendicular_direction.x)
        return math.degrees(abs(angle))

    def __keepAngleBetweenNeighbors(self):
        closest_neighbor, closest_neighbor_distance = self._findClosestNeighbor()
        opposite_neighbor, opposite_neighbor_distance = self._findRobotOnOppositeSide(
            closest_neighbor)
        angle = self.checkAngle(closest_neighbor, self.robot, opposite_neighbor)
        if angle < 160:
            if self.robot.velocity_level >= 1:
                self.robot.velocity_level /= 4
            print(angle)
            self._equalizeDistances(closest_neighbor, opposite_neighbor)
        
    def __insideRobotArchCreation(self):
        self.__saveDirectionsIfEmpty()
        self.__saveDirectionToNeighbor()
        self.robot.direction = self.direction_to_neighbor.copy()
        self.robot.direction.negate()
        if spot.is_any_collision(self.robot, 0.3):
            self.robot.direction = self.perpendicular_direction.copy()
            self.__moveIfPathIsFree()
        self.robot.direction = self.direction_to_neighbor.copy()

    def __moveIfPathIsFree(self):
        if not spot.is_any_collision(self.robot, 0.15):
            self.robot.direction.normalize()
            self.makeMove(True)

    def _keepStaticLine(self):
        self.same_cluster_neighbors.clear()
        self.same_cluster_neighbors = self._getSameClusterMembers()
        if self._isEdgeRobot():
            if self.__isCornerEdgeRobot():
                self.__cornerEdgeRobotFunctionallity()
            else:
                self.__nonCornerEdgeRobotFunctionallity()            
        else:
            if self.__isPrioritySet():
                self.__executeInsidePriorities()


    def update(self):
        self.robot.update_color()
        self.check_phase()
        self.robot.velocity = Velocity(0, 0)
        self._keepStaticLine()
        self.robot.isAlloneInSupercluster()            

    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 6:
                    robot.broadcast["superAS"] = self.robot.super_cluster_id
                    self.upgrade(m["Phase"], self.robot.super_cluster_id)
                    return

    def upgrade(self, next_phase=4, superAS=None):
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
