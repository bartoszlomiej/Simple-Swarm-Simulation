from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from simulation.phases.shapes.Shape import Shape
from utils import SpotNeighbor as spot
import math

import pygame as pg #dbg

ANT = "Higher Priority"
EQUALIZE = "Equalize Distances"

class P_Shape(Shape):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        self.perpendicular_direction = None
        self.direction_to_neighbor = None
        self.higher_priority = None

    def __makePFormation(self):
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

    def __isCornerEdgeRobot(self):
        self.robot.direction = Direction(1, 1)
        neighbor, rd = spot.find_best_neighbor(self.robot, True)
        if not neighbor:
            return True
        return False

    def __cornerEdgeRobotFunctionallity(self):
        self._keepDistanceInsideSuperAS(0.25, 0.2)

    def __nonCornerEdgeRobotFunctionallity(self):
        self.__saveDirectionsIfEmpty()
        if self.__isPrioritySet():
            self.__executeEdgePriorities()
        
    def __executeInsidePriorities(self):
        if self.higher_priority:
            self.__makeArch()
        else:
            self.__stay()
            self.insideRobotFunctionallity()

    def __makeArch(self):
        self.__saveDirectionsIfEmpty()
        self.__saveDirectionToNeighbor(False)
        self.__keepAngleBetweenNeighbors()
        #        self.__concentrateInsideRobots()


    def __stay(self):
        pass

    def __saveDirectionsIfEmpty(self):
        if not self.perpendicular_direction:
            self.__saveDirectionToNeighbor()
            self.__savePerpendicularDirection()

    def __isPrioritySet(self):
        if self.higher_priority == None:
            self.__checkIfHigherPriority()
            return False
        return True

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

    def __executeEdgePriorities(self):
        if self.higher_priority:
            self.robot.direction = self.direction_to_neighbor.copy()
            if self.__checkIfAngleIsProper(170):
                self.__resetTimer()
                #                self.robot.direction = self.perpendicular_direction.copy()
                #                self.robot.direction.rightRotation()
                self.__moveIfPathIsFree()
            else:
                self.__useTimerIfSet()
            self._keepDistanceInsideSuperAS(0.5, 0.4)
        else:
            self.__stay()

    def __checkIfAngleIsProper(self, angle=140):
        closest_neighbor, closest_distance = self._findClosestNeighbor()
        self.robot.direction.negate()
        other_cluster_neighbor, other_distance = self.__findClosestOtherClusterNeighbor()
        self.robot.direction.negate()
        if not other_cluster_neighbor:
            return False
        if self.checkAngle(closest_neighbor, self.robot, other_cluster_neighbor) < angle:
            return True
        return False


    def __useTimerIfSet(self):
        if not self.timerSet:
            self.__setTimer()
        else:
            self.robot.timer.tick()
            self.__setEqualizeDistanceSignal()

    def __setTimer(self):
        self.robot.setTimer(500) 
        self.timerSet = True

    def __setEqualizeDistanceSignal(self):
        if self.robot.timer.duration < 0:
            self.robot.broadcastMessage(EQUALIZE, True)
            self.paintItBlack((200, 0, 0))
            print("target executed properly.")

    def __resetTimer(self):
        self.timerSet = False            

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
            
    def __saveDirectionToNeighbor(self, initial=True):
        if initial:
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

    def __evaluateAngleBetween(self, n, direction):
        current_direction = self.robot.direction.copy()
        spot.direction_to_neighbor(self.robot, n)
        angle = math.atan2(self.robot.direction.y,
                           self.robot.direction.x) - math.atan2(
                               direction.y,
                               direction.x)
        self.robot.direction = current_direction
        return math.degrees(abs(angle))

    def __isSmaller(self, value, smaller_from_value):
        if value < smaller_from_value:
            return True
        return False

    def __isDistancePreserved(self, distance_to_closest, distance_to_opposite, desired_distance):
        distance_to_keep = desired_distance * (self.robot.sensor_range - self.robot.radius) + self.robot.radius
        print(distance_to_keep)
        if distance_to_closest < distance_to_keep \
           and distance_to_opposite < distance_to_keep:
            return True
        return False

    def __moveIfDistanceIsKept(self, distance_to_closest, distance_to_opposite, closest_neighbor, opposite_neighbor):
        if self.__isDistancePreserved(distance_to_closest, distance_to_opposite, 0.6):
            self.__moveIfPathIsFree()
        elif self.__isDistancePreserved(distance_to_closest, distance_to_opposite, 0.7):
            self._equalizeDistances(closest_neighbor, opposite_neighbor)
    

    def __keepAngleBetweenNeighbors(self):
        #        angle_to_closest, closest_neighbor, closest_neighbor_distance = self.__getAngleToClosestNeighbor()
        #        angle_to_opposite, opposite_neighbor, opposite_neighbor_distance = self.__getAngleToOppositeNeighbor(closest_neighbor)
        closest_neighbor, closest_neighbor_distance = self._findClosestNeighbor()
        self.robot.direction = self.direction_to_neighbor.copy()
        opposite_neighbor, opposite_neighbor_distance = self._findRobotOnOppositeSide(
            closest_neighbor)
        
        self.robot.direction = self.perpendicular_direction.copy()

        #if not self.__keepMaximalAngle(angle_to_closest, angle_to_opposite):
        if self.checkAngle(closest_neighbor, self.robot, opposite_neighbor) < 160:
            self._equalizeDistances(closest_neighbor, opposite_neighbor)
        else:
            self.__moveIfDistanceIsKept(closest_neighbor_distance, opposite_neighbor_distance, closest_neighbor, opposite_neighbor)

        self.__equalizeOnSignal(closest_neighbor, closest_neighbor_distance\
                                , opposite_neighbor, opposite_neighbor_distance)

    def __keepMaximalAngle(self, angle_to_closest, angle_to_opposite):
        if self.__isSmaller(angle_to_closest + angle_to_opposite, 200):
            self.robot.direction = self.perpendicular_direction.copy()
            #            self.paintItBlack()
            return True
        return False

    def __getAngleToClosestNeighbor(self):
        closest_neighbor, closest_neighbor_distance = self._findClosestNeighbor()
        angle_to_closest = self.__evaluateAngleBetween(closest_neighbor, self.perpendicular_direction)
        return angle_to_closest, closest_neighbor, closest_neighbor_distance

    def __getAngleToOppositeNeighbor(self, closest_neighbor):
        self.robot.direction = self.direction_to_neighbor
        opposite_neighbor, opposite_neighbor_distance = self._findRobotOnOppositeSide(
            closest_neighbor)
        angle_to_opposite = self.__evaluateAngleBetween(opposite_neighbor, self.perpendicular_direction)
        return angle_to_opposite, opposite_neighbor, opposite_neighbor_distance

    def __equalizeOnSignal(self, closest_neighbor, distance_to_closest, \
                           opposite_neighbor, distance_to_opposite):
        if self.__checkEqualizeInMessages():
            if not self.__isDistancePreserved(distance_to_closest, distance_to_opposite, 0.4):
                self._equalizeDistances(closest_neighbor, opposite_neighbor)
    
    def __checkEqualizeInMessages(self):
        for m in self.robot.received_messages:
            if EQUALIZE in m.keys():
                self.robot.broadcastMessage(EQUALIZE, True)
                return True
        return False

    def __concentrateInsideRobots(self, desired_distance=0.5):
        closest_neighbor, opposite_neighbor = self.__getTwoOppositeNeighbors()
        if not opposite_neighbor:
            return
        distance = spot.point_to_direction_rd(self.robot, opposite_neighbor)
        distance_to_keep = desired_distance* (self.robot.sensor_range - self.robot.radius) + self.robot.radius
        if distance < distance_to_keep:
            self.__setRobotPerpendicularDirection(opposite_neighbor)
            if not self.__isSurpassed(closest_neighbor):
                self.paintItBlack()
                self.__moveIfPathIsFree()

    def __getTwoOppositeNeighbors(self):
        self.same_cluster_neighbors.clear()
        self.same_cluster_neighbors = self._getSameClusterMembers()        
        self.robot.direction = self.perpendicular_direction.copy()        
        closest_neighbor, closest_neighbor_distance = self._findClosestNeighbor()
        spot.direction_to_neighbor(self.robot, closest_neighbor)
        opposite_neighbor, opposite_neighbor_distance = self._findRobotOnOppositeSide(
            closest_neighbor)
        return closest_neighbor, opposite_neighbor

    def __isSurpassed(self, closest_neighbor):
        epsilon = 20
        if self.__checkAngleFromDirectionToNeighbor(closest_neighbor) > 90 - epsilon:
            return True
        return False
            
            
    def __setRobotPerpendicularDirection(self, opposite_neighbor):
        self.robot.direction.leftRotation()
        if self.__checkAngleFromDirectionToNeighbor(opposite_neighbor) > 90:
            self.robot.direction.rightRotation()
            self.robot.direction.rightRotation()

    def __checkAngleFromDirectionToNeighbor(self, n):
        current_direction = self.robot.direction.copy()
        spot.direction_to_neighbor(self.robot, n)
        angle = math.atan2(self.robot.direction.y,
                           self.robot.direction.x) - math.atan2(
                               self.perpendicular_direction.y,
                               self.perpendicular_direction.x)
        self.robot.direction = current_direction
        return math.degrees(abs(angle))    

    def __moveIfPathIsFree(self):
        if not spot.is_any_collision(self.robot, 0.15):
            self.robot.direction.normalize()
            self.makeMove(True)

    def update(self):
        #self.robot.update_color()
        self.check_phase()
        self.robot.velocity = Velocity(0, 0)
        self.__makePFormation()
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
