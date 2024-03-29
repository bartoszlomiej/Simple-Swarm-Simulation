from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from simulation.phases.shapes.Shape import Shape
#from simulation.phases.shapes.W_shape import W_shape
from simulation.phases.shapes.Proxy_shape import W_Shape_Proxy
from utils import SpotNeighbor as spot
import math

import pygame as pg #dbg

class V_Shape(Shape):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        self.perpendicular_direction = None
        self.direction_to_neighbor = None
        self.robot.direction = Direction(1, 1)

    def __setTimer(self):
        self.robot.setTimer(500) 
        self.timerSet = True

    def __upgradeIfTimerFinished(self):
        if self.robot.timer.duration < 0:
            self.upgrade(5.5, self.robot.super_cluster_id)

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

    def __nonCornerEdgeRobotFunctionallity(self):
        if not self.perpendicular_direction:
            self.__saveDirectionToNeighbor()
            self.__savePerpendicularDirection()
        self.robot.direction = self.direction_to_neighbor.copy()
        self.__moveUntilAngleIsObtained()

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
            self.insideRobotFunctionallity()
            
    def serialize(self):
        return (self.phase, self.timerSet, self.perpendicular_direction, self.direction_to_neighbor)

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
                if m["Phase"] >= 5.5:
                    robot.broadcast["superAS"] = self.robot.super_cluster_id
                    self.upgrade(m["Phase"], self.robot.super_cluster_id)
                    return

    def upgrade(self, next_phase=4, superAS=None):
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 5.5:
            self.robot.faza = W_Shape_Proxy(self.robot, superAS)
            #            self.robot.faza = W_Shape(self.robot, superAS, self.perpendicular_direction)

