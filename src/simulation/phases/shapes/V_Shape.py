from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from simulation.phases.shapes.Shape import Shape
from utils import SpotNeighbor as spot
import math

import pygame as pg #dbg

class V_Shape(Shape):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        self.perpendicular_direction = None
        self.direction_to_neighbor = None
        self.robot.direction = Direction(1, 1)

    def paintItBlack(self):
        BLACK = (0, 200, 0)
        pg.draw.circle(self.robot.image, BLACK,
                       (self.robot.radius, self.robot.radius),
                       self.robot.radius)              

    def __isCornerEdgeRobot(self):
        self.robot.direction = Direction(1, 1)
        neighbor, rd = spot.find_best_neighbor(self.robot, True)
        if not neighbor:
            return True
        return False

    def __cornerEdgeRobotFunctionallity(self):
        self._keepDistanceInsideSuperAS()

    def __nonCornerEdgeRobotFunctionallity(self):
        self.paintItBlack()
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
        if self.__checkAngleFromDirectionToNeighbor(other_neighbor) >= 90:
            self.perpendicular_direction.rightRotation()
            self.perpendicular_direction.rightRotation()

    def __checkAngleFromDirectionToNeighbor(self, n):
        angle = math.atan2(n.position.y - self.robot.position.y,
                           n.position.x - self.robot.position.x) - math.atan2(
                               self.direction_to_neighbor.y,
                               self.direction_to_neighbor.x)

        return math.degrees(abs(angle))        
    
    def __getOtherSuperclusterNeighbors(self):
        other_neighbors = []
        for n in self.robot.neighbors:
            if not n in self.same_cluster_neighbors:
                other_neighbors.append(n)
        return other_neighbors

    def __moveUntilAngleIsObtained(self):
        desired_angle = 60
        if self.__evaluateAngle() < desired_angle:
            self.robot.direction = self.perpendicular_direction.copy()
            self.__moveIfPathIsFree()

    def __evaluateAngle(self):
        if self.direction_to_neighbor:
            self.robot.direction = self.direction_to_neighbor
        else:
            self.robot.direction = Direction(1, 1)
        spot.follower(self.robot)
        direction_to_neighbor = self.robot.direction.copy()
        angle = math.atan2(direction_to_neighbor.y, direction_to_neighbor.x) \
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
            self._keepDistanceInsideSuperAS()
        else:
            self.insideRobotFunctionallity()            

    def update(self):
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
