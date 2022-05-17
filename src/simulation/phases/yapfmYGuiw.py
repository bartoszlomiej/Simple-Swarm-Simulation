import pygame as pg
import math
from utils import SpotNeighbor as spot
from simulation.phases.phase import Phase
import simulation.phases.phaseone as ph1
import simulation.phases.phasetwo as ph2
from simulation.robot import RobotState
from simulation.robot.Velocity import Velocity


class StaticLineFormation(Phase):
    def __init__(self, Robot, superAS):
        super().__init__(Robot)
        self.phase = 3
        self.robot.cluster_id = superAS
        self.robot.super_cluster_id = superAS
        self.isIncreased = False
        self.timerSet = False
        self.robot.velocity = Velocity(0, 0)
        self.robot.state = RobotState.STOPPED
        self.same_cluster_neighbors = []

    def isEdgeRobot(self):
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
        closest_neighbor, closest_neighbor_distance = self.findClosestNeighbor(
        )
        opposite_neighbor, opposite_neighbor_distance = self.findRobotOnOppositeSide(
            closest_neighbor)
        self.equalizeDistances(closest_neighbor, opposite_neighbor)

    def equalizeDistances(self, closest_neighbor, opposite_neighbor):
        point_x = (closest_neighbor.position.x +
                   opposite_neighbor.position.x) / 2
        point_y = (closest_neighbor.position.y +
                   opposite_neighbor.position.y) / 2
        delta_x = (point_x - self.robot.position.x)
        delta_y = (point_y - self.robot.position.y)
        if not delta_x or not delta_y:
            self.robot.dir_x = 0
            self.robot.dir_y = 0
            return
        suma = math.sqrt(delta_x**2 + delta_y**2)
        self.robot.dir_x = delta_x / suma
        self.robot.dir_y = delta_y / suma
        self.moveIfPathIsFree()

    def findRobotOnOppositeSide(self, closest_neighbor):
        self.same_cluster_neighbors.remove(closest_neighbor)
        while self.same_cluster_neighbors:
            opposite_neighbor, distance = self.findClosestNeighbor()
            if self.checkAngle(closest_neighbor, self.robot,
                               opposite_neighbor) > 90.0:
                return opposite_neighbor, distance
            self.same_cluster_neighbors.remove(opposite_neighbor)
        print("BLAD!!!!!!!!!!!!!!!!!!!!!!!!")
        print(self.isEdgeRobot())
        print("There is a robot", opposite_neighbor)
        print(self.checkAngle(closest_neighbor, self.robot, opposite_neighbor))
        print("==============================================================")

    def edgeRobotFunctionallity(self):
        closest_neighbor, distance_to_neighbor = self.findClosestNeighbor()
        if not closest_neighbor:
            return
        self.keepDistance(closest_neighbor, distance_to_neighbor)

    def checkAngle(self, n1, robot, n2):
        angle = math.atan2(n1.position.y - robot.position.y,
                           n1.position.x - robot.position.x) - math.atan2(
                               n2.position.y - robot.position.y,
                               n2.position.x - robot.position.x)

        return math.degrees(abs(angle))

    def findClosestNeighbor(self):
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

    def keepDistance(self, neighbor, distance_to_neighbor):
        if distance_to_neighbor < (
                0.8 *
            (self.robot.sensor_range - self.robot.radius)) + self.robot.radius:
            spot.direction_to_neighbor(self.robot, neighbor)
            self.robot.dir_x *= -1
            self.robot.dir_y *= -1
            self.moveIfPathIsFree()

    def moveIfPathIsFree(self):
        a, b, d = spot.direction_line_equation(self.robot)
        if not spot.is_any_collision(self.robot):
            self.makeMove()
        else:
            self.dir_x = 0
            self.dir_y = 0

    def makeMove(self):
        while (self.robot.dir_x**2 + self.robot.dir_y**2) > 1:
            self.robot.dir_x /= 2
            self.robot.dir_y /= 2
        self.robot.velocity.x = self.robot.dir_x * self.robot.velocity_level
        self.robot.velocity.y = self.robot.dir_y * self.robot.velocity_level

    def getSameClusterMembers(self):
        self.same_cluster_neighbors = []
        for n in self.robot.neighbors:
            if n.cluster_id != self.robot.cluster_id:
                continue
            self.same_cluster_neighbors.append(n)
        return self.same_cluster_neighbors

    def update(self):
        self.check_phase()
        self.robot.velocity.x = 0
        self.robot.velocity.y = 0

        BLACK = (0, 0, 0)
        pg.draw.circle(self.robot.image, BLACK,
                       (self.robot.radius, self.robot.radius),
                       self.robot.radius)
        self.same_cluster_neighbors = self.getSameClusterMembers()
        if self.isEdgeRobot():
            self.edgeRobotFunctionallity()
        else:
            self.insideRobotFunctionallity()
        self.robot.broadcast["superAS"] = self.robot.super_cluster_id

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
        #        elif next_phase == 4:
        #            self.robot.faza = ph4.PhaseFour(self.robot, superAS)
