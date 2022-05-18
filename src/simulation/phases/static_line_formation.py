import pygame as pg
import math
from utils import SpotNeighbor as spot
from simulation.phases.phase import Phase
import simulation.phases.phaseone as ph1
import simulation.phases.phasetwo as ph2
from simulation.robot import RobotState
from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
import simulation.phases.merge_clusters_to_static_line as mg


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
        if not opposite_neighbor:
            return
        self.equalizeDistances(closest_neighbor, opposite_neighbor)

    def equalizeDistances(self, closest_neighbor, opposite_neighbor):
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
        self.moveIfPathIsFree()

    def findRobotOnOppositeSide(self, closest_neighbor):
        self.same_cluster_neighbors.remove(closest_neighbor)

        while self.same_cluster_neighbors:
            opposite_neighbor, distance = self.findClosestNeighbor()
            if self.checkAngle(closest_neighbor, self.robot,
                               opposite_neighbor) > 90.0:
                return opposite_neighbor, distance

            self.same_cluster_neighbors.remove(opposite_neighbor)

        self.upgrade(3, self.robot.super_cluster_id)
        return None, 0

    def changeClosestRobot(self, closest_neighbor):
        self.same_cluster_neighbors.remove(closest_neighbor)
        self.insideRobotFunctionallity()

    def edgeRobotFunctionallity(self):
        closest_neighbor, distance_to_neighbor = self.findClosestNeighbor()
        if not closest_neighbor:
            return
        self.keepDistance(closest_neighbor, distance_to_neighbor)

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
            self.robot.direction.negate()
            self.moveIfPathIsFree()

    def moveIfPathIsFree(self):
        if not spot.is_any_collision(self.robot):
            self.robot.direction.normalize()
            self.makeMove()
        else:
            self.robot.direction.stop()

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
        self.same_cluster_neighbors.clear()
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
        elif next_phase == 3:
            self.robot.faza = mg.MergeClustersToStaticLine(self.robot, superAS)
        #        elif next_phase == 4:
        #            self.robot.faza = ph4.PhaseFour(self.robot, superAS)
