import pygame as pg
import math
from utils import SpotNeighbor as spot
from simulation.robot.Timer import Timer
from simulation.phases.phase import Phase
import simulation.phases.phaseone as ph1
import simulation.phases.phasetwo as ph2
from simulation.robot import RobotState
from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
import simulation.phases.static_line_formation as st
import simulation.phases.attraction_point as dbg
from simulation.phases.fix_stacked_robots import Stacked
from simulation.robot.agreement.Downgrade import Downgrade

class MergeClustersToStaticLine(Phase):
    def __init__(self, Robot, superAS):
        super().__init__(Robot)
        self.phase = 3
        self.robot.super_cluster_id = superAS
        self.isIncreased = False
        self.robot.velocity = Velocity(0, 0)
        self.robot.update_color()
        self.robot.state = RobotState.MOVING
        self.stacked = None
        self.robot.waiting = False

    def followerFunctionallity(self):
        spot.follower(self.robot)
        self.moveIfPathIsFree()

    def leaderFunctionallity(self, main_cluster_neighbors):
        if len(main_cluster_neighbors) == 1:
            self.goToSingleRobot(main_cluster_neighbors[-1])
            self.joinTheEdgeRobot(main_cluster_neighbors[-1])
        elif len(main_cluster_neighbors) > 1:
            self.mergeIfPossible(main_cluster_neighbors)

    def goToSingleRobot(self, neighbor):
        spot.direction_to_neighbor(self.robot, neighbor)
        self.moveIfPathIsFree()

    def joinTheEdgeRobot(self, edge_robot):
        minimal_distance = 0.4 * (self.robot.sensor_range - self.robot.radius) \
                                  + self.robot.radius
        real_distance = spot.relative_distance(self.robot.position.x, \
                                               self.robot.position.y, \
                                               edge_robot.position.x, \
                                               edge_robot.position.y)
        if real_distance < minimal_distance:
            self.upgrade(3, self.robot.super_cluster_id)

    def getMainClusterNeighbors(self):
        main_cluster_neighbors = []
        for n in self.robot.neighbors:
            if n.cluster_id == self.robot.super_cluster_id:
                main_cluster_neighbors.append(n)
        return main_cluster_neighbors

    def mergeIfPossible(self, main_cluster_neighbors):
        first_neighbor, second_neighbor = self.chooseBestTwoNeighbors(
            main_cluster_neighbors)
        epsilon = 20
        angle = self.checkAngle(first_neighbor, self.robot, second_neighbor)
        if angle > (90.0 + epsilon):
            self.upgrade(3, self.robot.super_cluster_id)
        elif angle < (epsilon / 2):
            self.joinTheEdgeRobot(first_neighbor)
        else:
            self.goBetweenTwoRobots(first_neighbor, second_neighbor)

    def goBetweenTwoRobots(self, first_neighbor, second_neighbor):
        point_x = (first_neighbor.position.x + second_neighbor.position.x) / 2
        point_y = (first_neighbor.position.y + second_neighbor.position.y) / 2
        delta_x = (point_x - self.robot.position.x)
        delta_y = (point_y - self.robot.position.y)
        if not delta_x or not delta_y:
            self.robot.direction.stop()
            return
        suma = math.sqrt(delta_x**2 + delta_y**2)
        self.robot.direction = Direction(delta_x / suma, delta_y / suma)
        self.moveIfPathIsFree()

    def chooseBestTwoNeighbors(self, main_cluster_neighbors):
        first_neighbor = self.findClosestNeighbor(main_cluster_neighbors)
        main_cluster_neighbors.remove(first_neighbor)
        second_neighbor = self.findClosestNeighbor(main_cluster_neighbors)
        return first_neighbor, second_neighbor

    def findClosestNeighbor(self, main_cluster_neighbors):
        if not main_cluster_neighbors:
            return None
        closest_distance = 10000
        closest_neighbor = None
        distance = None
        for n in main_cluster_neighbors:
            distance = spot.relative_distance(self.robot.position.x,
                                              self.robot.position.y,
                                              n.position.x, n.position.y)
            if distance < closest_distance:
                closest_distance = distance
                closest_neighbor = n
        return closest_neighbor

    def moveIfPathIsFree(self):
        if not spot.is_any_collision(self.robot):
            self.robot.direction.normalize()
            self.makeMove()
        else:
            self.tryPerpendicularMotion()

    def downgrade(self):
        self.robot.broadcast["Downgrade"] = 2
        self.robot.direction = self.robot.find_direction()
        self.robot.broadcast["Direction"] = self.robot.direction.copy()

    def checkForDowngrade(self):
        downgrade = Downgrade(self.robot.cluster_id, self.robot.received_messages, self.robot.broadcastMessage, self.robot.repeatDirection)
        if downgrade.checkIfDowngrade():
            self.upgrade(2)
        else:
            if downgrade.is_downgrade:
                pass #don't check phase???

    def mainClusterTimeout(self):
        if not self.stacked:
            self.stacked = Stacked(self.robot)
        else:
            if self.stacked.isStacked():
                self.stacked = None
                self.downgrade()

    def tryPerpendicularMotion(self):
        main_cluster_neighbors = self.getMainClusterNeighbors()
        if not main_cluster_neighbors:
            self.mainClusterTimeout()
            GREEN = (100, 200, 50)
            pg.draw.circle(self.robot.image, GREEN,
                           (self.robot.radius, self.robot.radius),
                           self.robot.radius)
            return
        else:
            OTHER = (200, 50, 200)
            pg.draw.circle(self.robot.image, OTHER,
                           (self.robot.radius, self.robot.radius),
                           self.robot.radius)

        best_neighbor = self.findClosestNeighbor(main_cluster_neighbors)
        spot.direction_to_neighbor(self.robot, best_neighbor)
        self.robot.direction.perpendicular()

        if not spot.is_any_collision(self.robot, 0.15):
            self.robot.direction.normalize()
            self.makeMove()

    def update(self):
        self.check_phase()
        self.robot.velocity.x = 0
        self.robot.velocity.y = 0
        self.robot.update_color()

        main_cluster_neighbors = []
        main_cluster_neighbors = self.getMainClusterNeighbors()
        if main_cluster_neighbors:
            self.leaderFunctionallity(main_cluster_neighbors)
        else:
            self.followerFunctionallity()

        self.robot.broadcast["superAS"] = self.robot.super_cluster_id
        self.robot.is_allone()
        self.checkForDowngrade()

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
            self.robot.faza = dbg.AttractionPoint(self.robot)
        elif next_phase == 3:
            self.robot.faza = st.StaticLineFormation(self.robot, superAS)
        #        elif next_phase == 4:
        #            self.robot.faza = ph4.PhaseFour(self.robot, superAS)
