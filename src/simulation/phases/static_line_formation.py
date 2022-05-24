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
from simulation.robot.agreement.Flooding import Flooding
from simulation.phases.flooding.TimestampFlood import TimestampFlood
from simulation.robot.agreement.ThreeStateAgreement import SYN, SYN_ACK, ACK

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
        self.timestamp_flood = TimestampFlood(self.robot.threeStateAgreement, \
                                              Flooding(superAS, self.robot.received_messages, self.robot.broadcastMessage))
        self.robot.agreement_state = SYN

    def dbg_msg(self):
        print("TUTAJ JESTEM!!!")

    def updateColor(self, new_color):  #just for dbg
        pg.draw.circle(self.robot.image, new_color,
                       (self.robot.radius, self.robot.radius),
                       self.robot.radius)

    def __changeColorIfTimestamp(self, isEdgeRobot):  #just for dbg
        new_color = self.__checkForFlood(isEdgeRobot)
        if new_color > 0:
            self.dbg_msg()
            self.updateColor(BLUE)

    def __checkForFlood(self, isEdgeRobot):
        if self.timestamp_flood.repeat():
            return self.timestamp_flood.getTimeWhenFinished(isEdgeRobot)
        return 0

    def __startFlood(self):
        if self.timerSet and self.__isTimerFinished():
            self.timestamp_flood.spillOver()
            self.timerSet = False
        else:
            self.__setTimer()

    def __isTimerFinished(self):
        self.robot.timer.tick()
        if self.robot.timer.duration <= 0:
            return True
        return False

    def __setTimer(self):
        if not self.timerSet:
            self.timerSet = True
            self.robot.setRandomTimer(500, 1000)

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
        self.__changeColorIfTimestamp(False)
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
        self.__startFlood()
        self.__changeColorIfTimestamp(self)
        closest_neighbor, distance_to_neighbor = self.findClosestNeighbor()
        if not closest_neighbor:
            return
        self.__keepDistance(closest_neighbor, distance_to_neighbor)

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

    def __keepDistance(self, neighbor, distance_to_neighbor):
        if distance_to_neighbor < (
                0.8 *
            (self.robot.sensor_range - self.robot.radius)) + self.robot.radius:
            spot.direction_to_neighbor(self.robot, neighbor)
            self.robot.direction.negate()
            self.moveIfPathIsFree()

    def moveIfPathIsFree(self):
        if not spot.is_any_collision(self.robot, 0.2):
            self.robot.direction.normalize()
            self.makeMove()

    def getSameClusterMembers(self):
        self.same_cluster_neighbors.clear()
        for n in self.robot.neighbors:
            if n.cluster_id != self.robot.cluster_id:
                continue
            self.same_cluster_neighbors.append(n)
        return self.same_cluster_neighbors

    def update(self):
        self.check_phase()
        self.robot.velocity.x = 0
        self.robot.velocity.y = 0

        self.timestamp_flood.agreement.updateMessages(
            self.robot.received_messages)

        self.same_cluster_neighbors.clear()
        self.same_cluster_neighbors = self.getSameClusterMembers()
        if self.isEdgeRobot():
            '''
            pg.draw.circle(self.robot.image, HORRIBLE_YELLOW,
                           (self.robot.radius, self.robot.radius),
                           self.robot.radius)
            '''
            self.edgeRobotFunctionallity()
        else:
            BLACK = (0, 0, 0)
            pg.draw.circle(self.robot.image, BLACK,
                           (self.robot.radius, self.robot.radius),
                           self.robot.radius)
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
