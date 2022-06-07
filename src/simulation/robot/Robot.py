import numpy as np
import pygame as pg
import math

from simulation.robot.Timer import Timer
from utils import SpotNeighbor as spot
from simulation.phases.phaseone import PhaseOne, PhaseOneAndHalf
from simulation.phases.attraction_point import AttractionPoint
from simulation.phases.merge_clusters_to_static_line import MergeClustersToStaticLine
from simulation.phases.StepForward import StepForward
from simulation.robot import RobotState
from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from utils.colors import WHITE, GREEN
from simulation.robot.agreement.ThreeStateAgreement import SYN, SYN_ACK, ACK
from simulation.robot.agreement.TurnBack import TurnBack
from simulation.robot.agreement.Downgrade import Downgrade


class Robot(pg.sprite.Sprite):
    def __init__(self,
                 position,
                 board_resolution,
                 sensor_range,
                 velocity_level,
                 radius=10):
        super().__init__()

        self.position = position
        self.board_resolution = board_resolution
        self.sensor_range = sensor_range
        self.velocity = Velocity(0, 0) #Velocity.generateRandom(velocity_level)
        self.velocity_level = velocity_level
        self.radius = radius

        self.detected_robots_number = 0
        self.iterator = 0

        self.sensors_number = self.calculate_sensors_number(
            sensor_range, radius)
        self.neighbors = []
        self.received_messages = []
        self.broadcast = {}

        self.cluster_id = None
        self.super_cluster_id = None
        self.joined_to_cluster = True  # indicates if the AS was created by us or not

        self.timer = Timer(-1, -1, 0)
        '''
        It is important to posses the AS number -> if it changes than timer must immediately stop!
        tuple (AS number, timer_value, number_of_neighbors)
        timer_value = -1 -> indicates that no value of a timer was yet set, similarly with AS number
        '''

        self.image = pg.Surface([self.radius * 2, self.radius * 2])
        self.image.fill(WHITE)  # background color (white)

        pg.draw.circle(self.image, GREEN, (self.radius, self.radius),
                       self.radius)

        self.rect = self.image.get_rect()
        self.moved = False

        self.ap = None  # JUST FOR DBG

        # for the sake of changing direction
        self.direction = Direction.generateRandom()
        self.sensors = []
        self.S = []
        self.is_downgrade = False

        self.state = RobotState.MOVING  # initially robots move (just for aggregation algorithm)
        self.waiting = False
        self.agreement_state = SYN

        if not self.sensors_number % 2:
            self.sensors_number += 1
        self.initialize_sensors()
        self.faza = PhaseOne(self)

    def update(self):
        '''
        Updates the robot position on board, as well as it's state. Necessary behaviors are to be applied here.
        '''
        self.position.moveBy(self.velocity)
        self.moved = False

        self.update_msg()

        # place for the swarm behaviors
        # First phase bahaviors:

        self.clear_broadcast()

        self.faza.update()

        # boundary parameters
        if self.position.x < 0 or self.position.x > self.board_resolution.width - 2 * self.radius:
            if self.faza.phase == 2:  # just for dbg
                #                self.direction.negate()
                self.agreement_state = SYN#_ACK
                self.broadcast["Turn back"] = self.direction.copy()
            #self.velocity.x = -self.velocity.x
        if self.position.y < 0 or self.position.y > self.board_resolution.height - 2 * self.radius:
            if self.faza.phase == 2:  # just for dbg
                #self.direction.negate()
                self.agreement_state = SYN#_ACK
                self.broadcast["Turn back"] = self.direction.copy()
           # self.velocity.y = -self.velocity.y

        if not self.is_downgrade:
            self.broadcast[
                "Phase"] = self.faza.phase  # always broadcast the phase
        self.broadcast["AS"] = self.cluster_id
        if self.faza.phase > 2:
            self.broadcast["superAS"] = self.super_cluster_id

        self.neighbors.clear(
        )  # list of neighbors must be refreshed in each update
        self.detected_robots_number = 0

        self.rect.x = self.position.x
        self.rect.y = self.position.y

    def initialize_sensors(self):
        '''
        Calculates a slope for each sensor. This is important due to optimization.
        '''
        for i in range(1, self.sensors_number):
            a = spot.calculate_slope(
                0, 0, spot.calc_x(i, self.radius, self.sensors_number),
                spot.calc_y(i, self.radius, self.sensors_number))
            self.sensors.append(a)

    def sensorFunction(self, i):
        '''
        Returns the b in y = ax + b function from the current position of the robot.
        '''
        return (self.position.y - self.sensors[i] * self.position.x)

    def spotted(self, r):
        '''
        Adds the newly spotted neighbor to the list.
        '''
        self.neighbors.append(r)

    def in_range(self):
        '''
        Returns the number of neighbors that are in given range
        '''
        self.detected_robots_number = self.detected_robots_number + 1

    def is_allone(self):
        '''
        If any robot for any reason stays allone, then it come back to phase one, and it's AS is being created again
        '''
        if not self.neighbors:
            self.direction = Direction.generateRandom()
            self.faza = PhaseOne(self)
            return
        for n in self.neighbors:
            if self.cluster_id == n.cluster_id:
                return
        if not spot.is_collision_distance(self):
            self.direction = Direction.generateRandom()
            self.faza = PhaseOne(self)

    def isAlloneInSupercluster(self):
        '''
        If any robot for any reason stays allone, then it come back to phase one, and it's AS is being created again
        '''
        if not self.neighbors:
            self.faza = PhaseOne(self)
            return
        for n in self.neighbors:
            if self.super_cluster_id == n.super_cluster_id:
                return
        if not spot.is_collision_distance(self):
            self.faza = PhaseOne(self)            

    def update_msg(self):
        self.received_messages.clear()
        for n in self.neighbors:
            self.received_messages.append(n.broadcast)

    def clear_broadcast(self):
        self.broadcast.clear()
        self.broadcast["AS"] = self.cluster_id

    def update_color(self):
        '''
        Mathematical operations are performed in order to get nicer colors
        '''
        red = self.cluster_id % 256
        green = math.floor(self.cluster_id / 4) % 256
        blue = math.floor(math.sqrt(self.cluster_id)) % 256
        color = (red, green, blue)
        pg.draw.circle(self.image, color, (self.radius, self.radius),
                       self.radius)

    def create_AS(self):
        self.cluster_id = np.random.randint(
            0, 65025)  # arbitrary number to be changed in the future
        self.broadcast['AS'] = self.cluster_id
        self.update_color()

    def get_AS(self):
        votes = {}
        remain = False
        for m in self.received_messages:
            if 'AS' in m.keys():
                if not m['AS'] in votes:
                    votes[m['AS']] = 1
                    if m['AS'] == self.cluster_id:  # don't change AS if you can still spot your colleagues
                        remain = True
                else:
                    votes[m['AS']] = votes[m['AS']] + 1
        if not votes:  # or remain:
            return None
        k = list(votes.keys())
        v = list(votes.values())
        return k[v.index(max(v))]  # getting the AS that is most common nearby

    def setTimer(self, duration):
        self.timer = Timer(self.cluster_id, duration, len(self.neighbors))

    def setRandomTimer(self, duration_from=1000, duration_to=2000):
        self.timer = Timer.generateRandom(self.cluster_id, duration_from,
                                          duration_to, len(self.neighbors))

    def find_direction(self):
        '''
        Finds the path without any obstacles (in radar range). If it is not possible, then some other direction is given

        Returned value is the direction - pair (x, y). It should be multiplied by the velocity to start moving.
        '''
        S = self.S
        S.clear()
        radius = int(
            self.sensor_range * 1.4141
        )  # in simulation the range is not a circuit - it is a square

        for i in range(1, self.sensors_number):
            S.append(False)  # initial data, for the purpouse of chain
        S.append(spot.check_x0_line(self, radius))
        for i in range(self.sensors_number - 1):
            si = spot.check_line(self, i, self.sensors_number, radius)
            #            si = spot.check_line(self, i + 1, self.k, radius)
            S[i] = si
            if si:
                continue
        chain = (0, 0)  # pair -> [index, chain_lenght]
        longest_chain = (0, 0)
        iterator = 0
        '''
        Missing special case - when there is such scenario:
        S = [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0]
        0 - is false, 1 - is true
        Then in order to find best direction, one should treat table as a ring.
        '''
        for i in S:  # finding free chain
            if not i:
                if chain[0] == iterator - 1:
                    chain = (iterator, chain[1] + 1)
                    if chain[1] > longest_chain[1]:
                        longest_chain = chain
                else:
                    chain = (iterator, 1)
                    if chain[1] > longest_chain[1]:
                        longest_chain = chain
            iterator += 1
        if longest_chain[1] > (len(S) / 4):
            direction = longest_chain[0] - int(longest_chain[1] / 2)
            if direction == 0:
                return Direction(0, 1)
        else:
            # There is a need to change the leader
            self.direction.negate()
            if self.direction.x != 0 and self.direction.y != 0:
                self.agreement_state = SYN#_ACK
                self.broadcast["Turn back"] = self.direction.copy()
                return self.direction.copy()
        return Direction(
            spot.calc_x(direction, 100, self.sensors_number) / 100,
            spot.calc_y(direction, 100, self.sensors_number) / 100)

    def repeatDirection(self, message):
        if not message:
            self.broadcast["Direction"] = self.direction.copy()
        elif "Direction" in message.keys():
            self.direction = message["Direction"].copy()
            self.broadcast["Direction"] = self.direction.copy()

    def follower_msg(self):
        turn_back = TurnBack(self.cluster_id, self.received_messages,
                             self.broadcastMessage, self.getDirection,
                             self.checkCorrectness)
        if self.threeStateAgreement(turn_back):
            self.communicationFinished()
            return
        for m in self.received_messages:
            if "Direction" in m.keys() and m["AS"] == self.cluster_id:
                if not self.checkCorrectness(m["Direction"]):
                    continue
                self.broadcast["Direction"] = m["Direction"].copy()
                self.getDirection(m["Direction"])

    def threeStateAgreement(self, agreement):
        agreement.state = self.agreement_state
        if agreement.isAgreementOn():
            self.agreement_state = agreement.state
            return True
        return False

    def communicationFinished(self):
        if self.agreement_state == ACK:
            self.agreement_state = SYN

    def calculate_sensors_number(self, sensor_range, radius):
        return math.ceil(math.ceil(2 * np.pi * sensor_range) / (2 * radius))

    def broadcastMessage(self, message, value):
        self.broadcast[message] = value

    def getDirection(self, value):
        self.direction = value.copy()

    def checkCorrectness(self, direction):
        if direction.x == 0 and direction.y == 0:
            return False
        return True

    def getRobotState(self):
        state = (self.position, self.board_resolution, self.sensor_range,
                 self.velocity, self.velocity_level, self.radius,
                 self.detected_robots_number, self.iterator, self.sensors_number,
                 self.broadcast, self.cluster_id, self.super_cluster_id,
                 self.joined_to_cluster, self.timer, self.moved,
                 self.ap, self.direction, self.sensors, self.S,
                 self.is_downgrade, self.state, self.waiting,
                 self.agreement_state, self.faza.serialize()
                 )
        return state

    def loadState(self, serialized_data):
        self.position = serialized_data[0]
        self.board_resolution = serialized_data[1]
        self.sensor_range = serialized_data[2]
        self.velocity = serialized_data[3]
        self.velocity_level = serialized_data[4]
        self.radius = serialized_data[5]
        self.detected_robots_number = serialized_data[6]
        self.iterator = serialized_data[7]
        self.sensors_number = serialized_data[8]
        self.broadcast = serialized_data[9]
        self.cluster_id = serialized_data[10]
        self.super_cluster_id = serialized_data[11]
        self.joined_to_cluster = serialized_data[12]
        self.timer = serialized_data[13]
        self.moved = serialized_data[14]
        self.ap = serialized_data[15]
        self.direction = serialized_data[16]
        self.sensors = serialized_data[17]
        self.S = serialized_data[18]
        self.is_downgrade = serialized_data[19]
        self.state = serialized_data[20]
        self.waiting = serialized_data[21]
        self.agreement_state = serialized_data[22]
        self.__loadProperPhase(serialized_data[23])


    def __loadProperPhase(self, serialized_phase):
        if type(serialized_phase) == tuple:
            next_phase = serialized_phase[0]
        else:
            next_phase = serialized_phase
        if next_phase == 1:
            self.faza = PhaseOne(self)
            self.faza.state = serialized_phase[1]
        if next_phase == 1.5:
            self.faza = PhaseOneAndHalf(self)
        elif next_phase == 2:
            self.faza = AttractionPoint(self)
        elif next_phase == 3:
            self.faza = MergeClustersToStaticLine(self, self.super_cluster_id)
            self.faza.stacked = serialized_phase[1] 
        elif next_phase == 4:
            self.faza = StepForward(self, self.super_cluster_id) 
            self.faza.timerSet = serialized_phase[1] 
