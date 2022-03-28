import sys
import numpy as np
import pygame as pg
import math
import Simulation as sim
import SpotNeighbor as spot
import phase as ph
from phaseone import PhaseOne, PhaseOneAndHalf


class Robot(pg.sprite.Sprite):
    '''
    The object of the class Robot represents the robot on a board.
    
    It inherit from the base class sprite as this class is a basic representation of a character in pygame.
    '''
    def __init__(self, x, y, width, height, velocity=[0, 0], s_range=55):
        super().__init__()
        '''
        Creates a robot on board with the initial coordinates (x, y) and the given velocity.
        '''
        self.x = x
        self.y = y
        self.radius = 10
        self.width = width
        self.height = height
        self.s_range = s_range
        self.k = math.ceil(
            math.ceil(2 * np.pi * self.s_range) / (2 * self.radius))
        self.neighbors = []
        self.in_range_robots = 0
        self.iterator = 0
        self.messages = []  #obtained messages from other robots
        self.broadcast = {}  #messages to be broadcast
        '''
        Broadcast messages are the dictionary -> {msg_type : msg_value}
        Messages are the list of gather broadcasts from the neighbors
        It is assumed that it doesn't matter which robot place the massage
        '''
        self.AS = None
        self.joined = True  #indicates if the AS was created by us or not

        self.timer = (-1, -1, 0)
        '''
        It is important to posses the AS number -> if it changes than timer must immediately stop!
        tuple (AS number, timer_value, number_of_neighbors)
        timer_value = -1 -> indicates that no value of a timer was yet set, similarly with AS number
        '''

        self.image = pg.Surface([self.radius * 2, self.radius * 2])
        self.image.fill((255, 255, 255))  #background color (white)

        pg.draw.circle(self.image, (50, 150, 50), (self.radius, self.radius),
                       self.radius)  #(50, 150, 50) - GREEN

        self.rect = self.image.get_rect()
        self.position = np.array([x, y], dtype=np.float64)
        self.velocity = np.asarray(velocity, dtype=np.float64)
        self.moved = False

        #for the sake of changing direction
        self.dir_x = 0
        self.dir_y = 0

        self.state = "moving"  #initially robots move (just for aggregation algorithm)
        #        self.phase = 1  #the first phase is being currently run. This number can be only incremented!

        self.prev_coords = []  #previous coordinates of the neighbors

        self.faza = PhaseOne(self)

    def update(self):
        '''
        Updates the robot position on board, as well as it's state. Necessary behaviors are to be applied here.
        '''
        self.position += self.velocity
        x, y = self.position
        self.moved = False

        self.update_msg()
        #place for the swarm behaviors
        #First phase bahaviors:

        self.faza.update()

        #boundary parameters
        if x < 0 or x > self.width - 2 * self.radius:
            if self.faza.phase == 2:  #just for dbg
                self.velocity[0] = 0
                self.velocity[1] = 0
            self.velocity[0] = -self.velocity[0]
        if y < 0 or y > self.height - 2 * self.radius:
            if self.faza.phase == 2:  #just for dbg
                self.velocity[0] = 0
                self.velocity[1] = 0
            self.velocity[1] = -self.velocity[1]

        self.broadcast["Phase"] = self.faza.phase  #always broadcast the phase

        self.neighbors.clear(
        )  #list of neighbors must be refreshed in each update
        self.in_range_robots = 0

        self.rect.x = x
        self.rect.y = y
        self.x = int(x)
        self.y = int(y)

    def spotted(self, r):
        '''
        Adds the newly spotted neighbor to the list.
        '''
        self.neighbors.append(r)

    def in_range(self):
        '''
        Returns the number of neighbors that are in given range
        '''
        self.in_range_robots = self.in_range_robots + 1

    def is_allone(self):
        '''
        If any robot for any reason stays allone, then it come back to phase one, and it's AS is being created again
        '''
        if not self.neighbors:
            self.faza = PhaseOne(self)

    def update_msg(self):
        self.messages.clear()
        for n in self.neighbors:
            self.messages.append(n.broadcast)

    def clear_broadcast(self):
        self.broadcast.clear()
        self.broadcast["AS"] = self.AS

    def update_color(self):
        '''
        Mathematical operations are performed in order to get nicer colors
        '''
        red = self.AS % 256
        green = math.floor(self.AS / 4) % 256
        blue = math.floor(math.sqrt(self.AS)) % 256
        color = (red, green, blue)
        pg.draw.circle(self.image, color, (self.radius, self.radius),
                       self.radius)

    def create_AS(self):
        self.AS = np.random.randint(
            0, 65025)  #arbitrary number to be changed in the future
        self.broadcast['AS'] = self.AS
        self.update_color()

    def get_AS(self):
        votes = {}
        remain = False
        for m in self.messages:
            if 'AS' in m.keys():
                if not m['AS'] in votes:
                    votes[m['AS']] = 1
                    if m['AS'] == self.AS:  #don't change AS if you can still spot your colleagues
                        remain = True
                else:
                    votes[m['AS']] = votes[m['AS']] + 1
        if not votes:  #or remain:
            return None
        k = list(votes.keys())
        v = list(votes.values())
        return k[v.index(max(v))]  #getting the AS that is most common nearby

    def set_timer(self, t_min=1000, random=True, t_max=2000):
        '''
        Set the timer. if random is set to True, then the random time (from t_min to t_max) will be chosen
        Otherwise, the t_min value will be set
        '''
        if random:
            self.timer = (self.AS, np.random.randint(t_min, t_max),
                          len(self.neighbors))
        else:
            self.timer = (self.AS, t_min, len(self.neighbors))

    def find_direction(self):
        '''
        Finds the path without any obstacles (in radar range). If it is not possible, then some other direction is given

        Returned value is the direction - pair (x, y). It should be multiplied by the velocity to start moving.
        '''
        S = []
        radius = int(
            self.s_range *
            1.4141)  #in simulation the range is not a circuit - it is a square

        for i in range(1, self.k):
            S.append(False)  #initial data, for the purpouse of chain
        S.append(spot.check_x0_line(self, radius))
        for i in range(self.k - 1):
            si = spot.check_line(self, i + 1, self.k, radius)
            S[i] = si
            if si:
                continue
        chain = (0, 0)  #pair -> [index, chain_lenght]
        longest_chain = (0, 0)
        iterator = 0
        '''
        Missing special case - when there is such scenario:
        S = [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0]
        0 - is false, 1 - is true
        Then in order to find best direction, one should treat table as a ring.
        '''
        for i in S:  #finding free chain
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
        if longest_chain[1] > 0:
            direction = longest_chain[0] - int(longest_chain[1] / 2)
        else:
            #There is a need to change the leader
            pass
        if direction == 0:
            return (0, 1)
        return (spot.calc_x(direction, 100) / 100,
                spot.calc_y(direction, 100) / 100)

    def follower_msg(self):
        '''
        Gets the route given by the leader.
        '''
        for m in self.messages:
            if "Direction" in m.keys() and m["AS"] == self.AS:
                self.broadcast["Direction"] = m["Direction"]
                self.dir_x = m["Direction"][0]
                self.dir_y = m["Direction"][1]
