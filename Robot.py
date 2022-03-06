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
    def __init__(self, x, y, width, height, velocity=[0, 0]):
        super().__init__()
        '''
        Creates a robot on board with the initial coordinates (x, y) and the given velocity.
        '''
        self.x = x
        self.y = y
        self.radius = 10
        self.width = width
        self.height = height
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
        self.phase = 1  #the first phase is being currently run. This number can be only incremented!

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
        if self.phase == 1:
            self.faza.update()
        elif self.phase == 1.5:
            self.faza.update()
        elif self.phase == 2:
            self.faza.update()
            #            self.collective_movement()

        #boundary parameters
        if x < 0 or x > self.width - 2 * self.radius:
            if self.phase == 2:  #just for dbg
                self.velocity[0] = 0
                self.velocity[1] = 0
                self.phase = 3
            self.velocity[0] = -self.velocity[0]
        if y < 0 or y > self.height - 2 * self.radius:
            if self.phase == 2:  #just for dbg
                self.velocity[0] = 0
                self.velocity[1] = 0
                self.phase = 3
            self.velocity[1] = -self.velocity[1]

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

    def update_msg(self):
        self.messages.clear()
        for n in self.neighbors:
            self.messages.append(n.broadcast)

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

    def movement(self):
        '''
        The general movement function.
        '''
        '''
        if self.timer[1] > 0:
            self.timer = (self.timer[0], self.timer[1] - 1, 0
                          )  #number of neighbors doesn't matter yet
        else:
        '''
        if True:
            self.leader_follower()
            self.velocity[0] = self.dir_x * 0.5
            self.velocity[1] = self.dir_y * 0.5
            '''
            Leader/follower
            '''

    def find_direction(self):
        '''
        Finds the path without any obstacles (in radar range). If it is not possible, then some other direction is given

        Returned value is the direction - pair (x, y). It should be multiplied by the velocity to start moving.
        '''
        S = []
        radius = int(
            55 *
            1.4141)  #in simulation the range is not a circuit - it is a square

        for i in range(1, 15):
            S.append(100)  #initial data, for the purpouse of chain

        S.append(spot.check_x0_line(self, radius))

        chain = 0
        for i in range(14):  #data for k = 15
            si = spot.check_line(self, i + 1, 15, radius)
            if si > 0:
                S[i - 1] = si / 2
                S[i + 1] = si / 2
                chain = 0
            else:
                chain = chain + 1
            if chain >= 3:
                chain = i - 1
                break
            S[i] += si
        if S[-1]:
            S[0] += S[-1] / 2
            S[14] += S[-1] / 2
        if chain > 0:
            direction = chain
        else:
            direction = (S.index(min(S, key=abs)) + 1) % 15

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

    def leader_follower(self):
        pass
        '''
        Determines if the robot is leader or follower.
        If it is a leader - if there are no obstacles it simply goes in the known direction.
            if there are obstacles, than another direction should be calculated.
        If it is a follower - it should follow the neighbor of the same AS that is 
        the closest to the direction given by the leader.
        '''
        '''
        self.follower_msg()
        if not spot.is_follower(self):  #I am the leader

            if not self.dir_x or self.dir_y:
                self.dir_x, self.dir_y = self.find_direction()
            self.broadcast["Direction"] = (self.dir_x, self.dir_y)
            #just for dbg
            check_me = self.AS  #np.random.randint(0, 65025)
            red = check_me % 256
            green = math.floor(check_me / 4) % 256
            blue = math.floor(math.sqrt(check_me)) % 256
            color = (red, green, blue)
            pg.draw.circle(self.image, color, (self.radius, self.radius),
                           self.radius)
        else:
            spot.follower(self)
            BLACK = (0, 0, 0)
            pg.draw.circle(self.image, BLACK, (self.radius, self.radius),
                           self.radius)
            '''
