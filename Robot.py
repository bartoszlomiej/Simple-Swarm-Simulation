import sys
import numpy as np
import pygame as pg
import math
import Simulation as sim


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

        self.state = "moving"  #initially robots move (just for aggregation algorithm)
        self.phase = 1  #the first phase is being currently run. This number can be only incremented!

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
            aggregation_states = ("moving", "stopped", "moving after stopped")
            if self.state in aggregation_states:
                self.aggregate()
            self.autonomus_system()
            self.use_timer()
        elif self.phase == 1.5:
            self.use_timer()
        elif self.phase == 2:
            pass

        #boundary parameters
        if x < 0 or x > self.width - 2 * self.radius:
            self.velocity[0] = -self.velocity[0]
        if y < 0 or y > self.height - 2 * self.radius:
            self.velocity[1] = -self.velocity[1]

        self.neighbors.clear(
        )  #list of neighbors must be refreshed in each update

        self.rect.x = x
        self.rect.y = y

    def spotted(self, r):
        '''
        Adds the newly spotted neighbor to the list.
        '''
        self.neighbors.append(r)

    def aggregate(self):
        '''
        Simple aggregation behavior

        variable self.iterator indicates for how long should robot stay in a given state.
        In the future it might be good idea to use a number -1/other to indicate infinity
        '''
        if len(self.neighbors
               ) == 0 and self.state != "moving" and not self.velocity[
                   0] and not self.velocity[1]:
            '''
            If there are no neighbors -> start moving
            '''
            self.velocity = np.random.rand(2)
            self.velocity[0] = (self.velocity[0] - 0.5) * 4
            self.velocity[1] = (self.velocity[1] - 0.5) * 4  #start moving
            self.state = "moving"
        a = (np.random.rand(1) / 2) * (len(self.neighbors))
        p_coefficient = np.random.rand(1) * a * a
        if p_coefficient >= 0.6 and self.state == "moving":
            '''
            if there are some neighbors -> the robot might stop
            '''
            self.state = "stopped"
            self.velocity[0] = 0
            self.velocity[1] = 0
        if self.state == "stopped" and p_coefficient < 0.8:
            '''
            if robot is stopped than there is possibility that robot will start moving
            '''
            self.iterator = self.iterator + 1
            if self.iterator < -1:  #> 5000: #replace -1 with natural number to obtain move after stop behavior
                self.state = "moving after stopped"
                self.iterator = 0
                self.velocity = np.random.rand(2)
                self.velocity[0] = (self.velocity[0] - 0.5) * 4
                self.velocity[1] = (self.velocity[1] - 0.5) * 4  #start moving

        elif self.state == "moving after stopped":
            '''
            if robot is moving after being stopped, than for some time it cannot stop again
            '''
            self.iterator = self.iterator + 1
            if self.iterator > 50:
                self.state = "moving"
                self.iterator = 0

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

    def autonomus_system(self):
        self.broadcast.clear()  #clearing broadcast
        '''
        Shows the idea of the autonomus system task allocation
        '''
        new_AS = self.get_AS()
        if (not new_AS and self.joined) or not self.AS:
            self.create_AS()
            self.joined = False
        elif new_AS:
            self.joined = True
            self.AS = new_AS
            self.broadcast['AS'] = new_AS
            self.update_color()
        else:  #preserving the old AS
            self.broadcast['AS'] = self.AS

    def use_timer(self):
        '''
        just the loop (with each iteration of the simulation loop decrease by 1)
        '''
        if self.state == "moving":  #only robots that are not moving can use timers
            return

        if self.AS == self.timer[0]:
            if len(self.neighbors
                   ) > self.timer[2] and self.state != "Timer phase 1":
                self.state = "waiting"  #number of neighbors changed -> we are not border robot
            if self.state != "waiting":
                self.timer = (self.timer[0], self.timer[1] - 1, self.timer[2])

                if self.phase == 1.5:
                    self.broadcast['Timer phase 1'] = self.timer

                if self.timer[1] < 0:
                    if self.phase == 1:
                        self.set_timer(
                        )  #The second timer is to be set -> it will be used for synchronization\
                        self.state = "Timer phase 1"
                        self.phase = 1.5
                        return
                    self.phase = 2  #finally, going to phase 2!!!
                    #just for dbg
                    check_me = self.AS + 2000  #np.random.randint(0, 65025)
                    red = check_me % 256
                    green = math.floor(check_me / 4) % 256
                    blue = math.floor(math.sqrt(check_me)) % 256
                    color = (red, green, blue)
                    pg.draw.circle(self.image, color,
                                   (self.radius, self.radius), self.radius)
                    return

            for m in self.messages:
                if "Timer phase 1" in m.keys():
                    self.state = "Timer phase 1"
                    if self.timer[1] > m["Timer phase 1"][1] or self.timer[
                            1] == -1:
                        self.timer = m["Timer phase 1"]
                        self.phase = 1.5  #as we get the other timer, thus the second timer is to be used
        else:
            self.set_timer()

    def set_timer(self):
        self.timer = (self.AS, np.random.randint(1000,
                                                 2000), len(self.neighbors))
