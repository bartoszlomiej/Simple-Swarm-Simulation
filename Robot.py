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

        self.state = "moving"  #initially robots move (just for aggregation algorithm)
        self.phase = 1  #the first phase is being currently run. This number can be only incremented!

        self.prev_coords = []  #previous coordinates of the neighbors

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
            self.collective_movement()

        #boundary parameters
        if x < 0 or x > self.width - 2 * self.radius:
            self.velocity[0] = -self.velocity[0]
        if y < 0 or y > self.height - 2 * self.radius:
            self.velocity[1] = -self.velocity[1]

        self.neighbors.clear(
        )  #list of neighbors must be refreshed in each update
        self.in_range_robots = 0

        self.rect.x = x
        self.rect.y = y

    def spotted(self, r):
        '''
        Adds the newly spotted neighbor to the list.
        '''
        self.neighbors.append(r)

    def in_range(self):
        self.in_range_robots = self.in_range_robots + 1

    def aggregate(self):
        '''
        Simple aggregation behavior

        variable self.iterator indicates for how long should robot stay in a given state.
        In the future it might be good idea to use a number -1/other to indicate infinity

        The robots can stop iff they are in the distance of 20%-80% of the sensor range
        -the in_range_robots variable indicates the number of robots that are in that range.
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

        a = (
            np.random.rand(1) / 2
        ) * self.in_range_robots  #bad idea -> the clusters are mismatching!
        #remove this in the Simulation.py as well!
        #        a = (np.random.rand(1) / 2) * (len(self.neighbors))
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
                        )  #The second timer is to be set -> it will be used for synchronization
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

    def collective_movement(self):
        direction, velocity = self.movement_detection()
        if not velocity:  # or not self.state == "moving":
            self.velocity[0] = 0.1
            self.velocity[1] = 0.1
        else:
            self.velocity[0] = (velocity * direction[0]) / (
                math.sqrt(direction[0]**2 + direction[1]**2))
            self.velocity[1] = (velocity * direction[1]) / (
                math.sqrt(direction[0]**2 + direction[1]**2))


#        print("V_x: ", self.velocity[0], "V_y: ", self.velocity[1])
        self.state = "moving"

    def find_direction(self):
        '''
        Finds the path without any obstacles (in radar range). If it is not possible, then some other direction is given

        Returned value is the direction - pair (x, y). It should be multiplied by the velocity to start moving.
        '''
        S = []
        S.append(check_x0_line(self))
        for i in range(1, 12, 1):  #data for k = 13
            si = check_line(self, i)
            S.append(si)
        direction = S.index(min(S))
        if direction == 0:
            return (0, 1)
        return (1, calc_y(direction, 1))

    def leader_follower(self):
        '''
        If robot doesn't have any neighbors, of the same AS, in the given direction, then this robot becomes a leader

        The leader can change the direction, however, after the change of the direction the leader might loose the leadership
        
        Otherwise, the robot becomes a follower. The follower is chasing the robot in front of it (in the given direction); it must keep the distance between over robots


        What's need to be done?
        -Function "find_direction" - is needed to find the direction to which the robots would like to go
        -Change in timers -> the direction that was found must be given as well. If Robot is close to the border, 
        then direction should be changed (e.g. parallel to that direction)
        -leader_follower - must be done.
        -certain changes should be done in the function "movement_detection"
        '''

        pass

    def movement_detection(self):
        coords = []
        for n in self.neighbors:
            coords.append((n.x, n.y))
        if not self.prev_coords:
            self.prev_coords = coords
        else:
            if len(self.prev_coords) != len(coords):
                print("Prev: ", self.prev_coords, "Current: ", coords,
                      "my coords: ", (self.x, self.y))
                #                print("Houston, we've got a problem here:(", self.AS)
                return (0, 0), 0  #robot is unsure about the direction
            else:
                '''
                Good idea -> use some uncertainty - if the coordinates differ of more than given value/percent than this is not the same robot!
                '''
                for i in range(len(coords)):
                    if coords[i] != self.prev_coords[i]:
                        direction = (coords[i][0] - self.x,
                                     coords[i][1] - self.y)
                        velocity = math.sqrt(
                            (coords[i][0] - self.prev_coords[i][0])**2 +
                            (coords[i][1] - self.prev_coords[i][1])**2)
                        #the velocity value (not direction) is in fact the distance travelled. there is used Pythagorean thm to calculate it
                        return direction, velocity
            self.prev_coords = coords
        return (0, 0), 0
'''
Major issues so far:
-Robot should chase the leader! This means that the algorithm should look differently.
// I was trying to do so:( still some work needs to be donep
***THINK ABOUT THAT***
'''
