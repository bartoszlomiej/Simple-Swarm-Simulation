import sys
import numpy as np
import pygame as pg
import math

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 100, 255)
GREEN = (50, 150, 50)
PURPLE = (130, 0, 130)
GREY = (230, 230, 230)
HORRIBLE_YELLOW = (190, 175, 50)

BACKGROUND = WHITE


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

        self.timer = (-1, -1)  #tuple (AS number, timer_value)
        '''
        It is important to posses the AS number -> if it changes than timer must immediately stop!
        '''

        self.image = pg.Surface([self.radius * 2, self.radius * 2])
        self.image.fill(BACKGROUND)

        pg.draw.circle(self.image, GREEN, (self.radius, self.radius),
                       self.radius)

        self.rect = self.image.get_rect()
        self.position = np.array([x, y], dtype=np.float64)
        self.velocity = np.asarray(velocity, dtype=np.float64)
        self.moved = False

        self.state = "moving"  #initially robots move (just for aggregation algorithm)

    def update(self):
        '''
        Updates the robot position on board, as well as it's state. Necessary behaviors are to be applied here.
        '''
        self.position += self.velocity
        x, y = self.position
        self.moved = False

        self.update_msg()
        #place for the swarm behaviors
        self.aggregate()
        self.autonomus_system()

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

    def timer(self):
        '''
        just the loop (with each iteration of the simulation loop decrease by 1)
        '''


class Simulation:
    '''
    Main class of the simulation - it is responsible for generation of the board and robots. 
    Furthermore, it emulates the sensors for each robot.
    '''
    def __init__(self,
                 width=640,
                 height=400,
                 N=10,
                 s_range=40,
                 velocity_lvl=4):
        '''
        width - the width of the screen
        height - the height of the screen
        N - a swarm quantity
        s_range - sensor range in pixels (must be greater than 20)
        velocity_lvl 0 - multiplier of the velocity 
        '''
        self.width = width
        self.height = height
        self.size = (width, height)
        self.sensor_range = s_range  #(as 20 is the delimiter of the robot)
        self.velocity_lvl = velocity_lvl

        self.swarm = pg.sprite.Group()
        self.swarm_quantity = N

        self.initialize_robots()

    def initialize_robots(self):
        '''
        Initializes robots on empty board.
        '''
        for i in range(self.swarm_quantity):
            x = np.random.randint(0, self.width + 1)
            y = np.random.randint(0, self.height + 1)
            velocity = np.random.rand(2)
            velocity[0] = (velocity[0] - 0.5) * self.velocity_lvl
            velocity[1] = (velocity[1] - 0.5) * self.velocity_lvl
            '''
            TODO: Starting position could be checked here to avoid overlaping
            self.board[i] = [x, y]
            '''
            robot = Robot(x, y, self.width, self.height, velocity)
            self.swarm.add(robot)

    def check_collisions(self):
        '''
        For each robot in a swarm checks if the collision occurs. If so then the velocity is being changed accordingly.
        '''
        for robot in self.swarm:
            collision_group = pg.sprite.Group(
                [s for s in self.swarm if s != robot])
            collide = pg.sprite.spritecollide(robot, collision_group, False)
            if collide:
                #                robot.moved = True
                for c in collide:  #there can be numerous collisions however it is unlikely
                    #                    if c.moved == True:
                    #                        continue
                    self.collision_movement(robot)

    def collision_movement(self, robot):
        '''
        Robots should move in the semi-random direction after collision:
        having velocity = (x, y) -> new velocity = (-random * sign(x), -random * sign(y))
        as a result never two robots will go in the same direction after collision (no stucking)
        '''
        sign_x = np.sign(robot.velocity[0])
        sign_y = np.sign(robot.velocity[1])
        velocity = np.random.rand(2)
        velocity[0] = (velocity[0]) * self.velocity_lvl / 2  #must be positive!
        velocity[1] = (velocity[1]) * self.velocity_lvl / 2
        robot.velocity = [-sign_x * velocity[0], -sign_y * velocity[1]]
        '''
        Previous solution:
#        robot.velocity = [-robot.velocity[0], -robot.velocity[1]]
        '''

    def robot_vision(self):
        '''
        Emulates the very basic vision sensor of each robot in the swarm.
        '''
        for r in self.swarm:
            for i in self.swarm:
                if r == i:
                    continue
                if (abs(r.position[0] - i.position[0]) < self.sensor_range
                    ) and (abs(r.position[1] - i.position[1]) <
                           self.sensor_range):
                    r.spotted(i)

    def run(self):
        '''
        Runs the simulation. After certain time the simulation is closed.
        '''
        pg.init()
        screen = pg.display.set_mode([self.width, self.height])
        clock = pg.time.Clock()
        Time = 100000
        for i in range(Time):
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    sys.exit()
            self.swarm.update()
            self.check_collisions()
            self.robot_vision()
            screen.fill(BACKGROUND)
            self.swarm.draw(screen)
            pg.display.flip()
            clock.tick(100)
        pg.quit()


if __name__ == "__main__":
    '''
    Reminder:
    Simulation(width, height, N, s_range, velocity_lvl)
        width - the width of the screen
        height - the height of the screen
        N - a swarm quantity
        s_range - sensor range in pixels (must be greater than 20)
        velocity_lvl 0 - multiplier of the velocity 
    '''
    sim = Simulation(1024, 720, 25, 55, 4)
    sim.run()
    '''
    OBSERVATION:
    -There is a strong correlation between speed, p_coefficient and the number of clusters that will be created.
    -The sensor range might also have the influence on the amount of clusters that are being created.

    The higher the speed the more the system is likely to be homogeneous (MUST BE VERIFIED PROPERLY)
    '''
