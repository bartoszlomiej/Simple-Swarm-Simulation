'''
TODO:
1. Autonomus System task allocation:

  [*] -> First do prerequirements
  [-] -> Further apply the algorithm:
    [-] -> Ask for AS
    [-] -> Create AS
      [-] -> Color the robot with the random color
    [-] -> Get AS of closest neighbor
      [-] -> Get the color of the neighbor
    [-] -> Propagate AS
    [-] -> Check neighbors
      [-] -> If the AS nearby have the same color -> change it within AS
  [-] -> Reduce the speed of robots to see the results (good idea -> when robot enters the AS then decrease it's speed significantly)

2. Improve aggregation:
 [-] -> current problem is that none of the robots start moving after it was once stopped

Legend:
[-] -> the task is pending
[*] -> the task is done
'''
import sys
import numpy as np
import pygame as pg

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
    def __init__(self, x, y, width, height, velocity = [0, 0]):
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
        self.messages = [] #obtained messages from other robots
        self.broadcast = {} #messages to be broadcast
        '''
        Broadcast messages are the dictionary -> {msg_type : msg_value}
        Messages are the list of gather broadcasts from the neighbors
        It is assumed that it doesn't matter which robot place the massage
        '''
        self.AS = None
        
        self.image = pg.Surface([self.radius * 2, self.radius * 2])
        self.image.fill(BACKGROUND)
        
        pg.draw.circle(self.image, GREEN, (self.radius, self.radius), self.radius)
        
        self.rect = self.image.get_rect()
        self.position = np.array([x, y], dtype=np.float64)
        self.velocity = np.asarray(velocity, dtype=np.float64)
        self.moved = False

        self.state = "moving" #initially robots move (just for aggregation algorithm)

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
        if y < 0 or y > self.height - 2 *self.radius:
            self.velocity[1] = -self.velocity[1]
            
        self.neighbors.clear() #list of neighbors must be refreshed in each update

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
        '''
        if len(self.neighbors) == 0 and self.state != "moving":
            velocity = np.random.rand(2)
            velocity[0] = (velocity[0] - 0.5) * 4
            velocity[1] = (velocity[1] - 0.5) * 4  #start moving
        a =  (np.random.rand(1)/2) * (len(self.neighbors))
        p_coefficient = np.random.rand(1) * a * a
        if p_coefficient >= 0.75 and self.state == "moving":
            self.state = "stopped"
            self.velocity[0] = 0
            self.velocity[1] = 0
        if self.state == "stopped" and p_coefficient < 0.75:
            velocity = np.random.rand(2)
            velocity[0] = (velocity[0] - 0.5) * 4
            velocity[1] = (velocity[1] - 0.5) * 4  #start moving
#            self.state = "going to move"
            self.state = "moving"
        '''
        if self.state == "going to move":
            self.iterator = self.iterator + 1
            if self.iterator > 500:
                self.state = "moving"
                self.iterator = 0
                print("Am I here?")
        '''
    def update_msg(self):
        self.messages.clear()
        for n in self.neighbors:
            self.messages.append(n.broadcast)

    def update_color(self):
        color = (self.AS, self.AS, self.AS)
        pg.draw.circle(self.image, color, (self.radius, self.radius), self.radius)
            
    def create_AS(self):
        self.AS = np.random.randint(0, 255) #arbitrary number to be changed in the future
        self.broadcast['AS'] = self.AS
        self.update_color()

    def get_AS(self):
        votes = {}
        remain = False
        for m in self.messages:
            if 'AS' in m.keys():
                if not m['AS'] in votes:
                    votes[m['AS']] = 1
                    if m['AS'] == self.AS: #don't change AS if you can still spot your colleagues
                        remain = True
                else:
                    votes[m['AS']] = votes[m['AS']] + 1
        if not votes: #or remain:
            return None
        k=list(votes.keys())
        v=list(votes.values())
        return k[v.index(max(v))] #getting the AS that is most common nearby
            
    def autonomus_system(self):
        self.broadcast.clear() #clearing broadcast
        '''
            Shows the idea of the autonomus system task allocation
            '''
        new_AS = self.get_AS()
        if not new_AS and not self.AS:
            self.create_AS()
        elif new_AS:
            self.AS = new_AS
            self.broadcast['AS'] = new_AS
            self.update_color()
        else: #preserving the old AS
            self.broadcast['AS'] = self.AS
            


class Simulation:
    def __init__(self, width=640, height=400, N=10, s_range = 40):
        '''
        width - the width of the screen
        height - the height of the screen
        N - a swarm quantity
        s_range - sensor range in pixels (must be greater than 20)
        '''
        self.width = width
        self.height = height
        self.size = (width, height)
        self.sensor_range = s_range #(as 20 is the delimiter of the robot)
        
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
            velocity[0] = (velocity[0] - 0.5) * 4
            velocity[1] = (velocity[1] - 0.5) * 4

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
            collision_group = pg.sprite.Group([s for s in self.swarm if s != robot])
            collide = pg.sprite.spritecollide(robot, collision_group, False) 
            if collide:
                self.moved = True
                for c in collide: #there can be numerous collisions however it is unlikely
                    if c.moved == True:
                        continue
                    robot.velocity = [-robot.velocity[0], -robot.velocity[1]]
                    
    def robot_vision(self):
        '''
        Emulates the very basic vision sensor of each robot in the swarm.
        '''
        for r in self.swarm:
            for i in self.swarm:
                if r == i:
                    continue
                if (abs(r.position[0] - i.position[0]) < self.sensor_range) and (
                        abs(r.position[1] - i.position[1]) < self.sensor_range):
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
            clock.tick(30)
        pg.quit()


'''
class App:
    def __init__(self):
        self._running = True
        self._display_surf = None
        self.size = self.weight, self.height = 640, 400
 
    def on_init(self):
        pygame.init()
        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self._running = True
 
    def on_event(self, event):
        if event.type == pygame.QUIT:
            self._running = False
    def on_loop(self):
        pass
    def on_render(self):
        pass
    def on_cleanup(self):
        pygame.quit()
 
    def on_execute(self):
        if self.on_init() == False:
            self._running = False
 
        while( self._running ):
            for event in pygame.event.get():
                self.on_event(event)
            self.on_loop()
            self.on_render()
        self.on_cleanup()
'''
 
if __name__ == "__main__" :
    sim = Simulation(1024, 720, 25, 60)
    sim.run()
