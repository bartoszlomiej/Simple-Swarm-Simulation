'''
TODO:
2) Create a robot - a dot that will be able to move in any direction in the board
3) Add multiple robots - collision should occur (never transparent motion)
4)

#Game loop
while True:
    events()
    loop()
    render()

'''
import sys
import numpy as np
import pygame as pg
from pygame.locals import *

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 100, 255)
GREEN = (50, 150, 50)
PURPLE = (130, 0, 130)
GREY = (230, 230, 230)
HORRIBLE_YELLOW = (190, 175, 50)

BACKGROUND = WHITE

class Robot(pg.sprite.Sprite):
    def __init__(self, x, y, velocity = [0, 0]):
        super().__init__()
        '''
        Creates a robot on board with the initial coordinates (x, y)
        '''
        self.x = x
        self.y = y
        self.radius = 5
        self.width = 640
        self.height = 400

        self.image = pg.Surface([self.radius * 2, self.radius * 2])
        self.image.fill(BACKGROUND)
        
        pg.draw.circle(self.image, GREEN, (self.radius, self.radius), self.radius)
        
        self.rect = self.image.get_rect()
        self.position = np.array([x, y], dtype=np.float64)
        self.velocity = np.asarray(velocity, dtype=np.float64)

    def update(self):
        self.position += self.velocity
        x, y = self.position

        # Periodic boundary conditions
        if x < 0:
            self.position[0] = self.width
            x = self.width
        if x > self.width:
            self.position[0] = 0
            x = 0
        if y < 0:
            self.position[1] = self.height
            y = self.height
        if y > self.height:
            self.position[1] = 0
            y = 0

        self.rect.x = x
        self.rect.y = y

        '''
        I have no idea what the hell is this shit vel_norm
        '''
        vel_norm = np.linalg.norm(self.velocity)
        if vel_norm > 3:
            self.vel /= vel_norm
            
    def respawn(self, color, radius=5):
        return Robot(
            self.rect.x,
            self.rect.y,
            velocity=self.velocity
        )
'''
#Collsion must be done here? Or in the Simulator?
        collide = pg.sprite.spritecollide(self.swarm, self.swarm, False,
                                            pg.sprite.collide_circle)
        if collide:
            self.
'''
        


class Simulation:
    def __init__(self, width=640, height=400, N=10):
        '''
        N - is a swarm quantity
        '''
        self._width = width
        self._height = height
        self.size = (width, height)
        
        self._running = True #is it necessary?
#        self._display = None #is it necessary?

        self.swarm = pg.sprite.Group()
        self.swarm_quantity = N

        '''
        Later -> here the variables such as robot number should be placed
        '''
        
#        pygame.init()
#        self.display = pg.display.set_mode(self.size, pg.HWSURFACE | pg.DOUBLEBUF)
        #Finished here by now!

    def initialize_robots(self):
        for i in range(self.swarm_quantity):
            x = np.random.randint(0, self._width + 1)
            y = np.random.randint(0, self._height + 1)
            velocity = np.random.rand(2) * 2 - 1
            robot = Robot(x, y, velocity)
            self.swarm.add(robot)
        
    def run(self):
        pg.init()
        screen = pg.display.set_mode([self._width, self._height])
        self.initialize_robots()
        clock = pg.time.Clock()
        for i in range(1000):
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    sys.exit()
            self.swarm.update()
            self.swarm.draw(screen)
            screen.fill(BACKGROUND)
            pg.display.flip()
            clock.tick(10)
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
    sim = Simulation()
    sim.swarm_quantity = 100
    sim.run()

