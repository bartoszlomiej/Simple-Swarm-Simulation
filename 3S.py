'''
TODO:
3) collision:
-There should be an invisible frame of the screen - the robot should bounce from it
4) randomness in motion
5) robot "sensors"
6) robot behaviors
7) tweaking

#Game loop
while True:
    events()
    loop()
    render()

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
    def __init__(self, x, y, width, height, velocity = [0, 0]):
        super().__init__()
        '''
        Creates a robot on board with the initial coordinates (x, y)
        '''
        self.x = x
        self.y = y
        self.radius = 10
        self._width = width
        self._height = height

        self.image = pg.Surface([self.radius * 2, self.radius * 2])
        self.image.fill(BACKGROUND)
        
        pg.draw.circle(self.image, GREEN, (self.radius, self.radius), self.radius)
        
        self.rect = self.image.get_rect()
        self.position = np.array([x, y], dtype=np.float64)
        self.velocity = np.asarray(velocity, dtype=np.float64)
        self.moved = False

    def update(self):
        self.position += self.velocity
        x, y = self.position
        self.moved = False

        #boundary parameters
        if x < 0 or x > self._width - 2 * self.radius:
            self.velocity[0] = -self.velocity[0]
        if y < 0 or y > self._height - 2 *self.radius:
            self.velocity[1] = -self.velocity[1]

        self.rect.x = x
        self.rect.y = y

class Simulation:
    def __init__(self, width=640, height=400, N=10):
        '''
        N - is a swarm quantity
        '''
        self._width = width
        self._height = height
        self.size = (width, height)
        
        self.swarm = pg.sprite.Group()
        self.swarm_quantity = N

        self.initialize_robots()

        '''
        Later -> here the variables such as robot number should be placed
        '''

    def initialize_robots(self):
        for i in range(self.swarm_quantity):
            x = np.random.randint(0, self._width + 1)
            y = np.random.randint(0, self._height + 1)
            velocity = np.random.rand(2) * 2 - 1
            robot = Robot(x, y, self._width, self._height, velocity)
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
        
    def run(self):
        pg.init()
        screen = pg.display.set_mode([self._width, self._height])
        clock = pg.time.Clock()
        for i in range(1000):
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    sys.exit()
            self.swarm.update()
            self.check_collisions()
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
    sim = Simulation(600, 400, 10)
    sim.run()

