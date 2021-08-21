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
import pygame, sys
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

class robot:
    def __init__(self, x, y):
        '''
        Creates a robot on board with the initial coordinates (x, y)
        '''
        self._x = x
        self._y = y

    def movement_behavior(self):
        self.cos = 1

class Simulation:
    def __init__(self, width=640, height=400):
        self._width = width
        self._height = height

        self._running = True #is it necessary?
#        self._display = None #is it necessary?
        self.size = (width, height)

        '''
        Later -> here the variables such as robot number should be placed
        '''
        
#        pygame.init()
#        self.display = pg.display.set_mode(self.size, pg.HWSURFACE | pg.DOUBLEBUF)
        #Finished here by now!
    def run(self):
        pg.init()
        screen = pg.display.set_mode([self._width, self._height])
        clock = pg.time.Clock()
        for i in range(1000):
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    sys.exit()
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
    sim.run()
