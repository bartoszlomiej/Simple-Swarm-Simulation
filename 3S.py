import sys
import numpy as np
import pygame as pg
import math
import Simulation

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 100, 255)
GREEN = (50, 150, 50)
PURPLE = (130, 0, 130)
GREY = (230, 230, 230)
HORRIBLE_YELLOW = (190, 175, 50)

BACKGROUND = WHITE

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
    sim = Simulation.Simulation(1024, 720, 40, 55, 4)
    sim.run()
    '''
    OBSERVATION:
    -There is a strong correlation between speed, p_coefficient and the number of clusters that will be created.
    -The sensor range might also have the influence on the amount of clusters that are being created.

    The higher the speed the more the system is likely to be homogeneous (MUST BE VERIFIED PROPERLY)
    '''
