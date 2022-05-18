import pygame as pg
import math
from utils import SpotNeighbor as spot
from simulation.robot import RobotState
from simulation.robot.Velocity import Velocity


class MoveBehaviors():
    def __init__(self, robot):
        self.robot = robot

    def makeMove(self):
        while (self.robot.dir_x**2 + self.robot.dir_y**2) > 1:
            self.robot.dir_x /= 2
            self.robot.dir_y /= 2
        self.robot.velocity.x = self.robot.dir_x * self.robot.velocity_level
        self.robot.velocity.y = self.robot.dir_y * self.robot.velocity_level        
