import pygame as pg
import math
from utils import SpotNeighbor as spot
from simulation.robot import RobotState
from simulation.robot.Velocity import Velocity


class StaticBehaviors():
    def __init__(self, robot):
        self.robot = robot

        
    def distanceToNeighbor(self, neighbor):
        distance = spot.relative_distance(self.robot.position.x, \
                                               self.robot.position.y, \
                                               neighbor.position.x, \
                                               neighbor.position.y)
        return distance

    
    def minimalDistance(self, distance_factor):
        minimal_distance = distance_factor * (self.robot.sensor_range - self.robot.radius) \
            + self.robot.radius
        return minimal_distance
