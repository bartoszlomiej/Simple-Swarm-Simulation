import sys
import numpy as np
import pygame as pg
import math
import Simulation as sim
import SpotNeighbor as spot
from abc import ABC, abstractmethod


class Phase(ABC):
    '''
    The Phase is an abstract base class.
    It's subclasses will perform the specific operations needed for different phases.
    '''
    def __init__(self, Robot):
        self.robot = Robot
        
    @abstractmethod
    def check_phase(self):
        '''
        If robot in the same AS have higher phase, then we update ours
        '''
        pass
    @abstractmethod
    def upgrade(self, next_phase):
        '''
        Upgrades the phase to further one.
        '''
        pass

    @abstractmethod
    def update(self):
        '''
        Performes all operations in the given phase
        '''
        pass
