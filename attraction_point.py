import sys
import numpy as np
import pygame as pg
import math
import Simulation as sim
import SpotNeighbor as spot
import phase as ph
import phaseone as ph1
import phasethree as ph3


class AttractionPoint(ph.Phase):
    '''
    Class alternative to PhaseTwo. It is used only for testing purposes of the attraction point.
    '''
    def __init__(self, Robot):
        super().__init__(Robot)
        self.phase = 2
        self.next_phase = False
        Robot.clear_broadcast()
        Robot.initialize_sensors()
        
    def collective_movement(self):
        '''
        Transition from the state "stopped" to "moving" and runing the movement function
        #to be removed in the future.
        '''
        robot = self.robot
        if robot.state != "moving":
            self.initial_direction()
        else:
            self.movement()

    def movement(self):
        '''
        The general movement function.
        '''
        robot = self.robot
        self.leader_follower()
        robot.velocity[0] = robot.dir_x  # * 0.5
        robot.velocity[1] = robot.dir_y  # * 0.5
        '''
        Leader/follower
        '''

    def initial_direction(self):
        '''
        Sets the initial direction for all robots in the AS
        '''
        robot = self.robot
        robot.state = "moving"
        robot.velocity[0] = robot.dir_x
        robot.velocity[1] = robot.dir_y

        self.leader_follower()

    def leader_follower(self):
        '''
        Determines if the robot is leader or follower.
        If it is a leader - if there are no obstacles it simply goes in the known direction.
            if there are obstacles, than another direction should be calculated.
        If it is a follower - it should follow the neighbor of the same AS that is 
        the closest to the direction given by the leader.
        '''
        robot = self.robot

        robot.follower_msg()
        if not spot.is_follower(robot):  #I am the leader
            '''
            Simply goes in the given direction
            '''
            if spot.is_collision(robot):
                robot.dir_x, robot.dir_y =  robot.find_direction()
            else:
                robot.dir_x, robot.dir_y = self.__attract()
            robot.broadcast["Direction"] = (robot.dir_x, robot.dir_y)
        else:
            spot.follower(robot)

    def __attract(self):
        '''
        biases the direction we follow so as to get to the given point
        
        1) Calculate direction to the direction point
        2) Bias the current direction that robot follows
        '''
        ap = self.__direction_to_attraction_point()
        
        v1 = self.robot.dir_x + ap[0] * ap[2]
        v2 = self.robot.dir_y + ap[1] * ap[2]
        
        rescale = math.sqrt(v1**2 + v2**2)
        v1 /= rescale
        v2 /= rescale            

        return v1, v2
    
    def __direction_to_attraction_point(self):
        '''
        Change the Robot direction to approach the given neighbor.

        The dir_x, dir_y values assumes that the direction is always given on the unit circle.

        The closer the robot is to the attraction point the higher is the attraction value.
        '''
        
        delta_x = (self.robot.ap[0] - self.robot.x)
        delta_y = (self.robot.ap[1] - self.robot.y)

        if not delta_x and not delta_y: #robot catched the attraction point
            return 0, 0, 0
        
        suma = math.sqrt(delta_x**2 + delta_y**2)
        dir_x = delta_x / suma
        dir_y = delta_y / suma
        
        attraction_value = 1 - (spot.relative_distance(self.robot.x, self.robot.y, self.robot.ap[0], self.robot.ap[1])**2)/self.robot.ap[2]
        ap_val = 0.00001 if attraction_value < 0 else attraction_value

        if dir_x == 0 and dir_y == 0:
            print("here we have an error!!")
        
        return (dir_x, dir_y, ap_val)

            
    def minimal_distance(self):
        '''
        Checks if the minimal distance between robots is being kept.
        Returns true if minimal distance is being kept; otherwise returns false.
        '''
        robot = self.robot
        for n in robot.neighbors:
            if (abs(n.position[0] - robot.position[0]) <= robot.radius) and (
                    abs(n.position[1] - robot.position[1]) <= robot.radius):
                return False
        return True

    def isPhaseUpgrade(self):
        '''
        If phase should be upgraded then upgrade it (leader only)
        '''
        delta = 40
        x, y = self.robot.position
        if (x - self.robot.ap[0])**2 + (y - self.robot.ap[1])**2 <= delta**2:
            self.robot.faza.upgrade(3, self.robot.AS)
            self.robot.broadcast["superAS"] = self.robot.AS

    def check_phase(self):
        robot = self.robot
        for m in robot.messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 3:
                    superAS = m["superAS"]
                    robot.broadcast["superAS"] = superAS
                    self.upgrade(m["Phase"], superAS)
                    return

    def update(self):
        self.collective_movement()
        robot = self.robot
        '''
        if not self.minimal_distance():
            #robot.dir_x, robot.dir_y = 0, 0
            robot.dir_x, robot.dir_y = robot.find_direction()
        '''
        robot.is_allone()
        self.check_phase()
        self.isPhaseUpgrade()


    def upgrade(self, next_phase=3, superAS=None):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 3:
            self.robot.faza = ph3.PhaseThree(self.robot, superAS)
