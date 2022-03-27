import sys
import numpy as np
import pygame as pg
import math
import Simulation as sim
import SpotNeighbor as spot
import phase as ph
import phaseone as ph1
import phasethree as ph3


class PhaseTwo(ph.Phase):
    def __init__(self, Robot):
        super().__init__(Robot)
        self.phase = 2

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
                robot.dir_x, robot.dir_y = robot.find_direction()
            robot.broadcast["Direction"] = (robot.dir_x, robot.dir_y)
            #just for dbg
            check_me = robot.AS  #np.random.randint(0, 65025)
            red = check_me % 256
            green = math.floor(check_me / 4) % 256
            blue = math.floor(math.sqrt(check_me)) % 256
            color = (red, green, blue)
            pg.draw.circle(robot.image, color, (robot.radius, robot.radius),
                           robot.radius)
        else:
            spot.follower(robot)
            BLACK = (0, 0, 0)
            pg.draw.circle(robot.image, BLACK, (robot.radius, robot.radius),
                           robot.radius)

    def last_robot(self):
        '''
        Determines whether the robot is the last in the given cluster.
        '''
        a, b, d = spot.direction_line_equation(self.robot)
        for n in self.robot.neighbors:
            if n.AS == self.robot.AS:
                if not spot.neighbor_check(self.robot, n, a, b, d):
                    return False
        return True

    def update(self):
        self.collective_movement()
        robot = self.robot
        '''
        if not self.last_robot:  #dbg
            check_me = robot.AS  #np.random.randint(0, 65025)
            red = check_me % 256
            green = math.floor(check_me / 4) % 256
            blue = math.floor(math.sqrt(check_me)) % 256
            color = (red, green, blue)
            pg.draw.circle(robot.image, color, (robot.radius, robot.radius),
                           robot.radius)
        else:
            spot.follower(robot)
            BLACK = (0, 0, 0)
            pg.draw.circle(robot.image, BLACK, (robot.radius, robot.radius),
                           robot.radius)
        '''

    def upgrade(self, next_phase=3):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 3:
            self.robot.faza = ph3.PhaseThree(self.robot)
