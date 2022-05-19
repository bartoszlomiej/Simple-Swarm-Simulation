import math

from simulation.robot import RobotState
from utils import SpotNeighbor as spot
from simulation.phases.phase import Phase
from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction

import simulation.phases.phaseone as ph1
import simulation.phases.phasethree as ph3
import simulation.phases.static_line_formation as st
import simulation.phases.merge_clusters_to_static_line as mg

from utils.colors import GREY
import pygame as pg

class AttractionPoint(Phase):
    '''
    Class alternative to PhaseTwo. It is used only for testing purposes of the attraction point.
    '''
    def __init__(self, Robot):
        super().__init__(Robot)
        self.phase = 2
        self.next_phase = False
        Robot.clear_broadcast()
        Robot.initialize_sensors()
        self.robot.velocity_level /= 2  #just for dbg

    def collective_movement(self):
        '''
        Transition from the state RobotState.STOPPED to RobotState.MOVING and runing the movement function
        #to be removed in the future.
        '''
        robot = self.robot
        if robot.state != RobotState.MOVING:
            self.initial_direction()
        else:
            self.movement()

    def movement(self):
        '''
        The general movement function.
        '''
        self.robot.velocity = Velocity(0, 0)
        self.leader_follower()
        self.__moveIfPathIsFree()
        '''
        Leader/follower
        '''

    def initial_direction(self):
        '''
        Sets the initial direction for all robots in the AS
        '''
        self.__moveIfPathIsFree()
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
            pg.draw.circle(self.robot.image, GREY,
                           (self.robot.radius, self.robot.radius),
                           self.robot.radius)
            if spot.is_any_collision(robot, 0.4):
                robot.direction = robot.find_direction()
            else:
                robot.direction = self.__attract()
            robot.broadcast["Direction"] = robot.direction
        else:
            self.robot.update_color()
            spot.follower(robot)

    def __attract(self):
        '''
        biases the direction we follow so as to get to the given point
        
        1) Calculate direction to the direction point
        2) Bias the current direction that robot follows
        '''
        ap = self.__direction_to_attraction_point()

        v1 = self.robot.direction.x + ap[0] * ap[2]
        v2 = self.robot.direction.y + ap[1] * ap[2]

        rescale = math.sqrt(v1**2 + v2**2)
        v1 /= rescale
        v2 /= rescale

        return Direction(v1, v2)

    def __direction_to_attraction_point(self):
        '''
        Change the Robot direction to approach the given neighbor.

        The dir_x, dir_y values assumes that the direction is always given on the unit circle.

        The closer the robot is to the attraction point the higher is the attraction value.
        '''

        delta_x = (self.robot.ap[0] - self.robot.position.x)
        delta_y = (self.robot.ap[1] - self.robot.position.y)

        if not delta_x and not delta_y:  #robot catched the attraction point
            return 0, 0, 0

        suma = math.sqrt(delta_x**2 + delta_y**2)
        dir_x = delta_x / suma
        dir_y = delta_y / suma

        attraction_value = 1 - (spot.relative_distance(
            self.robot.position.x, self.robot.position.y, self.robot.ap[0],
            self.robot.ap[1])**2) / self.robot.ap[2]
        ap_val = 0.00001 if attraction_value < 0 else attraction_value

        return (dir_x, dir_y, ap_val)

    def minimal_distance(self):
        '''
        Checks if the minimal distance between robots is being kept.
        Returns true if minimal distance is being kept; otherwise returns false.
        '''
        robot = self.robot
        for n in robot.neighbors:
            if (abs(n.position.x - robot.position.x) <= robot.radius) and (
                    abs(n.position.y - robot.position.y) <= robot.radius):
                return False
        return True

    def isPhaseUpgrade(self):
        '''
        If phase should be upgraded then upgrade it (leader only)
        '''
        delta = 10
        if (self.robot.position.x - self.robot.ap[0])**2 + (
                self.robot.position.y - self.robot.ap[1])**2 <= delta**2:
            self.upgrade(3, self.robot.cluster_id)
            self.robot.broadcast["superAS"] = self.robot.cluster_id

    def __followerStoppingCondition(self):
        '''
        Followers should not enter the higher phase if they move
        (in order to get rid of annoying empty spaces between single AS)
        '''
        if spot.is_follower(self.robot):
            if spot.is_any_collision(self.robot, 0.3):
                return True
            return False
        return True

    def __moveIfPathIsFree(self):
        if not spot.is_any_collision(self.robot, 0.2):
            self.makeMove()

    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 3 and self.__followerStoppingCondition():
                    #if m["Phase"] >= 3: #unlock to make phase 3 work
                    superAS = m["superAS"]
                    robot.broadcast["superAS"] = superAS
                    self.upgrade(m["Phase"], superAS)
                    return

    def update(self):
        self.collective_movement()
        robot = self.robot
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
            if self.robot.cluster_id == superAS:
                self.robot.faza = st.StaticLineFormation(self.robot, superAS)
            else:
                self.robot.faza = mg.MergeClustersToStaticLine(
                    self.robot, superAS)
            #            self.robot.faza = ph3.PhaseThree(self.robot, superAS)
