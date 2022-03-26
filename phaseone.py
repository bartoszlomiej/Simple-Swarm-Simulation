import sys
import numpy as np
import pygame as pg
import math
import Simulation as sim
import SpotNeighbor as spot
import phase as ph
import phasetwo as ph2
import phasethree as ph3


class PhaseOne(ph.Phase):
    def __init__(self, Robot):
        super().__init__(Robot)
        self.phase = 1

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

    def autonomus_system(self):
        '''
        Shows the idea of the autonomus system task allocation.
        '''
        robot = self.robot
        robot.broadcast.clear()  #clearing broadcast
        new_AS = robot.get_AS()
        if (not new_AS and robot.joined) or not robot.AS:
            robot.create_AS()
            robot.joined = False
        elif new_AS:
            robot.joined = True
            robot.AS = new_AS
            robot.broadcast['AS'] = new_AS
            robot.update_color()
        else:  #preserving the old AS
            robot.broadcast['AS'] = robot.AS

    def aggregate(self):
        '''
        Simple aggregation behavior

        variable self.iterator indicates for how long should robot stay in a given state.
        In the future it might be good idea to use a number -1/other to indicate infinity

        The robots can stop iff they are in the distance of 20%-80% of the sensor range
        -the in_range_robots variable indicates the number of robots that are in that range.
        '''
        robot = self.robot
        if len(robot.neighbors
               ) == 0 and robot.state != "moving" and not robot.velocity[
                   0] and not robot.velocity[1]:
            '''
            If there are no neighbors -> start moving
            '''
            robot.velocity = np.random.rand(2)
            robot.velocity[0] = (robot.velocity[0] - 0.5) * 4
            robot.velocity[1] = (robot.velocity[1] - 0.5) * 4  #start moving
            robot.state = "moving"

        if len(robot.neighbors) != robot.in_range_robots:
            return  #if robots overlap, then they cannot stop.
        a = (np.random.rand(1) / 2) * robot.in_range_robots

        p_coefficient = np.random.rand(1) * a * a
        if p_coefficient >= 0.6 and robot.state == "moving":
            '''
            if there are some neighbors -> the robot might stop
            '''
            if not self.minimal_distance():
                return
            robot.state = "stopped"
            robot.velocity[0] = 0
            robot.velocity[1] = 0
        if robot.state == "stopped" and p_coefficient < 0.8:
            '''
            if robot is stopped than there is possibility that robot will start moving
            '''
            robot.iterator = robot.iterator + 1
            if robot.iterator < -1:  #> 5000: #replace -1 with natural number to obtain move after stop behavior
                robot.state = "moving after stopped"
                robot.iterator = 0
                robot.velocity = np.random.rand(2)
                robot.velocity[0] = (robot.velocity[0] - 0.5) * 4
                robot.velocity[1] = (robot.velocity[1] -
                                     0.5) * 4  #start moving

        elif robot.state == "moving after stopped":
            '''
            if robot is moving after being stopped, than for some time it cannot stop again
            '''
            robot.iterator = robot.iterator + 1
            if robot.iterator > 50:
                robot.state = "moving"
                robot.iterator = 0

    def use_timer(self):
        '''
        just the loop (with each iteration of the simulation loop decrease by 1)
        '''
        robot = self.robot
        if robot.state == "moving":  #only robots that are not moving can use timers
            return

        if robot.AS == robot.timer[0]:
            if len(robot.neighbors
                   ) > robot.timer[2] and robot.state != "Timer phase 1":
                robot.state = "waiting"  #number of neighbors changed -> we are not border robot
            if robot.state != "waiting":
                robot.timer = (robot.timer[0], robot.timer[1] - 1,
                               robot.timer[2])

                if robot.phase == 1.5:
                    robot.broadcast['Timer phase 1'] = robot.timer
                    robot.broadcast['Initial Direction'] = (robot.dir_x,
                                                            robot.dir_y)

                if robot.timer[1] < 0:
                    if robot.phase == 1:  #here enters only the edge robots
                        robot.dir_x, robot.dir_y = robot.find_direction()
                        robot.set_timer(
                        )  #The second timer is to be set -> it will be used for synchronization
                        robot.phase = 1.5
                        robot.faza.upgrade(1.5)
                        robot.state = "Timer phase 1"
                        HORRIBLE_YELLOW = (190, 175, 50)
                        pg.draw.circle(robot.image, HORRIBLE_YELLOW,
                                       (robot.radius, robot.radius),
                                       robot.radius)
                        return

                    robot.phase = 2  #finally, going to phase 2!!!
                    robot.faza.upgrade(2)
                    return

            for m in robot.messages:
                if "Timer phase 1" in m.keys():
                    if robot.timer[1] > m["Timer phase 1"][1] or robot.timer[
                            1] == -1 or robot.state != "Timer phase 1":
                        robot.state = "Timer phase 1"
                        robot.timer = m["Timer phase 1"]
                        robot.phase = 1.5  #as we get the other timer, thus the second timer is to be used
                if "Initial Direction" in m.keys():
                    robot.dir_x = m["Initial Direction"][0]
                    robot.dir_y = m["Initial Direction"][1]
        else:
            robot.set_timer()

    def update(self):
        '''
        Performes all functions of the current phase in proper order.
        '''
        robot = self.robot
        aggregation_states = ("moving", "stopped", "moving after stopped")
        if robot.state in aggregation_states:
            self.aggregate()  #???
        self.autonomus_system()
        self.use_timer()

    def upgrade(self, next_phase):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 3:
            self.robot.faza = ph3.PhaseThree(self.robot)


class PhaseOneAndHalf(ph.Phase):
    def __init__(self, Robot):
        super().__init__(Robot)
        self.phase = 1.5

    def use_timer(self):
        '''
        just the loop (with each iteration of the simulation loop decrease by 1)
        '''
        robot = self.robot
        if robot.state == "moving":  #only robots that are not moving can use timers
            return

        if robot.AS == robot.timer[0]:
            if len(robot.neighbors
                   ) > robot.timer[2] and robot.state != "Timer phase 1":
                robot.state = "waiting"  #number of neighbors changed -> we are not border robot
            if robot.state != "waiting":
                robot.timer = (robot.timer[0], robot.timer[1] - 1,
                               robot.timer[2])

                robot.broadcast['Timer phase 1'] = robot.timer
                robot.broadcast['Initial Direction'] = (robot.dir_x,
                                                        robot.dir_y)

                if robot.timer[1] < 0:
                    robot.phase = 2  #finally, going to phase 2!!!
                    robot.faza.upgrade(2)
                    return

            for m in robot.messages:
                if "Timer phase 1" in m.keys():
                    if robot.timer[1] > m["Timer phase 1"][1] or robot.timer[
                            1] == -1 or robot.state != "Timer phase 1":
                        robot.state = "Timer phase 1"
                        robot.timer = m["Timer phase 1"]
                        robot.phase = 1.5  #as we get the other timer, thus the second timer is to be used
                if "Initial Direction" in m.keys():
                    robot.dir_x = m["Initial Direction"][0]
                    robot.dir_y = m["Initial Direction"][1]
        else:
            robot.set_timer()

    def update(self):
        self.use_timer()

    def upgrade(self, next_phase):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 3:
            self.robot.faza = ph3.PhaseThree(self.robot)
