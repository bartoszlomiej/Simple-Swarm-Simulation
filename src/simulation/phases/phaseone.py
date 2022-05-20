import numpy as np
from simulation.phases.phase import Phase
import simulation.phases.phasetwo as ph2
import simulation.phases.phasethree as ph3
import simulation.phases.phasefour as ph4

import simulation.phases.attraction_point as dbg
from simulation.robot import RobotState
from simulation.robot.Timer import Timer
from simulation.robot.Velocity import Velocity
import simulation.phases.merge_clusters_to_static_line as mg
import pygame as pg


class PhaseOne(Phase):
    def __init__(self, Robot):
        super().__init__(Robot)
        self.phase = 1
        self.state = RobotState.MOVING

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

    def autonomus_system(self):
        '''
        Shows the idea of the autonomus system task allocation.
        '''
        robot = self.robot
        robot.broadcast.clear()  #clearing broadcast
        new_AS = robot.get_AS()
        if (not new_AS and robot.joined_to_cluster) or not robot.cluster_id:
            robot.create_AS()
            robot.joined_to_cluster = False
        elif new_AS:
            robot.joined_to_cluster = True
            robot.cluster_id = new_AS
            robot.broadcast['AS'] = new_AS
            robot.update_color()
        else:  #preserving the old AS
            robot.broadcast['AS'] = robot.cluster_id

    def aggregate(self):
        '''
        Simple aggregation behavior

        variable self.iterator indicates for how long should robot stay in a given state.
        In the future it might be good idea to use a number -1/other to indicate infinity

        The robots can stop iff they are in the distance of 20%-80% of the sensor range
        -the in_range_robots variable indicates the number of robots that are in that range.
        '''
        robot = self.robot
        if len(robot.neighbors) == 0 \
                and robot.state != RobotState.MOVING \
                and not robot.velocity.x \
                and not robot.velocity.y:
            '''
            If there are no neighbors -> start moving
            '''
            robot.velocity = Velocity.generateRandom(self.robot.velocity_level)
            robot.state = RobotState.MOVING

        if len(robot.neighbors) != robot.detected_robots_number:
            return  #if robots overlap, then they cannot stop.
        a = (np.random.rand(1) / 2) * robot.detected_robots_number

        p_coefficient = np.random.rand(1) * a * a
        if p_coefficient >= 0.6 and robot.state == RobotState.MOVING:
            '''
            if there are some neighbors -> the robot might stop
            '''
            if not self.minimal_distance():
                return
            robot.state = RobotState.STOPPED
            robot.velocity.x = 0
            robot.velocity.y = 0
        if robot.state == RobotState.STOPPED and p_coefficient < 0.8:
            '''
            if robot is stopped than there is possibility that robot will start moving
            '''
            robot.iterator = robot.iterator + 1
            if robot.iterator < -1:  #> 5000: #replace -1 with natural number to obtain move after stop behavior
                robot.state = RobotState.MOVINGAGAIN
                robot.iterator = 0
                robot.velocity = Velocity.generateRandom(robot.velocity_level)

        elif robot.state == RobotState.MOVINGAGAIN:
            '''
            if robot is moving after being stopped, than for some time it cannot stop again
            '''
            robot.iterator = robot.iterator + 1
            if robot.iterator > 50:
                robot.state = RobotState.MOVING
                robot.iterator = 0

    def use_timer(self):
        '''
        just the loop (with each iteration of the simulation loop decrease by 1)
        '''
        robot = self.robot
        if robot.state == RobotState.MOVING:  #only robots that are not moving can use timers
            return

        if robot.cluster_id == robot.timer.cluster_id:
            if len(
                    robot.neighbors
            ) > robot.timer.neighbors_number and robot.state != RobotState.TIMER:
                robot.state = RobotState.WAITING  #number of neighbors changed -> we are not border robot
            if robot.state != RobotState.WAITING:
                robot.timer.tick()
                if robot.timer.duration < 0:
                    robot.direction = robot.find_direction()
                    robot.broadcast["Direction"] = robot.direction.copy()
                    robot.setRandomTimer()
                    self.upgrade(1.5)
                    robot.state = RobotState.TIMER

                    return

            for m in robot.received_messages:
                if "Timer phase 1" in m.keys():
                    if robot.timer.duration > m[
                            "Timer phase 1"].duration or robot.timer.duration == -1 or robot.state != RobotState.TIMER:
                        robot.state = RobotState.TIMER
                        robot.timer = m["Timer phase 1"]
                        self.robot.follower_msg()

                        self.upgrade(1.5)
                        return

        else:
            robot.setRandomTimer()

    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] > 2:
                    self.AS = m["AS"]
                    superAS = m["superAS"]
                    self.upgrade(m["Phase"], superAS)
                    return
                elif m["Phase"] == 2:
                    self.AS = m["AS"]
                    self.upgrade(m["Phase"])
                    return

    def update(self):
        '''
        Performes all functions of the current phase in proper order.
        '''
        robot = self.robot
        aggregation_states = (RobotState.MOVING, RobotState.STOPPED,
                              RobotState.MOVINGAGAIN)
        if robot.state in aggregation_states:
            self.aggregate()  #???
        self.autonomus_system()
        self.use_timer()
        self.check_phase()

    def upgrade(self, next_phase, superAS=None):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = dbg.AttractionPoint(self.robot)
            #self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 3:
            self.robot.faza = mg.MergeClustersToStaticLine(self.robot, superAS)


class PhaseOneAndHalf(Phase):
    def __init__(self, Robot):
        super().__init__(Robot)
        self.phase = 1.5

    def use_timer(self):
        '''
        just the loop (with each iteration of the simulation loop decrease by 1)
        '''
        robot = self.robot
        if robot.state == RobotState.MOVING:  #only robots that are not moving can use timers
            return

        if robot.cluster_id == robot.timer.cluster_id:
            if len(
                    robot.neighbors
            ) > robot.timer.neighbors_number and robot.state != RobotState.TIMER:
                robot.state = RobotState.WAITING  #number of neighbors changed -> we are not border robot
            if robot.state != RobotState.WAITING:
                robot.timer.tick()

                robot.broadcast['Timer phase 1'] = robot.timer
                robot.broadcast['Direction'] = robot.direction.copy()

                if robot.timer.duration < 0:
                    self.upgrade(2)
                    return
            self.robot.follower_msg()
        else:
            robot.setRandomTimer()

    def update(self):
        self.use_timer()

    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] > 2:
                    self.AS = m["AS"]
                    superAS = m["superAS"]
                    self.upgrade(m["Phase"], superAS)
                    return
                elif m["Phase"] == 2:
                    self.AS = m["AS"]
                    self.upgrade(m["Phase"])
                    return

    def upgrade(self, next_phase, superAS=None):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = dbg.AttractionPoint(self.robot)
            #            self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 3:
            self.robot.faza = mg.MergeClustersToStaticLine(self.robot, superAS)
            #            self.robot.faza = ph3.PhaseThree(self.robot, superAS)
        elif next_phase == 4:
            self.robot.faza = ph4.PhaseFour(self.robot, superAS)
