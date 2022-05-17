import pygame as pg
from utils import SpotNeighbor as spot

from simulation.phases.phase import Phase
import simulation.phases.phaseone as ph1
import simulation.phases.phasetwo as ph2


class PhaseFive(Phase):
    def __init__(self, Robot, superAS):
        super().__init__(Robot)
        self.phase = 5
        self.robot.super_cluster_id = superAS
        self.dir_x = self.robot.dir_x  #just for dbg
        self.dir_y = self.robot.dir_y  #just for dbg

    def __allowedAS(self):
        '''
        Returns the AS's that are in the same superAS that can be spot by the given robot.
        '''
        allowed = []
        allowed.append(self.robot.cluster_id)
        for n in self.robot.neighbors:
            if n.super_cluster_id == self.robot.super_cluster_id:
                if not n.cluster_id in allowed:
                    allowed.append(n.cluster_id)
        return allowed

    def __closestNeighbor(self):
        '''
        Returns the closest neighbor's AS that is in the same superAS.
        '''
        allowedAS = self.__allowedAS()
        best_neighbor, best_rd = spot.find_best_neighbor(
            self.robot, True, allowedAS)
        if not best_neighbor:
            return None  #The leader don't have the best neighbor
        else:
            return best_neighbor

    def just_dbg(self):
        robot = self.robot
        color = (130, 21, 169)
        pg.draw.circle(robot.image, color, (robot.radius, robot.radius),
                       robot.radius)

    def update(self):
        self.just_dbg()
        self.robot.broadcast["superAS"] = self.robot.super_cluster_id

    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 4:
                    self.AS = m["AS"]
                    self.upgrade(m["Phase"])
                    return

    def upgrade(self, next_phase=5):
        '''
        Upgrades the phase to further one.
        '''
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        elif next_phase == 3:
            self.robot.faza = ph2.PhaseThree(self.robot)
