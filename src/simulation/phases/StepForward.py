from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from simulation.phases.StaticLine import StaticLine
from utils import SpotNeighbor as spot


class StepForward(StaticLine):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        self.phase = 4
        self.isIncreased = False
        self.robot.direction = Direction(1, 1)

    def __getSuperclusterMembers(self):
        cluster_members = []
        for n in self.robot.neighbors:
            if n.super_cluster_id == self.robot.super_cluster_id:
                cluster_members.append(n)
        return cluster_members

    def __getSuperclusterMembersID(self):
        allowed = []
        allowed.append(self.robot.cluster_id)
        for n in self.robot.neighbors:
            if n.super_cluster_id == self.robot.super_cluster_id:
                if not n.cluster_id in allowed:
                    allowed.append(n.cluster_id)
        return allowed

    def __getSameClusterMembers(self):
        self.same_cluster_neighbors.clear()
        for n in self.robot.neighbors:
            if n.cluster_id != self.robot.cluster_id:
                continue
            self.same_cluster_neighbors.append(n)
        return self.same_cluster_neighbors

    def __checkForStepForward(self):
        self.robot.direction = Direction(1, 1)
        #self.robot.follower_msg()  #there is a need of direction --this might cause some problems
        closest_neighbor, rd = spot.find_best_neighbor(
            self.robot, False, self.__getSuperclusterMembersID())
        if not closest_neighbor:
            return
        self.__stepForwardIfHigherPriority(closest_neighbor)

    def __stepForwardIfHigherPriority(self, closest_neighbor):
        if self.__isHigherClusterID(closest_neighbor):
            self.__stepOutFromLine()
            self.__moveIfPathIsFree()
        '''
        else:
            self.__goCloserToPreviousNeighbor()
        self.__moveIfPathIsFree()
        '''


    def __isHigherClusterID(self, closest_neighbor):
        if closest_neighbor.cluster_id < self.robot.cluster_id:
            return True
        return False

    def __stepOutFromLine(self):
        self.__tryPerpendicularMotion()
        '''
        if spot.is_follower(self.robot):
            spot.follower(self.robot)
            if self.robot.direction.x == 0 or self.robot.direction.y == 0:
                self.__tryPerpendicularMotion()
        else:
            self.__tryPerpendicularMotion()
        '''
    def __tryPerpendicularMotion(self):
        self.same_cluster_neighbors = self.__getSuperclusterMembers()
        if not self.same_cluster_neighbors:
            return

        best_neighbor, distance = self._findClosestNeighbor()
        spot.direction_to_neighbor(self.robot, best_neighbor)
        self.robot.direction.perpendicular()

    def __goCloserToPreviousNeighbor(self):
        spot.follower(self.robot)

    def __moveIfPathIsFree(self):
        if not spot.is_any_collision(self.robot, 0.15):
            self.robot.direction.normalize()
            self.makeMove()

    def __isInSameCluster(self, previous_robot_cluster_id):
        if previous_robot_cluster_id == self.robot.cluster_id:
            return True
        return False

    def __changeClusterIfSame(self, previous_robot_cluster_id):
        if self.__isInSameCluster(previous_robot_cluster_id):
            self.__updateClusterID()
        else:
            self.robot.update_color()  #just for dbg

    def __keepStaticLine(self):
        self.same_cluster_neighbors.clear()
        self.same_cluster_neighbors = self.__getSameClusterMembers()
        if self._isEdgeRobot():
            self.edgeRobotFunctionallity(0.5)
        else:
            self.insideRobotFunctionallity()

    def update(self):
        self.check_phase()
        self.robot.velocity = Velocity(0, 0)
        self.__checkForStepForward()
        self.__keepStaticLine()
        self.robot.isAlloneInSupercluster()

    def check_phase(self):
        robot = self.robot
        for m in robot.received_messages:
            if "Phase" in m.keys():
                if m["Phase"] >= 4:
                    robot.broadcast["superAS"] = self.robot.super_cluster_id
                    self.upgrade(m["Phase"], self.robot.super_cluster_id)
                    return

    def upgrade(self, next_phase=4, superAS=None):
        if next_phase == 1.5:
            self.robot.faza = ph1.PhaseOneAndHalf(self.robot)
        elif next_phase == 2:
            self.robot.faza = ph2.PhaseTwo(self.robot)
        #        elif next_phase == 4:
        #            self.robot.faza = ph4.PhaseFour(self.robot, superAS)
