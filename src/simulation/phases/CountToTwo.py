from simulation.robot.Velocity import Velocity
from simulation.robot.Direction import Direction
from simulation.phases.StaticLine import StaticLine
from utils import SpotNeighbor as spot
from simulation.phases.StepForward import StepForward


class CountToTwo(StaticLine):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        self.phase = 3.5
        self.isIncreased = False
        self.robot.direction = Direction(1, 1)
        print(self.robot.direction.x, self.robot.direction.y)

    def __getSuperclustersMembers(self):
        cluster_members = []
        for n in self.robot.neighbors:
            if n.super_cluster_id == self.robot.super_cluster_id:
                cluster_members.append(n)
        return cluster_members

    def __getSuperclustersMembersID(self):
        allowed = []
        allowed.append(self.robot.cluster_id)
        for n in self.robot.neighbors:
            if n.super_cluster_id == self.robot.super_cluster_id:
                if not n.cluster_id in allowed:
                    allowed.append(n.cluster_id)
        return allowed

    def __getClosestNeighborCluster(self):
        supercluster_members = self.__getSuperclustersMembersID()
        best_neighbor, best_rd = spot.find_best_neighbor(
            self.robot, False, supercluster_members)
        if not best_neighbor:
            return None  #The leader don't have the best neighbor
        else:
            return best_neighbor.cluster_id

    def __setTimer(self):
        self.robot.setTimer(20)
        self.timerSet = True

    def __upgradeIfTimerFinished(self):
        if self.robot.timer.duration < 0:
            #self.upgrade(4, self.robot.super_cluster_id)
            pass

    def __useTimerIfSet(self):
        if not self.timerSet:
            self.__setTimer()
        else:
            self.robot.timer.tick()
            self.__upgradeIfTimerFinished()

    def __upgradeCluster(self):
        self.robot.cluster_id += 100
        self.isIncreased = True

    def __downgradeCluster(self):
        self.robot.cluster_id -= 100
        self.isIncreased = False

    def __updateClusterID(self):
        if not self.isIncreased:
            self.__upgradeCluster()
        else:
            self.__downgradeCluster()

    def __isInSameCluster(self, previous_robot_cluster_id):
        if previous_robot_cluster_id == self.robot.cluster_id:
            return True
        return False

    def __changeClusterIfSame(self, previous_robot_cluster_id):
        if self.__isInSameCluster(previous_robot_cluster_id):
            self.__updateClusterID()
        else:
            self.robot.update_color()  #just for dbg

    def __countToTwo(self):
        previous_robot_cluster_id = self.__getClosestNeighborCluster()
        if not previous_robot_cluster_id: #only leader should not have a previous AS
            self.__useTimerIfSet()
            return
        self.__changeClusterIfSame(previous_robot_cluster_id)

    def __keepStaticLine(self):
        self.same_cluster_neighbors.clear()
        self.same_cluster_neighbors = self.__getSuperclustersMembers()
        if self._isEdgeRobot():
            self.edgeRobotFunctionallity(0.4)
        else:
            self.insideRobotFunctionallity()
        
    def update(self):
        self.check_phase()
        self.robot.direction = Direction(1, 1)
        self.__countToTwo()
        self.robot.velocity = Velocity(0, 0)
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
        elif next_phase == 4:
            self.robot.faza = StepForward(self.robot, superAS)
        #            self.robot.faza = ph4.PhaseFour(self.robot, superAS)
