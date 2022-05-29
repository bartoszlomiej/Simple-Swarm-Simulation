from simulation.robot.Velocity import Velocity
from simulation.phases.StaticLine import StaticLine
from utils import SpotNeighbor as spot

class StepForward(StaticLine):
    def __init__(self, Robot, superAS):
        super().__init__(Robot, superAS)
        self.phase = 4
        self.isIncreased = False

    def __getSameClusterMembers(self):
        self.same_cluster_neighbors.clear()
        for n in self.robot.neighbors:
            if n.cluster_id != self.robot.cluster_id:
                continue
            self.same_cluster_neighbors.append(n)
        return self.same_cluster_neighbors

    def __checkForStepForward(self):
        self.robot.follower_msg() #there is a need of direction
        closest_neighbor = spot.find_best_neighbor(self.robot)
        if not closest_neighbor:
            return
        self.__stepForwardIfHigherPriority(closest_neighbor)

    def __stepForwardIfHigherPriority(self, closest_neighbor):
        if self.__isHigherClusterID(closest_neighbor)
            self.__stepOutFromLine(closest_neighbor)
        else:
            self.__goCloserToPreviousNeighbor()
        self.__moveIfPathIsFree()

    def __isHigherClusterID(self, closest_neighbor):
        if closest_neighbor.cluster_id < self.robot.cluster_id:
            return True
        return False

    def __stepOutFromLine(self, closest_neighbor):
        if spot.is_follower(self.robot):
            spot.follower(self.robot)
            if self.robot.dir_x == 0 or self.robot.dir_y == 0:
                self.__tryPerpendicularMotion(closest_neighbor)
        else:
            self.__tryPerpendicularMotion(closest_neighbor)
        
    def __tryPerpendicularMotion(self):
        main_cluster_neighbors = self.getMainClusterNeighbors()
        if not main_cluster_neighbors:
            return
        
        best_neighbor = self._findClosestNeighbor(main_cluster_neighbors)
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
            self.robot.update_color() #just for dbg

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
        #self.__keepStaticLine()
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
