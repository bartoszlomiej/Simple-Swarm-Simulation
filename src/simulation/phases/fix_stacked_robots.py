from simulation.robot import RobotState
from simulation.robot.Timer import Timer

class Stacked():
    def __init__(self, robot):
        self.robot = robot
        self.robot.timer = None
        
    def changeStateIfStacked(self):
        if self.isStacked():
            #broadcast initial direction
            #three state change state (in order not to fall into a trap
            pass


    def isStacked(self):
        if self.robot.state == RobotState.STOPPED:
            self.useRobotTimerIfStopped()
            if self.robot.timer.duration <= 0:
                return True
        else:
            self.robot.timer = None

        
    def useRobotTimerIfStopped(self):
        if not self.robot.timer:
            self.robot.setRandomTimer(200, 500)
        self.robot.timer.tick()
            return False    
        
