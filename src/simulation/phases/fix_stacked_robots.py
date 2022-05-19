from simulation.robot import RobotState

class Stacked():
    def __init__(self, robot):
        self.robot = robot
        self.robot.setTimer(500)

    def isStacked(self):
        self.robot.timer.tick()
        if self.robot.timer.duration <= 0:
            return True
        return False
        
