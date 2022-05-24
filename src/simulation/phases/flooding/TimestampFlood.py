from time import time
from random import randint
from simulation.robot.agreement.ThreeStateAgreement import SYN_ACK, ACK

ANT = "Timestamp flooding"

class TimestampFlood:
    def __init__(self, threeStateAgreement, AgreementType):
        self.threeStateAgreement = threeStateAgreement
        self.agreement = AgreementType #It doesn't exists yet
        self.time = 0

    def __updateTime(self):
        self.time = time()
        
    def spillOver(self):
        self.__updateTime()
        self.agreement.broadcastMessage(ANT, self.time)
        self.agreement.state = SYN_ACK

    def repeat(self):
        return self.threeStateAgreement(self.agreement)

    def __inisideRobotFinished(self):
        return self.agreement.isFinishedEarly()

    def __edgeRobotFinished(self, isEdgeRobot):
        return ((self.agreement.state == ACK) and isEdgeRobot)

    def __estimateFloodingTime(self, messages):
        total_time = 0
        self.__updateTime()
        for m in messages:
             total_time += self.time - m
        return total_time
    
    def getTimeWhenFinished(self, isEdgeRobot):
        if self.__inisideRobotFinished or self.__edgeRobotFinished(isEdgeRobot):
            return self.__estimateFloodingTime(self.agreement.getMessages())
        return 0

