from time import time
from random import randint
from simulation.robot.agreement.ThreeStateAgreement import SYN_ACK, ACK

QUERY_ANT = "Stable formation flooding"
ANSWER_ANT = "Stable formation OK"

class StableFormationFlood:
    def __init__(self, threeStateAgreement, flooding):
        self.threeStateAgreement = threeStateAgreement
        self.agreement = flooding
        self.state = QUERY_ANT

    def __spillOver(self, announcement):
        self.__updateTime()
        self.agreement.broadcastMessage(announcement, self.time)
        self.agreement.state = SYN

    def repeat(self, is_ready):
        if is_ready:
            return self.threeStateAgreement(self.agreement)

    
    def spillOverQuery(self):
        self.__spillOver(QUERY_ANT)

    def spillOverAnswer(self):
        self.__spillOver(ANSWER_ANT)

    def updateState(self):
        '''
        1) we obtained messages from two sided and we are ready
        2) we are the edge robot who did not send this message
        3) we are ready, and we obtained the ANSWER_ANT 
        '''
        pass

    def __areTwoMessagesObtained(self):
        pass

    def __overEdgeRobotMessage(self):
        pass
