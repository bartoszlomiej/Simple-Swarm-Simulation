from simulation.robot.agreement.ThreeStateAgreement import ThreeStateAgreement, SYN, SYN_ACK, ACK

ANT = "Turn back" #announcement

class TurnBack(ThreeStateAgreement):
    def __init__(self, cluster_id, messages, broadcastMessage, getDirection, checkCorrectness):
        super().__init__(cluster_id, messages, broadcastMessage)
        self.getDirection = getDirection
        self.checkCorrectness = checkCorrectness
        self.isTurnBack = False


    def _syn(self, message):
        if not self.checkCorrectness(message[ANT]):
            return #???
        self.broadcastMessage(ANT, message[ANT].copy())
        self.getDirection(message[ANT])
        self.state = SYN_ACK
        return True

    def _synAck(self, message):
        self.state = SYN_ACK
        self.broadcastMessage("Waiting", self.state)
        if not self.checkCorrectness(message[ANT]):
            return #???
        self.getDirection(message[ANT])
        if self._searchInMessages("Waiting"):
            return True
        self.broadcastMessage(ANT, message[ANT].copy())
        return True

    def _ack(self, message):
        self.state = ACK
        return False        

    def __turnBack(self, message):
        messageExists = self._isAnnouncementIn(ANT, message)
        if messageExists and self.state == SYN:
            return self._syn(message)
        elif messageExists and self.state == SYN_ACK:
            return self._synAck(message)
        elif not messageExists and self.state == SYN_ACK:
            return self._ack(message)
        '''
        elif not messageExists and self._searchInMessages("Waiting"):
            return self._ack(message)
        '''



    def checkIfTurnBack(self):
        for m in self.messages:
            turnback_in_msg = self.__turnBack(m)
            if turnback_in_msg == True:
                self.isTurnBack = False
            elif turnback_in_msg == False:
                self.isTurnBack = True
                break
            self.isTurnBack = False

