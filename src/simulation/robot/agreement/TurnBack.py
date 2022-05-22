from robot.agreement.ThreeStateAgreement import ThreeStateAgreement, CLOSED, SYN, SYN_ACK, ACK

ANT = "Turn Back" #announcement

class TurnBack(ThreeStateAgreement):
    def __init__(self, cluster_id, messages, broadcastMessage, repeatDirection):
        self.super().__init__(cluster_id, messages, broadcastMessage)
        self.repeatDirection = repeatDirection

    def _syn(self, message):
        self.broadcastMessage(ANT, message[ANT])
        self.repeatDirection(message)
        self.state = SYN_ACK
        return True

    def _synAck(self, message):
        self.state = SYN_ACK
        self.broadcastMessage("Waiting", self.state)
        self.repeatDirection(message)
        if self._searchInMessages("Waiting"):
            return True
        self.broadcastMessage(ANT, message[ANT])
        return True

    def _ack(self, message):
            self.repeatDirection(message)
            self.state = ACK
            return False        

    def _turnBack(self, announcement, message):
        messageExists = self._isAnnouncementIn(announcement, message)
        if messageExists and self.state == SYN:
            return self._syn(message)
        elif messageExists and self.state == SYN_ACK:
            return self._synAck(message)
        elif not messageExists and self._searchInMessages("Waiting"):
            return self._ack(message)

    def checkIfTurnBack(self):
        for m in self.messages:
            turnback_in_msg = self._turnBack("Turn back", m)
            if downgrade_in_msg == True:
                self.is_downgrade = True
                return False
            elif downgrade_in_msg == False:
                self.is_downgrade = False
                return True
        return False
            
