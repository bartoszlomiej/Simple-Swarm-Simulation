from simulation.robot.agreement.ThreeStateAgreement import ThreeStateAgreement, SYN, SYN_ACK, ACK

ANT = "Downgrade" #announcement

class Downgrade(ThreeStateAgreement):
    def __init__(self, cluster_id, messages, broadcastMessage, repeatDirection):
        super().__init__(cluster_id, messages, broadcastMessage)
        self.repeatDirection = repeatDirection
        self.is_downgrade = False

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

    def _downgrade(self, announcement, message):
        messageExists = self._isAnnouncementIn(announcement, message)
        if messageExists and self.state == SYN:
            return self._syn(message)
        elif messageExists and self.state == SYN_ACK:
            return self._synAck(message)
        elif not messageExists and self._searchInMessages("Waiting"):
            return self._ack(message)

    def checkIfDowngrade(self):
        for m in self.messages:
            downgrade_in_msg = self._downgrade(ANT, m)
            if downgrade_in_msg == True:
                self.is_downgrade = True
                return False
            elif downgrade_in_msg == False:
                self.is_downgrade = False
                return True
        return False
