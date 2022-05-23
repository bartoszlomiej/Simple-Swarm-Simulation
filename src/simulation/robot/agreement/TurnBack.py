from simulation.robot.agreement.ThreeStateAgreement import ThreeStateAgreement, SYN, SYN_ACK, ACK

ANT = "Turn back" #announcement

class TurnBack(ThreeStateAgreement):
    def __init__(self, cluster_id, messages, broadcastMessage, getDirection, checkCorrectness):
        super().__init__(cluster_id, messages, broadcastMessage)
        self.getDirection = getDirection
        self.checkCorrectness = checkCorrectness

    def _syn(self, message):
        if not self.checkCorrectness(message[ANT]):
            return
        self.broadcastMessage(ANT, message[ANT].copy())
        self.getDirection(message[ANT])
        self.state = SYN_ACK

    def _synAck(self, message):
        self.broadcastMessage("Waiting", self.state)
        if self.checkCorrectness(message[ANT]):
            self.getDirection(message[ANT])
        if not self._searchInMessages("Waiting"):
            self.broadcastMessage(ANT, message[ANT].copy())

    def _ack(self, message):
        self.state = ACK

    def __turnBack(self, message):
        if message and self.state == SYN:
            self._syn(message)
        elif message and self.state == SYN_ACK:
            self._synAck(message)
        elif not message and self.state == SYN_ACK:
            self._ack(message)

    def isAgreementOn(self):
        message = self._searchInMessages(ANT)
        if not message and self.state != SYN_ACK:
            return False
        self.__turnBack(message)
        return True
