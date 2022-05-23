from simulation.robot.agreement.ThreeStateAgreement import ThreeStateAgreement, SYN, SYN_ACK, ACK

ANT = "Downgrade"  #announcement


class Downgrade(ThreeStateAgreement):
    def __init__(self, cluster_id, messages, broadcastMessage,
                 repeatDirection):
        super().__init__(cluster_id, messages, broadcastMessage)
        self.repeatDirection = repeatDirection
        self.is_downgrade = False

    def _syn(self, message):
        self.broadcastMessage(ANT, message[ANT])
        self.repeatDirection(message)
        self.state = SYN_ACK

    def _synAck(self, message):
        self.broadcastMessage("Waiting", self.state)
        self.repeatDirection(message)
        if not self._searchInMessages("Waiting"):
            self.broadcastMessage(ANT, message[ANT])

    def _ack(self, message):
        self.repeatDirection(message)
        self.state = ACK

    def __downgrade(self, message):
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
        self.__downgrade(message)
        return True
