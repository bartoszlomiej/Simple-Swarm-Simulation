from simulation.robot.agreement.ThreeStateAgreement import ThreeStateAgreement, SYN, SYN_ACK, ACK

ANT = "Timestamp flooding" #announcement

FINISHED = 4

class Flooding(ThreeStateAgreement):
    def __init__(self, cluster_id, messages, broadcastMessage):
        super().__init__(cluster_id, messages, broadcastMessage)
        self.previous_message = None
        self.output = None

    def _syn(self, message):
        self.broadcastMessage(ANT, message)
        self.state = SYN_ACK
        self.previous_message = message

    def _synAck(self, message):
        self.broadcastMessage("Waiting", self.state)
        if not self._searchInMessages("Waiting"):
            self.broadcastMessage(ANT, message)

    def _ack(self, message):
        self.state = ACK
        self.previous_message = None

    def _reset(self):
        self.state = SYN
        self.previous_message = None

    def __flood(self, message):
        if message and self.state == SYN:
            self._syn(message)
        elif message and self.state == SYN_ACK:
            self._synAck(message)
        elif not message and self.state == SYN_ACK:
            self._ack(message)
        elif not message and self.state == ACK:
            self._reset()

    def _searchInMessages(self, announcement):
        messages = None
        for m in self.messages:
            if self._isMessageInCluster(m) and self._isAnnouncementIn(announcement, m):
                if not m[ANT] in messages:
                    messages.append(m[ANT])
        return messages

    def __isEqual(self, previous_message, new_message):
        if not previous_message:
            return False
        if previous_message == new_message:
            return True
        return False
    
    def __serveSingleMessage(self, message):
        if message:
            message = message[-1]
            self.output = message
        if not self.__isEqual(self.previous_message, message):
            self._reset()        
        self.__flood(message)

    def __serveMultipleMessages(self, messages):
        self.state = FINISHED
        self.output = messages
        
    def isAgreementOn(self):
        messages = self._searchInMessages(ANT)
        if not messages and self.state != SYN_ACK:
            return False
        if len(messages) <= 1:
            self.__serveSingleMessage(messages)
        else:
            self.__serveMultipleMessages(messages)
        return True

    def isFinishedEarly(self):
        if self.state == FINISHED:
            return True
        return False

    def getMessages(self):
        return self.output

    def updateMessages(self, messages):
        self.messages = messages
