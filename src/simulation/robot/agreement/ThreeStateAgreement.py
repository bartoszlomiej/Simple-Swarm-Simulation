from abc import ABC, abstractmethod
from copy import deepcopy

SYN = 1
SYN_ACK = 2
ACK = 3

class ThreeStateAgreement(ABC):
    def __init__(self, cluster_id, messages, broadcastMessage):
        self.cluster_id = cluster_id
        self.messages = deepcopy(messages)
        self.broadcastMessage = broadcastMessage
        self.state = SYN

        
    def _isMessageInCluster(self, message):
        if message["AS"] == self.cluster_id:
            return True
        return False

    def _isAnnouncementIn(self, announcement, message):
        if announcement in message.keys():
            return True
        return False

    def _searchInMessages(self, announcement):
        for m in self.messages:
            if self._isMessageInCluster(m) and self._isAnnouncementIn(announcement, m):
                return m
        return None
    
    @abstractmethod
    def _syn(self, message):
        pass
        
    @abstractmethod
    def _synAck(self, message):
        pass
        
    @abstractmethod
    def _ack(self, message):
        pass
