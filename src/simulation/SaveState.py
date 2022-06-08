import pickle

class SavedStates:
    def __init__(self, filename):
        self.path = "../logs/"
        self.filename = self.path + filename
        self.filehandler = None

    def initializeSaving(self):
        self.filehandler = open(self.filename, 'wb')        
    
    def saveRobotState(self, robot):
        pickle.dump(robot.getRobotState(), self.filehandler)

    def initializeLoading(self):
        self.filehandler = open(self.filename, 'rb')        

    def loadRobotState(self):
        try:
            return pickle.load(self.filehandler)
        except EOFError:
            return None

    
        
