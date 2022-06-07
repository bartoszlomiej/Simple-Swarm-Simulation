import pickle

class SavedStates:
    def __init__(self, filename):
        self.path = "../logs/"
        self.filename = self.path + filename
        self.filehandler = None

    
    def saveRobotState(self, robot):
        filehandler = open(self.filename, 'ab')
        pickle.dump(robot.getRobotState(), filehandler)

    def initializeLoading(self):
        self.filehandler = open(self.filename, 'rb')        

    def loadRobotState(self):
        try:
            return pickle.load(self.filehandler)
        except EOFError:
            return None

    
        
