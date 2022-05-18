class Direction:
    def __init__(self, x, y):
        self.dir_x = x
        self.dir_y = y

    def negate(self):
        self.x = self.x * -1
        self.y = self.y * -1

    def perpendicular(self):
        self.x = self.x
        self.y = self.y * -1        
