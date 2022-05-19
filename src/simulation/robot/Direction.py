class Direction:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def negate(self):
        self.x = self.x * -1
        self.y = self.y * -1

    def perpendicular(self):
        (self.x, self.y) = (self.y, -self.x)

    def normalize(self):
        while (self.x**2 + self.y**2) > 1:
            self.x /= 2
            self.y /= 2

    def stop(self):
        return Direction(0, 0)
