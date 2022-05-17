from numpy.random import rand


class Velocity:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @staticmethod
    def generateRandom(multiplier):
        return Velocity((rand() - 0.5) * multiplier,
                        (rand() - 0.5) * multiplier)
    def negate(self):
        self.x = self.x * -1
        self.y = self.y * -1