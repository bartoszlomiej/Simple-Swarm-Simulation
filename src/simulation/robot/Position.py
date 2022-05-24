import random


class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def moveBy(self, velocity):
        self.x += velocity.x
        self.y += velocity.y

    @staticmethod
    def generateRandom(startX, endX, startY, endY):
        return Position(random.randint(startX, endX),
                        random.randint(startY, endY))
