from numpy import random


class Timer:
    def __init__(self, cluster_id, duration, neighbors_number):
        self.cluster_id = cluster_id
        self.duration = duration
        self.neighbors_number = neighbors_number

    def tick(self):
        self.duration -= 1

    @staticmethod
    def generateRandom(cluster_id, duration_from, duration_to, neighbors_number):
        return Timer(cluster_id, random.randint(duration_from, duration_to), neighbors_number)
