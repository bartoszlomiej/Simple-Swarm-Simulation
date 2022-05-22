from simulation.robot.Robot import Robot
from simulation.robot.ThreeStateAgreement import ThreeStateAgreement
from simulation.robot.Position import Position
#from utils.Resolution import Resolution

import time



from src.utils import SpotNeighbor as spot

r = Robot(200, 200, 1080, 720, [1, 1], 55)
n1 = Robot(225, 200, 1080, 720, [1, 1], 55)
n2 = Robot(170, 170, 1080, 720, [1, 1], 55)
n3 = Robot(220, 180, 1080, 720, [1, 1], 55)

r.neighbors.append(n1)
r.neighbors.append(n2)
r.neighbors.append(n3)
r.cluster_id = 1
n1.cluster_id = 1
n2.cluster_id = 1
n3.cluster_id = 3
dbg = ThreeStateAgreement(r.cluster_id, r.messages)
'''
for i in range(18):
    print(spot.check_line(r, i, 19, 55))
print(r.find_direction())
'''
'''
import math
import pygame as pg

def createSwarm():
    swarm = pg.sprite.Group()
    for i in range(200):
        robot = Robot(Position(200, 100), Resolution(640, 400), 50, 2, 8)
        swarm.add(robot)
    return swarm

def memoryLeak(swarm):
    for robot in swarm:
        other_robots = pg.sprite.Group([s for s in swarm if s != robot])
        collisions = pg.sprite.spritecollide(robot, other_robots, False, pg.sprite.collide_circle)
        other_robots.empty()
        #collisions.empty()
        del other_robots
        del collisions

def main():
    swarm = createSwarm()
    for i in range(100):
        memoryLeak(swarm)
        time.sleep(1/100)
        print("ok")

'''
#r.neighbors.append(n3)
#r.dir_x, r.dir_y = r.find_direction()
'''
n1.neighbors.append(r)
n1.neighbors.append(n2)

print(r.dir_x, r.dir_y)
n1.dir_x = r.dir_x
n1.dir_y = r.dir_y
a, b, d = spot.direction_line_equation(n1)
print("My line equation:", a, b, d)
print(spot.neighbor_check(n1, r, a, b, d))
print("My direction", spot.follower(n1))
'''
#dbg -> funkcja follower()
#objawy -> dir_x, dir_y = None if dirp
'''
    if d:
        if neighbor.y > (neighbor.x * a) + b:
            return True
    else:
        if neighbor.y < (neighbor.x * a) + b:  #this is breaking
            return True
    return False

wtedy dziala gora,
------------------------
    if d:
        if neighbor.y > (neighbor.x * a) + b:
            return True
    else:
        if neighbor.y < (neighbor.x * a) + b:  #this is breaking
            return True
    return False

a wtedy dziala dol xd
------------------------

    if d:
        if neighbor.y > (neighbor.x * a) + b:
            return True
    else:
        if neighbor.y < (neighbor.x * a) + b:  #this is breaking
            return True
        else:
            return True
    return False
a teraz mamy agregację -> zblizamy sie do siebie jak teletubisie
'''
