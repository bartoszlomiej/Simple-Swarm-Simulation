import sys
import numpy as np
import pygame as pg
import math
import Robot as rbt
import threading

#just for dbg
import os

class Simulation:
    '''
    Main class of the simulation - it is responsible for generation of the board and robots. 
    Furthermore, it emulates the sensors for each robot.
    '''
    def __init__(self,
                 width=640,
                 height=400,
                 N=10,
                 s_range=55,
                 velocity_lvl=4, threads=1,
                 attraction_point=(0,0,0)):
        '''
        width - the width of the screen
        height - the height of the screen
        N - a swarm quantity
        s_range - sensor range in pixels (must be greater than 20)
        velocity_lvl 0 - multiplier of the velocity 
        '''
        self.width = width
        self.height = height
        self.size = (width, height)
        self.sensor_range = s_range  #(as 20 is the delimiter of the robot)
        self.velocity_lvl = velocity_lvl

        self.attraction_point = attraction_point

        self.swarm = pg.sprite.Group()
        self.th_group = []
        self.swarm_quantity = N

        self.initialize_robots()

        self.threads = threads
        if self.threads > 1:
            self.initialize_multithreading(threads)
            
    def initialize_robots(self):
        '''
        Initializes robots on empty board.
        '''
        for i in range(self.swarm_quantity):
            collide = True
            while collide:
                x = np.random.randint(10, self.width - 9)
                y = np.random.randint(10, self.height - 9)
                new_rect = pg.Rect(x, y, 15, 15)
                if not any(n for n in self.swarm if new_rect.colliderect(n.x, n.y, 15, 15)):
                    collide = False
                    break
            if True:
                velocity = np.random.rand(2)
                velocity[0] = (velocity[0] - 0.5) * self.velocity_lvl
                velocity[1] = (velocity[1] - 0.5) * self.velocity_lvl
                
                robot = rbt.Robot(x, y, self.width, self.height, velocity,
                                  self.sensor_range)
                robot.ap = self.attraction_point #JUST FOR DBG
                self.swarm.add(robot)                    


    def initialize_multithreading(self, threads):
        '''
        Initialize multithreading with thread number equal to 'threads'.
        '''
        group_size = math.ceil(self.swarm_quantity / threads)
        k = 0
        i = 0
        self.th_group.append(pg.sprite.Group())
        for robot in self.swarm:
            if i >= (k+1) * group_size:
                self.th_group.append(pg.sprite.Group())
                k += 1
            self.th_group[k].add(robot)
            i += 1

    def update_group(self, group_no):
        '''
        Updates the th_group of the given number
        '''
        self.th_group[group_no].update()

    def update_all(self):
        '''
        Creates 'threads' number of threads that will run separately the update on given group of sprites
        '''
        t = []
        for i in range(self.threads):
            t.append(threading.Thread(target=self.update_group, args= (i,)))
            t[i].start()

        for i in t:
            i.join() #synchronization of threads

    def check_collisions(self):
        '''
        For each robot in a swarm checks if the collision occurs. If so then the velocity is being changed accordingly.
        '''
        for robot in self.swarm:
            collision_group = pg.sprite.Group(
                [s for s in self.swarm if s != robot])
            collide = pg.sprite.spritecollide(robot, collision_group, False)
            if collide:
                for c in collide:  #there can be numerous collisions however it is unlikely
                    self.collision_movement(robot)

    def collision_movement(self, robot):
        '''
        Robots should move in the semi-random direction after collision:
        having velocity = (x, y) -> new velocity = (-random * sign(x), -random * sign(y))
        as a result never two robots will go in the same direction after collision (no stucking)
        '''
        if robot.faza.phase == 1:
            sign_x = np.sign(robot.velocity[0])
            sign_y = np.sign(robot.velocity[1])
            velocity = np.random.rand(2)
            velocity[0] = (
                velocity[0]) * self.velocity_lvl / 2  #must be positive!
            velocity[1] = (velocity[1]) * self.velocity_lvl / 2
            robot.velocity = [-sign_x * velocity[0], -sign_y * velocity[1]]

    def robot_vision(self):
        '''
        Emulates the very basic vision sensor of each robot in the swarm.
        '''
        for r in self.swarm:
            for i in self.swarm:
                if r == i:
                    continue
                dx = abs(r.position[0] - i.position[0])
                dy = abs(r.position[1] - i.position[1])

                dx *= dx
                dy *= dy

                if (self.sensor_range**2) >= (dx + dy):  #using sqaure equation
                    r.spotted(i)
                    if (0.15 * self.sensor_range**2) < (dx + dy):
                        r.in_range()
    def run(self):
        '''
        Runs the simulation. After certain time the simulation is closed.
        '''
        pg.init()
        screen = pg.display.set_mode([self.width, self.height])
        clock = pg.time.Clock()
        Time = 100000
        for i in range(Time):
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    sys.exit()
            if self.threads > 1:
                self.update_all()
            else:
                self.swarm.update()
            self.check_collisions()
            self.robot_vision()
            screen.fill((255, 255, 255))  #background color (white)
            self.swarm.draw(screen)
            pg.display.flip()
            clock.tick(100)
        pg.quit()
