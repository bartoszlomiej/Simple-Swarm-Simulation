import timeit  #just for dbg

import sys
import numpy as np
import pygame as pg
import math
import Robot as rbt


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
                 velocity_lvl=4):
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

        self.swarm = pg.sprite.Group()
        self.swarm_quantity = N

        self.initialize_robots()

    def initialize_robots(self):
        '''
        Initializes robots on empty board.
        '''
        for i in range(self.swarm_quantity):
            x = np.random.randint(0, self.width + 1)
            y = np.random.randint(0, self.height + 1)
            velocity = np.random.rand(2)
            velocity[0] = (velocity[0] - 0.5) * self.velocity_lvl
            velocity[1] = (velocity[1] - 0.5) * self.velocity_lvl
            '''
            TODO: Starting position could be checked here to avoid overlaping
            self.board[i] = [x, y]
            '''
            robot = rbt.Robot(x, y, self.width, self.height, velocity,
                              self.sensor_range)
            self.swarm.add(robot)

    def check_collisions(self):
        '''
        For each robot in a swarm checks if the collision occurs. If so then the velocity is being changed accordingly.
        '''
        for robot in self.swarm:
            collision_group = pg.sprite.Group(
                [s for s in self.swarm if s != robot])
            collide = pg.sprite.spritecollide(robot, collision_group, False)
            if collide:
                #                robot.moved = True
                for c in collide:  #there can be numerous collisions however it is unlikely
                    #                    if c.moved == True:
                    #                        continue
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


#        else:
#            robot.velocity = [
        '''
        Previous solution:
#        robot.velocity = [-robot.velocity[0], -robot.velocity[1]]
        '''
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

    def dbg_timer(self, t):
        for i in self.swarm:
            if i.faza.phase == 2:
                print("Cluster: ", i.AS, "  TIME: ", t)  #just for dbg
                i.faza.phase = -1  #just for dbg!

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
            self.swarm.update()
            self.check_collisions()
            self.robot_vision()
            #            self.dbg_timer(i)
            screen.fill((255, 255, 255))  #background color (white)
            self.swarm.draw(screen)
            pg.display.flip()
            clock.tick(100)
        pg.quit()
