import sys

import numpy as np
import pygame as pg
import math
import threading

from simulation.robot import RobotState
from simulation.robot.Robot import Robot
from simulation.robot.Position import Position
from utils.colors import WHITE
from utils.Resolution import Resolution


class Simulation:
    def run(self):
        pg.init()
        screen = pg.display.set_mode(
            [self.window_resolution.width, self.window_resolution.height])
        clock = pg.time.Clock()
        time = 100000
        for i in range(time):
            self.__exitOnQuitEvent()
            self.__updateSwarm()
            self.__handleCollisions()
            self.__robotVision()
            screen.fill(WHITE)
            self.swarm.draw(screen)
            pg.display.flip()
            clock.tick(100)
        pg.quit()

    def __init__(self,
                 window_resolution=Resolution(640, 400),
                 swarm_qty=10,
                 sensor_range=55,
                 velocity_lvl=4,
                 multithreading=True,
                 attraction_point=(0, 0, 0),
                 robot_radius=10):
        self.window_resolution = window_resolution
        self.sensor_range = sensor_range
        self.velocity_lvl = velocity_lvl
        self.multithreading = multithreading
        self.attraction_point = attraction_point
        self.robot_radius = robot_radius

        self.swarm = pg.sprite.Group()
        self.th_group = []
        self.swarm_quantity = swarm_qty

        self.__initializeRobots()
        self.__initializeMultithreading()

    def __initializeRobots(self):
        for i in range(self.swarm_quantity):
            spawn_new_rect = self.__createNonCollidingRect()

            robot = Robot(Position(spawn_new_rect.x,
                                   spawn_new_rect.y), self.window_resolution,
                          self.sensor_range, self.velocity_lvl)

            robot.ap = self.attraction_point
            self.swarm.add(robot)

    def __createNonCollidingRect(self):
        tries = 100_000
        for _ in range(tries):
            spawn_distance_margin = self.robot_radius * 2 + 5
            spawn_new_rect = self.__createSpawnRect(spawn_distance_margin)
            if not any(
                    neighbor
                    for neighbor in self.swarm if spawn_new_rect.colliderect(
                        neighbor.position.x, neighbor.position.y,
                        spawn_distance_margin, spawn_distance_margin)):
                return spawn_new_rect

    def __createSpawnRect(self, spawn_distance_margin):
        robot_distance_from_edge = self.robot_radius * 2
        up_to_x = self.window_resolution.width - self.robot_radius * 2 - 10
        up_to_y = self.window_resolution.height - self.robot_radius * 2 - 10
        spawn_position = Position.generateRandom(robot_distance_from_edge,
                                                 up_to_x,
                                                 robot_distance_from_edge,
                                                 up_to_y)

        return pg.Rect(spawn_position.x, spawn_position.y,
                       spawn_distance_margin, spawn_distance_margin)

    def __initializeMultithreading(self):
        if self.multithreading:
            for robot in self.swarm:
                self.th_group.append(pg.sprite.Group(robot))

    def __updateSwarm(self):
        if self.multithreading:
            self.__updateAll()
        else:
            self.swarm.update()

    def __updateAll(self):
        t = []
        for i in range(self.swarm_quantity):
            t.append(threading.Thread(target=self.__updateGroup, args=(i, )))
            t[i].start()

        for i in t:
            i.join()

    def __updateGroup(self, group_no):
        self.th_group[group_no].update()

    def __handleCollisions(self):
        for robot in self.swarm:
            other_robots = pg.sprite.Group(
                [s for s in self.swarm if s != robot])
            collisions = pg.sprite.spritecollide(robot, other_robots, False,
                                                 pg.sprite.collide_circle)
            if collisions:
                for c in collisions:
                    self.__updateMovementsOnCollision(robot, c)
                    self.__higherPhaseCollision(robot)

    def __higherPhaseCollision(self, robot):
        if robot.faza.phase > 2:
            robot.direction = robot.find_direction()

    def __updateMovementsOnCollision(self, robot, neighbor):
        if robot.faza.phase > 1:
            return
        dx = robot.position.x - neighbor.position.x
        dy = robot.position.y - neighbor.position.y

        if robot.state == RobotState.MOVING and neighbor.state == RobotState.MOVING:
            self.__oppositeMovementWithNoise(robot)
        elif robot.state == RobotState.MOVING:
            robot.velocity.negate()

        angle = 0.5 * math.pi + math.atan2(dy, dx)
        self.__moveRobotByAngle(robot, angle)

    def __oppositeMovementWithNoise(self, robot):
        noise = np.random.uniform(-0.2, 0.2, 2)
        robot.velocity.negate()
        robot.velocity.x += noise[0]
        robot.velocity.y += noise[1]

    def __moveRobotByAngle(self, robot, angle, sign=1):
        if robot.state == RobotState.MOVING:
            robot.position.x += math.sin(angle) * sign
            robot.position.y -= math.cos(angle) * sign

    def __robotVision(self):
        for r in self.swarm:
            for i in self.swarm:
                if r == i:
                    continue
                dx = (r.position.x - i.position.x)**2
                dy = (r.position.y - i.position.y)**2

                if (self.sensor_range**2) >= (dx + dy):
                    r.spotted(i)
                    if (0.15 * self.sensor_range**2) < (dx + dy):
                        r.in_range()

    def __exitOnQuitEvent(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()
