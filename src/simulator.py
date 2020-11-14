from dataclasses import dataclass

import numpy as np


def adjacent(tile1, tile2):
    if tile1[0] == tile2[0] and abs(tile1[1] - tile2[1]) == 1:
        return True
    elif tile1[1] == tile2[1] and abs(tile1[0] - tile2[0]) == 1:
        return True
    return False


@dataclass
class RobotState:
    pos: tuple
    fluid: int
    fuel: int


class Simulator:
    def __init__(self, problem, robots):
        self.max_fluid = problem.max_fluid
        self.max_fuel = problem.max_fuel
        self.contamination = problem.floor
        self.shape = self.contamination.shape
        self.robot_positions = set()

        self.robots = {}
        self.stations = set()
        for r in robots:
            initial_pos = tuple(r[1])
            self.robots[r[0]] = RobotState(initial_pos, self.max_fluid, self.max_fuel)
            if initial_pos in self.stations:
                raise ValueError('solution places two robots in the same initial position')
            elif self.on_board(initial_pos):
                raise ValueError('base station was placed in the competition hall')
            self.stations.add(initial_pos)
            self.robot_positions.add(initial_pos)

        self.fuel_expended = 0

    def simulate(self, actions):
        for action in actions:
            self.apply(action)

    def apply(self, action):
        if action[0] not in self.robots:
            return
        robot = self.robots[action[0]]

        if action[1] == 'move':
            self.move(robot, tuple(action[2]))
        elif action[1] == 'clean':
            self.clean(robot, action[2])
        elif action[1] == 'resupply':
            self.resupply(robot)

    def move(self, robot, dest):
        if not adjacent(dest, robot.pos):
            return
        elif robot.fuel == 0:
            return
        elif dest in self.robot_positions:
            return
        self.robot_positions.remove(robot.pos)
        robot.pos = dest
        robot.fuel -= 1
        self.fuel_expended += 1
        self.robot_positions.add(dest)
    
    def clean(self, robot, amount):
        actual_amount = min(amount, robot.fluid)
        robot.fluid -= actual_amount
        tile = robot.pos
        if self.on_board(tile):
            self.contamination[tile] = max(self.contamination[tile] - actual_amount, 0)

    def resupply(self, robot):
        if robot.pos in self.stations:
            robot.fluid = self.max_fluid
            robot.fuel = self.max_fuel

    def on_board(self, tile):
        return tile[0] >= 0 and tile[1] >= 0 and \
               tile[0] < self.shape[0] and tile[1] < self.shape[1]
