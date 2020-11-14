from dataclasses import dataclass
import json

import numpy as np


@dataclass
class Problem:
    max_fluid: int
    max_fuel: int
    floor: np.ndarray


class Robot:
    def __init__(self, name="", pos=[0, 0], fluid=0, fuel=0):
        self.name = name
        self.pos = pos
        self.fluid = fluid
        self.fuel = fuel
        self.at_charging_station = False
        self.is_going_up = True

    def clean(self, fluid_to_use):
        # print("Cleaning here")
        self.fluid -= fluid_to_use
        return [self.name, "clean", fluid_to_use]

    def move(self, dest):
        self.pos = dest
        self.fuel -= 1
        if self.fuel == 0:
            exit()
        return [self.name, "move", dest]

    def out_of_fluid(self):
        return self.fluid == 0

    def on_floor(self, floor_dimensions):
        return self.pos[0] >= 0 and self.pos[1] >= 0 and \
            self.pos[0] < floor_dimensions[0] and self.pos[1] < floor_dimensions[1]

    def output(self):
        return []


class Game:
    def __init__(self, problem):
        self.charging_stations = {}  # tuple -> bool for empty/not
        self.robots = []  # list of Robot objects
        self.robot_positions = set()
        self.num_robots = 0
        self.floor_dimensions = problem.floor.shape
        self.max_fuel = problem.max_fuel
        self.max_fluid = problem.max_fluid
        self.floor = problem.floor

    def adjacent(self, tile1, tile2):
        if tile1[0] == tile2[0] and abs(tile1[1] - tile2[1]) == 1:
            return True
        elif tile1[1] == tile2[1] and abs(tile1[0] - tile2[0]) == 1:
            return True
        return False

    def out_of_bounds(self, pos):
        return pos[0] < 0 or pos[1] < 0

    def can_robot_move(self, robot, dest):
        if self.out_of_bounds(dest) and (tuple(dest) not in self.charging_stations):
            # print("OUT OF BOUNDS: ", self.out_of_bounds(dest))
            # print("NOT IN A CHARGING STATION", tuple(
            # dest) not in self.charging_stations)
            # print("dest is out of bounds and not a charging stations")
            return False

        if not self.adjacent(dest, robot.pos):
            # print("Cannot move")
            return False
        elif robot.fuel == 0:
            # print("no fuel")
            return False
        elif tuple(dest) in self.robot_positions:
            # A robot already exists at that spot
            # print("spot taken")
            return False
        # print("Can move")
        return True

    def should_clean(self, rob):
        # print("checking cleaning at ", rob.pos)
        pos = rob.pos
        # if rob.at_charging_station:
        #     return False
        if self.floor[pos[0]][pos[1]] > 0 and rob.fluid > 0:
            # print("SHOULD CLEAN")
            return True
        return False

    def clean(self, robot):
        # if self.should_clean(robot):
        if robot.at_charging_station:
            self.resupply(robot)

        fluid_to_use = min(
            self.floor[robot.pos[0]][robot.pos[1]], robot.fluid)
        self.floor[robot.pos[0]][robot.pos[1]] -= fluid_to_use
        action = robot.clean(fluid_to_use)
        return action
        # return None

    def move(self, robot, coordinates):
        if self.can_robot_move(robot, coordinates):
            # if not robot.at_charging_station:
            self.robot_positions.remove(tuple(robot.pos))
            action = robot.move(coordinates)
            if tuple(robot.pos) in self.charging_stations:
                robot.at_charging_station = True
            else:
                robot.at_charging_station = False
            self.robot_positions.add(tuple(coordinates))
            return action
        return None

    def resupply(self, robot):
        # print(robot.at_charging_station)
        # print(robot.fuel)
        # print(robot.pos)
        # print("&&&&&&&&&&")
        if robot.at_charging_station:
            robot.fluid = self.max_fluid
            robot.fuel = self.max_fuel
            return [robot.name, "resupply"]
        return None

    def find_nearest_charging_station(self, robot):
        # TODO: this doesnt account for if there is a robot in the way
        min_moves_required = self.floor_dimensions[0] + \
            self.floor_dimensions[1] + 1
        for coord in self.charging_stations:
            closest_charging_station = coord
            break

        for coord in self.charging_stations:
            if self.charging_stations[coord]:
                continue
            moves_required = abs(robot.pos[0] - coord[0]) + \
                abs(robot.pos[1] - coord[1])
            if moves_required < min_moves_required:
                min_moves_required = moves_required
                closest_charging_station = (coord[0], coord[1])

        return (closest_charging_station, min_moves_required)

    def should_recharge(self, robot):
        # go home if your fuel is euqal to the amount of fuel needed to go back
        # go home if you are out of fluid
        # 🤦🏽‍♂️
        if robot.out_of_fluid():
            # print("OUT OF FLUID")
            return True
        _, moves_required = self.find_nearest_charging_station(robot)
        if moves_required == robot.fuel:
            return True

        return False

    def is_robot_beside_charging_station(self, robot, charging_station):

        if robot.pos[0] == charging_station[0] + 1 and robot.pos[1] == charging_station[1]:
            # robot is directly to the right of a charging station
            return True
        if robot.pos[1] == charging_station[1] + 1 and robot.pos[0] == charging_station[0]:
            # robot is directly below a charging station
            return True
        if robot.pos[0] == charging_station[0] - 1 and robot.pos[1] == charging_station[1]:
            # robot is directly to the left of a charging station
            return True
        if robot.pos[1] == charging_station[1] - 1 and robot.pos[0] == charging_station[0]:
            # robot is directly above a charging station
            return True
        return False

    def go_recharge(self, robot):

        closest_charging_station, _ = self.find_nearest_charging_station(robot)
        actions = []

        # print("******")
        # print(closest_charging_station)
        while robot.pos[0] > closest_charging_station[0]:
            # print("while1")
            # if robot.pos[0] >= -1:
            action = self.move(robot, (robot.pos[0]-1, robot.pos[1]))
            if action is not None:
                actions.append(action)
            else:
                break
            # print(robot.pos)
            # print(robot.pos[0]-1)
            # else:
            #     break
        while robot.pos[0] < closest_charging_station[0]:
            # print("while2")
            if robot.pos[0] <= self.floor_dimensions[1]:
                action = self.move(robot, (robot.pos[0]+1, robot.pos[1]))
                if action is not None:
                    actions.append(action)
            else:
                break
        while robot.pos[1] > closest_charging_station[1]:
            # print("while3")
            if robot.pos[0] <= self.floor_dimensions[1]:
                action = self.move(robot, (robot.pos[0], robot.pos[1]-1))
                if action is not None:
                    actions.append(action)
                else:
                    break
            else:
                break
        while robot.pos[1] < closest_charging_station[1]:
            # print("while4")
            actions = self.move(robot, (robot.pos[0], robot.pos[1]+1))
            if action is not None:
                actions.append(action)

        if self.is_robot_beside_charging_station(robot, closest_charging_station):
            # print("while5")
            action = self.move(robot, closest_charging_station)
            if action is not None:
                actions.append(action)

        # print(robot.pos)
        actions.append(self.resupply(robot))
        # print("******")
        return actions

    def determine_num_robots(self):
        # return min(self.floor_dimensions[0], self.floor_dimensions[1])
        return 1

    # Given the number of robots find the best starting locations
    def find_starting_locations(self, num_robots):
        # place at middle locations
        # return (0, 0)  # hardcoded for one robot

        # places robot in the middle of the longest dimension
        # if self.floor_dimensions[0] > self.floor_dimensions[1]:
        #     return [(self.floor_dimensions[0] // 2, -1)]
        # else:
        #     return [-1, self.floor_dimensions[1] // 2)]
        return {(-1, 0): False}

    def move_robot_onto_floor(self, robot):
        if robot.pos[0] == -1:
            # robot is to the left of the floor
            action = self.move(robot, [0, robot.pos[1]])
        elif robot.pos[1] == -1:
            # robot is above the floor
            action = self.move(robot, [robot.pos[0], 0])
        elif robot.pos[0] > self.floor_dimensions[0]:
            # robot is to the right of the floor
            action = self.move(robot, [robot.pos[0]-1, robot.pos[1]])
        else:
            # robot is below the floor
            action = self.move(robot, [robot.pos[0], robot.pos[1] - 1])
        return action

    def all_tiles_cleaned(self):
        for x in self.floor:
            for y in x:
                if y != 0:
                    return False
        # return True is the entire floor is clean
        return True

    def get_solution(self):
        # determine number of robots
        # assign charging station locations
        num_robots = self.determine_num_robots()
        self.charging_stations = self.find_starting_locations(num_robots)

        # print("here1")

        # create robots
        output_robots = []
        for i in range(num_robots):
            # starting_location = starting_locations[i]
            starting_location = (-1, 0)  # changed to dynamic
            robot = Robot(f"Robot_{i}", starting_location,
                          self.max_fluid, self.max_fuel)
            self.robot_positions.add(starting_location)
            output_robots.append([robot.name, list(starting_location)])
            self.robots.append(robot)

        # print("here2")
        # move robots onto the board
        # TODO: while loop conditions: all tiles cleaned
        output_file = open("solution.json", "w")
        actions = []
        while not self.all_tiles_cleaned():
            # print(self.floor)

            for robot in self.robots:
                if not robot.on_floor(self.floor_dimensions):
                    # print("here3")
                    actions.append(self.move_robot_onto_floor(robot))

                elif self.should_recharge(robot):
                    # recharge
                    actions.extend(self.go_recharge(robot))
                    # print("here4")
                elif self.should_clean(robot):
                    # print("here5")
                    # print(robot.pos)
                    action = self.clean(robot)
                    # print(action)
                    if action:
                        actions.append(action)
                else:
                    # move robot
                    # get coordinates for it to go somewhere
                    coordinates = robot.pos
                    action = []
                    if robot.is_going_up:
                        # print("here6")
                        if robot.pos[0] < self.floor_dimensions[0] - 1:
                            # move up
                            coordinates = [robot.pos[0] + 1, robot.pos[1]]
                            action = self.move(robot, coordinates)
                            # print("here7")
                        else:
                            # print("here8")
                            # move right
                            if robot.pos[1] < self.floor_dimensions[1] - 1:
                                coordinates = [robot.pos[0],
                                               robot.pos[1] + 1]
                                action = self.move(robot, coordinates)
                                robot.is_going_up = False
                                # print("here9")
                            # else:
                                # probably done, go home
                                # action = robot.go_recharge()
                    else:
                        # print("here10")
                        if robot.pos[0] > 0:
                            # print("here11")
                            # move down
                            coordinates = [robot.pos[0] - 1, robot.pos[1]]
                            action = self.move(robot, coordinates)
                            # print("here12")

                        else:
                            # print("here13")
                            # move right
                            if robot.pos[1] < self.floor_dimensions[1] - 1:
                                coordinates = [robot.pos[0],
                                               robot.pos[1] + 1]
                                action = self.move(robot, coordinates)
                                robot.is_going_up = True
                                # print("here14")

                            # else:
                                # probably done, go home
                                # action = robot.go_recharge()

                    # print("here15")
                    print("----------")
                    actions.append(action)
                    for act in actions:
                        if not act == None:
                            print(act)
                    print("----------")

                    # give actions to each robot
                    # output to json file

        output_actions = []
        for action in actions:
            if not action == None:
                output_actions.append(action)

        output = {"robots": output_robots, "actions": output_actions}
        output_file.write(output)
        print(output)
        # make sure all robots are in a charging station
        print("here16")
        output_file.close()


@ dataclass
class Solution():
    robots: list
    actions: list


def load_problem(path):
    with open(path) as f:
        r1 = f.readline().split()
        max_fluid = int(r1[0])
        max_fuel = int(r1[1])

        r2 = f.readline().split()
        num_rows = int(r2[0])
        num_cols = int(r2[1])

        floor = np.zeros((num_rows, num_cols))
        for i in range(num_rows):
            line = f.readline().split()
            for j in range(num_cols):
                floor[i, j] = int(line[j])

    return Problem(max_fluid, max_fuel, floor)


def load_solution(path):
    with open(path) as f:
        data = json.load(f)

    return Solution(data['robots'], data['actions'])
