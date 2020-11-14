from dataclasses import dataclass
import json

import numpy as np


@dataclass
class Problem:
    max_fluid: int
    max_fuel: int
    floor: np.ndarray


@dataclass
class Solution:
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
