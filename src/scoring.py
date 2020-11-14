import numpy as np

from .simulator import Simulator


def get_final_score(simulator):
    num_tiles = simulator.shape[0] * simulator.shape[1]
    contamination = np.sum(simulator.contamination)
    fuel = simulator.fuel_expended
    deployed = len(simulator.robots)
    stranded = 0

    for rob in simulator.robots.values():
        if rob.pos not in simulator.stations:
            stranded += 1

    numerator = 20 * num_tiles - (0.5 * contamination + 2 * fuel + 15 * deployed + 50 * stranded)

    return max(numerator / (20 * num_tiles), 0)


def evaluate(problem, solution):
    sim = Simulator(problem, solution.robots)
    sim.simulate(solution.actions)
    return get_final_score(sim)
