import numpy as np

from ..problem import Problem, Solution
from ..scoring import evaluate


def uniform_problem(val):
    return Problem(10, 10, np.ones((5, 2)) * val)


def test_scoring_did_nothing():
    prob = uniform_problem(50)
    soln = Solution([], [])

    assert evaluate(prob, soln) == 0


def test_scoring_theoretically_perfect():
    prob = uniform_problem(0)
    soln = Solution([], [])

    assert evaluate(prob, soln) == 1


def test_scoring_deployed_one_robot():
    prob = uniform_problem(0)
    soln = Solution([['gavin', [-1, 0]]], [])

    assert evaluate(prob, soln) == (20 * 10 - 15) / (20 * 10)


def test_scoring_stranded_gavin():
    prob = uniform_problem(0)
    soln = Solution([['gavin', [-1, 0]]], [
        ['gavin', 'move', [0, 0]]
    ])

    assert evaluate(prob, soln) == (20 * 10 - (15 + 2 + 50)) / (20 * 10)


def test_scoring_brought_gavin_home():
    prob = uniform_problem(0)
    soln = Solution([['gavin', [-1, 0]]], [
        ['gavin', 'move', [0, 0]],
        ['gavin', 'move', [-1, 0]]
    ])

    assert evaluate(prob, soln) == (20 * 10 - (15 + 2 * 2)) / (20 * 10)


def test_scoring_did_some_cleaning():
    prob = uniform_problem(1)
    soln = Solution([['gavin', [-1, 0]]], [
        ['gavin', 'move', [0, 0]],
        ['gavin', 'clean', 1],
        ['gavin', 'move', [-1, 0]]
    ])

    assert evaluate(prob, soln) == (20 * 10 - (0.5 * 9 + 15 + 2 * 2)) / (20 * 10)
