import numpy as np
import pytest

from ..simulator import adjacent, Simulator
from ..problem import Problem


def simple_problem():
    return Problem(10, 10, np.ones((3, 3)) * 5)


def test_adjacent():
    assert adjacent((1, 1), (1, 2))
    assert adjacent((1, 1), (2, 1))
    assert adjacent((1, 1), (1, 0))
    assert adjacent((1, 1), (0, 1))

    assert not adjacent((1, 1), (2, 2))
    assert not adjacent((1, 1), (0, 0))


def test_simulator_init():
    robots = [['gavin', [-1, 0]], ['jeremy', [-1, 1]]]
    sim = Simulator(simple_problem(), robots)

    np.testing.assert_equal(sim.contamination, np.ones((3, 3)) * 5)
    assert sim.stations == {(-1, 0), (-1, 1)}
    assert sim.robot_positions == {(-1, 0), (-1, 1)}


def test_simulator_init_failures():
    robots = [['gavin', [-1, 0]], ['jeremy', [-1, 0]]]
    with pytest.raises(ValueError):
        sim = Simulator(simple_problem(), robots)

    robots = [['gavin', [2, 2]]]
    with pytest.raises(ValueError):
        sim = Simulator(simple_problem(), robots)


def test_simulator_move():
    robots = [['gavin', [-1, 0]], ['jeremy', [-1, 1]]]
    sim = Simulator(simple_problem(), robots)

    sim.apply(['gavin', 'move', [0, 0]])

    assert sim.robots['gavin'].pos == (0, 0)
    assert sim.robots['gavin'].fuel == 9
    assert sim.robot_positions == {(0, 0), (-1, 1)}
    assert sim.fuel_expended == 1


def test_simulator_move_failure():
    robots = [['gavin', [-1, 0]], ['jeremy', [-1, 1]]]
    sim = Simulator(simple_problem(), robots)

    sim.apply(['gavin', 'move', [-1, 1]])

    assert sim.robots['gavin'].pos == (-1, 0)
    assert sim.robots['gavin'].fuel == 10
    assert sim.robot_positions == {(-1, 0), (-1, 1)}
    assert sim.fuel_expended == 0


def test_simulator_clean():
    robots = [['gavin', [-1, 0]], ['jeremy', [-1, 1]]]
    sim = Simulator(simple_problem(), robots)

    sim.apply(['gavin', 'move', [0, 0]])
    sim.apply(['gavin', 'clean', 2])

    assert sim.robots['gavin'].fluid == 8
    assert sim.contamination[0, 0] == 3

    sim.apply(['gavin', 'clean', 4])

    assert sim.robots['gavin'].fluid == 4
    assert sim.contamination[0, 0] == 0

    sim.apply(['gavin', 'move', [0, 1]])
    sim.apply(['gavin', 'clean', 5])

    assert sim.robots['gavin'].fluid == 0
    assert sim.contamination[0, 1] == 1


def test_simulator_clean_off_board():
    robots = [['gavin', [-1, 0]], ['jeremy', [-1, 1]]]
    sim = Simulator(simple_problem(), robots)

    sim.apply(['gavin', 'clean', 2])

    assert sim.robots['gavin'].fluid == 8
    np.testing.assert_equal(sim.contamination, np.ones((3, 3)) * 5)


def test_simulator_resupply():
    robots = [['gavin', [-1, 0]], ['jeremy', [-1, 1]]]
    sim = Simulator(simple_problem(), robots)

    sim.apply(['gavin', 'move', [0, 0]])
    sim.apply(['gavin', 'clean', 3])

    assert sim.robots['gavin'].fuel == 9
    assert sim.robots['gavin'].fluid == 7

    sim.apply(['gavin', 'resupply'])

    assert sim.robots['gavin'].fuel == 9
    assert sim.robots['gavin'].fluid == 7

    sim.apply(['gavin', 'move', [-1, 0]])
    sim.apply(['gavin', 'resupply'])

    assert sim.robots['gavin'].fuel == 10
    assert sim.robots['gavin'].fluid == 10
