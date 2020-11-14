from src import problem, visualization, simulator


def main():
    prob = problem.load_problem('test_cases/case1.txt')
    game = problem.Game(prob)
    game.get_solution()


if __name__ == '__main__':
    main()
