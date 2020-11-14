import argparse

from src import problem, scoring


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Score a solution against a test case')
    parser.add_argument('-p', '--problem',
        type=str, help='Path to the test case', required=True)
    parser.add_argument('-s', '--solution',
        type=str, help='Path to the solution file', required=True)

    args = parser.parse_args()

    prob = problem.load_problem(args.problem)
    soln = problem.load_solution(args.solution)

    print(scoring.evaluate(prob, soln))
