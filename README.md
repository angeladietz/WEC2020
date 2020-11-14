# WEC 2020 Programming Competition

This repository contains the test cases, grading code, and visualization
code for the Fall 2020 WEC Programming Competition.

## Grading a Solution

The `mark_solution.py` script can be used to grade a single solution
against a particular test case, and can be used as follows:
```
python mark_solution.py -p prob_path -s soln_path
```
where `prob_path` is the path to the test case problem file and
`soln_path` is the path to the solution JSON file.

## Visualizing a Solution

The `visualize_solution.py` script can be used to generate images
visualizing each frame of a solution as it is executed. It should be
invoked as follows:
```
python visualize_solution.py -p prob_path -s soln_path -o output_dir
```
where `prob_path` and `soln_path` are as above, and `output_dir` is the
directory where the images should be saved.
