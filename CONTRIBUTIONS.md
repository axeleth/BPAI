# Contributions

The code in [`thesis/`](thesis/) combines provided code with my thesis work. This page says which is which. The version submitted with the thesis is tagged `v1.0-thesis-submission`.

## Starting point

1. **Course framework** (TU Delft AE4422-20). It was imported unchanged in commit `b9e4212` and provides:
   - the simulation loop skeleton in `run_me.py`
   - `visualization.py`
   - the base `Aircraft` class
   - unconstrained single-agent A* in `single_agent_planner.py`
   - the `independent.py`/`prioritized.py` stubs
   - the airport layout (`nodes.xlsx`, `edges.xlsx`).
2. **Group 6 CBS solution** (F. Sirghi & V. van Gorp), kept in [`archive/baseline-group6/`](archive/baseline-group6/). It assumes aircraft keep their planned speed. My `cbs.py` and the spawning logic in `run_me.py` started from it. Compare [`archive/baseline-group6/cbs.py`](archive/baseline-group6/cbs.py) with [`thesis/cbs.py`](thesis/cbs.py).

To see everything I changed relative to the course framework: [compare `b9e4212...v1.0-thesis-submission`](https://github.com/axeleth/BPAI/compare/b9e4212...v1.0-thesis-submission).

## My work, per file

| File | Contribution |
|---|---|
| `deviate.py` | **New.** `check_deviation` applies speed profiles (`cS` cautious start, `cE` cautious end, `fS` fast start, `fast`, `slow`). `detect_deviation` flags aircraft that arrive at a node too early or too late. `prep_CBS_replan` turns each aircraft's position into a replanning start `(node, time)`, including aircraft that are between nodes. |
| `Aircraft.py` | Speed profiles and initial speed (`init_speed`), deviation state (`deviated`, `deviation_type`, `waiting`), movement at different speeds, and distance tracking for the replanning-cost metric. |
| `cbs.py` | Collision detection for agents with different speeds: the vertex `2.x.y` and edge `2.x`/`3.x` cases, next to same-speed `vertex1`/`edge1`. `standard_splitting` turns each collision type into constraints, some of which span two time steps. `CBSSolver.find_solution` replans deviated aircraft from their current or next node while keeping the other aircraft's remaining plans. |
| `single_agent_planner.py` | `complex_single_agent_astar`: space-time A* with speed-dependent step times (`find_time_step_size`), a constraint table (`build_constraint_table`, `is_constrained`), and backtracking prevention based on the path flown so far. |
| `run_me.py` | Wiring for deviation checks and replanning in the loop, random speed profiles at spawn, repeated runs, and collection of metrics (CPU time mean and variance, cost of replanning, number of replannings) into `cpu_times.csv`. |
| `cpu_times.csv`, `cpu_times_no_dev.csv` | Experiment data with and without deviations. |
| `tests.ipynb`, `figures/` | Statistical analysis (including Mann–Whitney U tests) and every figure in the report. |
