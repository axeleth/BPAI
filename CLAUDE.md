# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Bachelor project (AI BSc, VU Amsterdam): Multi-Agent Path Finding (MAPF) for airport taxiing, using Conflict-Based Search (CBS) with aircraft that move at **different speeds** and **deviate** from their plans, triggering replanning. `README.md` and `CONTRIBUTIONS.md` document what is the author's work vs. the provided baseline (TU Delft AE4422 course framework + group 6's CBS); the submitted version is tagged `v1.0-thesis-submission`.

**The thesis code is `thesis/`.** `archive/` is reference material, not thesis output:
- `archive/mapf-tutorial/` – the grid-world MAPF warm-up exercise (classic CBS/prioritized/independent on `code/instances/*.txt`) and reference answers.
- `archive/baseline-group6/` – the baseline CBS solution the thesis `cbs.py` was built from. Don't edit it.

## Running

No build system, linter, or test suite. Install dependencies with `pip install -r requirements.txt`.

Final simulation — **must be run from its own directory**, because `nodes.xlsx`/`edges.xlsx` are loaded via `os.getcwd()` and `cpu_times.csv` is a relative path:
```bash
cd thesis
python run_me.py
```
Parameters are module-level constants at the top of `run_me.py` (`simulation_time`, `planner`, `visualization`, `visualization_speed`, `plot_graph`) plus `number_of_aircraft` and the `while runnumb <= 5` loop further down. Each run **prepends a row to `cpu_times.csv`** (aircraft count, mean/variance CPU time, cost of replanning, number of replannings) — running the script mutates experiment data. `tests.ipynb` is not tests; it's the statistical analysis/plotting of `cpu_times*.csv` that produced the PDFs in `thesis/figures/` (its `savefig` calls write to the working directory, not `figures/`). Set `visualization = False` for headless batch runs.

Warm-up grid exercise:
```bash
cd archive/mapf-tutorial/code
python run_experiments.py --instance instances/test_1.txt --solver CBS        # animated
python run_experiments.py --instance "instances/test_*" --solver Prioritized --batch   # writes results.csv
```

## Architecture of the final build

**Simulation loop (`run_me.py`)** – fixed-step loop with `dt = 0.05`. Aircraft spawn times are drawn randomly up front; at each spawn time an arriving (`A`, start nodes 37/38 → gates) or departing (`D`, gates 34/35/36/97/98 → runway entries 1/2) `Aircraft` is created with a random speed profile. Every 0.5 time units `check_deviation` may change an aircraft's speed; `detect_deviation` then compares actual positions against plans. CBS is invoked whenever a new aircraft spawns **or** any aircraft deviated. Only `planner = "CBS"` is wired into the loop — `independent.py`/`prioritized.py` are leftover stubs.

**Agent identity invariant** – an aircraft's `id` is its index in the sorted spawn-time list, and it doubles as the index into `aircraft_lst`, `existent_paths`, `paths_to_go`, `starts`, and `goals` (e.g. `ac_list[agent1]` in `cbs.detect_collision`). Preserve this when changing spawning.

**Time/speed model** – speeds are only 1 or 2; one graph edge (node step) takes 0.5 time units at speed 1 and 0.25 at speed 2 (`find_time_step_size`). Paths are lists of `(node_id, timestep, ac_id)` with float timesteps. Many functions branch explicitly on `speed == 1` / `speed == 2` and hard-code 0.25 offsets, so adding another speed touches `deviate.py`, `cbs.py`, and `single_agent_planner.py`.

**Speed profiles / deviations (`Aircraft.py`, `deviate.py`)** – profiles `fast`, `slow` (constant), `cS` (cautious start, departures: speeds up after 1/3 of path), `cE` and `fS` (arrivals: slow down near end / after 1/4). A deviation is an aircraft reaching its next node at the wrong time (`deviation_type` = `"early"`/`"late"`). `prep_CBS_replan` converts each taxiing aircraft's position into a replan start `(node, time, id)`; aircraft between nodes replan from their next node at `t + 0.25`.

**CBS (`cbs.py`)** – `CBSSolver.find_solution` builds the root from (a) fresh A* plans for aircraft spawning now, (b) replans for deviated aircraft, and (c) the remaining plans of everyone else, then runs standard CBS high-level search. Collision detection distinguishes same-speed cases (`vertex1`, `edge1`) from many mixed-speed cases (`vertex2.x.y`, `edge2.x`, `edge3.x`), and `standard_splitting` maps each type to constraints `{'agent', 'loc', 'timestep'}` — `loc` of length 1 is a vertex constraint, length 2 an edge constraint. The search raises after 8000 generated nodes. Mutable lists (`constraints`, `existent_paths`, `collisions`, `original_path_lst`) are passed in from `run_me.py` and mutated in place across calls.

**Low-level search (`single_agent_planner.py`)** – `calc_heuristics` precomputes all-pairs Dijkstra distances on the networkx graph. `complex_single_agent_astar` is the space-time A* used by CBS (constraint table + a no-immediate-backtracking check using `path_so_far`); `simple_single_agent_astar` is the unconstrained version used by `Aircraft.plan_independent` for the baseline "original path" length that the replanning-cost metric compares against.

**Layout** – `nodes.xlsx` (id, x_pos, y_pos, type ∈ `rwy_d`/`rwy_a`/`gate`/…) and `edges.xlsx` (from, to, length) define a directed graph. Node ids are hard-coded in `run_me.py` (spawn/goal sets, and a `range(1, 109)` scan to map positions to nodes), so changing the layout requires updating those. `visualization.py` is the pygame renderer (`map_initialization`, `map_running`).
