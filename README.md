# Conflict-Based Search with Speed Deviations for Airport Taxiing

Bachelor's thesis, BSc Artificial Intelligence, VU Amsterdam (Axel Ehrnrooth, 2024).

This project adds **speed deviations** to multi-agent path finding for taxiing aircraft: aircraft speed up or slow down partway along their route. It also extends **Conflict-Based Search (CBS)** so it can detect the resulting mixed-speed conflicts and replan around them.

📄 **Report:** [`docs/thesis.pdf`](docs/thesis.pdf). It covers the method, experiments, results and discussion.

## Repository structure

```
thesis/               The thesis code: the airport taxiing simulation with deviations and the extended CBS
  run_me.py           Entry point: simulation loop, spawning, experiment data collection
  deviate.py          Speed profiles, deviation detection, replanning preparation
  cbs.py              CBS with same-speed and mixed-speed collision types and constraints
  single_agent_planner.py   Speed-aware space-time A* (CBS low level)
  Aircraft.py, visualization.py, nodes.xlsx, edges.xlsx   Agent model, pygame view, airport layout
  cpu_times.csv       Experiment results with deviations
  cpu_times_no_dev.csv      Benchmark results without deviations
  tests.ipynb         Statistical analysis and plots of the experiment data
  figures/            Figures used in the report
docs/                 The thesis report
archive/              Course material and baseline code this work builds on (not thesis output)
```

## Provided vs. my work

| Source | Contents |
|---|---|
| TU Delft AE4422-20 ABMS course | Simulation framework: layout, visualization, aircraft movement, single-agent A* skeleton |
| Group 6 (F. Sirghi & V. van Gorp) | Graph-based CBS for same-speed aircraft, archived in [`archive/baseline-group6/`](archive/baseline-group6/) |
| **This thesis** | Speed profiles and deviations, multi-speed conflict detection and constraints in CBS, deviation-triggered replanning, experiments, data, analysis and figures |

See [CONTRIBUTIONS.md](CONTRIBUTIONS.md) for a per-file breakdown.

## Running

```bash
pip install -r requirements.txt
cd thesis            # must run from here: layout and data files are loaded from the working directory
python run_me.py
```

Simulation settings are at the top of `run_me.py`. Each run appends a row to `cpu_times.csv`.

## Credits

- Simulation framework: course AE4422-20 *Agent-based Modelling and Simulation in Air Transport*, TU Delft.
- Baseline CBS implementation: Florina Sirghi and Vincent van Gorp (AE4422-20, group 6, 2021/22).
