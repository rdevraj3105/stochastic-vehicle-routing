# stochastic-vehicle-routing
This research project (part of the [**Space Systems Optimization Group** (SSOG) at Georgia Tech](https://ssog.ae.gatech.edu/)) investigates energy-efficient surface navigation for planetary rovers under terrain uncertainty. The goal is to combine graph search, global optimization, and rolling-horizon control into a unified planning framework that supports long-horizon mission objectives while remaining responsive to locally revealed terrain information.

The project combines operations research, space systems modeling, and stochastic planning to study how global route optimization and local replanning can be integrated for long-horizon autonomous exploration.

## System Architecture
The planning framework follows a three-layer architecture that separates cost estimation, global sequencing, and local execution.

**1. Cost Generation via A\* search**

The Martian surface is discretized into a terrain graph with elevation-based traversal costs. A* search is run between the start node and each target site, as well as between all pairs of target sites, to generate a pairwise cost matrix. These costs represent nominal energy-optimal paths between mission-relevant locations and abstract away low-level navigation details.

**2. Global Planning via MILP (Nominal Path)**

The pairwise costs from the A* layer are used as inputs to a Mixed-Integer Linear Programming (MILP) formulation that determines the optimal visit sequence over the target sites. This layer captures mission-level constraints (e.g., tour feasibility and sequencing) and produces a deterministic, globally optimal nominal plan under fully known terrain assumptions.
The MILP is implemented in Pyomo and solved using Gurobi.

**3. Local Planning Under Uncertainty via MPC (In Progress)**

To model realistic sensing limitations, terrain costs are treated as uncertain and only locally observable. The rover executes the nominal plan using a Model Predictive Control (MPC) strategy:

* A prediction horizon (e.g., 10 steps) is used to reason about future uncertainty.

* Only a short control horizon (e.g., 3 steps) is executed.

* As the rover moves, true terrain costs are revealed, uncertainty is updated, the horizon is shifted forward, and a local subproblem is re-optimized.

