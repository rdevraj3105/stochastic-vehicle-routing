# A Multi-Scale Path Planning Optimization Framework for Planetary Rovers in Uncertain Terrain

### Hybrid Optimization + Adaptive Planning Framework

This project develops a **multi-layer optimization and planning framework** for autonomous planetary rover routing under terrain uncertainty in which the rover must visit a set of scientific target sites. It integrates **graph search (A\*)**, **mixed-integer linear programming (MILP)**, and a **stochastic MDP-based local planner** to handle incomplete and noisy terrain information.

---

## Core Idea

We separate the routing problem into three layers:

1. **Global Path Cost Estimation (A\*)**
2. **Route Optimization (MILP / TSP variant)**
3. **Local Adaptive Planning (MDP on refined grid)**

This decomposition allows us to:

* Maintain **global optimal structure**
* Adapt to **local terrain uncertainty**

## Architecture Overview

### Layer 1 — Global Planning

* Computes shortest paths between all target sites on a **coarse elevation grid** using the A* algorithm
* Edge costs:
  $c = mg \cdot \Delta \text{elevation}$
* Produces:

  * Pairwise cost matrix
  * Coarse paths between targets

---

### **Layer 2 — MILP Route Optimization**

* Solves a **fixed-start, open TSP (Hamiltonian path)**
* Implemented in **Pyomo + Gurobi**
* Uses:

  * Binary decision variables ( $x_{ij}$ )
  * MTZ subtour elimination

**Key Feature:**

* The endpoint is **not fixed** — it emerges from optimization

---

### **Layer 3 — Adaptive Local Planning (MDP)**

* Refines each segment using a **fine-resolution grid**
* Uses:
  * **Bilinear interpolation** for terrain
  * **Stochastic penalty** for interpolation uncertainty

#### Cost Model

$c(s, s') = mg \cdot |Z(s') - Z(s)| + \phi$


$\phi \sim \mathcal{N}(\mu, \sigma^2)$

---


## Features

* A* pathfinding on elevation grids
* MILP-based routing (Pyomo + Gurobi)
* Adaptive grid refinement
* Bilinear interpolation of terrain
* Stochastic modeling of terrain uncertainty
* Obstacle injection for robustness testing



## How to Run

### 1. Install dependencies

```bash
pip install pyomo numpy scipy
```

You will also need:

* Gurobi installed and licensed

---
## Research Contributions

This work introduces a **hybrid deterministic–stochastic planning framework** that:

* Bridges **global combinatorial optimization** with **local stochastic control**
* Avoids full stochastic MILP formulations (which are intractable)
* Uses adaptive replanning 
* Provides a scalable approach for:

  * Planetary exploration
  * Autonomous navigation
  * Robotics under uncertainty

---

