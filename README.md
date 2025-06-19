# stochastic-vehicle-routing
This research project (part of the [**Space Systems Optimization Group** (SSOG) at Georgia Tech](https://ssog.ae.gatech.edu/) focuses on optimizing surface routes for a Mars rover using elevation-based terrain costs. A deterministic **Mixed-Integer Linear Programming (MILP)** model is used to compute minimum-cost tours that account for energy expenditure due to elevation changes. The model is built in **Pyomo** and solved using **Gurobi**.

A stochastic extension of the model — where the rover re-plans routes as uncertain terrain costs are gradually revealed — is currently in progress. This work leverages operations research, data science, probabilistic methods, and space systems principles to advance efficient navigation strategies for Mars rovers, supporting more effective space exploration campaigns.

