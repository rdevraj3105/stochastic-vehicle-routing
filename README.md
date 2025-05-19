# stochastic-vehicle-routing
This research project focuses on optimizing surface routes for a Mars rover using elevation-based terrain costs. A Mixed-Integer Linear Programming (MILP) model is used to compute minimum-cost tours that account for energy expenditure due to elevation changes on the Martian surface. The model is built in Pyomo (a python based optimization language) and solved using Gurobi.

A stochastic extension of the model — where the rover re-plans routes as uncertain terrain costs are gradually revealed — is currently in progress.

## Features
MILP routing with elevation-based movement costs

MTZ-based subtour elimination

Rolling horizon planning (in progress)

Stochastic modeling with uncertain terrain (in progress)
