import pyomo.environ as pyo
import numpy as np
import time
from pyomo.opt import SolverFactory
import csv
import heapq

def elevationCosts(dataFile):
    with open(dataFile, "r") as file:
        lines = file.readlines()

    
    data = np.zeros((len(lines), 3))

    
    for i, row in enumerate(lines):
        for k,j in enumerate(row.split()):
            #print(float(j))
            data[i,k] = float(j)  

    x = np.unique(data[:, 0])
    y = np.unique(data[:, 1])
    


    dimX,dimY = len(y),len(x)
    

    
    Z = data[:, 2].reshape(len(y), len(x))

    #print(Z)

    m = 1.0    # mass in kg
    g = 9.81   # gravitational acceleration in m/s²

    # Initialize force array
    f = {} 

    

    # Loop through the coordinates within bounds to calculate forces
    for i in range(Z.shape[0]):
        for j in range(Z.shape[1]):
            if i == 0:
                dx_ls = [0, 1]
            elif i == Z.shape[0]-1:
                dx_ls = [-1, 0]
            else:
                dx_ls = [-1, 0, 1]

            if j == 0:
                dy_ls = [0, 1]
            elif j == Z.shape[1]-1:
                dy_ls = [-1, 0]
            else:
                dy_ls = [-1, 0, 1]

            for dx in dx_ls:
                for dy in dy_ls:
                    f[(i, j, i + dx, j + dy)] = abs(m * g * (Z[i + dx, j + dy] - Z[i, j]))


    #print(len(x))
    #print(len(y))
    return dimX, dimY, f, Z


def rover_routing_model(X, Y, costs, startNode, endNode):
    """
    Creates a Pyomo model for a small deterministic Mars rover vehicle routing problem.
    The cost matrix c is represented as a dictionary 'cost_dict' where keys are tuples (i, j)
    representing edges, and values are the travel costs between nodes i and j.

    Parameters:
    - nodes (int): Number of nodes (1 depot and n-1 nodes/locations).
    - costs (list): Cost matrix representing travel costs between each pair of nodes.

    
    Returns:
    - model (ConcreteModel): A Pyomo model for the vehicle routing problem.
    """
    
    model = pyo.ConcreteModel()
    #print(X,Y)
    model.x_start = pyo.RangeSet(0, X -1)
    model.y_start = pyo.RangeSet(0, Y -1)
    
    model.x_end = pyo.RangeSet(0, X -1)
    model.y_end = pyo.RangeSet(0, Y -1)

    # i = range(costs.shape()[0])
    # j = range(costs.shape()[1])
    
  
    #print([key for key in costs.keys() if 30 in key])

    model.z = pyo.Var(costs.keys(), within=pyo.Binary)

    def obj_rule(model):
        return sum(model.z[index] * costs[index] for index in costs.keys())
    model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

    """    def leave_onceRule(model, x, y):
        
       # Ensure that each node except the endNode is left exactly once.
        
        if (x, y) == endNode:
            return pyo.Constraint.Skip  # Skip the end node since it doesn't leave anywhere
        else:
            valid_edges = [(x, y, x1, y1) for x1 in model.x_end for y1 in model.y_end if (x, y, x1, y1) in costs.keys()]
            if not valid_edges:  # If there are no valid edges, skip the constraint
                return pyo.Constraint.Skip
            return sum(model.z[(x,y)] for (edge) in valid_edges) == 1
    """
    def leave_once_rule(model, x, y):
        if (x,y) == endNode:
            return sum(sum(model.z[x,y,k,l] for k in model.x_end if (x, y, k, l) in costs.keys()) for l in model.y_end) == 0 # Skip the end node since it doesn't leave anywhere
        elif (x,y) == startNode:
            return sum(sum(model.z[x,y,k,l] for k in model.x_end if (x, y, k, l) in costs.keys()) for l in model.y_end) == 1
        else:
            return sum(sum(model.z[x,y,k,l] for k in model.x_end if (x, y, k, l) in costs.keys()) for l in model.y_end) <= 1
 
        
        # Ensure there are valid edges to move forward
        #valid_edges = [(x, y) for y in model.x_end if (x, y) in costs.keys()]
        
        """ if not valid_edges:
            return pyo.Constraint.Skip """  # Skip the constraint if no valid edges
        
        # Return the constraint expression
       

    model.leave_once = pyo.Constraint(model.x_start, model.y_start, rule=leave_once_rule)
 
    

    """ def arrive_once_rule(model, x, y):
        
       # Ensures that each node (except the startNode) is entered exactly once.
        
        if (x, y) == startNode:
            return pyo.Constraint.Skip  # Skip the start node since it doesn't enter from anywhere
        else:
            valid_edges = [(x1, y1, x, y) for x1 in model.x_end for y1 in model.y_end if (x1, y1, x, y) in costs.keys()]
            if not valid_edges:  # If there are no valid edges, skip the constraint
                return pyo.Constraint.Skip
            return sum(model.z[edge] for edge in valid_edges) == 1 """
    
    def arrive_once_rule(model, x, y):
        if (x,y) == startNode:
            return sum(sum(model.z[k,l,x,y] for k in model.x_end if (k, l, x, y) in costs.keys()) for l in model.y_end) == 0   # Skip the start node since it doesn't enter from anywhere
        elif (x,y) == endNode:
            return sum(sum(model.z[k,l,x,y] for k in model.x_end if (k, l, x, y) in costs.keys()) for l in model.y_end) == 1
        else:
            return sum(sum(model.z[k,l,x,y] for k in model.x_end if (k, l, x, y) in costs.keys()) for l in model.y_end) <= 1
    """     # Ensure there are valid edges to move from
        valid_edges = [(x, y) for x in model.x_start if (x, y) in costs.keys()]
        
        if not valid_edges:
            return pyo.Constraint.Skip  # Skip the constraint if no valid edges
         """
        # Return the constraint expression
   
    model.arrive_once = pyo.Constraint(model.x_end, model.y_end, rule=arrive_once_rule)   


    #model.arrive_once = pyo.Constraint(model.x_start, model.y_start, rule= arrive_once_rule)

    def enter_must_leave_rule(model, x, y):
        if (x,y) == startNode or (x,y) == endNode:
            return pyo.Constraint.Skip
        else:
            influx = sum(sum(model.z[x,y,k,l] for k in model.x_end if (x,y,k,l) in costs.keys() and (x,y) != (k,l)) for l in model.y_end)
            outflux = sum(sum(model.z[k,l,x,y] for k in model.x_end if (k,l,x,y) in costs.keys() and (x,y) != (k,l)) for l in model.y_end)
            return influx == outflux
            
            
    model.enter_must_leave_rule = pyo.Constraint(model.x_end, model.y_end, rule = enter_must_leave_rule)

    #Subtour Elimination Constraints (MTZ)
    #Let model.u represent the sequences of visits for each node
    model.u = pyo.Var(model.x_start, model.y_start, within=pyo.NonNegativeIntegers, bounds=(1, X * Y))

    # Ensure start node is first in sequence
    model.u[startNode] = 1
    def subtour_elimination_rule(model, x, y, k, l):
        if (x, y, k, l) in costs.keys() and (x, y) != startNode and (k, l) != startNode:
            return model.u[x, y] + 1 <= model.u[k, l] + X * Y * (1 - model.z[x, y, k, l])
        
        #if (x, y, k, l) in costs.keys() and (x, y) != startNode and (k, l) != startNode:
            #return model.u[x, y] + 1 >= model.u[k, l] + X * Y * (1 - model.z[x, y, k, l])
        return pyo.Constraint.Skip

    model.subtour_elimination = pyo.Constraint(model.x_start, model.y_start, model.x_end, model.y_end, rule=subtour_elimination_rule)
    
    return model


""" 
A* algorithm implementation
"""

def heuristic(current, goal, min_cost_estimate=1.0):
    """
    Admissible heuristic: Manhattan distance scaled by minimum possible cost
    """
    manhattan_dist = abs(current[0] - goal[0]) + abs(current[1] - goal[1])
    return manhattan_dist * min_cost_estimate


def a_star(X, Y, Z, start, goal, forces_dict=None):
    m, g = 1.0, 9.81
    
    # Calculate minimum cost for heuristic
    if forces_dict:
        min_cost_estimate = min(forces_dict.values()) if forces_dict else 1.0
    else:
        min_cost_estimate = 1.0
    
    # Priority queue: (f_score, g_score, current, path)
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal, min_cost_estimate), 0, start, [start]))
    
    # Track best cost to each node
    best_cost = {start: 0}
    closed_set = set()
    nodes_explored = 0
    
    while open_set:
        f_score, cost_so_far, current, path = heapq.heappop(open_set)
        nodes_explored += 1
        
        # Skip if we've already found a better path to this node
        if current in closed_set:
            continue
            
        closed_set.add(current)
        
        if current == goal:
            return path, cost_so_far, nodes_explored

        i, j = current
        for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            ni, nj = i + di, j + dj
            
            
            if 0 <= ni < X and 0 <= nj < Y:
                if forces_dict and (i, j, ni, nj) in forces_dict:
                    move_cost = forces_dict[(i, j, ni, nj)]
                else:
                    move_cost = abs(Z[ni][nj] - Z[i][j]) * m * g
                
                new_cost = cost_so_far + move_cost
                
                if (ni, nj) not in best_cost or new_cost < best_cost[(ni, nj)]:
                    best_cost[(ni, nj)] = new_cost
                    priority = new_cost + heuristic((ni, nj), goal, min_cost_estimate)
                    heapq.heappush(open_set, (priority, new_cost, (ni, nj), path + [(ni, nj)]))
                    
    return None, float('inf'), nodes_explored


def a_star_fixed(X, Y, Z, start, goal, forces_dict):
    """
    Fixed A* implementation that matches the MILP formulation more clearly
    """
    # Calculate minimum cost for heuristic
    min_cost_estimate = min(forces_dict.values()) if forces_dict else 1.0
    
    # Priority queue: (f_score, g_score, current, path)
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal, min_cost_estimate), 0, start, [start]))
    
    # Track best cost to each node
    best_cost = {start: 0}
    closed_set = set()
    nodes_explored = 0
    
    while open_set:
        f_score, cost_so_far, current, path = heapq.heappop(open_set)
        nodes_explored += 1
        
        if current in closed_set:
            continue
            
        closed_set.add(current)
        
        if current == goal:
            return path, cost_so_far, nodes_explored

        i, j = current
        
        # Use the same neighbor generation as MILP and check bounds like in elevationCosts function
        if i == 0:
            dx_ls = [0, 1]
        elif i == X-1:  
            dx_ls = [-1, 0]
        else:
            dx_ls = [-1, 0, 1]

        if j == 0:
            dy_ls = [0, 1]
        elif j == Y-1:  
            dy_ls = [-1, 0]
        else:
            dy_ls = [-1, 0, 1]

        for dx in dx_ls:
            for dy in dy_ls:
                if dx == 0 and dy == 0:  # Skip self
                    continue
                    
                ni, nj = i + dx, j + dy
                
                # Use exact same cost as MILP
                if (i, j, ni, nj) in forces_dict:
                    move_cost = forces_dict[(i, j, ni, nj)]
                else:
                    continue  # Skip if no valid edge
                
                new_cost = cost_so_far + move_cost
                
                if (ni, nj) not in best_cost or new_cost < best_cost[(ni, nj)]:
                    best_cost[(ni, nj)] = new_cost
                    priority = new_cost + heuristic((ni, nj), goal, min_cost_estimate)
                    heapq.heappush(open_set, (priority, new_cost, (ni, nj), path + [(ni, nj)]))
                    
    return None, float('inf'), nodes_explored

def naive_baseline_cost(X, Y, Z):
    m = 1.0
    g = 9.81
    cost = 0
    path = []

    for j in range(Y):
        # Move left to right if even row, right to left if odd row (snake pattern)
        if j % 2 == 0:
            # left to right along the row j
            for i in range(X):
                path.append((j, i))
                if i > 0:
                    delta = Z.get((j, i - 1, j, i), 0)
                    cost += m * g * delta
            # Move down one row if not last row
            if j < Y - 1:
                delta = Z.get((j, X - 1, j + 1, X - 1), 0)
                cost += m * g * delta
                path.append((j + 1, X - 1))
        else:
            # right to left along the row j
            for i in range(X - 1, -1, -1):
                path.append((j, i))
                if i < X - 1:
                    delta = Z.get((j, i + 1, j, i), 0)
                    cost += delta
            # Move down one row if not last row
            if j < Y - 1:
                delta = Z.get((j, 0, j + 1, 0), 0)
                cost += m * g * delta
                path.append((j + 1, 0))

    return cost, path


def rolling_horizon_optimization(X, Y, Z_true, Z_estimate, horizon = 3, start = (0,0), goal = None):
    # if goal isn't specified, default plan to top-right corner
    if goal is None:
        goal = (X-1, Y-1)

    path = [start]
    current = start
    total_cost = 0
    opt = SolverFactory("gurobi", tee=True)
    
    while current != goal:
        visible_nodes = []
        for dx in range(-horizon, horizon + 1):
            for dy in range(-horizon, horizon + 1):
                next_X, next_Y = current[0] + dx, current[1] + dy
                if 0 <= next_X < X and 0 <= next_Y < Y:
                    visible_nodes.append((next_X, next_Y))

        # only move within the visible window 
        for (i, j) in visible_nodes:
            for (di, dj) in [(-1,0), (1,0), (0,-1), (0,1)]:
                next_I, next_J = i + di, j + dj
                if 0 <= next_I < X and 0 <= next_J < Y:
                    Z_estimate[i, j, next_I, next_J] = abs(Z_true[next_I][next_J] - Z_true[i][j]) * 9.81

        # rebuild costs with updated info using only adjacent nodes
        costs = {}
        for (i, j, k, l), val in Z_estimate.items():
            if (abs(i-k) + abs(j-l)) == 1:
                costs[(i, j, k, l)] = val
        
        model = rover_routing_model(X, Y, costs, startNode = current, endNode = goal)
        results = opt.solve(
            model,
            #"MIPGap": 0.05,
            #"TimeLimit": 10,
            tee=True)

        next_step = None

        # extract edge selected by the model from the current location
        for z_var in model.z:
            if pyo.value(model.z[z_var]) >= 0.5:
                if z_var[:2] == current:
                    next_step = z_var[2:]
                    break
        if not next_step:
            print("No valid next step found: ", current)
            break
        

        costs = costs[(current[0], current[1], next_step[0], next_step[1])]
        total_cost += costs
        current = next_step
        path.append(current)

    return path, total_cost, results


def run_deterministic():
    
    #Mars Data CSV file
    with open('marsCuriosity.csv', 'r') as mars_csv:
        csv_reader = csv.reader(mars_csv)
        lines = list(csv_reader)

    # Writing to a .txt file
    with open('marsCuriosity.txt', 'w') as txt_file:
        for line in lines[1:]:
            txt_file.write(f"{float(line[0]):.2f} {float(line[1]):.2f} {float(line[2]):.2f}\n")
    
   

    opt = SolverFactory("gurobi", tee=True)
    X, Y, forces, Z = elevationCosts("elevationdata.txt")
    
    total_nodes = X * Y
    print(f"Total nodes: {total_nodes}")

    
    model = rover_routing_model(X, Y, forces, startNode = (0,0), endNode = (X - 1, Y-1))

    start_time = time.time()

    results = opt.solve(model, options={"OutputFlag": 1})

    end_time = time.time()
    solver_runtime = end_time - start_time
    print(results)
    for var in model.component_objects(pyo.Var, active=True):
        if var.name == "z":
            for index in var:
                value =var[index].value
                if value >= 0.5:
                    print("varname: {}, value :{}".format(var.name, var[index]))

    with open("vrp_output.txt", "w") as f:
        for var in model.component_objects(pyo.Var, active=True):
            if var.name == "z":
                for index in var:
                    value = var[index].value
                    if value >= 0.5:
                        f.write(f"{index}\n")

    
    #test comparisons
    optimized_cost = pyo.value(model.obj)
    print(f"Optimized total elevation cost: {optimized_cost:.4f}")
    print(f"Solver runtime (seconds): {solver_runtime:.4f}")

    baseline_cost, baseline_path = naive_baseline_cost(X, Y, forces)
    print(f"Naive baseline routing cost: {baseline_cost:.4f}")

    # % improvement over baseline
    cost_reduction = (baseline_cost - optimized_cost) / baseline_cost * 100
    print(f"Cost reduction compared to baseline: {cost_reduction:.2f}%")

    with open("baseline_cost.txt", 'w') as f:
        baseline_cost, path = naive_baseline_cost(X, Y, forces)
        for coord in path:
            f.write(f"{coord}\n")
    
    num_vars = len([v for v in model.component_data_objects(pyo.Var)])
    num_constraints = len([c for c in model.component_data_objects(pyo.Constraint)])

    print(f"Number of variables: {num_vars}")
    print(f"Number of constraints: {num_constraints}")


def run_stochastic():
    X, Y, forces, Z = elevationCosts("elevationdata.txt")

    # Initial noisy estimates
    Z_estimate = {}
    for i in range(X):
        for j in range(Y):
            for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                next_i, next_j = i + di, j + dj
                if 0 <= next_i < X and 0 <= next_j < Y:
                    true_cost = abs(Z[i][j] - Z[next_i][next_j]) * 9.81
                    noisy_estimate = true_cost + np.random.normal(0, 0)
                    Z_estimate[i, j, next_i, next_j] = max(0, noisy_estimate)

    path, cost, results = rolling_horizon_optimization(X, Y, Z, Z_estimate, horizon=3)

    print("Final adaptive path:", path)
    print("Total rolling horizon cost:", cost)


if __name__ == "__main__":
    run_deterministic()
    X, Y, forces, Z = elevationCosts("elevationdata.txt")
    
    start = (0, 0)
    goal = (X-1, Y-1)

    astar_path, astar_cost, nodes_explored = a_star(X, Y, Z, start, goal, forces)
    print("A* path:", astar_path)
    print("A* total cost:", astar_cost)
    print(nodes_explored)

    astar_path_fixed, astar_cost_fixed, nodes_explored_fixed = a_star_fixed(X, Y, Z, start, goal, forces)
    print("A* fixed path:", astar_path_fixed)
    print("A* fixed total cost:", astar_cost_fixed)
    print(nodes_explored_fixed)
    
    