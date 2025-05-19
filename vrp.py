import pyomo.environ as pyo
import numpy as np
from pyomo.opt import SolverFactory
import csv

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
    return dimX, dimY, f


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
        
        if (x, y, k, l) in costs.keys() and (x, y) != startNode and (k, l) != startNode:
            return model.u[x, y] + 1 >= model.u[k, l] + X * Y * (1 - model.z[x, y, k, l])
        return pyo.Constraint.Skip

    model.subtour_elimination = pyo.Constraint(model.x_start, model.y_start, model.x_end, model.y_end, rule=subtour_elimination_rule)
    
    return model

if __name__ == "__main__":
    """ opt = SolverFactory("gurobi", tee=True)
    costMatrix = np.array([[80, 1, 2, 3],
                           [80, 1, 4, 5],
                           [80, 4, 3, 6],
                           [3, 5, 6, 4]])
    model = rover_routing_model(4, costMatrix, 0, 0)
    results = opt.solve(model)
    

    print("Solver metrics")
    print(results)
    print("\nDecision Variables")
    for i in model.nodes:
       t for j in model.nodes:
            if model.x[i,j] is not None:
                print(i,j, model.x[i,j].value) """
    

    


    #Mars Data CSV file
    with open('marsCuriosity.csv', 'r') as mars_csv:
        csv_reader = csv.reader(mars_csv)
        lines = list(csv_reader)

    # Writing to a .txt file
    with open('marsCuriosity.txt', 'w') as txt_file:
        for line in lines[1:]:
            txt_file.write(f"{float(line[0]):.2f} {float(line[1]):.2f} {float(line[2]):.2f}\n")
    
   

    opt = SolverFactory("gurobi", tee=True)
    X, Y, forces = elevationCosts("elevationdata.txt")

    
    model = rover_routing_model(X, Y, forces, startNode = (0,0), endNode = (X - 1, Y-1))
    results = opt.solve(model, options={"OutputFlag": 1})
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
    


    
    

        
    

