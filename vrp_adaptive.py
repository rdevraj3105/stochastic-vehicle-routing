import pyomo.environ as pyo
import numpy as np
import time
from pyomo.opt import SolverFactory
import csv
import heapq
import random 
from scipy import stats

def elevationCosts(dataFile):
    with open(dataFile, "r") as file:
        lines = file.readlines()

    data = np.zeros((len(lines), 3))
    for i, row in enumerate(lines):
        for k, j in enumerate(row.split()):
            data[i, k] = float(j)  


    x = np.unique(data[:, 0])
    y = np.unique(data[:, 1])
    dimX, dimY = len(y), len(x)
    Z = data[:, 2].reshape(len(y), len(x))
   
    #obstacles
    # for _ in range(20):
    #    i, j = random.randint(0, Z.shape[0]-1), random.randint(0, Z.shape[1]-1)
    #    Z[i, j] = 1e6


    m, g = 1.0, 9.81
    f = {} 

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
                    if dx == 0 and dy == 0:
                        continue
                    f[(i, j, i + dx, j + dy)] = abs(m * g * (Z[i + dx, j + dy] - Z[i, j]))

    return dimX, dimY, f, Z


# ==============================================================================
# MDP-BASED LOCAL PATH PLANNING WITH ADAPTIVE GRID
# ==============================================================================

class AdaptiveGridMDP:
    """
    MDP for local path planning on an adaptive grid with bilinear interpolation.
    
    The coarse grid G^c comes from orbiter data. The fine grid G^f is generated
    adaptively for local planning. Elevation values on G^f are interpolated.
    
    The interpolation penalty phi is modeled as a stochastic normal random variable
    to account for uncertainty in interpolated elevation values.
    
    Cost distribution is computed analytically (no Monte Carlo needed).
    """
    
    def __init__(self, Z_coarse, m=1.0, g=9.81, refinement_factor=2, 
                 phi_mean=0.1, phi_std=0.05):
        """
        Initialize the adaptive grid MDP.
        
        Args:
            Z_coarse: Elevation data on coarse grid (from orbiter)
            m: Mass parameter
            g: Gravity constant
            refinement_factor: How many fine-grid cells per coarse cell (in each dimension)
            phi_mean: Mean of the normal distribution for phi (expected penalty)
            phi_std: Standard deviation of the normal distribution for phi (uncertainty)
        """
        self.Z_coarse = Z_coarse
        self.m = m
        self.g = g
        self.refinement_factor = refinement_factor
        self.phi_mean = phi_mean
        self.phi_std = phi_std
        
        # Coarse grid dimensions
        self.coarse_rows, self.coarse_cols = Z_coarse.shape
        
        # Fine grid dimensions (including coarse grid nodes)
        # Each coarse cell is subdivided into refinement_factor x refinement_factor subcells
        self.fine_rows = (self.coarse_rows - 1) * refinement_factor + 1
        self.fine_cols = (self.coarse_cols - 1) * refinement_factor + 1
        
        # Spacing between fine grid nodes
        self.dx_fine = 1.0 / refinement_factor
        self.dy_fine = 1.0 / refinement_factor
        
        # Cache for interpolated elevations
        self._elevation_cache = {}
        
        # Define action space: 8-connected neighbors
        self.actions = {
            'N': (-1, 0),   # North
            'S': (1, 0),    # South
            'E': (0, 1),    # East
            'W': (0, -1),   # West
            'NE': (-1, 1),  # Northeast
            'NW': (-1, -1), # Northwest
            'SE': (1, 1),   # Southeast
            'SW': (1, -1)   # Southwest
        }
    
    def is_coarse_node(self, i, j):
        """Check if (i,j) on fine grid corresponds to a coarse grid node."""
        return (i % self.refinement_factor == 0) and (j % self.refinement_factor == 0)
    
    def fine_to_coarse_coords(self, i_fine, j_fine):
        """Convert fine grid coordinates to coarse grid coordinates."""
        i_coarse = i_fine // self.refinement_factor
        j_coarse = j_fine // self.refinement_factor
        return i_coarse, j_coarse
    
    def bilinear_interpolation(self, i_fine, j_fine):
        """
        Compute interpolated elevation at fine grid position (i_fine, j_fine).
        
        Uses bilinear interpolation based on the four surrounding coarse grid corners.
        For coarse grid nodes, returns the exact elevation value.
        """
        # Check cache first
        if (i_fine, j_fine) in self._elevation_cache:
            return self._elevation_cache[(i_fine, j_fine)]
        
        # If this is a coarse grid node, return exact value
        if self.is_coarse_node(i_fine, j_fine):
            i_coarse, j_coarse = self.fine_to_coarse_coords(i_fine, j_fine)
            elevation = self.Z_coarse[i_coarse, j_coarse]
            self._elevation_cache[(i_fine, j_fine)] = elevation
            return elevation
        
        # Find the coarse cell containing this fine node
        # Lower-left corner of the coarse cell
        i_coarse_ll = i_fine // self.refinement_factor
        j_coarse_ll = j_fine // self.refinement_factor
        
        # Clamp to valid range to handle edge cases
        # If we're at the boundary, use the last valid cell
        i_coarse_ll = min(i_coarse_ll, self.coarse_rows - 2)
        j_coarse_ll = min(j_coarse_ll, self.coarse_cols - 2)
        
        # Get the four corner elevations
        Z_00 = self.Z_coarse[i_coarse_ll, j_coarse_ll]         # Lower-left
        Z_10 = self.Z_coarse[i_coarse_ll, j_coarse_ll + 1]     # Lower-right
        Z_01 = self.Z_coarse[i_coarse_ll + 1, j_coarse_ll]     # Upper-left
        Z_11 = self.Z_coarse[i_coarse_ll + 1, j_coarse_ll + 1] # Upper-right
        
        # Compute normalized position within the coarse cell
        # How far across the cell (in fine grid units)
        i_offset = i_fine - (i_coarse_ll * self.refinement_factor)
        j_offset = j_fine - (j_coarse_ll * self.refinement_factor)
        
        # Normalized coordinates (0 to 1)
        alpha = j_offset / self.refinement_factor  # Horizontal position
        beta = i_offset / self.refinement_factor   # Vertical position
        
        # Clamp alpha and beta to [0, 1] for edge cases
        alpha = max(0.0, min(1.0, alpha))
        beta = max(0.0, min(1.0, beta))
        
        # Bilinear interpolation formula
        Z_interp = ((1 - alpha) * (1 - beta) * Z_00 +
                    alpha * (1 - beta) * Z_10 +
                    (1 - alpha) * beta * Z_01 +
                    alpha * beta * Z_11)
        
        self._elevation_cache[(i_fine, j_fine)] = Z_interp
        return Z_interp
    
    def get_elevation(self, i, j):
        """Get elevation at fine grid position (i, j)."""
        return self.bilinear_interpolation(i, j)
    
    def transition(self, state, action):
        """
        Deterministic transition function: s' = f(s, a).
        
        Args:
            state: (i, j) tuple on fine grid
            action: Action name from self.actions
        
        Returns:
            next_state: (i', j') tuple or None if out of bounds
        """
        i, j = state
        di, dj = self.actions[action]
        next_i, next_j = i + di, j + dj
        
        # Check bounds
        if (0 <= next_i < self.fine_rows and 
            0 <= next_j < self.fine_cols):
            return (next_i, next_j)
        return None
    
    def transition_probability(self, next_state, state, action):
        """
        Transition probability P(s' | s, a).
        Deterministic: 1 if s' = f(s,a), 0 otherwise.
        """
        expected_next = self.transition(state, action)
        if expected_next is None:
            return 0.0
        return 1.0 if next_state == expected_next else 0.0
    
    def get_penalty_distribution_params(self, state, next_state):
        """
        Get the distribution parameters for phi(s, s').
        
        Returns:
            (mean, std): Parameters of the normal distribution, or (0, 0) for coarse transitions
        """
        is_coarse_s = self.is_coarse_node(*state)
        is_coarse_s_prime = self.is_coarse_node(*next_state)
        
        if is_coarse_s and is_coarse_s_prime:
            return 0.0, 0.0
        
        return self.phi_mean, self.phi_std
    
    def expected_cost(self, state, next_state):
        """
        Expected movement cost E[c(s, s')].
        
        E[c(s, s')] = m*g * |Z_hat(s') - Z_hat(s)| + E[phi(s, s')]
        
        where E[phi] = phi_mean for fine grid transitions, 0 for coarse transitions
        """
        Z_s = self.get_elevation(*state)
        Z_s_prime = self.get_elevation(*next_state)
        
        elevation_cost = self.m * self.g * abs(Z_s_prime - Z_s)
        
        # Expected penalty
        mean_penalty, _ = self.get_penalty_distribution_params(state, next_state)
        
        return elevation_cost + mean_penalty
    
    def get_valid_actions(self, state):
        """Get list of valid actions from given state."""
        valid = []
        for action in self.actions:
            next_state = self.transition(state, action)
            if next_state is not None:
                valid.append(action)
        return valid
    
    def get_path_cost_distribution(self, path):
        """
        Analytically compute the exact distribution of path cost.
        
        For a path with independent Gaussian penalties phi ~ N(mu, sigma^2),
        the total cost is:
            c_total = sum(elevation_costs) + sum(phi_i)
        
        Since sum of independent Gaussians is Gaussian:
            sum(phi_i) ~ N(k*mu, k*sigma^2)  where k = number of fine transitions
        
        Therefore:
            c_total ~ N(mu_total, sigma_total^2)
        
        Args:
            path: List of states representing the path
        
        Returns:
            dict with exact distribution parameters and quantiles
        """
        if len(path) < 2:
            return {
                'mean': 0.0,
                'std': 0.0,
                'variance': 0.0,
                'num_fine_transitions': 0,
                'num_coarse_transitions': 0,
                'elevation_cost': 0.0,
                'expected_penalty_cost': 0.0
            }
        
        
        total_elevation_cost = 0.0
        num_fine_transitions = 0
        num_coarse_transitions = 0
        obstacle_penalty_cost = 0.0
        
        for i in range(len(path) - 1):
            state = path[i]
            next_state = path[i + 1]
            
            # Elevation cost (deterministic)
            Z_s = self.get_elevation(*state)
            Z_s_prime = self.get_elevation(*next_state)
            elevation_cost = self.m * self.g * abs(Z_s_prime - Z_s)
            total_elevation_cost += elevation_cost
            
            if Z_s > 1e5 or Z_s_prime > 1e5:
                obstacle_penalty_cost += elevation_cost
            else:
                total_elevation_cost += elevation_cost
            # Count transition type
            if self.is_coarse_node(*state) and self.is_coarse_node(*next_state):
                num_coarse_transitions += 1
            else:
                num_fine_transitions += 1
        
        # Exact distribution of total cost
        # c_total = total_elevation_cost + sum(phi_i)
        # where sum(phi_i) ~ N(k*phi_mean, k*phi_std^2)
        
        mean_penalty_cost = num_fine_transitions * self.phi_mean
        variance_penalty_cost = num_fine_transitions * (self.phi_std ** 2)
        std_penalty_cost = np.sqrt(variance_penalty_cost)
        
        total_mean = total_elevation_cost + mean_penalty_cost
        total_std = std_penalty_cost  # Elevation cost is deterministic
        total_variance = variance_penalty_cost
        
        # Exact quantiles from normal distribution
        # Using scipy.stats.norm for exact quantiles
        if total_std > 0:
            norm_dist = stats.norm(loc=total_mean, scale=total_std)
            q25 = norm_dist.ppf(0.25)
            q50 = norm_dist.ppf(0.50)  # median
            q75 = norm_dist.ppf(0.75)
            q95 = norm_dist.ppf(0.95)
            ci_lower = norm_dist.ppf(0.025)  # 95% CI
            ci_upper = norm_dist.ppf(0.975)
        else:
            # No variance (all coarse transitions)
            q25 = q50 = q75 = q95 = total_mean
            ci_lower = ci_upper = total_mean
        
        return {
            'mean': total_mean,
            'std': total_std,
            'variance': total_variance,
            'median': q50,
            'q25': q25,
            'q75': q75,
            'q95': q95,  # 95th percentile (VaR)
            'ci_lower': ci_lower,  # 95% confidence interval
            'ci_upper': ci_upper,
            'num_fine_transitions': num_fine_transitions,
            'num_coarse_transitions': num_coarse_transitions,
            'elevation_cost': total_elevation_cost,
            'expected_penalty_cost': mean_penalty_cost,
            'pure_elevation_cost':total_elevation_cost,
            'obstacle_penalty_cost': obstacle_penalty_cost
        }
    
    def print_grid_info(self):
        """Print information about the adaptive grid."""
        print("\n" + "="*70)
        print("ADAPTIVE GRID MDP INFORMATION")
        print("="*70)
        print(f"Coarse grid size: {self.coarse_rows} x {self.coarse_cols}")
        print(f"Fine grid size: {self.fine_rows} x {self.fine_cols}")
        print(f"Refinement factor: {self.refinement_factor}")
        print(f"Fine grid spacing: dx={self.dx_fine:.3f}, dy={self.dy_fine:.3f}")
        print(f"Interpolation penalty phi ~ N({self.phi_mean:.4f}, {self.phi_std:.4f}^2)")
        print(f"  Expected phi: {self.phi_mean:.4f}")
        print(f"  Phi std dev: {self.phi_std:.4f}")
        print(f"  Phi variance: {self.phi_std**2:.6f}")
        print(f"Total fine grid nodes: {self.fine_rows * self.fine_cols}")
        print(f"Total coarse grid nodes: {self.coarse_rows * self.coarse_cols}")
        print("\nNote: Path cost distributions computed analytically (exact, not sampled)")
        print("="*70 + "\n")




# ==============================================================================
# A* WITH MDP COSTS (for local path planning)
# ==============================================================================

def a_star_mdp(mdp, start, goal):
    """
    A* search on the adaptive grid using MDP expected cost function.
    
    Plans using E[c(s,s')] to find optimal path under expected costs.
    The actual realized cost will be stochastic due to phi ~ N(mu, sigma^2).
    
    Args:
        mdp: AdaptiveGridMDP instance
        start: Start state (i, j) on fine grid
        goal: Goal state (i, j) on fine grid
    
    Returns:
        path: List of states from start to goal
        expected_cost: Expected total path cost
        nodes_explored: Number of nodes explored
    """
    def heuristic(current, goal):
        # Admissible heuristic: straight-line distance * minimum elevation cost
        di = abs(current[0] - goal[0])
        dj = abs(current[1] - goal[1])
        # Euclidean distance scaled by minimum possible cost
        dist = np.sqrt(di**2 + dj**2)
        return dist * (mdp.m * mdp.g * 0.01)  # Conservative underestimate
    
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start, [start]))
    
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
        
        # Explore neighbors using MDP actions
        for action in mdp.get_valid_actions(current):
            next_state = mdp.transition(current, action)
            if next_state is None:
                continue
            
            # Use expected cost for planning
            move_cost = mdp.expected_cost(current, next_state)
            new_cost = cost_so_far + move_cost
            
            if next_state not in best_cost or new_cost < best_cost[next_state]:
                best_cost[next_state] = new_cost
                priority = new_cost + heuristic(next_state, goal)
                heapq.heappush(open_set, (priority, new_cost, next_state, path + [next_state]))
    
    return None, float('inf'), nodes_explored


# ==============================================================================
# ORIGINAL LAYER 1 & 2 CODE (modified to use MDP for local planning)
# ==============================================================================

def heuristic(current, goal, min_cost_estimate=1.0):
    manhattan_dist = abs(current[0] - goal[0]) + abs(current[1] - goal[1])
    return manhattan_dist * min_cost_estimate


def a_star(X, Y, Z, start, goal, forces_dict):
    """Original A* for coarse grid planning."""
    min_cost_estimate = min(forces_dict.values()) if forces_dict else 1.0
    
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal, min_cost_estimate), 0, start, [start]))
    
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
                if dx == 0 and dy == 0:
                    continue
                    
                ni, nj = i + dx, j + dy
                
                if (i, j, ni, nj) in forces_dict:
                    move_cost = forces_dict[(i, j, ni, nj)]
                else:
                    continue
                
                new_cost = cost_so_far + move_cost
                
                if (ni, nj) not in best_cost or new_cost < best_cost[(ni, nj)]:
                    best_cost[(ni, nj)] = new_cost
                    priority = new_cost + heuristic((ni, nj), goal, min_cost_estimate)
                    heapq.heappush(open_set, (priority, new_cost, (ni, nj), path + [(ni, nj)]))
                    
    return None, float('inf'), nodes_explored


def compute_pairwise_costs(X, Y, Z, forces_dict, nodes):
    """Original pairwise cost computation for coarse grid."""
    print("Layer 1: Computing A* Pairwise Costs (Coarse Grid)")

    cost_matrix = {}
    path_matrix = {}
    
    for i, start_node in enumerate(nodes):
        for j, end_node in enumerate(nodes):
            if i == j:
                cost_matrix[(start_node, end_node)] = 0
                path_matrix[(start_node, end_node)] = [start_node]
                continue
            
            path, cost, nodes_explored = a_star(X, Y, Z, start_node, end_node, forces_dict)
            
            if path is None:
                print(f"No path found from {start_node} to {end_node}")
                cost = float('inf')
                path = []
            
            cost_matrix[(start_node, end_node)] = cost
            path_matrix[(start_node, end_node)] = path
            print(f"  {start_node} -> {end_node}: cost={cost:.4f}, path_length={len(path)}, nodes_explored={nodes_explored}")
    
    return cost_matrix, path_matrix


def tsp_milp_model(nodes, cost_matrix, start_node):
    """Original TSP MILP model."""
    print("Layer 2: MILP using pairwise costs")
    
    model = pyo.ConcreteModel()
    
    node_to_idx = {node: idx for idx, node in enumerate(nodes)}
    start_idx = node_to_idx[start_node]

    model.nodes = pyo.Set(initialize=range(len(nodes)))
    model.x = pyo.Var(model.nodes, model.nodes, within=pyo.Binary)
    model.u = pyo.Var(model.nodes, within=pyo.NonNegativeIntegers, bounds=(0, len(nodes)))

    model.u[start_idx].fix(0)
    
    def obj_rule(model):
        return sum(model.x[i, j] * cost_matrix[(nodes[i], nodes[j])] 
                   for i in model.nodes for j in model.nodes if i != j)
    model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)
    
    def leave_rule(model, i):
        if i == start_idx:
            return sum(model.x[i, j] for j in model.nodes if j != i) == 1
        else:
            return sum(model.x[i, j] for j in model.nodes if j != i) <= 1
    model.leave = pyo.Constraint(model.nodes, rule=leave_rule)
    
    def enter_rule(model, j):
        if j == start_idx:
            return sum(model.x[i, j] for i in model.nodes if i != j) == 0
        else:
            return sum(model.x[i, j] for i in model.nodes if i != j) == 1
    model.enter = pyo.Constraint(model.nodes, rule=enter_rule)
    
    def flow_rule(model, k):
        if k == start_idx:
            return pyo.Constraint.Skip
        return (sum(model.x[k, j] for j in model.nodes if j != k) <= 
                sum(model.x[i, k] for i in model.nodes if i != k))
    model.flow = pyo.Constraint(model.nodes, rule=flow_rule)
    
    def subtour_rule(model, i, j):
        if i == j or i == start_idx:
            return pyo.Constraint.Skip
        M = len(nodes)
        return model.u[j] >= model.u[i] + 1 - M * (1 - model.x[i, j])
    
    model.subtour = pyo.Constraint(model.nodes, model.nodes, rule=subtour_rule)
    
    return model, node_to_idx


def extract_tour_sequence(model, nodes, node_to_idx, start_node):
    """Extract tour sequence from solved MILP model."""
    start_idx = node_to_idx[start_node]
    
    next_node = {}
    for i in model.nodes:
        for j in model.nodes:
            if i != j and pyo.value(model.x[i, j]) >= 0.5:
                next_node[i] = j
                break
    
    tour_indices = [start_idx]
    current = start_idx
    while len(tour_indices) < len(nodes):
        current = next_node[current]
        tour_indices.append(current)
    
    tour_sequence = [nodes[idx] for idx in tour_indices]
    return tour_sequence


# ==============================================================================
# MAIN EXECUTION WITH MDP LOCAL PLANNING (ANALYTICAL STATISTICS)
# ==============================================================================

def coarse_to_fine_path(coarse_path, refinement_factor):

    """
    Convert coarse grid path to fine grid path by filling in all 
    intermediate fine grid points between coarse nodes.
    """
    if len(coarse_path) < 2:
        if len(coarse_path) == 1:
            i, j = coarse_path[0]
            return [(i * refinement_factor, j * refinement_factor)]
        return []
    
    fine_path = []
    
    for idx in range(len(coarse_path) - 1):
        # Start and end coarse nodes
        (i_start, j_start) = coarse_path[idx]
        (i_end, j_end) = coarse_path[idx + 1]
        
        # Convert to fine grid coordinates
        fi_start = i_start * refinement_factor
        fj_start = j_start * refinement_factor
        fi_end = i_end * refinement_factor
        fj_end = j_end * refinement_factor
        
        # Direction of movement
        di = 1 if fi_end > fi_start else (-1 if fi_end < fi_start else 0)
        dj = 1 if fj_end > fj_start else (-1 if fj_end < fj_start else 0)
        
        # Add all intermediate fine grid points
        fi, fj = fi_start, fj_start
        while (fi, fj) != (fi_end, fj_end):
            fine_path.append((fi, fj))
            
            # Move one step on fine grid
            if fi != fi_end:
                fi += di
            if fj != fj_end:
                fj += dj
    
    
    i_final, j_final = coarse_path[-1]
    fine_path.append((i_final * refinement_factor, j_final * refinement_factor))
    
   
    return fine_path

def path_hits_obstacle(path, Z, refinement_factor):
    """Check if path intersects any obstacle in coarse grid."""
    for (i, j) in path:
        ci, cj = i // refinement_factor, j // refinement_factor
        if Z[ci, cj] > 1e5:
            return True
    return False




def run_two_layer_routing_with_mdp():
    """
    Enhanced routing with MDP-based local path planning using stochastic phi.
    
    Flow:
    1. Load coarse grid elevation data
    2. Perform global planning on coarse grid (Layer 1 & 2)
    3. For each segment, perform local planning on adaptive grid using MDP
    4. Compute exact stochastic cost distribution analytically (no Monte Carlo)
    """
    X, Y, forces, Z = elevationCosts("elevationdata.txt")

    
    
    print(f"Grid size: {X} x {Y}")
    print(f"Total grid nodes: {X * Y}")
    
    # Select nodes to visit
    start_node = (0, 0)
    random.seed(41)
    all_nodes = [(i, j) for i in range(X) for j in range(Y)]
    all_nodes.remove(start_node)
    
    num_targets = min(12, len(all_nodes))
    target_sites = random.sample(all_nodes, num_targets)
    nodes_to_visit = [start_node] + target_sites
    
    print(f"\nStart node: {start_node}")
    print(f"Target sites ({len(target_sites)}): {target_sites}")
    print(f"Total nodes to visit: {len(nodes_to_visit)}")
    
    # Layer 1: Compute pairwise costs using A* on coarse grid
    start_time = time.time()
    cost_matrix, path_matrix = compute_pairwise_costs(X, Y, Z, forces, nodes_to_visit)
    layer1_time = time.time() - start_time
    print(f"\nLayer 1 completed in {layer1_time:.2f} seconds")
    
    # Layer 2: Solve TSP using MILP
    start_time = time.time()
    tsp_model, node_to_idx = tsp_milp_model(nodes_to_visit, cost_matrix, start_node)
    

    opt = SolverFactory("gurobi", tee=False)
    results = opt.solve(tsp_model, tee=False)
    


    if results.solver.termination_condition != pyo.TerminationCondition.optimal:
        print("WARNING: TSP solver did not find optimal solution")
        return None
    
        # Total number of variables
    num_vars = pyo.value(tsp_model.nvariables())

    # Total number of constraints
    num_constraints = pyo.value(tsp_model.nconstraints())


    print(f"Variables: {num_vars}")
    print(f"Constraints: {num_constraints}")

    tour_sequence = extract_tour_sequence(tsp_model, nodes_to_visit, node_to_idx, start_node)
    nominal_cost = pyo.value(tsp_model.obj)
    layer2_time = time.time() - start_time
    print(f"Layer 2 completed in {layer2_time:.2f} seconds")
    
    # Print coarse grid tour
    print("\n" + "="*70)
    print("COARSE GRID TOUR (Global Planning)")
    print("="*70)
    for i, node in enumerate(tour_sequence):
        if i < len(tour_sequence) - 1:
            next_node = tour_sequence[i + 1]
            segment_cost = cost_matrix[(node, next_node)]
            print(f"  {i+1}. {node} -> {next_node} (cost: {segment_cost:.4f})")
        else:
            print(f"  {i+1}. {node} (terminal)")
    print(f"\nTotal coarse tour cost: {nominal_cost:.4f}")
    

    print("\n" + "="*70)
    print("ADDING OBSTACLES FOR LAYER 3 LOCAL PLANNING")
    print("="*70)

    # Place obstacles ALONG the A* paths to force MDP to navigate around them
    random.seed(123)  # For reproducibility
    num_obstacles = 20

    # Collect all cells from A* paths
    all_path_cells = set()
    for i in range(len(tour_sequence) - 1):
        start_node = tour_sequence[i]
        end_node = tour_sequence[i + 1]
        
        if (start_node, end_node) in path_matrix:
            astar_path = path_matrix[(start_node, end_node)]
            # Add all cells in this A* path
            for cell in astar_path:
                all_path_cells.add(cell)

    print(f"  A* paths cover {len(all_path_cells)} cells")

    # Place obstacles near (but not exactly on) the A* paths
    Z_with_obstacles = Z.copy()
    obstacle_locations = []
    attempts = 0
    max_attempts = num_obstacles * 100

    while len(obstacle_locations) < num_obstacles and attempts < max_attempts:
        attempts += 1
        
        # Pick a random cell from the A* paths
        if not all_path_cells:
            break
        
        path_cell = random.choice(list(all_path_cells))
        path_i, path_j = path_cell
        
        # Place obstacle NEAR this path cell (offset by 1-3 cells)
        offset_i = random.randint(-3, 3)
        offset_j = random.randint(-3, 3)
        
        # Skip if offset is too small (would be on the path)
        if abs(offset_i) + abs(offset_j) < 1:
            continue
        
        obs_i = path_i + offset_i
        obs_j = path_j + offset_j
        
        # Check bounds
        if not (0 <= obs_i < Z.shape[0] and 0 <= obs_j < Z.shape[1]):
            continue
        
        # Don't place directly on A* path
        if (obs_i, obs_j) in all_path_cells:
            continue
        
        # Don't place duplicates
        if (obs_i, obs_j) in obstacle_locations:
            continue
        
        # Add obstacle
        obstacle_locations.append((obs_i, obs_j))
        Z_with_obstacles[obs_i, obs_j] = 1e6

    print(f"✓ Added {len(obstacle_locations)} obstacles near A* paths")
    print(f"  Obstacle locations: {obstacle_locations}")
    print(f"  These will ONLY affect Layer 3 (MDP local planning)")
    print(f"  Layers 1 & 2 used clean terrain")



    # ==============================================================================
    # Layer 3: MDP-BASED LOCAL PLANNING WITH ANALYTICAL STATISTICS
    # ==============================================================================
    
    print("\n" + "="*70)
    print("LAYER 3: MDP-BASED LOCAL PATH PLANNING (STOCHASTIC PHI)")
    print("="*70)
    
    # Initialize MDP with adaptive grid and stochastic phi
    refinement_factor = 3  # Each coarse cell becomes 3x3 fine cells
    phi_mean = 0.5  # Expected interpolation penalty
    phi_std = 0.2   # Uncertainty in interpolation penalty
    
    mdp = AdaptiveGridMDP(
        Z_coarse=Z_with_obstacles,
        m=1.0,
        g=9.81,
        refinement_factor=refinement_factor,
        phi_mean=phi_mean,
        phi_std=phi_std
    )
    
    mdp.print_grid_info()
    
    # Refine each segment of the coarse tour using MDP
    print("Refining tour segments with adaptive grid MDP (stochastic phi)...\n")
    
    refined_paths = {}
    refined_expected_costs = {}
    refined_distributions = {}
    total_expected_cost = 0.0
    total_variance = 0.0

    total_det_cost = 0.0
    total_det_variance = 0.0

    total_det_obstacle_penalty = 0.0
    total_mdp_obstacle_penalty = 0.0

    local_planning_time = 0.0
    
    for i in range(len(tour_sequence) - 1):
        coarse_start = tour_sequence[i]
        coarse_goal = tour_sequence[i + 1]

        fine_start = (coarse_start[0] * refinement_factor, 
                    coarse_start[1] * refinement_factor)
        fine_goal = (coarse_goal[0] * refinement_factor, 
                    coarse_goal[1] * refinement_factor)

        print(f"Segment {i+1}: {coarse_start} -> {coarse_goal}")
        print(f"  Fine grid: {fine_start} -> {fine_goal}")

        # ============================================================
        # (A) BASELINE: Deterministic path (NO reoptimization)
        # ============================================================
        
        coarse_astar_path = path_matrix[(coarse_start, coarse_goal)]

        # CORRECTED: Properly interpolate to fine grid
        deterministic_fine_path = coarse_to_fine_path(coarse_astar_path, refinement_factor)

        print(f"  Coarse path length: {len(coarse_astar_path)} nodes")
        print(f"  Deterministic fine path length: {len(deterministic_fine_path)} nodes")

        deterministic_dist = mdp.get_path_cost_distribution(deterministic_fine_path)
        hits_obstacle = path_hits_obstacle(deterministic_fine_path, Z_with_obstacles, refinement_factor)




        # ============================================================
        # (B) YOUR METHOD: MDP REFINED PATH
        # ============================================================
        segment_start_time = time.time()

        refined_path, expected_cost, nodes_explored = a_star_mdp(
            mdp, fine_start, fine_goal
        )

        dist_stats = mdp.get_path_cost_distribution(refined_path)

        segment_time = time.time() - segment_start_time
        local_planning_time += segment_time

        # Store totals
        total_expected_cost += dist_stats['mean']
        total_variance += dist_stats['variance']

        total_det_obstacle_penalty += deterministic_dist['obstacle_penalty_cost']
        total_mdp_obstacle_penalty += dist_stats['obstacle_penalty_cost']

        total_det_cost += deterministic_dist['mean']
        total_det_variance += deterministic_dist['variance']

        # ============================================================
        # PRINT COMPARISON (THIS IS YOUR EXPERIMENT)
        # ============================================================

        print("\n  --- BASELINE: Deterministic path (no reoptimization) ---")
        print(f"  Path length: {len(deterministic_fine_path)}")

        if hits_obstacle:
            print("  ⚠️ Hits obstacle → infeasible / unrealistic path")

        print(f"  Expected cost: {deterministic_dist['mean']:.4f}")
        print(f"  Std dev: {deterministic_dist['std']:.4f}")
        print(f"  95% CI: [{deterministic_dist['ci_lower']:.4f}, {deterministic_dist['ci_upper']:.4f}]")
        print(f"  VaR (95%): {deterministic_dist['q95']:.4f}")

        print("\n  --- PROPOSED: MDP refined path ---")
        print(f"  Path length: {len(refined_path)}")
        print(f"  Expected cost: {dist_stats['mean']:.4f}")
        print(f"  Std dev: {dist_stats['std']:.4f}")
        print(f"  95% CI: [{dist_stats['ci_lower']:.4f}, {dist_stats['ci_upper']:.4f}]")
        print(f"  VaR (95%): {dist_stats['q95']:.4f}")

        improvement = deterministic_dist['mean'] - dist_stats['mean']
        print(f"\n  Improvement (baseline - MDP): {improvement:+.4f}")

        print(f"  Nodes explored (MDP): {nodes_explored}")
        print(f"  Time: {segment_time:.3f}s")
        print("-" * 60)
    
    # Aggregate statistics
    total_std = np.sqrt(total_variance)
    
    # Compute aggregate distribution (sum of independent normals)
    if total_std > 0:
        total_dist = stats.norm(loc=total_expected_cost, scale=total_std)
        total_ci_lower = total_dist.ppf(0.025)
        total_ci_upper = total_dist.ppf(0.975)
        total_var_95 = total_dist.ppf(0.95)
    else:
        total_ci_lower = total_ci_upper = total_var_95 = total_expected_cost
    
    print("="*70)
    print("AGGREGATE STOCHASTIC ANALYSIS (ANALYTICAL)")
    print("="*70)
    print(f"Total tour cost distribution:")
    print(f"  Coarse grid cost: {nominal_cost:.4f}")
    print(f"  Expected cost E[c]: {total_expected_cost:.4f}")
    print(f"  Total std dev: {total_std:.4f}")
    print(f"  Total variance: {total_variance:.6f}")
    print(f"  95% confidence interval: [{total_ci_lower:.4f}, {total_ci_upper:.4f}]")
    print(f"  Risk measure VaR(95%): {total_var_95:.4f}")
    print(f"\nInterpretation:")
    print(f"  - Expected cost is {total_expected_cost - nominal_cost:+.4f} "
          f"({100*(total_expected_cost - nominal_cost)/nominal_cost:+.2f}%) "
          f"higher than coarse grid")
    print(f"  - With 95% confidence, actual cost will be in [{total_ci_lower:.4f}, {total_ci_upper:.4f}]")
    print(f"  - There's a 95% chance actual cost ≤ {total_var_95:.4f}")
    
    # Final summary
    print("\n" + "="*70)
    print("FINAL RESULTS")
    print("="*70)
    print(f"Coarse grid tour cost: {nominal_cost:.4f}")
    print(f"Refined expected tour cost: {total_expected_cost:.4f}")
    print(f"Cost difference: {total_expected_cost - nominal_cost:+.4f} " + 
          f"({100*(total_expected_cost - nominal_cost)/nominal_cost:+.2f}%)")
    print(f"Cost uncertainty (std): {total_std:.4f}")
    print(f"Coefficient of variation: {total_std/total_expected_cost:.4f}")
    print(f"\nTiming breakdown:")
    print(f"  Layer 1 (A* pairwise): {layer1_time:.2f}s")
    print(f"  Layer 2 (TSP MILP): {layer2_time:.2f}s")
    print(f"  Layer 3 (MDP local planning): {local_planning_time:.2f}s")
    print(f"  Total: {layer1_time + layer2_time + local_planning_time:.2f}s")
    print("\nNote: All statistics computed analytically (exact closed-form solution)")
    print("="*70)

    print("\n" + "="*70)
    print("SUMMARY (NO REOPT vs MDP)")
    print("="*70)

    

    print(f"Deterministic (no reopt): "
        f"{total_det_cost - total_det_obstacle_penalty:.4f} "
        f"(+ {total_det_obstacle_penalty:.4f} obstacle penalty) "
        f"= {total_det_cost:.4f}")

    print(f"MDP refined: "
        f"{total_expected_cost - total_mdp_obstacle_penalty:.4f} "
        f"(+ {total_mdp_obstacle_penalty:.4f} obstacle penalty) "
        f"= {total_expected_cost:.4f}")

    print(f"\nImprovement (total): {total_det_cost - total_expected_cost:.4f}")
    print(f"Improvement (excluding obstacles): "
        f"{(total_det_cost - total_det_obstacle_penalty) - (total_expected_cost - total_mdp_obstacle_penalty):.4f}")
        
    return tour_sequence, refined_paths, refined_expected_costs, refined_distributions, mdp, Z, Z_with_obstacles, path_matrix


if __name__ == "__main__":

    run_two_layer_routing_with_mdp()