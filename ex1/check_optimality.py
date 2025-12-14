import time
import random
import heapq
import sys

# --- IMPORT YOUR SOLUTION ---
try:
    import ex1
except ImportError:
    print("Error: Could not import 'ex1.py'. Make sure it is in the same folder.")
    sys.exit(1)

# ==========================================
# 🛠️ USER CONFIGURATION (EDIT THIS)
# ==========================================

# How many random tests to run?
TEST_COUNT = 1000

# Map Dimensions (Rows, Cols)
MAP_SIZE = (20, 20) 

# Number of Robots (Randomly chosen between Min and Max)
ROBOTS_RANGE = (1, 1) 

# Number of Plants and Taps (Randomly chosen between Min and Max)
PLANTS_RANGE = (1, 3)
TAPS_RANGE   = (1, 2)

# Wall Density (0.0 to 1.0)
WALL_PROB = 0.2

# Node limit for UCS (Increase if you get too many TIMEOUTS)
UCS_NODE_LIMIT = 500000

# ==========================================
# END CONFIGURATION
# ==========================================

# --- COLORS ---
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"

class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
    def __lt__(self, other):
        return self.path_cost < other.path_cost

def run_search(problem, heuristic_func, limit):
    """
    Returns: (Node, status_string)
    Status: "SOLVED", "UNSOLVABLE", "TIMEOUT"
    """
    node = Node(problem.initial)
    frontier = []
    # Push tuple (f_score, node)
    heapq.heappush(frontier, (node.path_cost + heuristic_func(node), node))
    explored = set()
    expanded = 0

    while frontier:
        _, node = heapq.heappop(frontier)
        
        if problem.goal_test(node.state):
            return node, "SOLVED"
        
        state_key = node.state
        if state_key in explored: continue
        explored.add(state_key)
        
        expanded += 1
        if expanded > limit:
            return None, "TIMEOUT"

        for action, next_state in problem.successor(node.state):
            child_node = Node(next_state, node, action, node.path_cost + 1)
            if child_node.state not in explored:
                f_score = child_node.path_cost + heuristic_func(child_node)
                heapq.heappush(frontier, (f_score, child_node))
                
    return None, "UNSOLVABLE"

class RandomMapGenerator:
    def generate(self):
        rows, cols = MAP_SIZE
        walls = set()
        
        # 1. Walls
        for r in range(rows):
            for c in range(cols):
                if random.random() < WALL_PROB:
                    walls.add((r, c))

        def get_empty_spot():
            # Try 100 times to find empty spot
            for _ in range(100):
                r, c = random.randint(0, rows-1), random.randint(0, cols-1)
                if (r, c) not in walls: return (r, c)
            return (0, 0) # Fallback

        # 2. Determine Counts
        num_robots = random.randint(ROBOTS_RANGE[0], ROBOTS_RANGE[1])
        num_plants = random.randint(PLANTS_RANGE[0], PLANTS_RANGE[1])
        num_taps   = random.randint(TAPS_RANGE[0], TAPS_RANGE[1])

        # 3. Place Entities (avoid overlaps for clarity)
        occupied = set()
        
        def get_unique_spot():
            loc = get_empty_spot()
            attempts = 0
            while loc in occupied and attempts < 100:
                loc = get_empty_spot()
                attempts += 1
            occupied.add(loc)
            return loc

        robots = {}
        for i in range(num_robots):
            loc = get_unique_spot()
            # ID starts at 10. Capacity random 1-3.
            robots[10+i] = (loc[0], loc[1], 0, random.randint(1, 3))

        plants = {}
        for _ in range(num_plants):
            loc = get_unique_spot()
            plants[loc] = random.randint(1, 3) # Needs 1-3 water

        taps = {}
        for _ in range(num_taps):
            loc = get_unique_spot()
            taps[loc] = random.randint(10, 20) # Plenty of water

        return {
            "Size": MAP_SIZE,
            "Walls": walls,
            "Taps": taps,
            "Plants": plants,
            "Robots": robots, 
        }

def main():
    generator = RandomMapGenerator()
    
    print(f"\n{CYAN}{'='*75}")
    print(f" OPTIMALITY CHECKER")
    print(f" Config: Size={MAP_SIZE} | Robots={ROBOTS_RANGE} | Plants={PLANTS_RANGE}")
    print(f" UCS Limit: {UCS_NODE_LIMIT} nodes")
    print(f"{'='*75}{RESET}")
    print(f"{'ID':<3} | {'STATUS':<12} | {'A*':<6} | {'UCS':<6} | {'TIME'}")
    print("-" * 75)

    stats = {"PASS": 0, "FAIL": 0, "TIMEOUT": 0, "UNSOLVABLE": 0}

    for i in range(1, TEST_COUNT + 1):
        
        map_data = generator.generate()
        
        try:
            problem = ex1.create_watering_problem(map_data)
            
            # 1. Run User A*
            t0 = time.time()
            node_astar, status_astar = run_search(problem, problem.h_astar, limit=UCS_NODE_LIMIT)
            t_astar = time.time() - t0
            
            cost_astar = node_astar.path_cost if node_astar else float('inf')

            # 2. Run UCS (Baseline) - Heuristic is lambda n: 0
            node_ucs, status_ucs = run_search(problem, lambda n: 0, limit=UCS_NODE_LIMIT)
            cost_ucs = node_ucs.path_cost if node_ucs else float('inf')

            # 3. Compare Results
            if status_ucs == "TIMEOUT":
                print(f"{i:<3} | {YELLOW}UCS TIMEOUT {RESET} | {cost_astar:<6} | {'inf':<6} | {t_astar:.2f}s")
                stats["TIMEOUT"] += 1
            
            elif status_astar == "TIMEOUT":
                 print(f"{i:<3} | {RED}A* TIMEOUT  {RESET} | {'inf':<6} | {cost_ucs:<6} | -")
                 stats["TIMEOUT"] += 1

            elif status_astar == "UNSOLVABLE" and status_ucs == "UNSOLVABLE":
                print(f"{i:<3} | {CYAN}UNSOLVABLE  {RESET} | -      | -      | -")
                stats["UNSOLVABLE"] += 1

            elif cost_astar == cost_ucs:
                print(f"{i:<3} | {GREEN}PASS        {RESET} | {cost_astar:<6} | {cost_ucs:<6} | {t_astar:.4f}s")
                stats["PASS"] += 1

            elif cost_astar > cost_ucs:
                print(f"{i:<3} | {RED}FAIL        {RESET} | {cost_astar:<6} | {cost_ucs:<6} | {t_astar:.4f}s")
                print(f"      {RED}>>> Suboptimal! A*:{cost_astar} > UCS:{cost_ucs}{RESET}")
                
                # --- ADD THIS BLOCK ---
                print(f"\n{YELLOW}=== FAILED MAP DATA (Copy this) ==={RESET}")
                print(map_data)
                print(f"{YELLOW}==================================={RESET}\n")
                # ----------------------
                
                stats["FAIL"] += 1
                
            elif cost_astar < cost_ucs:
                print(f"{i:<3} | {RED}LOGIC ERR   {RESET} | {cost_astar:<6} | {cost_ucs:<6} | -")
                print(f"      {RED}>>> A* beat UCS? Impossible.{RESET}")
                stats["FAIL"] += 1

        except Exception as e:
            print(f"{i:<3} | {RED}CRASH       {RESET} | -      | -      | {e}")

    print("-" * 75)
    print(f"Passed: {GREEN}{stats['PASS']}{RESET}")
    print(f"Failed: {RED}{stats['FAIL']}{RESET}")
    print(f"Timeouts: {YELLOW}{stats['TIMEOUT']}{RESET}")
    print(f"Unsolvable: {CYAN}{stats['UNSOLVABLE']}{RESET}")

if __name__ == "__main__":
    main()