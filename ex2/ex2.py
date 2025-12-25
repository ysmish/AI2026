import ext_plant
from collections import deque

# Update with your ID
id = ["216764803"]

# Directions mapping for BFS
_MOVES = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

class Controller:
    """Controller for the stochastic Plant Watering problem using BFS Gradient Maps."""

    def __init__(self, game: ext_plant.Game):
        """
        Constructor. Parses input and pre-calculates BFS distance maps
        to create a 'gradient' that guides robots from ANY cell.
        """
        self.original_game = game
        initial = game.get_problem()
        self.problem = initial

        # --- 2. Parse Static Physics Data ---
        self.size = initial["Size"]
        self.rows, self.cols = self.size
        self.walls = frozenset(initial.get("Walls", set()))
        self.probs = initial.get("robot_chosen_action_prob", {})
        self.goal_reward = initial.get("goal_reward", 0)
        
        # Sort positions for deterministic behavior during runs
        self.plant_positions = tuple(sorted(frozenset(initial["Plants"].keys())))
        self.tap_positions = tuple(sorted(frozenset(initial["Taps"].keys())))
        self.capacities = game.get_capacities()

        # Pre-calculate Average Reward for planning [cite: 43]
        # (Since actual rewards are random, we plan using the expected value)
        self.plant_avg_rewards = {}
        if "plants_reward" in self.problem:
            for pos, rewards in self.problem["plants_reward"].items():
                self.plant_avg_rewards[pos] = sum(rewards) / len(rewards)

        # --- 3. Pre-Calculate BFS Gradient Maps ---
        # We calculate the real grid distance from every POI (Plant/Tap) to ALL cells.
        # This creates a "vector field": if a robot slips to a random cell, 
        # the map immediately tells it the distance from that NEW location 
        # to the target, allowing it to recover optimally.
        
        self.plant_bfs_maps = {}
        for p_pos in self.plant_positions:
            self.plant_bfs_maps[p_pos] = self._bfs_map(p_pos)
        
        self.tap_bfs_maps = {}
        for t_pos in self.tap_positions:
            self.tap_bfs_maps[t_pos] = self._bfs_map(t_pos)

        # Search Depth Configuration (Adjustable)
        self.search_depth = 4

    # --------------------------------------------------------------------------
    # Helper Methods (Internal)
    # --------------------------------------------------------------------------

    def _bfs_map(self, start_pos):
        """
        Performs a Breadth-First Search from start_pos to every reachable cell on the grid.
        Returns: Dict {(r,c): distance}
        
        This map accounts for walls[cite: 8], ensuring distances are physically valid.
        """
        queue = deque([(start_pos, 0)])
        distances = {start_pos: 0}
        visited = {start_pos}
        
        while queue:
            (r, c), dist = queue.popleft()
            
            for dr, dc in _MOVES.values():
                nr, nc = r + dr, c + dc
                
                # Check Bounds [cite: 26]
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    # Check Walls [cite: 8]
                    if (nr, nc) not in self.walls and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        distances[(nr, nc)] = dist + 1
                        queue.append(((nr, nc), dist + 1))
        return distances

    def choose_next_action(self, state):
        """ Choose the next action given a state."""
        raise ValueError("Fill the function")