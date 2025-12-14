import ex1_check
import search
import utils
# I've solved the assignment by myself, and used gemini during the process.

# Student ID
id = ["216764803"]

# Directions mapping
_MOVES = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

class WateringProblem(search.Problem):
    """
    This class implements the Watering Problem for AI Search.
    
    It solves the problem of moving robots to taps to fill up and to plants to pour water,
    navigating around walls and other robots.
    """

    def __init__(self, initial):
        """
        Constructor. Parses the input and performs extensive pre-computation 
        to speed up the search phase.
        """
        # --- 1. Parse Initial State ---
        self.size = initial["Size"]
        self.walls = frozenset(initial.get("Walls", set()))
        self.plants_targets = dict(initial["Plants"])
        self.robots_capacities = self._extract_robot_capacities(initial["Robots"])
        
        # Sort positions for deterministic behavior
        self.plant_positions = tuple(sorted(frozenset(initial["Plants"].keys())))
        self.tap_positions = tuple(sorted(frozenset(initial["Taps"].keys())))

        # --- 2. Pre-Calculate BFS Maps (All-Pairs Shortest Paths from POIs) ---
        # Instead of calculating distances on the fly (slow), we calculate the 
        # real grid distance (accounting for walls) from every Plant and Tap 
        # to every other cell in the grid.
        self.plant_bfs_maps = {}
        for p_pos in self.plant_positions:
            self.plant_bfs_maps[p_pos] = self._bfs_map(p_pos)
        
        self.tap_bfs_maps = {}
        for t_pos in self.tap_positions:
            self.tap_bfs_maps[t_pos] = self._bfs_map(t_pos)

        # --- 3. OPTIMIZATION: Highway Pruning (Single Robot Strategy) ---
        # If there is only 1 robot, we can aggressively prune the search space.
        # We identify the "Highways" (shortest paths) connecting all Points of Interest (POIs).
        # Any cell NOT on a highway is treated as a wall, drastically reducing branching.
        robot_starts = set((r, c) for (r, c, _, _) in initial["Robots"].values())
        num_robots = len(initial["Robots"])

        if num_robots == 1:
            start_pos = list(robot_starts)[0]
            start_bfs_map = self._bfs_map(start_pos)
            
            # Points of Interest: Plants, Taps, and the Robot's Start
            pois = set(self.plant_positions) | set(self.tap_positions) | {start_pos}
            
            # Helper to fetch pre-calc maps
            def get_dist_map(p):
                if p == start_pos: return start_bfs_map
                if p in self.plant_bfs_maps: return self.plant_bfs_maps[p]
                if p in self.tap_bfs_maps: return self.tap_bfs_maps[p]
                return None

            valid_highway_cells = set()

            # Generate all logical connections (Start->Targets, Taps<->Plants, Plants<->Plants)
            pairs = []
            for target in pois:
                if start_pos != target: pairs.append((start_pos, target))
            
            taps = list(self.tap_positions)
            plants = list(self.plant_positions)
            
            for t in taps:
                for p in plants:
                    pairs.append((t, p)); pairs.append((p, t))
            for p1 in plants:
                for p2 in plants:
                    if p1 != p2: pairs.append((p1, p2))

            # "Gradient Walk" to find cells strictly on the shortest path
            for src, dst in pairs:
                d_map_src = get_dist_map(src)
                d_map_dst = get_dist_map(dst)
                
                # Check connectivity
                if not d_map_src or dst not in d_map_src: continue
                
                shortest_len = d_map_src[dst]
                
                # Local BFS to trace the path
                queue = [src]
                visited_local = {src}
                valid_highway_cells.add(src)
                
                idx = 0
                while idx < len(queue):
                    curr = queue[idx]
                    idx += 1
                    if curr == dst: continue 
                    
                    for dr, dc in _MOVES.values():
                        nr, nc = curr[0] + dr, curr[1] + dc
                        
                        # Bounds and duplicates check
                        if (nr, nc) in visited_local: continue
                        if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
                        
                        d_n_src = d_map_src.get((nr, nc))
                        d_n_dst = d_map_dst.get((nr, nc))
                        
                        # Strict Shortest Path Condition: Dist(Start->Node) + Dist(Node->End) == Optimal
                        if d_n_src is not None and d_n_dst is not None:
                            if d_n_src + d_n_dst == shortest_len:
                                valid_highway_cells.add((nr, nc))
                                visited_local.add((nr, nc))
                                queue.append((nr, nc))

            # Apply Pruning: Treat non-highway cells as new walls
            if valid_highway_cells: 
                new_walls = set(self.walls)
                for r in range(self.size[0]):
                    for c in range(self.size[1]):
                        if (r, c) not in self.walls and (r, c) not in valid_highway_cells:
                            new_walls.add((r, c))
                self.walls = frozenset(new_walls)

            # Re-calculate maps with new constraints
            self.plant_bfs_maps = {}
            for p_pos in self.plant_positions:
                self.plant_bfs_maps[p_pos] = self._bfs_map(p_pos)
            self.tap_bfs_maps = {}
            for t_pos in self.tap_positions:
                self.tap_bfs_maps[t_pos] = self._bfs_map(t_pos)

        # --- 4. Initialize Dynamic State ---
        robot_states = self._build_robot_states(initial["Robots"])
        plant_states = self._build_plant_states(initial["Plants"])
        tap_states = self._build_tap_states(initial["Taps"])
        total_remaining = sum(initial["Plants"].values())

        initial_state = (robot_states, plant_states, tap_states, total_remaining)
        search.Problem.__init__(self, initial_state)

    def _bfs_map(self, start_pos):
        """
        Performs a Breadth-First Search from start_pos to every reachable cell on the grid.
        Returns: Dict {(r,c): distance}
        """
        queue = [(start_pos, 0)]
        distances = {start_pos: 0}
        visited = {start_pos}
        
        idx = 0
        while idx < len(queue):
            (r, c), dist = queue[idx]
            idx += 1
            
            for dr, dc in _MOVES.values():
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.size[0] and 0 <= nc < self.size[1]:
                    # Process only if not a wall and not visited
                    if (nr, nc) not in self.walls and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        new_dist = dist + 1
                        distances[(nr, nc)] = new_dist
                        queue.append(((nr, nc), new_dist))
        return distances

    # --------------------------------------------------------------------------
    # Helper Methods for State Construction
    # --------------------------------------------------------------------------

    @staticmethod
    def _extract_robot_capacities(robots_data):
        """Extracts static capacity data for robots."""
        return {
            robot_id: capacity
            for robot_id, (_, _, _, capacity) in robots_data.items()
        }

    def _build_robot_states(self, robots_data):
        """Creates a frozenset of robot dynamic states: (id, row, col, load)."""
        return frozenset(
            (robot_id, r, c, load)
            for robot_id, (r, c, load, _) in sorted(robots_data.items())
        )

    def _build_plant_states(self, plants_data):
        """Creates a tuple of currently poured amounts for plants."""
        return tuple(0 for _ in self.plant_positions)

    def _build_tap_states(self, taps_data):
        """Creates a tuple of remaining water in taps."""
        return tuple(taps_data.get(pos, 0) for pos in self.tap_positions)

    # --------------------------------------------------------------------------
    # Successor Generation
    # --------------------------------------------------------------------------

    def successor(self, state):
        """
        Generates valid successor states.
        Logic:
        1. Identify occupied cells to prevent collisions.
        2. Prioritize logical actions (POUR / LOAD) over movement.
        3. If moving, prioritize "Improving Moves" (moving closer to target).
        """
        robot_states, plant_states, tap_states, total_remaining = state
        successors = []
        current_total_load = sum(l for (_, _, _, l) in robot_states)

        for robot_entry in sorted(robot_states):
            robot_id, r, c, load = robot_entry

            # --- 1. Collision Check ---
            occupied = {
                (or_r, or_c)
                for (oid, or_r, or_c, _) in robot_states
                if oid != robot_id
            }
            
            is_blocking = False
            for dr, dc in _MOVES.values():
                if (r + dr, c + dc) in occupied:
                    is_blocking = True; break

            possible_actions = []

            # --- 2. Action Generation (POUR / LOAD) ---
            # Try POUR
            if load > 0 and (r, c) in self.plant_positions:
                 possible_actions.extend(
                    self._get_pour_successor(robot_states, plant_states, tap_states, total_remaining,
                                             robot_entry, robot_id, r, c, load)
                 )

            # Try LOAD
            if load < self.robots_capacities[robot_id] and (r, c) in self.tap_positions:
                if total_remaining > current_total_load:
                    possible_actions.extend(
                        self._get_load_successor(robot_states, plant_states, tap_states, total_remaining,
                                                 robot_entry, robot_id, r, c, load)
                    )

            # --- 3. Optimization: Prioritize Actions ---
            # If we can act (Load/Pour), do it and skip movement unless we are blocking someone.
            if possible_actions:
                successors.extend(possible_actions)
                if not is_blocking:
                    continue 

            # --- 4. Move Generation ---
            successors.extend(
                self._get_movement_successors(robot_states, plant_states, tap_states, total_remaining,
                                              robot_entry, robot_id, r, c, load, occupied)
            )

        return successors

    def _get_movement_successors(self, robot_states, plant_states, tap_states, total_remaining,
                                 robot_entry, robot_id, r, c, load, occupied):
        """
        Generates movement successors with 'Improving Moves' pruning.
        """
        successors = []
        capacity = self.robots_capacities[robot_id]
        
        # A. Determine which distance maps are relevant based on robot state
        if load == 0:
            target_maps = list(self.tap_bfs_maps.values())
        elif load == capacity:
            target_maps = list(self.plant_bfs_maps.values())
        else:
            target_maps = list(self.tap_bfs_maps.values()) + list(self.plant_bfs_maps.values())

        # B. Get all physically valid moves
        valid_candidates = []
        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc
            if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
            if (nr, nc) in self.walls: continue
            if (nr, nc) in occupied: continue
            valid_candidates.append((action_name, nr, nc))

        # C. Filter for "Improving" moves (moves that strictly reduce distance to a target)
        improving_candidates = []
        for action_name, nr, nc in valid_candidates:
            is_improving = False
            for bfs_map in target_maps:
                curr_dist = bfs_map.get((r, c))
                next_dist = bfs_map.get((nr, nc))
                
                if curr_dist is not None and next_dist is not None:
                    if next_dist == curr_dist - 1:
                        is_improving = True; break
            
            if is_improving:
                improving_candidates.append((action_name, nr, nc))

        # D. Decision: Use improving moves if available, otherwise fallback (e.g., getting out of way)
        final_candidates = improving_candidates if improving_candidates else valid_candidates

        # E. Create States
        for action_name, nr, nc in final_candidates:
            old_robot_entry = robot_entry
            new_robot_entry = (robot_id, nr, nc, load)
            new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}
            
            new_state = (new_robot_states, plant_states, tap_states, total_remaining)
            successors.append(((robot_id, action_name), new_state))

        return successors

    def _get_load_successor(self, robot_states, plant_states, tap_states, total_remaining,
                             robot_entry, robot_id, r, c, load):
        """Generates the state after a LOAD action."""
        if (r, c) not in self.tap_positions: return []
        try:
            tidx = self.tap_positions.index((r, c))
        except ValueError: return []

        remaining_tap = tap_states[tidx]
        if remaining_tap <= 0: return []

        capacity = self.robots_capacities.get(robot_id, 0)
        if load >= capacity: return []

        old_robot_entry = robot_entry
        new_robot_entry = (robot_id, r, c, load + 1)
        new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}
        
        # Decrement tap
        new_tap_states = tuple(
            (remaining_tap - 1) if i == tidx else amt
            for i, amt in enumerate(tap_states)
        )

        new_state = (new_robot_states, plant_states, new_tap_states, total_remaining)
        return [((robot_id, "LOAD"), new_state)]

    def _get_pour_successor(self, robot_states, plant_states, tap_states, total_remaining,
                             robot_entry, robot_id, r, c, load):
        """Generates the state after a POUR action."""
        if load <= 0: return []
        if (r, c) not in self.plant_positions: return []
        try:
            pidx = self.plant_positions.index((r, c))
        except ValueError: return []
        
        poured = plant_states[pidx]
        target = self.plants_targets.get((r, c), 0)
        
        if poured >= target: return []

        old_robot_entry = robot_entry
        new_robot_entry = (robot_id, r, c, load - 1)
        new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}

        # Increment plant, decrement total remaining
        new_plant_states = tuple(
            (poured + 1) if i == pidx else amt
            for i, amt in enumerate(plant_states)
        )
        new_total_remaining = total_remaining - 1
        new_state = (new_robot_states, new_plant_states, tap_states, new_total_remaining)
        return [((robot_id, "POUR"), new_state)]

    def goal_test(self, state):
        return state[3] == 0

    # --------------------------------------------------------------------------
    # Heuristics (A* and GBFS)
    # --------------------------------------------------------------------------

    def h_astar(self, node):
        """
        Admissible Heuristic for A*.
        Combines:
        1. Minimum Spanning Forest (MSF) estimate of travel distance.
        2. Interaction Cost (minimum actions to load/pour).
        """
        state = node.state
        robot_states, plant_states, _, total_remaining = state
        
        robot_positions = [(r, c) for (_, r, c, _) in sorted(robot_states)]
        unsatisfied_plants = []
        for i, pos in enumerate(self.plant_positions):
            if plant_states[i] < self.plants_targets.get(pos, 0):
                unsatisfied_plants.append(pos)
        
        # Base Cost: Every unit of water needs at least 1 Load + 1 Pour interaction
        carried = sum(load for (_, _, _, load) in robot_states)
        interaction_cost = (2 * total_remaining) - carried
        
        if not unsatisfied_plants:
            return max(0, interaction_cost)
            
        # Prim's Algorithm for Minimum Spanning Forest
        # Connects Robots and Plants into a minimal graph
        num_robots = len(robot_positions)
        total_vertices = num_robots + len(unsatisfied_plants)
        
        visited = [False] * total_vertices
        min_dists = [float('inf')] * total_vertices
        min_dists[0] = 0
        mst_weight = 0
        
        for _ in range(total_vertices):
            # Find closest unvisited node
            u = -1
            min_val = float('inf')
            for i in range(total_vertices):
                if not visited[i] and min_dists[i] < min_val:
                    min_val = min_dists[i]
                    u = i
            
            if u == -1 or min_val == float('inf'): break
                
            visited[u] = True
            mst_weight += min_val
            
            pos_u = None
            is_robot_u = (u < num_robots)
            if is_robot_u:
                pos_u = robot_positions[u]
            else:
                pos_u = unsatisfied_plants[u - num_robots]

            # Update neighbors
            for v in range(total_vertices):
                if not visited[v]:
                    weight = float('inf')
                    is_robot_v = (v < num_robots)
                    
                    if is_robot_u and is_robot_v:
                        weight = 0 # No cost between robots (conceptual)
                    else:
                        pos_v = robot_positions[v] if is_robot_v else unsatisfied_plants[v - num_robots]
                        raw_dist = float('inf')
                        
                        # Use Pre-Calculated BFS Maps for O(1) distance lookup
                        if not is_robot_u:
                            raw_dist = self.plant_bfs_maps[pos_u].get(pos_v, float('inf'))
                        elif not is_robot_v:
                            raw_dist = self.plant_bfs_maps[pos_v].get(pos_u, float('inf'))
                            
                        if raw_dist != float('inf'):
                            # Divide by 1.5 to maintain admissibility in multi-robot contexts
                            # where agents work in parallel.
                            weight = raw_dist / 1.5
                            
                    if weight < min_dists[v]:
                        min_dists[v] = weight
                        
        return mst_weight + max(0, interaction_cost)

    def h_gbfs(self, node):
        """
        Greedy Best-First Search Heuristic.
        Prioritizes:
        1. Finishing the game (Remaining == 0).
        2. Delivering water (Reduced Remaining).
        3. Minimizing distance to the NEXT immediate task (Tap or Plant).
        """
        state = node.state
        robot_states, plant_states, _, total_remaining = state
        
        # Priority 1: Done
        if total_remaining == 0: return 0
        
        # Priority 2: Use "Remaining" as primary tier (Weighted heavily)
        base_score = total_remaining * 100
        
        unsatisfied = [
            pos for i, pos in enumerate(self.plant_positions)
            if plant_states[i] < self.plants_targets.get(pos, 0)
        ]
        
        if not unsatisfied: return base_score

        # Priority 3: Minimum Effort for next step
        min_effort = float('inf')
        
        for (_, r, c, load) in robot_states:
            r_pos = (r, c)
            
            # If loaded -> Check dist to Plant
            if load > 0:
                for p_pos in unsatisfied:
                    dist = self.plant_bfs_maps[p_pos].get(r_pos, float('inf'))
                    if dist < min_effort: min_effort = dist
            
            # If empty -> Check dist to Tap -> Plant
            else:
                dist_to_tap = float('inf')
                best_t_pos = None
                
                # Find closest tap
                for t_pos in self.tap_positions:
                    d = self.tap_bfs_maps[t_pos].get(r_pos, float('inf'))
                    if d < dist_to_tap:
                        dist_to_tap = d
                        best_t_pos = t_pos
                
                # Add distance from that tap to closest plant
                if dist_to_tap != float('inf'):
                    dist_tap_to_plant = float('inf')
                    for p_pos in unsatisfied:
                        d = self.plant_bfs_maps[p_pos].get(best_t_pos, float('inf'))
                        if d < dist_tap_to_plant: dist_tap_to_plant = d
                    
                    total = dist_to_tap + dist_tap_to_plant
                    if total < min_effort: min_effort = total

        if min_effort == float('inf'): return float('inf')
        return base_score + min_effort

def create_watering_problem(game):
    return WateringProblem(game)

if __name__ == '__main__':
    ex1_check.main()