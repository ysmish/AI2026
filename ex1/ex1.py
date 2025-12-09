import ex1_check
import search
import utils

id = ["No numbers - I'm special!"]

_MOVES = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

class WateringProblem(search.Problem):
    """This class implements a plant watering problem"""

    def __init__(self, initial):
        """ Constructor only needs the initial state. """
        self.size = initial["Size"]
        self.walls = frozenset(initial.get("Walls", set()))
        self.plants_targets = dict(initial["Plants"])
        self.robots_capacities = self._extract_robot_capacities(initial["Robots"])
        
        self.plant_positions = tuple(sorted(frozenset(initial["Plants"].keys())))
        self.tap_positions = tuple(sorted(frozenset(initial["Taps"].keys())))

        # --- PRE-CALCULATION FOR HEURISTIC & PRUNING ---
        self.plant_bfs_maps = {}
        for p_pos in self.plant_positions:
            self.plant_bfs_maps[p_pos] = self._bfs_map(p_pos)
        
        self.tap_bfs_maps = {}
        for t_pos in self.tap_positions:
            self.tap_bfs_maps[t_pos] = self._bfs_map(t_pos)

        # --- OPTIMIZATION: RELAXED STATIC MAP PRUNING ---
        robot_starts = set((r, c) for (r, c, _, _) in initial["Robots"].values())
        
        # Calculate BFS from robot start positions to check initial connectivity
        self.start_bfs_maps = {}
        for pos in robot_starts:
            # Avoid re-calculating if start is on a plant/tap
            if pos not in self.plant_bfs_maps and pos not in self.tap_bfs_maps:
                self.start_bfs_maps[pos] = self._bfs_map(pos)

        valid_cells = set()
        
        def get_dist_map(p):
            if p in self.plant_bfs_maps: return self.plant_bfs_maps[p]
            if p in self.tap_bfs_maps: return self.tap_bfs_maps[p]
            return self.start_bfs_maps.get(p)

        sources = robot_starts | set(self.plant_positions) | set(self.tap_positions)
        destinations = set(self.plant_positions) | set(self.tap_positions)

        # TOLERANCE: Allow paths slightly longer than optimal to let robots step aside.
        # K=0 is strict shortest path. K=6 allows a 3-step detour (e.g. step out, wait, step back).
        TOLERANCE = 6 

        for src in sources:
            d_map_src = get_dist_map(src)
            if not d_map_src: continue 
            
            for dst in destinations:
                if src == dst: continue
                if dst not in d_map_src: continue
                
                shortest_dist = d_map_src[dst]
                d_map_dst = get_dist_map(dst)

                # Check all reachable cells
                for cell, d_from_src in d_map_src.items():
                    d_from_dst = d_map_dst.get(cell)
                    
                    if d_from_dst is not None:
                        # If this cell is part of a path <= shortest + TOLERANCE, keep it.
                        if d_from_src + d_from_dst <= shortest_dist + TOLERANCE:
                            valid_cells.add(cell)

        # Block cells NOT in valid_cells
        if valid_cells: 
            new_walls = set(self.walls)
            for r in range(self.size[0]):
                for c in range(self.size[1]):
                    if (r, c) not in self.walls and (r, c) not in valid_cells:
                        new_walls.add((r, c))
            self.walls = frozenset(new_walls)
        # --- END OPTIMIZATION ---

        # 2. Initialize dynamic values.
        robot_states = self._build_robot_states(initial["Robots"])
        plant_states = self._build_plant_states(initial["Plants"])
        tap_states = self._build_tap_states(initial["Taps"])
        
        total_remaining = sum(initial["Plants"].values())

        # State is: (robot_states, plant_states, tap_states, total_remaining)
        initial_state = (robot_states, plant_states, tap_states, total_remaining)
        search.Problem.__init__(self, initial_state)
    def _bfs_map(self, start_pos):
        """Runs BFS to find shortest path from start_pos to all NON-WALL cells."""
        queue = [(start_pos, 0)]
        distances = {start_pos: 0}
        visited = {start_pos}
        
        idx = 0
        while idx < len(queue):
            (r, c), dist = queue[idx]
            idx += 1
            
            for dr, dc in _MOVES.values():
                nr, nc = r + dr, c + dc
                
                # Bounds check
                if 0 <= nr < self.size[0] and 0 <= nc < self.size[1]:
                    # STRICT CHECK: Only process if NOT a wall
                    if (nr, nc) not in self.walls and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        new_dist = dist + 1
                        distances[(nr, nc)] = new_dist
                        queue.append(((nr, nc), new_dist))
        return distances

    @staticmethod
    def _extract_robot_capacities(robots_data):
        return {
            robot_id: capacity
            for robot_id, (_, _, _, capacity) in robots_data.items()
        }

    def _build_robot_states(self, robots_data):
        return frozenset(
            (robot_id, r, c, load)
            for robot_id, (r, c, load, _) in sorted(robots_data.items())
        )

    def _build_plant_states(self, plants_data):
        return tuple(0 for _ in self.plant_positions)

    def _build_tap_states(self, taps_data):
        return tuple(taps_data.get(pos, 0) for pos in self.tap_positions)

    def successor(self, state):
        robot_states, plant_states, tap_states, total_remaining = state
        successors = []

        # Calculate current total load (for Mandatory Load check)
        current_total_load = sum(l for (_, _, _, l) in robot_states)

        # Sort for deterministic expansion
        for robot_entry in sorted(robot_states):
            robot_id, r, c, load = robot_entry

            # --- 1. BLOCKING CHECK ---
            # Check if this robot is adjacent to any other robot
            occupied = {
                (or_r, or_c)
                for (oid, or_r, or_c, _) in robot_states
                if oid != robot_id
            }
            
            is_blocking = False
            for dr, dc in _MOVES.values():
                if (r + dr, c + dc) in occupied:
                    is_blocking = True
                    break

            # --- 2. IDENTIFY POTENTIAL ACTIONS (LOAD / POUR) ---
            possible_actions = []

            # Check POUR
            if load > 0 and (r, c) in self.plant_positions:
                 possible_actions.extend(
                    self._get_pour_successor(robot_states, plant_states, tap_states, total_remaining,
                                             robot_entry, robot_id, r, c, load)
                 )

            # Check LOAD
            # We only load if we have space AND the global goal requires more water
            if load < self.robots_capacities[robot_id] and (r, c) in self.tap_positions:
                if total_remaining > current_total_load:
                    possible_actions.extend(
                        self._get_load_successor(robot_states, plant_states, tap_states, total_remaining,
                                                 robot_entry, robot_id, r, c, load)
                    )

            # --- 3. APPLY OPTIMIZATION: ACTION PRIORITY ---
            # "If action performed, skip move unless robot is blocking another robot"
            if possible_actions:
                successors.extend(possible_actions)
                
                if not is_blocking:
                    # If we can act and we aren't in anyone's way, DO NOT generate moves.
                    # This prunes the search tree significantly.
                    continue 
                
                # If we ARE blocking, we added the actions above, 
                # but we ALSO fall through to generate moves below 
                # (so the robot can step aside if necessary).

            # --- 4. MOVES (Standard Expansion) ---
            successors.extend(
                self._get_movement_successors(robot_states, plant_states, tap_states, total_remaining,
                                              robot_entry, robot_id, r, c, load, occupied)
            )

        return successors

    def _get_movement_successors(self, robot_states, plant_states, tap_states, total_remaining,
                                 robot_entry, robot_id, r, c, load, occupied):
        successors = []
        
        # --- KEY FIX: INTELLIGENT TARGET SELECTION ---
        capacity = self.robots_capacities[robot_id]
        
        # 1. Determine Target Maps based on Load vs Capacity
        if load == 0:
            # Empty -> Must go to Tap
            target_maps = list(self.tap_bfs_maps.values())
        elif load == capacity:
            # Full -> Must go to Plant
            target_maps = list(self.plant_bfs_maps.values())
        else:
            # Partially Full -> Can go to Tap (to fill) OR Plant (to deliver)
            target_maps = list(self.tap_bfs_maps.values()) + list(self.plant_bfs_maps.values())

        # 2. Collect ALL physically valid moves first
        valid_candidates = []
        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc
            if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
            if (nr, nc) in self.walls: continue
            if (nr, nc) in occupied: continue
            valid_candidates.append((action_name, nr, nc))

        # 3. Filter for "Improving" moves (Strict Pruning)
        improving_candidates = []
        for action_name, nr, nc in valid_candidates:
            is_improving = False
            for bfs_map in target_maps:
                curr_dist = bfs_map.get((r, c))
                next_dist = bfs_map.get((nr, nc))
                
                if curr_dist is not None and next_dist is not None:
                    if next_dist == curr_dist - 1:
                        is_improving = True
                        break
            
            if is_improving:
                improving_candidates.append((action_name, nr, nc))

        # 4. DECISION: Prefer Improving Moves, Fallback if Blocked
        final_candidates = improving_candidates if improving_candidates else valid_candidates

        # 5. Generate Successors
        for action_name, nr, nc in final_candidates:
            old_robot_entry = robot_entry
            new_robot_entry = (robot_id, nr, nc, load)
            new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}
            
            new_state = (new_robot_states, plant_states, tap_states, total_remaining)
            successors.append(((robot_id, action_name), new_state))

        return successors

    def _get_load_successor(self, robot_states, plant_states, tap_states, total_remaining,
                             robot_entry, robot_id, r, c, load):
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

        new_tap_states = tuple(
            (remaining_tap - 1) if i == tidx else amt
            for i, amt in enumerate(tap_states)
        )

        new_state = (new_robot_states, plant_states, new_tap_states, total_remaining)
        return [((robot_id, "LOAD"), new_state)]

    def _get_pour_successor(self, robot_states, plant_states, tap_states, total_remaining,
                             robot_entry, robot_id, r, c, load):
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

        new_plant_states = tuple(
            (poured + 1) if i == pidx else amt
            for i, amt in enumerate(plant_states)
        )
        
        new_total_remaining = total_remaining - 1
        new_state = (new_robot_states, new_plant_states, tap_states, new_total_remaining)
        return [((robot_id, "POUR"), new_state)]

    def goal_test(self, state):
        return state[3] == 0

    def h_astar(self, node):
        state = node.state
        robot_states, plant_states, _, total_remaining = state
        
        robot_positions = [(r, c) for (_, r, c, _) in sorted(robot_states)]
        unsatisfied_plants = []
        for i, pos in enumerate(self.plant_positions):
            if plant_states[i] < self.plants_targets.get(pos, 0):
                unsatisfied_plants.append(pos)
        
        carried = sum(load for (_, _, _, load) in robot_states)
        interaction_cost = (2 * total_remaining) - carried
        
        if not unsatisfied_plants:
            return max(0, interaction_cost)
            
        num_robots = len(robot_positions)
        num_plants = len(unsatisfied_plants)
        total_vertices = num_robots + num_plants
        
        visited = [False] * total_vertices
        min_dists = [float('inf')] * total_vertices
        min_dists[0] = 0
        mst_weight = 0
        
        for _ in range(total_vertices):
            u = -1
            min_val = float('inf')
            for i in range(total_vertices):
                if not visited[i] and min_dists[i] < min_val:
                    min_val = min_dists[i]
                    u = i
            
            if u == -1 or min_val == float('inf'):
                break
                
            visited[u] = True
            mst_weight += min_val
            
            pos_u = None
            is_robot_u = (u < num_robots)
            if is_robot_u:
                pos_u = robot_positions[u]
            else:
                pos_u = unsatisfied_plants[u - num_robots]

            for v in range(total_vertices):
                if not visited[v]:
                    weight = float('inf')
                    is_robot_v = (v < num_robots)
                    
                    if is_robot_u and is_robot_v:
                        weight = 0
                    else:
                        pos_v = robot_positions[v] if is_robot_v else unsatisfied_plants[v - num_robots]
                        raw_dist = float('inf')
                        
                        if not is_robot_u:
                            raw_dist = self.plant_bfs_maps[pos_u].get(pos_v, float('inf'))
                        elif not is_robot_v:
                            raw_dist = self.plant_bfs_maps[pos_v].get(pos_u, float('inf'))
                            
                        if raw_dist != float('inf'):
                            weight = raw_dist // 2
                            
                    if weight < min_dists[v]:
                        min_dists[v] = weight
                        
        return mst_weight + max(0, interaction_cost)

    def h_gbfs(self, node):
        state = node.state
        robot_states, plant_states, _, total_remaining = state
        remaining = total_remaining
        if remaining == 0: return 0
        
        unsatisfied = [
            pos
            for i, pos in enumerate(self.plant_positions)
            if plant_states[i] < self.plants_targets.get(pos, 0)
        ]
        
        if not unsatisfied: return remaining
        
        robot_positions = [(rr, rc) for (_, rr, rc, _) in robot_states]
        if not robot_positions: return remaining

        min_distance = min(
            abs(pr - rr) + abs(pc - rc)
            for (rr, rc) in robot_positions
            for (pr, pc) in unsatisfied
        )
        return remaining + min_distance

def create_watering_problem(game):
    return WateringProblem(game)

if __name__ == '__main__':
    ex1_check.main()