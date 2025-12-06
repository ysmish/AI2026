import search
import math

id = ["No numbers - I'm special!"]

_MOVES = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

class WateringProblem(search.Problem):
    """This class implements a Multi-Tap Plant Watering problem"""

    def __init__(self, initial):
        """ Constructor initializes the problem and pre-calculates maps. """
        
        self.size = initial["Size"]
        self.walls = frozenset(initial.get("Walls", set()))
        self.plants_targets = dict(initial["Plants"])
        self.robots_capacities = self._extract_robot_capacities(initial["Robots"])
        
        # Sorted tuples for deterministic ordering
        self.plant_positions = tuple(sorted(frozenset(initial["Plants"].keys())))
        self.tap_positions = tuple(sorted(frozenset(initial["Taps"].keys())))

        # Initialization of distance maps
        self.plant_distances = {}
        self.tap_distances = {}

        # --- OPTIMIZATION 1: Single Robot Pruning & Reuse ---
        if len(initial["Robots"]) == 1:
            robot_start = list(initial["Robots"].values())[0][0:2]
            pois = set(self.plant_positions) | set(self.tap_positions) | {robot_start}
            
            # Compute BFS from all POIs using original walls
            poi_distances = {}
            for p in pois:
                poi_distances[p] = self._bfs_map(p, self.walls, self.size)

            keep_cells = set()
            poi_list = list(pois)
            
            candidate_cells = set().union(*[d.keys() for d in poi_distances.values()])

            # Keep cells that are on ANY shortest path between two POIs
            for i in range(len(poi_list)):
                u = poi_list[i]
                for j in range(i + 1, len(poi_list)):
                    v = poi_list[j]
                    dist_uv = poi_distances[u].get(v)
                    if dist_uv is None: continue

                    for cell in candidate_cells:
                        if cell in keep_cells: continue
                        d_uc = poi_distances[u].get(cell)
                        d_cv = poi_distances[v].get(cell)
                        if d_uc is not None and d_cv is not None:
                            if d_uc + d_cv == dist_uv:
                                keep_cells.add(cell)
            
            # Update self.walls to block everything else
            new_walls = set(self.walls)
            for r in range(self.size[0]):
                for c in range(self.size[1]):
                    if (r, c) not in keep_cells:
                        new_walls.add((r, c))
            self.walls = frozenset(new_walls)

            # Reuse BFS results
            for p in self.plant_positions:
                self.plant_distances[p] = poi_distances[p]
            for t in self.tap_positions:
                self.tap_distances[t] = poi_distances[t]

        else:
            # Multi-robot case
            for plant_pos in self.plant_positions:
                self.plant_distances[plant_pos] = self._bfs_map(plant_pos, self.walls, self.size)

            for tap_pos in self.tap_positions:
                self.tap_distances[tap_pos] = self._bfs_map(tap_pos, self.walls, self.size)
        # --- END OPTIMIZATION 1 ---

        # 2. Pre-calculate MST Graph
        self.mst_graph = {}
        for p1 in self.plant_positions:
            # Distance to nearest tap
            min_tap_dist = float('inf')
            for tap_pos in self.tap_positions:
                d = self.plant_distances[p1].get(tap_pos, float('inf'))
                if d < min_tap_dist:
                    min_tap_dist = d
            
            # Distances to other plants
            siblings = {}
            for p2 in self.plant_positions:
                if p1 == p2: continue
                d = self.plant_distances[p1].get(p2, float('inf'))
                siblings[p2] = d
            
            self.mst_graph[p1] = {
                'nearest_tap': min_tap_dist,
                'siblings': siblings
            }

        # Initialize dynamic state
        robot_states = self._build_robot_states(initial["Robots"])
        plant_states = self._build_plant_states(initial["Plants"])
        tap_states = self._build_tap_states(initial["Taps"])
        total_remaining = sum(initial["Plants"].values())

        initial_state = (robot_states, plant_states, tap_states, total_remaining)
        search.Problem.__init__(self, initial_state)

    @staticmethod
    def _bfs_map(start_pos, walls, size):
        queue = [(start_pos, 0)]
        distances = {start_pos: 0}
        visited = {start_pos}
        
        idx = 0
        while idx < len(queue):
            (curr_r, curr_c), dist = queue[idx]
            idx += 1
            
            for move in _MOVES.values():
                dr, dc = move
                nr, nc = curr_r + dr, curr_c + dc
                
                if 0 <= nr < size[0] and 0 <= nc < size[1]:
                    if (nr, nc) not in walls and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        distances[(nr, nc)] = dist + 1
                        queue.append(((nr, nc), dist + 1))
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

        for robot_entry in robot_states:
            # --- OPTIMIZATION 2: Action Dominance ---
            # If we can POUR, we MUST POUR (Action Priority 1)
            # This prunes all movement and loading branches for this robot in this state.
            pour_actions = self._get_pour_successor(state, robot_entry)
            if pour_actions:
                successors.extend(pour_actions)
                continue  # Skip LOAD and MOVE

            # If we can LOAD, we MUST LOAD (Action Priority 2)
            # This prunes all movement branches.
            load_actions = self._get_load_successor(state, robot_entry)
            if load_actions:
                successors.extend(load_actions)
                continue # Skip MOVE
            
            # Fallback: Generate Movement (Priority 3)
            successors.extend(self._get_movement_successors(state, robot_entry))

        return successors

    def _get_movement_successors(self, state, robot_entry):
        robot_states, plant_states, tap_states, total_remaining = state
        robot_id, r, c, load = robot_entry
        successors = []

        occupied = {
            (robot_r, robot_c)
            for (rid, robot_r, robot_c, _) in robot_states
            if rid != robot_id
        }

        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc
            
            if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
            if (nr, nc) in self.walls: continue
            if (nr, nc) in occupied: continue

            new_robot_entry = (robot_id, nr, nc, load)
            new_robot_states = (robot_states - {robot_entry}) | {new_robot_entry}
            new_state = (new_robot_states, plant_states, tap_states, total_remaining)
            successors.append(((robot_id, action_name), new_state))
        return successors

    def _get_load_successor(self, state, robot_entry):
        robot_states, plant_states, tap_states, total_remaining = state
        robot_id, r, c, load = robot_entry
        
        if (r, c) not in self.tap_positions: return []

        tidx = self.tap_positions.index((r, c))
        remaining_tap = tap_states[tidx]
        
        if remaining_tap <= 0: return []
        capacity = self.robots_capacities.get(robot_id, 0)
        if load >= capacity: return []

        new_robot_entry = (robot_id, r, c, load + 1)
        new_robot_states = (robot_states - {robot_entry}) | {new_robot_entry}
        new_tap_states = tuple(
            (remaining_tap - 1) if i == tidx else amt
            for i, amt in enumerate(tap_states)
        )
        new_state = (new_robot_states, plant_states, new_tap_states, total_remaining)
        return [((robot_id, "LOAD"), new_state)]

    def _get_pour_successor(self, state, robot_entry):
        robot_states, plant_states, tap_states, total_remaining = state
        robot_id, r, c, load = robot_entry
        
        if load <= 0: return []
        if (r, c) not in self.plant_positions: return []

        pidx = self.plant_positions.index((r, c))
        poured = plant_states[pidx]
        target = self.plants_targets.get((r, c), 0)
        
        if poured >= target: return []

        new_robot_entry = (robot_id, r, c, load - 1)
        new_robot_states = (robot_states - {robot_entry}) | {new_robot_entry}
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
        
        remaining = total_remaining
        carried = sum(load for (_, _, _, load) in robot_states)
        action_cost = max(0, 2 * remaining - carried)

        unsatisfied = []
        for i, pos in enumerate(self.plant_positions):
            if plant_states[i] < self.plants_targets.get(pos, 0):
                unsatisfied.append(pos)
        
        mst_cost = 0
        if unsatisfied:
            min_dists = {pos: self.mst_graph[pos]['nearest_tap'] for pos in unsatisfied}
            unvisited = set(unsatisfied)
            
            while unvisited:
                u = min(unvisited, key=lambda k: min_dists[k])
                dist_u = min_dists[u]
                
                if dist_u == float('inf'): break
                
                mst_cost += dist_u
                unvisited.remove(u)
                
                for v in unvisited:
                    dist_uv = self.mst_graph[u]['siblings'].get(v, float('inf'))
                    if dist_uv < min_dists[v]:
                        min_dists[v] = dist_uv

        num_robots = len(robot_states)
        travel_cost = mst_cost / num_robots if num_robots > 0 else mst_cost
        
        return action_cost + travel_cost

    def h_gbfs(self, node):
        state = node.state
        robot_states, plant_states, _, total_remaining = state
        
        remaining = total_remaining
        if remaining == 0: return 0

        unsatisfied_indices = [
            i for i, pos in enumerate(self.plant_positions)
            if plant_states[i] < self.plants_targets.get(pos, 0)
        ]

        if not unsatisfied_indices: return remaining
        
        min_distance = float('inf')
        
        for (_, rr, rc, load) in robot_states:
            if load > 0:
                for pidx in unsatisfied_indices:
                    plant_pos = self.plant_positions[pidx]
                    dist = self.plant_distances[plant_pos].get((rr, rc), float('inf'))
                    if dist < min_distance:
                        min_distance = dist
            else:
                for tap_pos in self.tap_positions:
                    dist = self.tap_distances[tap_pos].get((rr, rc), float('inf'))
                    if dist < min_distance:
                        min_distance = dist

        if min_distance == float('inf'): min_distance = 0
        return remaining + min_distance

def create_watering_problem(game):
    return WateringProblem(game)