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
        
        self.plant_positions = tuple(sorted(frozenset(initial["Plants"].keys())))
        self.tap_positions = tuple(sorted(frozenset(initial["Taps"].keys())))

        self.plant_distances = {}
        self.tap_distances = {}
        
        # Heuristic Cache
        self._mst_cache = {}

        # --- OPTIMIZATION: Single Robot Pruning ---
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

        # Pre-calculate Distance Graph for MST
        self.static_dist_graph = {}
        all_pois = list(self.plant_positions) + list(self.tap_positions)
        
        for u in all_pois:
            self.static_dist_graph[u] = {}
            d_map = self.plant_distances[u] if u in self.plant_distances else self.tap_distances[u]
            for v in all_pois:
                if u == v: continue
                d = d_map.get(v, float('inf'))
                self.static_dist_graph[u][v] = d

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
                nr, nc = curr_r + move[0], curr_c + move[1]
                if 0 <= nr < size[0] and 0 <= nc < size[1]:
                    if (nr, nc) not in walls and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        distances[(nr, nc)] = dist + 1
                        queue.append(((nr, nc), dist + 1))
        return distances

    @staticmethod
    def _extract_robot_capacities(robots_data):
        return {rid: cap for rid, (_, _, _, cap) in robots_data.items()}

    def _build_robot_states(self, robots_data):
        return frozenset((rid, r, c, load) for rid, (r, c, load, _) in sorted(robots_data.items()))

    def _build_plant_states(self, plants_data):
        return tuple(0 for _ in self.plant_positions)

    def _build_tap_states(self, taps_data):
        return tuple(taps_data.get(pos, 0) for pos in self.tap_positions)

    def successor(self, state):
        robot_states, plant_states, tap_states, total_remaining = state
        successors = []

        # Safety Check: Radius 6 to prevent blocking in Load Test
        safe_to_prune = True
        if len(robot_states) > 1:
            robot_positions = [(r, c) for (_, r, c, _) in robot_states]
            min_dist = float('inf')
            for i in range(len(robot_positions)):
                for j in range(i + 1, len(robot_positions)):
                    d = abs(robot_positions[i][0] - robot_positions[j][0]) + \
                        abs(robot_positions[i][1] - robot_positions[j][1])
                    if d < min_dist: min_dist = d
            if min_dist <= 6:
                safe_to_prune = False

        occupied = {(r, c) for (_, r, c, _) in robot_states}

        for robot_entry in robot_states:
            robot_id, r, c, load = robot_entry
            
            # Local Adjacency Check
            is_blocked = False
            if len(robot_states) > 1:
                for move in _MOVES.values():
                    if (r + move[0], c + move[1]) in occupied:
                        is_blocked = True; break
            
            use_strict_logic = safe_to_prune and not is_blocked

            if use_strict_logic:
                # 1. Pour
                pour_succ = self._get_pour_successor(state, robot_entry)
                if pour_succ:
                    successors.extend(pour_succ)
                    continue

                # 2. Load
                load_succ = self._get_load_successor(state, robot_entry)
                if load_succ:
                    successors.extend(load_succ)
                    continue
                
                # 3. Strict Movement
                successors.extend(self._get_strict_movement_successors(state, robot_entry))
            
            else:
                # Safe Mode
                successors.extend(self._get_pour_successor(state, robot_entry))
                successors.extend(self._get_load_successor(state, robot_entry))
                successors.extend(self._get_movement_successors(state, robot_entry))

        return successors

    def _get_strict_movement_successors(self, state, robot_entry):
        robot_states, plant_states, tap_states, total_remaining = state
        robot_id, r, c, load = robot_entry
        successors = []
        occupied = {(rid, rr, cc) for (rid, rr, cc, _) in robot_states if rid != robot_id}

        # Identify Active Targets
        active_maps = []
        capacity = self.robots_capacities.get(robot_id, 0)
        
        # A) Needs Water?
        if load < capacity and load < total_remaining:
             for i, pos in enumerate(self.tap_positions):
                if tap_states[i] > 0:
                    active_maps.append(self.tap_distances[pos])

        # B) Has Water?
        if load > 0:
            for i, pos in enumerate(self.plant_positions):
                target = self.plants_targets.get(pos, 0)
                if plant_states[i] < target:
                    active_maps.append(self.plant_distances[pos])

        if not active_maps: return [] 

        # Pre-fetch current distances
        valid_objectives = []
        for d_map in active_maps:
            d = d_map.get((r, c), float('inf'))
            if d != float('inf'):
                valid_objectives.append((d_map, d))

        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc
            if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
            if (nr, nc) in self.walls: continue
            
            is_occ = False
            for _, orr, occ in occupied:
                if nr == orr and nc == occ: is_occ = True; break
            if is_occ: continue

            # STRICT CHECK: Must improve at least one target
            is_productive = False
            for d_map, cur_dist in valid_objectives:
                if d_map.get((nr, nc), float('inf')) < cur_dist:
                    is_productive = True
                    break
            
            if is_productive:
                new_robot_entry = (robot_id, nr, nc, load)
                new_robot_states = (robot_states - {robot_entry}) | {new_robot_entry}
                new_state = (new_robot_states, plant_states, tap_states, total_remaining)
                successors.append(((robot_id, action_name), new_state))
                
        return successors

    def _get_movement_successors(self, state, robot_entry):
        robot_states, plant_states, tap_states, total_remaining = state
        robot_id, r, c, load = robot_entry
        successors = []
        occupied = {(rid, rr, cc) for (rid, rr, cc, _) in robot_states if rid != robot_id}

        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc
            if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
            if (nr, nc) in self.walls: continue
            
            is_occ = False
            for _, orr, occ in occupied:
                if nr == orr and nc == occ: is_occ = True; break
            if is_occ: continue

            new_robot_entry = (robot_id, nr, nc, load)
            new_robot_states = (robot_states - {robot_entry}) | {new_robot_entry}
            new_state = (new_robot_states, plant_states, tap_states, total_remaining)
            successors.append(((robot_id, action_name), new_state))
        return successors

    def _get_load_successor(self, state, robot_entry):
        robot_states, plant_states, tap_states, total_remaining = state
        robot_id, r, c, load = robot_entry
        
        if (r, c) not in self.tap_positions: return []
        if load >= total_remaining: return [] 

        tidx = self.tap_positions.index((r, c))
        remaining_tap = tap_states[tidx]
        if remaining_tap <= 0: return []
        
        capacity = self.robots_capacities.get(robot_id, 0)
        if load >= capacity: return []

        new_robot_entry = (robot_id, r, c, load + 1)
        new_robot_states = (robot_states - {robot_entry}) | {new_robot_entry}
        new_tap_states = tuple((remaining_tap - 1) if i == tidx else amt for i, amt in enumerate(tap_states))
        return [((robot_id, "LOAD"), (new_robot_states, plant_states, new_tap_states, total_remaining))]

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
        new_plant_states = tuple((poured + 1) if i == pidx else amt for i, amt in enumerate(plant_states))
        new_total_remaining = total_remaining - 1
        return [((robot_id, "POUR"), (new_robot_states, new_plant_states, tap_states, new_total_remaining))]

    def goal_test(self, state):
        return state[3] == 0

    def _get_mst_and_proximity(self, state):
        """
        Helper to calculate the MST cost and Robot Proximity used in both heuristics.
        Returns: (internal_mst_cost, min_proximity)
        """
        robot_states, plant_states, _, total_remaining = state
        carried = sum(load for (_, _, _, load) in robot_states)
        
        # Build cache key
        mst_nodes_indices = []
        for i, pos in enumerate(self.plant_positions):
            if plant_states[i] < self.plants_targets.get(pos, 0):
                mst_nodes_indices.append(i)
        
        need_water = carried < total_remaining
        cache_key = (tuple(mst_nodes_indices), need_water)

        # 1. Internal MST (Connectivity between objectives)
        if cache_key in self._mst_cache:
            internal_mst_cost = self._mst_cache[cache_key]
            mst_nodes = None 
        else:
            mst_nodes = set()
            for idx in mst_nodes_indices:
                mst_nodes.add(self.plant_positions[idx])
            if need_water:
                mst_nodes.update(self.tap_positions)
            
            internal_mst_cost = 0
            if mst_nodes:
                start_node = next(iter(mst_nodes))
                min_dists = {node: float('inf') for node in mst_nodes}
                min_dists[start_node] = 0
                unvisited = set(mst_nodes)
                
                while unvisited:
                    u = min(unvisited, key=lambda k: min_dists[k])
                    dist_u = min_dists[u]
                    if dist_u == float('inf'): break
                    internal_mst_cost += dist_u
                    unvisited.remove(u)
                    
                    for v in unvisited:
                        dist_uv = self.static_dist_graph[u].get(v, float('inf'))
                        if dist_uv < min_dists[v]: min_dists[v] = dist_uv
            
            self._mst_cache[cache_key] = internal_mst_cost

        # 2. Proximity (Robot -> MST)
        min_proximity = 0
        if internal_mst_cost > 0 or (cache_key[0] or cache_key[1]):
            if mst_nodes is None:
                mst_nodes = set()
                for idx in mst_nodes_indices:
                    mst_nodes.add(self.plant_positions[idx])
                if need_water:
                    mst_nodes.update(self.tap_positions)
            
            min_proximity = float('inf')
            for target in mst_nodes:
                for (_, rr, rc, _) in robot_states:
                    if target in self.plant_distances:
                        d = self.plant_distances[target].get((rr, rc), float('inf'))
                    else:
                        d = self.tap_distances[target].get((rr, rc), float('inf'))
                    if d < min_proximity: min_proximity = d
            
            if min_proximity == float('inf'): min_proximity = 0
            
        return internal_mst_cost, min_proximity

    def h_astar(self, node):
        state = node.state
        robot_states, _, _, total_remaining = state
        
        remaining = total_remaining
        carried = sum(load for (_, _, _, load) in robot_states)
        action_cost = max(0, 2 * remaining - carried)

        mst_cost, prox_cost = self._get_mst_and_proximity(state)
        
        return action_cost + mst_cost + prox_cost

    def h_gbfs(self, node):
        state = node.state
        robot_states, _, _, total_remaining = state
        if total_remaining == 0: return 0

        # Water Needed: Plants + Robot Refills
        carried = sum(load for (_, _, _, load) in robot_states)
        water_needed = total_remaining + max(0, total_remaining - carried)

        # MST Cost (Same as A*)
        mst_cost, prox_cost = self._get_mst_and_proximity(state)

        return water_needed + mst_cost + prox_cost

def create_watering_problem(game):
    return WateringProblem(game)