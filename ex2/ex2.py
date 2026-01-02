import ext_plant
from collections import deque
import math
import bisect
import sys

# Update with your ID
id = ["216764803"]

# Directions mapping for BFS
_MOVES = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

# --- Configuration (tunable) ---
RELIABILITY_THRESHOLD = 0.8
SMALL_BOARD_AREA = 16
LOAD_AVOID_STEPS = 3  # if remaining steps < this, avoid forcing LOAD actions


# ==============================================================================
# 1. COPIED UTILS & SEARCH CLASSES (Dependencies for A*)
# ==============================================================================

infinity = 1.0e400

def update(x, **entries):
    if isinstance(x, dict):
        x.update(entries)
    else:
        x.__dict__.update(entries)
    return x

def memoize(fn, slot=None):
    if slot:
        def memoized_fn(obj, *args):
            if hasattr(obj, slot):
                return getattr(obj, slot)
            else:
                val = fn(obj, *args)
                setattr(obj, slot, val)
                return val
    else:
        def memoized_fn(*args):
            if not memoized_fn.cache.has_key(args):
                memoized_fn.cache[args] = fn(*args)
            return memoized_fn.cache[args]
        memoized_fn.cache = {}
    return memoized_fn

class Queue:
    def __init__(self):
        raise NotImplementedError

    def extend(self, items):
        for item in items: self.append(item)

class PriorityQueue(Queue):
    def __init__(self, order=min, f=lambda x: x):
        update(self, A=[], order=order, f=f)

    def append(self, item):
        bisect.insort(self.A, (self.f(item), item))

    def __len__(self):
        return len(self.A)

    def pop(self):
        if self.order == min:
            return self.A.pop(0)[1]
        else:
            return self.A.pop()[1]

class Node:
    """A node in a search tree."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        update(self, state=state, parent=parent, action=action,
               path_cost=path_cost, depth=0)
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def path(self):
        x, result = self, [self]
        while x.parent:
            result.append(x.parent)
            x = x.parent
        return result

    def expand(self, problem):
        return [Node(next, self, act,
                     problem.path_cost(self.path_cost, self.state, act, next))
                for (act, next) in problem.successor(self.state)]

    # --- Added Comparison Methods (Crucial for PriorityQueue) ---
    def __eq__(self, other):
        return (self.f == other.f)

    def __ne__(self, other):
        return not (self == other)

    def __lt__(self, other):
        return (self.f < other.f)

    def __gt__(self, other):
        return (self.f > other.f)

    def __le__(self, other):
        return (self < other) or (self == other)

    def __ge__(self, other):
        return (self > other) or (self == other)

class Problem:
    def __init__(self, initial, goal=None):
        self.initial = initial
        self.goal = goal

    def successor(self, state):
        raise NotImplementedError

    def goal_test(self, state):
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        return c + 1

    def value(self):
        raise NotImplementedError

# --- Search Algorithms ---

def graph_search(problem, fringe):
    closed = {}
    expanded = 0
    fringe.append(Node(problem.initial))
    while fringe:
        node = fringe.pop()
        if problem.goal_test(node.state):
            return node, expanded
        if node.state not in closed:
            closed[node.state] = True
            fringe.extend(node.expand(problem))
            expanded += 1
    return None

def best_first_graph_search(problem, f):
    f = memoize(f, 'f')
    return graph_search(problem, PriorityQueue(min, f))

def astar_search(problem, h=None):
    h = h or problem.h
    def f(n):
        return max(getattr(n, 'f', -infinity), n.path_cost + h(n))
    return best_first_graph_search(problem, f)


# ==============================================================================
# 2. WATERING PROBLEM CLASS (From ex1.py)
# ==============================================================================

class WateringProblem(Problem):
    """
    This class implements the Watering Problem for AI Search.
    It solves the problem of moving robots to taps to fill up and to plants to pour water.
    """

    def __init__(self, initial):
        # --- 1. Parse Initial State ---
        self.size = initial["Size"]
        self.walls = frozenset(initial.get("Walls", set()))
        self.plants_targets = dict(initial["Plants"])
        self.robots_capacities = self._extract_robot_capacities(initial["Robots"])
        
        # Sort positions for deterministic behavior
        self.plant_positions = tuple(sorted(frozenset(initial["Plants"].keys())))
        self.tap_positions = tuple(sorted(frozenset(initial["Taps"].keys())))

        # expected reward per plant (if provided in problem dict)
        self.plants_reward = dict(initial.get("plants_reward", {}))
        self.plant_expected_rewards = {}
        for p_pos in self.plant_positions:
            rewards = self.plants_reward.get(p_pos, [])
            self.plant_expected_rewards[p_pos] = (sum(rewards) / len(rewards)) if rewards else 0.0

        # --- 2. Pre-Calculate BFS Maps ---
        self.plant_bfs_maps = {}
        for p_pos in self.plant_positions:
            self.plant_bfs_maps[p_pos] = self._bfs_map(p_pos)
        
        self.tap_bfs_maps = {}
        for t_pos in self.tap_positions:
            self.tap_bfs_maps[t_pos] = self._bfs_map(t_pos)

        # --- 3. OPTIMIZATION: Highway Pruning (Single Robot) ---
        robot_starts = set((r, c) for (r, c, _, _) in initial["Robots"].values())
        num_robots = len(initial["Robots"])

        if num_robots == 1:
            start_pos = list(robot_starts)[0]
            start_bfs_map = self._bfs_map(start_pos)
            pois = set(self.plant_positions) | set(self.tap_positions) | {start_pos}
            
            def get_dist_map(p):
                if p == start_pos: return start_bfs_map
                if p in self.plant_bfs_maps: return self.plant_bfs_maps[p]
                if p in self.tap_bfs_maps: return self.tap_bfs_maps[p]
                return None

            valid_highway_cells = set()
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

            for src, dst in pairs:
                d_map_src = get_dist_map(src)
                d_map_dst = get_dist_map(dst)
                if not d_map_src or dst not in d_map_src: continue
                
                shortest_len = d_map_src[dst]
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
                        if (nr, nc) in visited_local: continue
                        if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
                        
                        d_n_src = d_map_src.get((nr, nc))
                        d_n_dst = d_map_dst.get((nr, nc))
                        
                        if d_n_src is not None and d_n_dst is not None:
                            if d_n_src + d_n_dst == shortest_len:
                                valid_highway_cells.add((nr, nc))
                                visited_local.add((nr, nc))
                                queue.append((nr, nc))

            if valid_highway_cells: 
                new_walls = set(self.walls)
                for r in range(self.size[0]):
                    for c in range(self.size[1]):
                        if (r, c) not in self.walls and (r, c) not in valid_highway_cells:
                            new_walls.add((r, c))
                self.walls = frozenset(new_walls)

            # Re-calculate maps
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
        Problem.__init__(self, initial_state)

    def _bfs_map(self, start_pos):
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
        current_total_load = sum(l for (_, _, _, l) in robot_states)

        for robot_entry in sorted(robot_states):
            robot_id, r, c, load = robot_entry

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

            if possible_actions:
                successors.extend(possible_actions)
                if not is_blocking:
                    continue 

            successors.extend(
                self._get_movement_successors(robot_states, plant_states, tap_states, total_remaining,
                                              robot_entry, robot_id, r, c, load, occupied)
            )

        # Fallback: choose a safe legal action instead of RESET to avoid resetting the game
        def fallback_action(state_obj, preferred_robot_ids=None):
            robots_t, plants_t, taps_t, total = state_obj
            occupied = {(rr, cc) for (rrid, (rr, cc), l) in robots_t}
            # prefer highest reliability robot among preferred_robot_ids, else any
            probs_map = self.game.get_problem().get("robot_chosen_action_prob", {})
            candidates = []
            for (rid, (rpos_r, rpos_c), l) in robots_t:
                if preferred_robot_ids is not None and rid not in preferred_robot_ids:
                    continue
                candidates.append((probs_map.get(rid, 0), rid, (rpos_r, rpos_c), l))
            if not candidates:
                # try any robot
                for (rid, (rpos_r, rpos_c), l) in robots_t:
                    candidates.append((probs_map.get(rid, 0), rid, (rpos_r, rpos_c), l))
            if not candidates:
                return "RESET"
            # pick robot with highest reliability
            candidates.sort(reverse=True)
            for _, rid, (rpos_r, rpos_c), l in candidates:
                pos = (rpos_r, rpos_c)
                base_actions = self.game.applicable_actions.get(pos, [])
                for a in base_actions:
                    if a in ("UP", "DOWN", "LEFT", "RIGHT"):
                        if a == "UP": t = (pos[0]-1, pos[1])
                        elif a == "DOWN": t = (pos[0]+1, pos[1])
                        elif a == "LEFT": t = (pos[0], pos[1]-1)
                        else: t = (pos[0], pos[1]+1)
                        # ensure not occupied by another robot
                        if t in occupied and t != pos:
                            continue
                        return f"{a}({rid})"
                # try LOAD/POUR if applicable
                if "LOAD" in base_actions:
                    # check tap and capacity
                    taps_positions = {p for (p, w) in taps_t}
                    if pos in taps_positions:
                        cap = self.game.get_capacities().get(rid, 0)
                        if l < cap:
                            return f"LOAD({rid})"
                if "POUR" in base_actions:
                    plants_positions = {p for (p, need) in plants_t}
                    if pos in plants_positions and l > 0:
                        return f"POUR({rid})"
            return "RESET"
        return successors

    def _get_movement_successors(self, robot_states, plant_states, tap_states, total_remaining,
                                 robot_entry, robot_id, r, c, load, occupied):
        successors = []
        capacity = self.robots_capacities[robot_id]
        
        if load == 0:
            target_maps = list(self.tap_bfs_maps.values())
        elif load == capacity:
            target_maps = list(self.plant_bfs_maps.values())
        else:
            target_maps = list(self.tap_bfs_maps.values()) + list(self.plant_bfs_maps.values())

        valid_candidates = []
        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc
            if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
            if (nr, nc) in self.walls: continue
            if (nr, nc) in occupied: continue
            valid_candidates.append((action_name, nr, nc))

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

        final_candidates = improving_candidates if improving_candidates else valid_candidates

        for action_name, nr, nc in final_candidates:
            old_robot_entry = robot_entry
            new_robot_entry = (robot_id, nr, nc, load)
            new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}
            
            new_state = (new_robot_states, plant_states, tap_states, total_remaining)
            
            action_str = f"{action_name}{{{robot_id}}}"
            successors.append((action_str, new_state))

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
        
        action_str = f"LOAD{{{robot_id}}}"
        return [(action_str, new_state)]

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
        
        action_str = f"POUR{{{robot_id}}}"
        return [(action_str, new_state)]

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
        total_vertices = num_robots + len(unsatisfied_plants)
        
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
            
            if u == -1 or min_val == float('inf'): break
                
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
                            # bias distances by expected reward: higher EV -> lower effective weight
                            reward_factor = 1.0
                            # if pos_u is a plant, include its expected reward
                            if (not is_robot_u) and pos_u in self.plant_expected_rewards:
                                reward_factor *= (1.0 + float(self.plant_expected_rewards.get(pos_u, 0.0)))
                            # if pos_v is a plant, include its expected reward
                            if (not is_robot_v) and pos_v in self.plant_expected_rewards:
                                reward_factor *= (1.0 + float(self.plant_expected_rewards.get(pos_v, 0.0)))
                            weight = raw_dist / (1.5 * reward_factor)
                            
                    if weight < min_dists[v]:
                        min_dists[v] = weight
                        
        return mst_weight + max(0, interaction_cost)
    
    # Mapping 'h' to 'h_astar' for the search algorithm
    h = h_astar


class Controller:
    """Controller for the stochastic Plant Watering problem using BFS Gradient Maps."""

    def __init__(self, game: ext_plant.Game):
        """
        Constructor. Parses input and pre-calculates BFS distance maps
        to create a 'gradient' that guides robots from ANY cell.
        """
        # This part is for the specific ex2 task (not strictly needed for the a_star request but good to keep)
        self.game = game

    def a_star(self, state):
        """
        Returns an optimal moves plan using the exact A* logic from ex1.py.
        """
        # 1. Initialize the problem with the given state
        problem = WateringProblem(state)
        
        # 2. Run A* search
        # Note: In the provided search.py, graph_search returns (node, expanded).
        # astar_search calls best_first_graph_search which calls graph_search.
        result = astar_search(problem)
        
        # Handle the result. If no solution found, result might be None.
        if result is None:
            return []
            
        # The provided search.py returns a tuple (node, expanded)
        goal_node = result[0]
        
        # 3. Reconstruct the path of actions
        actions = []
        curr = goal_node
        while curr.parent is not None:
            actions.append(curr.action)
            curr = curr.parent
            
        # Reverse to get Start -> Goal
        return actions[::-1]

    def choose_next_action(self, state):
        """ Choose the next action given a state."""
        # Maintain planner state across calls
        if not hasattr(self, 'plan_actions'):
            self.plan_actions = []
            self.plan_idx = 0
            self.prev_state = None
            self.last_action = None
            self.consecutive_pour_failures = 0
            self.stupid_robots = set()

        # Helper: build problem dict for A* from current game problem and a subset of robots and plants
        def build_problem_from_state(cur_state, chosen_robot_ids, plants_to_keep=None):
            # base problem from original
            base = dict(self.game.get_problem())
            robots_t, plants_t, taps_t, total = cur_state

            # Robots dict: id -> (r, c, load, cap)
            capacities = self.game.get_capacities()

            # Use the dynamic walls we calculated earlier (includes stupid robots as walls)
            current_walls_for_astar = set(dynamic_walls)

            robots_dict = {}
            for (rid, (r_pos_r, r_pos_c), load) in robots_t:
                if rid in chosen_robot_ids:
                    cap = capacities.get(rid, 0)
                    robots_dict[rid] = (r_pos_r, r_pos_c, load, cap)
                # Note: Stupid robots are already in current_walls_for_astar

            # Plants dict
            plants_dict = {}
            for (pos, need) in plants_t:
                if (plants_to_keep is None) or (pos in plants_to_keep):
                    plants_dict[pos] = need

            taps_dict = {pos: water for (pos, water) in taps_t}

            prob = {
                "Size": base.get("Size"),
                "Walls": current_walls_for_astar,  # A* now sees the blockage
                "Taps": taps_dict,
                "Plants": plants_dict,
                "plants_reward": base.get("plants_reward", {}),
                "Robots": robots_dict,
            }
            return prob

        # Helper: pick chosen robots according to probabilities
        probs = self.game.get_problem().get("robot_chosen_action_prob", {})
        # derive robot ids from state (all) and active (non-zero prob)
        robot_ids_all = [rid for (rid, (r, c), load) in state[0]]
        active_robot_ids = [rid for rid in robot_ids_all if float(probs.get(rid, 0.0)) > 0.0]

        if active_robot_ids:
            chosen = [rid for rid in active_robot_ids if probs.get(rid, 0) >= RELIABILITY_THRESHOLD]
            if not chosen:
                # choose single active robot with highest prob
                best = max(active_robot_ids, key=lambda x: probs.get(x, 0))
                chosen = [best]
            else:
                # Last-resort: ensure plants have a nearby robot available for planning.
                # If a plant's nearest robot isn't in `chosen`, add that robot (even
                # if its reliability is below RELIABILITY_THRESHOLD) so we can plan
                # using the geographically-appropriate agent as a fallback.
                try:
                    # get robot positions from current state
                    robot_pos_map = {rid: (rpos_r, rpos_c) for (rid, (rpos_r, rpos_c), l) in state[0]}
                    for (p_pos, need) in state[1]:
                        # find nearest robot to this plant
                        best_r = None; best_d = float('inf')
                        for rid in robot_pos_map:
                            rp = robot_pos_map[rid]
                            d = bfs_dist(rp, [p_pos]).get(p_pos, float('inf'))
                            if d < best_d:
                                best_d = d; best_r = rid
                        if best_r is not None and best_r not in chosen:
                            # include even low-prob robots as last resort (if reachable)
                            if not math.isinf(best_d):
                                chosen.append(best_r)
                except Exception:
                    pass
        else:
            # no active robots (all have 0 prob) -> fall back to using all robots
            chosen = [max(robot_ids_all, key=lambda x: probs.get(x, 0))]

        stupid = set(robot_ids_all) - set(chosen)

        # Calculate Dynamic Walls (Static Walls + Stupid Robots)
        game_walls = set(self.game.get_problem().get("Walls", set()))
        dynamic_walls = set(game_walls)
        for (rid, (r, c), _) in state[0]:
            if rid in stupid:
                dynamic_walls.add((r, c))

        horizon = self.game.get_problem().get("horizon", 0)
        remaining_steps = self.game.get_max_steps() - self.game.get_current_steps()

        # Function to compute threshold (time-aware urgency)
        def compute_threshold(pmin, num_stupid, remaining=remaining_steps):
            # urgency increases as remaining steps shrink (simple linear factor)
            urgency = 1.0 + max(0.0, (1.0 - (remaining / max(1.0, horizon))))
            base = horizon + horizon * (1 - pmin) * (5.0 / 3.0) + num_stupid * 2
            return int(math.ceil(base * urgency))

        # Function to run A* with optional plant pruning
        def run_astar_with_pruned_plants(plants_to_keep=None):
            prob_dict = build_problem_from_state(state, chosen, plants_to_keep)
            actions = self.a_star(prob_dict)
            return actions

        # Run A* with an explicit chosen set of robots
        def run_astar_with_chosen(chosen_set, plants_to_keep=None):
            prob_dict = build_problem_from_state(state, chosen_set, plants_to_keep)
            actions = self.a_star(prob_dict)
            return actions

        # Local fallback: choose a safe legal action instead of RESET
        def fallback_local(preferred_robot_ids=None):
            robots_t, plants_t, taps_t, total = state
            occupied = {(rr, cc) for (rrid, (rr, cc), l) in robots_t}
            probs_map = self.game.get_problem().get("robot_chosen_action_prob", {})
            candidates = []
            # prefer active robots (non-zero prob)
            preferred_pool = []
            for (rid, (rpos_r, rpos_c), l) in robots_t:
                if float(probs_map.get(rid, 0.0)) > 0.0:
                    preferred_pool.append((rid, (rpos_r, rpos_c), l))

            iters = preferred_pool if preferred_pool else [(rid, pos, l) for (rid, pos, l) in robots_t]
            for (rid, (rpos_r, rpos_c), l) in iters:
                if preferred_robot_ids is not None and rid not in preferred_robot_ids:
                    continue
                candidates.append((probs_map.get(rid, 0), rid, (rpos_r, rpos_c), l))
            if not candidates:
                return "RESET"
            candidates.sort(reverse=True)
            taps_positions = {p for (p, w) in taps_t}
            plants_positions = {p for (p, need) in plants_t}
            for _, rid, (rpos_r, rpos_c), l in candidates:
                pos = (rpos_r, rpos_c)
                base_actions = self.game.applicable_actions.get(pos, [])
                occ_others = {(rr, cc) for (rrid, (rr, cc), _) in robots_t if rrid != rid}
                # Prefer POUR when on a plant and have load
                if "POUR" in base_actions and pos in plants_positions and l > 0:
                    return f"POUR({rid})"
                # Then prefer LOAD when on a tap and have capacity
                if "LOAD" in base_actions and pos in taps_positions:
                    cap = self.game.get_capacities().get(rid, 0)
                    if l < cap:
                        return f"LOAD({rid})"
                # Finally try safe movements
                for a in base_actions:
                    if a in ("UP", "DOWN", "LEFT", "RIGHT"):
                        if a == "UP": t = (pos[0]-1, pos[1])
                        elif a == "DOWN": t = (pos[0]+1, pos[1])
                        elif a == "LEFT": t = (pos[0], pos[1]-1)
                        else: t = (pos[0], pos[1]+1)
                        if t in occ_others:
                            continue
                        return f"{a}({rid})"
            return "RESET"

        # Prepare plant pruning candidate list (sorted by expected reward ascending)
        plants_reward = self.game.get_problem().get("plants_reward", {})
        # current plant positions
        current_plants = [pos for (pos, need) in state[1]]
        expected_rewards = {}
        for pos in current_plants:
            rewards = plants_reward.get(pos, [])
            expected_rewards[pos] = (sum(rewards) / len(rewards)) if rewards else 0

        # Helper: BFS distance on the static map (respect walls)
        def bfs_dist(start, targets, walls_to_use=None):
            # returns dict target->dist (inf if unreachable)
            size = self.game.get_problem().get("Size", (0, 0))
            walls = walls_to_use if walls_to_use is not None else set(self.game.get_problem().get("Walls", set()))
            from collections import deque as _dq
            q = _dq()
            q.append((start, 0))
            seen = {start}
            dists = {start: 0}
            while q:
                (r, c), d = q.popleft()
                for dr, dc in _MOVES.values():
                    nr, nc = r + dr, c + dc
                    np = (nr, nc)
                    if not (0 <= nr < size[0] and 0 <= nc < size[1]):
                        continue
                    if np in walls or np in seen:
                        continue
                    seen.add(np)
                    dists[np] = d + 1
                    q.append((np, d + 1))
            res = {}
            for t in targets:
                res[t] = dists.get(t, float('inf'))
            return res

        # Helper: Get reachable plants from a start position
        def get_reachable_plants(start_pos, plant_positions, walls):
            """Returns a set of plant positions that are reachable from start_pos."""
            size = self.game.get_problem().get("Size", (0, 0))
            from collections import deque
            q = deque([start_pos])
            visited = {start_pos}
            reachable = set()
            
            # Optimization: If start is already on a plant
            if start_pos in plant_positions:
                reachable.add(start_pos)
            
            while q:
                r, c = q.popleft()
                if (r, c) in plant_positions:
                    reachable.add((r, c))
                
                # Standard BFS neighbors
                for dr, dc in _MOVES.values():
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < size[0] and 0 <= nc < size[1]:
                        if (nr, nc) not in walls and (nr, nc) not in visited:
                            visited.add((nr, nc))
                            q.append((nr, nc))
            return reachable

        # Filter plants: Remove blocked ones (only keep reachable plants)
        # Get position of our smart robot(s)
        smart_positions = []
        for (rid, (r, c), _) in state[0]:
            if rid in chosen:
                smart_positions.append((r, c))
        
        # If we have smart robots, filter plants that are physically reachable
        valid_plants = set(current_plants)
        if smart_positions:
            # Check reachability from any smart robot
            all_reachable = set()
            for smart_pos in smart_positions:
                reachable_set = get_reachable_plants(smart_pos, set(current_plants), dynamic_walls)
                all_reachable.update(reachable_set)
            
            # Intersect current plants with reachable ones
            valid_plants = set(current_plants).intersection(all_reachable)
            
            # EDGE CASE: If NO plants are reachable (total blockage), 
            # fall back to original list so we don't crash
            if not valid_plants and current_plants:
                valid_plants = set(current_plants)
        
        # Update current_plants to only include valid (reachable) plants
        current_plants = list(valid_plants)

        # Expected-value-per-step estimator (approximate, uses robot reliability)
        def ev_per_step_for_pair(cur_state, rid, plant_pos):
            probs = self.game.get_problem().get("robot_chosen_action_prob", {})
            p = float(probs.get(rid, 0.0))
            if p <= 0.0:
                return 0.0

            # extract robot current pos and load
            robot_pos = None; robot_load = 0
            for (rrid, (rpos_r, rpos_c), l) in cur_state[0]:
                if rrid == rid:
                    robot_pos = (rpos_r, rpos_c); robot_load = l; break
            if robot_pos is None:
                return 0.0

            taps = list(self.game.get_problem().get("Taps", {}).keys())
            # if no taps or unreachable, EV is zero
            if not taps:
                return 0.0

            # distances (use dynamic_walls to account for stupid robots)
            d_robot_to_plant = bfs_dist(robot_pos, [plant_pos], dynamic_walls).get(plant_pos, float('inf'))
            d_plant_to_taps = bfs_dist(plant_pos, taps, dynamic_walls)
            d_tap_to_plant = min(d_plant_to_taps.values()) if d_plant_to_taps else float('inf')

            # nearest tap from robot (for initial load if needed)
            d_robot_to_taps = bfs_dist(robot_pos, taps, dynamic_walls)
            d_robot_to_tap = min(d_robot_to_taps.values()) if d_robot_to_taps else float('inf')

            # unreachable checks
            if math.isinf(d_robot_to_plant) and math.isinf(d_robot_to_tap):
                return 0.0

            # approximate movement cost: each intended step expected attempts ~= 1/p
            def move_cost(dist):
                if math.isinf(dist):
                    return float('inf')
                return dist / max(1e-6, p)

            # expected cost to ensure we have at least 1 WU available for the pour
            expected_load_cost = 0.0
            if robot_load <= 0:
                # go to nearest tap, load (expected attempts ~1/p)
                if math.isinf(d_robot_to_tap):
                    return 0.0
                expected_load_cost = move_cost(d_robot_to_tap) + (1.0 / max(1e-6, p))

            # travel cost to plant (from robot current pos or from tap if we loaded)
            if robot_load <= 0:
                travel_to_plant_cost = move_cost(d_tap_to_plant)
            else:
                travel_to_plant_cost = move_cost(d_robot_to_plant)

            # pour expected cost: attempt (1 step) plus expected extra reload cycles caused by spills
            # if pour fails (prob 1-p) robot loses 1 WU and must reload: cost to go to tap, load, and return
            # approximate reload cycle cost from plant: plant->tap + load attempts + tap->plant
            reload_cycle = float('inf')
            if d_tap_to_plant is not None and not math.isinf(d_tap_to_plant):
                reload_cycle = move_cost(d_tap_to_plant) + (1.0 / max(1e-6, p)) + move_cost(d_tap_to_plant)

            # expected extra cost due to spills: (1-p)/p * reload_cycle (approx)
            if math.isinf(reload_cycle):
                expected_pour_cost = 1.0
            else:
                expected_pour_cost = 1.0 + ((1.0 - p) / max(1e-6, p)) * reload_cycle

            total_expected_steps = expected_load_cost + travel_to_plant_cost + expected_pour_cost
            if total_expected_steps <= 0 or math.isinf(total_expected_steps):
                return 0.0

            # reward per unit (dynamic greedy multiplier)
            prob_dict = self.game.get_problem()
            goal_reward = float(prob_dict.get('goal_reward', 1.0))
            initial_total_need = sum(prob_dict.get('Plants', {}).values()) if prob_dict.get('Plants') else 1
            total_remaining = max(1, cur_state[3])
            per_unit_base = goal_reward / max(1, initial_total_need)
            multiplier = 1.0 + (1.0 - (float(total_remaining) / max(1.0, initial_total_need)))
            reward_per_unit = per_unit_base * multiplier

            ev = reward_per_unit / total_expected_steps
            return ev

        # Small-board brute-force fallback: if board area is small, try exhaustive BFS (deterministic)
        size = self.game.get_problem().get("Size", (0, 0))
        area = size[0] * size[1]
        # On large boards, pick a single best plant by estimated EV per step (consider chosen robots)
        if area > SMALL_BOARD_AREA and current_plants:
            best_ev = -1.0
            candidates = []
            for p in current_plants:
                best_for_p = 0.0
                for rid in chosen:
                    ev = ev_per_step_for_pair(state, rid, p)
                    if ev > best_for_p:
                        best_for_p = ev
                if best_for_p > best_ev:
                    best_ev = best_for_p
                    candidates = [p]
                elif abs(best_for_p - best_ev) < 1e-12:
                    candidates.append(p)

            if not candidates:
                # fallback to reward-based heuristic
                max_ev = max(expected_rewards.values())
                candidates = [p for p, ev in expected_rewards.items() if ev == max_ev]

            if len(candidates) > 1:
                # tie-breaker 1: closest to a tap
                taps = list(self.game.get_problem().get("Taps", {}).keys())
                cand_tap_dist = {}
                for p in candidates:
                    dmap = bfs_dist(p, taps, dynamic_walls)
                    cand_tap_dist[p] = min(dmap.values()) if taps else float('inf')
                min_tap = min(cand_tap_dist.values())
                candidates = [p for p in candidates if cand_tap_dist[p] == min_tap]
                if len(candidates) > 1:
                    # tie-breaker 2: closest to any robot
                    robots_positions = [pos for (rid, pos, l) in state[0]]
                    cand_robot_dist = {}
                    for p in candidates:
                        dmap = bfs_dist(p, robots_positions, dynamic_walls)
                        cand_robot_dist[p] = min(dmap.values()) if robots_positions else float('inf')
                    min_robot = min(cand_robot_dist.values())
                    candidates = [p for p in candidates if cand_robot_dist[p] == min_robot]

            # choose first candidate as target plant
            target_plant = candidates[0]
            # restrict planning to that plant only
            current_plants = [target_plant]
        if area <= SMALL_BOARD_AREA:
            # build deterministic WateringProblem including chosen robots
            prob_small = build_problem_from_state(state, chosen, plants_to_keep=None)

            class SimpleFIFO:
                def __init__(self):
                    self.d = deque()
                def append(self, x): self.d.append(x)
                def extend(self, items):
                    for it in items: self.append(it)
                def pop(self): return self.d.popleft()
                def __len__(self): return len(self.d)

            wf = WateringProblem(prob_small)
            res = graph_search(wf, SimpleFIFO())
            if res is not None:
                goal_node, _ = res
                acts = []
                cur = goal_node
                while cur.parent is not None:
                    acts.append(cur.action)
                    cur = cur.parent
                acts = acts[::-1]
                converted = []
                for a in acts:
                    if a is None: continue
                    if "{" in a and "}" in a:
                        act, rid = a.split("{")
                        rid = rid.strip("}")
                        converted.append(f"{act}({rid})")
                    else:
                        converted.append(a)
                self.plan_actions = converted
                self.plan_idx = 0
                self.stupid_robots = set(robot_ids_all) - set(chosen)
                # fall through to normal action return

        # iterative try: prune 0, then remove 1/3, 2/3 etc until threshold satisfied
        prune_fraction = 0
        final_actions = []
        while True:
            # determine which plants to keep
            keep = set(current_plants)
            if prune_fraction > 0:
                k = max(0, int(len(current_plants) * (1 - prune_fraction)))
                # keep top-k by expected reward
                ordered = sorted(current_plants, key=lambda p: expected_rewards.get(p, 0), reverse=True)
                keep = set(ordered[:k])

            actions = run_astar_with_pruned_plants(keep)
            if not actions:
                # failed to find plan; try pruning more
                prune_fraction += 1.0/3.0
                if prune_fraction >= 1.0:
                    # give up
                    self.plan_actions = []
                    self.plan_idx = 0
                    self.prev_state = state
                    self.last_action = None
                    return fallback_local(chosen)
                continue

            plan_len = len(actions)
            pmin = min(probs.get(rid, 0) for rid in chosen)
            thresh = compute_threshold(pmin, len(stupid))
            if plan_len < thresh:
                final_actions = actions
                break
            # else prune more plants
            prune_fraction += 1.0/3.0
            if prune_fraction >= 1.0:
                # can't satisfy, accept current plan
                final_actions = actions
                break

        # Convert actions from A* format e.g. UP{10} -> UP(10)
        converted = []
        for a in final_actions:
            if a is None: continue
            # a like 'UP{10}' or 'LOAD{10}'
            if "{" in a and "}" in a:
                act, rid = a.split("{")
                rid = rid.strip("}")
                converted.append(f"{act}({rid})")
            else:
                # fallback
                converted.append(a)

        # If we planned no POURs but plants remain, try planning with all robots
        if current_plants:
            has_pour = any((('POUR' in str(x)) or (x and x.upper().startswith('POUR'))) for x in converted)
            if not has_pour:
                all_active_rids = [rid for (rid, (r, c), load) in state[0] if float(probs.get(rid, 0.0)) > 0.0]
                if all_active_rids:
                    alt_actions = run_astar_with_chosen(all_active_rids, keep if 'keep' in locals() else None)
                else:
                    alt_actions = None
                if alt_actions:
                    # convert alt_actions similarly
                    alt_converted = []
                    for a in alt_actions:
                        if a is None: continue
                        if "{" in a and "}" in a:
                            act, rid = a.split("{")
                            rid = rid.strip("}")
                            alt_converted.append(f"{act}({rid})")
                        else:
                            alt_converted.append(a)
                    # if alternative contains POUR, adopt it
                    if any((('POUR' in str(x)) or (x and x.upper().startswith('POUR'))) for x in alt_converted):
                        converted = alt_converted

        # Save plan
        self.plan_actions = converted
        self.plan_idx = 0
        self.stupid_robots = stupid
        if getattr(self.game, '_debug', False):
            print(f"DEBUG PLAN chosen={chosen} stupid={list(self.stupid_robots)} plan={self.plan_actions}")

        # If we had a previous action, detect its success and react
        if self.prev_state is not None and self.last_action is not None:
            old = self.prev_state
            new = state
            last = self.last_action
            # parse last like 'UP(10)'
            try:
                t, rid_s = last.split("(")
                rid = int(rid_s.strip(") "))
                t = t.upper()
            except Exception:
                rid = None; t = None

            def find_robot(state_obj, rid):
                for (rrid, (rpos_r, rpos_c), load) in state_obj[0]:
                    if rrid == rid:
                        return (rpos_r, rpos_c, load)
                return None

            success = True
            if rid is not None and t is not None:
                old_robot = find_robot(old, rid)
                new_robot = find_robot(new, rid)
                if t == 'POUR':
                    # check plant need decreased at robot pos
                    old_plants = {pos: need for (pos, need) in old[1]}
                    new_plants = {pos: need for (pos, need) in new[1]}
                    rpos = (old_robot[0], old_robot[1]) if old_robot else None
                    old_need = old_plants.get(rpos)
                    new_need = new_plants.get(rpos)
                    if old_need is None:
                        success = True
                    else:
                        if new_need is None or new_need < old_need:
                            success = True
                        else:
                            success = False
                elif t == 'LOAD':
                    # load increased and tap decreased
                    old_load = old_robot[2] if old_robot else None
                    new_load = new_robot[2] if new_robot else None
                    if new_load is not None and old_load is not None and new_load > old_load:
                        success = True
                    else:
                        success = False
                else:
                    # MOVE: check whether robot moved to intended cell
                    if old_robot and new_robot:
                        dr = 0; dc = 0
                        if t == 'UP': dr = -1
                        if t == 'DOWN': dr = 1
                        if t == 'LEFT': dc = -1
                        if t == 'RIGHT': dc = 1
                        intended = (old_robot[0] + dr, old_robot[1] + dc)
                        if (new_robot[0], new_robot[1]) == intended:
                            success = True
                        else:
                            success = False

            if not success:
                # handle failure: for MOVE, try to move back; for LOAD/POUR, retry same action
                # update consecutive pour-failure counter
                if t == 'POUR':
                    self.consecutive_pour_failures = getattr(self, 'consecutive_pour_failures', 0) + 1
                else:
                    self.consecutive_pour_failures = 0

                # if too many POUR failures in a row, consider RESET to recover
                if getattr(self, 'consecutive_pour_failures', 0) >= 3:
                    self.consecutive_pour_failures = 0
                    # clear plan to force replanning and perform RESET as last resort
                    self.plan_actions = []
                    self.plan_idx = 0
                    self.prev_state = state
                    self.last_action = None
                    return "RESET"

                if t in ('UP', 'DOWN', 'LEFT', 'RIGHT') and rid is not None:
                    # try to move robot back to old position
                    if new_robot is not None and old_robot is not None:
                        back_r = old_robot[0]
                        back_c = old_robot[1]
                        cur_r, cur_c = new_robot[0], new_robot[1]
                        dr = back_r - cur_r
                        dc = back_c - cur_c
                        # map delta to action
                        if abs(dr) + abs(dc) == 1:
                            if dr == -1:
                                fix = f"UP({rid})"
                            elif dr == 1:
                                fix = f"DOWN({rid})"
                            elif dc == -1:
                                fix = f"LEFT({rid})"
                            else:
                                fix = f"RIGHT({rid})"
                            self.prev_state = state
                            self.last_action = fix
                            return fix
                else:
                    # retry same action if still applicable, otherwise recompute
                    # validate applicability for 'last' in current state
                    retry_act = last
                    try:
                        act_n, rid_s = retry_act.split("(")
                        act_n = act_n.upper()
                        rid_n = int(rid_s.strip(") "))
                    except Exception:
                        act_n = retry_act.upper(); rid_n = None

                    # find robot current info
                    robot_now = None
                    for (rrid, (rpos_r, rpos_c), l) in state[0]:
                        if rrid == rid_n:
                            robot_now = ((rpos_r, rpos_c), l)
                            break

                    applicable = True
                    if act_n == 'POUR':
                        plant_positions = {pos for (pos, need) in state[1]}
                        if robot_now is None or robot_now[0] not in plant_positions or robot_now[1] == 0:
                            applicable = False
                    if act_n == 'LOAD':
                        tap_positions = {pos for (pos, water) in state[2]}
                        cap = self.game.get_capacities().get(rid_n, 0)
                        if robot_now is None or robot_now[0] not in tap_positions or robot_now[1] >= cap:
                            applicable = False

                    if applicable:
                        self.prev_state = state
                        self.last_action = last
                        return last
                    else:
                        # cannot retry
                        self.prev_state = state
                        self.last_action = None
                        return fallback_local(chosen)

        # Normal case: return next planned action but validate applicability
        if self.plan_idx < len(self.plan_actions):
            act = self.plan_actions[self.plan_idx]

            # Validate dynamic applicability similar to ext_plant rules
            robots_t, plants_t, taps_t, total = state
            occupied = {(rr, cc) for (rrid, (rr, cc), l) in robots_t}

            # parse act like 'UP(10)'
            try:
                act_name, rid_s = act.split("(")
                act_name = act_name.upper()
                rid_target = int(rid_s.strip(") "))
            except Exception:
                rid_target = None
                act_name = act.upper()

            # locate robot
            robot_pos = None
            robot_load = None
            for (rrid, (rpos_r, rpos_c), l) in robots_t:
                if rrid == rid_target:
                    robot_pos = (rpos_r, rpos_c)
                    robot_load = l
                    break

            # If action is a movement, check if target is occupied
            if act_name in ("UP", "DOWN", "LEFT", "RIGHT") and robot_pos is not None:
                dr = 0; dc = 0
                if act_name == 'UP': dr = -1
                if act_name == 'DOWN': dr = 1
                if act_name == 'LEFT': dc = -1
                if act_name == 'RIGHT': dc = 1
                target = (robot_pos[0] + dr, robot_pos[1] + dc)
                if target in occupied:
                    # who is blocking?
                    blocker = None
                    for (rrid, (rpos_r, rpos_c), l) in robots_t:
                        if (rpos_r, rpos_c) == target:
                            blocker = rrid; break
                    # if blocker is stupid, try to move it to a free adjacent cell
                    if blocker in self.stupid_robots:
                        # find moves for blocker
                        bpos = None
                        for (rrid, (rpos_r, rpos_c), l) in robots_t:
                            if rrid == blocker:
                                bpos = (rpos_r, rpos_c); break
                        if bpos is not None:
                            # If the blocker robot has very low reliability, avoid pushing it aside
                            p_blocker = float(probs.get(blocker, 0.0))
                            if p_blocker < 0.05:
                                # don't attempt risky push-aside; skip this planned action
                                self.plan_idx += 1
                                return fallback_local(chosen)
                            base_actions = self.game.applicable_actions[bpos]
                            # occupied without the blocker itself
                            occ_without_blocker = {p for p in occupied if p != bpos}
                            for a in base_actions:
                                if a in ("UP", "DOWN", "LEFT", "RIGHT"):
                                    if a == "UP": t = (bpos[0]-1, bpos[1])
                                    elif a == "DOWN": t = (bpos[0]+1, bpos[1])
                                    elif a == "LEFT": t = (bpos[0], bpos[1]-1)
                                    else: t = (bpos[0], bpos[1]+1)
                                    if (0 <= t[0] < self.game.get_problem()["Size"][0] and 0 <= t[1] < self.game.get_problem()["Size"][1]) and t not in self.game.get_problem().get("Walls", set()) and t not in occ_without_blocker:
                                        move_cmd = f"{a}({blocker})"
                                        self.prev_state = state
                                        self.last_action = move_cmd
                                        return move_cmd
                    # otherwise, just recompute later (skip returning blocked action)
                    # fallback: try next action in plan (advance) or reset
                    self.plan_idx += 1
                    return fallback_local(chosen)

            # For POUR/LOAD ensure applicability in current dynamic state
            if act_name == 'POUR':
                # must be on a plant and have load>0
                plant_positions = {pos for (pos, need) in plants_t}
                if robot_pos is None or robot_pos not in plant_positions or robot_load == 0:
                    # If another robot is standing on a plant cell with load>0, use it to pour
                    # prefer non-zero-prob robots for opportunistic POUR
                    ordered = sorted(list(robots_t), key=lambda x: float(probs.get(x[0], 0.0)), reverse=True)
                    for (other_rid, (or_r, or_c), ol) in ordered:
                        if (or_r, or_c) in plant_positions and ol > 0 and float(probs.get(other_rid, 0.0)) > 0.0:
                            self.prev_state = state
                            self.last_action = f"POUR({other_rid})"
                            return f"POUR({other_rid})"
                    # fallback: allow zero-prob robots if nothing else
                    for (other_rid, (or_r, or_c), ol) in ordered:
                        if (or_r, or_c) in plant_positions and ol > 0:
                            self.prev_state = state
                            self.last_action = f"POUR({other_rid})"
                            return f"POUR({other_rid})"
                    # invalid now; recompute
                    self.plan_idx += 1
                    return fallback_local(chosen)

            if act_name == 'LOAD':
                tap_positions = {pos for (pos, water) in taps_t}
                cap = self.game.get_capacities().get(rid_target, 0)
                if robot_pos is None or robot_pos not in tap_positions or robot_load >= cap:
                    # If another robot is on a tap and can load, use it
                    ordered = sorted(list(robots_t), key=lambda x: float(probs.get(x[0], 0.0)), reverse=True)
                    for (other_rid, (or_r, or_c), ol) in ordered:
                        if (or_r, or_c) in tap_positions and ol < self.game.get_capacities().get(other_rid, 0) and float(probs.get(other_rid, 0.0)) > 0.0:
                            self.prev_state = state
                            self.last_action = f"LOAD({other_rid})"
                            return f"LOAD({other_rid})"
                    for (other_rid, (or_r, or_c), ol) in ordered:
                        if (or_r, or_c) in tap_positions and ol < self.game.get_capacities().get(other_rid, 0):
                            self.prev_state = state
                            self.last_action = f"LOAD({other_rid})"
                            return f"LOAD({other_rid})"
                    self.plan_idx += 1
                    return fallback_local(chosen)

            # If action is LOAD and few steps remain, avoid forcing LOAD
            if act_name == 'LOAD':
                remaining = self.game.get_max_steps() - self.game.get_current_steps()
                if remaining < LOAD_AVOID_STEPS:
                    # skip forcing a load now; recompute plan next tick
                    self.plan_idx += 1
                    return fallback_local(chosen)

            # action seems okay; return it
            self.plan_idx += 1
            self.prev_state = state
            self.last_action = act
            return act

        # No plan / exhausted
        return fallback_local(chosen)