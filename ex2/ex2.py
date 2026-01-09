import ext_plant
from collections import deque
import math
import bisect
import sys

# Update with your ID
id = ["216764803"]

# Directions mapping
_MOVES = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

# ==============================================================================
# A* SEARCH IMPLEMENTATION
# ==============================================================================

infinity = 1.0e400

def update(x, **entries):
    if isinstance(x, dict):
        x.update(entries)
    else:
        x.__dict__.update(entries)
    return x

class Queue:
    def extend(self, items):
        for item in items: self.append(item)

class PriorityQueue(Queue):
    def __init__(self, order=min, f=lambda x: x):
        update(self, A=[], order=order, f=f)

    def append(self, item):
        priority = self.f(item)
        bisect.insort(self.A, (priority, item))

    def __len__(self):
        return len(self.A)

    def pop(self):
        if self.order == min:
            return self.A.pop(0)[1]
        else:
            return self.A.pop()[1]

class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        update(self, state=state, parent=parent, action=action,
               path_cost=path_cost, depth=0)
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def expand(self, problem):
        return [Node(next_state, self, act,
                     problem.path_cost(self.path_cost, self.state, act, next_state))
                for (act, next_state) in problem.successor(self.state)]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __lt__(self, other):
        return False

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

def graph_search(problem, fringe):
    closed = {}
    fringe.append(Node(problem.initial))
    while fringe:
        node = fringe.pop()
        if problem.goal_test(node.state):
            return node
        if node.state not in closed:
            closed[node.state] = True
            fringe.extend(node.expand(problem))
    return None

def astar_search(problem, h=None):
    h = h or problem.h
    def f(n): return n.path_cost + h(n)
    return graph_search(problem, PriorityQueue(min, f))

class WateringProblem(Problem):
    def __init__(self, initial, blocked_robots=None, unreachable_plants=None):
        self.size = initial["Size"]
        self.walls = set(initial.get("Walls", set()))
        if blocked_robots:
            self.walls.update(blocked_robots)
        
        self.plants_targets = dict(initial["Plants"])
        if unreachable_plants:
            for p_pos in unreachable_plants:
                if p_pos in self.plants_targets:
                    del self.plants_targets[p_pos]
        
        self.plant_positions = tuple(sorted(self.plants_targets.keys()))
        self.tap_positions = tuple(sorted(initial["Taps"].keys()))

        robot_data_list = []
        self.initial_capacities = {}
        for rid, (r, c, load, cap) in initial["Robots"].items():
            robot_data_list.append((rid, (r, c), load))
            self.initial_capacities[rid] = cap

        robot_states = frozenset(robot_data_list)
        plant_states = tuple(0 for _ in self.plant_positions)
        tap_states = tuple(initial["Taps"].get(pos, 0) for pos in self.tap_positions)
        total_remaining = sum(self.plants_targets.values())

        Problem.__init__(self, (robot_states, plant_states, tap_states, total_remaining))

    def successor(self, state):
        robot_states, plant_states, tap_states, total_remaining = state
        successors = []

        for robot_entry in sorted(robot_states):
            rid, (r, c), load = robot_entry
            occupied = {pos for (oid, pos, l) in robot_states if oid != rid}
            
            if (r, c) in self.tap_positions:
                tidx = self.tap_positions.index((r, c))
                if tap_states[tidx] > 0 and load < self.initial_capacities[rid]:
                    new_taps = list(tap_states)
                    new_taps[tidx] -= 1
                    new_robots = (robot_states - {robot_entry}) | {(rid, (r, c), load + 1)}
                    successors.append((f"LOAD{{{rid}}}", (new_robots, plant_states, tuple(new_taps), total_remaining)))

            if (r, c) in self.plant_positions:
                pidx = self.plant_positions.index((r, c))
                if plant_states[pidx] < self.plants_targets[(r, c)] and load > 0:
                    new_plants = list(plant_states)
                    new_plants[pidx] += 1
                    new_robots = (robot_states - {robot_entry}) | {(rid, (r, c), load - 1)}
                    successors.append((f"POUR{{{rid}}}", (new_robots, tuple(new_plants), tap_states, total_remaining - 1)))

            for move, (dr, dc) in _MOVES.items():
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.size[0] and 0 <= nc < self.size[1] and (nr, nc) not in self.walls and (nr, nc) not in occupied:
                    new_robots = (robot_states - {robot_entry}) | {(rid, (nr, nc), load)}
                    successors.append((f"{move}{{{rid}}}", (new_robots, plant_states, tap_states, total_remaining)))
        
        return successors

    def goal_test(self, state):
        return state[3] == 0

    # ==============================================================================
    # HEURISTIC V3 (REVERTED): Max-Min + Deficit
    # This stable version fixes pdf2 (symmetry) and supports the "One Robot" strategy.
    # ==============================================================================
    def h(self, node):
        robot_states, plant_states, tap_states, total_remaining = node.state
        
        if total_remaining == 0:
            return 0
            
        needy_plants = []
        for i, pos in enumerate(self.plant_positions):
            if plant_states[i] < self.plants_targets[pos]:
                needy_plants.append(pos)
        
        if not needy_plants:
            return 0

        active_taps = [pos for i, pos in enumerate(self.tap_positions) if tap_states[i] > 0]
        
        def manhattan(p1, p2):
            return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

        # Work Deficit: Rewards loading actions
        total_current_load = sum(r[2] for r in robot_states)
        water_deficit = max(0, total_remaining - total_current_load)
        
        # Max-Min Distance: Focuses on the BOTTLENECK plant
        max_min_dist = 0
        
        for p_pos in needy_plants:
            min_dist_to_this_plant = float('inf')
            
            for rid, (r, c), load in robot_states:
                # Cost logic:
                # If loaded: Distance to plant
                # If empty: Distance to Tap + 1 (Load Cost) + Dist Tap to Plant
                if load > 0:
                    dist = manhattan((r, c), p_pos)
                else:
                    if not active_taps: 
                        dist = 999
                    else:
                        d_tap = min(manhattan((r, c), t) for t in active_taps)
                        d_plant = min(manhattan(t, p_pos) for t in active_taps)
                        dist = d_tap + 1 + d_plant
                
                if dist < min_dist_to_this_plant:
                    min_dist_to_this_plant = dist
            
            if min_dist_to_this_plant == float('inf'):
                min_dist_to_this_plant = 999
                
            if min_dist_to_this_plant > max_min_dist:
                max_min_dist = min_dist_to_this_plant

        return total_remaining + water_deficit + max_min_dist

class Controller:
    def __init__(self, game: ext_plant.Game):
        self.game = game
        problem_dict = game.get_problem()
        
        self.size = problem_dict["Size"]
        self.walls = set(problem_dict["Walls"])
        
        robot_probs = problem_dict["robot_chosen_action_prob"]
        self.RELIABILITY_THRESHOLD = 0.7
        self.active_robots = {rid for rid, p in robot_probs.items() if p >= self.RELIABILITY_THRESHOLD}
        
        capacities = game.get_capacities()
        
        # ==========================================================================
        # IMPROVED ROBOT SELECTION
        # We bucket probabilities by 5% (0.05). If probs are similar, we pick the 
        # robot with HIGHER CAPACITY. This prefers Robot 10 (Cap 5) over 11 (Cap 1).
        # ==========================================================================
        self.smart_robot_id = max(
            robot_probs.keys(), 
            key=lambda rid: (
                int(robot_probs[rid] / 0.05), # Priority 1: Reliability Bucket
                capacities.get(rid, 0),       # Priority 2: Capacity
                robot_probs[rid]              # Priority 3: Raw Probability tie-break
            )
        )
        
        if not self.active_robots:
            self.active_robots = {self.smart_robot_id}
        
        self.blocked_robots_positions = set()
        self.unreachable_plants = set()
        self.active_plant_positions = {pos for pos, need in problem_dict["Plants"].items() if need > 0}
        self.initial_total_water_need = sum(problem_dict["Plants"].values())
        
        self.plant_watered_this_run = False
        self.current_plan = []
        self.plan_index = 0
        self.expected_robot_pos = {}
        self.last_robot_pos = {}
        self.correction_targets = {}
        self.current_target_plant = None
        self.last_action_was_reset = True 
        self.slip_counter = {}
        self._recalculate_plan()

    def _return_reset(self):
        self.plant_watered_this_run = False
        self.current_plan = []
        self.plan_index = 0
        self.blocked_robots_positions = set()
        self.unreachable_plants = set()
        self.last_action_was_reset = True
        self.expected_robot_pos = {}
        return "RESET"

    def _build_reduced_problem(self, state, pd):
        robots_t, plants_t, taps_t, _ = state
        plant_rews = {p: sum(r)/len(r) for p, r in pd["plants_reward"].items()}
        area = self.size[0] * self.size[1]
        
        self.strategy = "WATER_ALL"
        
        rewards = list(plant_rews.values())
        max_r = max(rewards) if rewards else 0
        min_r = min(rewards) if rewards else 0
        is_farming = (max_r > 2.0 * min_r + 2)
        
        # For Maps larger than 16 (4x4), default to targeting specific plants
        if area > 16 or is_farming:
            self.strategy = "WATER_ONE"

        selected_plants = {}
        if self.strategy == "WATER_ONE":
            smart_pos = next(p for i, p, l in robots_t if i == self.smart_robot_id)
            best_p, highest_score = None, -float('inf')
            
            for p_pos, p_need in plants_t:
                if p_need > 0 and p_pos not in self.unreachable_plants:
                    reward = plant_rews.get(p_pos, 0)
                    d1 = self._bfs_distance(smart_pos, p_pos, pd)
                    dist_cost = d1 if d1 < 999 else 999
                    score = (reward * 10.0) - dist_cost
                    
                    if score > highest_score:
                        highest_score, best_p = score, p_pos
            
            if best_p:
                selected_plants[best_p] = next(n for p, n in plants_t if p == best_p)
                self.current_target_plant = best_p
        else:
            selected_plants = {p: n for p, n in plants_t if n > 0 and p not in self.unreachable_plants}

        if self.strategy == "WATER_ONE":
            # Aggressive Filtering:
            # If the smart robot is good, IGNORE others to prevent "weak" robots
            # from stealing tasks in the plan.
            target_rids = {self.smart_robot_id}
            
            # Only include others if they are VERY close in quality (Prob + Cap)
            # But generally, for WATER_ONE, single-agent A* is fastest and safest.
        else:
            target_rids = self.active_robots
            
        red_robots = {rid: (pos[0], pos[1], load, self.game.get_capacities()[rid]) 
                      for rid, pos, load in robots_t if rid in target_rids}

        return {
            "Size": self.size, "Walls": self.walls, "Taps": {p: w for p, w in taps_t},
            "Plants": selected_plants, "Robots": red_robots,
            "robot_chosen_action_prob": {rid: pd["robot_chosen_action_prob"][rid] for rid in red_robots}
        }

    def choose_next_action(self, state):
        robots_t, plants_t, taps_t, total_water_need = state
        current_active_plants = {pos for pos, need in plants_t if need > 0}
        
        if self.last_action_was_reset:
            self.active_plant_positions, self.initial_total_water_need = current_active_plants, total_water_need
            self.last_action_was_reset = False
            self.expected_robot_pos = {}
        
        slipped = False
        for rid in self.active_robots:
            actual_pos = self._get_robot_position(rid, robots_t)
            if rid in self.expected_robot_pos and actual_pos is not None:
                expected_pos = self.expected_robot_pos[rid]
                if actual_pos != expected_pos:
                    slipped = True
                    break
        
        if slipped:
            self._recalculate_plan()
            self.expected_robot_pos = {}
        
        if self.strategy == "WATER_ONE":
            if len(current_active_plants) < len(self.active_plant_positions):
                return self._return_reset()
            
            rid = self.smart_robot_id
            load = self._get_robot_load(rid, robots_t)
            if load == 0:
                pos = self._get_robot_position(rid, robots_t)
                start_pos = self.game.get_problem()["Robots"][rid][0:2]
                
                dist_current = self._get_dist_to_nearest_tap(pos, taps_t)
                dist_start = self._get_dist_to_nearest_tap(start_pos, taps_t)
                
                if dist_start < dist_current:
                     return self._return_reset()

        while self.plan_index < len(self.current_plan):
            act_str = self.current_plan[self.plan_index]
            name, rid = act_str.split('{')[0], int(act_str.split('{')[1].strip('}'))
            pos, load = self._get_robot_position(rid, robots_t), self._get_robot_load(rid, robots_t)
            
            if pos is None: 
                self.plan_index += 1; continue

            if name in _MOVES:
                is_at_tap = any(p == pos for p, w in taps_t)
                if is_at_tap:
                    cap = self.game.get_capacities()[rid]
                    needed = cap
                    if self.current_target_plant:
                         p_need = next((n for p, n in plants_t if p == self.current_target_plant), cap)
                         needed = min(cap, p_need)
                    
                    if load < needed:
                         if self._is_action_legal("LOAD", pos, load, state, rid):
                             self.expected_robot_pos[rid] = pos
                             return f"LOAD({rid})"

                target = self._get_target_position(pos, name)
                blocker = self._get_robot_at_position(target, robots_t, exclude_id=rid)
                
                if blocker is not None:
                    self.blocked_robots_positions.add(target)
                    self._recalculate_plan()
                    return self._greedy_action(state)

            if not self._is_action_legal(name, pos, load, state, rid):
                self.plan_index += 1; continue

            if name in _MOVES:
                self.expected_robot_pos[rid] = self._get_target_position(pos, name)
            else:
                self.expected_robot_pos[rid] = pos

            self.plan_index += 1
            return f"{name}({rid})"

        self._recalculate_plan()
        if not self.current_plan:
            return self._greedy_action(state)
            
        f_act = self.current_plan[0]
        f_name, f_rid = f_act.split('{')[0], int(f_act.split('{')[1].strip('}'))
        f_pos, f_load = self._get_robot_position(f_rid, robots_t), self._get_robot_load(f_rid, robots_t)
        
        if self._is_action_legal(f_name, f_pos, f_load, state, f_rid):
            if f_name in _MOVES:
                self.expected_robot_pos[f_rid] = self._get_target_position(f_pos, f_name)
            else:
                self.expected_robot_pos[f_rid] = f_pos
            
            self.plan_index = 1
            return f"{f_name}({f_rid})"
        
        return self._greedy_action(state)

    def _get_dist_to_nearest_tap(self, pos, taps_t):
        best = float('inf')
        for t_pos, _ in taps_t:
            d = self._bfs_distance(pos, t_pos, self.game.get_problem())
            if d < best: best = d
        return best

    def _is_action_legal(self, action_name, robot_pos, robot_load, state, robot_id):
        robots_t, plants_t, taps_t, _ = state
        capacities = self.game.get_capacities()
        if action_name in _MOVES:
            target = self._get_target_position(robot_pos, action_name)
            if not (0 <= target[0] < self.size[0] and 0 <= target[1] < self.size[1]): return False
            if target in self.walls or target in self.blocked_robots_positions: return False
            if self._get_robot_at_position(target, robots_t, exclude_id=robot_id): return False
            return True
        elif action_name == "LOAD":
            tap_water = next((w for p, w in taps_t if p == robot_pos), 0)
            if tap_water <= 0 or robot_load >= capacities[robot_id]: return False
            if self.current_target_plant:
                need = next((n for p, n in plants_t if p == self.current_target_plant), 0)
                prob = 1.0 - self.game.get_problem()["robot_chosen_action_prob"][robot_id]
                target_total = min(math.ceil(need * (1 + prob)) + 1, capacities[robot_id])
                if robot_load >= target_total: return False
            return True
        elif action_name == "POUR":
            return robot_load > 0 and any(p == robot_pos and n > 0 for p, n in plants_t)
        return False

    def _handle_stupid_robot_blocking(self, state, smart_pos, target_pos, blocker_id, acting_id):
        blocker_pos = self._get_robot_position(blocker_id, state[0])
        pois = set(self.game.get_problem()["Taps"].keys()) | set(self.game.get_problem()["Plants"].keys())
        for move in _MOVES:
            if self._is_action_legal(move, blocker_pos, 0, state, blocker_id):
                escape = self._get_target_position(blocker_pos, move)
                if escape != smart_pos and escape != target_pos and escape not in pois:
                    return f"{move}({blocker_id})"
        return self._return_reset()

    def _greedy_action(self, state):
        rid = self.smart_robot_id
        pos, load = self._get_robot_position(rid, state[0]), self._get_robot_load(rid, state[0])
        for move in _MOVES:
            if self._is_action_legal(move, pos, load, state, rid):
                self.expected_robot_pos[rid] = self._get_target_position(pos, move)
                return f"{move}({rid})"
        return self._return_reset()

    def _bfs_distance(self, start, end, problem_dict):
        if start == end: return 0
        queue, visited = deque([(start, 0)]), {start}
        while queue:
            (r, c), dist = queue.popleft()
            for dr, dc in _MOVES.values():
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.size[0] and 0 <= nc < self.size[1] and (nr, nc) not in self.walls and (nr, nc) not in visited:
                    if (nr, nc) == end: return dist + 1
                    visited.add((nr, nc))
                    queue.append(((nr, nc), dist + 1))
        return float('inf')

    def _get_robot_position(self, rid, robots_t): return next((p for i, p, l in robots_t if i == rid), None)
    def _get_robot_load(self, rid, robots_t): return next((l for i, p, l in robots_t if i == rid), 0)
    def _get_target_position(self, pos, move): return (pos[0] + _MOVES[move][0], pos[1] + _MOVES[move][1])
    def _get_robot_at_position(self, pos, robots_t, exclude_id=None): return next((i for i, p, l in robots_t if p == pos and i != exclude_id), None)
    
    def _recalculate_plan(self):
        state = self.game.get_current_state()
        reduced = self._build_reduced_problem(state, self.game.get_problem())
        problem = WateringProblem(reduced, self.blocked_robots_positions, self.unreachable_plants)
        node = astar_search(problem)
        if node:
            actions = []
            while node.parent: actions.append(node.action); node = node.parent
            self.current_plan = actions[::-1]
        else: self.current_plan = []
        self.plan_index = 0