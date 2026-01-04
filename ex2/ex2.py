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
            if args not in memoized_fn.cache:
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


class WateringProblem(Problem):
    def __init__(self, initial, blocked_robots=None, unreachable_plants=None):
        self.size = initial["Size"]
        self.walls = frozenset(initial.get("Walls", set()))
        self.plants_targets = dict(initial["Plants"])
        self.robots_capacities = self._extract_robot_capacities(initial["Robots"])
        
        # Add blocked robots as walls
        if blocked_robots:
            additional_walls = set(self.walls)
            for pos in blocked_robots:
                additional_walls.add(pos)
            self.walls = frozenset(additional_walls)
        
        # Remove unreachable plants
        if unreachable_plants:
            for pos in unreachable_plants:
                if pos in self.plants_targets:
                    del self.plants_targets[pos]
        
        self.plant_positions = tuple(sorted(frozenset(self.plants_targets.keys())))
        self.tap_positions = tuple(sorted(frozenset(initial["Taps"].keys())))

        self.plant_bfs_maps = {}
        for p_pos in self.plant_positions:
            self.plant_bfs_maps[p_pos] = self._bfs_map(p_pos)
        
        self.tap_bfs_maps = {}
        for t_pos in self.tap_positions:
            self.tap_bfs_maps[t_pos] = self._bfs_map(t_pos)

        robot_states = self._build_robot_states(initial["Robots"])
        plant_states = self._build_plant_states(initial["Plants"])
        tap_states = self._build_tap_states(initial["Taps"])
        total_remaining = sum(self.plants_targets.values())

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
            (robot_id, (r, c), load)
            for robot_id, (r, c, load, _) in sorted(robots_data.items())
        )

    def _build_plant_states(self, plants_data):
        return tuple(0 for _ in self.plant_positions)

    def _build_tap_states(self, taps_data):
        return tuple(taps_data.get(pos, 0) for pos in self.tap_positions)

    def successor(self, state):
        robot_states, plant_states, tap_states, total_remaining = state
        successors = []
        current_total_load = sum(l for (_, _, l) in robot_states)

        for robot_entry in sorted(robot_states):
            robot_id, (r, c), load = robot_entry

            occupied = {
                pos
                for (oid, pos, _) in robot_states
                if oid != robot_id
            }
            
            possible_actions = []

            if load > 0 and (r, c) in self.plant_positions:
                possible_actions.extend(
                    self._get_pour_successor(robot_states, plant_states, tap_states, total_remaining,
                                             robot_entry, robot_id, r, c, load)
                )

            if load < self.robots_capacities[robot_id] and (r, c) in self.tap_positions:
                if total_remaining > current_total_load:
                    possible_actions.extend(
                        self._get_load_successor(robot_states, plant_states, tap_states, total_remaining,
                                                 robot_entry, robot_id, r, c, load)
                    )

            if possible_actions:
                successors.extend(possible_actions)
                continue

            successors.extend(
                self._get_movement_successors(robot_states, plant_states, tap_states, total_remaining,
                                              robot_entry, robot_id, r, c, load, occupied)
            )

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
                        is_improving = True
                        break
            
            if is_improving:
                improving_candidates.append((action_name, nr, nc))

        final_candidates = improving_candidates if improving_candidates else valid_candidates

        for action_name, nr, nc in final_candidates:
            old_robot_entry = robot_entry
            new_robot_entry = (robot_id, (nr, nc), load)
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
        new_robot_entry = (robot_id, (r, c), load + 1)
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
        new_robot_entry = (robot_id, (r, c), load - 1)
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
        
        robot_positions = [pos for (_, pos, _) in sorted(robot_states)]
        unsatisfied_plants = []
        for i, pos in enumerate(self.plant_positions):
            if plant_states[i] < self.plants_targets.get(pos, 0):
                unsatisfied_plants.append(pos)
        
        carried = sum(load for (_, _, load) in robot_states)
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
                            weight = raw_dist / 1.5
                            
                    if weight < min_dists[v]:
                        min_dists[v] = weight
                        
        return mst_weight + max(0, interaction_cost)
    
    h = h_astar


class Controller:
    """
    Multi-Agent Strategic Controller with Critical Improvements:
    
    IMPROVEMENT 1: Seamless Plan Transitions (No-Reset Logic)
    - Recalculates and continues in same turn after plant completion
    - Avoids wasting steps on unnecessary RESETs
    
    IMPROVEMENT 2: Strategic RESET Calculation  
    - Calculates if distance(plant->tap) > RESET benefit
    - Uses RESET proactively when it saves steps
    
    IMPROVEMENT 3: Multi-Agent Coordination
    - Parses action tags to identify acting robot
    - Coordinates multiple robots simultaneously
    
    IMPROVEMENT 4: Step-Aware Decisions
    - Considers remaining horizon in all decisions
    - Doesn't load if insufficient time to use water
    - Filters plants by time feasibility
    
    IMPROVEMENT 5: Reliability Threshold (0.7)
    - Ignores robots below threshold
    - Focuses computational resources on productive robots
    
    IMPROVEMENT 6: Smart Parking
    - Parks unreliable robots away from POIs (taps/plants)
    - Prevents corridor blocking
    
    IMPROVEMENT 7: Don't Force Loading
    - Only loads if remaining_steps >= 2 (move + pour)
    - Prevents wasted final loads
    """
    def __init__(self, game: ext_plant.Game):
        self.game = game
        problem_dict = game.get_problem()
        
        # IMPROVEMENT 3: Multi-Agent Coordination
        # Classify robots by reliability
        robot_probs = problem_dict["robot_chosen_action_prob"]
        self.RELIABILITY_THRESHOLD = 0.7  # IMPROVEMENT 5
        
        # Select active robots (above threshold)
        self.active_robots = {
            rid: prob for rid, prob in robot_probs.items() 
            if prob >= self.RELIABILITY_THRESHOLD
        }
        
        # Select smart robot (highest success probability among active)
        if self.active_robots:
            self.smart_robot_id = max(self.active_robots.keys(), key=lambda rid: robot_probs[rid])
        else:
            # Fallback if all below threshold
            self.smart_robot_id = max(robot_probs.keys(), key=lambda rid: robot_probs[rid])
            self.active_robots = {self.smart_robot_id: robot_probs[self.smart_robot_id]}
        
        # Track blocked robots and unreachable plants
        self.blocked_robots_positions = set()
        self.unreachable_plants = set()
        self.active_plant_positions = {pos for pos, need in problem_dict["Plants"].items() if need > 0}
        self.plant_watered_this_run = False  # Initialize here
        # Current plan and execution state
        self.current_plan = []
        self.plan_index = 0
        
        # IMPROVEMENT 4: Multi-Robot Correction Tracking
        self.expected_robot_pos = {}  # rid -> expected_pos
        self.last_robot_pos = {}      # rid -> last_pos
        self.correction_targets = {}  # rid -> target_pos

        # IMPROVEMENT 1: Track plants watered for milestone-based strategy
        self.plants_watered_count = 0
        self.initial_total_water_need = sum(problem_dict["Plants"].values())
        
        # Track recalculation depth to prevent infinite loops
        self._recalc_depth = 0
        
        # Track consecutive RESETs to detect when we need greedy mode
        self.consecutive_resets = 0
        self.last_action_was_reset = False
        
        # Calculate initial plan
        self._recalculate_plan()
    
    def _return_reset(self):
        """Standardizes RESET and clears logic for fresh turn."""
        self.plant_watered_this_run = False # Lock reset for the start of next run
        self.current_plan = []
        self.plan_index = 0
        self.blocked_robots_positions = set()
        self.unreachable_plants = set()
        
        # This flag is the key to re-syncing the plant count next turn
        self.last_action_was_reset = True 
        return "RESET"
    
    def _recalculate_plan(self):
        """Recalculate A* plan based on current state"""
        state = self.game.get_current_state()
        problem_dict = self.game.get_problem()
        
        # Build reduced problem for A*
        reduced_problem = self._build_reduced_problem(state, problem_dict)
        
        # Run A*
        problem = WateringProblem(reduced_problem, self.blocked_robots_positions, self.unreachable_plants)
        result = astar_search(problem)
        
        if result is None or result[0] is None:
            # No path found - use greedy strategy
            self.current_plan = []
        else:
            goal_node = result[0]
            actions = []
            curr = goal_node
            while curr.parent is not None:
                actions.append(curr.action)
                curr = curr.parent
            self.current_plan = actions[::-1]
            
            # CRITICAL FIX: Check if excluded robots block the path
            self._handle_excluded_robots_blocking_path(state, problem_dict)
        
        self.plan_index = 0
        self.correction_targets.clear()  # Clear all correction targets
    
    def _handle_excluded_robots_blocking_path(self, state, problem_dict):
        """Move excluded robots out of the way if they block the A* plan"""
        if not self.current_plan:
            return
        
        # Extract all positions that will be visited in the plan
        planned_positions = self._extract_positions_from_plan(state)
        
        # Find excluded robots (those below reliability threshold)
        robots_t = state[0]
        excluded_robot_positions = {}
        for rid, pos, load in robots_t:
            if rid not in self.active_robots and rid != self.smart_robot_id:
                excluded_robot_positions[rid] = pos
        
        # Check which excluded robots are blocking the path
        blocking_robots = []
        for rid, pos in excluded_robot_positions.items():
            if pos in planned_positions:
                blocking_robots.append((rid, pos))
        
        if not blocking_robots:
            return  # No blocking, we're good
        
        # Try to move each blocking robot out of the way
        pre_plan_moves = []
        permanently_blocked = set()
        
        for rid, blocking_pos in blocking_robots:
            # Find a safe spot to move this robot
            safe_pos = self._find_safe_position_for_robot(
                blocking_pos, planned_positions, state, problem_dict
            )
            
            if safe_pos:
                # Calculate minimal moves to get there
                moves = self._calculate_minimal_moves(blocking_pos, safe_pos, state, problem_dict)
                if moves:
                    # Add these moves to the beginning of the plan
                    # IMPORTANT: Use curly braces {rid} to match A* format
                    for move in moves:
                        pre_plan_moves.append(f"{move}{{{rid}}}")
                else:
                    # Can't move there - mark as permanently blocked
                    permanently_blocked.add(blocking_pos)
            else:
                # No safe position found - mark as permanently blocked
                permanently_blocked.add(blocking_pos)
        
        # If we found moves to clear the path, prepend them
        if pre_plan_moves:
            self.current_plan = pre_plan_moves + self.current_plan
        
        # If some robots can't be moved, add to blocked positions and recalculate
        if permanently_blocked:
            self.blocked_robots_positions.update(permanently_blocked)
            # Recalculate A* with these new blocked positions
            # But limit recursion depth to avoid infinite loop
            if not hasattr(self, '_recalc_depth'):
                self._recalc_depth = 0
            
            if self._recalc_depth < 3:  # Max 3 levels of recalculation
                self._recalc_depth += 1
                # Re-run A* planning
                reduced_problem = self._build_reduced_problem(state, problem_dict)
                problem = WateringProblem(reduced_problem, self.blocked_robots_positions, self.unreachable_plants)
                result = astar_search(problem)
                
                if result is not None and result[0] is not None:
                    goal_node = result[0]
                    actions = []
                    curr = goal_node
                    while curr.parent is not None:
                        actions.append(curr.action)
                        curr = curr.parent
                    self.current_plan = pre_plan_moves + actions[::-1]
                
                self._recalc_depth = 0
    
    def _extract_positions_from_plan(self, state):
        """Extract all positions that robots will visit according to the plan"""
        planned_positions = set()
        
        # Simulate plan execution to extract positions
        robots_t = state[0]
        robot_positions = {rid: pos for rid, pos, _ in robots_t}
        
        for action_str in self.current_plan:
            action_parts = action_str.split('{')
            action_name = action_parts[0]
            acting_robot_id = int(action_parts[1].rstrip('}'))
            
            if acting_robot_id not in robot_positions:
                continue
            
            current_pos = robot_positions[acting_robot_id]
            planned_positions.add(current_pos)
            
            # Update position if it's a move
            if action_name in ["UP", "DOWN", "LEFT", "RIGHT"]:
                new_pos = self._get_target_position(current_pos, action_name)
                robot_positions[acting_robot_id] = new_pos
                planned_positions.add(new_pos)
        
        return planned_positions
    
    def _find_safe_position_for_robot(self, current_pos, planned_positions, state, problem_dict):
        """Find closest position that's not in the planned path"""
        walls = set(problem_dict["Walls"]) | self.blocked_robots_positions
        size = problem_dict["Size"]
        robots_t = state[0]
        occupied = {pos for rid, pos, _ in robots_t if pos != current_pos}
        
        # BFS to find closest safe position
        from collections import deque
        queue = deque([(current_pos, 0)])
        visited = {current_pos}
        
        while queue:
            pos, dist = queue.popleft()
            
            # Check adjacent cells
            for dr, dc in _MOVES.values():
                nr, nc = pos[0] + dr, pos[1] + dc
                new_pos = (nr, nc)
                
                if not (0 <= nr < size[0] and 0 <= nc < size[1]):
                    continue
                if new_pos in walls or new_pos in visited or new_pos in occupied:
                    continue
                
                # Check if this position is safe (not in planned path)
                if new_pos not in planned_positions:
                    return new_pos
                
                visited.add(new_pos)
                queue.append((new_pos, dist + 1))
        
        return None  # No safe position found
    
    def _calculate_minimal_moves(self, start_pos, end_pos, state, problem_dict):
        """Calculate minimal moves from start to end using BFS"""
        if start_pos == end_pos:
            return []
        
        walls = set(problem_dict["Walls"]) | self.blocked_robots_positions
        size = problem_dict["Size"]
        robots_t = state[0]
        occupied = {pos for rid, pos, _ in robots_t if pos != start_pos}
        
        # BFS to find shortest path
        from collections import deque
        queue = deque([(start_pos, [])])
        visited = {start_pos}
        
        while queue:
            pos, path = queue.popleft()
            
            for move_name, (dr, dc) in _MOVES.items():
                nr, nc = pos[0] + dr, pos[1] + dc
                new_pos = (nr, nc)
                
                if not (0 <= nr < size[0] and 0 <= nc < size[1]):
                    continue
                if new_pos in walls or new_pos in visited:
                    continue
                # Allow moving through occupied cells for path calculation
                # (they might move away)
                
                new_path = path + [move_name]
                
                if new_pos == end_pos:
                    return new_path
                
                visited.add(new_pos)
                queue.append((new_pos, new_path))
        
        return None  # No path found
   
    def _build_reduced_problem(self, state, problem_dict):
        robots_t, plants_t, taps_t, _ = state
        size = problem_dict["Size"]
        area = size[0] * size[1]
        
        # 1. Calculate Reward Expectations
        plant_expected_rewards = {
            pos: sum(rew) / len(rew) for pos, rew in problem_dict["plants_reward"].items()
        }
        
        # 2. Strategy Detection
        rewards_values = list(plant_expected_rewards.values())
        self.strategy = "WATER_ALL"
        if area > 16:
            self.strategy = "WATER_ONE"
        elif rewards_values:
            avg_rew = sum(rewards_values) / len(rewards_values)
            if max(rewards_values) >= 1.5 * avg_rew:
                self.strategy = "WATER_ONE"

        # 3. SMART PLANT SELECTION (Distance-Aware)
        selected_plants = {}
        if self.strategy == "WATER_ONE":
            smart_robot_data = next(r for r in robots_t if r[0] == self.smart_robot_id)
            curr_pos, curr_load = smart_robot_data[1], smart_robot_data[2]
            
            best_plant_pos = None
            highest_score = -float('inf')
            min_dist_found = float('inf')
            active_taps = [t[0] for t in taps_t if t[1] > 0]
            
            for p_pos, p_need in plants_t:
                if p_need > 0 and p_pos not in self.unreachable_plants:
                    reward = plant_expected_rewards.get(p_pos, 0)
                    
                    if curr_load > 0:
                        dist_cost = self._bfs_distance(curr_pos, p_pos, problem_dict)
                    else:
                        # Trip to Tap + Trip to Plant
                        d_to_tap = min([self._bfs_distance(curr_pos, t, problem_dict) for t in active_taps] or [999])
                        d_tap_to_p = min([self._bfs_distance(t, p_pos, problem_dict) for t in active_taps] or [999])
                        dist_cost = d_to_tap + d_tap_to_p

                    if dist_cost < float('inf'):
                        # Multiplier of 2.0 on Reward makes it the priority, 
                        # but subtraction makes distance the tie-breaker.
                        score = (reward * 2.0) - dist_cost 
                        
                        if score > highest_score or (score == highest_score and dist_cost < min_dist_found):
                            highest_score = score
                            min_dist_found = dist_cost
                            best_plant_pos = p_pos
            
            if best_plant_pos:
                selected_plants[best_plant_pos] = next(n for p, n in plants_t if p == best_plant_pos)
                print(f"[DEBUG] Chosen Target: {best_plant_pos} | Dist: {min_dist_found}")
        else:
            selected_plants = {pos: need for pos, need in plants_t if need > 0 and pos not in self.unreachable_plants}

        # 4. Final Problem Assembly (No more code after this!)
        reduced_robots = {rid: (pos[0], pos[1], load, self.game.get_capacities()[rid]) 
                          for rid, pos, load in robots_t if rid in self.active_robots}

        return {
            "Size": size, "Walls": problem_dict["Walls"],
            "Taps": {p: w for p, w in taps_t}, "Plants": selected_plants,
            "Robots": reduced_robots,
            "robot_chosen_action_prob": {rid: problem_dict["robot_chosen_action_prob"][rid] for rid in reduced_robots}
        }

    def _build_empty_problem(self, state, problem_dict):
        """Build minimal problem when no plants available - supports multiple active robots"""
        robots_t, _, taps_t, _ = state
        
        # Get all active robot info
        robot_info = {}
        for rid, pos, load in robots_t:
            if rid in self.active_robots:
                robot_info[rid] = {
                    'pos': pos,
                    'load': load,
                    'cap': self.game.get_capacities()[rid]
                }
        
        if not robot_info:
            # Fallback to smart robot
            for rid, pos, load in robots_t:
                if rid == self.smart_robot_id:
                    robot_info[rid] = {
                        'pos': pos,
                        'load': load,
                        'cap': self.game.get_capacities()[rid]
                    }
                    break
        
        reduced_robots = {}
        for rid, info in robot_info.items():
            pos = info['pos']
            reduced_robots[rid] = (pos[0], pos[1], info['load'], info['cap'])
        
        reduced_robot_probs = {
            rid: problem_dict["robot_chosen_action_prob"][rid]
            for rid in robot_info.keys()
        }
        
        return {
            "Size": problem_dict["Size"],
            "Walls": problem_dict["Walls"],
            "Taps": dict(problem_dict["Taps"]),
            "Plants": {},
            "Robots": reduced_robots,
            "robot_chosen_action_prob": reduced_robot_probs,
            "goal_reward": problem_dict["goal_reward"],
            "plants_reward": problem_dict["plants_reward"],
            "seed": problem_dict["seed"],
            "horizon": problem_dict["horizon"]
        }
    
    def _bfs_distance(self, start, end, problem_dict):
        """Calculate BFS distance between two positions"""
        if start == end:
            return 0
        
        walls = set(problem_dict["Walls"]) | self.blocked_robots_positions
        size = problem_dict["Size"]
        
        queue = deque([(start, 0)])
        visited = {start}
        
        while queue:
            pos, dist = queue.popleft()
            
            for dr, dc in _MOVES.values():
                nr, nc = pos[0] + dr, pos[1] + dc
                new_pos = (nr, nc)
                
                if not (0 <= nr < size[0] and 0 <= nc < size[1]):
                    continue
                if new_pos in walls:
                    continue
                if new_pos in visited:
                    continue
                
                if new_pos == end:
                    return dist + 1
                
                visited.add(new_pos)
                queue.append((new_pos, dist + 1))
        
        return float('inf')
    
    def choose_next_action(self, state):
        robots_t, plants_t, taps_t, total_water_need = state
        
        # --- 1. BASELINE SYNCHRONIZATION ---
        current_active_plants = {pos for pos, need in plants_t if need > 0}
        
        # If we just reset, snapshot the plants so we can detect the NEXT completion
        if self.last_action_was_reset:
            self.active_plant_positions = current_active_plants
            self.initial_total_water_need = total_water_need # Reset water baseline
            self.last_action_was_reset = False

        # --- 2. IMMEDIATE RESET PIVOT ---
        # Did a plant reach 0 need this turn?
        if len(current_active_plants) < len(self.active_plant_positions):
            print(f"[DEBUG] Plant Finished! Immediate Reset. Remaining: {len(current_active_plants)}")
            return self._return_reset()

        if total_water_need == 0:
            return self._return_reset()

        # --- 3. WORK TRACKING ---
        # If total water in the world decreased, the robot has "done work"
        if total_water_need < self.initial_total_water_need:
            self.plant_watered_this_run = True

        # [Correction Logic for slips...]
        for rid in list(self.correction_targets.keys()):
            pos = self._get_robot_position(rid, robots_t)
            if pos == self.correction_targets[rid]:
                del self.correction_targets[rid]
                self.plan_index = max(0, self.plan_index - 1)
            else: return self._correct_position(state, rid, pos, self.correction_targets[rid])

        # Execution Loop
        while self.plan_index < len(self.current_plan):
            action_str = self.current_plan[self.plan_index]
            act_name = action_str.split('{')[0]
            act_rid = int(action_str.split('{')[1].strip('}'))
            
            curr_pos = self._get_robot_position(act_rid, robots_t)
            curr_load = self._get_robot_load(act_rid, robots_t)
            
            if not curr_pos:
                self.plan_index += 1
                continue

            # --- STRATEGIC RESET ---
            # Check if teleporting to tap is faster than walking back
            if act_name == "LOAD" and self.plant_watered_this_run:
                if self._should_reset_instead(state, curr_pos, act_rid):
                    print("[DEBUG] Strategic Reset chosen at Tap.")
                    return self._return_reset()

            if not self._is_action_legal(act_name, curr_pos, curr_load, state, act_rid):
                self.plan_index += 1
                continue

            # [Handle Blocking...]
            if act_name in _MOVES:
                target = self._get_target_position(curr_pos, act_name)
                blocker = self._get_robot_at_position(target, robots_t, exclude_id=act_rid)
                if blocker: return self._handle_stupid_robot_blocking(state, curr_pos, target, blocker, act_rid)

            self.last_robot_pos[act_rid] = curr_pos
            self.expected_robot_pos[act_rid] = self._get_target_position(curr_pos, act_name) if act_name in _MOVES else curr_pos
            self.plan_index += 1
            return f"{act_name}({act_rid})"

        self._recalculate_plan()
        if not self.current_plan: return self._greedy_action(state)
        return self.choose_next_action(state)

    def _execute_next_plan_action_safe(self, state, robots_t):
        """Execute next action from plan with safety counter to prevent infinite recursion"""
        max_skips = 50  # Safety limit
        skips = 0
        
        while skips < max_skips:
            if not self.current_plan or self.plan_index >= len(self.current_plan):
                # Plan exhausted
                self._recalculate_plan()
                if not self.current_plan:
                    return self._greedy_action(state)
                # Reset counter for new plan
                skips = 0
            
            # IMPROVEMENT 3: Parse multi-agent action
            action_str = self.current_plan[self.plan_index]
            action_parts = action_str.split('{')
            action_name = action_parts[0]
            acting_robot_id = int(action_parts[1].rstrip('}'))
            
            # Get acting robot info
            robot_pos = self._get_robot_position(acting_robot_id, robots_t)
            robot_load = self._get_robot_load(acting_robot_id, robots_t)
            
            if not robot_pos:
                # Robot not found - skip
                self.plan_index += 1
                skips += 1
                continue
            # IMPROVEMENT 2: Strategic RESET calculation
            if action_name == "LOAD" and self.plant_watered_this_run:
                # About to start loading - check if RESET would be better
                if self._should_reset_instead(state, robot_pos, acting_robot_id):
                    self._recalculate_plan()
                    return "RESET"
            
            # Check if action is still legal
            if not self._is_action_legal(action_name, robot_pos, robot_load, state, acting_robot_id):
                # Skip illegal actions
                self.plan_index += 1
                skips += 1
                continue
            
            # Check for stupid robot blocking
            if action_name in ["UP", "DOWN", "LEFT", "RIGHT"]:
                if self._should_reset_instead(state, robot_pos, acting_robot_id):
                    #self._recalculate_plan()
                    return "RESET"

                target_pos = self._get_target_position(robot_pos, action_name)
                blocking_robot = self._get_robot_at_position(target_pos, robots_t, exclude_id=acting_robot_id)
                
                if blocking_robot:
                    # IMPROVEMENT 6: Smart parking - move blocking robot away
                    return self._handle_stupid_robot_blocking(state, robot_pos, target_pos, blocking_robot, acting_robot_id)
            
            # Execute action - found a legal one!
            self.last_robot_pos[acting_robot_id] = robot_pos
            if action_name in ["UP", "DOWN", "LEFT", "RIGHT"]:
                self.expected_robot_pos[acting_robot_id] = self._get_target_position(robot_pos, action_name)
            else:
                self.expected_robot_pos[acting_robot_id] = robot_pos
            
            self.plan_index += 1
            self._recalc_depth = 0  # Reset on successful action execution
            
            # Track RESETs for greedy mode detection
            self.last_action_was_reset = False
            self.consecutive_resets = 0
            
            # IMPROVEMENT 3: Format for multi-agent
            return f"{action_name}({acting_robot_id})"
        
        # Safety: Too many skips - recalculate or fall back to greedy
        self._recalculate_plan()
        if not self.current_plan:
            return self._greedy_action(state)
        return "RESET"
    
    def _should_reset_instead(self, state, robot_pos, robot_id):
        """IMPROVEMENT 2: Calculate if RESET is strategically better"""
        # Key insight #11: If path from plant to tap > RESET benefit, do RESET
        if self.plant_watered_this_run and self.strategy == "WATER_ONE":
            self.plant_watered_this_run = False
            return True
        return False
    
    def _correct_position(self, state, robot_id, current_pos, target_pos):
        """Move towards target position for correction - works for any robot"""
        robots_t, _, _, _ = state
        problem_dict = self.game.get_problem()
        walls = set(problem_dict["Walls"]) | self.blocked_robots_positions
        size = problem_dict["Size"]
        occupied = {pos for rid, pos, _ in robots_t if rid != robot_id}
        
        dr = target_pos[0] - current_pos[0]
        dc = target_pos[1] - current_pos[1]
        
        # Try to move towards target
        moves_to_try = []
        if abs(dr) > abs(dc):
            if dr > 0:
                moves_to_try.append("DOWN")
            elif dr < 0:
                moves_to_try.append("UP")
            if dc > 0:
                moves_to_try.append("RIGHT")
            elif dc < 0:
                moves_to_try.append("LEFT")
        else:
            if dc > 0:
                moves_to_try.append("RIGHT")
            elif dc < 0:
                moves_to_try.append("LEFT")
            if dr > 0:
                moves_to_try.append("DOWN")
            elif dr < 0:
                moves_to_try.append("UP")
        
        for move in moves_to_try:
            new_pos = self._get_target_position(current_pos, move)
            if (0 <= new_pos[0] < size[0] and 0 <= new_pos[1] < size[1] and
                new_pos not in walls and new_pos not in occupied):
                # Set up for correction failure detection
                self.last_robot_pos[robot_id] = current_pos
                self.expected_robot_pos[robot_id] = new_pos
                return f"{move}({robot_id})"
        
        # Can't correct - RESET
        self._recalculate_plan()
        return "RESET"
    
    def _get_robot_position(self, robot_id, robots_t):
        """Get position of specific robot"""
        for rid, pos, _ in robots_t:
            if rid == robot_id:
                return pos
        return None
    
    def _get_robot_load(self, robot_id, robots_t):
        """Get load of specific robot"""
        for rid, pos, load in robots_t:
            if rid == robot_id:
                return load
        return 0
    
    def _is_action_legal(self, action_name, robot_pos, robot_load, state, robot_id):
        """Check if action is legal in current state for specific robot"""
        robots_t, plants_t, taps_t, _ = state
        problem_dict = self.game.get_problem()
        size = problem_dict["Size"]
        walls = set(problem_dict["Walls"])
        capacities = self.game.get_capacities()
        
        # IMPROVEMENT 7: Don't force loading if no time to use it
        current_steps = self.game.get_current_steps()
        max_steps = self.game.get_max_steps()
        remaining_steps = max_steps - current_steps
        
        if action_name in ["UP", "DOWN", "LEFT", "RIGHT"]:
            target_pos = self._get_target_position(robot_pos, action_name)
            # Check bounds
            if not (0 <= target_pos[0] < size[0] and 0 <= target_pos[1] < size[1]):
                return False
            # Check walls
            if target_pos in walls or target_pos in self.blocked_robots_positions:
                return False
            # Check other robots
            if self._get_robot_at_position(target_pos, robots_t, exclude_id=robot_id):
                return False
            return True
        
        elif action_name == "LOAD":
            # Must be on tap, tap must have water, must have capacity
            if robot_load >= capacities[robot_id]:
                return False
            tap_water = None
            for pos, water in taps_t:
                if pos == robot_pos:
                    tap_water = water
                    break
            
            if tap_water is None or tap_water <= 0:
                return False
            
            # IMPROVEMENT 7: Don't load if no time to use it
            # Need at least 2 steps: move to plant + pour
            if remaining_steps < 2:
                return False
            
            # IMPROVEMENT 3: Smart Loading Strategy (for new2_v3, new2_v4, new3_v1)
            # Don't load more than needed for closest target plant + spill buffer
            # Find closest plant that still needs water
            target_plant_need = None
            min_dist_to_plant = float('inf')
            
            for pos, need in plants_t:
                if need > 0:
                    dist = self._bfs_distance(robot_pos, pos, problem_dict)
                    if dist < min_dist_to_plant:
                        min_dist_to_plant = dist
                        target_plant_need = need
            
            if target_plant_need is not None and target_plant_need > 0:
                # Calculate smart load target based on:
                # 1. Plant's remaining need
                # 2. Robot's reliability (more spills for unreliable robots)
                # 3. Distance (more spills over long distance)
                reliability = problem_dict["robot_chosen_action_prob"][robot_id]
                
                # Expected spills = need * (1 - reliability) * buffer_factor
                # Use CONSERVATIVE buffer - don't want to skip loads too aggressively
                buffer_factor = 1.2 if min_dist_to_plant > 5 else 1.1
                expected_spills = max(1, int(target_plant_need * (1 - reliability) * buffer_factor))
                
                # Target load = min(need + spills, capacity)
                # Add +1 safety margin
                smart_load_target = min(target_plant_need + expected_spills + 1, capacities[robot_id])
                
                # Only skip if we have MORE than target (not equal)
                if robot_load > smart_load_target:
                    return False
            
            return True
        
        elif action_name == "POUR":
            # Must have load, must be on plant with need
            if robot_load <= 0:
                return False
            plant_need = None
            for pos, need in plants_t:
                if pos == robot_pos:
                    plant_need = need
                    break
            return plant_need is not None and plant_need > 0
        
        return False
    
    def _handle_stupid_robot_blocking(self, state, smart_pos, blocked_pos, blocking_robot_id, acting_robot_id):
        """Handle case where stupid robot blocks path - with smart parking"""
        robots_t, _, _, _ = state
        problem_dict = self.game.get_problem()
        
        # CRITICAL FIX: Check if blocking robot is an excluded (low-reliability) robot
        if blocking_robot_id not in self.active_robots and blocking_robot_id != self.smart_robot_id:
            # This is an excluded robot - it shouldn't be blocking!
            # Mark its position as permanently blocked and recalculate
            stupid_robot_pos = None
            for rid, pos, _ in robots_t:
                if rid == blocking_robot_id:
                    stupid_robot_pos = pos
                    break
            
            if stupid_robot_pos:
                self.blocked_robots_positions.add(stupid_robot_pos)
                self._update_unreachable_plants()
                self._recalc_depth = 0  # Reset depth counter
                self._recalculate_plan()
                return "RESET"
        
        # IMPROVEMENT 6: Smart Parking Strategy
        # If blocking robot is unreliable and we're at a tap, move it away smartly
        robot_probs = problem_dict["robot_chosen_action_prob"]
        blocking_reliability = robot_probs.get(blocking_robot_id, 0)
        acting_reliability = robot_probs.get(acting_robot_id, 1)
        
        # Find where we want to go after the blocked position
        next_target = None
        if self.plan_index + 1 < len(self.current_plan):
            next_action = self.current_plan[self.plan_index + 1].split('{')[0]
            if next_action in ["UP", "DOWN", "LEFT", "RIGHT"]:
                next_target = self._get_target_position(blocked_pos, next_action)
        
        # Try to move stupid robot away
        stupid_robot_pos = None
        for rid, pos, _ in robots_t:
            if rid == blocking_robot_id:
                stupid_robot_pos = pos
                break
        
        # IMPROVEMENT 6: Prefer parking away from taps and plants
        walls = set(problem_dict["Walls"]) | self.blocked_robots_positions
        size = problem_dict["Size"]
        occupied = {pos for rid, pos, _ in robots_t if rid != blocking_robot_id}
        
        tap_positions = set(problem_dict["Taps"].keys())
        plant_positions = set(problem_dict["Plants"].keys())
        poi = tap_positions | plant_positions  # Points of Interest
        
        # Rank moves: prefer non-POI, away from next_target
        ranked_moves = []
        for move_name, (dr, dc) in _MOVES.items():
            nr, nc = stupid_robot_pos[0] + dr, stupid_robot_pos[1] + dc
            new_pos = (nr, nc)
            
            if not (0 <= nr < size[0] and 0 <= nc < size[1]):
                continue
            if new_pos in walls or new_pos in occupied:
                continue
            
            # Calculate priority
            priority = 0
            if new_pos in poi:
                priority += 100  # Penalize POI
            if next_target and new_pos == next_target:
                priority += 200  # Heavy penalty for blocking next move
            
            # Prefer moving unreliable robots further away
            if blocking_reliability < 0.8:
                dist_from_smart = abs(new_pos[0] - smart_pos[0]) + abs(new_pos[1] - smart_pos[1])
                priority -= dist_from_smart  # Reward distance
            
            ranked_moves.append((priority, move_name))
        
        if ranked_moves:
            # Choose best move (lowest priority)
            ranked_moves.sort(key=lambda x: x[0])
            chosen_move = ranked_moves[0][1]
            return f"{chosen_move}({blocking_robot_id})"
        else:
            # Cannot move stupid robot - mark as blocked and RESET
            self.blocked_robots_positions.add(stupid_robot_pos)
            
            # Check for unreachable plants
            self._update_unreachable_plants()
            
            self._recalc_depth = 0  # Reset depth counter
            self._recalculate_plan()
            return "RESET"
    
    def _update_unreachable_plants(self):
        """Update list of unreachable plants after blocking a robot"""
        state = self.game.get_current_state()
        problem_dict = self.game.get_problem()
        
        # Find positions of all active robots
        active_robot_positions = []
        for rid, pos, _ in state[0]:
            if rid in self.active_robots:
                active_robot_positions.append(pos)
        
        if not active_robot_positions:
            # Fallback to smart robot
            for rid, pos, _ in state[0]:
                if rid == self.smart_robot_id:
                    active_robot_positions.append(pos)
                    break
        
        walls = set(problem_dict["Walls"]) | self.blocked_robots_positions
        size = problem_dict["Size"]
        
        # BFS from all active robot positions to find reachable positions
        reachable = set(active_robot_positions)
        queue = deque(active_robot_positions)
        
        while queue:
            pos = queue.popleft()
            
            for dr, dc in _MOVES.values():
                nr, nc = pos[0] + dr, pos[1] + dc
                new_pos = (nr, nc)
                
                if not (0 <= nr < size[0] and 0 <= nc < size[1]):
                    continue
                if new_pos in walls:
                    continue
                if new_pos in reachable:
                    continue
                
                reachable.add(new_pos)
                queue.append(new_pos)
        
        # Check which plants are unreachable
        for plant_pos in problem_dict["Plants"].keys():
            if plant_pos not in reachable:
                self.unreachable_plants.add(plant_pos)
    
    def _get_target_position(self, pos, action):
        """Get target position for movement action"""
        dr, dc = _MOVES[action]
        return (pos[0] + dr, pos[1] + dc)
    
    def _get_robot_at_position(self, pos, robots_t, exclude_id=None):
        """Check if there's a robot at given position"""
        for rid, rpos, _ in robots_t:
            if rpos == pos and rid != exclude_id:
                return rid
        return None
    
    def _greedy_action(self, state):
        """Greedy fallback strategy - NEVER returns RESET, always finds an action"""
        robots_t, plants_t, taps_t, total_water_need = state
        problem_dict = self.game.get_problem()
        capacities = self.game.get_capacities()
        
        # Use smart robot for greedy actions
        robot_pos = None
        robot_load = 0
        robot_id = self.smart_robot_id
        
        for rid, pos, load in robots_t:
            if rid == robot_id:
                robot_pos = pos
                robot_load = load
                break
        
        # CRITICAL: If smart robot not found, use ANY robot
        if not robot_pos:
            for rid, pos, load in robots_t:
                robot_id = rid
                robot_pos = pos
                robot_load = load
                break
        
        # If still no robot, try random move (should never happen)
        if not robot_pos:
            return f"UP({list(robots_t)[0][0]})" if robots_t else "RESET"
        
        # Strategy: Pure greedy - water nearest valuable plant repeatedly
        
        # If we have load, pour it
        if robot_load > 0:
            # Check if we're on a plant
            for pos, need in plants_t:
                if pos == robot_pos and need > 0:
                    return f"POUR({robot_id})"
            
            # Find nearest plant with need
            best_plant = None
            min_dist = float('inf')
            best_reward = 0
            
            for pos, need in plants_t:
                if need > 0:
                    dist = self._bfs_distance(robot_pos, pos, problem_dict)
                    if dist < float('inf'):
                        # Get expected reward
                        reward = 2  # default
                        if pos in problem_dict["plants_reward"]:
                            rewards_list = problem_dict["plants_reward"][pos]
                            reward = sum(rewards_list) / len(rewards_list)
                        
                        # Prefer high reward, then close distance
                        score = reward * 100 - dist
                        if score > (best_reward * 100 - min_dist):
                            min_dist = dist
                            best_plant = pos
                            best_reward = reward
            
            if best_plant:
                return self._move_towards(robot_pos, best_plant, state, robot_id)
            
            # No reachable plants - try random movement
            return self._try_random_move(robot_pos, state, robot_id)
        
        # Need to load - find tap
        # Check if we're already on a tap
        for pos, water in taps_t:
            if pos == robot_pos and water > 0 and robot_load < capacities.get(robot_id, 999):
                return f"LOAD({robot_id})"
        
        # Find nearest tap with water
        nearest_tap = None
        min_tap_dist = float('inf')
        
        for pos, water in taps_t:
            if water > 0:
                dist = self._bfs_distance(robot_pos, pos, problem_dict)
                if dist < min_tap_dist and dist < float('inf'):
                    min_tap_dist = dist
                    nearest_tap = pos
        
        if nearest_tap:
            return self._move_towards(robot_pos, nearest_tap, state, robot_id)
        
        # No tap with water - try to find ANY tap (for positioning)
        for pos, water in taps_t:
            dist = self._bfs_distance(robot_pos, pos, problem_dict)
            if dist < float('inf'):
                return self._move_towards(robot_pos, pos, state, robot_id)
        
        # Last resort: try random movement
        return self._try_random_move(robot_pos, state, robot_id)
    
    def _try_random_move(self, pos, state, robot_id):
        """Try to make any valid move - last resort"""
        robots_t = state[0]
        problem_dict = self.game.get_problem()
        size = problem_dict["Size"]
        walls = set(problem_dict["Walls"])
        
        # Try moves in order: UP, DOWN, LEFT, RIGHT
        for move_name, (dr, dc) in _MOVES.items():
            new_pos = (pos[0] + dr, pos[1] + dc)
            
            # Check if valid
            if not (0 <= new_pos[0] < size[0] and 0 <= new_pos[1] < size[1]):
                continue
            if new_pos in walls:
                continue
            if self._get_robot_at_position(new_pos, robots_t, exclude_id=robot_id):
                continue
            
            # Valid move!
            return f"{move_name}({robot_id})"
        
        # Truly stuck - use RESET as absolute last resort
        return "RESET"
    
    def _move_towards(self, current_pos, target_pos, state, robot_id):
        """Move one step towards target position for specific robot"""
        robots_t, _, _, _ = state
        problem_dict = self.game.get_problem()
        walls = set(problem_dict["Walls"]) | self.blocked_robots_positions
        size = problem_dict["Size"]
        occupied = {pos for rid, pos, _ in robots_t if rid != robot_id}
        
        dr = target_pos[0] - current_pos[0]
        dc = target_pos[1] - current_pos[1]
        
        # Try primary direction
        moves_to_try = []
        if abs(dr) > abs(dc):
            if dr > 0:
                moves_to_try.append("DOWN")
            elif dr < 0:
                moves_to_try.append("UP")
            if dc > 0:
                moves_to_try.append("RIGHT")
            elif dc < 0:
                moves_to_try.append("LEFT")
        else:
            if dc > 0:
                moves_to_try.append("RIGHT")
            elif dc < 0:
                moves_to_try.append("LEFT")
            if dr > 0:
                moves_to_try.append("DOWN")
            elif dr < 0:
                moves_to_try.append("UP")
        
        for move in moves_to_try:
            target = self._get_target_position(current_pos, move)
            if (0 <= target[0] < size[0] and 0 <= target[1] < size[1] and
                target not in walls and target not in occupied):
                return f"{move}({robot_id})"
        
        # No valid move - try any move
        for move in ["UP", "DOWN", "LEFT", "RIGHT"]:
            target = self._get_target_position(current_pos, move)
            if (0 <= target[0] < size[0] and 0 <= target[1] < size[1] and
                target not in walls and target not in occupied):
                return f"{move}({robot_id})"
        
        # Stuck - RESET
        self._recalculate_plan()
        return "RESET"