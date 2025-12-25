import random

ids = ["123456789"]

class Controller:
    def __init__(self, problem):
        # 1. Extract the model
        if hasattr(problem, 'get_problem'):
            self.model = problem.get_problem()
        elif hasattr(problem, 'get_model'):
            self.model = problem.get_model()
        elif hasattr(problem, '_model'):
            self.model = problem._model
        else:
            self.model = problem 

        # 2. Parse static data
        self.walls = set(self.model.get('Walls', []))
        self.rows, self.cols = self.model['Size']
        self.probs = self.model['robot_chosen_action_prob']
        self.plants_reward = self.model['plants_reward']
        self.robot_capacities = {
            rid: val[3] for rid, val in self.model['Robots'].items()
        }
        
        # Max search depth (keep small to stay within time limit)
        self.MAX_DEPTH = 2 

    def get_legal_actions(self, state):
        robots, plants, taps, _ = state
        legal_actions = []
        plant_locs = {p[0]: p[1] for p in plants}
        tap_locs = {t[0]: t[1] for t in taps}
        robot_positions = {r[1] for r in robots}

        for robot in robots:
            rid, (r, c), load = robot
            capacity = self.robot_capacities[rid]

            # MOVES
            moves = [('UP', (-1, 0)), ('DOWN', (1, 0)), ('LEFT', (0, -1)), ('RIGHT', (0, 1))]
            for move_name, (dr, dc) in moves:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if (nr, nc) not in self.walls:
                        if (nr, nc) not in robot_positions:
                            legal_actions.append(f"{move_name} ({rid})")

            # LOAD
            if (r, c) in tap_locs and tap_locs[(r, c)] > 0 and load < capacity:
                legal_actions.append(f"LOAD ({rid})")

            # POUR
            if (r, c) in plant_locs and plant_locs[(r, c)] > 0 and load > 0:
                legal_actions.append(f"POUR ({rid})")

        legal_actions.append("RESET")
        return legal_actions

    def heuristic(self, state):
        """
        Estimates the value of a state when we stop searching.
        Rewards: Having water, being close to plants, being close to taps.
        """
        robots, plants, taps, total_need = state
        value = 0
        
        # 1. Reward for satisfying needs (The goal is to reduce this)
        # We use negative need as a base score
        value -= (total_need * 10) 

        plant_locs = [p[0] for p in plants]
        tap_locs = [t[0] for t in taps]

        for rid, (r, c), load in robots:
            # 2. Value for carrying water (potential reward)
            value += load * 2.0 

            # 3. Distance Heuristic
            if load > 0:
                # If loaded, move to nearest plant
                if plant_locs:
                    dists = [abs(r-pr) + abs(c-pc) for pr, pc in plant_locs]
                    min_dist = min(dists)
                    value -= min_dist * 0.5 # Penalty for distance
            else:
                # If empty, move to nearest tap
                if tap_locs:
                    dists = [abs(r-tr) + abs(c-tc) for tr, tc in tap_locs]
                    min_dist = min(dists)
                    value -= min_dist * 0.5

        return value

    def apply_action(self, state, action):
        """
        Simulates the NEXT state deterministically for the search.
        (Simplified transition model: Assumes success to keep branching factor low)
        """
        if action == "RESET":
            # In a real search, we might just return the initial state, 
            # but RESET is complex. We'll treat it as a terminal bad move for depth search.
            return None 

        parts = action.split(' ')
        act_type = parts[0]
        rid = int(parts[1].replace('(', '').replace(')', ''))

        robots, plants, taps, need = state
        
        # Deep copy structures (tuples are immutable, so we reconstruct)
        new_robots = []
        curr_robot = None
        
        for r_data in robots:
            if r_data[0] == rid:
                curr_robot = r_data
            else:
                new_robots.append(r_data)
        
        r, c = curr_robot[1]
        load = curr_robot[2]
        
        reward = 0
        
        # --- EXECUTE LOGIC (Simplified Success Case) ---
        if act_type == "UP":    r -= 1
        elif act_type == "DOWN":  r += 1
        elif act_type == "LEFT":  c -= 1
        elif act_type == "RIGHT": c += 1
        
        elif act_type == "LOAD":
            # Find tap
            new_taps = []
            for t_pos, t_amt in taps:
                if t_pos == (r, c):
                    if t_amt - 1 > 0: new_taps.append((t_pos, t_amt - 1))
                else:
                    new_taps.append((t_pos, t_amt))
            taps = tuple(new_taps)
            load += 1
            
        elif act_type == "POUR":
            # Find plant
            new_plants = []
            for p_pos, p_need in plants:
                if p_pos == (r, c):
                    # Calc Avg Reward
                    possible = self.plants_reward.get((r,c), [0])
                    reward = sum(possible)/len(possible)
                    
                    if p_need - 1 > 0: new_plants.append((p_pos, p_need - 1))
                    need -= 1
                else:
                    new_plants.append((p_pos, p_need))
            plants = tuple(new_plants)
            load -= 1

        # Reconstruct state
        new_robots.append((rid, (r, c), load))
        new_robots.sort(key=lambda x: x[0]) # Keep sorted by ID
        
        return (tuple(new_robots), plants, taps, need), reward

    def expectimax(self, state, depth):
        """
        Recursive Bellman Backup (approximated).
        V(s) = max_a [ R + gamma * V(s') ]
        """
        if depth == 0 or state[3] == 0: # Base case or Goal reached
            return self.heuristic(state)

        actions = self.get_legal_actions(state)
        # Optimization: prune RESET from search tree
        actions = [a for a in actions if "RESET" not in a]
        
        if not actions:
            return self.heuristic(state)

        best_val = -float('inf')

        for action in actions:
            # 1. Simulate Outcome (assuming success for speed, or add failure branch)
            # To be truly rigorous based on slides, we should do:
            # Value = P(success)*V(success_state) + P(fail)*V(fail_state)
            # Here we simplify to just Success Outcome for the assignment time limit.
            
            result = self.apply_action(state, action)
            if result is None: continue
            
            next_state, immediate_reward = result
            
            # Recursive call
            val = immediate_reward + self.expectimax(next_state, depth - 1)
            
            if val > best_val:
                best_val = val
        
        return best_val

    def choose_next_action(self, state):
        """
        The Agent's decision loop.
        """
        legal_actions = self.get_legal_actions(state)
        candidates = [a for a in legal_actions if "RESET" not in a]
        if not candidates: return "RESET"

        best_action = candidates[0]
        max_val = -float('inf')

        for action in candidates:
            # 1. Get immediate next state
            res = self.apply_action(state, action)
            if res is None: continue
            next_s, r = res
            
            # 2. Run Expectimax from that state
            # Total Value = Immediate Reward + Future Value
            val = r + self.expectimax(next_s, self.MAX_DEPTH)
            
            if val > max_val:
                max_val = val
                best_action = action
        
        return best_action