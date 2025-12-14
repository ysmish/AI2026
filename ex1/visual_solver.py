import tkinter as tk
from tkinter import ttk
import time
import re
import heapq
import random
import threading
import copy

# --- IMPORT YOUR MODULES ---
try:
    import ex1
    import ex1_check
except ImportError as e:
    print(f"Error: Could not import modules. {e}")
    print("Ensure 'ex1.py' and 'ex1_check.py' are in the same folder.")
    exit(1)

# ==========================================
# PART 1: SEARCH ENGINE
# ==========================================
class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
    def __lt__(self, other):
        return self.path_cost < other.path_cost

def astar_search(problem, heuristic=None):
    h = heuristic if heuristic is not None else problem.h_astar
    node = Node(problem.initial)
    frontier = []
    heapq.heappush(frontier, (node.path_cost + h(node), node))
    explored = set()
    count = 0
    
    # DEBUG: Check initial heuristic
    if heuristic is None:
        print(f"DEBUG: Initial h(start) = {h(node)}")
    
    while frontier:
        _, node = heapq.heappop(frontier)
        if problem.goal_test(node.state): return node
        
        state_key = node.state
        if state_key in explored: continue
        explored.add(state_key)
        
        count += 1
        if count > 300000: return None

        for action, next_state in problem.successor(node.state):
            child_node = Node(next_state, node, action, node.path_cost + 1)
            if child_node.state not in explored:
                heapq.heappush(frontier, (child_node.path_cost + h(child_node), child_node))
    return None

def reconstruct_path(node):
    path = []
    while node.parent:
        path.append(node.action)
        node = node.parent
    return list(reversed(path))

# ==========================================
# PART 2: MAP UTILS
# ==========================================
def get_tests_from_ex1_check():
    tests = {}
    for name, val in ex1_check.__dict__.items():
        if isinstance(val, dict) and "Size" in val and "Robots" in val:
            tests[name] = val
    if "inputs" in ex1_check.__dict__ and isinstance(ex1_check.inputs, list):
        for i, val in enumerate(ex1_check.inputs):
            if isinstance(val, dict): tests[f"inputs[{i}]"] = val
    return tests

def generate_random_map():
    # You can paste your FAILED MAP DATA here to reproduce the 163 vs 161 error
    rows = random.randint(8, 12) 
    cols = random.randint(8, 12)
    walls = set()
    for r in range(rows):
        for c in range(cols):
            if random.random() < 0.2: walls.add((r, c))
    
    def get_empty():
        for _ in range(100):
            r, c = random.randint(0, rows-1), random.randint(0, cols-1)
            if (r,c) not in walls: return (r,c)
        return (0,0)

    robots = {}
    for i in range(random.randint(1, 2)):
        pos = get_empty()
        robots[10+i] = (pos[0], pos[1], 0, random.randint(1, 3))

    taps = {}
    total_supply = 0
    for _ in range(random.randint(1, 2)):
        pos = get_empty()
        while pos in robots.values(): pos = get_empty()
        amt = random.randint(5, 10)
        taps[pos] = amt
        total_supply += amt

    plants = {}
    total_needed = 0
    for _ in range(random.randint(1, 3)):
        pos = get_empty()
        while pos in taps or pos in [r[:2] for r in robots.values()]: pos = get_empty()
        amt = random.randint(1, 4)
        plants[pos] = amt
        total_needed += amt

    while total_needed > total_supply:
        tap_pos = random.choice(list(taps.keys()))
        taps[tap_pos] += 1
        total_supply += 1

    return {"Size": (rows, cols), "Walls": walls, "Taps": taps, "Plants": plants, "Robots": robots}

# ==========================================
# PART 3: VISUALIZER APP V4
# ==========================================
class VisualizerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("AI Visualizer: A* vs UCS + Pruning View")
        
        self.map_data = None
        self.virtual_walls = set()
        
        self.history_astar = []
        self.path_astar = []
        self.history_ucs = []
        self.path_ucs = []
        
        self.active_history = []
        self.active_path = []
        self.current_step = 0
        self.is_playing = False
        self.show_pruning = tk.BooleanVar(value=True) 
        
        self.cell_size = 30
        self.offset_x = 0
        self.offset_y = 0
        self.rows = 0
        self.cols = 0
        
        self.panel = tk.Frame(root, width=320, bg="#f0f0f0")
        self.panel.pack(side=tk.RIGHT, fill=tk.Y)
        self.panel.pack_propagate(False) 
        
        self.canvas_frame = tk.Frame(root, bg="#333")
        self.canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(self.canvas_frame, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self.on_resize)

        self.build_control_panel()

    def build_control_panel(self):
        p = self.panel
        
        tk.Label(p, text="SETUP", font=("Arial", 10, "bold"), bg="#f0f0f0").pack(pady=(10,2))
        self.available_tests = get_tests_from_ex1_check()
        self.selected_test_name = tk.StringVar()
        if self.available_tests:
            self.selected_test_name.set(list(self.available_tests.keys())[0])
            self.dropdown = ttk.OptionMenu(p, self.selected_test_name, list(self.available_tests.keys())[0], *self.available_tests.keys())
            self.dropdown.pack(pady=2, padx=10, fill=tk.X)
            tk.Button(p, text="Load Test", command=self.load_selected_test, bg="#ddd").pack(pady=2, fill=tk.X, padx=10)
        tk.Button(p, text="Random Map", command=self.load_random_map, bg="#ffcccb").pack(pady=2, fill=tk.X, padx=10)
        
        ttk.Separator(p, orient='horizontal').pack(fill='x', pady=10)
        
        tk.Label(p, text="SOLVER", font=("Arial", 10, "bold"), bg="#f0f0f0").pack(pady=2)
        self.btn_solve_astar = tk.Button(p, text="Run A* Only", command=lambda: self.run_solver(mode="ASTAR"), bg="#90ee90", state=tk.DISABLED)
        self.btn_solve_astar.pack(pady=2, fill=tk.X, padx=10)
        
        self.btn_compare = tk.Button(p, text="Compare A* vs UCS", command=lambda: self.run_solver(mode="COMPARE"), bg="#ADD8E6", state=tk.DISABLED)
        self.btn_compare.pack(pady=2, fill=tk.X, padx=10)

        self.lbl_result = tk.Label(p, text="Ready", bg="white", relief="sunken", height=4, justify="left", anchor="nw")
        self.lbl_result.pack(pady=5, padx=10, fill=tk.X)

        tk.Label(p, text="VIEW OPTIONS", font=("Arial", 10, "bold"), bg="#f0f0f0").pack(pady=(10, 2))
        
        self.chk_pruning = tk.Checkbutton(p, text="Show Pruned Rocks", variable=self.show_pruning, command=self.render_current_step, bg="#f0f0f0")
        self.chk_pruning.pack(anchor="w", padx=10)

        self.view_var = tk.StringVar(value="ASTAR")
        self.frm_view = tk.Frame(p, bg="#f0f0f0")
        self.frm_view.pack(fill=tk.X, padx=10, pady=5)
        self.rb_astar = tk.Radiobutton(self.frm_view, text="Show A*", variable=self.view_var, value="ASTAR", command=self.switch_view, bg="#f0f0f0", state=tk.DISABLED)
        self.rb_astar.pack(side=tk.LEFT, expand=True)
        self.rb_ucs = tk.Radiobutton(self.frm_view, text="Show UCS", variable=self.view_var, value="UCS", command=self.switch_view, bg="#f0f0f0", state=tk.DISABLED)
        self.rb_ucs.pack(side=tk.LEFT, expand=True)

        ttk.Separator(p, orient='horizontal').pack(fill='x', pady=10)

        tk.Label(p, text="TIMELINE", font=("Arial", 10, "bold"), bg="#f0f0f0").pack(pady=2)
        self.slider = ttk.Scale(p, from_=0, to=100, orient=tk.HORIZONTAL, command=self.on_slider_drag)
        self.slider.pack(fill=tk.X, padx=10, pady=5)
        
        btn_frame = tk.Frame(p, bg="#f0f0f0")
        btn_frame.pack(pady=5)
        tk.Button(btn_frame, text="<<", width=4, command=self.step_back).pack(side=tk.LEFT, padx=2)
        self.btn_play = tk.Button(btn_frame, text="Play", width=6, command=self.toggle_play)
        self.btn_play.pack(side=tk.LEFT, padx=2)
        tk.Button(btn_frame, text=">>", width=4, command=self.step_fwd).pack(side=tk.LEFT, padx=2)

        self.lbl_step = tk.Label(p, text="Step: 0/0", bg="#f0f0f0")
        self.lbl_step.pack(pady=2)
        self.lbl_action = tk.Label(p, text="", bg="#f0f0f0", fg="blue")
        self.lbl_action.pack()

    def on_resize(self, event):
        if not self.map_data: return
        self.recalc_metrics(event.width, event.height)
        self.render_current_step()

    def recalc_metrics(self, canvas_w, canvas_h):
        if self.rows == 0 or self.cols == 0: return
        pad = 20
        scale_w = (canvas_w - pad) / self.cols
        scale_h = (canvas_h - pad) / self.rows
        self.cell_size = min(scale_w, scale_h)
        if self.cell_size < 5: self.cell_size = 5
        self.offset_x = (canvas_w - (self.cols * self.cell_size)) / 2
        self.offset_y = (canvas_h - (self.rows * self.cell_size)) / 2

    def load_selected_test(self):
        name = self.selected_test_name.get()
        if name in self.available_tests:
            self.load_map(self.available_tests[name], f"Loaded: {name}")

    def load_random_map(self):
        self.load_map(generate_random_map(), "Loaded: Random Map")

    def load_map(self, map_dict, msg):
        self.map_data = map_dict
        self.rows, self.cols = map_dict["Size"]
        self.recalc_metrics(self.canvas.winfo_width(), self.canvas.winfo_height())
        
        self.is_playing = False
        self.btn_play.config(text="Play")
        self.slider.config(value=0)
        self.rb_astar.config(state=tk.DISABLED)
        self.rb_ucs.config(state=tk.DISABLED)
        self.view_var.set("ASTAR")
        self.virtual_walls = set() 

        snapshot = {
            "Robots": copy.deepcopy(map_dict["Robots"]),
            "Taps": copy.deepcopy(map_dict["Taps"]),
            "Plants": copy.deepcopy(map_dict["Plants"])
        }
        self.active_history = [snapshot]
        self.active_path = []
        
        self.lbl_result.config(text=msg)
        self.render_current_step()
        self.btn_solve_astar.config(state=tk.NORMAL)
        self.btn_compare.config(state=tk.NORMAL)

    def run_solver(self, mode):
        self.btn_solve_astar.config(state=tk.DISABLED)
        self.btn_compare.config(state=tk.DISABLED)
        self.lbl_result.config(text="Computing...", fg="black")
        
        thread = threading.Thread(target=self.solve_thread, args=(mode,))
        thread.daemon = True
        thread.start()

    def solve_thread(self, mode):
        try:
            problem = ex1.create_watering_problem(self.map_data)
            
            # Extract Virtual Walls
            original_walls = set(self.map_data["Walls"])
            pruned_walls = problem.walls - original_walls
            
            t0 = time.time()
            node_astar = astar_search(problem)
            t_astar = time.time() - t0
            path_astar = reconstruct_path(node_astar) if node_astar else []
            hist_astar = self.precalculate_history(path_astar)

            path_ucs = []
            hist_ucs = []
            t_ucs = 0
            
            if mode == "COMPARE":
                t0 = time.time()
                node_ucs = astar_search(problem, heuristic=lambda n: 0)
                t_ucs = time.time() - t0
                path_ucs = reconstruct_path(node_ucs) if node_ucs else []
                hist_ucs = self.precalculate_history(path_ucs)
            
            self.root.after(0, lambda: self.on_solve_done(mode, path_astar, hist_astar, t_astar, path_ucs, hist_ucs, t_ucs, pruned_walls))

        except Exception as e:
            self.root.after(0, lambda: self.lbl_result.config(text=f"Error:\n{e}", fg="red"))
            print(e)

    def precalculate_history(self, path):
        current_state = {
            "Robots": copy.deepcopy(self.map_data["Robots"]),
            "Taps": copy.deepcopy(self.map_data["Taps"]),
            "Plants": copy.deepcopy(self.map_data["Plants"])
        }
        history = [copy.deepcopy(current_state)]
        
        robots = current_state["Robots"]
        taps = current_state["Taps"]
        plants = current_state["Plants"]
        
        for action_str in path:
            match = re.match(r"([A-Z]+)\{(\d+)\}", action_str)
            if match:
                act, rid_str = match.groups()
                rid = int(rid_str)
                r, c, load, cap = robots[rid]
                
                if act == "UP": r -= 1
                elif act == "DOWN": r += 1
                elif act == "LEFT": c -= 1
                elif act == "RIGHT": c += 1
                elif act == "LOAD":
                    if (r, c) in taps: taps[(r,c)] -= 1
                    load += 1
                elif act == "POUR":
                    if (r, c) in plants: plants[(r,c)] -= 1
                    load -= 1
                robots[rid] = (r, c, load, cap)
            
            history.append({
                "Robots": copy.deepcopy(robots),
                "Taps": copy.deepcopy(taps),
                "Plants": copy.deepcopy(plants)
            })
        return history

    def on_solve_done(self, mode, p_astar, h_astar, t_astar, p_ucs, h_ucs, t_ucs, pruned_walls):
        self.virtual_walls = pruned_walls 
        self.btn_solve_astar.config(state=tk.NORMAL)
        self.btn_compare.config(state=tk.NORMAL)
        
        self.path_astar = p_astar; self.history_astar = h_astar
        self.path_ucs = p_ucs; self.history_ucs = h_ucs
        
        if mode == "ASTAR":
            self.rb_astar.config(state=tk.NORMAL)
            self.rb_ucs.config(state=tk.DISABLED)
            self.view_var.set("ASTAR")
            self.switch_view()
            self.lbl_result.config(text=f"A* Solved: {len(p_astar)} steps\nTime: {t_astar:.4f}s")
        elif mode == "COMPARE":
            self.rb_astar.config(state=tk.NORMAL)
            self.rb_ucs.config(state=tk.NORMAL)
            self.view_var.set("ASTAR")
            self.switch_view()
            txt = f"A*: {len(p_astar)} steps ({t_astar:.3f}s)\nUCS: {len(p_ucs)} steps ({t_ucs:.3f}s)\n"
            if len(p_astar) > len(p_ucs): txt += "FAIL: Suboptimal!"
            elif len(p_astar) == len(p_ucs): txt += "PASS: Optimal."
            self.lbl_result.config(text=txt)

    def switch_view(self):
        mode = self.view_var.get()
        self.is_playing = False
        self.btn_play.config(text="Play")
        if mode == "ASTAR":
            self.active_history = self.history_astar
            self.active_path = self.path_astar
        else:
            self.active_history = self.history_ucs
            self.active_path = self.path_ucs
        self.slider.config(to=len(self.active_history) - 1, value=0)
        self.current_step = 0
        self.render_current_step()

    def toggle_play(self):
        if not self.active_history: return
        self.is_playing = not self.is_playing
        self.btn_play.config(text="Pause" if self.is_playing else "Play")
        if self.is_playing: self.run_animation_loop()

    def step_fwd(self):
        if self.current_step < len(self.active_history) - 1:
            self.current_step += 1
            self.slider.set(self.current_step)
            self.render_current_step()

    def step_back(self):
        if self.current_step > 0:
            self.current_step -= 1
            self.slider.set(self.current_step)
            self.render_current_step()

    def on_slider_drag(self, val):
        self.current_step = int(float(val))
        self.render_current_step()

    def run_animation_loop(self):
        if not self.is_playing: return
        if self.current_step < len(self.active_history) - 1:
            self.current_step += 1
            self.slider.set(self.current_step)
            self.render_current_step()
            self.root.after(100, self.run_animation_loop)
        else:
            self.is_playing = False
            self.btn_play.config(text="Play")

    def render_current_step(self):
        if not self.active_history: return
        self.canvas.delete("all")
        
        state = self.active_history[self.current_step]
        self.lbl_step.config(text=f"Step: {self.current_step} / {len(self.active_history)-1}")
        
        if 0 < self.current_step <= len(self.active_path):
            self.lbl_action.config(text=f"Last: {self.active_path[self.current_step-1]}")
        else:
            self.lbl_action.config(text="Start")

        walls = self.map_data["Walls"]
        show_ghosts = self.show_pruning.get()
        
        for r in range(self.rows):
            for c in range(self.cols):
                x1 = self.offset_x + c * self.cell_size
                y1 = self.offset_y + r * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                # Draw Walls
                if (r, c) in walls:
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill="#444", outline="")
                elif show_ghosts and (r, c) in self.virtual_walls:
                    # GHOST WALLS (Red Stipple)
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill="#FF4444", stipple="gray50", outline="")
                else:
                    self.canvas.create_rectangle(x1, y1, x2, y2, outline="#eee")

        for (r, c), amt in state["Taps"].items(): self.draw_entity(r, c, "blue", amt, "T")
        for (r, c), amt in state["Plants"].items(): self.draw_entity(r, c, "green", amt, "P")

        colors = ["red", "orange", "purple", "cyan"]
        for i, rid in enumerate(sorted(state["Robots"].keys())):
            r, c, load, cap = state["Robots"][rid]
            self.draw_robot(rid, r, c, load, cap, colors[i % len(colors)])

    def draw_entity(self, r, c, color, amt, char):
        x = self.offset_x + c * self.cell_size + self.cell_size/2
        y = self.offset_y + r * self.cell_size + self.cell_size/2
        sz = self.cell_size / 3
        if char == "T": self.canvas.create_oval(x-sz, y-sz, x+sz, y+sz, fill=color)
        else: self.canvas.create_rectangle(x-sz, y-sz, x+sz, y+sz, fill=color)
        self.canvas.create_text(x, y, text=str(amt), fill="white", font=("Arial", int(self.cell_size/2.5), "bold"))

    def draw_robot(self, rid, r, c, load, cap, color):
        x = self.offset_x + c * self.cell_size + self.cell_size/2
        y = self.offset_y + r * self.cell_size + self.cell_size/2
        sz = self.cell_size / 2 - 2
        self.canvas.create_oval(x-sz, y-sz, x+sz, y+sz, fill=color, outline="black", width=2)
        self.canvas.create_text(x, y, text=f"{rid}\n{load}/{cap}", font=("Arial", int(self.cell_size/3.5), "bold"), fill="black")

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1000x650")
    app = VisualizerApp(root)
    root.mainloop()