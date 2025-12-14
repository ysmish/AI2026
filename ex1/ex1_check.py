import time
import ex1
import search

# --- ANSI COLORS FOR TERMINAL OUTPUT ---
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"

def run_problem(func, targs=(), kwargs=None):
    if kwargs is None:
        kwargs = {}
    try:
        result = func(*targs, **kwargs)
        return result
    except Exception as e:
        return (-3, e)

def solve_problems(problem, algorithm):
    try:
        p = ex1.create_watering_problem(problem)
    except Exception as e:
        print(f"{RED}Error creating problem: {e}{RESET}")
        return None

    if algorithm == "gbfs":
        result = run_problem((lambda p: search.greedy_best_first_graph_search(p, p.h_gbfs)), targs=[p])
    else:
        result = run_problem((lambda p: search.astar_search(p, p.h_astar)), targs=[p])

    if result and isinstance(result[0], search.Node):
        solve = result[0].path()[::-1]
        solution = [pi.action for pi in solve][1:]
        return len(solution)
    else:
        return None  # No solution found

# ==========================================
# PROBLEM DEFINITIONS
# ==========================================

# Problem 1: Tiny, no walls (Optimal: 8)
problem1 = {
    "Size":   (3, 3),
    "Walls":  set(),
    "Taps":   {(1, 1): 3},
    "Plants": {(0, 2): 2},
    "Robots": {10: (2, 0, 0, 2)},
}

# Problem 2: Small with walls (Optimal: 20)
problem2 = {
    "Size":  (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps":  {(1, 1): 6},
    "Plants": {(0, 2): 3, (2, 0): 2},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
}

# Problem 3: Corridor 5x3 (Optimal: 28)
problem3 = {
    "Size":  (5, 3),
    "Walls": {(1, 1), (3, 1)},
    "Taps": {(0, 0): 5},
    "Plants": {(4, 2): 4},
    "Robots": {10: (2, 0, 0, 2)},
}

# Problem 4 (Optimal: 13)
problem4 = {
    "Size":  (5, 5),
    "Walls": {(0, 1),(1, 1),(2, 1), (0, 3),(1, 3),(2, 3)},
    "Taps": {(3, 2): 1, (4, 2): 1},
    "Plants": {(0, 2): 1, (1, 2): 1},
    "Robots": {10: (3, 1, 0, 1), 11: (3, 3, 0, 1)},
}

# Problem 5: Deadend (Impossible)
problem5_deadend = {
    "Size":  (3, 4),
    "Walls": set(),
    "Taps": {(1, 1): 3},
    "Plants": {(0, 3): 2, (2, 3): 2},
    "Robots": {10: (1, 0, 0, 2)},
}

# Problem 6 (Optimal: 8)
problem6 = {
    "Size":  (8, 8),
    "Walls": {*((r, c) for r in range(8) for c in range(8) if not (r == 1 and c in (0, 1, 2)))},
    "Taps": {(1, 1): 3},
    "Plants": {(1, 2): 3},
    "Robots": {10: (1, 0, 0, 3)},
}

# Problem 7 (Optimal: 21)
problem7 = {
    "Size":  (4, 4),
    "Walls": set(),
    "Taps": {(2, 2): 18},
    "Plants": {(0, 3): 3, (3, 0): 3},
    "Robots": {10: (2, 1, 0, 3), 11: (2, 0, 0, 3)},
}

# Hard 1 (Optimal: 31)
problem_hard1 = {
    "Size":  (5, 6),
    "Walls": {(1, 2), (1, 3), (3, 2), (3, 3)},
    "Taps": {(2, 2): 12},
    "Plants": {(0, 1): 3, (4, 5): 6},
    "Robots": {10: (2, 1, 0, 6), 11: (2, 4, 0, 3)},
}

# Hard 2 (Optimal: 24)
problem_hard2 = {
    "Size":  (5, 5),
    "Walls": {(0, 0), (0, 4), (4, 0), (4, 4)},
    "Taps": {(2, 2): 9},
    "Plants": {(1, 2): 3, (2, 1): 3, (3, 2): 3},
    "Robots": {10: (2, 3, 0, 3)},
}

# Hard 3 (Optimal: 24)
problem_hard3 = {
    "Size":  (5, 5),
    "Walls": {(0, 0), (0, 4), (4, 0), (4, 4)},
    "Taps": {(2, 2): 9},
    "Plants": {(1, 2): 3, (2, 1): 3, (3, 2): 3},
    "Robots": {10: (2, 3, 0, 3)}, 
}

# Hard 4 (Optimal: 25)
problem_hard4 = {
    "Size":  (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 6, (3, 3): 6},
    "Plants": {(0, 0): 3, (4, 5): 3},
    "Robots": {10: (1, 1, 0, 3), 11: (3, 4, 0, 2)},
}

# Hard 5 (Optimal: 29)
problem_hard5 = {
    "Size":  (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 8, (3, 3): 8},
    "Plants": {(0, 0): 4, (4, 5): 4},
    "Robots": {10: (1, 1, 0, 4), 11: (3, 4, 0, 3)},
}

# Hard 6 (Optimal: 33)
problem_hard6 = {
    "Size":  (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 10, (3, 3): 10},
    "Plants": {(0, 0): 5, (4, 5): 5},
    "Robots": {10: (1, 1, 0, 5), 11: (3, 4, 0, 4)},
}

# Load (Optimal: 65)
problem_load = {
    "Size":  (10, 4),
    "Walls": {(0,1),(1, 1), (2, 1), (3, 1), (4, 1), (6, 1), (7, 1), (8, 1), (9, 1),(4,2), (4,3),(6,2), (6,3)},
    "Taps": {(5, 3): 20},
    "Plants": {(0, 0): 10, (9, 0): 10},
    "Robots": {10: (2, 0, 0, 2), 11: (7, 0, 0, 20)},
}

# 10x10 Single (Optimal: 106)
problem_10x10_single = {
    "Size":  (10, 10),
    "Walls": set(),
    "Taps": {(5, 5): 24},
    "Plants": {(0, 0): 5, (0, 9): 5, (9, 0): 5, (9, 9): 5},
    "Robots": {10: (9, 5, 0, 5)},
}

# 12x12 Snake (Optimal: 249)
problem_12x12_snake = {
    "Size":  (12, 12),
    "Walls": {
        (1, 3), (2, 3), (3, 3), (4, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3),
        (1, 6), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (8, 6), (10, 6),
        (1, 9), (2, 9), (3, 9), (4, 9), (5, 9), (6, 9), (8, 9), (9, 9), (10, 9),
    },
    "Taps": {(5, 1): 24},
    "Plants": {(2, 11): 5, (7, 11): 5, (10, 10): 5, (0, 8): 5},
    "Robots": {10: (11, 1, 0, 3)},
}

# 12x12 Snake Hard (Optimal: 343)
problem_12x12_snake_hard = {
    "Size":  (12, 12),
    "Walls": {
        (1, 3), (2, 3), (3, 3), (4, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3),
        (1, 6), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (8, 6), (10, 6),
        (1, 9), (2, 9), (3, 9), (4, 9), (5, 9), (6, 9), (8, 9), (9, 9), (10, 9),
    },
    "Taps": {(5, 1): 24},
    "Plants": {(2, 11): 5, (7, 11): 5, (10, 10): 5, (0, 8): 5},
    "Robots": {10: (11, 1, 0, 2)},
}

# Cannot be solved in a reasonable time using the A* or GBFS method:
# If there is a solution in GBFS, it probably includes an illegal step.
# If there is a solution in A*, it is not the optimal solution. (run time: unknown)
itay1 = {
    "Size": (10, 10),
    "Walls": {
        (1, 1), (1, 2), (1, 3), (1, 5), (1, 6), (1, 7),
        (3, 1), (3, 7),
        (5, 2), (5, 3), (5, 4), (5, 5), (5, 6),
        (7, 4)
    },
    "Taps": {(0, 0): 100},
    "Plants": {(9, 9): 10},
    "Robots": {
        1: (0, 9, 0, 2), 
        2: (9, 0, 0, 2),
        3: (4, 4, 0, 2)
    }
}

# optimal solution length: 620
# Best run time achieved: 0.11646419763565064 seconds
itay2 = {
    "Size": (12, 12),
    "Walls": {
        (r, 6) for r in range(0, 10)
    },
    "Taps": {(0, 0): 100},
    "Plants": {(0, 11): 10},
    "Robots": {10: (11, 0, 0, 1)}
}

# optimal solution length: 159
# Best run time achieved: 0.0471789042154948 seconds
itay3 = {
    "Size": (16, 16),
    "Walls": {
        *[(5, c) for c in range(5, 12)],
        *[(r, 5) for r in range(5, 15)],
        *[(r, 11) for r in range(5, 15)],
    },
    "Taps": {(0, 8): 100},
    "Plants": {(10, 8): 5},
    "Robots": {10: (0, 0, 0, 2)}
}

# Cannot be solved in a reasonable time using the A* or GBFS method:
# If there is a solution in GBFS, it probably includes an illegal step.
# If there is a solution in A*, it is not the optimal solution. (run time: unknown)
itay4 = {
    "Size": (20, 20),
    "Walls": {
        *[(r, 10) for r in range(20) if r != 5 and r != 15],
        *[(10, c) for c in range(20) if c != 5 and c != 15],
    },
    "Taps": {(2, 2): 100},
    "Plants": {(18, 18): 6},
    "Robots": {
        10: (2, 18, 0, 2),
        11: (18, 2, 0, 2)
    }
}

# optimal solution length: 82
# Best run time achieved: 0.01587849855422974 seconds
itay5 = {
    "Size": (25, 25),
    "Walls": {
        (r, c) 
        for r in range(1, 24, 2) 
        for c in range(1, 24, 2)
    } | {
        (r, c+1) for r in range(1, 24, 4) for c in range(1, 23, 4)
    } | {
        (r+1, c) for r in range(1, 23, 4) for c in range(1, 24, 4)
    },
    "Taps": {(0, 0): 100},
    "Plants": {(24, 24): 5},
    "Robots": {10: (12, 12, 0, 5)}
}

# Solvable in reasonable time using GBFS. Optimal running time for GBFS only: 1.50040117899573 seconds
# Not solvable in reasonable time in A*, if there is a solution in A* it is not the optimal solution.
itay6 = {
    "Size": (10, 10),
    "Walls": {
        (3, 3), (3, 6), 
        (6, 3), (6, 6)
    },
    "Taps": {(5, 5): 100},
    "Plants": {
        (0, 0): 1, (0, 9): 1, 
        (9, 0): 1, (9, 9): 1
    },
    "Robots": {
        1: (4, 5, 0, 2),
        2: (5, 4, 0, 2),
        3: (5, 6, 0, 2)
    }
}

# optimal solution length: 700
# Best run time achieved: 0.5729579329490662 seconds
itay7 = {
    "Size": (50, 50),
    "Walls": {
        (r, 25) for r in range(0, 40)
    },
    "Taps": {(0, 0): 100},
    "Plants": {(0, 49): 3},
    "Robots": {10: (49, 0, 0, 1)}
}

# Cannot be solved in a reasonable time using the A* or GBFS method:
# If there is a solution in GBFS, it probably includes an illegal step.
# If there is a solution in A*, it is not the optimal solution. (run time: unknown)
itay8 = {
    "Size": (10, 10),
    "Walls": (
        set((r, 2) for r in range(2, 9)) |
        set((2, c) for c in range(2, 9))
    ),
    "Taps": {(0, 0): 10},
    "Plants": {(9, 9): 9},
    "Robots": {3: (1, 1, 0, 1), 4: (0, 1, 0, 1), 5: (1, 0, 0, 1)}, 
}

# optimal solution length: no solution
# Best run time achieved: 0.707038164138794 seconds
itay9 = {
    "Size": (8, 8),
    "Walls": (
        set((r, 3) for r in range(1, 7)) |
        set((4, c) for c in range(1, 7))
    ),
    "Taps": {(0, 7): 5},
    "Plants": {(7, 0): 4},
    "Robots": {1: (0, 0, 1, 0), 2: (7, 7, -1, 0)},
}

# optimal solution length: no solution
# Best run time achieved: 0.01062673330307 seconds
itay10 = {
    "Size": (12, 6),
    "Walls": (
        set((r, 1) for r in range(0, 12)) |
        set((r, 4) for r in range(0, 12))
    ),
    "Taps": {(11, 2): 7, (0, 3): 6},
    "Plants": {(5, 2): 3, (6, 3): 2},
    "Robots": {7: (3, 0, 0, 1), 8: (3, 5, 0, 1)},
}

# Cannot be solved in a reasonable time using the A* or GBFS method:
# If there is a solution in GBFS, it probably includes an illegal step.
# If there is a solution in A*, it is not the optimal solution. (run time: unknown)
itay11 = {
    "Size": (13, 13),
    "Walls": (
        set((1, c) for c in range(1, 11)) |
        set((10, c) for c in range(2, 11)) |
        set((r, 1) for r in range(2, 11)) |
        set((r, 10) for r in range(1, 10)) |
        set((3, c) for c in range(3, 9)) |
        set((8, c) for c in range(5, 7)) |
        set((r, 3) for r in range(4, 6)) |
        set((r, 8) for r in range(4, 6))
    ),
    "Taps": {(0, 0): 9, (12, 12): 8},
    "Plants": {(6, 6): 13},
    "Robots": {
        1: (5, 5, 0, 1),
        2: (7, 7, 0, 1),
        3: (6, 4, 0, 2),
        4: (6, 8, 0, 1)
    }
}

# Cannot be solved in a reasonable time using the A* or GBFS method:
# If there is a solution in GBFS, it probably includes an illegal step.
# If there is a solution in A*, it is not the optimal solution. (run time: unknown)
itay12 = {
    "Size": (15, 15),
    "Walls": {
        *[(r, 7) for r in range(15) if r not in (3, 11)], 
        *[(r, 3) for r in range(15) if r not in (5, 9)]
    },
    "Taps": {
        (0, 0): 12,
        (14, 14): 15
    },
    "Plants": {
        (7, 7): 10,
        (0, 14): 5,
        (14, 0): 8
    },
    "Robots": {
        1: (2, 2, 0, 3),
        2: (12, 12, 0, 2)
    }
}

# optimal solution length: 88
# Best run time achieved: 0.4599889119466146 seconds
itay13 = {
    "Size": (9, 9),
    "Walls": {
        (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6), (1, 7), (1, 8),
        (3, 0), (3, 1), (3, 2), (3, 3), (3, 4), (3, 5), (3, 6), (3, 7),
        (5, 1), (5, 2), (5, 3), (5, 4), (5, 6), (5, 7),
        (7, 1), (7, 2), (7, 3), (7, 4), (7, 5), (7, 6), (7, 7),
        (6, 1), (6, 7), (4, 7)
    },
    "Taps": {(0, 7): 9}, 
    "Plants": {(6, 2): 3},
    "Robots": {1: (0, 2, 0, 1), 2: (0, 6, 0, 3)},
}

# optimal solution length: no solution
# Best run time achieved: 0.000223477681478 seconds
itay14 = {
    "Size": (3, 3),
    "Walls": {
        (1, 1), (0, 2), (2, 0)
    },
    "Taps": {(1, 2): 7, (2, 1): 6}, 
    "Plants": {(2, 2): 10},
    "Robots": {13: (0, 0, 0, 3), 22: (0, 1, 0, 4)},
}

# optimal solution length: 880
# Best run time achieved: 20.80921936035156 seconds
itay15 = {
    "Size": (400, 400),
    "Walls": {
        (398, 398), (397, 397), (396, 396), (395, 395), (384, 384),
        (394, 394), (393, 393), (392, 392), (391, 391), (390, 390),
        (389, 389), (388, 388), (387, 387), (386, 386), (385, 385), 
        (383, 383), (382, 382), (381, 381), (380, 380), (379, 379),
        (378, 378), (377, 377), (376, 376), (375, 375), (374, 374),
        (373, 373), (372, 372), (371, 371), (370, 370), (369, 369),
        (368, 368), (367, 367), (366, 366), (365, 365), (364, 364),
        (363, 363), (362, 362), (361, 361), (360, 360), (359, 359),
    },
    "Taps": {(399, 398): 20, (398, 399): 20}, 
    "Plants": {(399, 399): 40},
    "Robots": {15: (0, 0, 0, 20)},
}

# Solvable in reasonable time using GBFS. Optimal running time for GBFS only: 18.3891539573675 seconds
# Not solvable in reasonable time in A*, if there is a solution in A* it is not the optimal solution.
itay16 = {
    "Size": (20, 20),
    "Walls": {
        (18, 18), (17, 17), (16, 16), (15, 15), (14, 14),
        (13, 13), (12, 12), (11, 11), (10, 10), (9, 9),
        (8, 8), (7, 7), (6, 6), (5, 5)
    },
    "Taps": {(19, 18): 20, (18, 19): 20}, 
    "Plants": {(19, 19): 40},
    "Robots": {15: (0, 0, 0, 20), 31: (0, 5, 0, 3), 74: (3, 2, 0, 2)},
}

# No solution exists
custom = {
    "Size":  (6, 6),
    "Walls": {(1, 1), (1, 2), (1, 3), (3, 1), (3, 2), (3, 3)},
    "Taps": {(0, 0): 10, (5, 5): 10},
    "Plants": {(0, 5): 5, (5, 0): 5},
    "Robots": {10: (2, 0, 0, 0), 11: (3, 5, 0, 0)},
}

# No solution exists
blocked1 = {
    "Size":  (3, 3),
    "Walls": {(1, 0), (1, 1), (1, 2)},
    "Taps": {(0, 0): 5},
    "Plants": {(2, 2): 5},
    "Robots": {10: (0, 1, 0, 5)},
}

# No solution exists
blocked2 = {
    "Size":  (1, 10),
    "Walls": set(),
    "Taps": {(0, 0): 5},
    "Plants": {(0, 9): 5},
    "Robots": {10: (0, 1, 0, 5), 11: (0, 8, 0, 5)},
}

# No solution exists
blocked3 = {
    "Size":  (10, 10),
    "Walls": {(r, 5) for r in range(0, 10)},
    "Taps": {(0, 0): 5},
    "Plants": {(0, 9): 5},
    "Robots": {10: (1, 0, 0, 5), 11: (8, 8, 0, 5)},
}

# Optimal: 1330
tunnels = {
    "Size": (14, 18),
    "Walls": {(r, 4*c + 1) for r in range(0, 13) for c in range(0, 5)} | \
             {(r, 4*c - 1) for r in range(1, 14) for c in range(1, 5)},
    "Taps": {(0, 0): 5},
    "Plants": {(12, 16): 5},
    "Robots": {10: (1, 0, 0, 1), 11: (2, 0, 0, 1)},
}

# No solution exists
tunnels_nosol = {
    "Size": (14, 18),
    "Walls": {(r, 4*c + 1) for r in range(0, 13) for c in range(0, 4)} | \
             {(r, 17) for r in range(0, 14)} | \
             {(r, 4*c - 1) for r in range(1, 14) for c in range(1, 5)},
    "Taps": {(0, 0): 5},
    "Plants": {(13, 16): 5},
    "Robots": {10: (1, 0, 0, 1), 11: (2, 0, 0, 1)},
}

plants_check = {
    "Size": (5, 5),
    "Walls": set(),
    "Taps": {},
    "Plants": {(0, 0): 5, (0, 2): 5, (2, 0): 30},
    "Robots": {10: (0, 0, 30, 30), 11: (0, 2, 10, 10)},
}

tap_fail = {'Size': (6, 6), 'Walls': {(2, 4), (1, 1), (2, 0), (2, 3), (3, 3), (5, 3)}, 'Taps': {(1, 2): 12}, 'Plants': {(0, 5): 2, (4, 0): 1}, 'Robots': {10: (3, 1, 0, 3), 11: (5, 2, 0, 2), 12: (1, 5, 0, 1)}}

# ==========================================
# TEST SUITE CONFIGURATION
# ==========================================

# 1. STANDARD SUITE (Run on both A* and GBFS)
TEST_SUITE = [
    ("Problem 1", problem1, 8),
    ("Problem 2", problem2, 20),
    ("Problem 3", problem3, 28),
    ("Problem 4", problem4, 13),
    ("Problem 5 (Deadend)", problem5_deadend, None),
    ("Problem 6", problem6, 8),
    ("Problem 7", problem7, 21),
    ("Hard 1", problem_hard1, 31),
    ("Hard 2", problem_hard2, 24),
    ("Hard 3", problem_hard3, 24),
    ("Hard 4", problem_hard4, 25),
    ("Hard 5", problem_hard5, 29),
    ("Hard 6", problem_hard6, 33),
    ("Load Test", problem_load, 65),
    ("10x10 Single", problem_10x10_single, 106),
    ("12x12 Snake", problem_12x12_snake, 249),
    ("12x12 Snake Hard", problem_12x12_snake_hard, 343),
    ("itay2", itay2, 620),
    ("itay3", itay3, 158),
    ("itay5", itay5, 82),
    ("itay7", itay7, 700),
    ("itay13", itay13, 88),
    ("itay15", itay15, 880),

    ("Custom No Solution", custom, None),
    ("Blocked Test 1", blocked1, None),
    ("Blocked Test 2", blocked2, None),
    ("Blocked Test 3", blocked3, None),
    ("Tunnels", tunnels, 1330),
    ("Tunnels No Solution", tunnels_nosol, None),
    ("Plants Check", plants_check, 44),
    ("Tap Fail Check", tap_fail, 21),
]

# 2. GBFS ONLY SUITE (Stress tests, difficult for A*)
# Use '?' for expected cost if we just want to ensure it finds a solution.
GBFS_SUITE = [
    ("itay1", itay1, '?'),
    ("itay4", itay4, '?'),
    ("itay6", itay6, '?'),
    ("itay16", itay16, '?'),
]

def main():
    overall_start = time.time()
    failed_tests = []

    print(f"{'TEST NAME':<25} | {'ALGO':<6} | {'TIME':<8} | {'RESULT':<10} | {'EXPECTED':<10} | {'STATUS'}")
    print("-" * 85)

    # --- RUN STANDARD SUITE ---
    for name, problem_data, expected in TEST_SUITE:
        for algo in ['astar', 'gbfs']:
            
            start = time.time()
            result = solve_problems(problem_data, algo)
            duration = time.time() - start
            
            status, status_color = evaluate_result(result, expected, algo)
            
            if "FAIL" in status:
                failed_tests.append(f"{name} [{algo}] -> Result: {result}, Expected: {expected}")

            print(f"{name:<25} | {algo:<6} | {duration:>6.3f}s  | {str(result):<10} | {str(expected):<10} | {status_color}{status}{RESET}")

    # --- RUN GBFS ONLY SUITE ---
    print("\n" + "="*30 + " GBFS STRESS TESTS " + "="*30)
    for name, problem_data, expected in GBFS_SUITE:
        algo = 'gbfs'
        start = time.time()
        result = solve_problems(problem_data, algo)
        duration = time.time() - start

        status, status_color = evaluate_result(result, expected, algo)

        if "FAIL" in status:
            failed_tests.append(f"{name} [{algo}] -> Result: {result}, Expected: {expected}")

        print(f"{name:<25} | {algo:<6} | {duration:>6.3f}s  | {str(result):<10} | {str(expected):<10} | {status_color}{status}{RESET}")

    overall_end = time.time()
    
    print("-" * 85)
    print(f"Total Time: {overall_end - overall_start:.4f} seconds.")
    
    if failed_tests:
        print(f"\n{RED}=== FAILED TESTS ==={RESET}")
        for f in failed_tests:
            print(f"- {f}")
    else:
        print(f"\n{GREEN}=== ALL TESTS PASSED ==={RESET}")

def evaluate_result(result, expected, algo):
    if expected is None:
        # Expecting NO Solution
        if result is None:
            return "PASS", GREEN
        else:
            return "FAIL (Should be None)", RED
    
    elif expected == '?':
        # Don't care about cost, just want a solution
        if result is not None:
             return f"PASS ({result})", GREEN
        else:
             return "FAIL (No Sol)", RED

    else:
        # Expecting Valid Number
        if result is None:
            return "FAIL (No Sol)", RED
        else:
            if algo == 'astar':
                if result == expected:
                    return "PASS", GREEN
                elif result > expected:
                    return f"FAIL (+{result-expected})", RED
                else:
                    return f"BETTER? ({result})", YELLOW
            else:
                # GBFS
                diff = result - expected
                if diff == 0:
                    return "OPTIMAL", GREEN
                else:
                    return f"+{diff} (Subopt)", YELLOW

if __name__ == '__main__':
    main()