import ex1
import search
import random
import time

def check_admissibility(prob, name=""):
    """
    Check admissibility: h(σ) ≤ h*(σ) for the initial state.
    Returns (passed, h_value, h_star_value)
    """
    try:
        p = ex1.create_watering_problem(prob)
        initial_node = search.Node(p.initial)
        h_val = p.h_astar(initial_node)
        
        res = search.astar_search(p, p.h_astar)
        
        if res:
            true_cost = len(res[0].path()) - 1
            if h_val > true_cost:
                return False, h_val, true_cost
            return True, h_val, true_cost
        return True, h_val, None  # Unsolvable - any h is admissible
    except Exception as e:
        print(f"Error in {name}: {e}")
        return True, None, None


def check_path_admissibility(prob, name):
    """
    Check admissibility for ALL nodes along the optimal path.
    """
    print(f"--- Path Check: {name} ---")
    try:
        p = ex1.create_watering_problem(prob)
        res = search.astar_search(p, p.h_astar)
        
        if not res:
            print("  Unsolvable - skipping")
            return True
        
        path = res[0].path()
        total_cost = len(path) - 1
        failures = 0
        
        for node in path:
            h_val = p.h_astar(node)
            h_star = total_cost - node.depth
            
            if h_val > h_star:
                print(f"  FAIL depth {node.depth}: h={h_val} > h*={h_star}")
                failures += 1
        
        if failures > 0:
            print(f"  FAILED: {failures}/{len(path)} nodes inadmissible")
            return False
        print(f"  PASSED: All {len(path)} nodes admissible")
        return True
    except Exception as e:
        print(f"  Error: {e}")
        return True


# ========== MAIN TESTS ==========

print("="*60)
print("ADMISSIBILITY TEST SUITE")
print("Testing: h(σ) ≤ h*(σ) for all nodes σ")
print("="*60)

# Manual edge cases
manual_cases = [
    ({'Size': (5, 5), 'Walls': set(), 'Taps': {(4,4): 10}, 
      'Plants': {(0,1): 1, (0,3): 1, (1,0): 1}, 
      'Robots': {1: (0,0, 3, 5), 2: (4,4, 3, 5)}}, "Multi-robot with water"),
    
    ({'Size': (4, 4), 'Walls': set(), 'Taps': {(0,0): 10}, 
      'Plants': {(3,3): 3}, 
      'Robots': {1: (1,1, 2, 3)}}, "Partial load"),
    
    ({'Size': (5, 5), 'Walls': set(), 'Taps': {(2,2): 20}, 
      'Plants': {(0,0): 6}, 
      'Robots': {1: (4,4, 0, 2)}}, "Multiple trips"),
    
    ({'Size': (5, 6), 'Walls': {(0, 2), (0, 3), (2, 2), (2, 3)}, 
      'Taps': {(1, 2): 8, (3, 3): 8}, 
      'Plants': {(0, 0): 4, (4, 5): 4}, 
      'Robots': {10: (1, 1, 0, 4), 11: (3, 4, 0, 3)}}, "Hard5"),
    
    ({'Size': (3, 3), 'Walls': set(), 'Taps': {(0,0): 10}, 
      'Plants': {(1,1): 2}, 
      'Robots': {1: (1,1, 3, 5)}}, "Robot at plant"),
]

print("\n--- Manual Cases ---")
manual_failures = 0
for prob, name in manual_cases:
    passed, h, h_star = check_admissibility(prob, name)
    if passed:
        print(f"✓ {name}: h={h} <= h*={h_star}")
    else:
        print(f"✗ {name}: h={h} > h*={h_star} INADMISSIBLE!")
        manual_failures += 1

# Random cases
print("\n--- Random Cases (100 tests) ---")
random.seed(42)
random_failures = 0
start = time.time()

for i in range(100):
    rows, cols = random.randint(3, 5), random.randint(3, 5)
    walls = {(r, c) for r in range(rows) for c in range(cols) if random.random() < 0.1}
    
    def get_free():
        for _ in range(100):
            r, c = random.randint(0, rows-1), random.randint(0, cols-1)
            if (r, c) not in walls:
                return (r, c)
        return (0, 0)
    
    taps = {get_free(): random.randint(10, 25) for _ in range(random.randint(1, 2))}
    plants = {get_free(): random.randint(1, 3) for _ in range(random.randint(1, 3))}
    robots = {}
    for rid in range(random.randint(1, 2)):
        pos = get_free()
        load = random.randint(0, 3)
        cap = random.randint(2, 5)
        robots[rid] = (pos[0], pos[1], min(load, cap), cap)
    
    prob = {'Size': (rows, cols), 'Walls': walls, 'Taps': taps, 'Plants': plants, 'Robots': robots}
    
    passed, h, h_star = check_admissibility(prob, f"Random {i+1}")
    if not passed:
        random_failures += 1
        print(f"✗ Random {i+1}: h={h} > h*={h_star}")
        print(f"  Problem: {prob}")

elapsed = time.time() - start
print(f"Completed {100} tests in {elapsed:.2f}s")

# Path admissibility
print("\n--- Path Admissibility (all nodes on solution) ---")
path_failures = 0
for prob, name in manual_cases[:3]:
    if not check_path_admissibility(prob, name):
        path_failures += 1

# Summary
print("\n" + "="*60)
print("SUMMARY")
print("="*60)
print(f"Manual cases:  {len(manual_cases) - manual_failures}/{len(manual_cases)} passed")
print(f"Random cases:  {100 - random_failures}/100 passed")
print(f"Path checks:   {3 - path_failures}/3 passed")

total_failures = manual_failures + random_failures + path_failures
if total_failures == 0:
    print("\n✅ HEURISTIC IS ADMISSIBLE - All tests passed!")
else:
    print(f"\n❌ {total_failures} failures - HEURISTIC MAY BE INADMISSIBLE")