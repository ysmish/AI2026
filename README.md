# AI2026 - AI Coursework Repository

## Overview
This repository contains AI coursework focused on **search algorithms** and **planning problems**. The assignments involve solving complex multi-robot coordination and watering tasks using classical AI techniques like A* search with heuristics.

---

## **Exercise 1: Watering Problem with Graph Search**

### Problem Statement
The task is to coordinate multiple robots to water plants by:
- Moving robots on a grid to **tap locations** to fill up with water
- Transporting water to **plant locations** and pouring it
- Optimizing the path to minimize total steps while satisfying all plants' water requirements

### Key Concepts & Implementation

#### **State Representation**
```
(robot_states, plant_states, tap_states, total_remaining)
```
- **robot_states**: Tuple of (robot_id, position, current_load) sorted by ID
- **plant_states**: Tuple of water satisfied at each plant
- **tap_states**: Remaining water available at each tap
- **total_remaining**: Total water units still needed globally

#### **Actions**
- **Movement**: UP, DOWN, LEFT, RIGHT (1 unit per action)
- **LOAD**: Pick up 1 unit of water at a tap
- **POUR**: Drop 1 unit of water at a plant

#### **Search Optimization Techniques**

1. **Highway Pruning** (for single robots)
   - Pre-calculates shortest paths between all Points of Interest (taps & plants)
   - Identifies "essential" cells that must be visited on optimal paths
   - Prunes unreachable cells to reduce search space

2. **BFS Pre-computation**
   - For each tap and plant location, pre-computes BFS distance maps
   - Dramatically speeds up heuristic calculation during search

3. **Deterministic State Ordering**
   - Sorts robot IDs, plant positions, and tap positions
   - Ensures consistent state representation and reduces duplicate states


---

## **Exercise 2: Stochastic Planning with Robot Reliability**

### Problem Statement
Extended version where robots have **probabilistic failures**:
- Each robot has a success probability (reliability) for each action
- Failed movements cause the robot to slip (move randomly or stay)
- Failed LOAD/POUR actions waste resources
- Goal: Coordinate unreliable robots to water plants within a time horizon

### Advanced AI Techniques

#### **State-Space Reduction Strategy**
The controller dynamically switches between two strategies based on board characteristics:

1. **WATER_ALL** (default for small boards)
   - All robots target all plants simultaneously
   - Used when board area ≤ 16 cells

2. **WATER_ONE** (activated for large boards or farming scenarios)
   - Focuses resources on a single highest-value plant
   - Triggered when:
     - Board area > 16 cells, OR
     - High reward disparity detected (max_reward > 2×min_reward + 2)
   - Reduces multi-robot coordination complexity

#### **Smart Robot Selection (Multi-Tiered)**
Selects a "lead agent" for complex operations using a priority key:
```python
max(robot_probs.keys(), key=lambda rid: (
    int(robot_probs[rid] / 0.05),    # Priority 1: Reliability bucket (5%)
    capacities.get(rid, 0),          # Priority 2: Water capacity
    robot_probs[rid]                 # Priority 3: Raw probability tie-break
))
```
- Groups robots into reliability buckets (5% increments)
- Prefers high-capacity robots when probabilities are similar
- Ensures the most competent, capable robot leads the mission

#### **Stochastic Drift Compensation ("GPS-style" Re-planning)**
Tracks expected vs. actual robot positions:
- After each action, stores expected position of robot
- If actual position ≠ expected position → **immediate plan recalculation**
- Compensates for random slips caused by failures

#### **Physical Occupancy & Blocker Guard**
Detects when unreliable robots obstruct critical paths:
- Treats blocking robots as **temporary walls** in the A* search
- Forces alternative route planning
- Automatically recalculates plan when blockage detected

#### **Reliability-Based Agent Filtering**
- Only considers robots with success probability ≥ **0.7** in complex modes
- Prevents low-reliability agents from wasting moves on high-risk tasks
- Filters out "stupid" robots to reduce plan congestion

#### **Proactive Strategic Reset**
Triggers a RESET (returning to initial state) when:
- Current robot is farther from nearest tap than starting position
- Effectively "teleports" robot to save steps during resource collection phase
- Only applied when task is complete and robot is out of position

#### **Admissible Tie-Breaking**
Uses fractional Manhattan distance in A* cost function:
```
f(n) = g(n) + h(n) + distance/100.0
```
- Provides directionality to prevent oscillation in corridors
- Remains admissible (doesn't overestimate true cost)

### Refined Heuristic for Stochastic Environment
Enhanced version of Exercise 1's heuristic accounting for:
- Robot reliability probabilities
- Partial load constraints
- Target plant selection in WATER_ONE mode
