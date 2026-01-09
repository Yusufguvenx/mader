# MADER MATLAB Examples

Standalone MATLAB implementations of the core algorithms from the MADER trajectory planner. These examples are designed for learning and prototyping without requiring ROS or Gurobi.

## Reference

Jesus Tordesillas, et al. "MADER: Trajectory Planner in Multi-Agent and Dynamic Environments" IEEE Transactions on Robotics, 2021.

## Examples Overview

| Example | File | Description |
|---------|------|-------------|
| 1 | `example1_bspline_basics.m` | B-spline trajectory representation |
| 2 | `example2_trajectory_optimization.m` | Trajectory optimization with constraints |
| 3 | `example3_full_planner.m` | Full planner with obstacles |
| 4 | `example4_minvo_basics.m` | MINVO basis for tighter collision bounds |

---

## Example 1: B-Spline Basics

**File:** `example1_bspline_basics.m`

Demonstrates the B-spline trajectory representation used in MADER:
- Uniform cubic B-spline basis matrix
- Control points to polynomial coefficients conversion
- Trajectory evaluation (position, velocity, acceleration)
- Visualization

**Key concepts:**
- `M_bspline` - The 4x4 basis matrix for uniform cubic B-splines
- `PieceWisePol` structure for trajectory storage
- Evaluation using normalized parameter `u ∈ [0,1]`

---

## Example 2: Trajectory Optimization

**File:** `example2_trajectory_optimization.m`

Demonstrates trajectory optimization using MATLAB's `fmincon`:

**Key Features:**
- **LOCAL PLANNING**: Uses planning radius Ra to compute intermediate goals
- Cost function: minimize jerk² + terminal cost
- Constraints: velocity, acceleration, jerk limits
- Sphere constraint: trajectory stays within Ra of start

**Important Concept - Planning Horizon Ra:**
```
If goal is OUTSIDE Ra:
    → Compute intermediate goal G on boundary of Ra sphere
    → Plan from start to G (not directly to final goal)

If goal is WITHIN Ra:
    → Plan directly to goal
```

This is how MADER handles long-distance goals with local planning.

---

## Example 3: Full Planner with Obstacles

**File:** `example3_full_planner.m`

Complete planning pipeline with collision avoidance. Includes **extensive documentation** comparing the MATLAB implementation with the ROS/C++ version.

**Pipeline:**
1. A* search for initial guess (simplified OctopusSearch)
2. Separating plane computation
3. Trajectory optimization with collision constraints
4. Collision verification and visualization

**Key Differences from ROS Version (documented in file):**

| Aspect | ROS (C++) | MATLAB |
|--------|-----------|--------|
| Execution | Real-time at ~100 Hz | Single batch execution |
| Optimizer | Gurobi QCQP | fmincon SQP |
| A* Search | Full OctopusSearch with LP | Simplified greedy search |
| Collision | MINVO basis + LP separators | B-spline basis + geometric planes |
| Obstacles | Static + dynamic | Static only |
| Performance | ~10-50 ms | ~100-1000+ ms |

---

## Example 4: MINVO Basis Fundamentals

**File:** `example4_minvo_basics.m`

Demonstrates the MINVO (Minimum Volume) basis - a key innovation in MADER.

**Why MINVO Matters:**
```
When checking if a trajectory collides with an obstacle:
  1. We bound the trajectory curve with a convex hull of control points
  2. If hull doesn't intersect obstacle → trajectory is safe

Problem: B-spline/Bezier control points give LARGE convex hulls
         → Many FALSE POSITIVES (trajectory seems to collide but doesn't)

Solution: MINVO control points give SMALLEST possible convex hull
         → FEWER false positives → Navigate tighter spaces
```

**What the example shows:**
- Transformation matrices: B-spline → MINVO, B-spline → Bezier
- Same polynomial curve, different control point positions
- Convex hull volume comparison (MINVO is smallest)
- Collision checking advantage demonstration
- Multi-segment trajectory example

---

## Requirements

- MATLAB R2019b or later
- Optimization Toolbox (for `fmincon` in Examples 2, 3)

## Running the Examples

```matlab
cd matlab_examples

% Start with basics
run('example1_bspline_basics.m')

% Then optimization
run('example2_trajectory_optimization.m')

% Full planner with obstacles
run('example3_full_planner.m')

% MINVO basis understanding
run('example4_minvo_basics.m')
```

## Recommended Learning Path

1. **Example 1** - Understand B-spline trajectory representation
2. **Example 4** - Learn why MINVO basis matters for collision checking
3. **Example 2** - See how optimization works with constraints
4. **Example 3** - Put it all together with obstacles

## Key Differences from C++ Implementation

| Aspect | C++ (Original) | MATLAB (Converted) |
|--------|----------------|-------------------|
| Optimizer | Gurobi (QCQP) | fmincon (SQP) |
| Geometry | CGAL library | Built-in convhull |
| A* Search | Full OctopusSearch | Simplified greedy |
| ROS | Full integration | Standalone scripts |
| Real-time | ~100 Hz replanning | Batch processing |
| Performance | 10-50 ms | 100-1000+ ms |

## Extending the Examples

**Dynamic obstacles:**
- Extend Example 3 to sample obstacle positions at different times
- Create convex hulls that encompass obstacle motion

**Multi-agent:**
- Add trajectory constraints from other agents
- Implement the "check-recheck" protocol from the paper

**MINVO for collision:**
- Modify Example 3 to use MINVO control points for collision checking
- Use transformation matrices from Example 4

**Different bases:**
- Implement Bezier basis option (matrices in `mader_types.hpp`)
- Compare performance with B-spline and MINVO

## File Structure

```
matlab_examples/
├── example1_bspline_basics.m       # B-spline fundamentals
├── example2_trajectory_optimization.m  # Optimization with fmincon
├── example3_full_planner.m         # Full pipeline + ROS comparison
├── example4_minvo_basics.m         # MINVO basis explanation
└── README.md                       # This file
```

## Troubleshooting

**"No feasible point found" in Example 2 or 3:**
- Try increasing Ra (planning radius)
- Reduce v_max or a_max constraints
- Check that goal is reachable within Ra

**Slow optimization:**
- Reduce num_pol (fewer segments)
- Increase OptimalityTolerance
- Use 'interior-point' algorithm instead of 'sqp'

**MATLAB out of memory:**
- Reduce t_final/dc ratio (fewer sample points)
- Clear workspace between runs
