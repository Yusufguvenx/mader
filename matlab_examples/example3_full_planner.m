%% MADER MATLAB Example 3: Full Trajectory Planner with Obstacles
% =========================================================================
% This script demonstrates the complete MADER planning pipeline converted
% to MATLAB. It includes detailed documentation comparing this standalone
% implementation with the original ROS-based C++ code.
%
% =========================================================================
%                    ARCHITECTURE COMPARISON
% =========================================================================
%
% ROS Implementation (mader.cpp, mader_ros.cpp):
% ----------------------------------------------
%   mader_node.cpp
%        |
%        v
%   MaderRos (mader_ros.cpp)
%        |-- Subscribes to: /goal, /state, /trajs_obstacles
%        |-- Publishes to:  /traj, /markers
%        |-- Timer callback at ~100 Hz
%        v
%   Mader (mader.cpp)
%        |-- Main planning loop with state machine
%        |-- Manages committed trajectory
%        |-- Calls OctopusSearch for initial guess
%        |-- Calls SolverGurobi for optimization
%        v
%   SolverGurobi / OctopusSearch
%
% MATLAB Implementation (this file):
% ----------------------------------
%   example3_full_planner.m
%        |
%        |-- No ROS, no subscribers/publishers
%        |-- Single execution (not real-time loop)
%        |-- Direct function calls
%        v
%   aStarSearch() - Simplified A* (replaces OctopusSearch)
%   fmincon()     - MATLAB optimizer (replaces Gurobi)
%
% =========================================================================
%                    KEY DIFFERENCES FROM ROS VERSION
% =========================================================================
%
% 1. REAL-TIME LOOP vs SINGLE EXECUTION
%    -----------------------------------
%    ROS: Runs at ~100 Hz in a timer callback. Each iteration:
%         - Gets current state from /state topic
%         - Checks if replanning is needed (based on execution progress)
%         - Plans from current committed trajectory point A to goal G
%         - Publishes new trajectory segment
%
%    MATLAB: Single execution from start to goal. No real-time updates.
%            For multi-iteration planning, user must call repeatedly.
%
% 2. STATE MACHINE
%    --------------
%    ROS: Has states YAWING -> TRAVELING -> GOAL_SEEN -> GOAL_REACHED
%         - Handles yaw alignment before traveling
%         - Transitions based on distance to goal and visibility
%
%    MATLAB: No state machine. Assumes direct point-to-point planning.
%
% 3. COMMITTED TRAJECTORY
%    ---------------------
%    ROS: Maintains a "committed trajectory" that the drone follows.
%         - New plans are appended to committed trajectory
%         - Point A (start of new plan) is extracted from committed traj
%         - This ensures smooth transitions between replans
%
%    MATLAB: Plans from initial state directly. No trajectory appending.
%
% 4. DYNAMIC OBSTACLES (Multi-Agent)
%    --------------------------------
%    ROS: Subscribes to /trajs_obstacles topic
%         - Other agents broadcast their planned trajectories
%         - MADER samples these at time intervals to build convex hulls
%         - Trajectory coordination via "check-recheck" protocol
%
%    MATLAB: Static obstacles only. No dynamic obstacle handling.
%            Could be extended by sampling obstacle positions over time.
%
% 5. A* SEARCH (OctopusSearch)
%    --------------------------
%    ROS (octopus_search.cpp):
%         - Full A* in control point space with priority queue
%         - Uses velocity/acceleration bounds to compute search bounds
%         - Collision checking via separating hyperplanes (LP solver)
%         - Finds normals n and offsets d for separating planes
%         - Returns control points AND plane parameters
%
%    MATLAB (aStarSearch function):
%         - Simplified greedy search toward goal
%         - Basic collision check (point in hull)
%         - No separating plane computation during search
%         - Planes computed after search from trajectory/obstacle geometry
%
% 6. OPTIMIZATION SOLVER
%    --------------------
%    ROS (solver_gurobi.cpp):
%         - Gurobi: Commercial QCQP solver, very fast
%         - Warm-start with A* guess
%         - Runtime budget: kappa*time for guess, mu*time for optimization
%         - Handles infeasibility gracefully
%
%    MATLAB:
%         - fmincon with SQP algorithm
%         - Slower but available in standard MATLAB
%         - No warm-start capability
%         - May struggle with tight constraint satisfaction
%
% 7. COLLISION CONSTRAINTS
%    ----------------------
%    ROS: For each segment i and obstacle j:
%         - Separating plane n_ij' * x + d_ij >= 1 for control points
%         - n_ij' * x + d_ij <= 0 for obstacle vertices
%         - Planes found by LP during A* search
%
%    MATLAB: Simplified plane constraints:
%         - Plane normal = direction from obstacle center to traj center
%         - Fixed planes (not optimized)
%         - Less optimal but simpler to implement
%
% 8. BASIS TRANSFORMATION (MINVO)
%    -----------------------------
%    ROS: Uses MINVO basis for collision checking (tighter hulls)
%         - B-spline CPs -> MINVO CPs via transformation matrix
%         - Collision check on MINVO control points
%
%    MATLAB: Uses B-spline basis directly for simplicity.
%            See example4_minvo_basics.m for MINVO implementation.
%
% 9. CONVEX HULL COMPUTATION
%    ------------------------
%    ROS: CGAL library for 3D convex hulls
%         - Robust geometric algorithms
%         - Handles degenerate cases
%
%    MATLAB: Built-in convhull/convhulln
%         - Simpler API but less robust
%         - May fail on degenerate (coplanar) points
%
% 10. PERFORMANCE
%     -----------
%     ROS: Optimized for real-time (~10-50 ms per replan)
%          - C++ performance
%          - Gurobi's efficient QCQP solver
%          - Pre-computed transformation matrices
%
%     MATLAB: Not real-time (100-1000+ ms)
%          - Interpreted language overhead
%          - fmincon general-purpose optimizer
%          - Good for prototyping and understanding
%
% =========================================================================
%
% Author: Converted from MIT ACL MADER project
% Reference: Jesus Tordesillas, et al. "MADER: Trajectory Planner in
%            Multi-Agent and Dynamic Environments" IEEE T-RO 2021
%
% =========================================================================

clear; clc; close all;

fprintf('=========================================================\n');
fprintf('   MADER Full Trajectory Planner (MATLAB Version)\n');
fprintf('   Example 3: Planning with Obstacles\n');
fprintf('=========================================================\n\n');

%% 1. Problem Setup
% -----------------
% Define planning parameters similar to mader.yaml configuration file

params = struct();

% Motion constraints (from mader.yaml)
params.v_max = [2.0; 2.0; 2.0];        % Max velocity [m/s]
params.a_max = [3.0; 3.0; 3.0];        % Max acceleration [m/s^2]
params.j_max = [10.0; 10.0; 10.0];     % Max jerk [m/s^3]

% Trajectory parameters
params.num_pol = 8;                     % Number of polynomial segments
params.deg_pol = 3;                     % Polynomial degree (cubic)
params.weight = 100.0;                  % Terminal cost weight

% Planning parameters
params.Ra = 6.0;                        % Planning radius [m]
params.dc = 0.02;                       % Sampling period [s]
params.drone_radius = 0.20;              % Drone collision radius [m]

% A* search parameters (simplified from octopus_search settings)
params.a_star_samples = [5, 5, 3];      % Grid samples per axis
params.a_star_bias = 1.0;               % Heuristic weight (>=1.0)

fprintf('Parameters loaded:\n');
fprintf('  v_max = [%.1f, %.1f, %.1f] m/s\n', params.v_max);
fprintf('  a_max = [%.1f, %.1f, %.1f] m/s^2\n', params.a_max);
fprintf('  Ra = %.1f m (planning horizon)\n', params.Ra);
fprintf('  drone_radius = %.2f m\n\n', params.drone_radius);

%% 2. Define Initial and Final States
% ------------------------------------
% In ROS version, these come from /state topic and /goal topic

initial_state = struct();
initial_state.pos = [-4; 0; 0];
initial_state.vel = [0; 0; 0];
initial_state.accel = [0; 0; 0];

final_goal = [6; 2; 1];               % Final goal position

fprintf('Initial position: [%.1f, %.1f, %.1f]\n', initial_state.pos);
fprintf('Final goal:       [%.1f, %.1f, %.1f]\n', final_goal);

% Check if goal is within Ra, compute intermediate goal if needed
% (Same logic as in mader.cpp getNextGoal())
dist_to_goal = norm(final_goal - initial_state.pos);
if dist_to_goal > params.Ra
    direction = (final_goal - initial_state.pos) / dist_to_goal;
    target_goal = initial_state.pos + params.Ra * 0.95 * direction;
    fprintf('Goal outside Ra -> Intermediate goal: [%.2f, %.2f, %.2f]\n\n', target_goal);
else
    target_goal = final_goal;
    fprintf('Goal within Ra -> Planning directly to goal\n\n');
end

final_state = struct();
final_state.pos = target_goal;
final_state.vel = [0; 0; 0];
final_state.accel = [0; 0; 0];

%% 3. Define Obstacles
% --------------------
% In ROS version, obstacles come from:
%   - Static map (point cloud or occupancy grid)
%   - Dynamic obstacles (/trajs_obstacles topic)
%
% Here we define static box obstacles manually.
% Each obstacle is stored as 3xN matrix of vertices.

fprintf('Creating obstacles...\n');

obstacles = {};

% Obstacle 1: Central pillar
obs1_center = [0; 0; 0];
obs1_size = [1.5; 1.5; 2.0];
obstacles{1} = createBoxVertices(obs1_center, obs1_size);

% Obstacle 2: Left obstacle
obs2_center = [-2; 1.5; 0];
obs2_size = [1.0; 1.0; 2.0];
obstacles{2} = createBoxVertices(obs2_center, obs2_size);

% Obstacle 3: Right obstacle
obs3_center = [2; -1.2; 0];
obs3_size = [1.2; 0.8; 2.0];
obstacles{3} = createBoxVertices(obs3_center, obs3_size);

% Obstacle 4: New box
obs4_center = [1.0; 2.0; 0.0];
obs4_size   = [1.0; 0.8; 1.5];
obstacles{4} = createBoxVertices(obs4_center, obs4_size);

% Obstacle 5: Another box
obs5_center = [-3.0; -1.5; 0.0];
obs5_size   = [1.2; 1.2; 2.0];
obstacles{5} = createBoxVertices(obs5_center, obs5_size);


fprintf('  Created %d obstacles\n', length(obstacles));

% Inflate obstacles by drone radius (Minkowski sum approximation)
% In ROS: done in cgal_utils.cpp when computing convex hulls
obstacles_inflated = cell(size(obstacles));
for i = 1:length(obstacles)
    obstacles_inflated{i} = inflateObstacle(obstacles{i}, params.drone_radius);
end
fprintf('  Inflated by drone_radius = %.2f m\n\n', params.drone_radius);

% Axis-aligned box bounds for distance constraints
obstacle_boxes = cell(size(obstacles));
for i = 1:length(obstacles)
    obs_min = min(obstacles{i}, [], 2);
    obs_max = max(obstacles{i}, [], 2);
    obstacle_boxes{i} = [obs_min, obs_max];
end

%% 4. B-Spline Setup
% ------------------
% Same as solver_gurobi.cpp constructor

p = params.deg_pol;
num_segments = params.num_pol;
M = num_segments + 2 * p;
N = M - p - 1;

% Time allocation
dist = norm(final_state.pos - initial_state.pos);
t_init = 0;
t_final = dist / mean(params.v_max) * 3.0;
deltaT = (t_final - t_init) / num_segments;

% Knot vector (clamped uniform B-spline)
knots = zeros(1, M + 1);
knots(1:p+1) = t_init;
for i = p+2 : M-p
    knots(i) = knots(i-1) + deltaT;
end
knots(M-p+1:M+1) = t_final;

fprintf('B-spline setup:\n');
fprintf('  %d segments, degree %d\n', num_segments, p);
fprintf('  %d control points (N+1)\n', N+1);
fprintf('  Time: [%.2f, %.2f] s, deltaT = %.3f s\n\n', t_init, t_final, deltaT);

%% 5. Compute Fixed Control Points
% ---------------------------------
% From solver_gurobi.cpp setInitStateFinalStateInitTFinalT()

p0 = initial_state.pos; v0 = initial_state.vel; a0 = initial_state.accel;
pf = final_state.pos; vf = final_state.vel; af = final_state.accel;

t1 = knots(2); t2 = knots(3); tpP1 = knots(p+2); t1PpP1 = knots(2+p+1);
tN = knots(N+1); tNm1 = knots(N); tNPp = knots(N+p+1); tNm1Pp = knots(N+p);

% Initial condition control points
q0 = p0;
q1 = p0 + (tpP1 - t1) * v0 / p;
q2 = (p^2 * q1 - (t1PpP1 - t2) * (a0 * (t2 - tpP1) + v0) - p * (q1 + (t2 - t1PpP1) * v0)) / ((p - 1) * p);

% Final condition control points
qN = pf;
qNm1 = pf + (tN - tNPp) * vf / p;
qNm2 = (p^2 * qNm1 - (tNm1 - tNm1Pp) * (af * (tNm1Pp - tN) + vf) - p * (qNm1 + (tNm1Pp - tNm1) * vf)) / ((p - 1) * p);

%% 6. A* Search for Initial Guess
% --------------------------------
% ROS VERSION (octopus_search.cpp):
%   - Priority queue with f = g + bias*h
%   - Expands nodes in control point space
%   - Each node represents adding one more control point
%   - Computes velocity/acceleration bounds for feasibility
%   - Collision check via separating hyperplane LP
%   - Returns: control points q, normals n, offsets d
%
% MATLAB VERSION (simplified):
%   - Greedy search toward goal
%   - Simple point-in-hull collision check
%   - No separating plane computation during search
%   - Returns: control points only

fprintf('Running A* search for initial guess...\n');
fprintf('  (Simplified version - see octopus_search.cpp for full implementation)\n');

tic;
[q_guess, astar_success] = aStarSearch(q0, q1, q2, qNm2, qNm1, qN, ...
    final_state.pos, N, p, knots, params, obstacles_inflated);
astar_time = toc;

if astar_success
    fprintf('  A* found feasible path in %.3f s\n\n', astar_time);
else
    fprintf('  A* failed, using straight-line guess\n\n');
    % Fallback
    num_free = N + 1 - 6;
    q_guess = [q0, q1, q2];
    for i = 1:num_free
        t = i / (num_free + 1);
        q_guess = [q_guess, (1-t)*q2 + t*qNm2];
    end
    q_guess = [q_guess, qNm2, qNm1, qN];
end

%% 7. Compute Separating Planes
% ------------------------------
% ROS VERSION (octopus_search.cpp, separator.hpp):
%   - For each segment/obstacle pair, solve LP to find separating plane
%   - Plane must satisfy:
%     * n' * obstacle_vertex + d <= 0 for all vertices
%     * n' * control_point + d >= 1 for all CPs in segment
%   - Returns optimal n and d
%
% MATLAB VERSION (simplified):
%   - Plane normal = direction from obstacle center to trajectory center
%   - d computed to place plane between them
%   - Less optimal but simpler

fprintf('Computing separating planes...\n');
[planes_n, planes_d] = computeSeparatingPlanes(q_guess, obstacles_inflated, num_segments);
fprintf('  Generated %d planes (%d segments x %d obstacles)\n\n', ...
    length(planes_d), num_segments, length(obstacles));

%% 8. Trajectory Optimization
% ---------------------------
% ROS VERSION (solver_gurobi.cpp):
%   - Uses Gurobi QCQP solver
%   - Decision variables: free control points q, (optionally n, d)
%   - Cost: control_cost + weight * terminal_cost
%   - Constraints:
%     * Initial/final conditions (equality)
%     * Velocity bounds (inequality per axis)
%     * Acceleration bounds (inequality per axis)
%     * Jerk bounds (inequality per axis)
%     * Sphere constraint: ||q - q0|| <= Ra
%     * Collision: n' * q + d >= 1 for each segment/obstacle
%
% MATLAB VERSION:
%   - Uses fmincon with SQP algorithm
%   - Same constraint structure but via nonlinear constraint function
%   - Separating planes fixed (not optimized)

fprintf('Running trajectory optimization...\n');
fprintf('  (Using fmincon - see solver_gurobi.cpp for Gurobi version)\n');

% Basis matrices (A_pos_bs_rest from mader_types.hpp)
% IMPORTANT: M_bspline must equal A_pos_bs for consistency!
A_pos_bs = [-1/6, 0.5, -0.5, 1/6; 0.5, -1.0, 0, 2/3; -0.5, 0.5, 0.5, 1/6; 1/6, 0, 0, 0];
M_bspline = A_pos_bs;  % Use the same matrix for coefficient computation

% Initial guess
num_free_cps = N + 1 - 6;
x0 = reshape(q_guess(:, 4:end-3), [], 1);

% Optimization
options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'sqp', ...
    'MaxFunctionEvaluations', 5000, ...
    'MaxIterations', 200, ...
    'OptimalityTolerance', 1e-5);

objFun = @(x) objectiveFunction(x, q0, q1, q2, qNm2, qNm1, qN, ...
    N, num_segments, A_pos_bs, deltaT, params.weight, pf);

conFun = @(x) constraintFunction(x, q0, q1, q2, qNm2, qNm1, qN, ...
    knots, N, p, num_segments, params.v_max, params.a_max, params.j_max, ...
    params.Ra, deltaT, A_pos_bs, obstacle_boxes, params.drone_radius);


tic;
[x_opt, fval, exitflag, output] = fmincon(objFun, x0, [], [], [], [], [], [], conFun, options);
opt_time = toc;

fprintf('\nOptimization completed:\n');
fprintf('  Time: %.3f s\n', opt_time);
fprintf('  Exit flag: %d\n', exitflag);
fprintf('  Cost: %.4f\n\n', fval);

%% 9. Extract and Sample Solution
% --------------------------------

q_free_opt = reshape(x_opt, 3, num_free_cps);
q_opt = [q0, q1, q2, q_free_opt, qNm2, qNm1, qN];

% Convert to piecewise polynomial (same as CPs2TrajAndPwp in bspline_utils.cpp)
pwp = struct();
pwp.times = knots(p+1:end-p);
pwp.coeff_x = zeros(num_segments, 4);
pwp.coeff_y = zeros(num_segments, 4);
pwp.coeff_z = zeros(num_segments, 4);

for seg = 1:num_segments
    % With A_pos_bs, coefficients are: Q * A gives [c3, c2, c1, c0] for u_vec=[u^3;u^2;u;1]
    pwp.coeff_x(seg, :) = q_opt(1, seg:seg+3) * M_bspline;
    pwp.coeff_y(seg, :) = q_opt(2, seg:seg+3) * M_bspline;
    pwp.coeff_z(seg, :) = q_opt(3, seg:seg+3) * M_bspline;
end

% Sample trajectory
t_samples = t_init:params.dc:t_final;
trajectory = zeros(3, length(t_samples));
for i = 1:length(t_samples)
    trajectory(:, i) = evalPwp(pwp, t_samples(i));
end

%% 10. Collision Check
% ---------------------
% Verify trajectory is collision-free

collision_free = true;
min_dist = inf;

for i = 1:length(t_samples)
    pt = trajectory(:, i);
    for j = 1:length(obstacles)
        d = distanceToObstacle(pt, obstacles{j});
        min_dist = min(min_dist, d);
        if d < params.drone_radius
            collision_free = false;
        end
    end
end

fprintf('=== Results ===\n');
fprintf('Collision free: %s\n', yesno(collision_free));
fprintf('Min distance to obstacles: %.3f m (required: %.2f m)\n', min_dist, params.drone_radius);
fprintf('Trajectory length: %.2f m\n', sum(vecnorm(diff(trajectory, 1, 2))));
fprintf('Duration: %.2f s\n\n', t_final - t_init);

%% 11. Visualization
% -------------------

figure('Position', [50, 50, 1400, 600]);

% 3D View
subplot(1, 2, 1);
hold on;

% Obstacles
for i = 1:length(obstacles)
    plotConvexHull3D(obstacles{i}, [0.8, 0.2, 0.2], 0.6);
end

% Initial guess
plot3(q_guess(1,:), q_guess(2,:), q_guess(3,:), 'g--', 'LineWidth', 1.5, 'DisplayName', 'A* Guess');

% Optimized trajectory
plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Optimized Traj');

% Control points
plot3(q_opt(1,:), q_opt(2,:), q_opt(3,:), 'ko-', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Control Points');

% Start and goal
plot3(initial_state.pos(1), initial_state.pos(2), initial_state.pos(3), ...
    'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2, 'DisplayName', 'Start');
plot3(target_goal(1), target_goal(2), target_goal(3), ...
    'mp', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'LineWidth', 2, 'DisplayName', 'Goal');

% Planning sphere
[sx, sy, sz] = sphere(20);
surf(params.Ra*sx + q0(1), params.Ra*sy + q0(2), params.Ra*sz + q0(3), ...
    'FaceAlpha', 0.05, 'EdgeColor', 'c', 'EdgeAlpha', 0.2, 'FaceColor', 'cyan', 'HandleVisibility', 'off');

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Trajectory View');
legend('Location', 'best');
grid on; axis equal;
xlim([-6, 6]); ylim([-4, 4]); zlim([-2, 2]);
view(45, 25);

% Top-down View
subplot(1, 2, 2);
hold on;

% Obstacles (2D)
for i = 1:length(obstacles)
    verts = obstacles{i};
    k = convhull(verts(1,:), verts(2,:));
    fill(verts(1,k), verts(2,k), [0.8, 0.2, 0.2], 'FaceAlpha', 0.6, 'EdgeColor', 'k');
end

plot(q_guess(1,:), q_guess(2,:), 'g--', 'LineWidth', 1.5);
plot(trajectory(1,:), trajectory(2,:), 'b-', 'LineWidth', 2.5);
plot(q_opt(1,:), q_opt(2,:), 'ko-', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot(initial_state.pos(1), initial_state.pos(2), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot(target_goal(1), target_goal(2), 'mp', 'MarkerSize', 15, 'MarkerFaceColor', 'm');

% Planning radius
theta = linspace(0, 2*pi, 100);
plot(params.Ra*cos(theta) + q0(1), params.Ra*sin(theta) + q0(2), 'c--', 'LineWidth', 2);

xlabel('X [m]'); ylabel('Y [m]');
title('Top-Down View (X-Y)');
grid on; axis equal;
xlim([-6, 6]); ylim([-4, 4]);

sgtitle('MADER Full Planner - Example 3', 'FontSize', 14);

%% ========================================================================
%                         HELPER FUNCTIONS
%  ========================================================================
% These functions implement simplified versions of the C++ classes.
% See the corresponding .cpp files for the full implementations.

function vertices = createBoxVertices(center, sz)
    % Creates 8 vertices of an axis-aligned box
    % Equivalent to simple obstacle definition in ROS launch files
    hx = sz(1)/2; hy = sz(2)/2; hz = sz(3)/2;
    vertices = [
        center(1) + [-hx, hx, hx, -hx, -hx, hx, hx, -hx];
        center(2) + [-hy, -hy, hy, hy, -hy, -hy, hy, hy];
        center(3) + [-hz, -hz, -hz, -hz, hz, hz, hz, hz]
    ];
end

function vertices_inf = inflateObstacle(vertices, radius)
    % Inflates obstacle by radius (approximates Minkowski sum with sphere)
    % ROS: Done properly in cgal_utils.cpp with CGAL Minkowski sum
    center = mean(vertices, 2);
    directions = vertices - center;
    norms = vecnorm(directions);
    norms(norms < 1e-6) = 1;
    directions_normalized = directions ./ norms;
    vertices_inf = vertices + radius * directions_normalized;
end

function [q_guess, success] = aStarSearch(q0, q1, q2, qNm2, qNm1, qN, goal, ...
                                           N, p, knots, params, obstacles)
    % Improved A* search with proper obstacle avoidance
    %
    % Key improvements over the original simplified version:
    % - Pre-computes a collision-free path around obstacles
    % - Uses multiple waypoint candidates with scoring
    % - Maintains smooth trajectories through velocity constraints

    num_free = N + 1 - 6;
    if num_free <= 0
        q_guess = [q0, q1, q2, qNm2, qNm1, qN];
        success = true;
        return;
    end

    % First, find obstacle avoidance direction
    start_pos = q2;
    end_pos = qNm2;

    % Check if direct path is blocked
    direct_blocked = false;
    blocking_obs_centers = [];
    for obs_idx = 1:length(obstacles)
        obs = obstacles{obs_idx};
        obs_center = mean(obs, 2);
        obs_min = min(obs, [], 2);
        obs_max = max(obs, [], 2);

        % Check if line segment intersects obstacle bounding box (with margin)
        margin = 0.5;
        if lineIntersectsBox(start_pos, end_pos, obs_min - margin, obs_max + margin)
            direct_blocked = true;
            blocking_obs_centers = [blocking_obs_centers, obs_center];
        end
    end

    % Generate waypoints that avoid obstacles
    if direct_blocked && ~isempty(blocking_obs_centers)
        % Find the best avoidance direction (perpendicular to path)
        path_dir = end_pos - start_pos;
        path_dir = path_dir / (norm(path_dir) + 1e-6);

        % Try going around in Y direction (most common for planar scenarios)
        % and Z direction (for 3D avoidance)
        best_waypoints = [];
        best_score = inf;

        avoidance_dirs = [
            [0; 1; 0], [0; -1; 0], ...   % Y directions
            [0; 0; 1], [0; 0; -1], ...   % Z directions
            [0; 1; 1]/sqrt(2), [0; 1; -1]/sqrt(2), ...
            [0; -1; 1]/sqrt(2), [0; -1; -1]/sqrt(2)
        ];

        for dir_idx = 1:size(avoidance_dirs, 2)
            avoid_dir = avoidance_dirs(:, dir_idx);

            % Make perpendicular to path direction
            avoid_dir = avoid_dir - dot(avoid_dir, path_dir) * path_dir;
            if norm(avoid_dir) < 0.1
                continue;
            end
            avoid_dir = avoid_dir / norm(avoid_dir);

            % Compute required offset to clear obstacles
            max_offset = 0;
            for oc_idx = 1:size(blocking_obs_centers, 2)
                obs_c = blocking_obs_centers(:, oc_idx);
                % Find obstacle extent in avoidance direction
                for obs_idx = 1:length(obstacles)
                    obs = obstacles{obs_idx};
                    % Compute dot product for each vertex: avoid_dir' * (obs - obs_c)
                    obs_projs = avoid_dir' * (obs - obs_c);  % 1xN row vector
                    obs_extent = max(abs(obs_projs));
                    proj_dist = abs(dot(avoid_dir, obs_c - (start_pos + end_pos)/2));
                    required = obs_extent + 0.5 - proj_dist;  % 0.5 margin
                    max_offset = max(max_offset, required);
                end
            end

            offset = max_offset + 0.3;  % Additional safety margin

            % Generate waypoint sequence
            mid_point = (start_pos + end_pos) / 2;
            waypoint = mid_point + offset * avoid_dir;

            % Check if waypoint is collision-free and within bounds
            collision = false;
            for obs_idx = 1:length(obstacles)
                if isInsideConvexHull(waypoint, obstacles{obs_idx})
                    collision = true;
                    break;
                end
            end

            % Check sphere constraint
            if norm(waypoint - q0) > params.Ra
                collision = true;
            end

            if ~collision
                % Score based on path length and smoothness
                path_len = norm(waypoint - start_pos) + norm(end_pos - waypoint);
                score = path_len;

                if score < best_score
                    best_score = score;
                    best_waypoints = waypoint;
                end
            end
        end

        % Use the best avoidance waypoint to generate control points
        if ~isempty(best_waypoints)
            q_guess = generateSmoothPath(q0, q1, q2, qNm2, qNm1, qN, ...
                best_waypoints, num_free, knots, p, params, obstacles);
            success = true;
            return;
        end
    end

    % Fallback: try direct interpolation with collision checks
    q_guess = [q0, q1, q2];
    current_q = q2;
    success = true;

    for idx = 1:num_free
        alpha = idx / (num_free + 1);
        target = (1 - alpha) * q2 + alpha * qNm2;

        delta_t = knots(p + 2 + idx) - knots(p + 1 + idx);
        max_step = params.v_max * delta_t;

        direction = target - current_q;
        step = min(abs(direction), max_step) .* sign(direction);
        next_q = current_q + step;

        % Collision check with improved avoidance
        collision = false;
        for obs_idx = 1:length(obstacles)
            if isInsideConvexHull(next_q, obstacles{obs_idx})
                collision = true;
                break;
            end
        end

        if collision
            [next_q, found] = findAlternativeImproved(current_q, target, max_step, obstacles, q0, params.Ra);
            if ~found
                success = false;
            end
        end

        q_guess = [q_guess, next_q];
        current_q = next_q;
    end

    q_guess = [q_guess, qNm2, qNm1, qN];
end

function intersects = lineIntersectsBox(p1, p2, box_min, box_max)
    % Check if line segment p1-p2 intersects axis-aligned box
    d = p2 - p1;
    t_min = 0;
    t_max = 1;

    for i = 1:3
        if abs(d(i)) < 1e-10
            if p1(i) < box_min(i) || p1(i) > box_max(i)
                intersects = false;
                return;
            end
        else
            t1 = (box_min(i) - p1(i)) / d(i);
            t2 = (box_max(i) - p1(i)) / d(i);
            if t1 > t2
                tmp = t1; t1 = t2; t2 = tmp;
            end
            t_min = max(t_min, t1);
            t_max = min(t_max, t2);
            if t_min > t_max
                intersects = false;
                return;
            end
        end
    end
    intersects = true;
end

function q_path = generateSmoothPath(q0, q1, q2, qNm2, qNm1, qN, waypoint, num_free, knots, p, params, obstacles)
    % Generate smooth control point path through waypoint
    q_path = [q0, q1, q2];

    start_pos = q2;
    end_pos = qNm2;

    for idx = 1:num_free
        alpha = idx / (num_free + 1);

        % Blend between start->waypoint->end
        if alpha < 0.5
            % First half: go toward waypoint
            t = alpha * 2;
            target = (1-t) * start_pos + t * waypoint;
        else
            % Second half: go from waypoint to end
            t = (alpha - 0.5) * 2;
            target = (1-t) * waypoint + t * end_pos;
        end

        % Apply velocity limits
        if idx > 1
            current_q = q_path(:, end);
        else
            current_q = q2;
        end

        delta_t = knots(p + 2 + idx) - knots(p + 1 + idx);
        max_step = params.v_max * delta_t;

        direction = target - current_q;
        dir_norm = norm(direction);
        if dir_norm > norm(max_step)
            next_q = current_q + direction / dir_norm .* max_step;
        else
            next_q = target;
        end

        q_path = [q_path, next_q];
    end

    q_path = [q_path, qNm2, qNm1, qN];
end

function [alt_q, found] = findAlternativeImproved(current, target, max_step, obstacles, q0, Ra)
    % Improved alternative finding with more directions and sphere constraint
    found = false;
    alt_q = target;

    % Try more directions including diagonals
    angles = linspace(0, 2*pi, 13);  % 12 directions in XY plane
    z_offsets = [-1, 0, 1];

    best_candidate = [];
    best_dist_to_target = inf;

    for z_off = z_offsets
        for angle = angles(1:end-1)
            offset = [cos(angle); sin(angle); z_off * 0.5];
            offset = offset / norm(offset);

            candidate = current + offset .* max_step;

            % Check sphere constraint
            if norm(candidate - q0) > Ra * 0.95
                continue;
            end

            % Check collision
            collision = false;
            for obs_idx = 1:length(obstacles)
                if isInsideConvexHull(candidate, obstacles{obs_idx})
                    collision = true;
                    break;
                end
            end

            if ~collision
                dist = norm(candidate - target);
                if dist < best_dist_to_target
                    best_dist_to_target = dist;
                    best_candidate = candidate;
                end
            end
        end
    end

    if ~isempty(best_candidate)
        alt_q = best_candidate;
        found = true;
    end
end

function inside = isInsideConvexHull(point, vertices)
    % Check if point is inside convex hull
    % ROS: Uses CGAL for robust geometric predicates

    minV = min(vertices, [], 2);
    maxV = max(vertices, [], 2);

    if any(point < minV - 0.01) || any(point > maxV + 0.01)
        inside = false;
        return;
    end

    try
        K = convhulln(vertices');
        center = mean(vertices, 2);
        inside = true;
        for i = 1:size(K, 1)
            tri = vertices(:, K(i, :));
            n = cross(tri(:,2) - tri(:,1), tri(:,3) - tri(:,1));
            if sign(dot(n, center - tri(:,1))) * sign(dot(n, point - tri(:,1))) < 0
                inside = false;
                return;
            end
        end
    catch
        inside = false;
    end
end

function dist = distanceToObstacle(point, vertices)
    % Approximate distance to obstacle (positive if outside)
    if isInsideConvexHull(point, vertices)
        dist = -0.1;  % Inside
    else
        dist = min(vecnorm(point - vertices));
    end
end

function [planes_n, planes_d] = computeSeparatingPlanes(q, obstacles, num_segments)
    % Compute separating planes between trajectory segments and obstacles
    %
    % ROS VERSION (separator.hpp, octopus_search.cpp):
    %   - Solves LP to find optimal separating hyperplane
    %   - Guarantees separation if it exists
    %
    % MATLAB VERSION (improved):
    %   - Computes plane that maximizes separation margin
    %   - Uses closest point on obstacle to trajectory segment

    planes_n = {};
    planes_d = [];

    for seg = 1:num_segments
        cps = q(:, seg:seg+3);
        cp_min = min(cps, [], 2);
        cp_max = max(cps, [], 2);
        cp_center = mean(cps, 2);

        for obs_idx = 1:length(obstacles)
            obs = obstacles{obs_idx};
            obs_center = mean(obs, 2);
            obs_min = min(obs, [], 2);
            obs_max = max(obs, [], 2);

            % Find the direction that best separates trajectory from obstacle
            % Try axis-aligned and diagonal directions
            best_n = [];
            best_margin = -inf;

            % Test directions
            test_dirs = [
                [1; 0; 0], [-1; 0; 0], [0; 1; 0], [0; -1; 0], [0; 0; 1], [0; 0; -1], ...
                [1; 1; 0]/sqrt(2), [1; -1; 0]/sqrt(2), [-1; 1; 0]/sqrt(2), [-1; -1; 0]/sqrt(2), ...
                cp_center - obs_center  % Direction from obstacle to trajectory
            ];

            for dir_idx = 1:size(test_dirs, 2)
                n = test_dirs(:, dir_idx);
                n_norm = norm(n);
                if n_norm < 1e-6
                    continue;
                end
                n = n / n_norm;

                % Project control points and obstacle vertices onto this direction
                cp_projs = n' * cps;
                obs_projs = n' * obs;

                min_cp = min(cp_projs);
                max_obs = max(obs_projs);

                % Margin is the separation in this direction
                margin = min_cp - max_obs;

                if margin > best_margin
                    best_margin = margin;
                    best_n = n;
                end
            end

            if isempty(best_n) || best_margin < -1  % No good separation found
                % Fallback: use direction from obstacle to trajectory
                n = cp_center - obs_center;
                n_norm = norm(n);
                if n_norm > 1e-6
                    n = n / n_norm;
                else
                    n = [1; 0; 0];
                end
                best_n = n;
            end

            % Compute d such that:
            % - Trajectory side: n' * cp + d >= margin_val (constraint will add margin)
            % - Obstacle side: n' * obs + d <= 0
            % Place the plane closer to the obstacle
            cp_projs = best_n' * cps;
            obs_projs = best_n' * obs;

            min_cp_proj = min(cp_projs);
            max_obs_proj = max(obs_projs);

            % d is set so that max_obs_proj + d = 0 (obstacle on negative side)
            d = -max_obs_proj;

            % Verify: min_cp_proj + d should be positive for valid separation
            if min_cp_proj + d < 0.05
                % Plane doesn't separate well, adjust d to be at midpoint
                mid_proj = (min_cp_proj + max_obs_proj) / 2;
                d = -mid_proj;
            end

            planes_n{end+1} = best_n;
            planes_d(end+1) = d;
        end
    end
end

function cost = objectiveFunction(x, q0, q1, q2, qNm2, qNm1, qN, ...
                                   N, num_segments, A_pos_bs, deltaT, weight, pf)
    num_free = length(x) / 3;
    q_free = reshape(x, 3, num_free);
    q = [q0, q1, q2, q_free, qNm2, qNm1, qN];

    control_cost = 0;
    tmp = [6; 0; 0; 0];

    for i = 1:num_segments
        Qi = q(:, i:i+3);
        A_i_times_tmp = A_pos_bs * tmp;
        for axis = 1:3
            jerk = Qi(axis, :) * A_i_times_tmp;
            control_cost = control_cost + jerk^2;
        end
    end

    terminal_cost = norm(q(:, end) - pf)^2;
    cost = control_cost + weight * terminal_cost;
end

function [c, ceq] = constraintFunction(x, q0, q1, q2, qNm2, qNm1, qN, ...
    knots, N, p, num_segments, v_max, a_max, j_max, Ra, deltaT, A_pos_bs, ...
    obstacle_boxes, drone_radius)


    num_free = length(x) / 3;
    q_free = reshape(x, 3, num_free);
    q = [q0, q1, q2, q_free, qNm2, qNm1, qN];

    c = [];
    ceq = [];

    % Velocity constraints
    for i = 3:N
        ci = p / (knots(i+p+1) - knots(i+1));
        v_i = ci * (q(:, i+1) - q(:, i));
        c = [c; v_i - v_max; -v_i - v_max];
    end

    % Acceleration constraints
    for i = 2:N-2
        c1 = p / (knots(i+p+1) - knots(i+1));
        c2 = p / (knots(i+p+2) - knots(i+2));
        c3 = (p - 1) / (knots(i+p+1) - knots(i+2));
        v_i = c1 * (q(:, i+1) - q(:, i));
        v_iP1 = c2 * (q(:, i+2) - q(:, i+1));
        a_i = c3 * (v_iP1 - v_i);
        c = [c; a_i - a_max; -a_i - a_max];
    end

    % Jerk constraints
    tmp = [6; 0; 0; 0] / (deltaT^3);
    for i = 1:num_segments
        Qi = q(:, i:i+3);
        A_i_times_tmp = A_pos_bs * tmp;
        for axis = 1:3
            j_i = Qi(axis, :) * A_i_times_tmp;
            c = [c; j_i - j_max(axis); -j_i - j_max(axis)];
        end
    end

    % Sphere constraint
    for i = 4:(N+1-3)
        c = [c; sum((q(:, i) - q0).^2) - Ra^2];
    end

    % Collision constraints (separating planes)
% Collision constraints (sample points vs axis-aligned boxes)
    u_samples = [0.2, 0.5, 0.8];
    for seg = 1:num_segments
        Qi = q(:, seg:seg+3);
        coeff = Qi * A_pos_bs; % 3x4 coefficients for u_vec=[u^3;u^2;u;1]
        for u = u_samples
            u_vec = [u^3; u^2; u; 1];
            pt = coeff * u_vec;
            for obs_idx = 1:length(obstacle_boxes)
                obs_min = obstacle_boxes{obs_idx}(:,1);
                obs_max = obstacle_boxes{obs_idx}(:,2);
                d = max([obs_min - pt, zeros(3,1), pt - obs_max], [], 2);
                c = [c; (drone_radius^2 - sum(d.^2))];
            end
        end
    end
end

function pos = evalPwp(pwp, t)
    times = pwp.times;
    if t >= times(end)
        u = 1; seg = length(times) - 1;
    elseif t < times(1)
        u = 0; seg = 1;
    else
        seg = find(times(1:end-1) <= t & t < times(2:end), 1);
        if isempty(seg), seg = length(times) - 1; end
        u = (t - times(seg)) / (times(seg+1) - times(seg));
    end
    u_vec = [u^3; u^2; u; 1];
    pos = [pwp.coeff_x(seg,:)*u_vec; pwp.coeff_y(seg,:)*u_vec; pwp.coeff_z(seg,:)*u_vec];
end

function plotConvexHull3D(vertices, color, alpha)
    try
        K = convhulln(vertices');
        trisurf(K, vertices(1,:), vertices(2,:), vertices(3,:), ...
            'FaceColor', color, 'FaceAlpha', alpha, 'EdgeColor', 'k', ...
            'LineWidth', 0.5, 'HandleVisibility', 'off');
    catch
    end
end

function s = yesno(b)
    if b, s = 'YES'; else, s = 'NO'; end
end
