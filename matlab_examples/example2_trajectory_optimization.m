%% MADER MATLAB Example 2: Trajectory Optimization
% =========================================================================
% This script demonstrates trajectory optimization as used in MADER.
% Converted from the C++ implementation in mader/src/solver_gurobi.cpp
%
% KEY CONCEPT: LOCAL PLANNING with PLANNING HORIZON Ra
% ----------------------------------------------------
% MADER uses a LOCAL planning approach with radius Ra:
%   - Ra defines the "planning horizon" (how far ahead we plan)
%   - If the goal is WITHIN Ra: plan directly to goal
%   - If the goal is OUTSIDE Ra: plan to an INTERMEDIATE goal G on the
%     boundary of the Ra sphere, in the direction of the final goal
%
% This is crucial because:
%   1. Local planning is computationally faster
%   2. Re-planning handles dynamic obstacles
%   3. The trajectory stays within a known safe region
%
% OPTIMIZATION PROBLEM:
%   minimize:   control_cost (jerk^2) + weight * terminal_cost
%   subject to:
%     - Initial conditions (pos, vel, accel)
%     - Final conditions (pos, vel=0, accel=0)
%     - Velocity limits: |v| <= v_max per axis
%     - Acceleration limits: |a| <= a_max per axis
%     - Jerk limits: |j| <= j_max per axis
%     - Sphere constraint: all control points within Ra of start
%
% Author: Converted from MIT ACL MADER project
% Reference: Jesus Tordesillas, et al. "MADER: Trajectory Planner" IEEE T-RO 2021
% =========================================================================

clear; clc; close all;

%% 1. Problem Parameters (from solver_params.hpp)
params = struct();
params.v_max = [2.0; 2.0; 2.0];        % Max velocity [m/s]
params.a_max = [3.0; 3.0; 3.0];        % Max acceleration [m/s^2]
params.j_max = [10.0; 10.0; 10.0];     % Max jerk [m/s^3]
params.num_pol = 6;                     % Number of polynomial segments
params.deg_pol = 3;                     % Polynomial degree (cubic)
params.weight = 100.0;                  % Weight for terminal cost
params.Ra = 4.0;                        % Planning radius (sphere constraint)
params.dc = 0.01;                       % Time discretization
params.goal_radius = 0.1;              % Tolerance to consider goal reached

fprintf('=== MADER Trajectory Optimization (Example 2) ===\n\n');
fprintf('Parameters:\n');
fprintf('  v_max: [%.1f, %.1f, %.1f] m/s\n', params.v_max);
fprintf('  a_max: [%.1f, %.1f, %.1f] m/s^2\n', params.a_max);
fprintf('  j_max: [%.1f, %.1f, %.1f] m/s^3\n', params.j_max);
fprintf('  Number of segments: %d\n', params.num_pol);
fprintf('  Planning radius Ra: %.1f m\n\n', params.Ra);

%% 2. Define Initial State and FINAL Goal
% The final goal may be far away; we'll compute intermediate goal if needed

initial_state = struct();
initial_state.pos = [0; 0; 0];         % Start at origin
initial_state.vel = [0.5; 0; 0];       % Moving in +X direction
initial_state.accel = [0; 0; 0];

% FINAL goal (may be outside Ra)
final_goal = [6; 2; 1];

fprintf('Initial state: pos=[%.1f, %.1f, %.1f], vel=[%.1f, %.1f, %.1f]\n', ...
    initial_state.pos, initial_state.vel);
fprintf('Final goal:    [%.1f, %.1f, %.1f]\n', final_goal);

dist_to_goal = norm(final_goal - initial_state.pos);
fprintf('Distance to goal: %.2f m\n', dist_to_goal);

%% 3. Compute INTERMEDIATE Goal (if needed)
% =========================================================================
% THIS IS THE KEY MADER CONCEPT!
% If goal is outside planning sphere, compute intermediate goal G on the
% boundary of the sphere, in the direction of the final goal.
% =========================================================================

if dist_to_goal > params.Ra
    % Goal is outside Ra -> compute intermediate goal G
    direction = (final_goal - initial_state.pos) / dist_to_goal;
    intermediate_goal = initial_state.pos + params.Ra * 0.95 * direction;  % 0.95 for margin

    fprintf('\n*** Goal is OUTSIDE planning radius Ra! ***\n');
    fprintf('Computing intermediate goal G within Ra...\n');
    fprintf('Intermediate goal G: [%.2f, %.2f, %.2f]\n', intermediate_goal);
    fprintf('Distance to G: %.2f m (< Ra = %.1f m)\n\n', ...
        norm(intermediate_goal - initial_state.pos), params.Ra);

    target_goal = intermediate_goal;
    goal_type = 'INTERMEDIATE';
else
    % Goal is within Ra -> plan directly to goal
    fprintf('\nGoal is WITHIN planning radius Ra. Planning directly to goal.\n\n');
    target_goal = final_goal;
    goal_type = 'FINAL';
end

% Final state for this planning iteration
final_state = struct();
final_state.pos = target_goal;
final_state.vel = [0; 0; 0];           % Stop at intermediate/final goal
final_state.accel = [0; 0; 0];

%% 4. Compute B-Spline Parameters
% Following the notation from solver_gurobi.cpp:
% p = degree, M = num_pol + 2*p, N = M - p - 1

p = params.deg_pol;
num_segments = params.num_pol;
M = num_segments + 2 * p;  % Total number of knots - 1
N = M - p - 1;             % Number of control points - 1

fprintf('B-spline setup:\n');
fprintf('  Degree p = %d\n', p);
fprintf('  N+1 control points = %d\n', N + 1);
fprintf('  %d polynomial segments\n\n', num_segments);

%% 5. Set Up Time Allocation and Knot Vector
% Time allocation based on distance and max velocity
dist = norm(target_goal - initial_state.pos);
t_init = 0;
t_final = dist / mean(params.v_max) * 1.5;  % Conservative time estimate

% Compute deltaT (uniform knot spacing)
deltaT = (t_final - t_init) / num_segments;

% Build clamped uniform B-spline knot vector
knots = zeros(1, M + 1);
knots(1:p+1) = t_init;
for i = p+2 : M-p
    knots(i) = knots(i-1) + deltaT;
end
knots(M-p+1:M+1) = t_final;

fprintf('Time allocation:\n');
fprintf('  t_init = %.2f s, t_final = %.2f s\n', t_init, t_final);
fprintf('  deltaT = %.3f s\n\n', deltaT);

%% 6. Compute Fixed Control Points from Initial/Final Conditions
% From solver_gurobi.cpp lines 589-596
% These equations enforce initial/final pos, vel, accel
%
% For a cubic B-spline (p=3):
%   - First 3 control points (q0, q1, q2) are determined by initial pos/vel/accel
%   - Last 3 control points (qN-2, qN-1, qN) are determined by final pos/vel/accel

p0 = initial_state.pos;
v0 = initial_state.vel;
a0 = initial_state.accel;
pf = final_state.pos;
vf = final_state.vel;
af = final_state.accel;

% Knot values needed for the formulas
t1 = knots(2);          % knots_(1) in C++ (0-indexed)
t2 = knots(3);          % knots_(2)
tpP1 = knots(p+2);      % knots_(p+1)
t1PpP1 = knots(2+p+1);  % knots_(1+p+1)

tN = knots(N+1);        % knots_(N)
tNm1 = knots(N);        % knots_(N-1)
tNPp = knots(N+p+1);    % knots_(N+p)
tNm1Pp = knots(N+p);    % knots_(N-1+p)

% Fixed control points for INITIAL conditions
q0 = p0;
q1 = p0 + (tpP1 - t1) * v0 / p;
q2 = (p^2 * q1 - (t1PpP1 - t2) * (a0 * (t2 - tpP1) + v0) - p * (q1 + (t2 - t1PpP1) * v0)) / ((p - 1) * p);

% Fixed control points for FINAL conditions
qN = pf;
qNm1 = pf + (tN - tNPp) * vf / p;
qNm2 = (p^2 * qNm1 - (tNm1 - tNm1Pp) * (af * (tNm1Pp - tN) + vf) - p * (qNm1 + (tNm1Pp - tNm1) * vf)) / ((p - 1) * p);

fprintf('Fixed control points:\n');
fprintf('  q0  = [%.3f, %.3f, %.3f] (initial pos)\n', q0);
fprintf('  q1  = [%.3f, %.3f, %.3f] (initial vel)\n', q1);
fprintf('  q2  = [%.3f, %.3f, %.3f] (initial accel)\n', q2);
fprintf('  qN-2= [%.3f, %.3f, %.3f] (final accel)\n', qNm2);
fprintf('  qN-1= [%.3f, %.3f, %.3f] (final vel)\n', qNm1);
fprintf('  qN  = [%.3f, %.3f, %.3f] (final pos = %s goal)\n\n', qN, goal_type);

% Verify all fixed points are within Ra (sanity check)
fixed_points = [q0, q1, q2, qNm2, qNm1, qN];
max_dist_fixed = max(vecnorm(fixed_points - q0));
fprintf('Max distance of fixed control points from start: %.3f m\n', max_dist_fixed);
if max_dist_fixed > params.Ra
    warning('Some fixed control points are outside Ra! This may cause infeasibility.');
else
    fprintf('All fixed control points are within Ra. Good!\n\n');
end

%% 7. Set Up the B-Spline Basis Matrices
% B-spline basis matrix for uniform cubic splines
M_bspline = (1/6) * [
    1,  4,  1, 0;
   -3,  0,  3, 0;
    3, -6,  3, 0;
   -1,  3, -3, 1
];

% A matrix for jerk computation (from basisConverter in mader_types.hpp)
A_pos_bs_rest = [
   -1/6,   0.5,   -0.5,   1/6;
    0.5,  -1.0,      0,   2/3;
   -0.5,   0.5,    0.5,   1/6;
    1/6,     0,      0,     0
];

%% 8. Define Optimization Variables
% Free control points are q3, q4, ..., qN-3
% Fixed: q0, q1, q2 (initial) and qN-2, qN-1, qN (final)

num_free_cps = N + 1 - 6;  % Total CPs minus 6 fixed ones
fprintf('Optimization setup:\n');
fprintf('  Total control points: %d\n', N + 1);
fprintf('  Fixed control points: 6 (3 initial + 3 final)\n');
fprintf('  Free control points: %d\n', num_free_cps);
fprintf('  Decision variables: %d (3 coords x %d free CPs)\n\n', 3 * num_free_cps, num_free_cps);

%% 9. Define Cost Function
% Cost = control_cost (jerk^2) + weight * terminal_cost (distance to goal)

function cost = objectiveFunction(x, q0, q1, q2, qNm2, qNm1, qN, ...
                                   N, num_segments, A_pos_bs, deltaT, weight, pf)
    % Reshape decision variables to 3 x num_free matrix
    num_free = length(x) / 3;
    q_free = reshape(x, 3, num_free);

    % Assemble all control points [q0, q1, q2, free..., qN-2, qN-1, qN]
    q = [q0, q1, q2, q_free, qNm2, qNm1, qN];

    % Control cost: sum of squared jerk over all segments
    % Jerk = d^3/dt^3 of position = 6 * leading coefficient
    control_cost = 0;
    tmp = [6; 0; 0; 0];  % Multiplier to get jerk from polynomial coefficients

    for i = 1:num_segments
        Qi = q(:, i:i+3);  % 4 control points for segment i
        A_i_times_tmp = A_pos_bs * tmp;

        for axis = 1:3
            jerk_component = Qi(axis, :) * A_i_times_tmp;
            control_cost = control_cost + jerk_component^2;
        end
    end

    % Terminal cost: squared distance from last control point to goal
    % Note: qN = pf by construction, so this is always 0
    % The weight helps drive intermediate CPs toward the goal direction
    terminal_cost = norm(q(:, end) - pf)^2;

    cost = control_cost + weight * terminal_cost;
end

%% 10. Define Constraint Function
% Constraints: velocity, acceleration, jerk limits + sphere constraint

function [c, ceq] = constraintFunction(x, q0, q1, q2, qNm2, qNm1, qN, ...
                                        knots, N, p, num_segments, ...
                                        v_max, a_max, j_max, Ra, deltaT, A_pos_bs)
    num_free = length(x) / 3;
    q_free = reshape(x, 3, num_free);
    q = [q0, q1, q2, q_free, qNm2, qNm1, qN];

    c = [];   % Inequality constraints: c(x) <= 0
    ceq = []; % Equality constraints: ceq(x) = 0

    % =====================================================================
    % VELOCITY CONSTRAINTS
    % For B-splines, velocity control points are: v_i = p/(t_{i+p+1} - t_{i+1}) * (q_{i+1} - q_i)
    % We require: -v_max <= v_i <= v_max (per axis)
    % =====================================================================
    for i = 3:N  % Start at 3 because v0, v1 are determined by initial conditions
        ci = p / (knots(i+p+1) - knots(i+1));
        v_i = ci * (q(:, i+1) - q(:, i));

        c = [c; v_i - v_max];      % v_i <= v_max
        c = [c; -v_i - v_max];     % v_i >= -v_max (i.e., -v_i <= v_max)
    end

    % =====================================================================
    % ACCELERATION CONSTRAINTS
    % a_i = (p-1)/(t_{i+p+1} - t_{i+2}) * (v_{i+1} - v_i)
    % We require: -a_max <= a_i <= a_max
    % =====================================================================
    for i = 2:N-2  % a_0 determined by initial conditions
        c1 = p / (knots(i+p+1) - knots(i+1));
        c2 = p / (knots(i+p+2) - knots(i+2));
        c3 = (p - 1) / (knots(i+p+1) - knots(i+2));

        v_i = c1 * (q(:, i+1) - q(:, i));
        v_iP1 = c2 * (q(:, i+2) - q(:, i+1));
        a_i = c3 * (v_iP1 - v_i);

        c = [c; a_i - a_max];
        c = [c; -a_i - a_max];
    end

    % =====================================================================
    % JERK CONSTRAINTS
    % Jerk is constant within each segment (cubic polynomial)
    % j_i = 6 * (leading coeff) / deltaT^3
    % =====================================================================
    tmp = [6; 0; 0; 0] / (deltaT^3);
    for i = 1:num_segments
        Qi = q(:, i:i+3);
        A_i_times_tmp = A_pos_bs * tmp;

        for axis = 1:3
            j_i = Qi(axis, :) * A_i_times_tmp;
            c = [c; j_i - j_max(axis)];
            c = [c; -j_i - j_max(axis)];
        end
    end

    % =====================================================================
    % SPHERE CONSTRAINT (Planning Horizon)
    % All FREE control points must be within Ra of the start position q0
    % This ensures the trajectory stays within the local planning region
    % =====================================================================
    for i = 4:(N+1-3)  % Indices of free control points (1-indexed)
        dist_sq = sum((q(:, i) - q0).^2);  % squared distance
        c = [c; dist_sq - Ra^2];           % dist^2 <= Ra^2
    end
end

%% 11. Initialize and Solve
% Initial guess: linear interpolation between q2 and qN-2
x0 = [];
for i = 1:num_free_cps
    t = i / (num_free_cps + 1);
    q_init = (1 - t) * q2 + t * qNm2;
    x0 = [x0; q_init];
end

% Optimization options
options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'sqp', ...
    'MaxFunctionEvaluations', 10000, ...
    'MaxIterations', 500, ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-8);

% Create function handles
objFun = @(x) objectiveFunction(x, q0, q1, q2, qNm2, qNm1, qN, ...
                                N, num_segments, A_pos_bs_rest, deltaT, params.weight, pf);

conFun = @(x) constraintFunction(x, q0, q1, q2, qNm2, qNm1, qN, ...
                                  knots, N, p, num_segments, ...
                                  params.v_max, params.a_max, params.j_max, ...
                                  params.Ra, deltaT, A_pos_bs_rest);

% Solve
fprintf('Starting optimization...\n');
tic;
[x_opt, fval, exitflag, output] = fmincon(objFun, x0, [], [], [], [], [], [], conFun, options);
opt_time = toc;

fprintf('\nOptimization completed in %.3f seconds\n', opt_time);
fprintf('Exit flag: %d (%s)\n', exitflag, getExitFlagMessage(exitflag));
fprintf('Final cost: %.4f\n', fval);
fprintf('Iterations: %d\n\n', output.iterations);

%% 12. Extract Solution
num_free = length(x_opt) / 3;
q_free_opt = reshape(x_opt, 3, num_free);
q_opt = [q0, q1, q2, q_free_opt, qNm2, qNm1, qN];

fprintf('Optimized control points:\n');
for i = 1:size(q_opt, 2)
    dist_from_start = norm(q_opt(:, i) - q0);
    in_sphere = dist_from_start <= params.Ra;
    fprintf('  q%d = [%6.3f, %6.3f, %6.3f]  dist=%.2f %s\n', i-1, q_opt(:, i), ...
        dist_from_start, conditional(in_sphere, '', ' OUTSIDE Ra!'));
end

%% 13. Convert to Piecewise Polynomial and Sample
pwp = struct();
pwp.times = knots(p+1:end-p);  % Active knot span
pwp.coeff_x = zeros(num_segments, 4);
pwp.coeff_y = zeros(num_segments, 4);
pwp.coeff_z = zeros(num_segments, 4);

for seg = 1:num_segments
    cps_x = q_opt(1, seg:seg+3)';
    cps_y = q_opt(2, seg:seg+3)';
    cps_z = q_opt(3, seg:seg+3)';

    pwp.coeff_x(seg, :) = flip(M_bspline * cps_x)';
    pwp.coeff_y(seg, :) = flip(M_bspline * cps_y)';
    pwp.coeff_z(seg, :) = flip(M_bspline * cps_z)';
end

% Sample trajectory
t_samples = t_init:params.dc:t_final;
trajectory = zeros(3, length(t_samples));
velocity = zeros(3, length(t_samples));
acceleration = zeros(3, length(t_samples));

for i = 1:length(t_samples)
    [trajectory(:, i), velocity(:, i), acceleration(:, i)] = evalPwpFull(pwp, t_samples(i));
end

%% 14. Visualization
figure('Position', [50, 50, 1500, 800]);

% 3D Trajectory
subplot(2, 3, 1);
hold on;

% Plot trajectory
plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'b-', 'LineWidth', 2.5);

% Plot control points
plot3(q_opt(1,:), q_opt(2,:), q_opt(3,:), 'ro-', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Plot start
plot3(initial_state.pos(1), initial_state.pos(2), initial_state.pos(3), ...
    'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);

% Plot intermediate goal (target for this iteration)
plot3(target_goal(1), target_goal(2), target_goal(3), ...
    'mp', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'LineWidth', 2);

% Plot final goal (if different from target)
if ~isequal(target_goal, final_goal)
    plot3(final_goal(1), final_goal(2), final_goal(3), ...
        'c^', 'MarkerSize', 12, 'MarkerFaceColor', 'c', 'LineWidth', 2);
    % Draw line from intermediate to final goal
    plot3([target_goal(1), final_goal(1)], [target_goal(2), final_goal(2)], ...
        [target_goal(3), final_goal(3)], 'c--', 'LineWidth', 1);
end

% Draw planning sphere
[sx, sy, sz] = sphere(30);
surf(params.Ra*sx + q0(1), params.Ra*sy + q0(2), params.Ra*sz + q0(3), ...
    'FaceAlpha', 0.1, 'EdgeColor', 'c', 'EdgeAlpha', 0.3, 'FaceColor', 'cyan');

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('Trajectory to %s Goal', goal_type));
if ~isequal(target_goal, final_goal)
    legend('Trajectory', 'Control Points', 'Start', 'Intermediate Goal G', ...
        'Final Goal', 'To Final', 'Planning Sphere Ra', 'Location', 'best');
else
    legend('Trajectory', 'Control Points', 'Start', 'Goal', 'Planning Sphere Ra', 'Location', 'best');
end
grid on; axis equal; view(45, 25);

% Position vs Time
subplot(2, 3, 2);
plot(t_samples, trajectory(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t_samples, trajectory(2,:), 'g-', 'LineWidth', 1.5);
plot(t_samples, trajectory(3,:), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Position [m]');
title('Position vs Time');
legend('X', 'Y', 'Z', 'Location', 'best');
grid on;

% Velocity vs Time
subplot(2, 3, 3);
plot(t_samples, velocity(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t_samples, velocity(2,:), 'g-', 'LineWidth', 1.5);
plot(t_samples, velocity(3,:), 'b-', 'LineWidth', 1.5);
yline(params.v_max(1), 'k--', 'LineWidth', 1.5);
yline(-params.v_max(1), 'k--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Velocity [m/s]');
title('Velocity vs Time');
legend('Vx', 'Vy', 'Vz', 'v_{max}', 'Location', 'best');
grid on;

% Acceleration vs Time
subplot(2, 3, 4);
plot(t_samples, acceleration(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t_samples, acceleration(2,:), 'g-', 'LineWidth', 1.5);
plot(t_samples, acceleration(3,:), 'b-', 'LineWidth', 1.5);
yline(params.a_max(1), 'k--', 'LineWidth', 1.5);
yline(-params.a_max(1), 'k--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
title('Acceleration vs Time');
legend('Ax', 'Ay', 'Az', 'a_{max}', 'Location', 'best');
grid on;

% Distance from start (verify sphere constraint)
subplot(2, 3, 5);
dist_from_start = vecnorm(trajectory - q0);
plot(t_samples, dist_from_start, 'b-', 'LineWidth', 2);
yline(params.Ra, 'r--', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Distance from Start [m]');
title('Sphere Constraint Verification');
legend('Trajectory Distance', 'Ra (Planning Radius)', 'Location', 'best');
grid on;

% Verify trajectory stays within Ra
if max(dist_from_start) <= params.Ra * 1.01  % 1% tolerance
    text(t_final/2, params.Ra*0.5, 'CONSTRAINT SATISFIED', ...
        'Color', 'g', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center');
else
    text(t_final/2, params.Ra*0.5, 'CONSTRAINT VIOLATED!', ...
        'Color', 'r', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center');
end

% Top-down view
subplot(2, 3, 6);
hold on;
plot(trajectory(1,:), trajectory(2,:), 'b-', 'LineWidth', 2.5);
plot(q_opt(1,:), q_opt(2,:), 'ro-', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(initial_state.pos(1), initial_state.pos(2), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot(target_goal(1), target_goal(2), 'mp', 'MarkerSize', 15, 'MarkerFaceColor', 'm');
if ~isequal(target_goal, final_goal)
    plot(final_goal(1), final_goal(2), 'c^', 'MarkerSize', 12, 'MarkerFaceColor', 'c');
end
% Planning circle
theta = linspace(0, 2*pi, 100);
plot(params.Ra*cos(theta) + q0(1), params.Ra*sin(theta) + q0(2), 'c-', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]');
title('Top-Down View (X-Y)');
grid on; axis equal;

sgtitle(sprintf('MADER Trajectory Optimization - %s Goal (Example 2)', goal_type), 'FontSize', 14);

%% 15. Verify Constraints
fprintf('\n=== Constraint Verification ===\n');
fprintf('Velocity:\n');
fprintf('  Max |Vx|: %.3f m/s (limit: %.1f)\n', max(abs(velocity(1,:))), params.v_max(1));
fprintf('  Max |Vy|: %.3f m/s (limit: %.1f)\n', max(abs(velocity(2,:))), params.v_max(2));
fprintf('  Max |Vz|: %.3f m/s (limit: %.1f)\n', max(abs(velocity(3,:))), params.v_max(3));
fprintf('Sphere constraint:\n');
fprintf('  Max distance from start: %.3f m (limit Ra: %.1f m)\n', max(dist_from_start), params.Ra);

if max(dist_from_start) <= params.Ra
    fprintf('  Status: SATISFIED\n');
else
    fprintf('  Status: VIOLATED by %.3f m\n', max(dist_from_start) - params.Ra);
end

fprintf('\nFinal position: [%.3f, %.3f, %.3f]\n', trajectory(:, end));
fprintf('Target goal:    [%.3f, %.3f, %.3f]\n', target_goal);
fprintf('Distance to target: %.4f m\n', norm(trajectory(:, end) - target_goal));

%% Helper Functions

function [pos, vel, accel] = evalPwpFull(pwp, t)
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

    delta_t = times(seg+1) - times(seg);

    u_vec = [u^3; u^2; u; 1];
    pos = [pwp.coeff_x(seg,:)*u_vec; pwp.coeff_y(seg,:)*u_vec; pwp.coeff_z(seg,:)*u_vec];

    du_vec = [3*u^2; 2*u; 1; 0];
    vel = [pwp.coeff_x(seg,:)*du_vec; pwp.coeff_y(seg,:)*du_vec; pwp.coeff_z(seg,:)*du_vec] / delta_t;

    ddu_vec = [6*u; 2; 0; 0];
    accel = [pwp.coeff_x(seg,:)*ddu_vec; pwp.coeff_y(seg,:)*ddu_vec; pwp.coeff_z(seg,:)*ddu_vec] / (delta_t^2);
end

function msg = getExitFlagMessage(flag)
    switch flag
        case 1, msg = 'Converged to solution';
        case 0, msg = 'Max iterations reached';
        case -1, msg = 'Stopped by output function';
        case -2, msg = 'No feasible point found';
        otherwise, msg = 'Unknown';
    end
end

function s = conditional(cond, true_str, false_str)
    if cond, s = true_str; else, s = false_str; end
end
