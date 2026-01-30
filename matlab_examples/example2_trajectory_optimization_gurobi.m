%% MADER MATLAB Example 2 (Gurobi): Trajectory Optimization
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
params.goal_radius = 0.1;               % Tolerance to consider goal reached

fprintf('=== MADER Trajectory Optimization (Example 2, Gurobi) ===\n\n');
fprintf('Parameters:\n');
fprintf('  v_max: [%.1f, %.1f, %.1f] m/s\n', params.v_max);
fprintf('  a_max: [%.1f, %.1f, %.1f] m/s^2\n', params.a_max);
fprintf('  j_max: [%.1f, %.1f, %.1f] m/s^3\n', params.j_max);
fprintf('  Number of segments: %d\n', params.num_pol);
fprintf('  Planning radius Ra: %.1f m\n\n', params.Ra);

%% 2. Define Initial State and FINAL Goal
initial_state = struct();
initial_state.pos = [0; 0; 0];         % Start at origin
initial_state.vel = [0.5; 0; 0];       % Moving in +X direction
initial_state.accel = [0; 0; 0];

% FINAL goal (may be outside Ra)
final_goal = [6; 2; 4];

fprintf('Initial state: pos=[%.1f, %.1f, %.1f], vel=[%.1f, %.1f, %.1f]\n', ...
    initial_state.pos, initial_state.vel);
fprintf('Final goal:    [%.1f, %.1f, %.1f]\n', final_goal);

dist_to_goal = norm(final_goal - initial_state.pos);
fprintf('Distance to goal: %.2f m\n', dist_to_goal);

%% 3. Compute INTERMEDIATE Goal (if needed)
if dist_to_goal > params.Ra
    direction = (final_goal - initial_state.pos) / dist_to_goal;
    intermediate_goal = initial_state.pos + params.Ra * 0.95 * direction;

    fprintf('\n*** Goal is OUTSIDE planning radius Ra! ***\n');
    fprintf('Computing intermediate goal G within Ra...\n');
    fprintf('Intermediate goal G: [%.2f, %.2f, %.2f]\n', intermediate_goal);
    fprintf('Distance to G: %.2f m (< Ra = %.1f m)\n\n', ...
        norm(intermediate_goal - initial_state.pos), params.Ra);

    target_goal = intermediate_goal;
    goal_type = 'INTERMEDIATE';
else
    fprintf('\nGoal is WITHIN planning radius Ra. Planning directly to goal.\n\n');
    target_goal = final_goal;
    goal_type = 'FINAL';
end

final_state = struct();
final_state.pos = target_goal;
final_state.vel = [0; 0; 0];
final_state.accel = [0; 0; 0];

%% 4. Compute B-Spline Parameters
p = params.deg_pol;
num_segments = params.num_pol;
M = num_segments + 2 * p;
N = M - p - 1;

fprintf('B-spline setup:\n');
fprintf('  Degree p = %d\n', p);
fprintf('  N+1 control points = %d\n', N + 1);
fprintf('  %d polynomial segments\n\n', num_segments);

%% 5. Set Up Time Allocation and Knot Vector
dist = norm(target_goal - initial_state.pos);
t_init = 0;
t_final = dist / mean(params.v_max) * 1.5;
deltaT = (t_final - t_init) / num_segments;

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
p0 = initial_state.pos;
v0 = initial_state.vel;
a0 = initial_state.accel;
pf = final_state.pos;
vf = final_state.vel;
af = final_state.accel;

t1 = knots(2);
t2 = knots(3);
tpP1 = knots(p+2);
t1PpP1 = knots(2+p+1);
tN = knots(N+1);
tNm1 = knots(N);
tNPp = knots(N+p+1);
tNm1Pp = knots(N+p);

q0 = p0;
q1 = p0 + (tpP1 - t1) * v0 / p;
q2 = (p^2 * q1 - (t1PpP1 - t2) * (a0 * (t2 - tpP1) + v0) ...
    - p * (q1 + (t2 - t1PpP1) * v0)) / ((p - 1) * p);

qN = pf;
qNm1 = pf + (tN - tNPp) * vf / p;
qNm2 = (p^2 * qNm1 - (tNm1 - tNm1Pp) * (af * (tNm1Pp - tN) + vf) ...
    - p * (qNm1 + (tNm1Pp - tNm1) * vf)) / ((p - 1) * p);

fprintf('Fixed control points:\n');
fprintf('  q0  = [%.3f, %.3f, %.3f] (initial pos)\n', q0);
fprintf('  q1  = [%.3f, %.3f, %.3f] (initial vel)\n', q1);
fprintf('  q2  = [%.3f, %.3f, %.3f] (initial accel)\n', q2);
fprintf('  qN-2= [%.3f, %.3f, %.3f] (final accel)\n', qNm2);
fprintf('  qN-1= [%.3f, %.3f, %.3f] (final vel)\n', qNm1);
fprintf('  qN  = [%.3f, %.3f, %.3f] (final pos = %s goal)\n\n', qN, goal_type);

fixed_points = [q0, q1, q2, qNm2, qNm1, qN];
max_dist_fixed = max(vecnorm(fixed_points - q0));
fprintf('Max distance of fixed control points from start: %.3f m\n', max_dist_fixed);
if max_dist_fixed > params.Ra
    warning('Some fixed control points are outside Ra! This may cause infeasibility.');
else
    fprintf('All fixed control points are within Ra. Good!\n\n');
end

%% 7. Set Up the B-Spline Basis Matrices
M_bspline = (1/6) * [
    1,  4,  1, 0;
   -3,  0,  3, 0;
    3, -6,  3, 0;
   -1,  3, -3, 1
];

A_pos_bs = [
   -1/6,   0.5,   -0.5,   1/6;
    0.5,  -1.0,      0,   2/3;
   -0.5,   0.5,    0.5,   1/6;
    1/6,     0,      0,     0
];

%% 8. Define Optimization Variables
num_free_cps = N + 1 - 6;
fprintf('Optimization setup:\n');
fprintf('  Total control points: %d\n', N + 1);
fprintf('  Fixed control points: 6 (3 initial + 3 final)\n');
fprintf('  Free control points: %d\n', num_free_cps);
fprintf('  Decision variables: %d (3 coords x %d free CPs)\n\n', 3 * num_free_cps, num_free_cps);

%% 9. Initialize and Solve with Gurobi
if exist('gurobi', 'file') == 0
    gurobi_home = getenv('GUROBI_HOME');
    if ~isempty(gurobi_home)
        gurobi_matlab = fullfile(gurobi_home, 'matlab');
        if isfolder(gurobi_matlab)
            addpath(gurobi_matlab);
        end
    end
    if exist('gurobi_setup', 'file') == 2
        gurobi_setup;
    end
end
if exist('gurobi', 'file') == 0
    error('Gurobi MATLAB API not found. Run init or gurobi_setup first.');
end

% Initial guess: linear interpolation between q2 and qN-2
x0 = [];
for i = 1:num_free_cps
    t = i / (num_free_cps + 1);
    q_init = (1 - t) * q2 + t * qNm2;
    x0 = [x0; q_init];
end

model = buildGurobiModel(q0, q1, q2, qNm2, qNm1, qN, ...
    knots, N, p, num_segments, params.v_max, params.a_max, params.j_max, ...
    params.Ra, deltaT, A_pos_bs);
model.start = x0;

params_gurobi = struct();
params_gurobi.OutputFlag = 1;

fprintf('Starting optimization with Gurobi...\n');
tic;
result = gurobi(model, params_gurobi);
opt_time = toc;

if ~isfield(result, 'x')
    error('Gurobi failed (status: %s).', result.status);
end

x_opt = result.x;
fval = result.objval;

fprintf('\nOptimization completed in %.3f seconds\n', opt_time);
fprintf('Status: %s\n', result.status);
fprintf('Final cost: %.4f\n\n', fval);

%% 10. Extract Solution
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

%% 11. Convert to Piecewise Polynomial and Sample
pwp = struct();
pwp.times = knots(p+1:end-p);
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

t_samples = t_init:params.dc:t_final;
trajectory = zeros(3, length(t_samples));
velocity = zeros(3, length(t_samples));
acceleration = zeros(3, length(t_samples));

for i = 1:length(t_samples)
    [trajectory(:, i), velocity(:, i), acceleration(:, i)] = evalPwpFull(pwp, t_samples(i));
end

%% 12. Visualization
figure('Position', [50, 50, 1500, 800]);

subplot(2, 3, 1);
hold on;
plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'b-', 'LineWidth', 2.5);
plot3(q_opt(1,:), q_opt(2,:), q_opt(3,:), 'ro-', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot3(initial_state.pos(1), initial_state.pos(2), initial_state.pos(3), ...
    'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot3(target_goal(1), target_goal(2), target_goal(3), ...
    'mp', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'LineWidth', 2);
if ~isequal(target_goal, final_goal)
    plot3(final_goal(1), final_goal(2), final_goal(3), ...
        'c^', 'MarkerSize', 12, 'MarkerFaceColor', 'c', 'LineWidth', 2);
    plot3([target_goal(1), final_goal(1)], [target_goal(2), final_goal(2)], ...
        [target_goal(3), final_goal(3)], 'c--', 'LineWidth', 1);
end

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

subplot(2, 3, 2);
plot(t_samples, trajectory(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t_samples, trajectory(2,:), 'g-', 'LineWidth', 1.5);
plot(t_samples, trajectory(3,:), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Position [m]');
title('Position vs Time');
legend('X', 'Y', 'Z', 'Location', 'best');
grid on;

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

subplot(2, 3, 5);
dist_from_start = vecnorm(trajectory - q0);
plot(t_samples, dist_from_start, 'b-', 'LineWidth', 2);
yline(params.Ra, 'r--', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Distance from Start [m]');
title('Sphere Constraint Verification');
legend('Trajectory Distance', 'Ra (Planning Radius)', 'Location', 'best');
grid on;

if max(dist_from_start) <= params.Ra * 1.01
    text(t_final/2, params.Ra*0.5, 'CONSTRAINT SATISFIED', ...
        'Color', 'g', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center');
else
    text(t_final/2, params.Ra*0.5, 'CONSTRAINT VIOLATED!', ...
        'Color', 'r', 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'center');
end

subplot(2, 3, 6);
hold on;
plot(trajectory(1,:), trajectory(2,:), 'b-', 'LineWidth', 2.5);
plot(q_opt(1,:), q_opt(2,:), 'ro-', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(initial_state.pos(1), initial_state.pos(2), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot(target_goal(1), target_goal(2), 'mp', 'MarkerSize', 15, 'MarkerFaceColor', 'm');
if ~isequal(target_goal, final_goal)
    plot(final_goal(1), final_goal(2), 'c^', 'MarkerSize', 12, 'MarkerFaceColor', 'c');
end
theta = linspace(0, 2*pi, 100);
plot(params.Ra*cos(theta) + q0(1), params.Ra*sin(theta) + q0(2), 'c-', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]');
title('Top-Down View (X-Y)');
grid on; axis equal;

sgtitle(sprintf('MADER Trajectory Optimization - %s Goal (Example 2, Gurobi)', goal_type), 'FontSize', 14);

%% 13. Verify Constraints
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

function model = buildGurobiModel(q0, q1, q2, qNm2, qNm1, qN, ...
    knots, N, p, num_segments, v_max, a_max, j_max, Ra, deltaT, A_pos_bs)
    free_indices = 4:(N-2);
    num_free_cps = numel(free_indices);
    nvar = 3 * num_free_cps;

    free_map = zeros(1, N + 1);
    free_map(free_indices) = 1:num_free_cps;

    q_fixed = nan(3, N + 1);
    q_fixed(:, 1) = q0;
    q_fixed(:, 2) = q1;
    q_fixed(:, 3) = q2;
    q_fixed(:, N-1) = qNm2;
    q_fixed(:, N) = qNm1;
    q_fixed(:, N+1) = qN;

    A = zeros(0, nvar);
    rhs = zeros(0, 1);

    % Velocity constraints
    for i = 3:N
        ci = p / (knots(i+p+1) - knots(i+1));
        for axis = 1:3
            [row, cst] = linDiff(axis, i+1, i, ci, free_map, q_fixed, nvar);
            [A, rhs] = addLeq(A, rhs, row, v_max(axis) - cst);
            [A, rhs] = addLeq(A, rhs, -row, v_max(axis) + cst);
        end
    end

    % Acceleration constraints
    for i = 2:N-2
        c1 = p / (knots(i+p+1) - knots(i+1));
        c2 = p / (knots(i+p+2) - knots(i+2));
        c3 = (p - 1) / (knots(i+p+1) - knots(i+2));
        for axis = 1:3
            [row1, cst1] = linDiff(axis, i+2, i+1, c2, free_map, q_fixed, nvar);
            [row0, cst0] = linDiff(axis, i+1, i, c1, free_map, q_fixed, nvar);
            row = c3 * (row1 - row0);
            cst = c3 * (cst1 - cst0);
            [A, rhs] = addLeq(A, rhs, row, a_max(axis) - cst);
            [A, rhs] = addLeq(A, rhs, -row, a_max(axis) + cst);
        end
    end

    % Jerk constraints
    tmp = [6; 0; 0; 0] / (deltaT^3);
    weights = (A_pos_bs * tmp).';
    for i = 1:num_segments
        for axis = 1:3
            [row, cst] = linCombo(axis, i:(i+3), weights, free_map, q_fixed, nvar);
            [A, rhs] = addLeq(A, rhs, row, j_max(axis) - cst);
            [A, rhs] = addLeq(A, rhs, -row, j_max(axis) + cst);
        end
    end

    % Objective: sum of squared jerk (same as solver_gurobi.cpp)
    Q = zeros(nvar, nvar);
    c = zeros(nvar, 1);
    tmp = [6; 0; 0; 0];
    weights = (A_pos_bs * tmp).';
    for i = 1:num_segments
        for axis = 1:3
            [row, cst] = linCombo(axis, i:(i+3), weights, free_map, q_fixed, nvar);
            Q = Q + 2 * (row' * row);
            c = c + 2 * cst * row';
        end
    end

    % Sphere constraints: ||q_i - q0||^2 <= Ra^2 for free control points
    qc = struct('Qc', {}, 'q', {}, 'rhs', {}, 'sense', {});
    for idx = free_indices
        slot = free_map(idx);
        var_idx = (slot - 1) * 3 + (1:3);
        Qc = sparse(var_idx, var_idx, 1, nvar, nvar);
        q = zeros(nvar, 1);
        q(var_idx) = -2 * q0;
        rhs_q = Ra^2 - sum(q0.^2);
        qc(end+1) = struct('Qc', Qc, 'q', q, 'rhs', rhs_q, 'sense', '<');
    end

    model = struct();
    model.A = sparse(A);
    model.rhs = rhs;
    model.sense = repmat('<', size(A, 1), 1);
    model.Q = sparse(Q);
    model.obj = c;
    model.modelsense = 'min';
    model.lb = -inf(nvar, 1);
    model.ub = inf(nvar, 1);
    if ~isempty(qc)
        model.quadcon = qc;
    end
end

function [A, rhs] = addLeq(A, rhs, row, bound)
    A(end+1, :) = row;
    rhs(end+1, 1) = bound;
end

function [row, cst] = linDiff(axis, idx1, idx0, coeff, free_map, q_fixed, nvar)
    [r1, c1] = cpExpr(axis, idx1, free_map, q_fixed, nvar);
    [r0, c0] = cpExpr(axis, idx0, free_map, q_fixed, nvar);
    row = coeff * (r1 - r0);
    cst = coeff * (c1 - c0);
end

function [row, cst] = linCombo(axis, idxs, weights, free_map, q_fixed, nvar)
    row = zeros(1, nvar);
    cst = 0;
    for k = 1:numel(idxs)
        [r, c] = cpExpr(axis, idxs(k), free_map, q_fixed, nvar);
        row = row + weights(k) * r;
        cst = cst + weights(k) * c;
    end
end

function [row, cst] = cpExpr(axis, idx, free_map, q_fixed, nvar)
    row = zeros(1, nvar);
    slot = free_map(idx);
    if slot > 0
        row((slot - 1) * 3 + axis) = 1;
        cst = 0;
    else
        cst = q_fixed(axis, idx);
    end
end

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

function s = conditional(cond, true_str, false_str)
    if cond, s = true_str; else, s = false_str; end
end
