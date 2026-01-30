%% MADER MATLAB Example 3 (Gurobi): Full Planner with Obstacles
% =========================================================================
% Gurobi-based QCQP version of Example 3.
% Uses fixed separating planes (computed geometrically) for collision
% constraints instead of fmincon's nonlinear distance-to-box constraints.
% =========================================================================

clear; clc; close all;

fprintf('=========================================================\n');
fprintf('   MADER Full Trajectory Planner (MATLAB Version)\n');
fprintf('   Example 3: Planning with Obstacles (Gurobi)\n');
fprintf('=========================================================\n\n');

%% 1. Problem Setup
params = struct();
params.v_max = [2.0; 2.0; 2.0];
params.a_max = [3.0; 3.0; 3.0];
params.j_max = [10.0; 10.0; 10.0];
params.num_pol = 8;
params.deg_pol = 3;
params.weight = 100.0;
params.Ra = 6.0;
params.dc = 0.02;
params.drone_radius = 0.20;
params.a_star_samples = [5, 5, 3];
params.a_star_bias = 1.0;

fprintf('Parameters loaded:\n');
fprintf('  v_max = [%.1f, %.1f, %.1f] m/s\n', params.v_max);
fprintf('  a_max = [%.1f, %.1f, %.1f] m/s^2\n', params.a_max);
fprintf('  Ra = %.1f m (planning horizon)\n', params.Ra);
fprintf('  drone_radius = %.2f m\n\n', params.drone_radius);

%% 2. Define Initial and Final States
initial_state = struct();
initial_state.pos = [-4; 0; 0];
initial_state.vel = [0; 0; 0];
initial_state.accel = [0; 0; 0];

final_goal = [12; 2; 1];

fprintf('Initial position: [%.1f, %.1f, %.1f]\n', initial_state.pos);
fprintf('Final goal:       [%.1f, %.1f, %.1f]\n', final_goal);

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
fprintf('Creating obstacles...\n');
obstacles = {};

obs1_center = [0; 0; 0];
obs1_size = [1.5; 1.5; 2.0];
obstacles{1} = createBoxVertices(obs1_center, obs1_size);

obs2_center = [-2; 1.5; 0];
obs2_size = [1.0; 1.0; 2.0];
obstacles{2} = createBoxVertices(obs2_center, obs2_size);

obs3_center = [2; -1.2; 0];
obs3_size = [1.2; 0.8; 2.0];
obstacles{3} = createBoxVertices(obs3_center, obs3_size);

obs4_center = [1.0; 2.0; 0.0];
obs4_size   = [1.0; 0.8; 1.5];
obstacles{4} = createBoxVertices(obs4_center, obs4_size);

obs5_center = [-3.0; -1.5; 0.0];
obs5_size   = [1.2; 1.2; 2.0];
obstacles{5} = createBoxVertices(obs5_center, obs5_size);

fprintf('  Created %d obstacles\n', length(obstacles));

obstacles_inflated = cell(size(obstacles));
for i = 1:length(obstacles)
    obstacles_inflated{i} = inflateObstacle(obstacles{i}, params.drone_radius);
end
fprintf('  Inflated by drone_radius = %.2f m\n\n', params.drone_radius);

%% 4. B-Spline Setup
p = params.deg_pol;
num_segments = params.num_pol;
M = num_segments + 2 * p;
N = M - p - 1;

dist = norm(final_state.pos - initial_state.pos);
t_init = 0;
t_final = dist / mean(params.v_max) * 3.0;
deltaT = (t_final - t_init) / num_segments;

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
p0 = initial_state.pos; v0 = initial_state.vel; a0 = initial_state.accel;
pf = final_state.pos; vf = final_state.vel; af = final_state.accel;

t1 = knots(2); t2 = knots(3); tpP1 = knots(p+2); t1PpP1 = knots(2+p+1);
tN = knots(N+1); tNm1 = knots(N); tNPp = knots(N+p+1); tNm1Pp = knots(N+p);

q0 = p0;
q1 = p0 + (tpP1 - t1) * v0 / p;
q2 = (p^2 * q1 - (t1PpP1 - t2) * (a0 * (t2 - tpP1) + v0) ...
    - p * (q1 + (t2 - t1PpP1) * v0)) / ((p - 1) * p);

qN = pf;
qNm1 = pf + (tN - tNPp) * vf / p;
qNm2 = (p^2 * qNm1 - (tNm1 - tNm1Pp) * (af * (tNm1Pp - tN) + vf) ...
    - p * (qNm1 + (tNm1Pp - tNm1) * vf)) / ((p - 1) * p);

%% 6. A* Search for Initial Guess
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
    num_free = N + 1 - 6;
    q_guess = [q0, q1, q2];
    for i = 1:num_free
        t = i / (num_free + 1);
        q_guess = [q_guess, (1-t)*q2 + t*qNm2];
    end
    q_guess = [q_guess, qNm2, qNm1, qN];
end

%% 7. Compute Separating Planes
fprintf('Computing separating planes...\n');
[planes_n, planes_d, planes_ok] = computeSeparatingPlanes(q_guess, obstacles_inflated, num_segments);
fprintf('  Generated %d planes (%d segments x %d obstacles)\n\n', ...
    length(planes_d), num_segments, length(obstacles));
num_planes_ok = sum(planes_ok);
if num_planes_ok < length(planes_ok)
    fprintf('  Skipping %d infeasible planes (no separation for seed)\n\n', ...
        length(planes_ok) - num_planes_ok);
end

%% 8. Trajectory Optimization (Gurobi QCQP)
fprintf('Running trajectory optimization with Gurobi...\n');

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

% Basis matrix (A_pos_bs from mader_types.hpp)
A_pos_bs = [-1/6, 0.5, -0.5, 1/6; 0.5, -1.0, 0, 2/3; -0.5, 0.5, 0.5, 1/6; 1/6, 0, 0, 0];
M_bspline = A_pos_bs;

num_free_cps = N + 1 - 6;
x0 = reshape(q_guess(:, 4:end-3), [], 1);

plane_margin = 0.0;
model = buildGurobiModel(q0, q1, q2, qNm2, qNm1, qN, ...
    knots, N, p, num_segments, params.v_max, params.a_max, params.j_max, ...
    params.Ra, deltaT, A_pos_bs, planes_n, planes_d, planes_ok, plane_margin, length(obstacles));
model.start = x0;

params_gurobi = struct();
params_gurobi.OutputFlag = 1;

tic;
result = gurobi(model, params_gurobi);
opt_time = toc;

if ~isfield(result, 'x')
    error('Gurobi failed (status: %s).', result.status);
end

x_opt = result.x;
fval = result.objval;

fprintf('\nOptimization completed:\n');
fprintf('  Time: %.3f s\n', opt_time);
fprintf('  Status: %s\n', result.status);
fprintf('  Cost: %.4f\n\n', fval);

%% 9. Extract and Sample Solution
q_free_opt = reshape(x_opt, 3, num_free_cps);
q_opt = [q0, q1, q2, q_free_opt, qNm2, qNm1, qN];

pwp = struct();
pwp.times = knots(p+1:end-p);
pwp.coeff_x = zeros(num_segments, 4);
pwp.coeff_y = zeros(num_segments, 4);
pwp.coeff_z = zeros(num_segments, 4);

for seg = 1:num_segments
    pwp.coeff_x(seg, :) = q_opt(1, seg:seg+3) * M_bspline;
    pwp.coeff_y(seg, :) = q_opt(2, seg:seg+3) * M_bspline;
    pwp.coeff_z(seg, :) = q_opt(3, seg:seg+3) * M_bspline;
end

t_samples = t_init:params.dc:t_final;
trajectory = zeros(3, length(t_samples));
for i = 1:length(t_samples)
    trajectory(:, i) = evalPwp(pwp, t_samples(i));
end

%% 10. Collision Check
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
figure('Position', [50, 50, 1400, 600]);

subplot(1, 2, 1);
hold on;
for i = 1:length(obstacles)
    plotConvexHull3D(obstacles{i}, [0.8, 0.2, 0.2], 0.6);
end
plot3(q_guess(1,:), q_guess(2,:), q_guess(3,:), 'g--', 'LineWidth', 1.5, 'DisplayName', 'A* Guess');
plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Optimized Traj');
plot3(q_opt(1,:), q_opt(2,:), q_opt(3,:), 'ko-', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Control Points');
plot3(initial_state.pos(1), initial_state.pos(2), initial_state.pos(3), ...
    'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2, 'DisplayName', 'Start');
plot3(target_goal(1), target_goal(2), target_goal(3), ...
    'mp', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'LineWidth', 2, 'DisplayName', 'Goal');

[sx, sy, sz] = sphere(20);
surf(params.Ra*sx + q0(1), params.Ra*sy + q0(2), params.Ra*sz + q0(3), ...
    'FaceAlpha', 0.05, 'EdgeColor', 'c', 'EdgeAlpha', 0.2, 'FaceColor', 'cyan', 'HandleVisibility', 'off');

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Trajectory View');
legend('Location', 'best');
grid on; axis equal;
xlim([-6, 6]); ylim([-4, 4]); zlim([-2, 2]);
view(45, 25);

subplot(1, 2, 2);
hold on;
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

theta = linspace(0, 2*pi, 100);
plot(params.Ra*cos(theta) + q0(1), params.Ra*sin(theta) + q0(2), 'c--', 'LineWidth', 2);

xlabel('X [m]'); ylabel('Y [m]');
title('Top-Down View (X-Y)');
grid on; axis equal;
xlim([-6, 6]); ylim([-4, 4]);

sgtitle('MADER Full Planner - Example 3 (Gurobi)', 'FontSize', 14);

%% ========================================================================
%                         HELPER FUNCTIONS
%  ========================================================================

function vertices = createBoxVertices(center, sz)
    hx = sz(1)/2; hy = sz(2)/2; hz = sz(3)/2;
    vertices = [
        center(1) + [-hx, hx, hx, -hx, -hx, hx, hx, -hx];
        center(2) + [-hy, -hy, hy, hy, -hy, -hy, hy, hy];
        center(3) + [-hz, -hz, -hz, -hz, hz, hz, hz, hz]
    ];
end

function vertices_inf = inflateObstacle(vertices, radius)
    center = mean(vertices, 2);
    directions = vertices - center;
    norms = vecnorm(directions);
    norms(norms < 1e-6) = 1;
    directions_normalized = directions ./ norms;
    vertices_inf = vertices + radius * directions_normalized;
end

function [q_guess, success] = aStarSearch(q0, q1, q2, qNm2, qNm1, qN, goal, ...
                                           N, p, knots, params, obstacles)
    num_free = N + 1 - 6;
    if num_free <= 0
        q_guess = [q0, q1, q2, qNm2, qNm1, qN];
        success = true;
        return;
    end

    start_pos = q2;
    end_pos = qNm2;

    direct_blocked = false;
    blocking_obs_centers = [];
    for obs_idx = 1:length(obstacles)
        obs = obstacles{obs_idx};
        obs_center = mean(obs, 2);
        obs_min = min(obs, [], 2);
        obs_max = max(obs, [], 2);

        margin = 0.5;
        if lineIntersectsBox(start_pos, end_pos, obs_min - margin, obs_max + margin)
            direct_blocked = true;
            blocking_obs_centers = [blocking_obs_centers, obs_center];
        end
    end

    if direct_blocked && ~isempty(blocking_obs_centers)
        path_dir = end_pos - start_pos;
        path_dir = path_dir / (norm(path_dir) + 1e-6);

        best_waypoints = [];
        best_score = inf;

        avoidance_dirs = [
            [0; 1; 0], [0; -1; 0], ...
            [0; 0; 1], [0; 0; -1], ...
            [0; 1; 1]/sqrt(2), [0; 1; -1]/sqrt(2), ...
            [0; -1; 1]/sqrt(2), [0; -1; -1]/sqrt(2)
        ];

        for dir_idx = 1:size(avoidance_dirs, 2)
            avoid_dir = avoidance_dirs(:, dir_idx);
            avoid_dir = avoid_dir - dot(avoid_dir, path_dir) * path_dir;
            if norm(avoid_dir) < 0.1
                continue;
            end
            avoid_dir = avoid_dir / norm(avoid_dir);

            max_offset = 0;
            for oc_idx = 1:size(blocking_obs_centers, 2)
                obs_c = blocking_obs_centers(:, oc_idx);
                for obs_idx = 1:length(obstacles)
                    obs = obstacles{obs_idx};
                    obs_projs = avoid_dir' * (obs - obs_c);
                    obs_extent = max(abs(obs_projs));
                    proj_dist = abs(dot(avoid_dir, obs_c - (start_pos + end_pos)/2));
                    required = obs_extent + 0.5 - proj_dist;
                    max_offset = max(max_offset, required);
                end
            end

            offset = max_offset + 0.3;
            mid_point = (start_pos + end_pos) / 2;
            waypoint = mid_point + offset * avoid_dir;

            collision = false;
            for obs_idx = 1:length(obstacles)
                if isInsideConvexHull(waypoint, obstacles{obs_idx})
                    collision = true;
                    break;
                end
            end

            if norm(waypoint - q0) > params.Ra
                collision = true;
            end

            if ~collision
                path_len = norm(waypoint - start_pos) + norm(end_pos - waypoint);
                score = path_len;

                if score < best_score
                    best_score = score;
                    best_waypoints = waypoint;
                end
            end
        end

        if ~isempty(best_waypoints)
            q_guess = generateSmoothPath(q0, q1, q2, qNm2, qNm1, qN, ...
                best_waypoints, num_free, knots, p, params, obstacles);
            success = true;
            return;
        end
    end

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
    q_path = [q0, q1, q2];

    start_pos = q2;
    end_pos = qNm2;

    for idx = 1:num_free
        alpha = idx / (num_free + 1);

        if alpha < 0.5
            t = alpha * 2;
            target = (1-t) * start_pos + t * waypoint;
        else
            t = (alpha - 0.5) * 2;
            target = (1-t) * waypoint + t * end_pos;
        end

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
    found = false;
    alt_q = target;

    angles = linspace(0, 2*pi, 13);
    z_offsets = [-1, 0, 1];

    best_candidate = [];
    best_dist_to_target = inf;

    for z_off = z_offsets
        for angle = angles(1:end-1)
            offset = [cos(angle); sin(angle); z_off * 0.5];
            offset = offset / norm(offset);

            candidate = current + offset .* max_step;

            if norm(candidate - q0) > Ra * 0.95
                continue;
            end

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
    if isInsideConvexHull(point, vertices)
        dist = -0.1;
    else
        dist = min(vecnorm(point - vertices));
    end
end

function [planes_n, planes_d, planes_ok] = computeSeparatingPlanes(q, obstacles, num_segments)
    planes_n = {};
    planes_d = [];
    planes_ok = [];

    for seg = 1:num_segments
        cps = q(:, seg:seg+3);
        cp_center = mean(cps, 2);

        for obs_idx = 1:length(obstacles)
            obs = obstacles{obs_idx};
            obs_center = mean(obs, 2);

            best_n = [];
            best_margin = -inf;

            test_dirs = [
                [1; 0; 0], [-1; 0; 0], [0; 1; 0], [0; -1; 0], [0; 0; 1], [0; 0; -1], ...
                [1; 1; 0]/sqrt(2), [1; -1; 0]/sqrt(2), [-1; 1; 0]/sqrt(2), [-1; -1; 0]/sqrt(2), ...
                cp_center - obs_center
            ];

            for dir_idx = 1:size(test_dirs, 2)
                n = test_dirs(:, dir_idx);
                n_norm = norm(n);
                if n_norm < 1e-6
                    continue;
                end
                n = n / n_norm;

                cp_projs = n' * cps;
                obs_projs = n' * obs;

                min_cp = min(cp_projs);
                max_obs = max(obs_projs);

                margin = min_cp - max_obs;

                if margin > best_margin
                    best_margin = margin;
                    best_n = n;
                end
            end

            if isempty(best_n) || best_margin < -1
                n = cp_center - obs_center;
                n_norm = norm(n);
                if n_norm > 1e-6
                    n = n / n_norm;
                else
                    n = [1; 0; 0];
                end
                best_n = n;
            end

            cp_projs = best_n' * cps;
            obs_projs = best_n' * obs;

            min_cp_proj = min(cp_projs);
            max_obs_proj = max(obs_projs);

            d = -max_obs_proj;
            plane_margin = 0.0;
            ok = (min_cp_proj + d) >= plane_margin;

            planes_n{end+1} = best_n;
            planes_d(end+1) = d;
            planes_ok(end+1) = ok;
        end
    end
end

function model = buildGurobiModel(q0, q1, q2, qNm2, qNm1, qN, ...
    knots, N, p, num_segments, v_max, a_max, j_max, Ra, deltaT, A_pos_bs, ...
    planes_n, planes_d, planes_ok, plane_margin, num_obstacles)

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

    % Plane collision constraints (fixed planes)
    for seg = 1:num_segments
        for obs_idx = 1:num_obstacles
            ip = (seg - 1) * num_obstacles + obs_idx;
            if ~planes_ok(ip)
                continue;
            end
            n = planes_n{ip};
            d = planes_d(ip);
            for u = 0:3
                cp_idx = seg + u;
                [row, cst] = dotExpr(n, cp_idx, free_map, q_fixed, nvar);
                cst = cst + d;
                [A, rhs] = addLeq(A, rhs, -row, cst - plane_margin);
            end
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

function [row, cst] = dotExpr(n, idx, free_map, q_fixed, nvar)
    row = zeros(1, nvar);
    cst = 0;
    for axis = 1:3
        [r, c] = cpExpr(axis, idx, free_map, q_fixed, nvar);
        row = row + n(axis) * r;
        cst = cst + n(axis) * c;
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
