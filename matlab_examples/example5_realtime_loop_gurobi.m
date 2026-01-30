%% MADER MATLAB Example 5 (Gurobi): Realtime Replanning Loop Simulation
% =========================================================================
% Simulates a fixed-rate replanning loop using the Gurobi QCQP planner
% from Example 3. This approximates the ROS timer callback behavior.
% =========================================================================

clear; clc; close all;

%% Simulation and planner parameters
params = struct();
% Match mader.yaml (single-agent)
params.v_max = [3.5; 3.5; 3.5];
params.a_max = [20.0; 20.0; 9.6];
params.j_max = [30.0; 30.0; 30.0];
params.x_min = -1e9;
params.x_max = 1e9;
params.y_min = -1e9;
params.y_max = 1e9;
params.z_min = -1e9;
params.z_max = 1e9;
params.num_pol = 4;
params.deg_pol = 3;
params.weight = 1000.0;  % C++ default (see mader.yaml)
params.Ra = 4.0;
params.dc = 0.01;
params.drone_radius = 0.05;
params.goal_radius = 0.15;
params.factor_alpha = 1.5;  % DeltaT = factor_alpha * states_last_replan
params.upper_bound_runtime = 0.35;  % [s]
params.lower_bound_runtime = 0.05;  % [s]
params.factor_alloc = 1.0;
params.factor_alloc_close = 1.5;
params.dist_factor_alloc_close = 2.0;
params.allow_infeasible_guess = false;  % C++ Gurobi returns false if A* fails
params.kappa = 0.4;  % fraction of runtime for A* (mader.yaml)
params.mu = 0.4;  % fraction of runtime for Gurobi (mader.yaml)
params.astar_min_runtime = 0.08;  % MATLAB needs a floor; C++ is faster
params.a_star_samp_x = 5;
params.a_star_samp_y = 5;
params.a_star_samp_z = 5;
params.a_star_fraction_voxel_size = 0.5;
params.a_star_bias = 1.0;
params.alpha_shrink = 0.95;
params.astar_goal_size = 0.05;
params.astar_bbox = 2000.0;
params.basis = 'MINVO';
params.astar_max_runtime = max(params.kappa * params.upper_bound_runtime, params.astar_min_runtime);
params.opt_max_runtime = params.mu * params.upper_bound_runtime;

params.replan_period = params.dc;
params.dt_sim = params.dc;
params.max_sim_time = 20.0;  % Increased from 30
params.plot_every = 5;
params.plane_margin = 1.0;  % Matches C++ epsilon in separator constraints
params.fallback_no_planes = false;  % Match C++: no "no-planes" fallback
params.validate_collision = true;  % C++ runs safetyCheckAfterOpt
params.plane_refine_iters = 1;
params.use_lp_planes = true;
params.use_minvo_planes = true;
params.use_astar_planes = true;
params.replan_Ra_scales = 1.0;
params.replan_num_pol_add = 0;
params.replan_plane_margins = params.plane_margin;

params.gurobi_output = 0;

fprintf('Realtime replanning loop (Gurobi)\n');
fprintf('  Replan period: %.2f s\n', params.replan_period);
fprintf('  Sim dt: %.2f s\n\n', params.dt_sim);

ensureGurobi();

%% Initial and goal states
initial_state = struct();
initial_state.pos = [-4; 0; 0];
initial_state.vel = [0; 0; 0];
initial_state.accel = [0; 0; 0];

final_goal = [3; 2; 1];

%% Obstacles
obstacles = {};
obstacles{1} = createBoxVertices([0; 0; 0], [1.5; 1.5; 2.0]);
obstacles{2} = createBoxVertices([-2; 1.5; 0], [1.0; 1.0; 2.0]);
obstacles{3} = createBoxVertices([2; -1.2; 0], [1.2; 0.8; 2.0]);
obstacles{4} = createBoxVertices([1.0; 2.0; 0.0], [1.0; 0.8; 1.5]);
obstacles{5} = createBoxVertices([-3.0; -1.5; 0.0], [1.2; 1.2; 2.0]);

obstacles_inflated = cell(size(obstacles));
for i = 1:length(obstacles)
    obstacles_inflated{i} = inflateObstacle(obstacles{i}, params.drone_radius);
end

%% Committed trajectory (deque-like structure as in C++)
% In C++, this is a deque<state> that gets appended/truncated
% Each entry: struct with pos, vel, accel, t (absolute time)
committed_traj = struct('states', [], 'dc', params.dc);

% Sample initial plan into committed trajectory
sim_time = 0;
last_replan_time = -inf;
plan_count = 0;
deltaT_states = max(1, round(params.replan_period / params.dc));

[plan, ok] = planSegmentGurobi(initial_state, final_goal, params, obstacles_inflated);
if ~ok
    error('Initial plan failed.');
end
plan.start_time = sim_time;
last_replan_time = sim_time;
plan_count = plan_count + 1;

% Convert PWP to discrete states and add to committed trajectory (like C++)
committed_traj = appendPlanToCommitted(committed_traj, plan, sim_time, params.dc);

history_t = [];
history_pos = [];

%% Visualization setup
figure('Position', [50, 50, 1300, 700]);

%% Replanning loop
step = 0;
done = false;
plot_enabled = true;
consecutive_failures = 0;
max_plan_age = 2.0;  % Maximum time to use an old plan before forcing a hover

while sim_time <= params.max_sim_time && ~done
    % Get current state from committed trajectory (like C++ getNextGoal)
    [pos, vel, accel, committed_traj] = getStateFromCommitted(committed_traj, sim_time);
    history_t(end+1, 1) = sim_time;
    history_pos(:, end+1) = pos;

    if norm(pos - final_goal) <= params.goal_radius
        done = true;
    end

    % Check if replanning is needed (like C++ isReplanningNeeded + replanCB)
    time_since_replan = sim_time - last_replan_time;
    remaining_traj_time = getCommittedRemainingTime(committed_traj, sim_time);

    need_replan = (time_since_replan >= params.replan_period) || ...
                  (remaining_traj_time < params.replan_period * 2);

    if need_replan
        % Extract point A from committed trajectory (like C++ replan function)
        % In C++: k_index = plan_.size() - 1 - k_index_end, A = plan_.get(k_index)
        % We look ahead by deltaT_states (number of samples)
        deltaT_states = min(max(deltaT_states, params.lower_bound_runtime / params.dc), ...
                            params.upper_bound_runtime / params.dc);
        [A_pos, A_vel, A_accel, k_index, t_start] = getPointAFromCommitted(committed_traj, sim_time, deltaT_states);

        A_state = struct('pos', A_pos, 'vel', A_vel, 'accel', A_accel);

        plan_size = length(committed_traj.states);
        k_index_end = max(plan_size - k_index, 0);
        if k_index_end ~= 0
            runtime_snlopt = (k_index - 1) * params.dc;
        else
            runtime_snlopt = params.upper_bound_runtime;
        end
        runtime_snlopt = min(max(runtime_snlopt, params.lower_bound_runtime), params.upper_bound_runtime);
        params.astar_max_runtime = max(params.kappa * runtime_snlopt, params.astar_min_runtime);
        params.opt_max_runtime = params.mu * runtime_snlopt;

        fprintf('[t=%.2f] Replanning from A (t_start=%.2f): pos=[%.2f, %.2f, %.2f], vel=[%.2f,%.2f,%.2f], dist_to_goal=%.2f\n', ...
            sim_time, t_start, A_pos, A_vel, norm(A_pos - final_goal));

        replan_tic = tic;
        [new_plan, ok] = planSegmentGurobi(A_state, final_goal, params, obstacles_inflated);
        replan_runtime = toc(replan_tic);
        states_last_replan = ceil(replan_runtime / params.dc);
        deltaT_states = max(params.factor_alpha * states_last_replan, 1.0);
        deltaT_states = min(deltaT_states, 2.0 / params.dc);

        if ok
            % Truncate committed trajectory up to point A (exclusive), then append new plan
            % Use k_index-1 to avoid duplicate state at connection point
            committed_traj = truncateCommittedAt(committed_traj, k_index - 1);

            % Append new plan to committed trajectory
            % This is like C++: for (i = 0; i < solution.size(); i++) plan_.push_back(...)
            new_plan.start_time = t_start;
            committed_traj = appendPlanToCommitted(committed_traj, new_plan, t_start, params.dc);

            plan = new_plan;  % Keep for visualization
            plan.start_time = t_start;
            last_replan_time = sim_time;
            plan_count = plan_count + 1;
            consecutive_failures = 0;

            % Debug output
            qN_opt = plan.q_opt(:, end);
            [end_pos, ~, ~] = evalPwpFull(plan.pwp, plan.duration);
            fprintf('  -> Success! duration=%.2f s, qN=[%.2f,%.2f,%.2f], endPos=[%.2f,%.2f,%.2f], target=[%.2f,%.2f,%.2f]\n', ...
                plan.duration, qN_opt, end_pos, plan.target_goal);
        else
            consecutive_failures = consecutive_failures + 1;
            fprintf('  -> FAILED! Holding committed trajectory (remaining=%.1fs, failures=%d).\n', remaining_traj_time, consecutive_failures);
            last_replan_time = sim_time;
        end
    end

    if plot_enabled && mod(step, params.plot_every) == 0
        try
            clf;
            subplot(1, 2, 1);
            hold on;
            for i = 1:length(obstacles)
                plotConvexHull3D(obstacles{i}, [0.8, 0.2, 0.2], 0.6);
            end
            plot3(history_pos(1,:), history_pos(2,:), history_pos(3,:), 'b-', 'LineWidth', 2.0);
            plot3(pos(1), pos(2), pos(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
            plot3(final_goal(1), final_goal(2), final_goal(3), 'mp', 'MarkerSize', 12, 'MarkerFaceColor', 'm');

            % Plot future trajectory from committed trajectory (more accurate)
            future_pos = sampleFutureFromCommitted(committed_traj, sim_time);
            if ~isempty(future_pos)
                plot3(future_pos(1,:), future_pos(2,:), future_pos(3,:), 'g--', 'LineWidth', 1.0);
            end

            [sx, sy, sz] = sphere(20);
            surf(params.Ra*sx + pos(1), params.Ra*sy + pos(2), params.Ra*sz + pos(3), ...
                'FaceAlpha', 0.05, 'EdgeColor', 'c', 'EdgeAlpha', 0.2, 'FaceColor', 'cyan', 'HandleVisibility', 'off');

            xlabel('X'); ylabel('Y'); zlabel('Z');
            title(sprintf('Realtime Loop (t=%.2f s, plans=%d)', sim_time, plan_count));
            grid on; axis equal;
            xlim([-6, 6]); ylim([-4, 4]); zlim([-2, 2]);
            view(45, 25);

            subplot(1, 2, 2);
            plot(history_t, history_pos(1,:), 'r-', 'LineWidth', 1.5); hold on;
            plot(history_t, history_pos(2,:), 'g-', 'LineWidth', 1.5);
            plot(history_t, history_pos(3,:), 'b-', 'LineWidth', 1.5);
            xlabel('Time [s]'); ylabel('Position [m]');
            title('Position vs Time');
            legend('X', 'Y', 'Z', 'Location', 'best');
            grid on;

            drawnow limitrate;
        catch plot_err
            plot_enabled = false;
            fprintf('Plotting disabled after error: %s\n', plot_err.message);
        end
    end

    sim_time = sim_time + params.dt_sim;
    step = step + 1;
end

fprintf('Done. Total replans: %d, final time: %.2f s\n', plan_count, sim_time);

%% Local functions

function ensureGurobi()
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
end

function [plan, ok] = planSegmentGurobi(initial_state, final_goal, params, obstacles_inflated)
    ra_scales = getFieldDefault(params, 'replan_Ra_scales', 1.0);
    num_pol_add = getFieldDefault(params, 'replan_num_pol_add', 0);
    plane_margins = getFieldDefault(params, 'replan_plane_margins', params.plane_margin);

    attempts = max([numel(ra_scales), numel(num_pol_add), numel(plane_margins)]);
    for k = 1:attempts
        params_local = params;
        params_local.Ra = params.Ra * ra_scales(min(k, numel(ra_scales)));
        params_local.num_pol = params.num_pol + num_pol_add(min(k, numel(num_pol_add)));
        params_local.plane_margin = plane_margins(min(k, numel(plane_margins)));

        [plan, ok] = planSegmentGurobiOnce(initial_state, final_goal, params_local, obstacles_inflated);
        if ok
            if k > 1
                fprintf('Replan succeeded on attempt %d.\n', k);
            end
            return;
        end
    end
    ok = false;
    plan = struct();
end

function [plan, ok] = planSegmentGurobiOnce(initial_state, final_goal, params, obstacles_inflated)
    ok = false;
    plan = struct();

    dist_to_goal = norm(final_goal - initial_state.pos);

    if dist_to_goal < 1e-3
        target_goal = final_goal;
    else
        ra = min(dist_to_goal - 0.001, params.Ra);
        direction = (final_goal - initial_state.pos) / dist_to_goal;
        target_goal = initial_state.pos + ra * direction;
    end

    p = params.deg_pol;
    num_segments = params.num_pol;
    M = num_segments + 2 * p;
    N = M - p - 1;
    num_obstacles = length(obstacles_inflated);

    t_init = 0;

    factor_alloc = params.factor_alloc;
    if dist_to_goal < params.dist_factor_alloc_close
        factor_alloc = params.factor_alloc_close;
    end

    time_allocated = factor_alloc * getMinTimeDoubleIntegrator3D( ...
        initial_state.pos, initial_state.vel, target_goal, [0; 0; 0], params.v_max, params.a_max);

    deltaT = time_allocated / num_segments;

    % Saturate deltaT to satisfy v1 constraints (matches solver_gurobi.cpp)
    v0 = clampVec(initial_state.vel, -params.v_max, params.v_max);
    a0 = clampVec(initial_state.accel, -params.a_max, params.a_max);
    for axis = 1:3
        if abs(a0(axis)) > 1e-7
            upper_bound = ((p - 1) * (sign(a0(axis)) * params.v_max(axis) - v0(axis)) / a0(axis));
            lower_bound = ((p - 1) * (-sign(a0(axis)) * params.v_max(axis) - v0(axis)) / a0(axis));
            if upper_bound <= 0
                return;
            end
            deltaT = min(max(deltaT, max(0.0, lower_bound)), upper_bound);
        end
    end

    t_final = t_init + num_segments * deltaT;

    knots = zeros(1, M + 1);
    knots(1:p+1) = t_init;
    for i = p+2 : M-p
        knots(i) = knots(i-1) + deltaT;
    end
    knots(M-p+1:M+1) = t_final;

    p0 = initial_state.pos; v0 = clampVec(initial_state.vel, -params.v_max, params.v_max);
    a0 = clampVec(initial_state.accel, -params.a_max, params.a_max);
    pf = target_goal; vf = [0; 0; 0]; af = [0; 0; 0];

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

    [q_guess, astar_planes_n, astar_planes_d, astar_success] = aStarSearch(q0, q1, q2, qNm2, qNm1, qN, ...
        pf, N, p, knots, params, obstacles_inflated);

    if ~astar_success
        if getFieldDefault(params, 'allow_infeasible_guess', false)
            num_free = N + 1 - 3;
            q_guess = [q0, q1, q2];
            for i = 1:num_free
                t = i / (num_free + 1);
                q_guess = [q_guess, (1-t)*q2 + t*qN];
            end
            astar_planes_n = {};
            astar_planes_d = [];
        else
            fprintf('A* failed to find a feasible guess.\n');
            return;
        end
    end

    M_bspline = (1/6) * [
        1,  4,  1, 0;
       -3,  0,  3, 0;
        3, -6,  3, 0;
       -1,  3, -3, 1
    ];

    num_free_cps = N + 1 - 3;
    x0 = reshape(q_guess(:, 4:end), [], 1);

    params_gurobi = struct();
    params_gurobi.OutputFlag = params.gurobi_output;
    if isfield(params, 'opt_max_runtime') && isfinite(params.opt_max_runtime) && params.opt_max_runtime > 0
        params_gurobi.TimeLimit = params.opt_max_runtime;
    end

    refine_iters = getFieldDefault(params, 'plane_refine_iters', 2);
    use_astar_planes = getFieldDefault(params, 'use_astar_planes', true);
    q_seed = q_guess;

    for refine = 1:refine_iters
        if use_astar_planes && refine == 1 && ~isempty(astar_planes_n)
            planes_n = astar_planes_n;
            planes_d = astar_planes_d;
            planes_margins = [];
            planes_ok = (numel(planes_n) == num_segments * num_obstacles);
            plane_margin = params.plane_margin;
        elseif params.use_lp_planes
            [planes_n, planes_d, planes_margins, planes_ok] = computeSeparatingPlanesLP( ...
                q_seed, obstacles_inflated, num_segments, params.gurobi_output, params.use_minvo_planes);
            plane_margin = params.plane_margin;
        else
            [planes_n, planes_d] = computeSeparatingPlanesHeuristic(q_seed, obstacles_inflated, num_segments, params.use_minvo_planes);
            planes_ok = true;
            planes_margins = [];
            plane_margin = params.plane_margin;
        end

        if ~planes_ok
            fprintf('Plane generation failed (refine %d).\n', refine);
            continue;
        end

        model = buildGurobiModel(q0, q1, q2, qNm2, qNm1, qN, pf, params.weight, ...
            knots, N, p, num_segments, params.v_max, params.a_max, params.j_max, ...
            params.Ra, deltaT, planes_n, planes_d, planes_margins, plane_margin, ...
            num_obstacles, params.use_minvo_planes);
        model.start = x0;

        result = gurobi(model, params_gurobi);
        if ~isfield(result, 'x') && params.fallback_no_planes
            model = buildGurobiModel(q0, q1, q2, qNm2, qNm1, qN, pf, params.weight, ...
                knots, N, p, num_segments, params.v_max, params.a_max, params.j_max, ...
                params.Ra, deltaT, {}, [], [], 0.0, 0, params.use_minvo_planes);
            model.start = x0;
            result = gurobi(model, params_gurobi);
        end
        if ~isfield(result, 'x')
            continue;
        end
        if ~isfield(result, 'status')
            continue;
        end
        allowed_status = {'OPTIMAL','TIME_LIMIT','ITERATION_LIMIT','NODE_LIMIT','SOLUTION_LIMIT','USER_OBJ_LIMIT'};
        if ~any(strcmpi(result.status, allowed_status))
            continue;
        end

        x_opt = result.x;
        if any(~isfinite(x_opt)) || numel(x_opt) ~= numel(x0)
            continue;
        end
        q_free_opt = reshape(x_opt, 3, num_free_cps);
        q_opt = [q0, q1, q2, q_free_opt];

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

        if ~planIsNumericallySane(q_opt, q0, params.Ra, num_segments, params.use_minvo_planes)
            fprintf('Plan rejected: sphere constraint violation / numeric instability (refine %d).\n', refine);
            q_seed = q_opt;
            x0 = x_opt;
            continue;
        end

        if params.validate_collision
            safe = trajectoryIsSafe(pwp, t_final, obstacles_inflated, max(params.dc, 0.02));
            if safe
                plan.pwp = pwp;
                plan.duration = t_final;
                plan.q_opt = q_opt;
                plan.target_goal = target_goal;
                ok = true;
                return;
            else
                fprintf('Plan rejected: collision detected (refine %d).\n', refine);
            end
        else
            plan.pwp = pwp;
            plan.duration = t_final;
            plan.q_opt = q_opt;
            plan.target_goal = target_goal;
            ok = true;
            return;
        end

        q_seed = q_opt;
        x0 = x_opt;
    end
end

function [pos, vel, accel] = samplePlan(plan, sim_time)
    t_local = sim_time - plan.start_time;
    if t_local < 0
        t_local = 0;
    end
    if t_local > plan.duration
        t_local = plan.duration;
    end
    [pos, vel, accel] = evalPwpFull(plan.pwp, t_local);
end

function plan = createHoverPlan(current_state, params)
    % Create a simple stopping trajectory that decelerates to hover
    p0 = current_state.pos;
    v0 = current_state.vel;

    % Estimate stopping time based on current velocity and max deceleration
    v_mag = norm(v0);
    a_max_avg = mean(params.a_max);
    t_stop = max(0.5, v_mag / a_max_avg * 1.5);  % At least 0.5s

    % Final position: current position + some overshoot based on velocity
    pf = p0 + v0 * t_stop * 0.3;  % Drift a bit while stopping

    % Create a simple polynomial trajectory: cubic for position
    % Constraints: p(0)=p0, p'(0)=v0, p(T)=pf, p'(T)=0

    num_segments = 4;
    pwp = struct();
    pwp.times = linspace(0, t_stop, num_segments + 1);
    pwp.coeff_x = zeros(num_segments, 4);
    pwp.coeff_y = zeros(num_segments, 4);
    pwp.coeff_z = zeros(num_segments, 4);

    dt_seg = t_stop / num_segments;

    for seg = 1:num_segments
        % Linear interpolation factor
        alpha0 = (seg - 1) / num_segments;
        alpha1 = seg / num_segments;

        % Position at segment boundaries (linear interp from p0 to pf)
        ps0 = p0 + alpha0 * (pf - p0);
        ps1 = p0 + alpha1 * (pf - p0);

        % Velocity decays linearly from v0 to 0
        vs0 = v0 * (1 - alpha0);
        vs1 = v0 * (1 - alpha1);

        % Cubic coefficients for each axis
        for axis = 1:3
            % Solve cubic: p(0)=ps0, p'(0)=vs0, p(dt)=ps1, p'(dt)=vs1
            A = [1, 0, 0, 0;
                 0, 1, 0, 0;
                 1, dt_seg, dt_seg^2, dt_seg^3;
                 0, 1, 2*dt_seg, 3*dt_seg^2];
            b = [ps0(axis); vs0(axis); ps1(axis); vs1(axis)];
            coeffs = A \ b;
            if axis == 1
                pwp.coeff_x(seg, :) = coeffs';
            elseif axis == 2
                pwp.coeff_y(seg, :) = coeffs';
            else
                pwp.coeff_z(seg, :) = coeffs';
            end
        end
    end

    plan = struct();
    plan.pwp = pwp;
    plan.duration = t_stop;
    plan.q_opt = [];  % Not used for hover plan
    plan.target_goal = pf;
end

%% Committed trajectory functions (mirrors C++ committedTrajectory class)
% These functions implement the deque-like committed trajectory management
% from mader.cpp and mader_types.hpp

function committed = appendPlanToCommitted(committed, plan, t_start, dc)
    % Append a plan's sampled states to the committed trajectory
    % Like C++: for (i = 0; i < solution.size(); i++) plan_.push_back(solution[i])

    t_samples = 0:dc:plan.duration;
    num_samples = length(t_samples);

    if isempty(committed.states)
        committed.states = struct('pos', {}, 'vel', {}, 'accel', {}, 't', {});
        start_idx = 1;
    else
        % Check if first sample would be a duplicate (same time as last state)
        last_time = committed.states(end).t;
        first_abs_time = t_start + t_samples(1);
        if abs(first_abs_time - last_time) < dc * 0.5
            start_idx = 2;  % Skip first sample to avoid duplicate
        else
            start_idx = 1;
        end
    end

    for i = start_idx:num_samples
        t_local = t_samples(i);
        [pos, vel, accel] = evalPwpFull(plan.pwp, t_local);

        state = struct();
        state.pos = pos;
        state.vel = vel;
        state.accel = accel;
        state.t = t_start + t_local;  % Absolute time

        committed.states(end+1) = state;
    end
end

function [pos, vel, accel, committed] = getStateFromCommitted(committed, ~)
    % Get current state from committed trajectory (like C++ getNextGoal)
    % Advance by popping one state each tick (dc should match loop rate)

    if isempty(committed.states)
        pos = [0; 0; 0];
        vel = [0; 0; 0];
        accel = [0; 0; 0];
        return;
    end

    s = committed.states(1);
    pos = s.pos;
    vel = s.vel;
    accel = s.accel;

    if length(committed.states) > 1
        committed.states = committed.states(2:end);
    end
end

function remaining_time = getCommittedRemainingTime(committed, sim_time)
    % Get remaining time in committed trajectory
    if isempty(committed.states)
        remaining_time = 0;
        return;
    end
    remaining_time = max(0, (length(committed.states) - 1) * committed.dc);
end

function [A_pos, A_vel, A_accel, k_index, t_start] = getPointAFromCommitted(committed, sim_time, deltaT_states)
    % Extract point A from committed trajectory (index-based, like C++)
    % deltaT_states is the look-ahead in discrete samples (not seconds).

    if isempty(committed.states)
        A_pos = [0; 0; 0];
        A_vel = [0; 0; 0];
        A_accel = [0; 0; 0];
        k_index = 1;
        t_start = 0.0;
        return;
    end

    plan_size = length(committed.states);
    k_index_end = max(floor(plan_size - deltaT_states), 0);
    if plan_size < 5
        k_index_end = 0;
    end

    % C++ uses 0-based: k_index = plan_size - 1 - k_index_end.
    % MATLAB is 1-based, so the equivalent is:
    k_index = plan_size - k_index_end;
    k_index = max(1, min(k_index, plan_size));

    s = committed.states(k_index);
    A_pos = s.pos;
    A_vel = s.vel;
    A_accel = s.accel;
    t_start = sim_time + (k_index - 1) * committed.dc;
end

function committed = truncateCommittedAt(committed, k_index)
    % Truncate committed trajectory at index k_index
    % Like C++: plan_.erase(plan_.end() - k_index_end - 1, plan_.end())

    if k_index <= 0
        committed.states = [];
    elseif k_index <= length(committed.states)
        committed.states = committed.states(1:k_index);
    end
end

function future_pos = sampleFutureFromCommitted(committed, sim_time)
    % Sample future positions from committed trajectory for visualization
    if isempty(committed.states)
        future_pos = [];
        return;
    end
    future_pos = zeros(3, length(committed.states));
    for i = 1:length(committed.states)
        future_pos(:, i) = committed.states(i).pos;
    end
end

function [pos] = sampleFuture(plan, sim_time, dt)
    t_local = sim_time - plan.start_time;
    if t_local < 0
        t_local = 0;
    end
    t_samples = t_local:dt:plan.duration;
    pos = zeros(3, length(t_samples));
    for i = 1:length(t_samples)
        pos(:, i) = evalPwp(plan.pwp, t_samples(i));
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

function safe = trajectoryIsSafe(pwp, duration, obstacles, dt_sample)
    safe = true;
    t_samples = 0:dt_sample:duration;
    for i = 1:length(t_samples)
        pt = evalPwp(pwp, t_samples(i));
        for j = 1:length(obstacles)
            if isInsideConvexHull(pt, obstacles{j})
                safe = false;
                return;
            end
        end
    end
end

function ok = planIsNumericallySane(q_opt, q0, Ra, num_segments, use_minvo)
    % Ensure sphere constraints are respected (as in C++) and avoid numeric explosions
    tol = 1e-3;
    for seg = 1:num_segments
        if use_minvo
            cps = transformBsplineToMinvo(q_opt(:, seg:seg+3), seg, num_segments);
        else
            cps = q_opt(:, seg:seg+3);
        end
        for u = 1:4
            if any(~isfinite(cps(:, u)))
                ok = false;
                return;
            end
            if norm(cps(:, u) - q0) > (Ra + tol)
                ok = false;
                return;
            end
        end
    end
    ok = true;
end


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

function [q_guess, planes_n, planes_d, success] = aStarSearch(q0, q1, q2, qNm2, qNm1, qN, goal, ...
                                           N, p, knots, params, obstacles)
    q_guess = [];
    planes_n = {};
    planes_d = [];
    success = false;

    num_segments = params.num_pol;
    num_obstacles = length(obstacles);

    if N <= 2
        q_guess = [q0, q1, q2, qNm2, qNm1, qN];
        success = true;
        return;
    end

    goal_size = getFieldDefault(params, 'astar_goal_size', 0.05);
    bias = getFieldDefault(params, 'a_star_bias', 1.0);
    max_runtime = getFieldDefault(params, 'astar_max_runtime', 0.1);
    num_samples_x = max(1, round(getFieldDefault(params, 'a_star_samp_x', 5)));
    num_samples_y = max(1, round(getFieldDefault(params, 'a_star_samp_y', 5)));
    num_samples_z = max(1, round(getFieldDefault(params, 'a_star_samp_z', 5)));
    if mod(num_samples_x, 2) == 0, num_samples_x = num_samples_x + 1; end
    if mod(num_samples_y, 2) == 0, num_samples_y = num_samples_y + 1; end
    if mod(num_samples_z, 2) == 0, num_samples_z = num_samples_z + 1; end
    fraction_voxel = getFieldDefault(params, 'a_star_fraction_voxel_size', 0.5);
    alpha_shrink = getFieldDefault(params, 'alpha_shrink', 0.95);
    use_minvo = getFieldDefault(params, 'use_minvo_planes', true);
    bbox = getFieldDefault(params, 'astar_bbox', 2000.0);

    combos = astarSampleCombos(num_samples_x, num_samples_y, num_samples_z);

    [voxel_size, orig] = astarComputeVoxelSize(q0, q1, q2, ...
        num_samples_x, num_samples_y, num_samples_z, fraction_voxel, ...
        params.v_max, params.a_max, knots, p, N, alpha_shrink, use_minvo, bbox);
    if ~isfinite(voxel_size) || voxel_size <= 1e-6
        voxel_size = 0.25;
    end

    nodes = struct('qi', {}, 'prev', {}, 'g', {}, 'h', {}, 'index', {});
    nodes(1).qi = q2;
    nodes(1).prev = 0;
    nodes(1).g = 0;
    nodes(1).h = norm(q2 - goal);
    nodes(1).index = 2;

    open = 1;
    visited = containers.Map('KeyType', 'char', 'ValueType', 'logical');

    closest_idx = 1;
    closest_dist = nodes(1).h;
    complete_closest_idx = 0;
    complete_closest_dist = inf;

    status = 0;
    goal_idx = 0;
    t_astar = tic;

    while ~isempty(open)
        if toc(t_astar) > max_runtime
            status = 1;
            break;
        end

        [best_pos, best_idx] = astarPopBest(open, nodes, bias);
        current_idx = open(best_pos);
        open(best_pos) = [];
        current = nodes(current_idx);

        dist = norm(current.qi - goal);
        if dist < closest_dist
            closest_dist = dist;
            closest_idx = current_idx;
        end
        if current.index == (N - 2) && dist < complete_closest_dist - 1e-4
            complete_closest_dist = dist;
            complete_closest_idx = current_idx;
        end

        key = astarVoxelKey(current.qi, orig, voxel_size);
        if isKey(visited, key)
            continue;
        end
        if norm(current.qi - q0) >= params.Ra
            continue;
        end

        if current.index < (N - 2)
            [ok_bounds, constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU] = ...
                astarComputeBounds(current.index, nodes, current_idx, q0, q1, ...
                params.v_max, params.a_max, knots, p, N, alpha_shrink, use_minvo);
            if ~ok_bounds
                continue;
            end
        end

        if astarCollides(nodes, current_idx, q0, q1, obstacles, use_minvo, num_segments, N, params.gurobi_output)
            continue;
        end

        visited(key) = true;

        if current.index == (N - 2) && dist < goal_size
            status = 2;
            goal_idx = current_idx;
            break;
        end

        if current.index == (N - 2)
            continue;
        end

        [nodes, open] = astarExpandNeighbors(nodes, open, current_idx, goal, combos, ...
            constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU, ...
            knots, p, params, q0, num_samples_x, num_samples_y, num_samples_z);
    end

    if status == 2
        best_idx = goal_idx;
    elseif complete_closest_idx ~= 0
        best_idx = complete_closest_idx;
    elseif closest_idx ~= 0
        best_idx = closest_idx;
    else
        q_guess = [];
        return;
    end

    while nodes(best_idx).index < (N - 2)
        new_node.qi = nodes(best_idx).qi;
        new_node.prev = best_idx;
        new_node.g = nodes(best_idx).g;
        new_node.h = norm(new_node.qi - goal);
        new_node.index = nodes(best_idx).index + 1;
        nodes(end+1) = new_node;
        best_idx = numel(nodes);
    end

    q_guess = astarRecoverPath(nodes, best_idx, q0, q1);
    [success, planes_n, planes_d] = astarCheckFeasAndFillND(q_guess, knots, p, ...
        params.v_max, params.a_max, obstacles, use_minvo, num_segments, params.gurobi_output);
    if success && num_obstacles == 0
        planes_n = {};
        planes_d = [];
    end
end

function [voxel_size, orig] = astarComputeVoxelSize(q0, q1, q2, ...
    num_samples_x, num_samples_y, num_samples_z, fraction_voxel_size, ...
    v_max, a_max, knots, p, N, alpha_shrink, use_minvo, bbox)

    [ok_bounds, constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU] = ...
        astarComputeUpperAndLowerConstraints(2, q0, q1, q2, v_max, a_max, knots, p, N, alpha_shrink, use_minvo);
    if ~ok_bounds
        voxel_size = 0.25;
        orig = q2 - [bbox; bbox; bbox] / 2;
        return;
    end

    min_voxel_size = inf;
    max_voxel_size = -inf;
    if num_samples_x <= 1, num_samples_x = 2; end
    if num_samples_y <= 1, num_samples_y = 2; end
    if num_samples_z <= 1, num_samples_z = 2; end

    delta_x = (constraint_xU - constraint_xL) / (num_samples_x - 1);
    delta_y = (constraint_yU - constraint_yL) / (num_samples_y - 1);
    delta_z = (constraint_zU - constraint_zL) / (num_samples_z - 1);

    for jx = 0:(num_samples_x - 1)
        for jy = 0:(num_samples_y - 1)
            for jz = 0:(num_samples_z - 1)
                vi = [constraint_xL + jx * delta_x; ...
                      constraint_yL + jy * delta_y; ...
                      constraint_zL + jz * delta_z];
                if norm(vi) < 1e-6
                    continue;
                end
                delta_knot = knotAt(knots, 2 + p + 1) - knotAt(knots, 2 + 1);
                neighbor = q2 + delta_knot * vi / p;
                dist = norm(neighbor - q2);
                min_voxel_size = min(min_voxel_size, dist);
                max_voxel_size = max(max_voxel_size, dist);
            end
        end
    end

    if ~isfinite(min_voxel_size) || ~isfinite(max_voxel_size)
        voxel_size = 0.25;
    else
        fraction_voxel_size = min(max(fraction_voxel_size, 0), 1);
        voxel_size = min_voxel_size + fraction_voxel_size * (max_voxel_size - min_voxel_size);
    end

    orig = q2 - [bbox; bbox; bbox] / 2;
end

function combos = astarSampleCombos(num_samples_x, num_samples_y, num_samples_z)
    [jx, jy, jz] = ndgrid(0:(num_samples_x - 1), 0:(num_samples_y - 1), 0:(num_samples_z - 1));
    combos = [jx(:), jy(:), jz(:)];
    if size(combos, 1) > 1
        combos = combos(randperm(size(combos, 1)), :);
    end
end

function [nodes, open] = astarExpandNeighbors(nodes, open, current_idx, goal, combos, ...
    constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU, ...
    knots, p, params, q0, num_samples_x, num_samples_y, num_samples_z)

    current = nodes(current_idx);
    i = current.index;

    if i >= (params.num_pol + 2 * p - p - 1 - 2)
        return;
    end

    delta_x = (constraint_xU - constraint_xL) / max((num_samples_x - 1), 1);
    delta_y = (constraint_yU - constraint_yL) / max((num_samples_y - 1), 1);
    delta_z = (constraint_zU - constraint_zL) / max((num_samples_z - 1), 1);

    delta_knot = knotAt(knots, i + p + 1) - knotAt(knots, i + 1);

    x_min = getFieldDefault(params, 'x_min', -inf);
    x_max = getFieldDefault(params, 'x_max', inf);
    y_min = getFieldDefault(params, 'y_min', -inf);
    y_max = getFieldDefault(params, 'y_max', inf);
    z_min = getFieldDefault(params, 'z_min', -inf);
    z_max = getFieldDefault(params, 'z_max', inf);

    for k = 1:size(combos, 1)
        jx = combos(k, 1);
        jy = combos(k, 2);
        jz = combos(k, 3);

        vi = [constraint_xL + jx * delta_x; ...
              constraint_yL + jy * delta_y; ...
              constraint_zL + jz * delta_z];

        if norm(vi) < 1e-5
            continue;
        end

        neighbor_q = current.qi + delta_knot * vi / p;

        if neighbor_q(1) > x_max || neighbor_q(1) < x_min || ...
           neighbor_q(2) > y_max || neighbor_q(2) < y_min || ...
           neighbor_q(3) > z_max || neighbor_q(3) < z_min || ...
           norm(neighbor_q - q0) >= params.Ra
            continue;
        end

        new_node.qi = neighbor_q;
        new_node.prev = current_idx;
        new_node.g = current.g + norm(neighbor_q - current.qi);
        new_node.h = norm(neighbor_q - goal);
        new_node.index = current.index + 1;

        nodes(end+1) = new_node;
        open(end+1) = numel(nodes);
    end
end

function [ok, constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU] = ...
    astarComputeBounds(i, nodes, current_idx, q0, q1, v_max, a_max, knots, p, N, alpha_shrink, use_minvo)

    if i == 2
        qiM2 = q0;
        qiM1 = q1;
    elseif i == 3
        qiM2 = q1;
        qiM1 = nodes(nodes(current_idx).prev).qi;
    else
        prev = nodes(current_idx).prev;
        prevprev = nodes(prev).prev;
        qiM2 = nodes(prevprev).qi;
        qiM1 = nodes(prev).qi;
    end

    qi = nodes(current_idx).qi;
    [ok, constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU] = ...
        astarComputeUpperAndLowerConstraints(i, qiM2, qiM1, qi, v_max, a_max, knots, p, N, alpha_shrink, use_minvo);
end

function [ok, constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU] = ...
    astarComputeUpperAndLowerConstraints(i, qiM2, qiM1, qi, v_max, a_max, knots, p, N, alpha_shrink, use_minvo)

    viM2 = p * (qiM1 - qiM2) / (knotAt(knots, i - 2 + p + 1) - knotAt(knots, i - 2 + 1));
    viM1 = p * (qi - qiM1) / (knotAt(knots, i - 1 + p + 1) - knotAt(knots, i - 1 + 1));
    num_segments = N - p + 1;
    interv = i - 2;

    constraint_xL = -inf;
    constraint_xU = inf;
    constraint_yL = -inf;
    constraint_yU = inf;
    constraint_zL = -inf;
    constraint_zU = inf;

    constraint_xL = max(constraint_xL, -v_max(1));
    constraint_xU = min(constraint_xU, v_max(1));
    constraint_yL = max(constraint_yL, -v_max(2));
    constraint_yU = min(constraint_yU, v_max(2));
    constraint_zL = max(constraint_zL, -v_max(3));
    constraint_zU = min(constraint_zU, v_max(3));

    % Extra velocity constraints for the last sampled velocity (matches OctopusSearch)
    if i == (N - 3)
        if use_minvo
            M_interv = getVelBs2mvMatrix(interv + 1, num_segments);
        else
            M_interv = eye(3);
        end

        tmp2 = viM1 * M_interv(1, :);

        if use_minvo
            M_interv = getVelBs2mvMatrix(interv + 2, num_segments);
        else
            M_interv = eye(3);
        end

        eps = 1e-5;
        for j = 1:3
            if abs(M_interv(2, j)) > eps
                vi_bound1 = (v_max(1) - tmp2(1, j)) / M_interv(2, j);
                vi_bound2 = (-v_max(1) - tmp2(1, j)) / M_interv(2, j);
                constraint_xL = max(constraint_xL, min(vi_bound1, vi_bound2));
                constraint_xU = min(constraint_xU, max(vi_bound1, vi_bound2));

                vi_bound1 = (v_max(2) - tmp2(2, j)) / M_interv(2, j);
                vi_bound2 = (-v_max(2) - tmp2(2, j)) / M_interv(2, j);
                constraint_yL = max(constraint_yL, min(vi_bound1, vi_bound2));
                constraint_yU = min(constraint_yU, max(vi_bound1, vi_bound2));

                vi_bound1 = (v_max(3) - tmp2(3, j)) / M_interv(2, j);
                vi_bound2 = (-v_max(3) - tmp2(3, j)) / M_interv(2, j);
                constraint_zL = max(constraint_zL, min(vi_bound1, vi_bound2));
                constraint_zU = min(constraint_zU, max(vi_bound1, vi_bound2));
            end
        end

        if use_minvo
            M_interv = getVelBs2mvMatrix(interv + 3, num_segments);
        else
            M_interv = eye(3);
        end

        for j = 1:3
            if abs(M_interv(1, j)) > eps
                vi_bound1 = v_max(1) / M_interv(1, j);
                vi_bound2 = -v_max(1) / M_interv(1, j);
                constraint_xL = max(constraint_xL, min(vi_bound1, vi_bound2));
                constraint_xU = min(constraint_xU, max(vi_bound1, vi_bound2));

                vi_bound1 = v_max(2) / M_interv(1, j);
                vi_bound2 = -v_max(2) / M_interv(1, j);
                constraint_yL = max(constraint_yL, min(vi_bound1, vi_bound2));
                constraint_yU = min(constraint_yU, max(vi_bound1, vi_bound2));

                vi_bound1 = v_max(3) / M_interv(1, j);
                vi_bound2 = -v_max(3) / M_interv(1, j);
                constraint_zL = max(constraint_zL, min(vi_bound1, vi_bound2));
                constraint_zU = min(constraint_zU, max(vi_bound1, vi_bound2));
            end
        end
    end

    if use_minvo
        mean_x = (constraint_xL + constraint_xU) / 2.0;
        dist_x = abs(mean_x - constraint_xL);
        constraint_xL = mean_x - alpha_shrink * dist_x;
        constraint_xU = mean_x + alpha_shrink * dist_x;

        mean_y = (constraint_yL + constraint_yU) / 2.0;
        dist_y = abs(mean_y - constraint_yL);
        constraint_yL = mean_y - alpha_shrink * dist_y;
        constraint_yU = mean_y + alpha_shrink * dist_y;

        mean_z = (constraint_zL + constraint_zU) / 2.0;
        dist_z = abs(mean_z - constraint_zL);
        constraint_zL = mean_z - alpha_shrink * dist_z;
        constraint_zU = mean_z + alpha_shrink * dist_z;
    end

    d = (knotAt(knots, i + p) - knotAt(knots, i + 1)) / (p - 1);
    max_vel = a_max * d + viM1;
    min_vel = -a_max * d + viM1;

    constraint_xL = max(constraint_xL, min_vel(1));
    constraint_xU = min(constraint_xU, max_vel(1));
    constraint_yL = max(constraint_yL, min_vel(2));
    constraint_yU = min(constraint_yU, max_vel(2));
    constraint_zL = max(constraint_zL, min_vel(3));
    constraint_zU = min(constraint_zU, max_vel(3));

    if i == (N - 3)
        vNm2 = [0; 0; 0];
        c = (knotAt(knots, N - 3 + p + 1) - knotAt(knots, N - 3 + 2)) / (p - 1);
        vNm3_max = vNm2 + c * a_max;
        vNm3_min = vNm2 - c * a_max;

        constraint_xL = max(constraint_xL, vNm3_min(1));
        constraint_xU = min(constraint_xU, vNm3_max(1));
        constraint_yL = max(constraint_yL, vNm3_min(2));
        constraint_yU = min(constraint_yU, vNm3_max(2));
        constraint_zL = max(constraint_zL, vNm3_min(3));
        constraint_zU = min(constraint_zU, vNm3_max(3));
    end

    ok = ~(constraint_xL > constraint_xU || constraint_yL > constraint_yU || constraint_zL > constraint_zU);
end

function collides = astarCollides(nodes, current_idx, q0, q1, obstacles, use_minvo, num_segments, N, gurobi_output)
    current = nodes(current_idx);
    if current.index < 3
        collides = false;
        return;
    end

    if current.index == 3
        last4 = [q0, q1, nodes(current.prev).qi, current.qi];
    elseif current.index == 4
        prev = nodes(current.prev).prev;
        last4 = [q1, nodes(prev).qi, nodes(current.prev).qi, current.qi];
    else
        prev1 = nodes(current.prev).prev;
        prev2 = nodes(prev1).prev;
        last4 = [nodes(prev2).qi, nodes(prev1).qi, nodes(current.prev).qi, current.qi];
    end

    collides = astarCollidesWithObstaclesGivenVertexes(last4, current.index, obstacles, use_minvo, num_segments, gurobi_output);
    if current.index == (N - 2)
        last4_tmp = [last4(:, 2), last4(:, 3), last4(:, 4), last4(:, 4)];
        collides = collides || astarCollidesWithObstaclesGivenVertexes(last4_tmp, N - 1, obstacles, use_minvo, num_segments, gurobi_output);
        last4_tmp = [last4(:, 3), last4(:, 4), last4(:, 4), last4(:, 4)];
        collides = collides || astarCollidesWithObstaclesGivenVertexes(last4_tmp, N, obstacles, use_minvo, num_segments, gurobi_output);
    end
end

function collides = astarCollidesWithObstaclesGivenVertexes(last4Cps, index_lastCP, obstacles, use_minvo, num_segments, gurobi_output)
    seg = index_lastCP - 2;
    if seg < 1 || seg > num_segments
        collides = false;
        return;
    end

    if use_minvo
        cps = transformBsplineToMinvo(last4Cps, seg, num_segments);
    else
        cps = last4Cps;
    end

    params_lp = struct();
    params_lp.OutputFlag = gurobi_output;

    for obs_idx = 1:length(obstacles)
        obs = obstacles{obs_idx};
        [~, ~, ~, feasible] = solvePlaneLP(cps, obs, params_lp);
        if ~feasible
            collides = true;
            return;
        end
    end
    collides = false;
end

function [ok, planes_n, planes_d] = astarCheckFeasAndFillND(q, knots, p, v_max, a_max, obstacles, use_minvo, num_segments, gurobi_output)
    num_obstacles = length(obstacles);
    planes_n = cell(1, num_segments * num_obstacles);
    planes_d = zeros(1, num_segments * num_obstacles);
    ok = true;
    N = size(q, 2) - 1;

    params_lp = struct();
    params_lp.OutputFlag = gurobi_output;

    for seg = 1:num_segments
        last4 = q(:, seg:seg+3);
        if use_minvo
            cps = transformBsplineToMinvo(last4, seg, num_segments);
        else
            cps = last4;
        end
        for obs_idx = 1:num_obstacles
            obs = obstacles{obs_idx};
            [n, d, margin, feasible] = solvePlaneLP(cps, obs, params_lp);
            if ~feasible
                ok = false;
                return;
            end
            n = n / margin;
            d = d / margin;
            ip = (seg - 1) * num_obstacles + obs_idx;
            planes_n{ip} = n;
            planes_d(ip) = d;
        end
    end

    epsilon = 1.0001;
    for i = 0:(N - 3)
        vi = p * (q(:, i+2) - q(:, i+1)) / (knotAt(knots, i + p + 1) - knotAt(knots, i + 1));
        vip1 = p * (q(:, i+3) - q(:, i+2)) / (knotAt(knots, i + 1 + p + 1) - knotAt(knots, i + 1 + 1));
        vip2 = p * (q(:, i+4) - q(:, i+3)) / (knotAt(knots, i + 1 + p + 1 + 1) - knotAt(knots, i + 1 + 1 + 1));

        ai = (p - 1) * (vip1 - vi) / (knotAt(knots, i + p + 1) - knotAt(knots, i + 2));

        Vbs = [vi, vip1, vip2];
        if use_minvo
            M_vel = getVelBs2mvMatrix(i + 1, num_segments);
        else
            M_vel = eye(3);
        end
        V_newbasis = Vbs * M_vel;
        if max(V_newbasis(:)) > epsilon * v_max(1) || min(V_newbasis(:)) < -epsilon * v_max(1)
            ok = false;
            return;
        end
        if any(ai > epsilon * a_max) || any(ai < -epsilon * a_max)
            ok = false;
            return;
        end
    end
end

function q_guess = astarRecoverPath(nodes, idx, q0, q1)
    q_guess = [];
    if idx == 0
        q_guess = [q0, q1];
        return;
    end
    tmp = idx;
    q_guess = [q_guess, nodes(tmp).qi, nodes(tmp).qi];
    while tmp ~= 0
        q_guess = [q_guess, nodes(tmp).qi];
        tmp = nodes(tmp).prev;
    end
    q_guess = [q_guess, q1, q0];
    q_guess = fliplr(q_guess);
end

function [best_pos, best_idx] = astarPopBest(open, nodes, bias)
    best_pos = 1;
    best_idx = open(1);
    best_cost = nodes(best_idx).g + bias * nodes(best_idx).h;
    best_h = nodes(best_idx).h;
    for k = 2:length(open)
        idx = open(k);
        cost = nodes(idx).g + bias * nodes(idx).h;
        if (cost < best_cost - 1e-5) || (abs(cost - best_cost) < 1e-5 && nodes(idx).h < best_h)
            best_cost = cost;
            best_h = nodes(idx).h;
            best_pos = k;
            best_idx = idx;
        end
    end
end

function key = astarVoxelKey(qi, orig, voxel_size)
    idx = round((qi - orig) / voxel_size);
    key = sprintf('%d_%d_%d', idx(1), idx(2), idx(3));
end

function val = knotAt(knots, idx_cpp)
    val = knots(idx_cpp + 1);
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

function [planes_n, planes_d, planes_margins, ok] = computeSeparatingPlanesLP(q, obstacles, num_segments, gurobi_output, use_minvo)
    planes_n = {};
    planes_d = [];
    planes_margins = [];
    ok = true;

    num_obstacles = length(obstacles);

    params_lp = struct();
    params_lp.OutputFlag = gurobi_output;

    for seg = 1:num_segments
        if use_minvo
            cps = transformBsplineToMinvo(q(:, seg:seg+3), seg, num_segments);
        else
            cps = q(:, seg:seg+3);
        end

        for obs_idx = 1:num_obstacles
            obs = obstacles{obs_idx};
            [n, d, margin, feasible] = solvePlaneLP(cps, obs, params_lp);
            if ~(feasible && margin > 1e-6)
                ok = false;
                return;
            end
            n = n / margin;
            d = d / margin;
            margin = 1.0;

            planes_n{end+1} = n;
            planes_d(end+1) = d;
            planes_margins(end+1) = margin;
        end
    end
end

function [n, d, margin, feasible] = solvePlaneLP(cps, obs, params_lp)
    % Solve LP to separate control points and obstacle vertices.
    % Variables: [nx ny nz d s], maximize s.
    n = [];
    d = [];
    margin = 0.0;
    feasible = false;

    num_vars = 5;
    A = [];
    rhs = [];

    for i = 1:size(cps, 2)
        cp = cps(:, i);
        % Control points on negative side: n'cp + d <= -s
        A = [A; cp(1), cp(2), cp(3), 1, 1];
        rhs = [rhs; 0];
    end

    for i = 1:size(obs, 2)
        v = obs(:, i);
        % Obstacle vertices on positive side: n'v + d >= s
        A = [A; -v(1), -v(2), -v(3), -1, 1];
        rhs = [rhs; 0];
    end

    model = struct();
    model.A = sparse(A);
    model.rhs = rhs;
    model.sense = repmat('<', size(A, 1), 1);
    model.obj = [0; 0; 0; 0; -1];
    model.modelsense = 'min';
    model.lb = [-1; -1; -1; -inf; 0];
    model.ub = [1; 1; 1; inf; inf];

    result = gurobi(model, params_lp);
    if ~isfield(result, 'x')
        return;
    end
    if strcmpi(result.status, 'OPTIMAL')
        margin = result.x(5);
        if margin > 1e-5
            n = result.x(1:3);
            d = result.x(4);
            feasible = true;
        end
    end
end

function [n, d] = heuristicPlaneFromPoints(cps, obs)
    cp_center = mean(cps, 2);
    obs_center = mean(obs, 2);
    n = obs_center - cp_center;
    n_norm = norm(n);
    if n_norm < 1e-6
        n = [1; 0; 0];
    else
        n = n / n_norm;
    end

    obs_projs = n' * obs;
    cp_projs = n' * cps;
    d = -min(obs_projs);

    if max(cp_projs) + d > -0.01
        mid = (max(cp_projs) + min(obs_projs)) / 2;
        d = -mid;
    end
end

function [planes_n, planes_d] = computeSeparatingPlanesHeuristic(q, obstacles, num_segments, use_minvo)
    planes_n = {};
    planes_d = [];

    for seg = 1:num_segments
        if use_minvo
            cps = transformBsplineToMinvo(q(:, seg:seg+3), seg, num_segments);
        else
            cps = q(:, seg:seg+3);
        end
        cp_center = mean(cps, 2);

        for obs_idx = 1:length(obstacles)
            obs = obstacles{obs_idx};
            obs_center = mean(obs, 2);

            best_n = [];
            best_margin = -inf;

            test_dirs = [
                [1; 0; 0], [-1; 0; 0], [0; 1; 0], [0; -1; 0], [0; 0; 1], [0; 0; -1], ...
                [1; 1; 0]/sqrt(2), [1; -1; 0]/sqrt(2), [-1; 1; 0]/sqrt(2), [-1; -1; 0]/sqrt(2), ...
                obs_center - cp_center
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

                max_cp = max(cp_projs);
                min_obs = min(obs_projs);

                margin = min_obs - max_cp;

                if margin > best_margin
                    best_margin = margin;
                    best_n = n;
                end
            end

            if isempty(best_n) || best_margin < -1
                n = obs_center - cp_center;
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

            max_cp_proj = max(cp_projs);
            min_obs_proj = min(obs_projs);

            d = -min_obs_proj;
            if max_cp_proj + d > -0.05
                mid_proj = (max_cp_proj + min_obs_proj) / 2;
                d = -mid_proj;
            end

            planes_n{end+1} = best_n;
            planes_d(end+1) = d;
        end
    end
end

function model = buildGurobiModel(q0, q1, q2, qNm2, qNm1, qN, pf, weight, ...
    knots, N, p, num_segments, v_max, a_max, j_max, Ra, deltaT, ...
    planes_n, planes_d, planes_margins, plane_margin, num_obstacles, use_minvo)

    free_indices = 4:(N+1);
    num_free_cps = numel(free_indices);
    nvar = 3 * num_free_cps;

    free_map = zeros(1, N + 1);
    free_map(free_indices) = 1:num_free_cps;

    q_fixed = nan(3, N + 1);
    q_fixed(:, 1) = q0;
    q_fixed(:, 2) = q1;
    q_fixed(:, 3) = q2;

    A = zeros(0, nvar);
    rhs = zeros(0, 1);
    sense = repmat('<', 0, 1);

    if use_minvo
        for i = 3:(N-1)
            ciM2 = p / (knots(i + p - 1) - knots(i - 1));
            ciM1 = p / (knots(i + p) - knots(i));
            ci = p / (knots(i + p + 1) - knots(i + 1));

            Mv = getVelBs2mvMatrix(i - 2, num_segments);
            rowsM2 = zeros(3, nvar); cstM2 = zeros(3, 1);
            rowsM1 = zeros(3, nvar); cstM1 = zeros(3, 1);
            rows = zeros(3, nvar); cst = zeros(3, 1);

            for axis = 1:3
                [rowsM2(axis, :), cstM2(axis)] = linDiff(axis, i-1, i-2, ciM2, free_map, q_fixed, nvar);
                [rowsM1(axis, :), cstM1(axis)] = linDiff(axis, i, i-1, ciM1, free_map, q_fixed, nvar);
                [rows(axis, :), cst(axis)] = linDiff(axis, i+1, i, ci, free_map, q_fixed, nvar);
            end

            for j = 1:3
                for axis = 1:3
                    row = Mv(1, j) * rowsM2(axis, :) + Mv(2, j) * rowsM1(axis, :) + Mv(3, j) * rows(axis, :);
                    cst_j = Mv(1, j) * cstM2(axis) + Mv(2, j) * cstM1(axis) + Mv(3, j) * cst(axis);
                    [A, rhs, sense] = addLeq(A, rhs, sense, row, v_max(axis) - cst_j);
                    [A, rhs, sense] = addLeq(A, rhs, sense, -row, v_max(axis) + cst_j);
                end
            end
        end
    else
        for i = 3:N
            ci = p / (knots(i+p+1) - knots(i+1));
            for axis = 1:3
                [row, cst] = linDiff(axis, i+1, i, ci, free_map, q_fixed, nvar);
                [A, rhs, sense] = addLeq(A, rhs, sense, row, v_max(axis) - cst);
                [A, rhs, sense] = addLeq(A, rhs, sense, -row, v_max(axis) + cst);
            end
        end
    end

    for i = 2:N-2
        c1 = p / (knots(i+p+1) - knots(i+1));
        c2 = p / (knots(i+p+2) - knots(i+2));
        c3 = (p - 1) / (knots(i+p+1) - knots(i+2));
        for axis = 1:3
            [row1, cst1] = linDiff(axis, i+2, i+1, c2, free_map, q_fixed, nvar);
            [row0, cst0] = linDiff(axis, i+1, i, c1, free_map, q_fixed, nvar);
            row = c3 * (row1 - row0);
            cst = c3 * (cst1 - cst0);
            [A, rhs, sense] = addLeq(A, rhs, sense, row, a_max(axis) - cst);
            [A, rhs, sense] = addLeq(A, rhs, sense, -row, a_max(axis) + cst);
        end
    end

    tmp = [6; 0; 0; 0] / (deltaT^3);
    for i = 1:num_segments
        A_pos_bs_seg = getABSplineMatrix(i, num_segments);
        weights = (A_pos_bs_seg * tmp).';
        for axis = 1:3
            [row, cst] = linCombo(axis, i:(i+3), weights, free_map, q_fixed, nvar);
            [A, rhs, sense] = addLeq(A, rhs, sense, row, j_max(axis) - cst);
            [A, rhs, sense] = addLeq(A, rhs, sense, -row, j_max(axis) + cst);
        end
    end

    for axis = 1:3
        [rowN, cstN] = cpExpr(axis, N+1, free_map, q_fixed, nvar);
        [rowNm1, cstNm1] = cpExpr(axis, N, free_map, q_fixed, nvar);
        row = rowN - rowNm1;
        cst = cstN - cstNm1;
        [A, rhs, sense] = addEq(A, rhs, sense, row, -cst);

        [rowNm2, cstNm2] = cpExpr(axis, N-1, free_map, q_fixed, nvar);
        row = rowNm1 - rowNm2;
        cst = cstNm1 - cstNm2;
        [A, rhs, sense] = addEq(A, rhs, sense, row, -cst);
    end

    for seg = 1:num_segments
        for obs_idx = 1:num_obstacles
            ip = (seg - 1) * num_obstacles + obs_idx;
            n = planes_n{ip};
            d = planes_d(ip);
            if ~isempty(planes_margins)
                local_margin = planes_margins(ip);
            else
                local_margin = plane_margin;
            end
            for u = 0:3
                if use_minvo
                    [row, cst] = dotExprMinvo(n, seg, u + 1, free_map, q_fixed, nvar, num_segments);
                else
                    cp_idx = seg + u;
                    [row, cst] = dotExpr(n, cp_idx, free_map, q_fixed, nvar);
                end
                cst = cst + d;
                [A, rhs, sense] = addLeq(A, rhs, sense, row, local_margin - cst);
            end
        end
    end

    Q = zeros(nvar, nvar);
    c = zeros(nvar, 1);
    tmp = [6; 0; 0; 0];
    for i = 1:num_segments
        A_pos_bs_seg = getABSplineMatrix(i, num_segments);
        weights = (A_pos_bs_seg * tmp).';
        for axis = 1:3
            [row, cst] = linCombo(axis, i:(i+3), weights, free_map, q_fixed, nvar);
            Q = Q + 2 * (row' * row);
            c = c + 2 * cst * row';
        end
    end

    slotN = free_map(N + 1);
    if slotN > 0
        var_idx = (slotN - 1) * 3 + (1:3);
        Q(var_idx, var_idx) = Q(var_idx, var_idx) + 2 * weight * eye(3);
        c(var_idx) = c(var_idx) - 2 * weight * pf;
    end

    qc = struct('Qc', {}, 'q', {}, 'rhs', {}, 'sense', {});
    for seg = 1:num_segments
        for u = 1:4
            if use_minvo
                [rowx, cstx] = minvoCtrlExpr(1, seg, u, free_map, q_fixed, nvar, num_segments);
                [rowy, csty] = minvoCtrlExpr(2, seg, u, free_map, q_fixed, nvar, num_segments);
                [rowz, cstz] = minvoCtrlExpr(3, seg, u, free_map, q_fixed, nvar, num_segments);
            else
                cp_idx = seg + u - 1;
                [rowx, cstx] = cpExpr(1, cp_idx, free_map, q_fixed, nvar);
                [rowy, csty] = cpExpr(2, cp_idx, free_map, q_fixed, nvar);
                [rowz, cstz] = cpExpr(3, cp_idx, free_map, q_fixed, nvar);
            end

            Qc = rowx' * rowx + rowy' * rowy + rowz' * rowz;
            q = 2 * (rowx' * (cstx - q0(1)) + rowy' * (csty - q0(2)) + rowz' * (cstz - q0(3)));
            rhs_q = Ra^2 - ((cstx - q0(1))^2 + (csty - q0(2))^2 + (cstz - q0(3))^2);
            qc(end+1) = struct('Qc', sparse(Qc), 'q', q, 'rhs', rhs_q, 'sense', '<');
        end
    end

    model = struct();
    model.A = sparse(A);
    model.rhs = rhs;
    model.sense = sense;
    model.Q = sparse(Q);
    model.obj = c;
    model.modelsense = 'min';
    model.lb = -inf(nvar, 1);
    model.ub = inf(nvar, 1);
    if ~isempty(qc)
        model.quadcon = qc;
    end
end

function [A, rhs, sense] = addLeq(A, rhs, sense, row, bound)
    A(end+1, :) = row;
    rhs(end+1, 1) = bound;
    sense(end+1, 1) = '<';
end

function [A, rhs, sense] = addEq(A, rhs, sense, row, bound)
    A(end+1, :) = row;
    rhs(end+1, 1) = bound;
    sense(end+1, 1) = '=';
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

function [row, cst] = minvoCtrlExpr(axis, seg, ctrl_idx, free_map, q_fixed, nvar, num_segments)
    row = zeros(1, nvar);
    cst = 0;
    M = getBs2mvMatrix(seg, num_segments);
    for k = 1:4
        weight = M(k, ctrl_idx);
        [r, c] = cpExpr(axis, seg + k - 1, free_map, q_fixed, nvar);
        row = row + weight * r;
        cst = cst + weight * c;
    end
end

function [row, cst] = dotExprMinvo(n, seg, ctrl_idx, free_map, q_fixed, nvar, num_segments)
    row = zeros(1, nvar);
    cst = 0;
    for axis = 1:3
        [r, c] = minvoCtrlExpr(axis, seg, ctrl_idx, free_map, q_fixed, nvar, num_segments);
        row = row + n(axis) * r;
        cst = cst + n(axis) * c;
    end
end

function Qmv = transformBsplineToMinvo(Qbs, seg, num_segments)
    M = getBs2mvMatrix(seg, num_segments);
    Qmv = Qbs * M;
end

function M = getBs2mvMatrix(seg, num_segments)
    % Matrices from mader_types.hpp (basisConverter)
    M_pos_bs2mv_seg0 = [
        1.1023313949144333,   0.3420572455666697,  -0.09273093424558287, -0.03203276669713062;
       -0.04968355625374918,  0.6578034732467718,   0.5305386376018690,   0.21181027098212013;
       -0.04730904421116235,  0.01559443689415559,  0.5051827557159350,   0.63650059656260427;
       -0.0053387944495217445, -0.015455155707597083, 0.057009540927778303, 0.18372189915240558
    ];

    M_pos_bs2mv_seg1 = [
        0.27558284872860833,  0.08551431139166743, -0.02318273356139572, -0.008008191674282655;
        0.60990427619758658,  0.63806904207840509,  0.29959938009132259,  0.12252106674808683;
        0.11985166952332682,  0.29187180223752446,  0.66657381254229420,  0.70176522577378930;
       -0.0053387944495217445, -0.015455155707597083, 0.057009540927778303, 0.18372189915240558
    ];

    M_pos_bs2mv_rest = [
        0.18372189915240555,  0.05700954092777831, -0.015455155707597118, -0.0053387944495218165;
        0.70176522577378919,  0.66657381254229420,  0.29187180223752385,   0.11985166952332582;
        0.11985166952332682,  0.29187180223752446,  0.66657381254229420,   0.70176522577378930;
       -0.0053387944495217445, -0.015455155707597083, 0.057009540927778303, 0.18372189915240558
    ];

    M_pos_bs2mv_seg_last2 = [
        0.18372189915240569,  0.05700954092777831, -0.015455155707597146, -0.0053387944495218165;
        0.70176522577378952,  0.66657381254229453,  0.29187180223752413,   0.11985166952332593;
        0.12252106674808753,  0.29959938009132281,  0.63806904207840498,   0.60990427619758625;
       -0.008008191674282615, -0.02318273356139562, 0.085514311391667444, 0.27558284872860833
    ];

    M_pos_bs2mv_seg_last = [
        0.18372189915240555,  0.05700954092777831, -0.015455155707597118, -0.0053387944495218165;
        0.63650059656260416,  0.50518275571593496,  0.015594436894155295, -0.047309044211162887;
        0.21181027098212069,  0.53053863760186915,  0.65780347324677146, -0.049683556253749622;
       -0.032032766697130462, -0.092730934245582486, 0.34205724556666978, 1.1023313949144333
    ];

    if seg == 1
        M = M_pos_bs2mv_seg0;
    elseif seg == 2
        M = M_pos_bs2mv_seg1;
    elseif seg >= (num_segments - 1)
        if seg == num_segments - 1
            M = M_pos_bs2mv_seg_last2;
        else
            M = M_pos_bs2mv_seg_last;
        end
    else
        M = M_pos_bs2mv_rest;
    end
end

function M = getVelBs2mvMatrix(seg, num_segments)
    M_vel_bs2mv_seg0 = [
        1.077349059083916,  0.1666702138890985, -0.07735049175615138;
       -0.03867488648729411,  0.7499977187062712,   0.5386802643920123;
       -0.03867417280506149, 0.08333206631563977,    0.538670227146185
    ];

    M_vel_bs2mv_rest = [
        0.538674529541958, 0.08333510694454926, -0.03867524587807569;
        0.4999996430546639,  0.8333328256508203,   0.5000050185139366;
       -0.03867417280506149, 0.08333206631563977,    0.538670227146185
    ];

    M_vel_bs2mv_seg_last = [
        0.538674529541958, 0.08333510694454926, -0.03867524587807569;
        0.5386738158597254,  0.7500007593351806, -0.03866520863224832;
       -0.07734834561012298,  0.1666641326312795,     1.07734045429237
    ];

    if seg <= 1
        M = M_vel_bs2mv_seg0;
    elseif seg >= num_segments
        M = M_vel_bs2mv_seg_last;
    else
        M = M_vel_bs2mv_rest;
    end
end

function A = getABSplineMatrix(seg, num_segments)
    A_pos_bs_seg0 = [
       -1.0000,  3.0000, -3.0000,  1.0000;
        1.7500, -4.5000,  3.0000,  0.0000;
       -0.9167,  1.5000,  0.0000,  0.0000;
        0.1667,  0.0000,  0.0000,  0.0000
    ];

    A_pos_bs_seg1 = [
       -0.2500,  0.7500, -0.7500,  0.2500;
        0.5833, -1.2500,  0.2500,  0.5833;
       -0.5000,  0.5000,  0.5000,  0.1667;
        0.1667,  0.0000,  0.0000,  0.0000
    ];

    A_pos_bs_rest = [
       -0.1667,  0.5000, -0.5000,  0.1667;
        0.5000, -1.0000,  0.0000,  0.6667;
       -0.5000,  0.5000,  0.5000,  0.1667;
        0.1667,  0.0000,  0.0000,  0.0000
    ];

    A_pos_bs_seg_last2 = [
       -0.1667,  0.5000, -0.5000,  0.1667;
        0.5000, -1.0000,  0.0000,  0.6667;
       -0.5833,  0.5000,  0.5000,  0.1667;
        0.2500,  0.0000,  0.0000,  0.0000
    ];

    A_pos_bs_seg_last = [
       -0.1667,  0.5000, -0.5000,  0.1667;
        0.9167, -1.2500, -0.2500,  0.5833;
       -1.7500,  0.7500,  0.7500,  0.2500;
        1.0000,  0.0000,  0.0000,  0.0000
    ];

    if seg == 1
        A = A_pos_bs_seg0;
    elseif seg == 2
        A = A_pos_bs_seg1;
    elseif seg >= (num_segments - 1)
        if seg == num_segments - 1
            A = A_pos_bs_seg_last2;
        else
            A = A_pos_bs_seg_last;
        end
    else
        A = A_pos_bs_rest;
    end
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

function value = getFieldDefault(s, name, default_value)
    if isfield(s, name)
        value = s.(name);
    else
        value = default_value;
    end
end

function v = clampVec(v, vmin, vmax)
    v = min(max(v, vmin), vmax);
end

function time = getMinTimeDoubleIntegrator1D(p0, v0, pf, vf, v_max, a_max)
    % Port of utils.cpp (mu::getMinTimeDoubleIntegrator1D)
    x1 = v0;
    x2 = p0;
    x1r = vf;
    x2r = pf;

    k1 = a_max;
    k2 = 1.0;
    x1_bar = v_max;

    B = (k2 / (2 * k1)) * sign(-x1 + x1r) * (x1^2 - x1r^2) + x2r;
    C = (k2 / (2 * k1)) * (x1^2 + x1r^2) - (k2 / k1) * x1_bar^2 + x2r;
    D = (-k2 / (2 * k1)) * (x1^2 + x1r^2) + (k2 / k1) * x1_bar^2 + x2r;

    if (x2 <= B) && (x2 >= C)
        inside = max(k2^2 * x1^2 - k1 * k2 * ((k2 / (2 * k1)) * (x1^2 - x1r^2) + x2 - x2r), 0);
        time = (-k2 * (x1 + x1r) + 2 * sqrt(inside)) / (k1 * k2);
    elseif (x2 <= B) && (x2 < C)
        time = (x1_bar - x1 - x1r) / k1 + (x1^2 + x1r^2) / (2 * k1 * x1_bar) + (x2r - x2) / (k2 * x1_bar);
    elseif (x2 > B) && (x2 <= D)
        inside = max(k2^2 * x1^2 + k1 * k2 * ((k2 / (2 * k1)) * (-x1^2 + x1r^2) + x2 - x2r), 0);
        time = (k2 * (x1 + x1r) + 2 * sqrt(inside)) / (k1 * k2);
    else
        time = (x1_bar + x1 + x1r) / k1 + (x1^2 + x1r^2) / (2 * k1 * x1_bar) + (-x2r + x2) / (k2 * x1_bar);
    end
end

function time = getMinTimeDoubleIntegrator3D(p0, v0, pf, vf, v_max, a_max)
    tx = getMinTimeDoubleIntegrator1D(p0(1), v0(1), pf(1), vf(1), v_max(1), a_max(1));
    ty = getMinTimeDoubleIntegrator1D(p0(2), v0(2), pf(2), vf(2), v_max(2), a_max(2));
    tz = getMinTimeDoubleIntegrator1D(p0(3), v0(3), pf(3), vf(3), v_max(3), a_max(3));
    time = max([tx, ty, tz]);
end
