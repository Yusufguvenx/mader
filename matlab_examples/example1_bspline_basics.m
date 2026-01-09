%% MADER MATLAB Example 1: B-Spline Trajectory Basics
% This script demonstrates the B-spline trajectory representation used in MADER.
% Converted from the C++ implementation in mader/src/bspline_utils.cpp
%
% Author: Converted from MIT ACL MADER project
% Reference: Jesus Tordesillas, et al. "MADER: Trajectory Planner in Multi-Agent
%            and Dynamic Environments" IEEE T-RO 2021

clear; clc; close all;

%% 1. Define the Uniform Cubic B-Spline Basis Matrix
% This is the core transformation matrix for cubic B-splines
% Converts 4 control points to polynomial coefficients

M_bspline = (1/6) * [
    1,  4,  1, 0;
   -3,  0,  3, 0;
    3, -6,  3, 0;
   -1,  3, -3, 1
];

fprintf('Uniform Cubic B-Spline Basis Matrix M:\n');
disp(M_bspline);

%% 2. Define Control Points for a 3D Trajectory
% Control points define the shape of the B-spline curve
% For MADER: these are the decision variables in the optimization

% Example: 7 control points for a curved 3D trajectory
control_points = [
    -4,  -2,   0,   2,   4,   5,   6;   % X coordinates
     0,   2,   3,   2,   0,  -1,  -2;   % Y coordinates
     0,   1,   2,   2,   1,   1,   1    % Z coordinates
];

num_control_points = size(control_points, 2);
degree = 3;  % Cubic B-spline
num_segments = num_control_points - degree;  % Number of polynomial segments

fprintf('Number of control points: %d\n', num_control_points);
fprintf('Number of polynomial segments: %d\n', num_segments);

%% 3. Generate Knot Vector
% For uniform B-splines with clamped ends
% Knots define the parameter intervals

t_start = 0;
t_end = num_segments;  % Each segment spans [i, i+1]
knots = linspace(t_start, t_end, num_segments + 1);

fprintf('Knot vector: ');
disp(knots);

%% 4. Convert Control Points to Piecewise Polynomial Coefficients
% This is the key transformation from bspline_utils.cpp CPs2TrajAndPwp()

% Storage for polynomial coefficients
% For each segment: coeff = [a, b, c, d] where p(u) = a*u^3 + b*u^2 + c*u + d
coeff_x = zeros(num_segments, 4);
coeff_y = zeros(num_segments, 4);
coeff_z = zeros(num_segments, 4);

for seg = 1:num_segments
    % Extract 4 control points for this segment
    cps_x = control_points(1, seg:seg+3)';  % 4x1 column vector
    cps_y = control_points(2, seg:seg+3)';
    cps_z = control_points(3, seg:seg+3)';

    % Transform to polynomial coefficients
    % Result is [d; c; b; a] format, we flip to [a; b; c; d]
    coeff_x(seg, :) = flip(M_bspline * cps_x)';
    coeff_y(seg, :) = flip(M_bspline * cps_y)';
    coeff_z(seg, :) = flip(M_bspline * cps_z)';
end

fprintf('\nPolynomial coefficients for segment 1:\n');
fprintf('  X: [%.4f, %.4f, %.4f, %.4f] (a*u^3 + b*u^2 + c*u + d)\n', coeff_x(1,:));
fprintf('  Y: [%.4f, %.4f, %.4f, %.4f]\n', coeff_y(1,:));
fprintf('  Z: [%.4f, %.4f, %.4f, %.4f]\n', coeff_z(1,:));

%% 5. Create PieceWisePol Structure (matches C++ mt::PieceWisePol)
% This structure stores the complete trajectory representation

pwp = struct();
pwp.times = knots;
pwp.coeff_x = coeff_x;
pwp.coeff_y = coeff_y;
pwp.coeff_z = coeff_z;

%% 6. Evaluate the Trajectory at Any Time t
% This implements the eval() method from C++ PieceWisePol

function pos = evalPwp(pwp, t)
    % Evaluate piecewise polynomial at time t
    % Returns [x; y; z] position

    times = pwp.times;

    % Handle boundary cases
    if t >= times(end)
        u = 1;
        seg = length(times) - 1;
    elseif t < times(1)
        u = 0;
        seg = 1;
    else
        % Find the segment
        seg = find(times(1:end-1) <= t & t < times(2:end), 1);
        if isempty(seg)
            seg = length(times) - 1;
        end
        % Compute normalized parameter u in [0, 1]
        u = (t - times(seg)) / (times(seg+1) - times(seg));
    end

    % Evaluate polynomial: a*u^3 + b*u^2 + c*u + d
    u_vec = [u^3; u^2; u; 1];
    pos = [
        pwp.coeff_x(seg, :) * u_vec;
        pwp.coeff_y(seg, :) * u_vec;
        pwp.coeff_z(seg, :) * u_vec
    ];
end

%% 7. Sample the Trajectory for Visualization
dt = 0.01;  % Sampling interval (similar to dc in MADER)
t_samples = t_start:dt:t_end;
trajectory = zeros(3, length(t_samples));

for i = 1:length(t_samples)
    trajectory(:, i) = evalPwp(pwp, t_samples(i));
end

%% 8. Compute Velocity and Acceleration
% Derivatives of the piecewise polynomial

function [vel, accel] = evalPwpDerivatives(pwp, t)
    times = pwp.times;

    if t >= times(end)
        u = 1;
        seg = length(times) - 1;
    elseif t < times(1)
        u = 0;
        seg = 1;
    else
        seg = find(times(1:end-1) <= t & t < times(2:end), 1);
        if isempty(seg)
            seg = length(times) - 1;
        end
        u = (t - times(seg)) / (times(seg+1) - times(seg));
    end

    % Duration of segment for derivative scaling
    delta_t = times(seg+1) - times(seg);

    % First derivative: 3a*u^2 + 2b*u + c
    du_vec = [3*u^2; 2*u; 1; 0];
    vel = [
        pwp.coeff_x(seg, :) * du_vec;
        pwp.coeff_y(seg, :) * du_vec;
        pwp.coeff_z(seg, :) * du_vec
    ] / delta_t;

    % Second derivative: 6a*u + 2b
    ddu_vec = [6*u; 2; 0; 0];
    accel = [
        pwp.coeff_x(seg, :) * ddu_vec;
        pwp.coeff_y(seg, :) * ddu_vec;
        pwp.coeff_z(seg, :) * ddu_vec
    ] / (delta_t^2);
end

% Sample velocity and acceleration
velocity = zeros(3, length(t_samples));
acceleration = zeros(3, length(t_samples));

for i = 1:length(t_samples)
    [velocity(:, i), acceleration(:, i)] = evalPwpDerivatives(pwp, t_samples(i));
end

%% 9. Visualization
figure('Position', [100, 100, 1400, 500]);

% Plot 1: 3D Trajectory
subplot(1, 3, 1);
plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(control_points(1,:), control_points(2,:), control_points(3,:), ...
    'ro-', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('B-Spline Trajectory');
legend('Trajectory', 'Control Points', 'Location', 'best');
grid on;
axis equal;
view(45, 30);

% Plot 2: Position vs Time
subplot(1, 3, 2);
plot(t_samples, trajectory(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t_samples, trajectory(2,:), 'g-', 'LineWidth', 1.5);
plot(t_samples, trajectory(3,:), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Position [m]');
title('Position vs Time');
legend('X', 'Y', 'Z', 'Location', 'best');
grid on;

% Mark segment boundaries
for k = 1:length(knots)
    xline(knots(k), '--k', 'Alpha', 0.3);
end

% Plot 3: Velocity and Acceleration Magnitudes
subplot(1, 3, 3);
vel_mag = vecnorm(velocity);
accel_mag = vecnorm(acceleration);
yyaxis left
plot(t_samples, vel_mag, 'b-', 'LineWidth', 1.5);
ylabel('Velocity [m/s]');
yyaxis right
plot(t_samples, accel_mag, 'r-', 'LineWidth', 1.5);
ylabel('Acceleration [m/s^2]');
xlabel('Time [s]');
title('Velocity & Acceleration Magnitudes');
legend('Velocity', 'Acceleration', 'Location', 'best');
grid on;

sgtitle('MADER B-Spline Trajectory Representation (Example 1)');

%% 10. Summary
fprintf('\n=== Trajectory Summary ===\n');
fprintf('Duration: %.2f s\n', t_end - t_start);
fprintf('Max velocity: %.2f m/s\n', max(vel_mag));
fprintf('Max acceleration: %.2f m/s^2\n', max(accel_mag));
fprintf('Trajectory length: %.2f m\n', sum(vecnorm(diff(trajectory, 1, 2))));

fprintf('\n=== Key MADER Concepts ===\n');
fprintf('1. Trajectory represented as piecewise cubic polynomials\n');
fprintf('2. Control points are optimization variables\n');
fprintf('3. B-spline ensures C2 continuity (smooth vel/accel)\n');
fprintf('4. Each segment: p(u) = a*u^3 + b*u^2 + c*u + d, u in [0,1]\n');
