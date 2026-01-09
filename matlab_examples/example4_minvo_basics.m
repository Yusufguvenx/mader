%% MADER MATLAB Example 4: MINVO Basis Fundamentals
% =========================================================================
% This script demonstrates the MINVO (Minimum Volume) basis used in MADER.
%
% MINVO is a key innovation in MADER that provides tighter convex hulls
% for collision checking compared to B-spline or Bezier control points.
%
% KEY INSIGHT:
% ------------
% When checking if a trajectory segment collides with an obstacle, we need
% to bound the trajectory curve. The control points of a polynomial curve
% form a convex hull that contains the curve. MINVO provides control points
% whose convex hull has MINIMUM VOLUME while still containing the curve.
%
% COMPARISON OF BASES:
% --------------------
%   B-Spline: Standard basis, smooth curves, control points spread out
%   Bezier:   Intuitive (curve passes through endpoints), moderate hull size
%   MINVO:    Minimum volume convex hull -> less conservative collision check
%
% WHY THIS MATTERS:
% -----------------
% In cluttered environments, a smaller convex hull means:
%   - Fewer false collision detections
%   - Tighter paths through narrow gaps
%   - Better optimization feasibility
%
% Reference:
%   Tordesillas & How, "MINVO Basis: Finding Simplexes with Minimum Volume
%   Enclosing Polynomial Curves", 2020
%
% Author: Converted from MIT ACL MADER project
% =========================================================================

clear; clc; close all;

%% 1. Define All Three Basis Transformation Matrices
% These matrices convert B-spline control points to other bases.
% Source: mader_types.hpp basisConverter class

fprintf('=== MINVO Basis Fundamentals ===\n\n');

% -------------------------------------------------------------------------
% B-SPLINE BASIS MATRIX (A_pos_bs_rest from mader_types.hpp)
% Maps control points to polynomial coefficients for u_vec = [u^3; u^2; u; 1]
% IMPORTANT: This must match the matrix used to derive M_bs2mv and M_bs2be!
% -------------------------------------------------------------------------
M_bspline = [
   -1/6,  1/2, -1/2, 1/6;
    1/2,   -1,    0, 4/6;
   -1/2,  1/2,  1/2, 1/6;
    1/6,    0,    0,   0
];

% -------------------------------------------------------------------------
% B-SPLINE TO MINVO TRANSFORMATION (M_pos_bs2mv_rest)
% Converts 4 B-spline control points to 4 MINVO control points
% The MINVO points form a tighter convex hull around the curve
% -------------------------------------------------------------------------
M_bs2mv = [
    0.18372189915240555,  0.057009540927778310, -0.015455155707597118, -0.0053387944495218165;
    0.70176522577378919,  0.66657381254229420,   0.29187180223752385,   0.11985166952332582;
    0.11985166952332682,  0.29187180223752446,   0.66657381254229420,   0.70176522577378930;
   -0.0053387944495217445, -0.015455155707597083, 0.057009540927778303,  0.18372189915240558
];

% -------------------------------------------------------------------------
% B-SPLINE TO BEZIER TRANSFORMATION (M_pos_bs2be_rest)
% Converts 4 B-spline control points to 4 Bezier control points
% Bezier: first and last control points are ON the curve
% -------------------------------------------------------------------------
M_bs2be = [
    1/6,   0,     0,     0;
    4/6, 4/6,   2/6,   1/6;
    1/6, 2/6,   4/6,   4/6;
      0,   0,     0,   1/6
];

% -------------------------------------------------------------------------
% MINVO BASIS MATRIX (A_pos_mv_rest)
% Maps MINVO control points to polynomial coefficients
% -------------------------------------------------------------------------
A_minvo = [
   -3.4416308968564118,  6.9895481477801393, -4.4622887507045297,  0.91437149978080234;
    6.6792587327074839, -11.84598990155675,   5.2523596690684613,  0;
   -6.6792587327074839,  8.1917862965657040, -1.5981560640774179,  0.085628500219197656;
    3.4416308968564118, -3.3353445427890960,  0.80808514571348655, 0
];

% -------------------------------------------------------------------------
% BEZIER BASIS MATRIX (A_pos_be_rest)
% Maps Bezier control points to polynomial coefficients
% -------------------------------------------------------------------------
A_bezier = [
   -1,  3, -3, 1;
    3, -6,  3, 0;
   -3,  3,  0, 0;
    1,  0,  0, 0
];

fprintf('Transformation matrices loaded.\n');
fprintf('  M_bs2mv: B-spline -> MINVO (%.0fx%.0f)\n', size(M_bs2mv));
fprintf('  M_bs2be: B-spline -> Bezier (%.0fx%.0f)\n\n', size(M_bs2be));

%% 2. Create Example B-Spline Control Points
% These define a single cubic polynomial segment

% B-spline control points (4 points for 1 cubic segment)
Q_bspline = [
   -2.0,  -0.5,   1.0,   2.5;   % X coordinates
    0.0,   1.5,   1.0,   0.0;   % Y coordinates
    0.0,   0.5,   1.0,   0.5    % Z coordinates
];

fprintf('B-spline control points (input):\n');
for i = 1:4
    fprintf('  Q_bs(%d) = [%6.2f, %6.2f, %6.2f]\n', i, Q_bspline(:,i));
end
fprintf('\n');

%% 3. Transform to MINVO and Bezier Control Points
% Apply the transformation matrices

% Transform: Q_minvo = Q_bspline * M_bs2mv'
% Each column of Q is a control point, so we multiply on the right
Q_minvo = Q_bspline * M_bs2mv';
Q_bezier = Q_bspline * M_bs2be';

fprintf('MINVO control points (transformed):\n');
for i = 1:4
    fprintf('  Q_mv(%d) = [%6.3f, %6.3f, %6.3f]\n', i, Q_minvo(:,i));
end
fprintf('\n');

fprintf('Bezier control points (transformed):\n');
for i = 1:4
    fprintf('  Q_be(%d) = [%6.3f, %6.3f, %6.3f]\n', i, Q_bezier(:,i));
end
fprintf('\n');

%% 4. Evaluate the Polynomial Curve (All Three Should Match!)
% The curve is the SAME regardless of which basis we use.
% Only the control points (convex hull) differ.

u_samples = linspace(0, 1, 100);
curve_bspline = zeros(3, length(u_samples));
curve_minvo = zeros(3, length(u_samples));
curve_bezier = zeros(3, length(u_samples));

for i = 1:length(u_samples)
    u = u_samples(i);
    u_vec = [u^3; u^2; u; 1];

    % Using B-spline: curve = Q_bs * M_bs * u_vec
    curve_bspline(:, i) = Q_bspline * M_bspline * u_vec;

    % Using MINVO: curve = Q_mv * A_mv * u_vec
    curve_minvo(:, i) = Q_minvo * A_minvo * u_vec;

    % Using Bezier: curve = Q_be * A_be * u_vec
    curve_bezier(:, i) = Q_bezier * A_bezier * u_vec;
end

% Verify all three produce the same curve
max_diff_mv = max(vecnorm(curve_bspline - curve_minvo));
max_diff_be = max(vecnorm(curve_bspline - curve_bezier));
fprintf('Verification (all bases should produce same curve):\n');
fprintf('  Max difference B-spline vs MINVO:  %.2e (should be ~0)\n', max_diff_mv);
fprintf('  Max difference B-spline vs Bezier: %.2e (should be ~0)\n\n', max_diff_be);

%% 5. Compute Convex Hull Volumes
% This is the key advantage of MINVO!

vol_bspline = computeConvexHullVolume(Q_bspline);
vol_minvo = computeConvexHullVolume(Q_minvo);
vol_bezier = computeConvexHullVolume(Q_bezier);

fprintf('=== CONVEX HULL VOLUMES ===\n');
fprintf('  B-spline: %.4f\n', vol_bspline);
fprintf('  Bezier:   %.4f (%.1f%% of B-spline)\n', vol_bezier, 100*vol_bezier/vol_bspline);
fprintf('  MINVO:    %.4f (%.1f%% of B-spline) <- SMALLEST!\n\n', vol_minvo, 100*vol_minvo/vol_bspline);

%% 6. Visualization
figure('Position', [50, 50, 1400, 500]);

% --- Plot 1: 3D View with All Three Convex Hulls ---
subplot(1, 3, 1);
hold on;

% Plot the curve (same for all bases)
plot3(curve_bspline(1,:), curve_bspline(2,:), curve_bspline(3,:), ...
    'k-', 'LineWidth', 3, 'DisplayName', 'Polynomial Curve');

% Plot B-spline control points and hull
plotControlPointsAndHull(Q_bspline, [0.2, 0.2, 0.8], 'B-spline', 's');

% Plot Bezier control points and hull
plotControlPointsAndHull(Q_bezier, [0.2, 0.8, 0.2], 'Bezier', 'd');

% Plot MINVO control points and hull
plotControlPointsAndHull(Q_minvo, [0.8, 0.2, 0.2], 'MINVO', 'o');

xlabel('X'); ylabel('Y'); zlabel('Z');
title('Comparison of Convex Hulls');
legend('Location', 'best');
grid on; axis equal;
view(45, 25);

% --- Plot 2: Top-Down View (X-Y) ---
subplot(1, 3, 2);
hold on;

% Curve
plot(curve_bspline(1,:), curve_bspline(2,:), 'k-', 'LineWidth', 3);

% B-spline hull (2D projection)
k = convhull(Q_bspline(1,:), Q_bspline(2,:));
fill(Q_bspline(1,k), Q_bspline(2,k), [0.2, 0.2, 0.8], 'FaceAlpha', 0.2, 'EdgeColor', [0.2, 0.2, 0.8], 'LineWidth', 2);
plot(Q_bspline(1,:), Q_bspline(2,:), 's', 'MarkerSize', 10, 'MarkerFaceColor', [0.2, 0.2, 0.8], 'Color', [0.2, 0.2, 0.8]);

% Bezier hull
k = convhull(Q_bezier(1,:), Q_bezier(2,:));
fill(Q_bezier(1,k), Q_bezier(2,k), [0.2, 0.8, 0.2], 'FaceAlpha', 0.2, 'EdgeColor', [0.2, 0.8, 0.2], 'LineWidth', 2);
plot(Q_bezier(1,:), Q_bezier(2,:), 'd', 'MarkerSize', 10, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'Color', [0.2, 0.8, 0.2]);

% MINVO hull
k = convhull(Q_minvo(1,:), Q_minvo(2,:));
fill(Q_minvo(1,k), Q_minvo(2,k), [0.8, 0.2, 0.2], 'FaceAlpha', 0.2, 'EdgeColor', [0.8, 0.2, 0.2], 'LineWidth', 2);
plot(Q_minvo(1,:), Q_minvo(2,:), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.8, 0.2, 0.2], 'Color', [0.8, 0.2, 0.2]);

xlabel('X'); ylabel('Y');
title('Top-Down View (X-Y Projection)');
legend('Curve', 'B-spline Hull', 'B-spline CPs', 'Bezier Hull', 'Bezier CPs', 'MINVO Hull', 'MINVO CPs');
grid on; axis equal;

% --- Plot 3: Volume Comparison Bar Chart ---
subplot(1, 3, 3);
bar_data = [vol_bspline, vol_bezier, vol_minvo];
bar_colors = [0.2, 0.2, 0.8; 0.2, 0.8, 0.2; 0.8, 0.2, 0.2];
b = bar(bar_data);
b.FaceColor = 'flat';
b.CData = bar_colors;
set(gca, 'XTickLabel', {'B-spline', 'Bezier', 'MINVO'});
ylabel('Convex Hull Volume');
title('Volume Comparison');
grid on;

% Add percentage labels
for i = 1:3
    text(i, bar_data(i) + 0.02, sprintf('%.1f%%', 100*bar_data(i)/vol_bspline), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

sgtitle('MINVO Basis: Minimum Volume Convex Hulls (Example 4)', 'FontSize', 14);

%% 7. Demonstrate Collision Checking Advantage
% Show how MINVO allows passing through tighter gaps

fprintf('=== COLLISION CHECKING EXAMPLE ===\n');

% Define an obstacle (a sphere)
obstacle_center = [0; 1.2; 0.5];
obstacle_radius = 0.3;

% Check collision with each convex hull
collides_bspline = checkHullSphereCollision(Q_bspline, obstacle_center, obstacle_radius);
collides_bezier = checkHullSphereCollision(Q_bezier, obstacle_center, obstacle_radius);
collides_minvo = checkHullSphereCollision(Q_minvo, obstacle_center, obstacle_radius);

% Check if actual curve collides
curve_collides = any(vecnorm(curve_bspline - obstacle_center) < obstacle_radius);

fprintf('Obstacle: sphere at [%.1f, %.1f, %.1f], radius %.2f\n', obstacle_center, obstacle_radius);
fprintf('\nCollision detection results:\n');
fprintf('  Actual curve collides:     %s\n', boolToStr(curve_collides));
fprintf('  B-spline hull collides:    %s\n', boolToStr(collides_bspline));
fprintf('  Bezier hull collides:      %s\n', boolToStr(collides_bezier));
fprintf('  MINVO hull collides:       %s <- Most accurate!\n\n', boolToStr(collides_minvo));

if ~curve_collides && collides_bspline && ~collides_minvo
    fprintf('MINVO correctly detected NO collision while B-spline gave FALSE POSITIVE!\n');
    fprintf('This is the key advantage: MINVO is less conservative.\n');
end

%% 8. Multi-Segment Example
% Show how MINVO works with multiple polynomial segments

fprintf('\n=== MULTI-SEGMENT TRAJECTORY ===\n');

% Define a trajectory with 4 segments (7 B-spline control points)
Q_multi_bs = [
   -3, -2, -1,  0,  1,  2,  3;   % X
    0,  1,  0, -1,  0,  1,  0;   % Y
    0, 0.5, 1, 1.2, 1, 0.5, 0    % Z
];

num_segments = size(Q_multi_bs, 2) - 3;
fprintf('Number of segments: %d\n', num_segments);

% Different transformation matrices for boundary segments
M_bs2mv_seg0 = [
    1.1023313949144333,   0.34205724556666972, -0.092730934245582874, -0.032032766697130621;
   -0.049683556253749178, 0.65780347324677180,  0.53053863760186903,   0.21181027098212013;
   -0.047309044211162346, 0.015594436894155586, 0.5051827557159349,    0.63650059656260427;
   -0.0053387944495217445, -0.015455155707597083, 0.057009540927778303, 0.18372189915240558
];

M_bs2mv_seg_last = [
    0.18372189915240555,  0.057009540927778310, -0.015455155707597118, -0.0053387944495218165;
    0.63650059656260416,  0.50518275571593496,   0.015594436894155295, -0.047309044211162887;
    0.21181027098212069,  0.53053863760186915,   0.65780347324677146,  -0.049683556253749622;
   -0.032032766697130462, -0.092730934245582486,  0.34205724556666978,   1.1023313949144333
];

% Visualize multi-segment trajectory
figure('Position', [100, 100, 1000, 400]);

subplot(1, 2, 1);
hold on;

% Sample and plot the full curve
t_full = linspace(0, num_segments, 200);
curve_full = zeros(3, length(t_full));

for i = 1:length(t_full)
    seg = min(floor(t_full(i)) + 1, num_segments);
    u = t_full(i) - (seg - 1);
    u = min(max(u, 0), 1);
    u_vec = [u^3; u^2; u; 1];

    Q_seg = Q_multi_bs(:, seg:seg+3);
    curve_full(:, i) = Q_seg * M_bspline * u_vec;
end

plot3(curve_full(1,:), curve_full(2,:), curve_full(3,:), 'k-', 'LineWidth', 2);

% Plot MINVO hulls for each segment
colors = lines(num_segments);
for seg = 1:num_segments
    Q_seg_bs = Q_multi_bs(:, seg:seg+3);

    % Select appropriate transformation matrix
    if seg == 1
        M_transform = M_bs2mv_seg0;
    elseif seg == num_segments
        M_transform = M_bs2mv_seg_last;
    else
        M_transform = M_bs2mv;
    end

    Q_seg_mv = Q_seg_bs * M_transform';

    % Plot MINVO control points
    plot3(Q_seg_mv(1,:), Q_seg_mv(2,:), Q_seg_mv(3,:), 'o-', ...
        'Color', colors(seg,:), 'MarkerSize', 6, 'MarkerFaceColor', colors(seg,:), ...
        'LineWidth', 1.5);

    % Plot convex hull
    try
        K = convhull(Q_seg_mv');
        trisurf(K, Q_seg_mv(1,:), Q_seg_mv(2,:), Q_seg_mv(3,:), ...
            'FaceColor', colors(seg,:), 'FaceAlpha', 0.15, 'EdgeColor', colors(seg,:));
    catch
        % Degenerate case
    end
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title('Multi-Segment MINVO Hulls');
grid on; axis equal;
view(45, 20);

subplot(1, 2, 2);
hold on;
plot(curve_full(1,:), curve_full(2,:), 'k-', 'LineWidth', 2);
for seg = 1:num_segments
    Q_seg_bs = Q_multi_bs(:, seg:seg+3);
    if seg == 1
        M_transform = M_bs2mv_seg0;
    elseif seg == num_segments
        M_transform = M_bs2mv_seg_last;
    else
        M_transform = M_bs2mv;
    end
    Q_seg_mv = Q_seg_bs * M_transform';

    k = convhull(Q_seg_mv(1,:), Q_seg_mv(2,:));
    fill(Q_seg_mv(1,k), Q_seg_mv(2,k), colors(seg,:), 'FaceAlpha', 0.2, 'EdgeColor', colors(seg,:));
    plot(Q_seg_mv(1,:), Q_seg_mv(2,:), 'o', 'Color', colors(seg,:), 'MarkerFaceColor', colors(seg,:));
end
xlabel('X'); ylabel('Y');
title('Top-Down View');
grid on; axis equal;

sgtitle('Multi-Segment MINVO Trajectory');

%% Summary
fprintf('\n=== SUMMARY: WHY USE MINVO? ===\n');
fprintf('1. SAME CURVE: B-spline, Bezier, and MINVO all represent the same polynomial\n');
fprintf('2. DIFFERENT HULLS: Control points define different convex hulls\n');
fprintf('3. MINVO = SMALLEST: MINVO hull has minimum volume (tightest bound)\n');
fprintf('4. BETTER COLLISION: Less conservative -> fewer false positives\n');
fprintf('5. PRACTICAL IMPACT: Navigate tighter spaces, more feasible trajectories\n');

%% ======================== Helper Functions ========================

function vol = computeConvexHullVolume(Q)
    % Compute volume of convex hull of 3D points
    try
        [~, vol] = convhull(Q');
    catch
        % Degenerate case (coplanar points)
        vol = 0;
    end
end

function plotControlPointsAndHull(Q, color, name, marker)
    % Plot control points and their convex hull
    plot3(Q(1,:), Q(2,:), Q(3,:), marker, 'MarkerSize', 10, ...
        'MarkerFaceColor', color, 'Color', color, 'DisplayName', [name ' CPs']);

    try
        K = convhull(Q');
        trisurf(K, Q(1,:), Q(2,:), Q(3,:), 'FaceColor', color, ...
            'FaceAlpha', 0.15, 'EdgeColor', color, 'DisplayName', [name ' Hull']);
    catch
        % Degenerate
    end
end

function collides = checkHullSphereCollision(Q, center, radius)
    % Check if convex hull of Q intersects with sphere
    % Simplified: check if any hull vertex or center is within radius

    % Check vertices
    for i = 1:size(Q, 2)
        if norm(Q(:,i) - center) < radius
            collides = true;
            return;
        end
    end

    % Check if sphere center is inside hull
    try
        K = convhull(Q');
        % Simplified check using bounding box
        minQ = min(Q, [], 2);
        maxQ = max(Q, [], 2);
        if all(center >= minQ - radius) && all(center <= maxQ + radius)
            % More detailed check would use proper point-in-hull test
            collides = true;
            return;
        end
    catch
    end

    collides = false;
end

function s = boolToStr(b)
    if b
        s = 'YES';
    else
        s = 'NO';
    end
end
