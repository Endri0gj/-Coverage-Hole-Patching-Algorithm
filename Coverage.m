% Define the detection area and node parameters
area_width = 125; area_height = 100;  % Dimensions of the detection area
num_nodes = 25;  % Total number of static nodes
rng(42);  % For reproducibility

% Node categories (sensing radii)
large_radius = 12;  % Large sensing radius (RS2)
small_radius = 8;   % Small sensing radius (RS1)
node_categories = randi([1, 2], num_nodes, 1);  % Randomly assign node categories

% Generate corrected node positions
node_positions = generate_non_overlapping_nodes(area_width, area_height, num_nodes, ...
    node_categories == 1 | node_categories == 2);  % Generate positions

% Plot the corrected version of Figure 7
figure;
hold on;
axis([0 area_width 0 area_height]);
title('Corrected Randomly Assigned Static Nodes (Figure 7)', 'FontSize', 16);
xlabel('Width (m)', 'FontSize', 12);
ylabel('Height (m)', 'FontSize', 12);

% Draw the sensing areas for the corrected positions
for i = 1:num_nodes
    radius = (node_categories(i) == 1) * large_radius + (node_categories(i) == 2) * small_radius;
    theta = linspace(0, 2*pi, 100);
    x = node_positions(i, 1) + radius * cos(theta);
    y = node_positions(i, 2) + radius * sin(theta);
    fill(x, y, 'b', 'FaceAlpha', 0.2);
    plot(node_positions(i, 1), node_positions(i, 2), 'bo');
end

legend('Static Node');
grid on;
hold off;
drawnow;  % Ensure the first figure is rendered before moving on

% Calculate the coverage for the static nodes
resolution = 1;  % 1 meter resolution for the grid
[coverage_grid, x_coords, y_coords] = calculate_coverage(node_positions, ...
    node_categories == 1 * large_radius + (node_categories == 2) * small_radius, area_width, area_height, resolution);

% Add mobile nodes with a sensing radius of RS1 (8m) until coverage goal is reached
patch_radius = small_radius;
coverage_goal = 0.9;  % 90% coverage goal
optimized_patch_positions = patch_coverage_holes_until_goal(coverage_grid, x_coords, y_coords, patch_radius, coverage_goal);

% Plot the updated coverage with patched nodes (Figure 8)
figure;
hold on;
axis([0 area_width 0 area_height]);
title('Optimized Coverage Holes Patched with Mobile Nodes (Figure 8)', 'FontSize', 16);
xlabel('Width (m)', 'FontSize', 12);
ylabel('Height (m)', 'FontSize', 12);

% Draw the original static nodes and their sensing areas
for i = 1:num_nodes
    radius = (node_categories(i) == 1) * large_radius + (node_categories(i) == 2) * small_radius;
    theta = linspace(0, 2*pi, 100);
    x = node_positions(i, 1) + radius * cos(theta);
    y = node_positions(i, 2) + radius * sin(theta);
    fill(x, y, 'b', 'FaceAlpha', 0.2);
    plot(node_positions(i, 1), node_positions(i, 2), 'bo');
end

% Draw the patched nodes and their sensing areas
for i = 1:size(optimized_patch_positions, 1)
    patch_x = optimized_patch_positions(i, 1);
    patch_y = optimized_patch_positions(i, 2);
    theta = linspace(0, 2*pi, 100);
    x = patch_x + patch_radius * cos(theta);
    y = patch_y + patch_radius * sin(theta);
    fill(x, y, 'r', 'FaceAlpha', 0.3);
    plot(patch_x, patch_y, 'ro');
end

legend('Static Node', 'Patched Node');
grid on;
hold off;

% Function to generate non-overlapping nodes
function positions = generate_non_overlapping_nodes(area_width, area_height, num_nodes, radii)
    positions = [];
    for i = 1:num_nodes
        while true
            % Generate a random position
            new_position = [rand() * area_width, rand() * area_height];
            % Check for minimum distance with existing nodes
            valid_position = true;
            for j = 1:size(positions, 1)
                if norm(new_position - positions(j, :)) <= 0.8 * (radii(i) + radii(j))
                    valid_position = false;
                    break;
                end
            end
            if valid_position
                positions = [positions; new_position];  % Add the new node position
                break;
            end
        end
    end
end

% Function to calculate coverage
function [coverage_grid, x_coords, y_coords] = calculate_coverage(node_positions, sensing_radii, area_width, area_height, resolution)
    x_coords = 0:resolution:area_width;
    y_coords = 0:resolution:area_height;
    coverage_grid = zeros(length(x_coords), length(y_coords));
    for i = 1:size(node_positions, 1)
        for xi = 1:length(x_coords)
            for yi = 1:length(y_coords)
                if norm([x_coords(xi) - node_positions(i, 1), y_coords(yi) - node_positions(i, 2)]) <= sensing_radii(i)
                    coverage_grid(xi, yi) = 1;
                end
            end
        end
    end
end


% Function to detect and patch coverage holes with prioritization of larger holes
function patch_positions = patch_coverage_holes_until_goal(coverage_grid, x_coords, y_coords, patch_radius, coverage_goal)
    uncovered_points = find(coverage_grid == 0);
    patch_positions = [];
    total_points = numel(coverage_grid);
    target_covered = floor(total_points * coverage_goal);
    current_coverage = sum(coverage_grid(:));

    while current_coverage < target_covered && ~isempty(uncovered_points)
        [i, j] = ind2sub(size(coverage_grid), uncovered_points(randi(length(uncovered_points))));
        patch_x = x_coords(i);
        patch_y = y_coords(j);

        % Count the number of uncovered points covered by this mobile node
        covered_by_patch = 0;
        for m = 1:length(x_coords)
            for n = 1:length(y_coords)
                if coverage_grid(m, n) == 0 && norm([x_coords(m) - patch_x, y_coords(n) - patch_y]) <= patch_radius
                    covered_by_patch = covered_by_patch + 1;
                end
            end
        end
        
        if covered_by_patch > 10
            patch_positions = [patch_positions; patch_x, patch_y];
            for m = 1:length(x_coords)
                for n = 1:length(y_coords)
                    if norm([x_coords(m) - patch_x, y_coords(n) - patch_y]) <= patch_radius
                        coverage_grid(m, n) = 1;
                    end
                end
            end
        end

        uncovered_points = find(coverage_grid == 0);
        current_coverage = sum(coverage_grid(:));
    end
end
