function plot_barriers()
% Define the grid size and the range
grid_size = 50;
x_range = linspace(-5, 5, grid_size);
y_range = linspace(-5, 5, grid_size);
z_range = linspace(-5, 5, grid_size);

% Create the 3D grid of points
[X, Y, Z] = meshgrid(x_range, y_range, z_range);

% Define the implicit functions for the spheres
h1 = @(x, y, z) (x - 2.2).^2 + (z - 1).^2 - 1 + (2.2 - 2.2).^2;
h2 = @(x, y, z) x.^2 + (y + 0.2).^2 + z.^2 - 1;
h3 = @(x, y, z) (x - 3).^2 + y.^2 + (z - 0.5).^2 - 1;

% Evaluate the functions on the grid
H1 = h1(X, Y, Z);
H2 = h2(X, Y, Z);
H3 = h3(X, Y, Z);

% Plot the spheres using isosurface
figure;
hold on;
isosurface(X, Y, Z, H1, 0);
isosurface(X, Y, Z, H2, 0);
isosurface(X, Y, Z, H3, 0);

% Set the plot properties
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Spheres representing h1, h2, and h3');
axis equal;
grid on;
legend({'h1', 'h2', 'h3'});
view(3);
hold off;
end